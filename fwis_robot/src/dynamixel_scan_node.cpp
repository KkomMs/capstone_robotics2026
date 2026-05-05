/**
 * dynamixel_scan_node.cpp
 *
 * 변경사항:
 *  - P0 구간 추가 (천천히 시작 구간)
 *  - scanner_id 기반 축 분리 (바코드 받은 쪽 축만 이동)
 *  - TiltState publish 추가
 *  - yaml 파라미터 기반 유지
 *  - /alignment_done → 딜레이 후 스캔 시작
 *  - /scan_done publish 유지
 *  - [수정] 각 축 복귀 시작 시 해당 축 스캔 결과 출력
 */

#include "scanner_interfaces/msg/barcode_event.hpp"
#include "scanner_interfaces/msg/tilt_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <map>
#include <set>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <string>
#include <atomic>
#include <thread>
#include <unistd.h>

#include "dynamixel_sdk.h"

static constexpr uint16_t ADDR_OPERATING_MODE   = 11;
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
static constexpr uint16_t ADDR_GOAL_POSITION    = 116;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
static constexpr double   PROTOCOL_VERSION      = 2.0;
static constexpr uint8_t  EXT_POSITION_MODE     = 4;
static constexpr uint8_t  TORQUE_ENABLE_VAL     = 1;
static constexpr uint8_t  TORQUE_DISABLE_VAL    = 0;

enum class MotorState {
    P0_STEP_MOVE, P0_STEP_WAIT,
    P1_SWEEP,
    P2_STEP_MOVE, P2_STEP_WAIT,
    P3_STEP_MOVE, P3_STEP_WAIT,
    RETURNING, DONE
};

struct MotorAxis {
    uint8_t     id      = 0;
    std::string name;
    int32_t start   = 0, p0_end = 0, p1_end = 0, p2_end = 0, end = 0;
    int     vel_p1  = 1, vel_step = 1, vel_return = 10;
    int     p0_step = 40, p2_step = 10, p3_step = 5;
    bool    p2_is_sweep = false;
    int32_t current_tick = 0;
    int32_t goal_tick    = 0;
    bool    summary_printed = false;  // 복귀 시 summary 중복 출력 방지
    MotorState   state      = MotorState::DONE;
    rclcpp::Time wait_start;

    bool arrived(int threshold) const {
        return std::abs(current_tick - goal_tick) <= threshold;
    }
};

struct SlotRecord {
    std::string serial;
    int         unit = -1;
};

struct ScanStats {
    // scanner_id별로 slot 결과 저장
    std::map<int, std::map<int, SlotRecord>> records_by_sid;

    void Reset() {
        records_by_sid.clear();
    }

    void Record(int slot, int sid, const std::string & serial) {
        records_by_sid[sid][slot] = {serial, slot};
    }
};

class DynamixelScanNode : public rclcpp::Node
{
public:
    explicit DynamixelScanNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("dynamixel_scan_node", options)
    {
        LoadParams();
        SetupAxes();
        InitDxl();

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        pub_scan_done_ = this->create_publisher<std_msgs::msg::Bool>("/scan_done", qos);
        pub_tilt_      = this->create_publisher<scanner_interfaces::msg::TiltState>(
            "/scanner/tilt_state", 100);

        sub_barcode_ = this->create_subscription<scanner_interfaces::msg::BarcodeEvent>(
            "/barcode/unit_event", rclcpp::QoS(100),
            [this](const scanner_interfaces::msg::BarcodeEvent::SharedPtr msg) {
                OnBarcodeEvent(*msg);
            });

        sub_alignment_done_ = this->create_subscription<std_msgs::msg::Bool>(
            "/alignment_done", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && !scanning_.load()) {
                    RCLCPP_INFO(get_logger(),
                        "[DXL] alignment_done → %.1f초 후 스캔 시작",
                        p_.alignment_done_delay_sec);
                    std::thread([this]() {
                        usleep(static_cast<useconds_t>(
                            p_.alignment_done_delay_sec * 1e6));
                        StartScan();
                    }).detach();
                }
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(p_.control_period_ms),
            std::bind(&DynamixelScanNode::ControlLoop, this));

        RCLCPP_INFO(get_logger(),
            "[DXL] node ready | left_id=%d right_id=%d | device=%s baudrate=%d",
            p_.left_id, p_.right_id, p_.device.c_str(), p_.baudrate);
    }

    ~DynamixelScanNode() { CloseDxl(); }

private:
    struct Params {
        std::string device   = "/dev/ttyDynamixel";
        int         baudrate = 57600;
        int left_id  = 6;
        int right_id = 7;
        int left_scanner_id  = 1;
        int right_scanner_id = 2;
        int32_t left_start  =  330;
        int32_t left_p0_end =  210;
        int32_t left_p1_end = -610;
        int32_t left_p2_end = -870;
        int32_t left_end    = -960;
        int32_t right_start  = 1470;
        int32_t right_p0_end = 1350;
        int32_t right_p1_end =  530;
        int32_t right_p2_end =  270;
        int32_t right_end    =  160;
        int left_vel_p1     = 3;
        int left_vel_step   = 1;
        int left_vel_return = 10;
        int right_vel_p1     = 4;
        int right_vel_step   = 1;
        int right_vel_return = 10;
        int left_p0_step  = 40;
        int left_p2_step  = 10;
        int left_p3_step  = 3;
        int right_p0_step = 40;
        int right_p2_step = 10;
        int right_p3_step = 5;
        bool left_p2_is_sweep  = false;
        bool right_p2_is_sweep = false;
        int    arrive_threshold         = 20;
        int    step_timeout_ms          = 3000;
        double alignment_done_delay_sec = 2.0;
        int    control_period_ms        = 100;
        int              total_slots = 51;
        std::vector<int> empty_slots = {9, 11, 14, 30, 39, 50};
    };

    void LoadParams()
    {
        auto gi  = [this](const std::string & n){ return (int)this->get_parameter(n).as_int(); };
        auto gd  = [this](const std::string & n){ return this->get_parameter(n).as_double(); };
        auto gs  = [this](const std::string & n){ return this->get_parameter(n).as_string(); };
        auto gb  = [this](const std::string & n){ return this->get_parameter(n).as_bool(); };
        auto giv = [this](const std::string & n) -> std::vector<int> {
            auto v = this->get_parameter(n).as_integer_array();
            std::vector<int> out;
            for (auto x : v) out.push_back((int)x);
            return out;
        };

        p_.device   = gs("dxl_device");
        p_.baudrate = gi("dxl_baudrate");
        p_.left_id  = gi("dxl_left_id");
        p_.right_id = gi("dxl_right_id");
        p_.left_scanner_id  = gi("dxl_left_scanner_id");
        p_.right_scanner_id = gi("dxl_right_scanner_id");
        p_.left_start  = (int32_t)gi("dxl_left_start");
        p_.left_p0_end = (int32_t)gi("dxl_left_p0_end");
        p_.left_p1_end = (int32_t)gi("dxl_left_p1_end");
        p_.left_p2_end = (int32_t)gi("dxl_left_p2_end");
        p_.left_end    = (int32_t)gi("dxl_left_end");
        p_.right_start  = (int32_t)gi("dxl_right_start");
        p_.right_p0_end = (int32_t)gi("dxl_right_p0_end");
        p_.right_p1_end = (int32_t)gi("dxl_right_p1_end");
        p_.right_p2_end = (int32_t)gi("dxl_right_p2_end");
        p_.right_end    = (int32_t)gi("dxl_right_end");
        p_.left_vel_p1     = gi("dxl_left_vel_p1");
        p_.left_vel_step   = gi("dxl_left_vel_step");
        p_.left_vel_return = gi("dxl_left_vel_return");
        p_.right_vel_p1     = gi("dxl_right_vel_p1");
        p_.right_vel_step   = gi("dxl_right_vel_step");
        p_.right_vel_return = gi("dxl_right_vel_return");
        p_.left_p0_step  = gi("dxl_left_p0_step");
        p_.left_p2_step  = gi("dxl_left_p2_step");
        p_.left_p3_step  = gi("dxl_left_p3_step");
        p_.right_p0_step = gi("dxl_right_p0_step");
        p_.right_p2_step = gi("dxl_right_p2_step");
        p_.right_p3_step = gi("dxl_right_p3_step");
        p_.left_p2_is_sweep  = gb("dxl_left_p2_is_sweep");
        p_.right_p2_is_sweep = gb("dxl_right_p2_is_sweep");
        p_.arrive_threshold         = gi("dxl_arrive_threshold");
        p_.step_timeout_ms          = gi("dxl_step_timeout_ms");
        p_.alignment_done_delay_sec = gd("alignment_done_delay_sec");
        p_.control_period_ms        = gi("dxl_control_period_ms");
        p_.total_slots = gi("total_slots");
        p_.empty_slots = giv("empty_slots");
        for (int s : p_.empty_slots) empty_slots_set_.insert(s);
    }

    void SetupAxes()
    {
        left_.id          = (uint8_t)p_.left_id;
        left_.name        = "LEFT";
        left_.start       = p_.left_start;
        left_.p0_end      = p_.left_p0_end;
        left_.p1_end      = p_.left_p1_end;
        left_.p2_end      = p_.left_p2_end;
        left_.end         = p_.left_end;
        left_.vel_p1      = p_.left_vel_p1;
        left_.vel_step    = p_.left_vel_step;
        left_.vel_return  = p_.left_vel_return;
        left_.p0_step     = p_.left_p0_step;
        left_.p2_step     = p_.left_p2_step;
        left_.p3_step     = p_.left_p3_step;
        left_.p2_is_sweep = p_.left_p2_is_sweep;
        left_.state       = MotorState::DONE;

        right_.id          = (uint8_t)p_.right_id;
        right_.name        = "RIGHT";
        right_.start       = p_.right_start;
        right_.p0_end      = p_.right_p0_end;
        right_.p1_end      = p_.right_p1_end;
        right_.p2_end      = p_.right_p2_end;
        right_.end         = p_.right_end;
        right_.vel_p1      = p_.right_vel_p1;
        right_.vel_step    = p_.right_vel_step;
        right_.vel_return  = p_.right_vel_return;
        right_.p0_step     = p_.right_p0_step;
        right_.p2_step     = p_.right_p2_step;
        right_.p3_step     = p_.right_p3_step;
        right_.p2_is_sweep = p_.right_p2_is_sweep;
        right_.state       = MotorState::DONE;
    }

    void InitDxl()
    {
        port_handler_   = dynamixel::PortHandler::getPortHandler(p_.device.c_str());
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        if (!port_handler_->openPort())
            throw std::runtime_error("[DXL] Failed to open port: " + p_.device);
        if (!port_handler_->setBaudRate(p_.baudrate))
            throw std::runtime_error("[DXL] Failed to set baudrate");
        usleep(100 * 1000);
        uint8_t err = 0;
        for (int id : {p_.left_id, p_.right_id}) {
            packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE,  TORQUE_DISABLE_VAL, &err);
            packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_OPERATING_MODE, EXT_POSITION_MODE,  &err);
            packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE_VAL,  &err);
            RCLCPP_INFO(get_logger(), "[DXL] ID %d initialized", id);
        }
    }

    void CloseDxl()
    {
        if (!port_handler_) return;
        uint8_t err = 0;
        for (int id : {p_.left_id, p_.right_id})
            packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, &err);
        port_handler_->closePort();
    }

    int32_t ReadPos(uint8_t id)
    {
        int32_t curr = 0; uint8_t err = 0;
        packet_handler_->read4ByteTxRx(port_handler_, id, ADDR_PRESENT_POSITION, (uint32_t*)&curr, &err);
        return curr;
    }

    void WriteVelGoal(uint8_t id, int vel, int32_t goal)
    {
        uint8_t err = 0;
        packet_handler_->write4ByteTxRx(port_handler_, id, ADDR_PROFILE_VELOCITY, (uint32_t)vel,  &err);
        packet_handler_->write4ByteTxRx(port_handler_, id, ADDR_GOAL_POSITION,    (uint32_t)goal, &err);
    }

    void StartScan()
    {
        scan_stats_.Reset();
        scanning_.store(true);
        left_.summary_printed  = false;
        right_.summary_printed = false;
        left_.goal_tick  = left_.start;
        right_.goal_tick = right_.start;
        StartP0(left_);
        StartP0(right_);
        RCLCPP_INFO(get_logger(), "[DXL] scan started");
    }

    void StartP0(MotorAxis & ax)
    {
        ax.state = MotorState::P0_STEP_MOVE;
        RCLCPP_INFO(get_logger(), "[%s] P0 step start: %d -> %d step=%d",
            ax.name.c_str(), ax.start, ax.p0_end, ax.p0_step);
        SendStep(ax, ax.p0_end, ax.p0_step);
    }

    void StartP1(MotorAxis & ax)
    {
        ax.goal_tick = ax.p1_end;
        ax.state     = MotorState::P1_SWEEP;
        WriteVelGoal(ax.id, ax.vel_p1, ax.p1_end);
        RCLCPP_INFO(get_logger(), "[%s] P1 sweep start vel=%d goal=%d",
            ax.name.c_str(), ax.vel_p1, ax.p1_end);
    }

    void StartP2(MotorAxis & ax)
    {
        ax.state = MotorState::P2_STEP_MOVE;
        RCLCPP_INFO(get_logger(), "[%s] P2 step-and-wait start", ax.name.c_str());
        SendStep(ax, ax.p2_end, ax.p2_step);
    }

    void StartP3(MotorAxis & ax)
    {
        ax.state = MotorState::P3_STEP_MOVE;
        RCLCPP_INFO(get_logger(), "[%s] P3 step-and-wait start", ax.name.c_str());
        SendStep(ax, ax.end, ax.p3_step);
    }

    // ── [수정] 복귀 시작 시 해당 축 summary 출력 ──────────────
    void StartReturn(MotorAxis & ax)
    {
        if (!ax.summary_printed) {
            int sid = (&ax == &left_) ? p_.left_scanner_id : p_.right_scanner_id;
            PrintAxisSummary(ax.name, sid);
            ax.summary_printed = true;
        }
        ax.goal_tick = ax.start;
        ax.state     = MotorState::RETURNING;
        WriteVelGoal(ax.id, ax.vel_return, ax.start);
        RCLCPP_INFO(get_logger(), "[%s] returning vel=%d goal=%d",
            ax.name.c_str(), ax.vel_return, ax.start);
    }

    void SendStep(MotorAxis & ax, int32_t limit, int step)
    {
        int32_t next = std::max(ax.goal_tick - step, limit);
        ax.goal_tick = next;
        WriteVelGoal(ax.id, ax.vel_step, next);
        RCLCPP_INFO(get_logger(), "[%s GOAL] tick=%d", ax.name.c_str(), next);
    }

    void HaltAndNext(MotorAxis & ax)
    {
        ax.goal_tick = ax.current_tick;
        WriteVelGoal(ax.id, ax.vel_step, ax.current_tick);
        usleep(50 * 1000);
        StepToNext(ax);
    }

    void StepToNext(MotorAxis & ax)
    {
        if (ax.state == MotorState::P0_STEP_MOVE || ax.state == MotorState::P0_STEP_WAIT) {
            if (ax.current_tick <= ax.p0_end + p_.arrive_threshold) {
                RCLCPP_INFO(get_logger(), "[%s] P0 done → P1 sweep", ax.name.c_str());
                StartP1(ax); return;
            }
            ax.state = MotorState::P0_STEP_MOVE;
            SendStep(ax, ax.p0_end, ax.p0_step); return;
        }
        if (ax.state == MotorState::P2_STEP_MOVE || ax.state == MotorState::P2_STEP_WAIT) {
            if (ax.current_tick <= ax.p2_end + p_.arrive_threshold) {
                RCLCPP_INFO(get_logger(), "[%s] P2 done → P3", ax.name.c_str());
                StartP3(ax); return;
            }
            ax.state = MotorState::P2_STEP_MOVE;
            SendStep(ax, ax.p2_end, ax.p2_step); return;
        }
        if (ax.state == MotorState::P3_STEP_MOVE || ax.state == MotorState::P3_STEP_WAIT) {
            if (ax.current_tick <= ax.end + p_.arrive_threshold) {
                RCLCPP_INFO(get_logger(), "[%s] P3 done → return", ax.name.c_str());
                StartReturn(ax); return;
            }
            ax.state = MotorState::P3_STEP_MOVE;
            SendStep(ax, ax.end, ax.p3_step); return;
        }
    }

    void OnBarcodeEvent(const scanner_interfaces::msg::BarcodeEvent & msg)
    {
        if (!scanning_.load()) return;
        int slot = msg.unit;
        int sid  = msg.scanner_id;
        if (slot < 1 || slot > p_.total_slots) return;

        scan_stats_.Record(slot, sid, msg.serial);
        RCLCPP_INFO(get_logger(),
            "[DXL] barcode sid=%d slot=%02d serial=%s → next step",
            sid, slot, msg.serial.c_str());

        MotorAxis * ax = nullptr;
        if      (sid == p_.left_scanner_id)  ax = &left_;
        else if (sid == p_.right_scanner_id) ax = &right_;
        else {
            RCLCPP_WARN(get_logger(), "[DXL] unknown scanner_id=%d", sid);
            return;
        }

        if (ax->state == MotorState::P2_STEP_MOVE || ax->state == MotorState::P3_STEP_MOVE)
            HaltAndNext(*ax);
        else if (ax->state == MotorState::P2_STEP_WAIT || ax->state == MotorState::P3_STEP_WAIT)
            StepToNext(*ax);
    }

    void ControlLoop()
    {
        if (!scanning_.load()) return;
        left_.current_tick  = ReadPos(left_.id);
        right_.current_tick = ReadPos(right_.id);

        scanner_interfaces::msg::TiltState tilt;
        tilt.stamp      = this->now();
        tilt.left_tick  = left_.current_tick;
        tilt.right_tick = right_.current_tick;
        pub_tilt_->publish(tilt);

        UpdateAxis(left_);
        UpdateAxis(right_);

        if (left_.state == MotorState::DONE && right_.state == MotorState::DONE) {
            RCLCPP_INFO(get_logger(), "[DXL] both axes done → scan finish");
            FinishScan();
        }
    }

    void UpdateAxis(MotorAxis & ax)
    {
        switch (ax.state) {
        case MotorState::P0_STEP_MOVE:
        case MotorState::P2_STEP_MOVE:
        case MotorState::P3_STEP_MOVE: {
            if (!ax.arrived(p_.arrive_threshold)) return;
            if      (ax.state == MotorState::P0_STEP_MOVE) ax.state = MotorState::P0_STEP_WAIT;
            else if (ax.state == MotorState::P2_STEP_MOVE) ax.state = MotorState::P2_STEP_WAIT;
            else                                            ax.state = MotorState::P3_STEP_WAIT;
            ax.wait_start = this->now();
            RCLCPP_INFO(get_logger(), "[%s WAIT] tick=%d", ax.name.c_str(), ax.current_tick);
            break;
        }
        case MotorState::P0_STEP_WAIT:
        case MotorState::P2_STEP_WAIT:
        case MotorState::P3_STEP_WAIT: {
            double elapsed_ms = (this->now() - ax.wait_start).seconds() * 1000.0;
            if (elapsed_ms < p_.step_timeout_ms) return;
            RCLCPP_WARN(get_logger(), "[%s TIMEOUT] tick=%d", ax.name.c_str(), ax.current_tick);
            StepToNext(ax); break;
        }
        case MotorState::P1_SWEEP: {
            if (!ax.arrived(p_.arrive_threshold)) return;
            RCLCPP_INFO(get_logger(), "[%s] P1 done → P2", ax.name.c_str());
            StartP2(ax); break;
        }
        case MotorState::RETURNING: {
            if (!ax.arrived(p_.arrive_threshold)) return;
            RCLCPP_INFO(get_logger(), "[%s] returned", ax.name.c_str());
            ax.state = MotorState::DONE; break;
        }
        case MotorState::DONE: break;
        }
    }

    // ── [추가] 축별 스캔 결과 출력 ───────────────────────────
    void PrintAxisSummary(const std::string & axis_name, int sid)
    {
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "========== %s SCANNER RESULT ==========", axis_name.c_str());

        int ok_count    = 0;
        int empty_count = 0;
        int miss_count  = 0;

        auto sit = scan_stats_.records_by_sid.find(sid);

        for (int u = 1; u <= p_.total_slots; ++u) {
            bool is_empty = empty_slots_set_.count(u) > 0;
            bool detected = (sit != scan_stats_.records_by_sid.end() &&
                             sit->second.count(u) > 0);

            if (detected) {
                ok_count++;
                RCLCPP_INFO(get_logger(), "  U%02d : OK    serial=%s",
                    u, sit->second.at(u).serial.c_str());
            } else if (is_empty) {
                empty_count++;
                RCLCPP_INFO(get_logger(), "  U%02d : [빈 슬롯]", u);
            } else {
                miss_count++;
                RCLCPP_WARN(get_logger(), "  U%02d : NOT_SEEN", u);
            }
        }

        int filled_slots = p_.total_slots - (int)empty_slots_set_.size();
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "  인식: %d / %d  (미인식: %d)",
            ok_count, filled_slots, miss_count);
        RCLCPP_INFO(get_logger(), "========== %s DONE ==========", axis_name.c_str());
        RCLCPP_INFO(get_logger(), "");
    }

    void FinishScan()
    {
        scanning_.store(false);
        std_msgs::msg::Bool done_msg;
        done_msg.data = true;
        pub_scan_done_->publish(done_msg);
        RCLCPP_INFO(get_logger(), "[DXL] /scan_done published");
    }

    Params            p_;
    MotorAxis         left_, right_;
    ScanStats         scan_stats_;
    std::set<int>     empty_slots_set_;
    std::atomic<bool> scanning_{false};

    dynamixel::PortHandler*   port_handler_   = nullptr;
    dynamixel::PacketHandler* packet_handler_ = nullptr;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                      pub_scan_done_;
    rclcpp::Publisher<scanner_interfaces::msg::TiltState>::SharedPtr       pub_tilt_;
    rclcpp::Subscription<scanner_interfaces::msg::BarcodeEvent>::SharedPtr sub_barcode_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                   sub_alignment_done_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    try {
        auto node = std::make_shared<DynamixelScanNode>(options);
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("dynamixel_scan_node"),
            "exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}