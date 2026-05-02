/**
 * dynamixel_scan_node.cpp
 *
 * 변경사항:
 *  - 모든 하드코딩 상수(#define) → yaml 파라미터로 이전
 *  - /alignment_done=true 수신 → alignment_done_delay_sec 대기 후 스캔 시작
 *  - /barcode/unit_event 구독 → 슬롯 집계
 *  - 스캔 완료 → 유닛 01~N 결과 터미널 출력 → /scan_done publish
 */

#include "scanner_interfaces/msg/barcode_event.hpp"
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

// ── Dynamixel 레지스터 주소 (프로토콜 고정값이므로 파라미터 불필요) ──
static constexpr uint16_t ADDR_OPERATING_MODE   = 11;
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
static constexpr uint16_t ADDR_GOAL_POSITION    = 116;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
static constexpr double   PROTOCOL_VERSION      = 2.0;
static constexpr uint8_t  EXT_POSITION_MODE     = 4;
static constexpr uint8_t  TORQUE_ENABLE_VAL     = 1;
static constexpr uint8_t  TORQUE_DISABLE_VAL    = 0;

// ── 모터 상태 ─────────────────────────────────────────────────
enum class MotorState {
    P1_SWEEP, P2_SWEEP, STEP_MOVE, STEP_WAIT, RETURNING, DONE
};

struct MotorAxis {
    uint8_t     id      = 0;
    std::string name;
    int32_t start   = 0, p1_end = 0, p2_end = 0, end = 0;
    int     vel_p1  = 1, vel_p2 = 1, vel_step = 1, vel_return = 10;
    int     p2_step = 10, p3_step = 5;
    bool    p2_is_sweep = false;
    int32_t current_tick = 0;
    int32_t goal_tick    = 0;
    MotorState   state      = MotorState::DONE;
    rclcpp::Time wait_start;

    bool arrived(int threshold) const {
        return std::abs(current_tick - goal_tick) <= threshold;
    }
};

// ── 바코드 집계 ───────────────────────────────────────────────
struct SlotRecord {
    std::string serial;
    int         unit = -1;
};

struct ScanStats {
    std::map<int, SlotRecord>   records;
    std::map<int, rclcpp::Time> first_time;
    std::map<int, rclcpp::Time> last_time;
    std::map<int, bool>         has_any;

    void Reset() {
        records.clear(); first_time.clear();
        last_time.clear(); has_any.clear();
    }

    void Record(int slot, int sid, const std::string & serial, const rclcpp::Time & t) {
        records[slot] = {serial, slot};
        if (!has_any[sid]) { first_time[sid] = t; has_any[sid] = true; }
        last_time[sid] = t;
    }

    void PrintResult(rclcpp::Logger logger,
                     int total_slots,
                     const std::set<int> & empty_slots) const
    {
        RCLCPP_INFO(logger, "");
        RCLCPP_INFO(logger, "========================================");
        RCLCPP_INFO(logger, "      바코드 스캔 결과 (U01~U%02d)      ", total_slots);
        RCLCPP_INFO(logger, "========================================");

        int filled_det = 0, empty_false = 0, empty_ok = 0;
        int filled_count = total_slots - (int)empty_slots.size();

        for (int slot = 1; slot <= total_slots; ++slot) {
            bool is_empty = empty_slots.count(slot) > 0;
            auto it       = records.find(slot);
            bool detected = (it != records.end());

            if (detected) {
                if (is_empty) {
                    RCLCPP_WARN(logger,
                        "  U%02d | [오인식] serial=%s  ← 빈 슬롯인데 감지됨",
                        slot, it->second.serial.c_str());
                    ++empty_false;
                } else {
                    RCLCPP_INFO(logger, "  U%02d | serial=%s",
                        slot, it->second.serial.c_str());
                    ++filled_det;
                }
            } else {
                if (is_empty) {
                    RCLCPP_INFO(logger, "  U%02d | [빈 슬롯]", slot);
                    ++empty_ok;
                } else {
                    RCLCPP_INFO(logger, "  U%02d | empty (미인식)", slot);
                }
            }
        }

        RCLCPP_INFO(logger, "========================================");
        RCLCPP_INFO(logger, "  바코드 슬롯 인식: %d / %d (%.1f%%)",
            filled_det, filled_count,
            filled_count > 0 ? (double)filled_det / filled_count * 100.0 : 0.0);
        RCLCPP_INFO(logger, "  빈 슬롯 정답:     %d / %d",
            empty_ok, (int)empty_slots.size());
        if (empty_false > 0)
            RCLCPP_WARN(logger, "  빈 슬롯 오인식:   %d건", empty_false);
        int total_correct = filled_det + empty_ok;
        RCLCPP_INFO(logger, "  전체 정답률:      %d / %d (%.1f%%)",
            total_correct, total_slots,
            (double)total_correct / total_slots * 100.0);
        RCLCPP_INFO(logger, "========================================");
        RCLCPP_INFO(logger, "");
    }
};

// ── ROS2 Node ────────────────────────────────────────────────
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
    // ── 파라미터 구조체 ────────────────────────────────────────
    struct Params {
        // 포트
        std::string device   = "/dev/ttyUSBDynamixel";
        int         baudrate = 57600;

        // 모터 ID
        int left_id  = 6;
        int right_id = 7;

        // 왼쪽 축 위치 (tick)
        int32_t left_start  =  330;
        int32_t left_p1_end = -660;
        int32_t left_p2_end = -870;
        int32_t left_end    = -980;

        // 오른쪽 축 위치 (tick)
        int32_t right_start  = 1470;
        int32_t right_p1_end =  480;
        int32_t right_p2_end =  270;
        int32_t right_end    =  140;

        // 왼쪽 속도
        int left_vel_p1     = 2;
        int left_vel_p2     = 1;
        int left_vel_step   = 1;
        int left_vel_return = 10;

        // 오른쪽 속도
        int right_vel_p1     = 4;
        int right_vel_p2     = 1;
        int right_vel_step   = 1;
        int right_vel_return = 10;

        // 스텝 크기 (tick)
        int left_p2_step  = 10;
        int left_p3_step  = 3;
        int right_p2_step = 15;
        int right_p3_step = 8;

        // P2 구간 sweep 여부
        bool left_p2_is_sweep  = false;
        bool right_p2_is_sweep = false;

        // 제어 파라미터
        int    arrive_threshold         = 20;
        int    step_timeout_ms          = 3000;
        double alignment_done_delay_sec = 2.0;
        int    control_period_ms        = 100;

        // 슬롯 설정
        int              total_slots = 51;
        std::vector<int> empty_slots = {9, 11, 14, 30, 39, 50};
    };

    // ── 파라미터 로드 ─────────────────────────────────────────
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

        p_.left_start  = (int32_t)gi("dxl_left_start");
        p_.left_p1_end = (int32_t)gi("dxl_left_p1_end");
        p_.left_p2_end = (int32_t)gi("dxl_left_p2_end");
        p_.left_end    = (int32_t)gi("dxl_left_end");

        p_.right_start  = (int32_t)gi("dxl_right_start");
        p_.right_p1_end = (int32_t)gi("dxl_right_p1_end");
        p_.right_p2_end = (int32_t)gi("dxl_right_p2_end");
        p_.right_end    = (int32_t)gi("dxl_right_end");

        p_.left_vel_p1     = gi("dxl_left_vel_p1");
        p_.left_vel_p2     = gi("dxl_left_vel_p2");
        p_.left_vel_step   = gi("dxl_left_vel_step");
        p_.left_vel_return = gi("dxl_left_vel_return");

        p_.right_vel_p1     = gi("dxl_right_vel_p1");
        p_.right_vel_p2     = gi("dxl_right_vel_p2");
        p_.right_vel_step   = gi("dxl_right_vel_step");
        p_.right_vel_return = gi("dxl_right_vel_return");

        p_.left_p2_step  = gi("dxl_left_p2_step");
        p_.left_p3_step  = gi("dxl_left_p3_step");
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

    // ── 축 초기화 ─────────────────────────────────────────────
    void SetupAxes()
    {
        left_  = { (uint8_t)p_.left_id,  "LEFT",
                   p_.left_start,  p_.left_p1_end,  p_.left_p2_end,  p_.left_end,
                   p_.left_vel_p1,  p_.left_vel_p2,  p_.left_vel_step,  p_.left_vel_return,
                   p_.left_p2_step,  p_.left_p3_step,  p_.left_p2_is_sweep };
        right_ = { (uint8_t)p_.right_id, "RIGHT",
                   p_.right_start, p_.right_p1_end, p_.right_p2_end, p_.right_end,
                   p_.right_vel_p1, p_.right_vel_p2, p_.right_vel_step, p_.right_vel_return,
                   p_.right_p2_step, p_.right_p3_step, p_.right_p2_is_sweep };
        left_.state  = MotorState::DONE;
        right_.state = MotorState::DONE;
    }

    // ── Dynamixel 초기화 / 종료 ───────────────────────────────
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

    // ── 스캔 시작 ─────────────────────────────────────────────
    void StartScan()
    {
        scan_stats_.Reset();
        scanning_.store(true);
        left_.goal_tick  = left_.start;
        right_.goal_tick = right_.start;
        StartP1(left_);
        StartP1(right_);
        RCLCPP_INFO(get_logger(), "[DXL] scan started");
    }

    // ── 바코드 이벤트 ─────────────────────────────────────────
    void OnBarcodeEvent(const scanner_interfaces::msg::BarcodeEvent & msg)
    {
        if (!scanning_.load()) return;
        int slot = msg.unit;
        int sid  = msg.scanner_id;
        if (slot < 1 || slot > p_.total_slots) return;

        scan_stats_.Record(slot, sid, msg.serial, this->now());
        RCLCPP_INFO(get_logger(),
            "[DXL] barcode sid=%d slot=%02d serial=%s → next step",
            sid, slot, msg.serial.c_str());

        for (auto ax : {&left_, &right_}) {
            if      (ax->state == MotorState::STEP_MOVE) HaltAndNext(*ax);
            else if (ax->state == MotorState::STEP_WAIT) StepToNext(*ax);
        }
    }

    // ── 모터 제어 ─────────────────────────────────────────────
    void StartP1(MotorAxis & ax)
    {
        ax.goal_tick = ax.p1_end;
        ax.state     = MotorState::P1_SWEEP;
        WriteVelGoal(ax.id, ax.vel_p1, ax.p1_end);
        RCLCPP_INFO(get_logger(), "[%s] P1 start vel=%d goal=%d",
            ax.name.c_str(), ax.vel_p1, ax.p1_end);
    }

    void StartP2(MotorAxis & ax)
    {
        if (ax.p2_is_sweep) {
            ax.goal_tick = ax.p2_end;
            ax.state     = MotorState::P2_SWEEP;
            WriteVelGoal(ax.id, ax.vel_p2, ax.p2_end);
        } else {
            ax.state = MotorState::STEP_MOVE;
            SendStep(ax, ax.p2_end, ax.p2_step);
            RCLCPP_INFO(get_logger(), "[%s] P2 step-and-wait", ax.name.c_str());
        }
    }

    void StartP3(MotorAxis & ax)
    {
        ax.state = MotorState::STEP_MOVE;
        SendStep(ax, ax.end, ax.p3_step);
        RCLCPP_INFO(get_logger(), "[%s] P3 step-and-wait", ax.name.c_str());
    }

    void StartReturn(MotorAxis & ax)
    {
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
        bool in_p2 = (!ax.p2_is_sweep) && (ax.goal_tick > ax.p2_end);
        if (in_p2) {
            if (ax.current_tick <= ax.p2_end + p_.arrive_threshold) { StartP3(ax); return; }
            ax.state = MotorState::STEP_MOVE;
            SendStep(ax, ax.p2_end, ax.p2_step);
        } else {
            if (ax.current_tick <= ax.end + p_.arrive_threshold) { StartReturn(ax); return; }
            ax.state = MotorState::STEP_MOVE;
            SendStep(ax, ax.end, ax.p3_step);
        }
    }

    // ── 제어 루프 ─────────────────────────────────────────────
    void ControlLoop()
    {
        if (!scanning_.load()) return;
        left_.current_tick  = ReadPos(left_.id);
        right_.current_tick = ReadPos(right_.id);
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
        case MotorState::P1_SWEEP:
            if (!ax.arrived(p_.arrive_threshold)) return;
            RCLCPP_INFO(get_logger(), "[%s] P1 done", ax.name.c_str());
            StartP2(ax); break;
        case MotorState::P2_SWEEP:
            if (!ax.arrived(p_.arrive_threshold)) return;
            RCLCPP_INFO(get_logger(), "[%s] P2 done", ax.name.c_str());
            StartP3(ax); break;
        case MotorState::STEP_MOVE:
            if (!ax.arrived(p_.arrive_threshold)) return;
            ax.state      = MotorState::STEP_WAIT;
            ax.wait_start = this->now();
            RCLCPP_INFO(get_logger(), "[%s WAIT] tick=%d", ax.name.c_str(), ax.current_tick);
            break;
        case MotorState::STEP_WAIT: {
            double elapsed_ms = (this->now() - ax.wait_start).seconds() * 1000.0;
            if (elapsed_ms < p_.step_timeout_ms) return;
            RCLCPP_WARN(get_logger(), "[%s TIMEOUT] tick=%d", ax.name.c_str(), ax.current_tick);
            StepToNext(ax); break;
        }
        case MotorState::RETURNING:
            if (!ax.arrived(p_.arrive_threshold)) return;
            RCLCPP_INFO(get_logger(), "[%s] returned", ax.name.c_str());
            ax.state = MotorState::DONE; break;
        case MotorState::DONE: break;
        }
    }

    // ── 스캔 완료 ─────────────────────────────────────────────
    void FinishScan()
    {
        scanning_.store(false);
        scan_stats_.PrintResult(this->get_logger(), p_.total_slots, empty_slots_set_);
        std_msgs::msg::Bool done_msg;
        done_msg.data = true;
        pub_scan_done_->publish(done_msg);
        RCLCPP_INFO(get_logger(), "[DXL] /scan_done published");
    }

    // ── 멤버 변수 ─────────────────────────────────────────────
    Params            p_;
    MotorAxis         left_, right_;
    ScanStats         scan_stats_;
    std::set<int>     empty_slots_set_;
    std::atomic<bool> scanning_{false};

    dynamixel::PortHandler*   port_handler_   = nullptr;
    dynamixel::PacketHandler* packet_handler_ = nullptr;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             pub_scan_done_;
    rclcpp::Subscription<scanner_interfaces::msg::BarcodeEvent>::SharedPtr sub_barcode_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          sub_alignment_done_;
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