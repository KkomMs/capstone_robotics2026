/**
 * dynamixel_scan_node.cpp
 *
 * 스캔 시퀀스:
 *   /alignment_done → P0 → P1
 *   P1 완료 → /dxl/p1_done + P2a (crab 이동 중)
 *   /crab_done → P2b step 스캔
 *   P2b 완료 → /dxl/p2b_done + P3a (crab 복귀 중)
 *   /crab_return_done → P3b_step (step=10, p3a_end→p3b_step_end)
 *                     → 일반 랙: P3b_sweep (vel=1, p3b_step_end→end)
 *                     → 마커ID=3 랙: P3b_fine_step (step=3, p3b_step_end→end)
 *   P3b 완료 → 초기 위치 복귀 → /scan_done
 */

#include "scanner_interfaces/msg/barcode_event.hpp"
#include "scanner_interfaces/msg/tilt_state.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
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

// 마커 ID 3에 해당하는 랙 (오른쪽 마커 기준)
static constexpr int kRack3MarkerId = 3;

enum class MotorState {
    P0_STEP_MOVE, P0_STEP_WAIT,
    P1_SWEEP,
    P2A_MOVE,
    P2B_STEP_MOVE, P2B_STEP_WAIT,
    P3A_MOVE,
    P3B_STEP_MOVE, P3B_STEP_WAIT,        // step=10, p3a_end → p3b_step_end
    P3B_SWEEP,                            // vel=1,   p3b_step_end → end  (일반 랙)
    P3B_FINE_STEP_MOVE, P3B_FINE_STEP_WAIT,  // step=3, p3b_step_end → end  (마커ID=3 랙)
    RETURNING,
    DONE
};

enum class ScanPhase {
    IDLE,
    P0_P1,
    WAIT_CRAB,
    P2B_RUNNING,
    WAIT_CRAB_RTN,
    P3B_RUNNING,
    RETURNING,
};

struct MotorAxis {
    uint8_t     id   = 0;
    std::string name;

    int32_t start        = 0;
    int32_t p0_end       = 0;
    int32_t p1_end       = 0;
    int32_t p2a_end      = 0;
    int32_t p2_end       = 0;
    int32_t p3a_end      = 0;
    int32_t p3b_step_end = 0;
    int32_t end          = 0;

    int vel_p1     = 4;
    int vel_p2a    = 10;
    int vel_p3a    = 10;
    int vel_p3b    = 1;
    int vel_step   = 1;
    int vel_return = 10;

    int p0_step   = 40;
    int p2_step   = 20;
    int p3b_step  = 10;
    int p3b_fine_step = 3;   // 마커ID=3 랙 전용 fine step

    int32_t      current_tick    = 0;
    int32_t      goal_tick       = 0;
    bool         summary_printed = false;
    MotorState   state           = MotorState::DONE;
    rclcpp::Time wait_start;

    bool arrived(int thr) const {
        return std::abs(current_tick - goal_tick) <= thr;
    }
};

struct SlotRecord { std::string serial; int unit = -1; };

struct ScanStats {
    std::map<int, std::map<int, SlotRecord>> records_by_sid;
    void Reset() { records_by_sid.clear(); }
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
        pub_p1_done_  = this->create_publisher<std_msgs::msg::Bool>("/dxl/p1_done",  qos);
        pub_p2b_done_ = this->create_publisher<std_msgs::msg::Bool>("/dxl/p2b_done", qos);

        sub_barcode_ = this->create_subscription<scanner_interfaces::msg::BarcodeEvent>(
            "/barcode/unit_event", rclcpp::QoS(100),
            [this](const scanner_interfaces::msg::BarcodeEvent::SharedPtr msg) {
                OnBarcodeEvent(*msg);
            });

        sub_aruco_left_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers_left", qos,
            [this](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                if (!msg->marker_ids.empty())
                    current_rack_marker_id_ = static_cast<int>(msg->marker_ids[0]);
            });
        sub_aruco_right_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers_right", qos,
            [this](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                if (!msg->marker_ids.empty())
                    current_rack_marker_id_ = static_cast<int>(msg->marker_ids[0]);
            });

        sub_alignment_done_ = this->create_subscription<std_msgs::msg::Bool>(
            "/alignment_done", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && !scanning_.load()) {
                    RCLCPP_INFO(get_logger(),
                        "[DXL] alignment_done → %.1f초 후 스캔 시작 (마커ID=%d, %s)",
                        p_.alignment_done_delay_sec,
                        current_rack_marker_id_,
                        IsRack3() ? "랙3=fine_step" : "일반=sweep");
                    std::thread([this]() {
                        usleep(static_cast<useconds_t>(
                            p_.alignment_done_delay_sec * 1e6));
                        StartScan();
                    }).detach();
                }
            });

        sub_crab_done_ = this->create_subscription<std_msgs::msg::Bool>(
            "/crab_done", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (!msg->data || !scanning_.load()) return;
                if (scan_phase_ != ScanPhase::WAIT_CRAB) {
                    RCLCPP_WARN(get_logger(), "[DXL] /crab_done but phase!=WAIT_CRAB");
                    return;
                }
                RCLCPP_INFO(get_logger(), "[DXL] /crab_done → P2b 시작");
                scan_phase_ = ScanPhase::P2B_RUNNING;
                StartP2b(left_);
                StartP2b(right_);
            });

        sub_crab_return_done_ = this->create_subscription<std_msgs::msg::Bool>(
            "/crab_return_done", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (!msg->data || !scanning_.load()) return;
                if (scan_phase_ != ScanPhase::WAIT_CRAB_RTN) {
                    RCLCPP_WARN(get_logger(), "[DXL] /crab_return_done but phase!=WAIT_CRAB_RTN");
                    return;
                }
                RCLCPP_INFO(get_logger(), "[DXL] /crab_return_done → P3b_step 시작");
                scan_phase_ = ScanPhase::P3B_RUNNING;
                StartP3bStep(left_);
                StartP3bStep(right_);
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(p_.control_period_ms),
            std::bind(&DynamixelScanNode::ControlLoop, this));

        RCLCPP_INFO(get_logger(),
            "[DXL] node ready | left_id=%d right_id=%d",
            p_.left_id, p_.right_id);
    }

    ~DynamixelScanNode() { CloseDxl(); }

private:
    // 현재 랙이 마커ID=3인지 (P3b fine step 적용)
    bool IsRack3() const { return current_rack_marker_id_ == kRack3MarkerId; }

    struct Params {
        std::string device   = "/dev/ttyDynamixel";
        int         baudrate = 57600;
        int left_id=6, right_id=7;
        int left_scanner_id=1, right_scanner_id=2;

        int32_t left_start=330, left_p0_end=210, left_p1_end=-630;
        int32_t left_p2a_end=-720, left_p2_end=-920;
        int32_t left_p3a_end=-800;
        int32_t left_p3b_step_end=-870;
        int32_t left_end=-970;

        int32_t right_start=1470, right_p0_end=1350, right_p1_end=510;
        int32_t right_p2a_end=630, right_p2_end=390;
        int32_t right_p3a_end=325;
        int32_t right_p3b_step_end=250;
        int32_t right_end=150;

        int left_vel_p1=4, left_vel_p2a=10, left_vel_p3a=10;
        int left_vel_p3b=1, left_vel_step=1, left_vel_return=10;

        int right_vel_p1=4, right_vel_p2a=10, right_vel_p3a=10;
        int right_vel_p3b=1, right_vel_step=1, right_vel_return=10;

        int left_p0_step=40,  left_p2_step=20,  left_p3b_step=10,  left_p3b_fine_step=3;
        int right_p0_step=40, right_p2_step=20, right_p3b_step=10, right_p3b_fine_step=3;

        int    arrive_threshold=20;
        int    step_timeout_ms=3000;
        double alignment_done_delay_sec=2.0;
        int    control_period_ms=100;
        int              total_slots=51;
        std::vector<int> empty_slots={};
    };

    void LoadParams()
    {
        auto gi  = [this](const std::string & n){ return (int)get_parameter(n).as_int(); };
        auto gd  = [this](const std::string & n){ return get_parameter(n).as_double(); };
        auto gs  = [this](const std::string & n){ return get_parameter(n).as_string(); };
        auto giv = [this](const std::string & n) {
            auto v = get_parameter(n).as_integer_array();
            std::vector<int> o; for (auto x:v) o.push_back((int)x); return o;
        };

        p_.device   = gs("dxl_device");
        p_.baudrate = gi("dxl_baudrate");
        p_.left_id  = gi("dxl_left_id");
        p_.right_id = gi("dxl_right_id");
        p_.left_scanner_id  = gi("dxl_left_scanner_id");
        p_.right_scanner_id = gi("dxl_right_scanner_id");

        p_.left_start        = gi("dxl_left_start");
        p_.left_p0_end       = gi("dxl_left_p0_end");
        p_.left_p1_end       = gi("dxl_left_p1_end");
        p_.left_p2a_end      = gi("dxl_left_p2a_end");
        p_.left_p2_end       = gi("dxl_left_p2_end");
        p_.left_p3a_end      = gi("dxl_left_p3a_end");
        p_.left_p3b_step_end = gi("dxl_left_p3b_step_end");
        p_.left_end          = gi("dxl_left_end");

        p_.right_start        = gi("dxl_right_start");
        p_.right_p0_end       = gi("dxl_right_p0_end");
        p_.right_p1_end       = gi("dxl_right_p1_end");
        p_.right_p2a_end      = gi("dxl_right_p2a_end");
        p_.right_p2_end       = gi("dxl_right_p2_end");
        p_.right_p3a_end      = gi("dxl_right_p3a_end");
        p_.right_p3b_step_end = gi("dxl_right_p3b_step_end");
        p_.right_end          = gi("dxl_right_end");

        p_.left_vel_p1=gi("dxl_left_vel_p1");
        p_.left_vel_p2a=gi("dxl_left_vel_p2a");
        p_.left_vel_p3a=gi("dxl_left_vel_p3a");
        p_.left_vel_p3b=gi("dxl_left_vel_p3b");
        p_.left_vel_step=gi("dxl_left_vel_step");
        p_.left_vel_return=gi("dxl_left_vel_return");

        p_.right_vel_p1=gi("dxl_right_vel_p1");
        p_.right_vel_p2a=gi("dxl_right_vel_p2a");
        p_.right_vel_p3a=gi("dxl_right_vel_p3a");
        p_.right_vel_p3b=gi("dxl_right_vel_p3b");
        p_.right_vel_step=gi("dxl_right_vel_step");
        p_.right_vel_return=gi("dxl_right_vel_return");

        p_.left_p0_step=gi("dxl_left_p0_step");
        p_.left_p2_step=gi("dxl_left_p2_step");
        p_.left_p3b_step=gi("dxl_left_p3b_step");
        p_.left_p3b_fine_step=gi("dxl_left_p3b_fine_step");

        p_.right_p0_step=gi("dxl_right_p0_step");
        p_.right_p2_step=gi("dxl_right_p2_step");
        p_.right_p3b_step=gi("dxl_right_p3b_step");
        p_.right_p3b_fine_step=gi("dxl_right_p3b_fine_step");

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
        auto fill = [](MotorAxis & ax, uint8_t id, const std::string & nm,
            int32_t st, int32_t p0e, int32_t p1e,
            int32_t p2ae, int32_t p2e,
            int32_t p3ae, int32_t p3bse, int32_t en,
            int vp1, int vp2a, int vp3a, int vp3b, int vstep, int vret,
            int s0, int s2, int s3b, int s3bf)
        {
            ax.id=id; ax.name=nm;
            ax.start=st; ax.p0_end=p0e; ax.p1_end=p1e;
            ax.p2a_end=p2ae; ax.p2_end=p2e;
            ax.p3a_end=p3ae; ax.p3b_step_end=p3bse; ax.end=en;
            ax.vel_p1=vp1; ax.vel_p2a=vp2a; ax.vel_p3a=vp3a;
            ax.vel_p3b=vp3b; ax.vel_step=vstep; ax.vel_return=vret;
            ax.p0_step=s0; ax.p2_step=s2; ax.p3b_step=s3b; ax.p3b_fine_step=s3bf;
            ax.state=MotorState::DONE;
        };

        fill(left_, p_.left_id, "LEFT",
             p_.left_start, p_.left_p0_end, p_.left_p1_end,
             p_.left_p2a_end, p_.left_p2_end,
             p_.left_p3a_end, p_.left_p3b_step_end, p_.left_end,
             p_.left_vel_p1, p_.left_vel_p2a, p_.left_vel_p3a,
             p_.left_vel_p3b, p_.left_vel_step, p_.left_vel_return,
             p_.left_p0_step, p_.left_p2_step, p_.left_p3b_step, p_.left_p3b_fine_step);

        fill(right_, p_.right_id, "RIGHT",
             p_.right_start, p_.right_p0_end, p_.right_p1_end,
             p_.right_p2a_end, p_.right_p2_end,
             p_.right_p3a_end, p_.right_p3b_step_end, p_.right_end,
             p_.right_vel_p1, p_.right_vel_p2a, p_.right_vel_p3a,
             p_.right_vel_p3b, p_.right_vel_step, p_.right_vel_return,
             p_.right_p0_step, p_.right_p2_step, p_.right_p3b_step, p_.right_p3b_fine_step);
    }

    void InitDxl()
    {
        port_handler_   = dynamixel::PortHandler::getPortHandler(p_.device.c_str());
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        if (!port_handler_->openPort())
            throw std::runtime_error("[DXL] port open failed: " + p_.device);
        if (!port_handler_->setBaudRate(p_.baudrate))
            throw std::runtime_error("[DXL] baudrate failed");
        usleep(100'000);
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
            packet_handler_->write1ByteTxRx(
                port_handler_, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, &err);
        port_handler_->closePort();
    }

    int32_t ReadPos(uint8_t id)
    {
        int32_t v = 0; uint8_t err = 0;
        packet_handler_->read4ByteTxRx(
            port_handler_, id, ADDR_PRESENT_POSITION, (uint32_t*)&v, &err);
        return v;
    }

    void WriteVelGoal(uint8_t id, int vel, int32_t goal)
    {
        uint8_t err = 0;
        packet_handler_->write4ByteTxRx(port_handler_, id, ADDR_PROFILE_VELOCITY, (uint32_t)vel,  &err);
        packet_handler_->write4ByteTxRx(port_handler_, id, ADDR_GOAL_POSITION,    (uint32_t)goal, &err);
    }

    // ── Phase 시작 함수 ──────────────────────────────────────────
    void StartScan()
    {
        scan_stats_.Reset();
        scanning_.store(true);
        scan_phase_ = ScanPhase::P0_P1;
        left_.summary_printed = right_.summary_printed = false;
        left_.goal_tick  = left_.start;
        right_.goal_tick = right_.start;
        StartP0(left_); StartP0(right_);
        RCLCPP_INFO(get_logger(), "[DXL] scan started → P0");
    }

    void StartP0(MotorAxis & ax)
    {
        ax.state = MotorState::P0_STEP_MOVE;
        RCLCPP_INFO(get_logger(), "[%s] P0: %d→%d step=%d",
            ax.name.c_str(), ax.start, ax.p0_end, ax.p0_step);
        SendStep(ax, ax.p0_end, ax.p0_step);
    }

    void StartP1(MotorAxis & ax)
    {
        ax.goal_tick = ax.p1_end;
        ax.state     = MotorState::P1_SWEEP;
        WriteVelGoal(ax.id, ax.vel_p1, ax.p1_end);
        RCLCPP_INFO(get_logger(), "[%s] P1 sweep vel=%d goal=%d",
            ax.name.c_str(), ax.vel_p1, ax.p1_end);
    }

    void StartP2a(MotorAxis & ax)
    {
        ax.goal_tick = ax.p2a_end;
        ax.state     = MotorState::P2A_MOVE;
        WriteVelGoal(ax.id, ax.vel_p2a, ax.p2a_end);
        RCLCPP_INFO(get_logger(), "[%s] P2a fast vel=%d goal=%d",
            ax.name.c_str(), ax.vel_p2a, ax.p2a_end);
    }

    void StartP2b(MotorAxis & ax)
    {
        ax.state = MotorState::P2B_STEP_MOVE;
        RCLCPP_INFO(get_logger(), "[%s] P2b step goal=%d step=%d",
            ax.name.c_str(), ax.p2_end, ax.p2_step);
        SendStep(ax, ax.p2_end, ax.p2_step);
    }

    void StartP3a(MotorAxis & ax)
    {
        ax.goal_tick = ax.p3a_end;
        ax.state     = MotorState::P3A_MOVE;
        WriteVelGoal(ax.id, ax.vel_p3a, ax.p3a_end);
        RCLCPP_INFO(get_logger(), "[%s] P3a fast vel=%d goal=%d",
            ax.name.c_str(), ax.vel_p3a, ax.p3a_end);
    }

    // P3b 1단계: step=p3b_step, p3a_end → p3b_step_end
    void StartP3bStep(MotorAxis & ax)
    {
        ax.state = MotorState::P3B_STEP_MOVE;
        RCLCPP_INFO(get_logger(), "[%s] P3b_step goal=%d step=%d",
            ax.name.c_str(), ax.p3b_step_end, ax.p3b_step);
        SendStep(ax, ax.p3b_step_end, ax.p3b_step);
    }

    // P3b 2단계 (일반): vel=vel_p3b(1), sweep
    void StartP3bSweep(MotorAxis & ax)
    {
        ax.goal_tick = ax.end;
        ax.state     = MotorState::P3B_SWEEP;
        WriteVelGoal(ax.id, ax.vel_p3b, ax.end);
        RCLCPP_INFO(get_logger(), "[%s] P3b_sweep vel=%d goal=%d",
            ax.name.c_str(), ax.vel_p3b, ax.end);
    }

    // P3b 2단계 (마커ID=3 랙): step=p3b_fine_step(3), p3b_step_end → end
    void StartP3bFineStep(MotorAxis & ax)
    {
        ax.state = MotorState::P3B_FINE_STEP_MOVE;
        RCLCPP_INFO(get_logger(), "[%s] P3b_fine_step goal=%d step=%d (마커ID=3)",
            ax.name.c_str(), ax.end, ax.p3b_fine_step);
        SendStep(ax, ax.end, ax.p3b_fine_step);
    }

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
        usleep(50'000);
        StepToNext(ax);
    }

    void StepToNext(MotorAxis & ax)
    {
        // P0
        if (ax.state == MotorState::P0_STEP_MOVE || ax.state == MotorState::P0_STEP_WAIT) {
            if (ax.current_tick <= ax.p0_end + p_.arrive_threshold) {
                StartP1(ax); return;
            }
            ax.state = MotorState::P0_STEP_MOVE;
            SendStep(ax, ax.p0_end, ax.p0_step); return;
        }
        // P2b
        if (ax.state == MotorState::P2B_STEP_MOVE || ax.state == MotorState::P2B_STEP_WAIT) {
            if (ax.current_tick <= ax.p2_end + p_.arrive_threshold) {
                ax.state = MotorState::DONE; return;
            }
            ax.state = MotorState::P2B_STEP_MOVE;
            SendStep(ax, ax.p2_end, ax.p2_step); return;
        }
        // P3b step (1단계)
        if (ax.state == MotorState::P3B_STEP_MOVE || ax.state == MotorState::P3B_STEP_WAIT) {
            if (ax.current_tick <= ax.p3b_step_end + p_.arrive_threshold) {
                // 마커ID=3 랙이고 오른쪽 축만 fine step, 나머지는 sweep
                bool is_right = (&ax == &right_);
                if (IsRack3() && is_right) {
                    RCLCPP_INFO(get_logger(), "[%s] P3b_step 완료 → P3b_fine_step (마커ID=3 오른쪽)",
                        ax.name.c_str());
                    StartP3bFineStep(ax);
                } else {
                    RCLCPP_INFO(get_logger(), "[%s] P3b_step 완료 → P3b_sweep",
                        ax.name.c_str());
                    StartP3bSweep(ax);
                }
                return;
            }
            ax.state = MotorState::P3B_STEP_MOVE;
            SendStep(ax, ax.p3b_step_end, ax.p3b_step); return;
        }
        // P3b fine step (2단계, 마커ID=3 전용)
        if (ax.state == MotorState::P3B_FINE_STEP_MOVE || ax.state == MotorState::P3B_FINE_STEP_WAIT) {
            if (ax.current_tick <= ax.end + p_.arrive_threshold) {
                StartReturn(ax); return;
            }
            ax.state = MotorState::P3B_FINE_STEP_MOVE;
            SendStep(ax, ax.end, ax.p3b_fine_step); return;
        }
    }

    void OnBarcodeEvent(const scanner_interfaces::msg::BarcodeEvent & msg)
    {
        if (!scanning_.load()) return;
        int slot = msg.unit, sid = msg.scanner_id;
        if (slot < 1 || slot > p_.total_slots) return;
        scan_stats_.Record(slot, sid, msg.serial);
        RCLCPP_INFO(get_logger(), "[DXL] barcode sid=%d slot=%02d serial=%s",
            sid, slot, msg.serial.c_str());

        MotorAxis * ax = nullptr;
        if      (sid == p_.left_scanner_id)  ax = &left_;
        else if (sid == p_.right_scanner_id) ax = &right_;
        else { RCLCPP_WARN(get_logger(), "[DXL] unknown sid=%d", sid); return; }

        if (ax->state == MotorState::P2B_STEP_MOVE  ||
            ax->state == MotorState::P3B_STEP_MOVE   ||
            ax->state == MotorState::P3B_FINE_STEP_MOVE)
            HaltAndNext(*ax);
        else if (ax->state == MotorState::P2B_STEP_WAIT  ||
                 ax->state == MotorState::P3B_STEP_WAIT   ||
                 ax->state == MotorState::P3B_FINE_STEP_WAIT)
            StepToNext(*ax);
    }

    void ControlLoop()
    {
        if (!scanning_.load()) return;

        left_.current_tick  = ReadPos(left_.id);
        right_.current_tick = ReadPos(right_.id);

        scanner_interfaces::msg::TiltState tilt;
        tilt.stamp = this->now();
        tilt.left_tick  = left_.current_tick;
        tilt.right_tick = right_.current_tick;
        pub_tilt_->publish(tilt);

        UpdateAxis(left_);
        UpdateAxis(right_);

        auto both_done = [this]() {
            return left_.state == MotorState::DONE && right_.state == MotorState::DONE;
        };
        auto pub_bool = [](auto & pub) {
            std_msgs::msg::Bool m; m.data=true; pub->publish(m);
        };

        switch (scan_phase_) {
        case ScanPhase::P0_P1:
            if (both_done()) {
                RCLCPP_INFO(get_logger(), "[DXL] P1 완료 → /dxl/p1_done + P2a");
                scan_phase_ = ScanPhase::WAIT_CRAB;
                StartP2a(left_); StartP2a(right_);
                pub_bool(pub_p1_done_);
            }
            break;
        case ScanPhase::WAIT_CRAB: break;
        case ScanPhase::P2B_RUNNING:
            if (both_done()) {
                RCLCPP_INFO(get_logger(), "[DXL] P2b 완료 → /dxl/p2b_done + P3a");
                scan_phase_ = ScanPhase::WAIT_CRAB_RTN;
                StartP3a(left_); StartP3a(right_);
                pub_bool(pub_p2b_done_);
            }
            break;
        case ScanPhase::WAIT_CRAB_RTN: break;
        case ScanPhase::P3B_RUNNING:
            if (both_done()) {
                RCLCPP_INFO(get_logger(), "[DXL] P3b 완료 → 복귀");
                scan_phase_ = ScanPhase::RETURNING;
                StartReturn(left_); StartReturn(right_);
            }
            break;
        case ScanPhase::RETURNING:
            if (both_done()) {
                RCLCPP_INFO(get_logger(), "[DXL] 복귀 완료 → scan finish");
                FinishScan();
            }
            break;
        case ScanPhase::IDLE: default: break;
        }
    }

    void UpdateAxis(MotorAxis & ax)
    {
        switch (ax.state) {
        case MotorState::P0_STEP_MOVE:
            if (!ax.arrived(p_.arrive_threshold)) return;
            ax.state = MotorState::P0_STEP_WAIT;
            ax.wait_start = this->now(); break;
        case MotorState::P0_STEP_WAIT:
            if ((this->now()-ax.wait_start).seconds()*1000.0 < p_.step_timeout_ms) return;
            RCLCPP_WARN(get_logger(), "[%s P0 TIMEOUT]", ax.name.c_str());
            StepToNext(ax); break;
        case MotorState::P1_SWEEP:
            if (!ax.arrived(p_.arrive_threshold)) return;
            RCLCPP_INFO(get_logger(), "[%s] P1 done", ax.name.c_str());
            ax.state = MotorState::DONE; break;
        case MotorState::P2A_MOVE:
            if (!ax.arrived(p_.arrive_threshold)) return;
            RCLCPP_INFO(get_logger(), "[%s] P2a done", ax.name.c_str());
            ax.state = MotorState::DONE; break;
        case MotorState::P2B_STEP_MOVE:
            if (!ax.arrived(p_.arrive_threshold)) return;
            ax.state = MotorState::P2B_STEP_WAIT;
            ax.wait_start = this->now();
            RCLCPP_INFO(get_logger(), "[%s P2b WAIT] tick=%d", ax.name.c_str(), ax.current_tick); break;
        case MotorState::P2B_STEP_WAIT:
            if ((this->now()-ax.wait_start).seconds()*1000.0 < p_.step_timeout_ms) return;
            RCLCPP_WARN(get_logger(), "[%s P2b TIMEOUT]", ax.name.c_str());
            StepToNext(ax); break;
        case MotorState::P3A_MOVE:
            if (!ax.arrived(p_.arrive_threshold)) return;
            RCLCPP_INFO(get_logger(), "[%s] P3a done", ax.name.c_str());
            ax.state = MotorState::DONE; break;
        case MotorState::P3B_STEP_MOVE:
            if (!ax.arrived(p_.arrive_threshold)) return;
            ax.state = MotorState::P3B_STEP_WAIT;
            ax.wait_start = this->now();
            RCLCPP_INFO(get_logger(), "[%s P3b_step WAIT] tick=%d", ax.name.c_str(), ax.current_tick); break;
        case MotorState::P3B_STEP_WAIT:
            if ((this->now()-ax.wait_start).seconds()*1000.0 < p_.step_timeout_ms) return;
            RCLCPP_WARN(get_logger(), "[%s P3b_step TIMEOUT]", ax.name.c_str());
            StepToNext(ax); break;
        case MotorState::P3B_SWEEP:
            if (!ax.arrived(p_.arrive_threshold)) return;
            RCLCPP_INFO(get_logger(), "[%s] P3b_sweep done → return", ax.name.c_str());
            StartReturn(ax); break;
        case MotorState::P3B_FINE_STEP_MOVE:
            if (!ax.arrived(p_.arrive_threshold)) return;
            ax.state = MotorState::P3B_FINE_STEP_WAIT;
            ax.wait_start = this->now();
            RCLCPP_INFO(get_logger(), "[%s P3b_fine WAIT] tick=%d", ax.name.c_str(), ax.current_tick); break;
        case MotorState::P3B_FINE_STEP_WAIT:
            if ((this->now()-ax.wait_start).seconds()*1000.0 < p_.step_timeout_ms) return;
            RCLCPP_WARN(get_logger(), "[%s P3b_fine TIMEOUT]", ax.name.c_str());
            StepToNext(ax); break;
        case MotorState::RETURNING:
            if (!ax.arrived(p_.arrive_threshold)) return;
            RCLCPP_INFO(get_logger(), "[%s] returned", ax.name.c_str());
            ax.state = MotorState::DONE; break;
        case MotorState::DONE: break;
        }
    }

    void PrintAxisSummary(const std::string & axis_name, int sid)
    {
        RCLCPP_INFO(get_logger(), "\n========== %s RESULT ==========", axis_name.c_str());
        int ok=0, empty_cnt=0, miss=0;
        auto sit = scan_stats_.records_by_sid.find(sid);
        for (int u=1; u<=p_.total_slots; ++u) {
            bool is_empty = empty_slots_set_.count(u) > 0;
            bool detected = sit != scan_stats_.records_by_sid.end() &&
                            sit->second.count(u) > 0;
            if (detected) {
                ok++;
                RCLCPP_INFO(get_logger(), "  U%02d : OK serial=%s",
                    u, sit->second.at(u).serial.c_str());
            } else if (is_empty) {
                empty_cnt++;
                RCLCPP_INFO(get_logger(), "  U%02d : [빈 슬롯]", u);
            } else {
                miss++;
                RCLCPP_WARN(get_logger(), "  U%02d : NOT_SEEN", u);
            }
        }
        int filled = p_.total_slots - (int)empty_slots_set_.size();
        RCLCPP_INFO(get_logger(), "  인식: %d / %d (미인식: %d)", ok, filled, miss);
        RCLCPP_INFO(get_logger(), "========== %s DONE ==========\n", axis_name.c_str());
    }

    void FinishScan()
    {
        scanning_.store(false);
        scan_phase_ = ScanPhase::IDLE;
        std_msgs::msg::Bool m; m.data=true;
        pub_scan_done_->publish(m);
        RCLCPP_INFO(get_logger(), "[DXL] /scan_done published");
    }

    Params        p_;
    MotorAxis     left_, right_;
    ScanStats     scan_stats_;
    std::set<int> empty_slots_set_;

    std::atomic<bool> scanning_{false};
    ScanPhase         scan_phase_            = ScanPhase::IDLE;
    int               current_rack_marker_id_ = -1;  // 오른쪽 또는 왼쪽 마커 ID

    dynamixel::PortHandler*   port_handler_   = nullptr;
    dynamixel::PacketHandler* packet_handler_ = nullptr;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_scan_done_;
    rclcpp::Publisher<scanner_interfaces::msg::TiltState>::SharedPtr pub_tilt_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_p1_done_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_p2b_done_;

    rclcpp::Subscription<scanner_interfaces::msg::BarcodeEvent>::SharedPtr    sub_barcode_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                      sub_alignment_done_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                      sub_crab_done_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                      sub_crab_return_done_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr sub_aruco_left_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr sub_aruco_right_;

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