/**
 * aruco_aligner_node.cpp
 *
 * 원본 코드 기반 + 추가 기능:
 *   - WHEEL_ALIGN 페이즈 (YAW_FINE 완료 후 바퀴 90도 수렴 대기)
 *   - /dxl/p1_done → 시간 기반 crab 왼쪽 이동 → /crab_done
 *   - /dxl/p2b_done → 시간 기반 crab 오른쪽 복귀 → /crab_return_done
 *
 * 기존 PublishFixed의 steer_tol 체크 유지
 * (조향 수렴 후 인휠 동작)
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "fwis_robot/kinematics.hpp"

namespace {

constexpr std::array<int, 4> kSteerMotorId   = {2, 1, 4, 3};
constexpr std::array<int, 4> kInWheelMotorId = {1, 2, 3, 4};

constexpr std::array<double, 4> kSpinSteerAngles    = {-45.0, 45.0,  45.0, -45.0};
constexpr std::array<double, 4> kForwardSteerAngles = {  0.0,  0.0,   0.0,   0.0};
constexpr std::array<double, 4> kCrabSteerAngles    = { 90.0, 90.0,  90.0,  90.0};

constexpr std::array<double, 4> kSpinInwheelSign = {-1.0, +1.0, -1.0, +1.0};
constexpr std::array<double, 4> kSameInwheelSign = {+1.0, +1.0, +1.0, +1.0};

constexpr double kYawFlipThreshold = M_PI * 0.5;

double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

double ApplyMinAbs(double v, double min_abs)
{
    if (v != 0.0 && std::fabs(v) < min_abs) return std::copysign(min_abs, v);
    return v;
}

double LpfLinear(double prev, double next, double alpha)
{
    return alpha * next + (1.0 - alpha) * prev;
}

double QuatToYaw(double qx, double qy, double qz, double qw)
{
    return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

double NormalizeAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

} // namespace

class ArucoAlignerNode : public rclcpp::Node
{
public:
    explicit ArucoAlignerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("aruco_aligner_node", options)
    {
        LoadParams();

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        for (int i = 0; i < 4; ++i) {
            steer_pubs_[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/motor_" + std::to_string(kSteerMotorId[i]) + "/steer", qos);
            inwheel_pubs_[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/motor_" + std::to_string(kInWheelMotorId[i]) + "/inwheel", qos);
        }
        done_pub_        = this->create_publisher<std_msgs::msg::Bool>("/alignment_done",    qos);
        pause_pub_       = this->create_publisher<std_msgs::msg::Bool>("/mobile_robot_pause", qos);
        detected_pub_    = this->create_publisher<std_msgs::msg::Bool>("/aruco_detected",    qos);
        crab_done_pub_   = this->create_publisher<std_msgs::msg::Bool>("/crab_done",         qos);
        crab_return_pub_ = this->create_publisher<std_msgs::msg::Bool>("/crab_return_done",  qos);

        left_sub_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers_left", qos,
            [this](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                MarkerCb(msg, Side::LEFT);
            });
        right_sub_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers_right", qos,
            [this](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                MarkerCb(msg, Side::RIGHT);
            });

        steer_front_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/steer_fb/front", qos,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() < 2) return;
                steer_fb_[0] = msg->data[1];
                steer_fb_[1] = msg->data[0];
            });
        steer_rear_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/steer_fb/rear", qos,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() < 2) return;
                steer_fb_[2] = msg->data[1];
                steer_fb_[3] = msg->data[0];
            });

        sub_scan_done_ = this->create_subscription<std_msgs::msg::Bool>(
            "/scan_done", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (!msg->data || state_ != State::DONE) return;
                RCLCPP_INFO(get_logger(), "[ArucoAligner] scan_done → 다음 사이클");
                ResetForNextCycle();
            });

        sub_aruco_start_ = this->create_subscription<std_msgs::msg::Bool>(
            "/aruco_start", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (!msg->data) return;
                RCLCPP_INFO(get_logger(), "[ArucoAligner] /aruco_start → 정렬 시작");
                StartAlign();
            });

        // DXL 핸드셰이크
        sub_p1_done_ = this->create_subscription<std_msgs::msg::Bool>(
            "/dxl/p1_done", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (!msg->data) return;
                OnP1Done();
            });

        sub_p2b_done_ = this->create_subscription<std_msgs::msg::Bool>(
            "/dxl/p2b_done", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (!msg->data) return;
                OnP2bDone();
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ArucoAlignerNode::ControlLoop, this));

        last_time_ = this->get_clock()->now();
        ApplyCurrentSequence();

        double crab_time = (p_.crab_vel > 0.0) ? p_.crab_move_dist / p_.crab_vel : 0.0;
        RCLCPP_INFO(get_logger(),
            "[ArucoAligner] start | 시퀀스=%zu세트 | crab=%.3fm@%.3fm/s=%.2f초 sign=%.1f",
            marker_sequences_.size(), p_.crab_move_dist, p_.crab_vel,
            crab_time, p_.crab_left_sign);
        LogCurrentSequence();
    }

private:
    enum class Side  { LEFT, RIGHT };
    enum class State {
        WAITING, DETECTED, ALIGNING, DONE,
        CRAB_MOVE,  // 시간 기반 왼쪽 crab
        CRAB_RTN,   // 시간 기반 오른쪽 crab 복귀
    };
    // Phase: 기존 + WHEEL_ALIGN 추가
    enum class Phase { YAW, YAW_SETTLE, X, Z, YAW_FINE, SEARCH, WHEEL_ALIGN };

    struct MF {
        bool   valid = false;
        int    id    = -1;
        double x = 0.0, z = 0.0, yaw = 0.0;
    };

    struct MarkerSet {
        std::vector<int> ids;
        double ref_x=0.0, ref_yaw_left=0.0, ref_yaw_right=0.0;
        double target_avg_dist=0.0, target_z_diff=0.0;
    };

    struct Params {
        // 기존 파라미터 (원본 그대로)
        double ref_x=0.0, ref_yaw_left=0.0, ref_yaw_right=0.0;
        double target_avg_dist=0.0, target_z_diff=0.0;
        int    marker_id_left=2, marker_id_right=1;

        double kp_w=2.0, kp_x=0.2, kp_y=0.2, kp_diff=0.2;
        double max_w=0.20, min_w=0.17;
        double max_vx=0.17, min_vx=0.13;
        double max_vy=0.17, min_vy=0.13;

        double thr_yaw=0.018, thr_center=0.015;
        double thr_dist=0.010, thr_diff=0.010;
        int    required_stable=3;

        double thr_yaw_enter_x=0.020, thr_yaw_back_to_yaw=0.025;
        int    yaw_ok_required_count=3;
        double x_phase_min_hold_time=0.7;
        double yaw_brake_band=0.03, yaw_brake_max_w=0.17;
        double yaw_settle_time=0.50;
        int    yaw_settle_required_count=2;
        int    both_visible_required=5;

        double marker_timeout=1.5, lpf_alpha=0.3, outlier_thr=0.5;
        double sign_yaw=-1.0, sign_x=1.0, sign_dist=-1.0, sign_diff=1.0;
        double steer_tol=5.0;  // ← 기존 steer_tol 유지

        // 추가 파라미터
        double wheel_align_tol=3.0;    // WHEEL_ALIGN 바퀴 수렴 허용 오차 [deg]
        int    wheel_stable_count=5;   // WHEEL_ALIGN 안정 카운트

        double crab_move_dist=0.43;    // crab 이동 거리 [m]
        double crab_vel=0.12;          // crab 이동 속도 [m/s]
        double crab_left_sign=1.0;     // 왼쪽 방향 부호

        // Kinematics (Z 페이즈 + crab IK용)
        double wheel_radius=0.0695;
        double wheel_x_offset=0.215;
        double wheel_y_offset=0.215;
        double deadzone_linear=0.005;
        double deadzone_angular=0.005;
    };

    struct Errors {
        bool   ok=false, both_visible=false;
        bool   left_visible=false, right_visible=false;
        double e_yaw=0.0, e_x=0.0, e_z_avg=0.0, e_z_diff=0.0;
        double e_yaw_l=0.0, e_yaw_r=0.0, e_z_diff_l=0.0;
    };

    // ════════════════════════════════════════════════════════
    //  파라미터 로드 (기존 + 추가분)
    // ════════════════════════════════════════════════════════
    void LoadParams()
    {
        auto g  = [this](const std::string & n){ return get_parameter(n).as_double(); };
        auto gi = [this](const std::string & n){ return (int)get_parameter(n).as_int(); };

        // 기존 파라미터 (원본 그대로)
        p_.ref_x=g("ref_x"); p_.ref_yaw_left=g("ref_yaw_left");
        p_.ref_yaw_right=g("ref_yaw_right");
        p_.target_avg_dist=g("target_avg_dist"); p_.target_z_diff=g("target_z_diff");
        p_.marker_id_left=gi("marker_id_left"); p_.marker_id_right=gi("marker_id_right");

        p_.kp_w=g("kp_w"); p_.kp_x=g("kp_x"); p_.kp_y=g("kp_y"); p_.kp_diff=g("kp_diff");
        p_.max_w=g("max_w"); p_.min_w=g("min_w");
        p_.max_vx=g("max_vx"); p_.min_vx=g("min_vx");
        p_.max_vy=g("max_vy"); p_.min_vy=g("min_vy");

        p_.thr_yaw=g("thr_yaw"); p_.thr_center=g("thr_center");
        p_.thr_dist=g("thr_dist"); p_.thr_diff=g("thr_diff");
        p_.required_stable=gi("required_stable");

        p_.thr_yaw_enter_x=g("thr_yaw_enter_x");
        p_.thr_yaw_back_to_yaw=g("thr_yaw_back_to_yaw");
        p_.yaw_ok_required_count=gi("yaw_ok_required_count");
        p_.x_phase_min_hold_time=g("x_phase_min_hold_time");
        p_.yaw_brake_band=g("yaw_brake_band"); p_.yaw_brake_max_w=g("yaw_brake_max_w");
        p_.yaw_settle_time=g("yaw_settle_time");
        p_.yaw_settle_required_count=gi("yaw_settle_required_count");
        p_.both_visible_required=gi("both_visible_required");

        p_.marker_timeout=g("marker_timeout"); p_.lpf_alpha=g("lpf_alpha");
        p_.outlier_thr=g("outlier_thr");
        p_.sign_yaw=g("sign_yaw"); p_.sign_x=g("sign_x");
        p_.sign_dist=g("sign_dist"); p_.sign_diff=g("sign_diff");
        p_.steer_tol=g("steer_position_tolerance");

        // 추가 파라미터
        p_.wheel_align_tol   = g("wheel_align_tol");
        p_.wheel_stable_count = gi("wheel_stable_count");
        p_.crab_move_dist    = g("crab_move_dist");
        p_.crab_vel          = g("crab_vel");
        p_.crab_left_sign    = g("crab_left_sign");

        p_.wheel_radius    = g("wheel_radius");
        p_.wheel_x_offset  = g("wheel_x_offset");
        p_.wheel_y_offset  = g("wheel_y_offset");
        p_.deadzone_linear = g("deadzone_linear");
        p_.deadzone_angular= g("deadzone_angular");

        // Kinematics 초기화
        ::Params kp;
        kp.wheel_radius    = p_.wheel_radius;
        kp.wheel_x_offset  = p_.wheel_x_offset;
        kp.wheel_y_offset  = p_.wheel_y_offset;
        kp.deadzone_linear = p_.deadzone_linear;
        kp.deadzone_angular= p_.deadzone_angular;
        kinematics_.SetParams(kp);

        LoadMarkerSequences();
    }

    void LoadMarkerSequences()
    {
        marker_sequences_.clear(); sequence_index_=0;
        try {
            auto lv = get_parameter("marker_sequence_left").as_integer_array();
            auto rv = get_parameter("marker_sequence_right").as_integer_array();
            auto load_dv = [this](const std::string & name, std::vector<double> & out,
                                  size_t n, double dfl) {
                try { out = get_parameter(name).as_double_array(); }
                catch (...) {
                    out.assign(n, dfl);
                    RCLCPP_WARN(get_logger(), "[ArucoAligner] %s 없음 → %.4f", name.c_str(), dfl);
                }
                while (out.size() < n) out.push_back(dfl);
            };
            size_t n = std::min(lv.size(), rv.size());
            std::vector<double> rx, ryl, ryr, rad, rdif;
            load_dv("rack_ref_x",           rx,   n, p_.ref_x);
            load_dv("rack_ref_yaw_left",    ryl,  n, p_.ref_yaw_left);
            load_dv("rack_ref_yaw_right",   ryr,  n, p_.ref_yaw_right);
            load_dv("rack_target_avg_dist", rad,  n, p_.target_avg_dist);
            load_dv("rack_target_z_diff",   rdif, n, p_.target_z_diff);
            for (size_t i = 0; i < n; ++i) {
                MarkerSet s;
                s.ids={int(lv[i]),int(rv[i])};
                s.ref_x=rx[i]; s.ref_yaw_left=ryl[i]; s.ref_yaw_right=ryr[i];
                s.target_avg_dist=rad[i]; s.target_z_diff=rdif[i];
                marker_sequences_.push_back(s);
            }
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
            RCLCPP_WARN(get_logger(), "[ArucoAligner] marker_sequence 없음 → 기본 ID");
        }
        if (marker_sequences_.empty()) {
            MarkerSet s;
            s.ids={p_.marker_id_left,p_.marker_id_right};
            s.ref_x=p_.ref_x; s.ref_yaw_left=p_.ref_yaw_left;
            s.ref_yaw_right=p_.ref_yaw_right;
            s.target_avg_dist=p_.target_avg_dist; s.target_z_diff=p_.target_z_diff;
            marker_sequences_.push_back(s);
        }
    }

    void ApplyCurrentSequence()
    {
        if (marker_sequences_.empty()) return;
        const auto & s = marker_sequences_[sequence_index_];
        if (s.ids.size() >= 2) {
            p_.marker_id_left=s.ids[0]; p_.marker_id_right=s.ids[1];
        }
        p_.ref_x=s.ref_x; p_.ref_yaw_left=s.ref_yaw_left;
        p_.ref_yaw_right=s.ref_yaw_right;
        p_.target_avg_dist=s.target_avg_dist; p_.target_z_diff=s.target_z_diff;
        RCLCPP_INFO(get_logger(),
            "[ArucoAligner] 랙 파라미터 적용: ref_x=%.4f avg_dist=%.4f",
            p_.ref_x, p_.target_avg_dist);
    }

    void LogCurrentSequence()
    {
        const auto & s = marker_sequences_[sequence_index_];
        std::string id_str;
        for (size_t i = 0; i < s.ids.size(); ++i) {
            if (i) id_str += ", ";
            id_str += std::to_string(s.ids[i]);
        }
        RCLCPP_INFO(get_logger(), "[ArucoAligner] 시퀀스 [%zu/%zu] IDs: [%s]",
            sequence_index_+1, marker_sequences_.size(), id_str.c_str());
    }

    // ════════════════════════════════════════════════════════
    //  기존 함수들 (원본 그대로)
    // ════════════════════════════════════════════════════════
    bool IsFresh(const rclcpp::Time & t) {
        return t.nanoseconds() > 0 &&
               (get_clock()->now() - t).seconds() <= p_.marker_timeout;
    }

    void UpdateFilter(MF & f, rclcpp::Time & t, int id, double nx, double nz, double nyaw)
    {
        if (std::fabs(nyaw) > kYawFlipThreshold) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "[ArucoAligner] yaw 이상값 차단: %.3f rad", nyaw);
            return;
        }
        if (!f.valid) {
            f = {true, id, nx, nz, nyaw};
        } else {
            if (std::fabs(nx - f.x) > p_.outlier_thr) { t=get_clock()->now(); return; }
            f.id=id;
            f.x   = LpfLinear(f.x, nx, p_.lpf_alpha);
            f.z   = LpfLinear(f.z, nz, p_.lpf_alpha);
            f.yaw = NormalizeAngle(f.yaw + p_.lpf_alpha * NormalizeAngle(nyaw - f.yaw));
        }
        t = get_clock()->now();
    }

    void PublishPause(bool v) {
        std_msgs::msg::Bool m; m.data=v; pause_pub_->publish(m);
    }

    void MarkerCb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg, Side side)
    {
        if (msg->marker_ids.empty() || state_ == State::DONE) return;
        // crab 이동 중엔 마커 무시 (바코드 모드라 어차피 안 옴)
        if (state_ == State::CRAB_MOVE || state_ == State::CRAB_RTN) return;

        for (size_t i = 0; i < msg->marker_ids.size(); ++i) {
            const int id = (int)msg->marker_ids[i];
            const auto & pose = msg->poses[i];
            const double ny = QuatToYaw(pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w);
            if (side == Side::LEFT  && id == p_.marker_id_left)
                UpdateFilter(l1_, l1_t_, id, pose.position.x, pose.position.z, ny);
            if (side == Side::RIGHT && id == p_.marker_id_right)
                UpdateFilter(r1_, r1_t_, id, pose.position.x, pose.position.z, ny);
        }
        TryDetect();
    }

    void TryDetect()
    {
        if (state_ != State::WAITING) return;
        if (!(l1_.valid && IsFresh(l1_t_)) && !(r1_.valid && IsFresh(r1_t_))) return;
        state_ = State::DETECTED;
        std_msgs::msg::Bool det; det.data=true; detected_pub_->publish(det);
        RCLCPP_INFO(get_logger(), "[ArucoAligner] 마커 감지 → /aruco_start 대기 중...");
    }

    void StartAlign()
    {
        if (state_ != State::DETECTED) {
            RCLCPP_WARN(get_logger(),
                "[ArucoAligner] /aruco_start but state=%d", (int)state_);
            return;
        }
        PublishPause(true);
        state_               = State::ALIGNING;
        phase_               = Phase::YAW;
        yaw_ok_count_        = 0;
        yaw_locked_          = false;
        yaw_settle_ok_count_ = 0;
        yaw_fine_ok_count_   = 0;
        both_visible_count_  = 0;
        was_both_visible_    = false;
        wheel_stable_cnt_    = 0;
        x_phase_enter_t_     = rclcpp::Time(0,0,RCL_ROS_TIME);
        yaw_settle_t_        = rclcpp::Time(0,0,RCL_ROS_TIME);
        align_start_t_       = get_clock()->now();
        ++trial_count_;
        RCLCPP_INFO(get_logger(), "[ArucoAligner] 정렬 시작");
    }

    // ── DXL 핸드셰이크 ─────────────────────────────────────────
    void OnP1Done()
    {
        if (state_ != State::DONE) {
            p1_done_pending_ = true;
            RCLCPP_WARN(get_logger(), "[ArucoAligner] p1_done pending (state=%d)", (int)state_);
            return;
        }
        StartCrabMove();
    }

    void OnP2bDone()
    {
        RCLCPP_INFO(get_logger(), "[ArucoAligner] p2b_done → crab 복귀");
        StartCrabReturn();
    }

    void StartCrabMove()
    {
        p1_done_pending_  = false;
        state_            = State::CRAB_MOVE;
        crab_elapsed_     = 0.0;
        crab_target_time_ = (p_.crab_vel > 0.0) ? p_.crab_move_dist / p_.crab_vel : 0.0;
        crab_dir_sign_    = p_.crab_left_sign;
        RCLCPP_INFO(get_logger(),
            "[ArucoAligner] crab 왼쪽 | %.3fm @ %.3fm/s = %.2f초 (sign=%.1f)",
            p_.crab_move_dist, p_.crab_vel, crab_target_time_, crab_dir_sign_);
    }

    void StartCrabReturn()
    {
        state_            = State::CRAB_RTN;
        crab_elapsed_     = 0.0;
        crab_target_time_ = (p_.crab_vel > 0.0) ? p_.crab_move_dist / p_.crab_vel : 0.0;
        crab_dir_sign_    = -p_.crab_left_sign;
        RCLCPP_INFO(get_logger(),
            "[ArucoAligner] crab 복귀 | %.3fm @ %.3fm/s = %.2f초 (sign=%.1f)",
            p_.crab_move_dist, p_.crab_vel, crab_target_time_, crab_dir_sign_);
    }

    Errors ComputeErrors()
    {
        Errors e;
        bool lf = l1_.valid && IsFresh(l1_t_);
        bool rf = r1_.valid && IsFresh(r1_t_);
        if (!lf && !rf) return e;

        e.ok=true; e.both_visible=lf&&rf;
        e.left_visible=lf; e.right_visible=rf;

        if (lf && rf) {
            double eyl = NormalizeAngle(l1_.yaw - p_.ref_yaw_left);
            double eyr = NormalizeAngle(r1_.yaw - p_.ref_yaw_right);
            e.e_yaw=0.5*(eyl+eyr); e.e_yaw_l=eyl; e.e_yaw_r=eyr;
            e.e_x    = 0.5*((l1_.x-p_.ref_x)+(-(r1_.x-p_.ref_x)));
            e.e_z_avg = 0.5*(l1_.z+r1_.z)-p_.target_avg_dist;
            e.e_z_diff = e.e_z_diff_l = (l1_.z-r1_.z)-p_.target_z_diff;
        } else if (lf) {
            e.e_yaw=NormalizeAngle(l1_.yaw-p_.ref_yaw_left);
            e.e_yaw_l=e.e_yaw; e.e_yaw_r=e.e_yaw;
            e.e_x=l1_.x-p_.ref_x; e.e_z_avg=l1_.z-p_.target_avg_dist;
            e.e_z_diff=e.e_z_diff_l=0.0;
            RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"[ArucoAligner] 왼쪽 마커만 보임");
        } else {
            e.e_yaw=NormalizeAngle(r1_.yaw-p_.ref_yaw_right);
            e.e_yaw_l=e.e_yaw; e.e_yaw_r=e.e_yaw;
            e.e_x=-(r1_.x-p_.ref_x); e.e_z_avg=r1_.z-p_.target_avg_dist;
            e.e_z_diff=e.e_z_diff_l=0.0;
            RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"[ArucoAligner] 오른쪽 마커만 보임");
        }
        return e;
    }

    bool IsYawEnterOk(const Errors & e) { return e.ok && std::fabs(e.e_yaw)<p_.thr_yaw_enter_x; }
    bool IsYawOk(const Errors & e)      { return e.ok && std::fabs(e.e_yaw)<p_.thr_yaw; }
    bool IsYawBad(const Errors & e) {
        return std::fabs(e.e_yaw_l)>p_.thr_yaw_back_to_yaw ||
               std::fabs(e.e_yaw_r)>p_.thr_yaw_back_to_yaw;
    }

    double CalcYawSpeed(double yaw_err)
    {
        double spd = p_.sign_yaw * p_.kp_w * yaw_err;
        if (std::fabs(yaw_err) < p_.yaw_brake_band)
            spd = ApplyMinAbs(Clamp(spd,-p_.yaw_brake_max_w,p_.yaw_brake_max_w),p_.min_w);
        else
            spd = ApplyMinAbs(Clamp(spd,-p_.max_w,p_.max_w),p_.min_w);
        return spd;
    }

    // ════════════════════════════════════════════════════════
    //  메인 제어 루프
    // ════════════════════════════════════════════════════════
    void ControlLoop()
    {
        const auto now = get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) dt = 0.02;
        last_time_ = now;

        // crab 이동은 별도 처리
        if (state_ == State::CRAB_MOVE) { RunCrabTimed(dt, false); return; }
        if (state_ == State::CRAB_RTN)  { RunCrabTimed(dt, true);  return; }

        if (state_ != State::ALIGNING) return;

        // WHEEL_ALIGN 페이즈
        if (phase_ == Phase::WHEEL_ALIGN) { RunWheelAlign(); return; }

        // ── 기존 정렬 로직 (원본 그대로) ──────────────────────
        const Errors e = ComputeErrors();

        if (!e.ok) {
            if (phase_ == Phase::YAW && !yaw_lost_recovery_) {
                if (std::fabs(last_e_yaw_) > p_.thr_yaw) {
                    double spd = CalcYawSpeed(last_e_yaw_);
                    last_e_yaw_ -= std::copysign(std::fabs(spd)*0.02, last_e_yaw_);
                    last_yaw_spd_ = spd;
                    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),500,
                        "[ArucoAligner] 마커 없음(YAW중) → last_e_yaw=%.4f", last_e_yaw_);
                    PublishFixed(kSpinSteerAngles, kSpinInwheelSign, spd);
                } else {
                    RCLCPP_WARN(get_logger(), "[ArucoAligner] 마커 없음 → X 전환");
                    yaw_lost_recovery_=true; marker_was_lost_=true;
                    phase_=Phase::X; last_yaw_spd_=0.0;
                }
            } else if (phase_ == Phase::X) {
                double spd = ApplyMinAbs(
                    Clamp(p_.sign_x*p_.min_vx,-p_.max_vx,p_.max_vx),p_.min_vx);
                if (std::fabs(last_e_x_) > p_.thr_center)
                    spd = std::copysign(spd, p_.sign_x*last_e_x_);
                RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),500,
                    "[ArucoAligner] 마커 없음(X중) last_e_x=%.3f spd=%.3f", last_e_x_, spd);
                PublishFixed(kForwardSteerAngles, kSameInwheelSign, spd);
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"[ArucoAligner] 마커 없음 → 정지");
                PublishStop();
            }
            both_visible_count_ = 0;
            return;
        }

        last_e_x_   = e.e_x;
        last_e_yaw_ = e.e_yaw;

        if (yaw_lost_recovery_) { yaw_lost_recovery_=false; last_yaw_spd_=0.0; }
        if (marker_was_lost_ && phase_==Phase::X) {
            RCLCPP_INFO(get_logger(), "[ArucoAligner] 마커 재인식 → YAW 재시작");
            phase_=Phase::YAW; yaw_ok_count_=0; yaw_settle_ok_count_=0;
            yaw_locked_=false; marker_was_lost_=false;
        }

        const bool yaw_enter = IsYawEnterOk(e);
        const bool yaw_ok    = IsYawOk(e);
        const bool center_ok = std::fabs(e.e_x)     < p_.thr_center;
        const bool dist_ok   = std::fabs(e.e_z_avg) < p_.thr_dist;
        const bool diff_ok   = !e.both_visible || (std::fabs(e.e_z_diff) < p_.thr_diff);

        if (phase_ == Phase::SEARCH) {
            if (e.both_visible) {
                RCLCPP_INFO(get_logger(), "[ArucoAligner] SEARCH → 양쪽 감지! YAW 재시작");
                phase_=Phase::YAW; yaw_ok_count_=0; yaw_locked_=false;
                yaw_settle_ok_count_=0; yaw_fine_ok_count_=0; both_visible_count_=0;
                x_phase_enter_t_=yaw_settle_t_=rclcpp::Time(0,0,RCL_ROS_TIME);
            } else {
                double spd = e.left_visible ? p_.min_vy : -p_.min_vy;
                PublishFixed(kCrabSteerAngles, kSameInwheelSign, spd);
                RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
                    "[ArucoAligner] SEARCH | %s | spd=%.3f",
                    e.left_visible?"왼쪽":"오른쪽", spd);
                return;
            }
        }

        if (phase_ == Phase::YAW) {
            yaw_ok_count_ = yaw_enter ? yaw_ok_count_+1 : 0;
            if (yaw_ok_count_ >= p_.yaw_ok_required_count) {
                phase_=Phase::YAW_SETTLE; yaw_ok_count_=0; yaw_settle_ok_count_=0;
                yaw_settle_t_=get_clock()->now();
                PublishStop();
                RCLCPP_INFO(get_logger(),"[ArucoAligner] YAW → YAW_SETTLE (%s)",
                    e.both_visible?"2개":"1개");
            }
        } else if (phase_ == Phase::YAW_SETTLE) {
            PublishStop();
            if (IsYawBad(e)) {
                phase_=Phase::YAW; yaw_locked_=false; yaw_ok_count_=0;
                RCLCPP_WARN(get_logger(),"[ArucoAligner] YSET 중 yaw 틀어짐 → YAW");
            } else {
                bool settle_done = yaw_settle_t_.nanoseconds()>0 &&
                    (get_clock()->now()-yaw_settle_t_).seconds()>=p_.yaw_settle_time;
                if (settle_done) {
                    yaw_settle_ok_count_ = yaw_enter ? yaw_settle_ok_count_+1 : 0;
                    if (yaw_settle_ok_count_ >= p_.yaw_settle_required_count) {
                        phase_=Phase::X; yaw_locked_=true;
                        x_phase_enter_t_=get_clock()->now();
                        RCLCPP_INFO(get_logger(),"[ArucoAligner] YAW_SETTLE → X");
                    }
                }
            }
        } else if (phase_==Phase::X && center_ok) {
            phase_=Phase::Z;
            RCLCPP_INFO(get_logger(),"[ArucoAligner] X → Z (center_ok, yaw=%.4f)",e.e_yaw);
        } else if (phase_==Phase::Z && dist_ok && diff_ok && e.both_visible) {
            phase_=Phase::YAW_FINE; yaw_fine_ok_count_=0;
            PublishStop();
            RCLCPP_INFO(get_logger(),"[ArucoAligner] Z → YAW_FINE (dist=%.4f diff=%.4f yaw=%.4f)",
                e.e_z_avg,e.e_z_diff,e.e_yaw);
            return;
        } else if (phase_==Phase::YAW_FINE) {
            yaw_fine_ok_count_ = yaw_ok ? yaw_fine_ok_count_+1 : 0;
            if (yaw_fine_ok_count_ >= p_.yaw_ok_required_count) {
                // YAW_FINE 완료 → WHEEL_ALIGN 진입
                PublishStop();
                phase_ = Phase::WHEEL_ALIGN;
                wheel_stable_cnt_ = 0;
                RCLCPP_INFO(get_logger(),"[ArucoAligner] YAW_FINE 완료 → WHEEL_ALIGN");
                return;
            }
        }

        // both_visible 카운터 안정화 (원본 그대로)
        if (e.both_visible) both_visible_count_++;
        else { both_visible_count_=0; was_both_visible_=false; }

        if (both_visible_count_>=p_.both_visible_required && !was_both_visible_ &&
            (phase_==Phase::YAW||phase_==Phase::YAW_SETTLE||
             phase_==Phase::X  ||phase_==Phase::Z)) {
            RCLCPP_INFO(get_logger(),
                "[ArucoAligner] 마커 2개 안정 감지(%d프레임)! → SEARCH",both_visible_count_);
            was_both_visible_=true; both_visible_count_=0;
            phase_=Phase::SEARCH; PublishStop(); return;
        }
        if (both_visible_count_>=p_.both_visible_required) was_both_visible_=true;

        if (phase_==Phase::Z && dist_ok && !e.both_visible) {
            RCLCPP_INFO(get_logger(),"[ArucoAligner] Z dist OK but 1개만 → SEARCH");
            phase_=Phase::SEARCH; PublishStop(); return;
        }

        const char * ps = (phase_==Phase::YAW)       ? "YAW"
                        : (phase_==Phase::YAW_SETTLE) ? "YSET"
                        : (phase_==Phase::X)          ? "X"
                        : (phase_==Phase::Z)          ? "Z"
                        : (phase_==Phase::YAW_FINE)   ? "YFIN"
                        : "SRCH";
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),250,
            "[ALIGN|%s|%s] eYL=%+.4f eYR=%+.4f eYavg=%+.4f(%s) eX=%+.4f(%s) "
            "eZavg=%+.4f(%s) eZdiff=%+.4f(%s) bv=%d",
            ps, e.both_visible?"BOTH":"ONE",
            e.e_yaw_l,e.e_yaw_r,e.e_yaw,yaw_ok?"OK":"--",
            e.e_x,center_ok?"OK":"--",
            e.e_z_avg,dist_ok?"OK":"--",
            e.e_z_diff,diff_ok?"OK":"--",
            both_visible_count_);

        switch (phase_) {
        case Phase::YAW:
        case Phase::YAW_FINE: {
            double spd = CalcYawSpeed(e.e_yaw);
            last_yaw_spd_=spd;
            PublishFixed(kSpinSteerAngles, kSpinInwheelSign, spd);
            break;
        }
        case Phase::YAW_SETTLE: PublishStop(); break;
        case Phase::X: {
            double spd = ApplyMinAbs(
                Clamp(p_.sign_x*p_.kp_x*e.e_x,-p_.max_vx,p_.max_vx),p_.min_vx);
            PublishFixed(kForwardSteerAngles, kSameInwheelSign, spd);
            break;
        }
        case Phase::Z: {
            double spd = 0.0;
            if (!dist_ok) {
                if (e.left_visible && !e.both_visible)
                    spd += -p_.sign_dist*p_.kp_y*e.e_z_avg;
                else
                    spd += p_.sign_dist*p_.kp_y*e.e_z_avg;
            }
            if (!diff_ok && e.both_visible) spd += p_.sign_diff*p_.kp_diff*e.e_z_diff;
            if (std::fabs(spd)>1e-6)
                spd = ApplyMinAbs(Clamp(spd,-p_.max_vy,p_.max_vy),p_.min_vy);
            // IK 사용: linear_y 기반으로 바퀴 각도 자동 계산
            // (앞/뒤 좌우 바퀴가 반대 방향으로 계산되어 몸체 안정)
            PublishIK(0.0, spd, 0.0);
            break;
        }
        case Phase::SEARCH: break;
        case Phase::WHEEL_ALIGN: break;
        }
    }

    // ════════════════════════════════════════════════════════
    //  WHEEL_ALIGN: 바퀴 90도 수렴 대기 (추가)
    // ════════════════════════════════════════════════════════
    void RunWheelAlign()
    {
        // 스티어 90도 명령, 인휠 0
        PublishFixed(kCrabSteerAngles, kSameInwheelSign, 0.0);

        // 바퀴가 IK 계산값에 수렴했는지 확인
        // IK로 linear_y 주면 앞 FL≈+90, FR≈-90, 뒤 RL≈+90, RR≈-90 (또는 반대)
        // steer_fb_ 절댓값이 90도 근처인지만 확인
        bool all_at_90 = true;
        for (int i = 0; i < 4; ++i) {
            if (std::fabs(std::fabs(steer_fb_[i]) - 90.0) > p_.wheel_align_tol) {
                all_at_90 = false; break;
            }
        }

        if (all_at_90) {
            if (++wheel_stable_cnt_ >= p_.wheel_stable_count) {
                wheel_stable_cnt_ = 0;
                PublishStop();
                state_ = State::DONE;
                double elapsed = (get_clock()->now()-align_start_t_).seconds();
                RCLCPP_INFO(get_logger(),
                    "[ArucoAligner] ★ 정렬 완료 ★ trial=%d seq=[%zu/%zu] t=%.3fs → /alignment_done",
                    trial_count_, sequence_index_+1, marker_sequences_.size(), elapsed);
                std_msgs::msg::Bool m; m.data=true; done_pub_->publish(m);
                if (p1_done_pending_) {
                    RCLCPP_INFO(get_logger(), "[ArucoAligner] pending p1_done → crab 시작");
                    StartCrabMove();
                }
            }
        } else {
            wheel_stable_cnt_ = 0;
            RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),300,
                "[ArucoAligner] WHEEL_ALIGN 바퀴 회전 중 [%.1f %.1f %.1f %.1f]",
                steer_fb_[0],steer_fb_[1],steer_fb_[2],steer_fb_[3]);
        }
    }

    // ════════════════════════════════════════════════════════
    //  crab 시간 기반 이동 (추가)
    // ════════════════════════════════════════════════════════
    void RunCrabTimed(double dt, bool is_return)
    {
        crab_elapsed_ += dt;

        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),300,
            "[ArucoAligner] crab %s | %.2f / %.2f초 (%.3fm)",
            is_return?"복귀":"이동", crab_elapsed_, crab_target_time_,
            crab_elapsed_*p_.crab_vel);

        if (crab_elapsed_ >= crab_target_time_) {
            PublishStop();
            RCLCPP_INFO(get_logger(),
                "[ArucoAligner] crab %s 완료 (%.2f초, 약%.3fm) → %s",
                is_return?"복귀":"이동",
                crab_elapsed_, crab_elapsed_*p_.crab_vel,
                is_return?"/crab_return_done":"/crab_done");
            state_ = State::DONE;
            std_msgs::msg::Bool m; m.data=true;
            if (is_return) crab_return_pub_->publish(m);
            else           crab_done_pub_->publish(m);
            return;
        }

        // IK 사용: linear_y 기반으로 바퀴 각도 자동 계산
        PublishIK(0.0, crab_dir_sign_*p_.crab_vel, 0.0);
    }

    // ════════════════════════════════════════════════════════
    //  기존 Publish 함수 (원본 그대로 - steer_tol 체크 유지)
    // ════════════════════════════════════════════════════════
    // IK 기반 publish (Z 페이즈 + crab 이동 전용)
    void PublishIK(double vx, double vy, double wz)
    {
        std::vector<double> steers(steer_fb_.begin(), steer_fb_.end());
        auto cmds = kinematics_.InverseKinematics(vx, vy, wz, steers);
        for (int i = 0; i < 4; ++i) {
            std_msgs::msg::Float32 s, w;
            s.data = (float)cmds[i].steering_ang;
            w.data = (float)cmds[i].wheel_vel;
            steer_pubs_[i]->publish(s);
            inwheel_pubs_[i]->publish(w);
        }
    }

    void PublishFixed(const std::array<double,4> & steers,
                      const std::array<double,4> & signs, double vel)
    {
        for (int i = 0; i < 4; ++i) {
            const double se = std::fabs(steers[i] - steer_fb_[i]);
            const float  iw = (se < p_.steer_tol) ? (float)(vel*signs[i]) : 0.0f;
            std_msgs::msg::Float32 s, w;
            s.data=(float)steers[i]; w.data=iw;
            steer_pubs_[i]->publish(s); inwheel_pubs_[i]->publish(w);
        }
    }

    void PublishStop()
    {
        for (int i = 0; i < 4; ++i) {
            std_msgs::msg::Float32 s, w;
            s.data=(float)steer_fb_[i]; w.data=0.0f;
            steer_pubs_[i]->publish(s); inwheel_pubs_[i]->publish(w);
        }
    }

    void OnAlignDone(const Errors & e)
    {
        PublishStop();
        state_ = State::DONE;
        double elapsed = (get_clock()->now()-align_start_t_).seconds();
        RCLCPP_INFO(get_logger(),
            "[ArucoAligner] ★ 정렬 완료 ★ trial=%d seq=[%zu/%zu] time=%.3fs | "
            "yaw=%.2fdeg ctr=%.1fmm dist=%.1fmm diff=%.1fmm",
            trial_count_, sequence_index_+1, marker_sequences_.size(), elapsed,
            std::fabs(e.e_yaw)*180.0/M_PI,
            std::fabs(e.e_x)*1000.0,
            std::fabs(e.e_z_avg)*1000.0,
            std::fabs(e.e_z_diff)*1000.0);
        std_msgs::msg::Bool dm; dm.data=true; done_pub_->publish(dm);
    }

    void ResetForNextCycle()
    {
        if (marker_sequences_.size() > 1) {
            sequence_index_ = (sequence_index_+1) % marker_sequences_.size();
            ApplyCurrentSequence(); LogCurrentSequence();
        }
        state_=State::WAITING; phase_=Phase::YAW;
        yaw_ok_count_=yaw_settle_ok_count_=yaw_fine_ok_count_=0;
        both_visible_count_=0; yaw_locked_=false; last_yaw_spd_=0.0;
        yaw_lost_recovery_=false; marker_was_lost_=false; was_both_visible_=false;
        last_e_x_=0.0; last_e_yaw_=0.0;
        wheel_stable_cnt_=0; p1_done_pending_=false; crab_elapsed_=0.0;
        l1_=r1_=MF{};
        l1_t_=r1_t_=rclcpp::Time(0,0,RCL_ROS_TIME);
        x_phase_enter_t_=yaw_settle_t_=rclcpp::Time(0,0,RCL_ROS_TIME);
        RCLCPP_INFO(get_logger(),"[ArucoAligner] 다음 사이클 대기 (%zu/%zu)",
            sequence_index_+1, marker_sequences_.size());
    }

    // ════════════════════════════════════════════════════════
    //  멤버 변수
    // ════════════════════════════════════════════════════════
    Params p_;
    Kinematics kinematics_;   // Z 페이즈 + crab IK용
    std::vector<MarkerSet> marker_sequences_;
    size_t sequence_index_ = 0;

    State state_ = State::WAITING;
    Phase phase_ = Phase::YAW;
    int yaw_ok_count_=0, yaw_settle_ok_count_=0, yaw_fine_ok_count_=0;
    int both_visible_count_=0;
    bool yaw_locked_=false;
    int trial_count_=0;

    double last_yaw_spd_=0.0;
    bool   yaw_lost_recovery_=false, marker_was_lost_=false, was_both_visible_=false;
    double last_e_x_=0.0, last_e_yaw_=0.0;

    MF l1_, r1_;
    rclcpp::Time l1_t_{0,0,RCL_ROS_TIME}, r1_t_{0,0,RCL_ROS_TIME};
    rclcpp::Time x_phase_enter_t_{0,0,RCL_ROS_TIME};
    rclcpp::Time yaw_settle_t_   {0,0,RCL_ROS_TIME};
    rclcpp::Time align_start_t_  {0,0,RCL_ROS_TIME};

    // 추가 변수
    int    wheel_stable_cnt_  = 0;
    double crab_elapsed_      = 0.0;
    double crab_target_time_  = 0.0;
    double crab_dir_sign_     = 1.0;
    bool   p1_done_pending_   = false;

    std::array<double,4> steer_fb_{};

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr left_sub_, right_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr steer_front_sub_, steer_rear_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_scan_done_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_aruco_start_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_p1_done_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_p2b_done_;

    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,4> steer_pubs_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,4> inwheel_pubs_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_, pause_pub_, detected_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr crab_done_pub_, crab_return_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<ArucoAlignerNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}