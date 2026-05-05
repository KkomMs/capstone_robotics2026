/**
 * aruco_aligner_node.cpp
 *
 * 4WIS 정렬 제어 버전 (C모드 전용)
 *
 * [정렬 흐름]
 * 1개만 보임 → YAW → YAW_SETTLE → X → Z (dist만)
 *     → dist 맞췄는데 아직 1개 → SEARCH (보이는 마커 쪽으로 crab)
 *     → 2개 보이면 → YAW부터 재시작
 *     → YAW → YAW_SETTLE → X → Z (2개, z_diff까지) → 완료
 * 완료 판정은 양쪽 마커 보일 때만
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

namespace {

constexpr std::array<int, 4> kSteerMotorId   = {2, 1, 4, 3};
constexpr std::array<int, 4> kInWheelMotorId = {1, 2, 3, 4};

constexpr std::array<double, 4> kSpinSteerAngles    = {-45.0, 45.0,  45.0, -45.0};
constexpr std::array<double, 4> kForwardSteerAngles = {  0.0,  0.0,   0.0,   0.0};
constexpr std::array<double, 4> kCrabSteerAngles    = { 90.0, 90.0,  90.0,  90.0};

constexpr std::array<double, 4> kSpinInwheelSign = {-1.0, +1.0, -1.0, +1.0};
constexpr std::array<double, 4> kSameInwheelSign = {+1.0, +1.0, +1.0, +1.0};

double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

double ApplyMinAbs(double v, double min_abs)
{
    if (v != 0.0 && std::fabs(v) < min_abs) return std::copysign(min_abs, v);
    return v;
}

double LpfLinear(double prev, double next, double alpha) { return alpha * next + (1.0 - alpha) * prev; }

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
        done_pub_     = this->create_publisher<std_msgs::msg::Bool>("/alignment_done", qos);
        pause_pub_    = this->create_publisher<std_msgs::msg::Bool>("/mobile_robot_pause", qos);
        detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/aruco_detected", qos);

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
                RCLCPP_INFO(this->get_logger(), "[ArucoAligner] scan_done 수신 → 다음 사이클");
                ResetForNextCycle();
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ArucoAlignerNode::ControlLoop, this));

        last_time_ = this->get_clock()->now();
        ApplyCurrentSequence();

        RCLCPP_INFO(this->get_logger(),
            "[ArucoAligner] start | 시퀀스=%zu세트", marker_sequences_.size());
        LogCurrentSequence();
    }

private:
    enum class Side  { LEFT, RIGHT };
    enum class State { WAITING, ALIGNING, DONE };
    enum class Phase { YAW, YAW_SETTLE, X, Z, SEARCH };

    struct MF {
        bool   valid = false;
        int    id    = -1;
        double x = 0.0, z = 0.0, yaw = 0.0;
    };

    struct MarkerSet { std::vector<int> ids; };

    struct Params {
        double ref_x           = 0.0;
        double ref_yaw_left    = 0.0;
        double ref_yaw_right   = 0.0;
        double target_avg_dist = 0.0;
        double target_z_diff   = 0.0;
        int    marker_id_left  = 2;
        int    marker_id_right = 1;

        double kp_w = 3.0, kp_x = 0.2, kp_y = 0.2, kp_diff = 0.2;
        double max_w = 0.15, min_w = 0.10;
        double max_vx = 0.15, min_vx = 0.10;
        double max_vy = 0.15, min_vy = 0.10;

        double thr_yaw = 0.018, thr_center = 0.015;
        double thr_dist = 0.015, thr_diff = 0.015;
        int    required_stable = 3;

        double thr_yaw_enter_x       = 0.015;
        double thr_yaw_back_to_yaw   = 0.030;
        int    yaw_ok_required_count = 3;
        double x_phase_min_hold_time = 0.7;
        double yaw_brake_band        = 0.03;
        double yaw_brake_max_w       = 0.05;
        double yaw_settle_time       = 0.20;
        int    yaw_settle_required_count = 2;

        double marker_timeout = 1.5;
        double lpf_alpha      = 0.5;
        double outlier_thr    = 0.5;

        double sign_yaw = -1.0, sign_x = 1.0;
        double sign_dist = -1.0, sign_diff = 1.0;
        double steer_tol = 5.0;
    };

    struct Errors {
        bool   ok           = false;
        bool   both_visible = false;
        bool   left_visible = false;
        bool   right_visible= false;
        double e_yaw = 0.0, e_x = 0.0, e_z_avg = 0.0, e_z_diff = 0.0;
        double e_yaw_l = 0.0, e_yaw_r = 0.0;
        double e_z_diff_l = 0.0;
    };

    // ── 파라미터 로드 ──────────────────────────────────────────
    void LoadParams()
    {
        auto g  = [this](const std::string & n){ return this->get_parameter(n).as_double(); };
        auto gi = [this](const std::string & n){ return (int)this->get_parameter(n).as_int(); };

        p_.ref_x           = g("ref_x");
        p_.ref_yaw_left    = g("ref_yaw_left");
        p_.ref_yaw_right   = g("ref_yaw_right");
        p_.target_avg_dist = g("target_avg_dist");
        p_.target_z_diff   = g("target_z_diff");
        p_.marker_id_left  = gi("marker_id_left");
        p_.marker_id_right = gi("marker_id_right");

        p_.kp_w = g("kp_w"); p_.kp_x = g("kp_x");
        p_.kp_y = g("kp_y"); p_.kp_diff = g("kp_diff");

        p_.max_w = g("max_w"); p_.min_w = g("min_w");
        p_.max_vx = g("max_vx"); p_.min_vx = g("min_vx");
        p_.max_vy = g("max_vy"); p_.min_vy = g("min_vy");

        p_.thr_yaw    = g("thr_yaw");
        p_.thr_center = g("thr_center");
        p_.thr_dist   = g("thr_dist");
        p_.thr_diff   = g("thr_diff");
        p_.required_stable = gi("required_stable");

        p_.thr_yaw_enter_x       = g("thr_yaw_enter_x");
        p_.thr_yaw_back_to_yaw   = g("thr_yaw_back_to_yaw");
        p_.yaw_ok_required_count = gi("yaw_ok_required_count");
        p_.x_phase_min_hold_time = g("x_phase_min_hold_time");
        p_.yaw_brake_band        = g("yaw_brake_band");
        p_.yaw_brake_max_w       = g("yaw_brake_max_w");
        p_.yaw_settle_time       = g("yaw_settle_time");
        p_.yaw_settle_required_count = gi("yaw_settle_required_count");

        p_.marker_timeout = g("marker_timeout");
        p_.lpf_alpha      = g("lpf_alpha");
        p_.outlier_thr    = g("outlier_thr");

        p_.sign_yaw  = g("sign_yaw");
        p_.sign_x    = g("sign_x");
        p_.sign_dist = g("sign_dist");
        p_.sign_diff = g("sign_diff");
        p_.steer_tol = g("steer_position_tolerance");

        LoadMarkerSequences();
    }

    void LoadMarkerSequences()
    {
        marker_sequences_.clear();
        sequence_index_ = 0;
        try {
            auto lv = this->get_parameter("marker_sequence_left").as_integer_array();
            auto rv = this->get_parameter("marker_sequence_right").as_integer_array();
            size_t n = std::min(lv.size(), rv.size());
            for (size_t i = 0; i < n; ++i) {
                MarkerSet s;
                s.ids = { static_cast<int>(lv[i]), static_cast<int>(rv[i]) };
                marker_sequences_.push_back(s);
            }
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
            RCLCPP_WARN(this->get_logger(), "[ArucoAligner] marker_sequence 없음 → 기본 ID 사용");
        }
        if (marker_sequences_.empty()) {
            MarkerSet s;
            s.ids = { p_.marker_id_left, p_.marker_id_right };
            marker_sequences_.push_back(s);
        }
    }

    void ApplyCurrentSequence()
    {
        if (marker_sequences_.empty()) return;
        const auto & s = marker_sequences_[sequence_index_];
        if (s.ids.size() >= 2) {
            p_.marker_id_left  = s.ids[0];
            p_.marker_id_right = s.ids[1];
        }
    }

    void LogCurrentSequence()
    {
        const auto & s = marker_sequences_[sequence_index_];
        std::string id_str;
        for (size_t i = 0; i < s.ids.size(); ++i) {
            if (i > 0) id_str += ", ";
            id_str += std::to_string(s.ids[i]);
        }
        RCLCPP_INFO(this->get_logger(), "[ArucoAligner] 시퀀스 [%zu/%zu] IDs: [%s]",
            sequence_index_ + 1, marker_sequences_.size(), id_str.c_str());
    }

    bool IsFresh(const rclcpp::Time & t)
    {
        return t.nanoseconds() > 0 &&
               (this->get_clock()->now() - t).seconds() <= p_.marker_timeout;
    }

    void UpdateFilter(MF & f, rclcpp::Time & t, int id, double nx, double nz, double nyaw)
    {
        if (!f.valid) {
            f = {true, id, nx, nz, nyaw};
        } else {
            if (std::fabs(nx - f.x) > p_.outlier_thr) { t = this->get_clock()->now(); return; }
            f.id  = id;
            f.x   = LpfLinear(f.x, nx, p_.lpf_alpha);
            f.z   = LpfLinear(f.z, nz, p_.lpf_alpha);
            double dy = NormalizeAngle(nyaw - f.yaw);
            f.yaw = NormalizeAngle(f.yaw + p_.lpf_alpha * dy);
        }
        t = this->get_clock()->now();
    }

    void PublishPause(bool v) { std_msgs::msg::Bool m; m.data = v; pause_pub_->publish(m); }

    void MarkerCb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg, Side side)
    {
        if (msg->marker_ids.empty() || state_ == State::DONE) return;
        for (size_t i = 0; i < msg->marker_ids.size(); ++i) {
            const int id = static_cast<int>(msg->marker_ids[i]);
            const auto & pose = msg->poses[i];
            const double nx = pose.position.x;
            const double nz = pose.position.z;
            const double ny = QuatToYaw(pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w);
            if (side == Side::LEFT  && id == p_.marker_id_left)  UpdateFilter(l1_, l1_t_, id, nx, nz, ny);
            if (side == Side::RIGHT && id == p_.marker_id_right) UpdateFilter(r1_, r1_t_, id, nx, nz, ny);
        }
        TryStartAlign();
    }

    // 하나만 있어도 시작
    bool ReadyToStart()
    {
        return (l1_.valid && IsFresh(l1_t_)) || (r1_.valid && IsFresh(r1_t_));
    }

    void TryStartAlign()
    {
        if (state_ != State::WAITING || !ReadyToStart()) return;
        std_msgs::msg::Bool det; det.data = true; detected_pub_->publish(det);
        RCLCPP_INFO(this->get_logger(), "[ArucoAligner] 마커 감지 → 정렬 시작");
        PublishPause(true);
        state_               = State::ALIGNING;
        phase_               = Phase::YAW;
        stable_count_        = 0;
        yaw_ok_count_        = 0;
        yaw_locked_          = false;
        yaw_settle_ok_count_ = 0;
        x_phase_enter_t_     = rclcpp::Time(0, 0, RCL_ROS_TIME);
        yaw_settle_t_        = rclcpp::Time(0, 0, RCL_ROS_TIME);
        align_start_t_       = this->get_clock()->now();
        ++trial_count_;
    }

    Errors ComputeErrors()
    {
        Errors e;
        bool lf = l1_.valid && IsFresh(l1_t_);
        bool rf = r1_.valid && IsFresh(r1_t_);
        if (!lf && !rf) return e;

        e.ok           = true;
        e.both_visible = lf && rf;
        e.left_visible = lf;
        e.right_visible= rf;

        if (lf && rf) {
            double eyl = NormalizeAngle(l1_.yaw - p_.ref_yaw_left);
            double eyr = NormalizeAngle(r1_.yaw - p_.ref_yaw_right);
            e.e_yaw   = 0.5 * (eyl + eyr);
            e.e_yaw_l = eyl; e.e_yaw_r = eyr;
            e.e_x     = 0.5 * ((l1_.x - p_.ref_x) + (-(r1_.x - p_.ref_x)));
            e.e_z_avg = 0.5 * (l1_.z + r1_.z) - p_.target_avg_dist;
            e.e_z_diff   = (l1_.z - r1_.z) - p_.target_z_diff;
            e.e_z_diff_l = e.e_z_diff;
        } else if (lf) {
            e.e_yaw   = NormalizeAngle(l1_.yaw - p_.ref_yaw_left);
            e.e_yaw_l = e.e_yaw; e.e_yaw_r = e.e_yaw;
            e.e_x     = l1_.x - p_.ref_x;
            e.e_z_avg = l1_.z - p_.target_avg_dist;
            e.e_z_diff = 0.0; e.e_z_diff_l = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[ArucoAligner] 왼쪽 마커만 보임");
        } else {
            e.e_yaw   = NormalizeAngle(r1_.yaw - p_.ref_yaw_right);
            e.e_yaw_l = e.e_yaw; e.e_yaw_r = e.e_yaw;
            e.e_x     = -(r1_.x - p_.ref_x);
            e.e_z_avg = r1_.z - p_.target_avg_dist;
            e.e_z_diff = 0.0; e.e_z_diff_l = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[ArucoAligner] 오른쪽 마커만 보임");
        }
        return e;
    }

    bool IsYawEnterOk(const Errors & e) {
        if (!e.ok) return false;
        return std::fabs(e.e_yaw_l) < p_.thr_yaw_enter_x &&
               std::fabs(e.e_yaw_r) < p_.thr_yaw_enter_x;
    }
    bool IsYawBackBad(const Errors & e) {
        if (!e.ok) return true;
        return std::fabs(e.e_yaw_l) > p_.thr_yaw_back_to_yaw ||
               std::fabs(e.e_yaw_r) > p_.thr_yaw_back_to_yaw;
    }
    bool IsYawOk(const Errors & e) {
        if (!e.ok) return false;
        return std::fabs(e.e_yaw_l) < p_.thr_yaw &&
               std::fabs(e.e_yaw_r) < p_.thr_yaw;
    }

    void ControlLoop()
    {
        const auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) dt = 0.02;
        last_time_ = now;

        if (state_ != State::ALIGNING) return;

        const Errors e = ComputeErrors();
        if (!e.ok) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[ArucoAligner] 마커 없음 → 정지");
            PublishStop(); return;
        }

        const bool yaw_enter = IsYawEnterOk(e);
        const bool yaw_bad   = IsYawBackBad(e);
        const bool yaw_ok    = IsYawOk(e);
        const bool center_ok = std::fabs(e.e_x)     < p_.thr_center;
        const bool dist_ok   = std::fabs(e.e_z_avg) < p_.thr_dist;
        const bool diff_ok   = !e.both_visible || (std::fabs(e.e_z_diff) < p_.thr_diff);

        // ── SEARCH phase ─────────────────────────────────────
        if (phase_ == Phase::SEARCH) {
            if (e.both_visible) {
                RCLCPP_INFO(this->get_logger(),
                    "[ArucoAligner] SEARCH → 양쪽 감지! YAW부터 재시작");
                phase_               = Phase::YAW;
                stable_count_        = 0;
                yaw_ok_count_        = 0;
                yaw_locked_          = false;
                yaw_settle_ok_count_ = 0;
                x_phase_enter_t_     = rclcpp::Time(0, 0, RCL_ROS_TIME);
                yaw_settle_t_        = rclcpp::Time(0, 0, RCL_ROS_TIME);
            } else {
                // 보이는 마커 쪽으로 crab 이동
                double spd = e.left_visible ? p_.min_vy : -p_.min_vy;
                PublishFixed(kCrabSteerAngles, kSameInwheelSign, spd);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[ArucoAligner] SEARCH | %s 마커만 보임 | spd=%.3f",
                    e.left_visible ? "왼쪽" : "오른쪽", spd);
                return;
            }
        }

        // ── 일반 phase 전환 ───────────────────────────────────
        if (phase_ == Phase::YAW) {
            yaw_ok_count_ = yaw_enter ? yaw_ok_count_ + 1 : 0;
            if (yaw_ok_count_ >= p_.yaw_ok_required_count) {
                phase_ = Phase::YAW_SETTLE;
                yaw_ok_count_ = 0; yaw_settle_ok_count_ = 0;
                yaw_settle_t_ = this->get_clock()->now();
                PublishStop();
                RCLCPP_INFO(this->get_logger(), "[ArucoAligner] YAW → YAW_SETTLE");
            }
        } else if (phase_ == Phase::YAW_SETTLE) {
            PublishStop();
            bool settle_done = yaw_settle_t_.nanoseconds() > 0 &&
                (this->get_clock()->now() - yaw_settle_t_).seconds() >= p_.yaw_settle_time;
            if (settle_done) {
                yaw_settle_ok_count_ = yaw_enter ? yaw_settle_ok_count_ + 1 : 0;
                if (yaw_settle_ok_count_ >= p_.yaw_settle_required_count) {
                    phase_ = Phase::X; yaw_locked_ = true;
                    x_phase_enter_t_ = this->get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "[ArucoAligner] YAW_SETTLE → X");
                } else if (yaw_bad) {
                    phase_ = Phase::YAW; yaw_locked_ = false;
                    RCLCPP_WARN(this->get_logger(), "[ArucoAligner] 오버슈트 → YAW");
                }
            }
        } else if (phase_ == Phase::X && yaw_ok && center_ok) {
            phase_ = Phase::Z;
            RCLCPP_INFO(this->get_logger(), "[ArucoAligner] X → Z");
        }

        if      (phase_ == Phase::Z && yaw_bad)    { phase_ = Phase::YAW; yaw_locked_ = false; RCLCPP_WARN(this->get_logger(), "[ArucoAligner] yaw 틀어짐 → YAW"); }
        else if (phase_ == Phase::Z && !center_ok) { phase_ = Phase::X;   RCLCPP_WARN(this->get_logger(), "[ArucoAligner] x 틀어짐 → X"); }
        else if (phase_ == Phase::X && yaw_bad &&
                 x_phase_enter_t_.nanoseconds() > 0 &&
                 (this->get_clock()->now() - x_phase_enter_t_).seconds() > p_.x_phase_min_hold_time)
        {
            phase_ = Phase::YAW; yaw_locked_ = false;
            RCLCPP_WARN(this->get_logger(), "[ArucoAligner] X중 yaw 틀어짐 → YAW");
        }

        // Z phase에서 dist OK인데 1개만 보이면 SEARCH
        if (phase_ == Phase::Z && dist_ok && !e.both_visible) {
            RCLCPP_INFO(this->get_logger(),
                "[ArucoAligner] Z dist OK but 1개만 보임 → SEARCH");
            phase_ = Phase::SEARCH;
            PublishStop();
            return;
        }

        // 완료 판정: 양쪽 마커 보일 때만
        const bool all_ok = yaw_ok && center_ok && dist_ok && diff_ok && e.both_visible;
        stable_count_ = all_ok ? stable_count_ + 1 : 0;

        const char * ps = (phase_==Phase::YAW)    ? "YAW"
                        : (phase_==Phase::YAW_SETTLE) ? "YSET"
                        : (phase_==Phase::X)       ? "X"
                        : (phase_==Phase::Z)       ? "Z"
                        : "SRCH";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250,
            "[ALIGN|%s|%s] eYL=%+.4f eYR=%+.4f(%s) eX=%+.4f(%s) "
            "eZavg=%+.4f(%s) eZdiff=%+.4f(%s) cnt=%d/%d",
            ps, e.both_visible ? "BOTH" : "ONE",
            e.e_yaw_l, e.e_yaw_r, yaw_ok?"OK":"--",
            e.e_x,     center_ok?"OK":"--",
            e.e_z_avg, dist_ok?"OK":"--",
            e.e_z_diff, diff_ok?"OK":"--",
            stable_count_, p_.required_stable);

        if (stable_count_ >= p_.required_stable) { OnAlignDone(e); return; }

        switch (phase_) {
        case Phase::YAW: {
            double yaw_err = (std::fabs(e.e_yaw_l) >= std::fabs(e.e_yaw_r)) ? e.e_yaw_l : e.e_yaw_r;
            double spd = p_.sign_yaw * p_.kp_w * yaw_err;
            if (std::fabs(yaw_err) < p_.yaw_brake_band)
                spd = Clamp(spd, -p_.yaw_brake_max_w, p_.yaw_brake_max_w);
            else
                spd = ApplyMinAbs(Clamp(spd, -p_.max_w, p_.max_w), p_.min_w);
            PublishFixed(kSpinSteerAngles, kSpinInwheelSign, spd);
            break;
        }
        case Phase::YAW_SETTLE: PublishStop(); break;
        case Phase::X: {
            double spd = ApplyMinAbs(Clamp(p_.sign_x * p_.kp_x * e.e_x, -p_.max_vx, p_.max_vx), p_.min_vx);
            PublishFixed(kForwardSteerAngles, kSameInwheelSign, spd);
            break;
        }
        case Phase::Z: {
            double spd = 0.0;
            if (!dist_ok) spd += p_.sign_dist * p_.kp_y * e.e_z_avg;
            if (!diff_ok && e.both_visible) spd += p_.sign_diff * p_.kp_diff * e.e_z_diff;
            if (std::fabs(spd) > 1e-6)
                spd = ApplyMinAbs(Clamp(spd, -p_.max_vy, p_.max_vy), p_.min_vy);
            PublishFixed(kCrabSteerAngles, kSameInwheelSign, spd);
            break;
        }
        case Phase::SEARCH: break;
        }
    }

    void PublishFixed(const std::array<double,4>& steers,
                      const std::array<double,4>& signs, double vel)
    {
        for (int i = 0; i < 4; ++i) {
            const double se = std::fabs(steers[i] - steer_fb_[i]);
            const float  iw = (se < p_.steer_tol) ? static_cast<float>(vel * signs[i]) : 0.0f;
            std_msgs::msg::Float32 s, w;
            s.data = static_cast<float>(steers[i]); w.data = iw;
            steer_pubs_[i]->publish(s); inwheel_pubs_[i]->publish(w);
        }
    }

    void PublishStop() {
        for (int i = 0; i < 4; ++i) {
            std_msgs::msg::Float32 s, w;
            s.data = static_cast<float>(steer_fb_[i]); w.data = 0.0f;
            steer_pubs_[i]->publish(s); inwheel_pubs_[i]->publish(w);
        }
    }

    void OnAlignDone(const Errors & e)
    {
        PublishStop();
        state_ = State::DONE;
        double elapsed = (this->get_clock()->now() - align_start_t_).seconds();
        constexpr double kR2D  = 180.0 / M_PI;
        constexpr double kM2MM = 1000.0;
        RCLCPP_INFO(this->get_logger(),
            "[ArucoAligner] ★ 정렬 완료 ★ trial=%d seq=[%zu/%zu] time=%.3fs | "
            "yaw=%.2fdeg ctr=%.1fmm dist=%.1fmm | ok=%d",
            trial_count_, sequence_index_ + 1, marker_sequences_.size(), elapsed,
            std::max(std::fabs(e.e_yaw_l), std::fabs(e.e_yaw_r)) * kR2D,
            std::fabs(e.e_x) * kM2MM, std::fabs(e.e_z_avg) * kM2MM,
            (IsYawOk(const_cast<Errors&>(e)) &&
             std::fabs(e.e_x) < p_.thr_center &&
             std::fabs(e.e_z_avg) < p_.thr_dist) ? 1 : 0);
        std_msgs::msg::Bool dm; dm.data = true; done_pub_->publish(dm);
    }

    void ResetForNextCycle()
    {
        if (marker_sequences_.size() > 1) {
            sequence_index_ = (sequence_index_ + 1) % marker_sequences_.size();
            ApplyCurrentSequence();
            LogCurrentSequence();
        }
        state_ = State::WAITING; phase_ = Phase::YAW;
        stable_count_ = yaw_ok_count_ = yaw_settle_ok_count_ = 0;
        yaw_locked_ = false;
        l1_ = r1_ = MF{};
        l1_t_ = r1_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        x_phase_enter_t_ = yaw_settle_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        RCLCPP_INFO(this->get_logger(), "[ArucoAligner] 다음 사이클 대기 중 (%zu/%zu)",
            sequence_index_ + 1, marker_sequences_.size());
    }

    Params p_;
    std::vector<MarkerSet> marker_sequences_;
    size_t sequence_index_ = 0;

    State state_ = State::WAITING;
    Phase phase_ = Phase::YAW;
    int stable_count_ = 0, yaw_ok_count_ = 0, yaw_settle_ok_count_ = 0;
    bool yaw_locked_ = false;
    int trial_count_ = 0;

    MF l1_, r1_;
    rclcpp::Time l1_t_{0,0,RCL_ROS_TIME}, r1_t_{0,0,RCL_ROS_TIME};
    rclcpp::Time x_phase_enter_t_{0,0,RCL_ROS_TIME};
    rclcpp::Time yaw_settle_t_   {0,0,RCL_ROS_TIME};
    rclcpp::Time align_start_t_  {0,0,RCL_ROS_TIME};

    std::array<double,4> steer_fb_{};

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr left_sub_, right_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr steer_front_sub_, steer_rear_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_scan_done_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,4> steer_pubs_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,4> inwheel_pubs_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_, pause_pub_, detected_pub_;
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