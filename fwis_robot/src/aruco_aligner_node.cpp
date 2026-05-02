/**
 * aruco_aligner_node.cpp
 *
 * 4WIS 정렬 제어 버전
 *
 * [변경사항]
 * - alignment_mode A, B 제거 → C, D 모드만 지원
 * - CSV 저장 기능 제거
 * - 마커 ID 시퀀스 순환 기능 유지
 *   yaml의 marker_sequence_* 리스트를 사이클마다 순서대로 사용
 *   마지막 세트 이후에는 다시 첫 번째 세트로 순환
 *
 * alignment_mode:
 *   C = 좌/우 스캐너 + 좌/우 마커 1개씩
 *   D = 좌/우 스캐너 + 각 2개씩 총 4마커
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
#include "std_msgs/msg/string.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

namespace {

constexpr std::array<int, 4> kSteerMotorId   = {2, 1, 4, 3};
constexpr std::array<int, 4> kInWheelMotorId = {1, 2, 3, 4};

constexpr std::array<double, 4> kSpinSteerAngles    = {-45.0, 45.0,  45.0, -45.0};
constexpr std::array<double, 4> kForwardSteerAngles = {  0.0,  0.0,   0.0,   0.0};
constexpr std::array<double, 4> kCrabSteerAngles    = { 90.0, 90.0,  90.0,  90.0};

constexpr std::array<double, 4> kSpinInwheelSign = {-1.0, +1.0, -1.0, +1.0};
constexpr std::array<double, 4> kSameInwheelSign = {+1.0, +1.0, +1.0, +1.0};

double Clamp(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

double ApplyMinAbs(double v, double min_abs)
{
    if (v != 0.0 && std::fabs(v) < min_abs)
        return std::copysign(min_abs, v);
    return v;
}

double LpfLinear(double prev, double next, double alpha)
{
    return alpha * next + (1.0 - alpha) * prev;
}

double QuatToYaw(double qx, double qy, double qz, double qw)
{
    return std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));
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

        // alignment_mode 검증 (C, D만 허용)
        if (alignment_mode_ != "C" && alignment_mode_ != "D") {
            RCLCPP_FATAL(this->get_logger(),
                "[ArucoAligner] 지원하지 않는 alignment_mode: %s (C 또는 D만 허용)",
                alignment_mode_.c_str());
            throw std::runtime_error("Unsupported alignment_mode: " + alignment_mode_);
        }

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        for (int i = 0; i < 4; ++i) {
            steer_pubs_[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/motor_" + std::to_string(kSteerMotorId[i]) + "/steer", qos);
            inwheel_pubs_[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/motor_" + std::to_string(kInWheelMotorId[i]) + "/inwheel", qos);
        }
        done_pub_     = this->create_publisher<std_msgs::msg::Bool>("/alignment_done", qos);
        pause_pub_    = this->create_publisher<std_msgs::msg::Bool>("/mobile_robot_pause", qos);
        detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/aruco_detected", qos); // [추가]

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
                if (!msg->data) return;
                if (state_ != State::DONE) return;

                RCLCPP_INFO(this->get_logger(), "[ArucoAligner] scan_done 수신 → 다음 사이클");
                ResetForNextCycle();
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ArucoAlignerNode::ControlLoop, this));

        last_time_ = this->get_clock()->now();

        // 첫 번째 시퀀스 세트 적용
        ApplyCurrentSequence();

        RCLCPP_INFO(this->get_logger(),
            "[ArucoAligner] start | mode=%s | 시퀀스=%zu세트",
            alignment_mode_.c_str(), marker_sequences_.size());
        LogCurrentSequence();
    }

private:
    enum class Side  { LEFT, RIGHT };
    enum class State { WAITING, ALIGNING, DONE };
    enum class Phase { YAW, YAW_SETTLE, X, Z };

    struct MF {
        bool   valid = false;
        int    id    = -1;
        double x = 0.0, z = 0.0, yaw = 0.0;
    };

    struct MarkerSet {
        std::vector<int> ids;
    };

    struct Params {
        double ref_x            = 0.0;
        double ref_yaw_left     = 0.0;
        double ref_yaw_right    = 0.0;
        double target_avg_dist  = 0.0;
        double target_z_diff    = 0.0;

        int marker_id_left  = 3;
        int marker_id_right = 2;

        int    marker_id_d_l1       = 3;
        int    marker_id_d_l2       = 4;
        int    marker_id_d_r1       = 2;
        int    marker_id_d_r2       = 1;
        double ref_yaw_d            = 0.0;
        double ref_x_d_left         = 0.0;
        double ref_x_d_right        = 0.0;
        double target_avg_dist_d    = 0.0;
        double target_z_diff_d_left = 0.0;
        double target_z_diff_d_right= 0.0;

        double kp_w = 3.0, kp_x = 0.2, kp_y = 0.2, kp_diff = 0.2;

        double max_w = 0.10, min_w = 0.05;
        double max_vx = 0.14, min_vx = 0.10;
        double max_vy = 0.14, min_vy = 0.10;

        double thr_yaw = 0.020, thr_center = 0.010;
        double thr_dist = 0.015, thr_diff = 0.015;
        int    required_stable = 3;

        double thr_yaw_enter_x     = 0.015;
        double thr_yaw_back_to_yaw = 0.035;
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
        bool ok = false;
        double e_yaw   = 0.0;
        double e_x     = 0.0;
        double e_z_avg = 0.0;
        double e_z_diff= 0.0;
        double e_yaw_l = 0.0, e_yaw_r = 0.0;
        double e_z_diff_l = 0.0, e_z_diff_r = 0.0;
    };

    // ── 파라미터 로드 ──────────────────────────────────────────
    void LoadParams()
    {
        auto g  = [this](const std::string & n){ return this->get_parameter(n).as_double(); };
        auto gi = [this](const std::string & n){ return this->get_parameter(n).as_int(); };
        auto gs = [this](const std::string & n){ return this->get_parameter(n).as_string(); };

        alignment_mode_ = gs("alignment_mode");

        p_.ref_x           = g("ref_x");
        p_.ref_yaw_left    = g("ref_yaw_left");
        p_.ref_yaw_right   = g("ref_yaw_right");
        p_.target_avg_dist = g("target_avg_dist");
        p_.target_z_diff   = g("target_z_diff");

        p_.marker_id_left  = gi("marker_id_left");
        p_.marker_id_right = gi("marker_id_right");

        p_.marker_id_d_l1        = gi("marker_id_d_l1");
        p_.marker_id_d_l2        = gi("marker_id_d_l2");
        p_.marker_id_d_r1        = gi("marker_id_d_r1");
        p_.marker_id_d_r2        = gi("marker_id_d_r2");
        p_.ref_yaw_d             = g("ref_yaw_d");
        p_.ref_x_d_left          = g("ref_x_d_left");
        p_.ref_x_d_right         = g("ref_x_d_right");
        p_.target_avg_dist_d     = g("target_avg_dist_d");
        p_.target_z_diff_d_left  = g("target_z_diff_d_left");
        p_.target_z_diff_d_right = g("target_z_diff_d_right");

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

    // ── 시퀀스 로드 ───────────────────────────────────────────
    void LoadMarkerSequences()
    {
        marker_sequences_.clear();
        sequence_index_ = 0;

        try {
            if (alignment_mode_ == "D") {
                auto l1 = this->get_parameter("marker_sequence_d_l1").as_integer_array();
                auto l2 = this->get_parameter("marker_sequence_d_l2").as_integer_array();
                auto r1 = this->get_parameter("marker_sequence_d_r1").as_integer_array();
                auto r2 = this->get_parameter("marker_sequence_d_r2").as_integer_array();

                size_t n = std::min({l1.size(), l2.size(), r1.size(), r2.size()});
                for (size_t i = 0; i < n; ++i) {
                    MarkerSet s;
                    s.ids = { static_cast<int>(l1[i]), static_cast<int>(l2[i]),
                              static_cast<int>(r1[i]), static_cast<int>(r2[i]) };
                    marker_sequences_.push_back(s);
                }
            } else {  // C
                auto lv = this->get_parameter("marker_sequence_left").as_integer_array();
                auto rv = this->get_parameter("marker_sequence_right").as_integer_array();

                size_t n = std::min(lv.size(), rv.size());
                for (size_t i = 0; i < n; ++i) {
                    MarkerSet s;
                    s.ids = { static_cast<int>(lv[i]), static_cast<int>(rv[i]) };
                    marker_sequences_.push_back(s);
                }
            }
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
            RCLCPP_WARN(this->get_logger(),
                "[ArucoAligner] marker_sequence 파라미터 없음 → 기본 ID 단일 세트 사용");
        }

        if (marker_sequences_.empty()) {
            MarkerSet s;
            if (alignment_mode_ == "D")
                s.ids = { p_.marker_id_d_l1, p_.marker_id_d_l2,
                          p_.marker_id_d_r1, p_.marker_id_d_r2 };
            else  // C
                s.ids = { p_.marker_id_left, p_.marker_id_right };
            marker_sequences_.push_back(s);
        }
    }

    // ── 현재 시퀀스 세트를 파라미터에 반영 ───────────────────
    void ApplyCurrentSequence()
    {
        if (marker_sequences_.empty()) return;

        const auto & s = marker_sequences_[sequence_index_];

        if (alignment_mode_ == "D" && s.ids.size() >= 4) {
            p_.marker_id_d_l1 = s.ids[0];
            p_.marker_id_d_l2 = s.ids[1];
            p_.marker_id_d_r1 = s.ids[2];
            p_.marker_id_d_r2 = s.ids[3];
        } else if (alignment_mode_ == "C" && s.ids.size() >= 2) {
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
        RCLCPP_INFO(this->get_logger(),
            "[ArucoAligner] 현재 시퀀스 [%zu/%zu] 마커 IDs: [%s]",
            sequence_index_ + 1, marker_sequences_.size(), id_str.c_str());
    }

    // ── 유틸 ──────────────────────────────────────────────────
    bool IsFresh(const rclcpp::Time & t)
    {
        return t.nanoseconds() > 0 &&
               (this->get_clock()->now() - t).seconds() <= p_.marker_timeout;
    }

    void UpdateFilter(MF & f, rclcpp::Time & t, int id,
                      double nx, double nz, double nyaw)
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

    void PublishPause(bool v) {
        std_msgs::msg::Bool m; m.data = v; pause_pub_->publish(m);
    }

    // ── 마커 콜백 ─────────────────────────────────────────────
    void MarkerCb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg, Side side)
    {
        if (msg->marker_ids.empty() || state_ == State::DONE) return;

        // [추가] 마커가 보일 때마다 /aruco_detected=true publish
        // waypoint_script.py 가 이 토픽을 구독해서 nav2 cancel 여부를 판단함
        

        for (size_t i = 0; i < msg->marker_ids.size(); ++i) {
            const int id = static_cast<int>(msg->marker_ids[i]);
            const auto & pose = msg->poses[i];
            const double nx = pose.position.x;
            const double nz = pose.position.z;
            const double ny = QuatToYaw(pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w);

            if (alignment_mode_ == "C") {
                if (side == Side::LEFT  && id == p_.marker_id_left)  UpdateFilter(l1_, l1_t_, id, nx, nz, ny);
                if (side == Side::RIGHT && id == p_.marker_id_right) UpdateFilter(r1_, r1_t_, id, nx, nz, ny);

            } else {  // D
                if (side == Side::LEFT) {
                    if (id == p_.marker_id_d_l1) UpdateFilter(l1_, l1_t_, id, nx, nz, ny);
                    if (id == p_.marker_id_d_l2) UpdateFilter(l2_, l2_t_, id, nx, nz, ny);
                } else {
                    if (id == p_.marker_id_d_r1) UpdateFilter(r1_, r1_t_, id, nx, nz, ny);
                    if (id == p_.marker_id_d_r2) UpdateFilter(r2_, r2_t_, id, nx, nz, ny);
                }
            }
        }

        TryStartAlign();
    }

    // ── 정렬 시작 조건 ─────────────────────────────────────────
    bool ReadyToStart()
    {
        if (alignment_mode_ == "C") return l1_.valid && IsFresh(l1_t_) && r1_.valid && IsFresh(r1_t_);
        return l1_.valid && IsFresh(l1_t_) && l2_.valid && IsFresh(l2_t_) &&
               r1_.valid && IsFresh(r1_t_) && r2_.valid && IsFresh(r2_t_);
    }

    void TryStartAlign()
    {
        if (state_ != State::WAITING || !ReadyToStart()) return;
        std_msgs::msg::Bool det; det.data = true; detected_pub_->publish(det);
        RCLCPP_INFO(this->get_logger(),
            "[ArucoAligner] mode=%s 마커 감지 완료 → 정렬 시작", alignment_mode_.c_str());
        PublishPause(true);
        state_           = State::ALIGNING;
        phase_           = Phase::YAW;
        stable_count_    = 0;
        yaw_ok_count_    = 0;
        yaw_locked_      = false;
        yaw_settle_ok_count_ = 0;
        x_phase_enter_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        yaw_settle_t_    = rclcpp::Time(0, 0, RCL_ROS_TIME);
        align_start_t_   = this->get_clock()->now();
        ++trial_count_;
    }

    // ── 오차 계산 ─────────────────────────────────────────────
    Errors ComputeErrors()
    {
        Errors e;

        if (alignment_mode_ == "C") {
            bool lf = l1_.valid && IsFresh(l1_t_);
            bool rf = r1_.valid && IsFresh(r1_t_);
            if (!lf || !rf) return e;
            e.ok = true;

            double eyl = NormalizeAngle(l1_.yaw - p_.ref_yaw_left);
            double eyr = NormalizeAngle(r1_.yaw - p_.ref_yaw_right);
            e.e_yaw   = 0.5 * (eyl + eyr);
            e.e_yaw_l = eyl; e.e_yaw_r = eyr;
            e.e_x     = 0.5 * ((l1_.x - p_.ref_x) + (-(r1_.x - p_.ref_x)));
            e.e_z_avg = 0.5 * (l1_.z + r1_.z) - p_.target_avg_dist;
            e.e_z_diff   = (l1_.z - r1_.z) - p_.target_z_diff;
            e.e_z_diff_l = e.e_z_diff;

        } else {  // D
            bool ll1 = l1_.valid && IsFresh(l1_t_);
            bool ll2 = l2_.valid && IsFresh(l2_t_);
            bool rr1 = r1_.valid && IsFresh(r1_t_);
            bool rr2 = r2_.valid && IsFresh(r2_t_);
            if (!ll1 || !ll2 || !rr1 || !rr2) return e;
            e.ok = true;

            double eyl1 = NormalizeAngle(l1_.yaw - p_.ref_yaw_d);
            double eyl2 = NormalizeAngle(l2_.yaw - p_.ref_yaw_d);
            double eyr1 = NormalizeAngle(r1_.yaw - p_.ref_yaw_d);
            double eyr2 = NormalizeAngle(r2_.yaw - p_.ref_yaw_d);
            e.e_yaw_l = 0.5 * (eyl1 + eyl2);
            e.e_yaw_r = 0.5 * (eyr1 + eyr2);
            e.e_yaw   = 0.5 * (e.e_yaw_l + e.e_yaw_r);

            double ex_l = (l1_.x + l2_.x) * 0.5 - p_.ref_x_d_left;
            double ex_r = -((r1_.x + r2_.x) * 0.5 - p_.ref_x_d_right);
            e.e_x = 0.5 * (ex_l + ex_r);

            double z_all = (l1_.z + l2_.z + r1_.z + r2_.z) * 0.25;
            e.e_z_avg = z_all - p_.target_avg_dist_d;

            e.e_z_diff_l = (l1_.z - l2_.z) - p_.target_z_diff_d_left;
            e.e_z_diff_r = (r1_.z - r2_.z) - p_.target_z_diff_d_right;
            e.e_z_diff   = 0.5 * (e.e_z_diff_l + e.e_z_diff_r);
        }

        return e;
    }

    // ── 완료 판정 헬퍼 ────────────────────────────────────────
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

    // ── 메인 제어 루프 ────────────────────────────────────────
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
        const bool center_ok = e.ok && std::fabs(e.e_x)     < p_.thr_center;
        const bool dist_ok   = e.ok && std::fabs(e.e_z_avg) < p_.thr_dist;
        const bool diff_ok   = e.ok && std::fabs(e.e_z_diff) < p_.thr_diff;

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

        const bool all_ok = yaw_ok && center_ok && dist_ok && diff_ok;
        stable_count_ = all_ok ? stable_count_ + 1 : 0;

        const char * ps = (phase_==Phase::YAW)?"YAW":(phase_==Phase::YAW_SETTLE)?"YSET":(phase_==Phase::X)?"X":"Z";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250,
            "[ALIGN|%s|%s] eYL=%+.4f eYR=%+.4f(%s) eX=%+.4f(%s) "
            "eZavg=%+.4f(%s) eZdiff=%+.4f(%s) cnt=%d/%d",
            alignment_mode_.c_str(), ps,
            e.e_yaw_l, e.e_yaw_r, yaw_ok?"OK":"--",
            e.e_x,     center_ok?"OK":"--",
            e.e_z_avg, dist_ok?"OK":"--",
            e.e_z_diff,diff_ok?"OK":"--",
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
            if (!dist_ok) spd += p_.sign_dist * p_.kp_y    * e.e_z_avg;
            if (!diff_ok) spd += p_.sign_diff * p_.kp_diff * e.e_z_diff;
            if (std::fabs(spd) > 1e-6)
                spd = ApplyMinAbs(Clamp(spd, -p_.max_vy, p_.max_vy), p_.min_vy);
            PublishFixed(kCrabSteerAngles, kSameInwheelSign, spd);
            break;
        }
        }
    }

    // ── 모터 publish ──────────────────────────────────────────
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

    // ── 정렬 완료 처리 ────────────────────────────────────────
    void OnAlignDone(const Errors & e)
    {
        PublishStop();
        state_ = State::DONE;
        double elapsed = (this->get_clock()->now() - align_start_t_).seconds();

        constexpr double kR2D  = 180.0 / M_PI;
        constexpr double kM2MM = 1000.0;
        double yaw_err_deg   = std::max(std::fabs(e.e_yaw_l), std::fabs(e.e_yaw_r)) * kR2D;
        double center_err_mm = std::fabs(e.e_x)     * kM2MM;
        double dist_err_mm   = std::fabs(e.e_z_avg) * kM2MM;
        bool   success       = IsYawOk(const_cast<Errors&>(e)) &&
                               (std::fabs(e.e_x) < p_.thr_center) &&
                               (std::fabs(e.e_z_avg) < p_.thr_dist);

        RCLCPP_INFO(this->get_logger(),
            "[ArucoAligner] ★ 정렬 완료 ★ mode=%s trial=%d seq=[%zu/%zu] time=%.3fs | "
            "yaw=%.2fdeg ctr=%.1fmm dist=%.1fmm | ok=%d",
            alignment_mode_.c_str(), trial_count_,
            sequence_index_ + 1, marker_sequences_.size(),
            elapsed, yaw_err_deg, center_err_mm, dist_err_mm,
            success ? 1 : 0);

        std_msgs::msg::Bool dm; dm.data = true; done_pub_->publish(dm);
    }

    // ── 리셋: 다음 사이클 + 시퀀스 인덱스 전진 ───────────────
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
        l1_=l2_=r1_=r2_=MF{};
        l1_t_=l2_t_=r1_t_=r2_t_=rclcpp::Time(0,0,RCL_ROS_TIME);
        x_phase_enter_t_=yaw_settle_t_=rclcpp::Time(0,0,RCL_ROS_TIME);

        RCLCPP_INFO(this->get_logger(),
            "[ArucoAligner] 다음 실험 대기 중 (시퀀스 %zu/%zu)",
            sequence_index_ + 1, marker_sequences_.size());
    }

    // ── 멤버 변수 ──────────────────────────────────────────────
    Params p_;
    std::string alignment_mode_{"C"};

    std::vector<MarkerSet> marker_sequences_;
    size_t sequence_index_ = 0;

    State state_ = State::WAITING;
    Phase phase_ = Phase::YAW;
    int stable_count_ = 0, yaw_ok_count_ = 0, yaw_settle_ok_count_ = 0;
    bool yaw_locked_ = false;
    int trial_count_ = 0;

    MF l1_, l2_, r1_, r2_;
    rclcpp::Time l1_t_{0,0,RCL_ROS_TIME}, l2_t_{0,0,RCL_ROS_TIME};
    rclcpp::Time r1_t_{0,0,RCL_ROS_TIME}, r2_t_{0,0,RCL_ROS_TIME};

    rclcpp::Time x_phase_enter_t_{0,0,RCL_ROS_TIME};
    rclcpp::Time yaw_settle_t_   {0,0,RCL_ROS_TIME};
    rclcpp::Time align_start_t_  {0,0,RCL_ROS_TIME};

    std::array<double,4> steer_fb_{};

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr left_sub_, right_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr steer_front_sub_, steer_rear_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_scan_done_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,4> steer_pubs_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,4> inwheel_pubs_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_, pause_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_; // [추가]
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