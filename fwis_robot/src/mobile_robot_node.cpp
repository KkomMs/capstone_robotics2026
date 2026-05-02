#include <memory>
#include <string>
#include <utility>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <numeric>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/bool.hpp"   

#include "fwis_robot/kinematics.hpp"
#include "fwis_robot/controller.hpp"


namespace {
// 필수 파라미터 로드 헬퍼
template<typename T>
T GetParam(const rclcpp::Node& node, const std::string& name)
{
    T value{};
    if (!node.get_parameter(name, value)) {
        throw std::runtime_error(
            "Required parameter '" + name + "' is not set. "
            "Set it via launch/YAML (parameters=[...]).");
    }
    return value;
}

// --- 바퀴 idx ---
// 조향: drive{1,2,3,4}_axis_joint / 인휠: motor{1,2,3,4}_axis_joint
// [FL, FR, RL, RR]
// 조향 좌우 반전, 구동 원래 순서대로
constexpr std::array<int, 4> SteerMotorId = {2, 1, 4, 3};
constexpr std::array<int, 4> InWheelMotorId = {1, 2, 3, 4};
} // namespace

class MobileRobotNode : public rclcpp::Node
{
public:
    explicit MobileRobotNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("mobile_robot_node", options),
        last_time_(this->get_clock()->now())
    {
        current_states_.resize(4, WheelState{0.0, 0.0});
        
        LoadParamsOnlyFromOverrides();
        SetupJointNameMaps();

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        //  Subscribe: /cmd_vel, /steer, /inwheel
        //  Publish:   /odom, /joint_states(시각화용), TF: odom -> base_footprint
        //             /motor_N/inwheel, /motor_N/steer
        // [todo] 카메라 관련 토픽 추가하기
        // --- Publisher -------------------------------------
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom", qos);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", qos);
        for (int i = 0; i < 4; i++) {
            motor_inwheel_pubs_[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/motor_" + std::to_string(InWheelMotorId[i]) + "/inwheel", qos);
            motor_steer_pubs_[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/motor_" + std::to_string(SteerMotorId[i]) + "/steer", qos);
        }
        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
        for (int i = 0; i < 4; i++) {
            filtered_vel_pubs_[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/filtered_vel_" + std::to_string(InWheelMotorId[i]), qos);
        }

        // --- Subscriber -------------------------------------
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", qos,
            std::bind(&MobileRobotNode::HandleCmdVel, this, std::placeholders::_1));

        // [추가] pause: aruco_aligner_node 가 정렬 시작 시 true publish
        pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mobile_robot_pause", qos,
            std::bind(&MobileRobotNode::HandlePause, this, std::placeholders::_1));
 
        // [추가] scan_done: dynamixel_scan_node 가 스캔 완료 시 true publish → pause 해제
        scan_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/scan_done", qos,
            std::bind(&MobileRobotNode::HandleScanDone, this, std::placeholders::_1));

        // 조향 피드백 [degL, degR]
        steer_front_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/steer_fb/front", qos,
            std::bind(&MobileRobotNode::HandleSteerFront, this, std::placeholders::_1));
        steer_rear_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/steer_fb/rear", qos,
            std::bind(&MobileRobotNode::HandleSteerRear, this, std::placeholders::_1));
        
        // 구동 피드백 [m/s, m/s]
        inwheel_front_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_vel/front", qos,
            std::bind(&MobileRobotNode::HandleInwheelFront, this, std::placeholders::_1));
        inwheel_rear_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_vel/rear", qos,
            std::bind(&MobileRobotNode::HandleInwheelRear, this, std::placeholders::_1));

        // --- Timer (50 Hz) -------------------------------------
        const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&MobileRobotNode::RunTimer, this));

        RCLCPP_INFO(this->get_logger(),
            "[MobileRobotNode] Started. base=%s odom=%s loop=%.1f Hz",
            base_frame_id_.c_str(), odom_frame_id_.c_str(), loop_hz_);
    }

private:
    // ==========================================================
    //  파라미터 로드 - YAML
    // ==========================================================
    void LoadParamsOnlyFromOverrides()
    {
        // --- Frames & loop -------------------------------------
        base_frame_id_ = GetParam<std::string>(*this, "base_frame_id");
        odom_frame_id_ = GetParam<std::string>(*this, "odom_frame_id");
        loop_hz_       = GetParam<double>(*this, "control_rate");
        publish_tf_    = GetParam<bool>(*this, "publish_tf");
        deadband_threshold_     = GetParam<double>(*this, "deadband_threshold");

        // --- Joint Names -------------------------------------
        steer_joint_names_[0] = GetParam<std::string>(*this, "front_left_steer_joint");
        steer_joint_names_[1] = GetParam<std::string>(*this, "front_right_steer_joint");
        steer_joint_names_[2] = GetParam<std::string>(*this, "rear_left_steer_joint");
        steer_joint_names_[3] = GetParam<std::string>(*this, "rear_right_steer_joint");

        inwheel_joint_names_[0] = GetParam<std::string>(*this, "front_left_inwheel_joint");
        inwheel_joint_names_[1] = GetParam<std::string>(*this, "front_right_inwheel_joint");
        inwheel_joint_names_[2] = GetParam<std::string>(*this, "rear_left_inwheel_joint");
        inwheel_joint_names_[3] = GetParam<std::string>(*this, "rear_right_inwheel_joint");

        // --- Kinematics -------------------------------------
        {
            Params kin;
            kin.wheel_radius     = GetParam<double>(*this, "wheel_radius");
            kin.wheel_x_offset   = GetParam<double>(*this, "wheel_x_offset");
            kin.wheel_y_offset   = GetParam<double>(*this, "wheel_y_offset");
            kin.deadzone_linear  = GetParam<double>(*this, "deadzone_linear");
            kin.deadzone_angular = GetParam<double>(*this, "deadzone_angular");
            kinematics_.SetParams(kin);
        }

        // --- Controller -------------------------------------
        {
            Controller::Params con;
            con.wheel_radius      = GetParam<double>(*this, "wheel_radius");
            con.wheel_x_offset    = GetParam<double>(*this, "wheel_x_offset");
            con.wheel_y_offset    = GetParam<double>(*this, "wheel_y_offset");
            con.wheel_max_rpm     = GetParam<double>(*this, "wheel_max_rpm");

            con.use_rate_limit       = GetParam<bool>(*this, "use_rate_limit");
            con.bound_cmd_speed      = GetParam<double>(*this, "bound_cmd_speed");
            con.add_cmd_speed        = GetParam<double>(*this, "add_cmd_speed");
            con.bound_cmd_ang_speed  = GetParam<double>(*this, "bound_cmd_ang_speed");
            con.add_cmd_ang_speed    = GetParam<double>(*this, "add_cmd_ang_speed");
            con.max_linear_accel     = GetParam<double>(*this, "max_linear_accel");
            con.max_angular_accel    = GetParam<double>(*this, "max_angular_accel");
            con.deadzone_linear      = GetParam<double>(*this, "deadzone_linear");
            con.deadzone_angular     = GetParam<double>(*this, "deadzone_angular");
            con.lpf_linear           = GetParam<double>(*this, "lpf_linear");
            con.lpf_angular          = GetParam<double>(*this, "lpf_angular");
            con.limit_linear_abs     = GetParam<double>(*this, "limit_linear_abs");
            con.limit_angular_abs    = GetParam<double>(*this, "limit_angular_abs");

            con.steer_max_velocity       = GetParam<double>(*this, "steer_profile.steer_max_velocity");
            con.steer_max_accel          = GetParam<double>(*this, "steer_profile.steer_max_accel");
            con.steer_position_tolerance = GetParam<double>(*this, "steer_profile.steer_position_tolerance");

            con.inwheel_max_accel  = GetParam<double>(*this, "inwheel_profile.inwheel_max_accel");
            con.inwheel_max_decel  = GetParam<double>(*this, "inwheel_profile.inwheel_max_decel");
            con.inwheel_min_rpm    = GetParam<double>(*this, "inwheel_profile.inwheel_min_rpm");

            con.wait_for_steer           = GetParam<bool>(*this, "steer_inwheel_sync.wait_for_steer");
            con.drive_scale_by_steer_err = GetParam<bool>(*this, "steer_inwheel_sync.drive_scale_by_steer_err");
            con.steer_err_full_drive     = GetParam<double>(*this, "steer_inwheel_sync.steer_err_full_drive");
            con.steer_err_zero_drive     = GetParam<double>(*this, "steer_inwheel_sync.steer_err_zero_drive");

            controller_.SetParams(con);
        }
    }

    // ==========================================================
    //  Joint Names -> Motor idx mapping
    // ==========================================================
    void SetupJointNameMaps()
    {
        for (int i = 0; i < 4; i++) {
            steer_name_to_idx_[steer_joint_names_[i]] = i;
            inwheel_name_to_idx_[inwheel_joint_names_[i]] = i;
        }
    }

    // ==========================================================
    //  cmd_vel 콜백
    // ==========================================================
    void HandleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (paused_) return;  // [추가] pause 중에는 cmd_vel 무시
        controller_.SetReference(msg->linear.x, msg->linear.y, msg->angular.z);
    }

    // [추가] pause 콜백 — aruco_aligner_node 가 정렬 시작 시 true
    void HandlePause(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == paused_) return;
        paused_ = msg->data;
        if (paused_) {
            controller_.SetReference(0.0, 0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "[MobileRobotNode] ★ PAUSED - 정렬 중");
        } else {
            RCLCPP_INFO(this->get_logger(), "[MobileRobotNode] ★ RESUMED");
        }
    }
 
    // [추가] scan_done 콜백 — 스캔 완료 후 pause 해제
    void HandleScanDone(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data || !paused_) return;
        paused_    = false;
        stop_sent_ = false;
        RCLCPP_INFO(this->get_logger(),
            "[MobileRobotNode] ★ 스캔 완료 수신 → 주행 재개");
    }

    // ==========================================================
    //  Steer motor feedback 콜백 [deg]
    // ==========================================================
    void HandleSteerFront(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[SteerFront] /steer_fb/front data size < 2");
            return;
        }
        current_states_[0].steering_ang = static_cast<double>(msg->data[1]);    // [FL]
        current_states_[1].steering_ang = static_cast<double>(msg->data[0]);    // [FR]
    }
    void HandleSteerRear(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[SteerFront] /steer_fb/rear data size < 2");
            return;
        }
        current_states_[2].steering_ang = static_cast<double>(msg->data[1]);    // [RL]
        current_states_[3].steering_ang = static_cast<double>(msg->data[0]);    // [RR]
    }
    
    // ==========================================================
    //  Inwheel motor feedback 콜백 [m/s] (Moving Average)
    // ==========================================================
    void HandleInwheelFront(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[InwheelFront] /wheel_vel/front data size < 2");
            return;
        }
        // front 모터 feedback에 Moving average 적용
        for (int i = 0; i < 2; i++) {
            double vel_data = msg->data[i];
            if (std::abs(vel_data) < 0.001) {
                vel_data = 0.0;
            }
            vel_history_[i].push_back(vel_data);
            if (vel_history_[i].size() > window_size_) {
                vel_history_[i].pop_front();        // 가장 오래된 데이터 삭제
            }
            // 평균 계산
            double sum = 0.0;
            for (double v : vel_history_[i]) sum += v;
            filtered_vel_[i] = sum / vel_history_[i].size();

            current_states_[i].wheel_vel = filtered_vel_[i];
        }
    }
    void HandleInwheelRear(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[InwheelFront] /wheel_vel/rear data size < 2");
            return;
        }
        // rear 모터 feedback에 Moving average 적용
        for (int i = 0; i < 2; i++) {
            double vel_data = msg->data[i];
            if (std::abs(vel_data) < 0.001) {
                vel_data = 0.0;
            }
            vel_history_[i+2].push_back(vel_data);
            if (vel_history_[i+2].size() > window_size_) {
                vel_history_[i+2].pop_front();
            }
            // 평균 계산
            double sum = 0.0;
            for (double v : vel_history_[i+2]) sum += v;
            filtered_vel_[i+2] = sum / vel_history_[i+2].size();

            current_states_[i+2].wheel_vel = filtered_vel_[i+2];
        }
    }

    // ==========================================================
    //  Timer (50 Hz)
    // ==========================================================
    void RunTimer()
    {
        const auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) dt = 1.0 / loop_hz_;
        last_time_ = now;

        // [추가] pause 중: 정지 명령 1회 전송 후 odom/TF 유지
        if (paused_) {
            if (!stop_sent_) {
                std::vector<Command> zero(4);
                for (int i = 0; i < 4; i++) {
                    zero[i].steering_ang = current_states_[i].steering_ang;
                    zero[i].wheel_vel    = 0.0;
                }
                PublishWheelCommands(zero);
                stop_sent_ = true;
            }
            const auto& pose = kinematics_.GetPose();
            PublishOdom(pose, now);
            if (publish_tf_) PublishTF(pose, now);
            return;
        }
        stop_sent_ = false;

        // --- motor cmd 계산 ---------------------------------
        const auto motor_cmds = controller_.Update(current_states_, dt);

        // --- FK 계산 ----------------------------------------
        bool use_feedback = true;
        std::vector<WheelState> fk_states(4);

        // 옵션1: 피드백 사용
        if (use_feedback) {
            for (int i = 0; i < 4; i++) {
                fk_states[i].steering_ang = current_states_[i].steering_ang;
                fk_states[i].wheel_vel = current_states_[i].wheel_vel;
                // fk_states[i].wheel_vel = motor_cmds[i].wheel_vel;
            }
        } else {    // 옵션2: 바퀴 명령 사용
            for (int i = 0; i < 4; i++) {
                fk_states[i].steering_ang = motor_cmds[i].steering_ang;
                fk_states[i].wheel_vel = motor_cmds[i].wheel_vel;
            }
        }
        kinematics_.ForwardKinematics(fk_states, dt);
        const auto& pose = kinematics_.GetPose();

        // --- Publish ----------------------------------------
        PublishWheelCommands(motor_cmds);
        PublishJointStates(motor_cmds, now);
        PublishOdom(pose, now);
        if (publish_tf_) PublishTF(pose, now);
        PublishFilteredVel();
    }

    // 임시: 필터링 값 publish
    void PublishFilteredVel()
    {
        for (int i = 0; i < 4; i++) {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(filtered_vel_[i]);
        filtered_vel_pubs_[i]->publish(msg);
    }
    }

    // ==========================================================
    //  개별 모터 명령 publish
    // ==========================================================
    void PublishWheelCommands(const std::vector<Command>& cmds)
    {
        for (int i = 0; i < 4; i++) {
            // steering deg
            std_msgs::msg::Float32 steer_msg;
            steer_msg.data = static_cast<float>(cmds[i].steering_ang);
            motor_steer_pubs_[i]->publish(steer_msg);

            // inwheel vel
            std_msgs::msg::Float32 inwheel_msg;
            inwheel_msg.data = static_cast<float>(cmds[i].wheel_vel);
            motor_inwheel_pubs_[i]->publish(inwheel_msg);
        }

        // [Debug] 조향 모터, 구동 모터 명령값 출력
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "\n"
            "  [motor cmd] steer(deg) / inwheel(m/s)\n"
            "  FL(1): steer=%6.2f  inwheel=%6.3f\n"
            "  FR(2): steer=%6.2f  inwheel=%6.3f\n"
            "  RL(3): steer=%6.2f  inwheel=%6.3f\n"
            "  RR(4): steer=%6.2f  inwheel=%6.3f",
            cmds[0].steering_ang, cmds[0].wheel_vel,
            cmds[1].steering_ang, cmds[1].wheel_vel,
            cmds[2].steering_ang, cmds[2].wheel_vel,
            cmds[3].steering_ang, cmds[3].wheel_vel);
    }

    // ==========================================================
    //  joint states publish (시각화)
    // ==========================================================
    void PublishJointStates(const std::vector<Command>& cmds, const rclcpp::Time& stamp)
    {
        const double r = controller_.GetParams().wheel_radius;

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name.resize(10);
        msg.position.resize(10, 0.0);
        msg.velocity.resize(10, 0.0);

        for (int i = 0; i < 4; i++) {
            msg.name[i] = steer_joint_names_[i];
            msg.position[i] = cmds[i].steering_ang * M_PI / 180.0;  // deg -> rad

            msg.name[i + 4] = inwheel_joint_names_[i];
            msg.velocity[i + 4] = (r > 0.0) ? (cmds[i].wheel_vel / r) : 0.0;    // m/s -> rad/s
        }
        // [todo] 임시. 카메라 joint 0.0으로 채워서 publish
        msg.name[8] = "scanner1_joint";
        msg.name[9] = "scanner2_joint";

        joint_pub_->publish(msg);
    }

    // ==========================================================
    //  odom publish
    // ==========================================================
    void PublishOdom(const Pose& pose, const rclcpp::Time& stamp)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pose.theta);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id  = base_frame_id_;

        odom.pose.pose.position.x  = pose.x;
        odom.pose.pose.position.y  = pose.y;
        odom.pose.pose.position.z  = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 간단 공분산
        odom.pose.covariance.fill(0.0);
        odom.pose.covariance[0]  = 0.05;  // x
        odom.pose.covariance[7]  = 0.05;  // y
        odom.pose.covariance[35] = 0.10;  // yaw

        odom.twist.twist.linear.x  = pose.vx;
        odom.twist.twist.linear.y  = pose.vy;
        odom.twist.twist.angular.z = pose.wz;
        odom.twist.covariance.fill(0.0);
        odom.twist.covariance[0]  = 0.05;
        odom.twist.covariance[7]  = 0.05;
        odom.twist.covariance[35] = 0.10;

        odom_pub_->publish(odom);
    }

    // ==========================================================
    //  TF publish (odom -> base_footprint)
    // ==========================================================
    void PublishTF(const Pose& pose, const rclcpp::Time& stamp)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pose.theta);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp    = stamp;
        tf_msg.header.frame_id = odom_frame_id_;
        tf_msg.child_frame_id  = base_frame_id_;
        tf_msg.transform.translation.x = pose.x;
        tf_msg.transform.translation.y = pose.y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf_msg);
    }

    // ==========================================================
    //  변수
    // ==========================================================
    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr          cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                pause_sub_; 
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                scan_done_sub_; 
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   steer_front_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   steer_rear_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   inwheel_front_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   inwheel_rear_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr          joint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                      tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                                        timer_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4>     motor_inwheel_pubs_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4>     motor_steer_pubs_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4>     filtered_vel_pubs_;

    // 파라미터
    std::string base_frame_id_;
    std::string odom_frame_id_;
    std::array<std::string, 4> steer_joint_names_;
    std::array<std::string, 4> inwheel_joint_names_;
    std::unordered_map<std::string, int> steer_name_to_idx_;
    std::unordered_map<std::string, int> inwheel_name_to_idx_;
    double      loop_hz_;
    bool        publish_tf_;
    double      deadband_threshold_;
    std::array<double, 4> filtered_vel_;
    std::array<std::deque<double>, 4> vel_history_;
    const size_t window_size_ = 5;

    // 제어 객체
    Controller controller_;
    Kinematics kinematics_;

    // 현재 상태
    std::vector<WheelState> current_states_;
    rclcpp::Time last_time_;

    bool paused_    = false;
    bool stop_sent_ = false;
};

// ==========================================================
//  main
// ==========================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<MobileRobotNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}