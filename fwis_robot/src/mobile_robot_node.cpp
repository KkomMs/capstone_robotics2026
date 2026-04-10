#include <memory>
#include <string>
#include <utility>
#include <cmath>
#include <algorithm>
#include <stdexcept>

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
constexpr std::array<int, 4> MotorId = {2, 1, 4, 3};
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
        //  Publish: /wheel_cmd, /odom, /joint_states(시각화용), TF: odom -> base_footprint
        //           /motor_N/inwheel, /motor_N/steer
        // [todo] 카메라 관련 토픽 추가하기
        // --- Publisher -------------------------------------
        wheel_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/wheel_cmd", qos);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom", qos);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", qos);  // [todo] 기존에 작성한 토픽 -> 추후에 삭제, 수정 등 하기
        for (int i = 0; i < 4; i++) {
            const std::string prefix = "/motor_" + std::to_string(MotorId[i]);
            motor_inwheel_pubs_[i] = this->create_publisher<std_msgs::msg::Float32>(
                prefix + "/inwheel", qos);
            motor_steer_pubs_[i] = this->create_publisher<std_msgs::msg::Float32>(
                prefix + "/steer", qos);
        }
        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }

        // --- Subscriber -------------------------------------
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", qos,
            std::bind(&MobileRobotNode::HandleCmdVel, this, std::placeholders::_1));

        // 조향 피드백 [degL, degR]
        steer_front_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/steer_fb/front", qos,
            std::bind(&MobileRobotNode::HandleSteerFront, this, std::placeholders::_1));
        steer_rear_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/steer_fb/rear", qos,
            std::bind(&MobileRobotNode::HandleSteerRear, this, std::placeholders::_1));
        
        // 구동 피드백 [?, ?]
        inwheel_front_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/inwheel/front", qos,
            std::bind(&MobileRobotNode::HandleInwheelFront, this, std::placeholders::_1));
        inwheel_rear_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/inwheel/rear", qos,
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
        controller_.SetReference(msg->linear.x, msg->linear.y, msg->angular.z);
    }

    // ==========================================================
    //  (모터로부터) joint_states_feedback 콜백
    //  [todo] 실제 토픽 확인 후 코드 수정
    // ==========================================================
    void HandleJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t k = 0; k < msg->name.size(); k++) {
            // steer joint
            auto st_idx = steer_name_to_idx_.find(msg->name[k]);
            if (st_idx != steer_name_to_idx_.end()) {
                int idx = st_idx->second;
                if (k < msg->position.size()) {
                    current_states_[idx].steering_ang = msg->position[k];   // [deg]
                }
                continue;
            }
            // inwheel joint
            auto in_idx = inwheel_name_to_idx_.find(msg->name[k]);
            if (in_idx != inwheel_name_to_idx_.end()) {
                int idx = in_idx->second;
                if (k < msg->velocity.size()) {
                    current_states_[idx].wheel_vel = msg->velocity[k];      // [m/s]
                }
            }
        }
    }
    // ==========================================================
    //  Steer motor feddback 콜백 [deg]
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
    //  [todo] 추후 실제 토픽으로 수정 Inwheel motor feddback 콜백 [?]
    // ==========================================================
    void HandleInwheelFront(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[InwheelFront] /inwheel/front data size < 2");
            return;
        }
        current_states_[0].wheel_vel = static_cast<double>(msg->data[1]);       // [FL]
        current_states_[1].wheel_vel = static_cast<double>(msg->data[0]);       // [FL]
    }
    void HandleInwheelRear(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[InwheelFront] /inwheel/rear data size < 2");
            return;
        }
        current_states_[2].wheel_vel = static_cast<double>(msg->data[1]);       // [RL]
        current_states_[3].wheel_vel = static_cast<double>(msg->data[0]);       // [RR]
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

        // --- motor cmd 계산 ---------------------------------
        const auto motor_cmds = controller_.Update(current_states_, dt);

        // --- FK 계산 ----------------------------------------
        ///// [임시] 조향 = 피드백 값, 구동 = 명령값
        std::vector<WheelState> fk_states(4);
        for (int i = 0; i < 4; i++) {
            fk_states[i].steering_ang = current_states_[i].steering_ang;
            fk_states[i].wheel_vel = motor_cmds[i].wheel_vel;
        }
        kinematics_.ForwardKinematics(fk_states, dt);
        // kinematics_.ForwardKinematics(current_states_, dt);
        const auto& pose = kinematics_.GetPose();

        // --- Publish ----------------------------------------
        PublishWheelCommands(motor_cmds);
        PublishJointStates(motor_cmds, now);
        PublishOdom(pose, now);
        if (publish_tf_) PublishTF(pose, now);
    }

    // ==========================================================
    //  // [todo] 기존 작성 코드. 추후 수정. 모터 명령 publish
    // ==========================================================
    void TempPublishWheelCommands(const std::vector<Command>& cmds, const rclcpp::Time& stamp)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name.resize(8);
        msg.position.resize(8, 0.0);
        msg.velocity.resize(8, 0.0);
        msg.effort.resize(8, 0.0);

        for (int i = 0; i < 4; i++) {
            msg.name[i] = steer_joint_names_[i];
            msg.position[i] = cmds[i].steering_ang;     // [deg]

            msg.name[i + 4] = inwheel_joint_names_[i];
            msg.velocity[i + 4] = cmds[i].wheel_vel;    // [m/s]
        }
        wheel_cmd_pub_->publish(msg);
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
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   steer_front_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   steer_rear_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   inwheel_front_sub_;     // [todo] 추후 실제 토픽명으로 수정
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   inwheel_rear_sub_;      // [todo] 추후 실제 토픽명으로 수정
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr          wheel_cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr          joint_pub_;  // [todo] 추후 수정
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                      tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                                        timer_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4>     motor_inwheel_pubs_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4>     motor_steer_pubs_;

    // 파라미터
    std::string base_frame_id_;
    std::string odom_frame_id_;
    std::array<std::string, 4> steer_joint_names_;
    std::array<std::string, 4> inwheel_joint_names_;
    std::unordered_map<std::string, int> steer_name_to_idx_;
    std::unordered_map<std::string, int> inwheel_name_to_idx_;
    double      loop_hz_;
    bool        publish_tf_;

    // 제어 객체
    Controller controller_;
    Kinematics kinematics_;

    // 현재 상태
    std::vector<WheelState> current_states_;
    rclcpp::Time last_time_;
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