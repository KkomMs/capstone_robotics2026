#include <algorithm>
#include <string>
#include <memory>
#include <tf2/utils.h>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_simulation/lyapunov_kinematics_controller.hpp"
#include "nav2_util/geometry_utils.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace lyapunov_kinematics_controller
{

template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
    if (begin == end) {
        return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it) {
        auto comp = getCompareVal(*it);
        if (comp < lowest) {
            lowest = comp;
            lowest_it = it;
        }
    }
    return lowest_it;
}

void LyapunovKinematicsController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;

    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // Parameters
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".kx", rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".ky", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".ktheta", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".kx", kx_);
    node->get_parameter(plugin_name_ + ".ky", ky_);
    node->get_parameter(plugin_name_ + ".ktheta", ktheta_);
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void LyapunovKinematicsController::cleanup()
{
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type lyapunov_kinematics_controller::LyapunovKinematicsController",
        plugin_name_.c_str());
    global_pub_.reset();
}

void LyapunovKinematicsController::activate()
{
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type lyapunov_kinematics_controller::LyapunovKinematicsController\"  %s",
        plugin_name_.c_str(), plugin_name_.c_str());
    global_pub_->on_activate();
}

void LyapunovKinematicsController::deactivate()
{
    RCLCPP_INFO(
        logger_,
        "Dectivating controller: %s of type lyapunov_kinematics_controller::LyapunovKinematicsController\"  %s",
        plugin_name_.c_str(), plugin_name_.c_str());
    global_pub_->on_deactivate();
}

void LyapunovKinematicsController::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
    (void)speed_limit;
    (void)percentage;
}

geometry_msgs::msg::TwistStamped LyapunovKinematicsController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker)
{
    (void)velocity;
    (void)goal_checker;

    // 1. 글로벌 경로를 로봇 로컬 좌표계(base_link)로 변환
    // 경로상의 점들은 로봇 중심으로부터의 상대 좌표(xe, ye)가 됨.
    auto transformed_plan = transformGlobalPlan(pose);

    // 2. 추종할 목표점 찾기
    auto goal_pose_it = std::find_if(
        transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto &ps)
        { return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_; });

    // If the last pose is still within lookahed distance, take the last pose
    if (goal_pose_it == transformed_plan.poses.end())
    {
        goal_pose_it = std::prev(transformed_plan.poses.end());
    }
    auto goal_pose = goal_pose_it->pose;

    // 3. 오차 계산
    // 로봇 local frame이므로 경로상의 점들이 에러(xe, ye)가 됨.
    // -> orientation 값 넘겨주는 planner 필요.
    double xe = goal_pose.position.x;
    double ye = goal_pose.position.y;
    double theta_e = tf2::getYaw(goal_pose.orientation);
    
    ////// [예전 코드] ==  헤딩(theta_e) 자동 계산
    // double theta_e = 0.0;
    // auto next_it = std::next(goal_pose_it);
    
    // if (next_it != transformed_plan.poses.end()) {
    //     // 목표점의 다음 점이 존재하면 두 점 사이의 벡터로 방향 계산
    //     double dx = next_it->pose.position.x - goal_pose.position.x;
    //     double dy = next_it->pose.position.y - goal_pose.position.y;
    //     theta_e = std::atan2(dy, dx);
    // } else if (goal_pose_it != transformed_plan.poses.begin()) {
    //     // 목표점이 경로의 끝점이라면, 바로 이전 점과 목표점 사이의 벡터로 방향 계산
    //     auto prev_it = std::prev(goal_pose_it);
    //     double dx = goal_pose.position.x - prev_it->pose.position.x;
    //     double dy = goal_pose.position.y - prev_it->pose.position.y;
    //     theta_e = std::atan2(dy, dx);
    // } else {
    //     // 예외 상황 폴백
    //     theta_e = tf2::getYaw(goal_pose.orientation);
    // }

    // 4. Global Control Law
    double v_A = desired_linear_vel_ * cos(theta_e) + kx_ * xe;
    double omega_R = 0.0;
    double omega_A = (1.0 / ky_) * ye * desired_linear_vel_ + ktheta_ * sin(theta_e) + omega_R;

    // 5. Create and publish a TwistStamped message with the desired velocity
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x = v_A;
    cmd_vel.twist.angular.z = omega_A;

    // (속도 제한 추가 가능)

    return cmd_vel;
}

void LyapunovKinematicsController::setPlan(const nav_msgs::msg::Path &path)
{
    global_pub_->publish(path);
    global_plan_ = path;
}

nav_msgs::msg::Path
LyapunovKinematicsController::transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped &pose)
{
    // Original mplementation taken fron nav2_dwb_controller

    if (global_plan_.poses.empty())
    {
        throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(
            tf_, global_plan_.header.frame_id, pose,
            robot_pose, transform_tolerance_))
    {
        throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
                            costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin =
        min_by(
            global_plan_.poses.begin(), global_plan_.poses.end(),
            [&robot_pose](const geometry_msgs::msg::PoseStamped &ps)
            {
                return euclidean_distance(robot_pose, ps);
            });

    // From the closest point, look for the first point that's further then dist_threshold from the
    // robot. These points are definitely outside of the costmap so we won't transform them.
    auto transformation_end = std::find_if(
        transformation_begin, end(global_plan_.poses),
        [&](const auto &global_plan_pose)
        {
            return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
        });

    // Helper function for the transform below. Transforms a PoseStamped from global frame to local
    auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose)
    {
        // We took a copy of the pose, let's lookup the transform at the current time
        geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = pose.header.stamp;
        stamped_pose.pose = global_plan_pose.pose;
        transformPose(
            tf_, costmap_ros_->getBaseFrameID(),
            stamped_pose, transformed_pose, transform_tolerance_);
        return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty())
    {
        throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
}

bool LyapunovKinematicsController::transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped &in_pose,
    geometry_msgs::msg::PoseStamped &out_pose,
    const rclcpp::Duration &transform_tolerance) const
{
    // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

    if (in_pose.header.frame_id == frame)
    {
        out_pose = in_pose;
        return true;
    }

    try
    {
        tf->transform(in_pose, out_pose, frame);
        return true;
    }
    catch (tf2::ExtrapolationException &ex)
    {
        auto transform = tf->lookupTransform(
            frame,
            in_pose.header.frame_id,
            tf2::TimePointZero);
        if (
            (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
            transform_tolerance)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("tf_help"),
                "Transform data too old when converting from %s to %s",
                in_pose.header.frame_id.c_str(),
                frame.c_str());
            RCLCPP_ERROR(
                rclcpp::get_logger("tf_help"),
                "Data time: %ds %uns, Transform time: %ds %uns",
                in_pose.header.stamp.sec,
                in_pose.header.stamp.nanosec,
                transform.header.stamp.sec,
                transform.header.stamp.nanosec);
            return false;
        }
        else
        {
            tf2::doTransform(in_pose, out_pose, transform);
            return true;
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("tf_help"),
            "Exception in transformPose: %s",
            ex.what());
        return false;
    }
    return false;
}

}   // namespace lyapunov_kinematics_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(lyapunov_kinematics_controller::LyapunovKinematicsController, nav2_core::Controller)