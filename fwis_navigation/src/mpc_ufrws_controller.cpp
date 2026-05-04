#include <algorithm>
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <limits>
#include <stdexcept>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "fwis_navigation/mpc_ufrws_controller.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace mpc_ufrws_controller
{

// ── 정적 유틸 ─────────────────────────────────────────────────────────────────
double MpcUFRWSController::normalizeAngle(double angle)
{
  while (angle >  M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

// ── NLopt 목적 함수 콜백 ───────────────────────────────────────────────────────
double MpcUFRWSController::nloptObjectiveCb(
  const std::vector<double> & u,
  std::vector<double> & grad,
  void * data)
{
  auto * d = static_cast<NloptCallbackData *>(data);

  // 현재 점에서 비용 계산
  const double cost = d->controller->mpcCost(u, *d->current_state, *d->target_seq);

  // 기울기가 필요할 때만 유한 차분 계산
  if (!grad.empty()) {
    std::vector<double> u_plus = u;           // 한 번만 복사, 이후 원소별 교란
    const double eps = d->eps;

    for (size_t i = 0; i < u.size(); ++i) {
      u_plus[i] += eps;
      grad[i] = (d->controller->mpcCost(u_plus, *d->current_state, *d->target_seq) - cost) / eps;
      u_plus[i] = u[i];                       // 원상 복구
    }
  }

  return cost;
}

// ── Nav2 수명 주기 ────────────────────────────────────────────────────────────
void MpcUFRWSController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_        = parent;
  tf_          = tf;
  plugin_name_ = name;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("MpcUFRWSController: Failed to lock node.");
  }
  logger_ = node->get_logger();
  clock_  = node->get_clock();

  // ── 차량 파라미터 ────────────────────────────────────────────────────────────
  declare_parameter_if_not_declared(node, plugin_name_ + ".wheelbase_front",
    rclcpp::ParameterValue(0.215));
  declare_parameter_if_not_declared(node, plugin_name_ + ".wheelbase_rear",
    rclcpp::ParameterValue(0.215));
  declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_track",
    rclcpp::ParameterValue(0.43));
  declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_radius",
    rclcpp::ParameterValue(0.0695));

  // ── MPC 파라미터 ─────────────────────────────────────────────────────────────
  declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel",
    rclcpp::ParameterValue(0.23));
  declare_parameter_if_not_declared(node, plugin_name_ + ".desired_angular_vel",
    rclcpp::ParameterValue(0.45));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist",
  rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".control_period",
    rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".prediction_horizon",
    rclcpp::ParameterValue(10));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_steer_angle",
    rclcpp::ParameterValue(89.9));  // [deg]

  // ── 주행 모드 파라미터 ────────────────────────────────────────────────────────
  declare_parameter_if_not_declared(node, plugin_name_ + ".reversing_mode",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".point_turning_mode",
    rclcpp::ParameterValue(false));

  // ── 비용 함수 가중치 ─────────────────────────────────────────────────────────
  declare_parameter_if_not_declared(node, plugin_name_ + ".Q_y",
    rclcpp::ParameterValue(200.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".Q_phi",
    rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".R_u",
    rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".R_delta",
    rclcpp::ParameterValue(5.0));

  // ── NLopt 최적화 파라미터 ────────────────────────────────────────────────────
  declare_parameter_if_not_declared(node, plugin_name_ + ".opt_max_eval",
    rclcpp::ParameterValue(100));
  declare_parameter_if_not_declared(node, plugin_name_ + ".opt_ftol_rel",
    rclcpp::ParameterValue(1e-6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".opt_xtol_rel",
    rclcpp::ParameterValue(1e-6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".opt_grad_eps",
    rclcpp::ParameterValue(1e-5));

  // ── 장애물 회피 파라미터 ─────────────────────────────────────────────────────────
  declare_parameter_if_not_declared(node, plugin_name_ + ".Q_obs_critical",
    rclcpp::ParameterValue(1000.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".Q_obs_repulsion",
    rclcpp::ParameterValue(40.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_footprint_collision",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".slowdown_ratio",
    rclcpp::ParameterValue(0.5));

  // ── TF ───────────────────────────────────────────────────────────────────────
  declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance",
    rclcpp::ParameterValue(0.1));

  // ── 파라미터 읽기 ────────────────────────────────────────────────────────────
  node->get_parameter(plugin_name_ + ".wheelbase_front", Lf_);
  node->get_parameter(plugin_name_ + ".wheelbase_rear",  Lr_);
  node->get_parameter(plugin_name_ + ".wheel_track",     W_);
  node->get_parameter(plugin_name_ + ".wheel_radius",    wheel_radius_);
  L_ = Lf_ + Lr_;

  node->get_parameter(plugin_name_ + ".desired_linear_vel",   V_ref_);
  node->get_parameter(plugin_name_ + ".desired_angular_vel",  desired_angular_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist",       lookahead_dist_);
  node->get_parameter(plugin_name_ + ".control_period",       dt_);
  node->get_parameter(plugin_name_ + ".prediction_horizon",   N_);

  double max_steer_deg{89.9};
  node->get_parameter(plugin_name_ + ".max_steer_angle", max_steer_deg);
  max_steer_ = max_steer_deg * M_PI / 180.0;

  node->get_parameter(plugin_name_ + ".reversing_mode", reversing_mode_);
  node->get_parameter(plugin_name_ + ".point_turning_mode", point_turning_mode_);

  if (reversing_mode_ && point_turning_mode_) {
    RCLCPP_WARN(logger_, "Both reversing_mode and point_turning_mode are true. Disabling both for safety. Falling back to default forward mode.");
    reversing_mode_ = false;
    point_turning_mode_ = false;
  }

  node->get_parameter(plugin_name_ + ".Q_y",    Q_y_);
  node->get_parameter(plugin_name_ + ".Q_phi",  Q_phi_);
  node->get_parameter(plugin_name_ + ".R_u",    R_u_);
  node->get_parameter(plugin_name_ + ".R_delta", R_delta_);

  node->get_parameter(plugin_name_ + ".opt_max_eval",  opt_max_eval_);
  node->get_parameter(plugin_name_ + ".opt_ftol_rel",  opt_ftol_rel_);
  node->get_parameter(plugin_name_ + ".opt_xtol_rel",  opt_xtol_rel_);
  node->get_parameter(plugin_name_ + ".opt_grad_eps",  opt_grad_eps_);

  node->get_parameter(plugin_name_ + ".Q_obs_critical",           Q_obs_critical_);
  node->get_parameter(plugin_name_ + ".Q_obs_repulsion",          Q_obs_repulsion_);
  node->get_parameter(plugin_name_ + ".use_footprint_collision",  use_footprint_collision_);
  node->get_parameter(plugin_name_ + ".slowdown_ratio",           slowdown_ratio_);

  double transform_tolerance{0.1};
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  // ── 워밍 스타트 초기화 ───────────────────────────────────────────────────────
  const size_t n_ctrl = static_cast<size_t>(N_) * 2;  // 2N 최적화 변수
  u0_.assign(n_ctrl, 0.0);

  // ── NLopt 옵티마이저 초기화  ──────────────────────────────────────────────────
  nlopt_opt_ = std::make_unique<nlopt::opt>(nlopt::LD_SLSQP, n_ctrl);

  // 박스 제약 설정
  std::vector<double> lb(n_ctrl, -max_steer_);
  std::vector<double> ub(n_ctrl,  max_steer_);
  nlopt_opt_->set_lower_bounds(lb);
  nlopt_opt_->set_upper_bounds(ub);

  // 수렴 기준 설정
  nlopt_opt_->set_ftol_rel(opt_ftol_rel_);
  nlopt_opt_->set_xtol_rel(opt_xtol_rel_);
  nlopt_opt_->set_maxeval(opt_max_eval_);

  // 콜백 데이터 초기화
  nlopt_cb_data_.controller   = this;
  nlopt_cb_data_.eps          = opt_grad_eps_;

  // ── 퍼블리셔 ────────────────────────────────────────────────────────────
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

  RCLCPP_INFO(logger_,
    "[MpcUFRWSController] Configured. "
    "L=%.3f m, W=%.3f m, V_ref=%.2f m/s, N=%d, dt=%.2f s, "
    "max_steer=%.1f deg, optimizer=NLopt::LD_SLSQP, max_eval=%d, "
    "Q_obs_critical=%.1f, Q_obs_repulsion=%.1f",
    L_, W_, V_ref_, N_, dt_, max_steer_deg, opt_max_eval_,
    Q_obs_critical_, Q_obs_repulsion_);

  // ── 타이머 ───────────────────────────────────────────────────────────
  last_cmd_time_ = clock_->now();
}

void MpcUFRWSController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up MpcUFRWSController: %s", plugin_name_.c_str());
  nlopt_opt_.reset();
  global_pub_.reset();
}

void MpcUFRWSController::activate()
{
  RCLCPP_INFO(logger_, "Activating MpcUFRWSController: %s", plugin_name_.c_str());
  global_pub_->on_activate();
}

void MpcUFRWSController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating MpcUFRWSController: %s", plugin_name_.c_str());
  global_pub_->on_deactivate();
}

void MpcUFRWSController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  V_ref_ = percentage ? (V_ref_ * speed_limit / 100.0) : speed_limit;
  RCLCPP_INFO(logger_, "Speed limit updated: V_ref=%.3f m/s", V_ref_);
}

void MpcUFRWSController::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;
  u0_.assign(static_cast<size_t>(N_) * 2, 0.0);
  RCLCPP_INFO(logger_, "Global plan set: %zu poses.", path.poses.size());
}

// ── map -> odom frame 변환 ───────────────────────────────────────────────────
nav_msgs::msg::Path MpcUFRWSController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // costmap 프레임(odom)
  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();

  // map -> odom 변환
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_->lookupTransform(
      costmap_frame, global_plan_.header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    throw nav2_core::PlannerException(
      "Unable to transform global plan to costmap frame: " + std::string(ex.what()));
  }

  // 로봇 위치에서 가장 가까운 경로점 찾기
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;

  double min_dist_sq = std::numeric_limits<double>::max();
  size_t closest_idx = 0;

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    // 경로 포즈를 costmap 프레임으로 변환
    geometry_msgs::msg::PoseStamped tf_pose;
    tf2::doTransform(global_plan_.poses[i], tf_pose, tf_stamped);

    const double dx = tf_pose.pose.position.x - robot_x;
    const double dy = tf_pose.pose.position.y - robot_y;
    const double dist_sq = dx * dx + dy * dy;

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }

  // 이미 지나간 경로 지우기
  if (closest_idx > 0) {
    global_plan_.poses.erase(
      global_plan_.poses.begin(),
      global_plan_.poses.begin() + static_cast<long>(closest_idx));
  }

  // 변환된 경로 생성 (costmap 범위 내)
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_frame;
  transformed_plan.header.stamp = robot_pose.header.stamp;

  const double max_costmap_extent = costmap_->getSizeInMetersX() / 2.0;

  for (const auto & pose : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped tf_pose;
    tf2::doTransform(pose, tf_pose, tf_stamped);
    tf_pose.header.frame_id = costmap_frame;

    // costmap 범위 확인 (로봇 중심에서 너무 먼 포인트 제외)
    const double dx = tf_pose.pose.position.x - robot_x;
    const double dy = tf_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > max_costmap_extent) {
      // 범위 밖이면 중단
      break;
    }

    transformed_plan.poses.push_back(tf_pose);
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException(
      "Resulting transformed plan has 0 poses in costmap bounds");
  }

  return transformed_plan;
}

// ── 메인 제어 루프 ─────────────────────────────────────────────────────────────
geometry_msgs::msg::TwistStamped MpcUFRWSController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("MpcUFRWSController: Global plan is empty.");
  }

  // 함수가 호출될 때마다 시간 갱신
  last_cmd_time_ = clock_->now();

  // costmap 포인터 갱신
  costmap_ = costmap_ros_->getCostmap();

  // ── 1. 현재 상태 추출(odom) ────────────────────────────────────────────────────
  VehicleState current_state;
  current_state.x = pose.pose.position.x;
  current_state.y = pose.pose.position.y;

  const auto & q = pose.pose.orientation;
  current_state.theta = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  
  // ── 2. global_plan을 odom frame으로 변환 ────────────────────────────────
  nav_msgs::msg::Path transformed_plan;
  try {
    transformed_plan = transformGlobalPlan(pose);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "transformGlobalPlan failed: %s. Stopping.", e.what());
    return geometry_msgs::msg::TwistStamped();
  }

  if (transformed_plan.poses.empty()) {
    RCLCPP_WARN(logger_, "Transformed plan is empty. Stopping.");
    return geometry_msgs::msg::TwistStamped();
  }
  
  // ── 3. goal 도달 여부 확인 ───────────────────────────────────────────────
  if (goal_checker) {
    geometry_msgs::msg::PoseStamped goal_pose_odom;
    if (transformPose(tf_, costmap_ros_->getGlobalFrameID(),
          global_plan_.poses.back(), goal_pose_odom, transform_tolerance_))
    {
      if (goal_checker->isGoalReached(pose.pose, goal_pose_odom.pose, velocity))
      {
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
          "Goal reached. Sending zero velocity.");
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.frame_id = pose.header.frame_id;
        stop_cmd.header.stamp    = clock_->now();
        return stop_cmd;
      }
    }
  }

  // ── 4. 후진 모드 또는 제자리 회전 모드 ──────────────────────────────────────
  geometry_msgs::msg::TwistStamped cmd_vel;
  if (orientationModes(pose, cmd_vel)) {
    return cmd_vel;
  }

  // ── 5. N 스텝 참조 궤적 생성 ──────────────────────────────────────────────
  std::vector<VehicleState> target_seq;
  try {
    target_seq = generateReferenceTrajectory(current_state, transformed_plan.poses);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Reference trajectory failed: %s", e.what());
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header = pose.header;
    stop_cmd.header.stamp    = clock_->now();
    return stop_cmd;
  }

  // ── 6. 장애물 회피를 위한 속도 조절 ────────────────────────────────────────────
  // double max_cost_ahead = 0.0;
  // for (const auto & target_state : target_seq) {
  //   double cost = use_footprint_collision_ ? footprintCostmapCost(target_state) : costmapCost(target_state.x, target_state.y);
  //   max_cost_ahead = std::max(max_cost_ahead, cost);
  // }

  // if (max_cost_ahead >= 0.9) {
  //   current_v_ref_ = 0.0;     // 충돌 위험 시 정지
  // } else if (max_cost_ahead > 0.2) {    // 거리에 비례하여 감속
  //   double penalty_ratio = (max_cost_ahead - 0.2) / 0.6;
  //   current_v_ref_ = current_v_ref_ * (1.0 - penalty_ratio * (1.0 - slowdown_ratio_));
  // }

  // ── 7. NLopt 최적화 ────────────────────────────────────────────────────────
  const std::vector<double> optimal_u = optimizeMPC(current_state, target_seq);

  // 첫 번째 스텝의 제어 입력만 실제로 인가
  const double delta_f = optimal_u[0];
  const double delta_r = optimal_u[1];

  // ── 8. 워밍 스타트 갱신 ───────────────────────────────────────────────────
  for (size_t i = 0; i < u0_.size() - 2; ++i) {
    u0_[i] = optimal_u[i + 2];
  }
  u0_[u0_.size() - 2] = 0.0;
  u0_[u0_.size() - 1] = 0.0;

  // ── 9. 최종 cmd_vel 계산 ─────────────────────────────────────────────────
  const WheelAngles steer = computeWheelAngles(delta_f, delta_r);
  const WheelVelocities vel = computeWheelVelocities(delta_f, delta_r);

  double cmd_vx = 0.0;
  double cmd_vy = 0.0;
  double cmd_wz = 0.0;

  wheelToCmdVel(steer, vel, cmd_vx, cmd_vy, cmd_wz);

  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp    = clock_->now();
  cmd_vel.twist.linear.x  = cmd_vx;
  cmd_vel.twist.linear.y  = cmd_vy;
  cmd_vel.twist.angular.z = cmd_wz;

  return cmd_vel;
}

// ── UFRWS 기구학 모델 ────────────────────────────────────────────

VehicleState MpcUFRWSController::ufrwsModel(
  const VehicleState & state,
  double delta_f,
  double delta_r) const
{
  const double avg = (delta_f + delta_r) / 2.0;

  VehicleState next;
  next.x = state.x + current_v_ref_ * std::cos(state.theta + avg) * dt_;
  next.y = state.y + current_v_ref_ * std::sin(state.theta + avg) * dt_;
  next.theta = normalizeAngle(state.theta + (current_v_ref_ * std::cos(avg) / L_) * (delta_f - delta_r) * dt_);

  return next;
}

// ── costmap 비용 ─────────────────────────────────────────────────────────────
double MpcUFRWSController::costmapCost(double wx, double wy) const
{
  if (!costmap_) {
    return 0.0;
  }
  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return 0.0;
  }

  const unsigned char cost = costmap_->getCost(mx, my);

  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return 1.0;   // 253, 254
  }
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return 0.0;
  }

  return static_cast<double>(cost) / 252.0;  // 0.0 ~ 1.0 정규화
}

// ── footprint cost ────────────────────────────────────────────────────────────
double MpcUFRWSController::footprintCostmapCost(const VehicleState & state) const
{
  double max_cost = costmapCost(state.x, state.y);
  if (max_cost >= 1.0) return max_cost;

  // footprint 사각형 코너
  const double cos_th = std::cos(state.theta);
  const double sin_th = std::sin(state.theta);

  const double half_w = W_ / 2.0;
  const std::array<std::pair<double, double>, 4> corners = {{
    { Lf_,  half_w},  // [FL]
    { Lf_, -half_w},  // [FR]
    {-Lr_,  half_w},  // [RL]
    {-Lr_, -half_w},  // [RR]
  }};

  for (const auto & [lx, ly] : corners) {
    const double gx = state.x + cos_th * lx - sin_th * ly;
    const double gy = state.y + sin_th * lx + cos_th * ly;
    max_cost = std::max(max_cost, costmapCost(gx, gy));
  }

  return max_cost;
}

// ── 궤적 충돌 검증 ─────────────────────────────────────────────────────────
bool MpcUFRWSController::isTrajectoryCollisionFree(
  const std::vector<double> & u_seq,
  const VehicleState & current_state) const
{
  if (!costmap_) return true;

  VehicleState state = current_state;
  const int steps = static_cast<int>(u_seq.size() / 2);

  for (int i = 0; i < steps; ++i) {
    const double df = u_seq[static_cast<size_t>(i) * 2];
    const double dr = u_seq[static_cast<size_t>(i) * 2 + 1];
    state = ufrwsModel(state, df, dr);

    const double cost = costmapCost(state.x, state.y);
    if (cost >= 1.0) {  // LETHAL 장애물과 직접 충돌
      return false;
    }
  }
  return true;
}

// ── MPC 비용 함수 ─────────────────────────────────────────────────────────────
double MpcUFRWSController::mpcCost(
  const std::vector<double> & u_seq,
  const VehicleState & current_state,
  const std::vector<VehicleState> & target_seq) const
{
  double cost = 0.0;
  VehicleState state = current_state;

  double prev_df = 0.0;
  double prev_dr = 0.0;

  const int steps = static_cast<int>(target_seq.size());

  for (int i = 0; i < steps; ++i) {
    const double delta_f = u_seq[static_cast<size_t>(i) * 2];
    const double delta_r = u_seq[static_cast<size_t>(i) * 2 + 1];

    state = ufrwsModel(state, delta_f, delta_r);

    const VehicleState & ref = target_seq[static_cast<size_t>(i)];

    // 전역 오차 -> 차체 로컬 좌표계 변환
    const double dx    = ref.x - state.x;
    const double dy    = ref.y - state.y;
    const double e_y   = -std::sin(state.theta) * dx + std::cos(state.theta) * dy;
    const double e_phi = normalizeAngle(ref.theta - state.theta);

    double dynamic_Q_y = Q_y_;
    double max_cost = 0.0;

    if (costmap_) {
      max_cost = use_footprint_collision_ ? footprintCostmapCost(state) : costmapCost(state.x, state.y);

      // 장애물이 인식되면 횡방향 오차 가중치 낮춤
      if (max_cost > 0.2) {
        dynamic_Q_y *= 0.1;
      }
    }

    // 상태 비용
    cost += dynamic_Q_y  * e_y   * e_y;
    cost += Q_phi_ * e_phi * e_phi;

    // 제어량 크기 비용 (조향각이 불필요하게 커지는 것 억제)
    cost += R_u_ * (delta_f * delta_f + delta_r * delta_r);

    // 제어 증분 비용 (급격한 조향 변화 억제)
    cost += R_delta_ * (
      (delta_f - prev_df) * (delta_f - prev_df) +
      (delta_r - prev_dr) * (delta_r - prev_dr));

    prev_df = delta_f;
    prev_dr = delta_r;

    // costmap 기반 장애물 비용
    // if ((Q_obs_critical_ > 0.0 || Q_obs_repulsion_ > 0.0) && costmap_) {
    //   if (max_cost >= 0.8) {
    //     cost += Q_obs_critical_ * max_cost * max_cost;
    //   } else if (max_cost > 0.0) {
    //     cost += Q_obs_repulsion_ * max_cost * max_cost;
    //   }
    // }
  }
  return cost;
}

// ── NLopt 최적화 ──────────────────────────────────────────────────────
std::vector<double> MpcUFRWSController::optimizeMPC(
  const VehicleState & current_state,
  const std::vector<VehicleState> & target_seq)
{
  // 1. 콜백 데이터 갱신
  nlopt_cb_data_.current_state = &current_state;
  nlopt_cb_data_.target_seq   = &target_seq;

  // 2. 목적 함수 등록
  nlopt_opt_->set_min_objective(nloptObjectiveCb, &nlopt_cb_data_);

  // 3. 초기값: 워밍 스타트 + 박스 제약 클리핑
  std::vector<double> u = u0_;
  for (auto & val : u) {
    val = std::max(-max_steer_, std::min(max_steer_, val));
  }

  // 4. NLopt 최적화 실행
  double min_cost = std::numeric_limits<double>::max();

  try {
    nlopt::result result = nlopt_opt_->optimize(u, min_cost);

    // 수렴 결과 로깅
    RCLCPP_DEBUG(logger_,
      "NLopt result: %d (1=success, 3=ftol, 4=xtol, 5=maxeval), cost=%.6f",
      static_cast<int>(result), min_cost);

  } catch (const nlopt::roundoff_limited &) {
    // 수치 정밀도 한계에 도달하면 현재까지의 최적해를 안전하게 사용
    RCLCPP_DEBUG(logger_,
      "NLopt: roundoff_limited, using best solution found (cost=%.6f).", min_cost);

  } catch (const nlopt::forced_stop &) {
    RCLCPP_WARN(logger_, "NLopt: forced_stop triggered.");

  } catch (const std::exception & e) {
    // 복구 불가 예외 -> 워밍 스타트(이전 해)를 폴백으로 반환
    RCLCPP_ERROR(logger_,
      "NLopt exception: %s. Falling back to warm-start solution.", e.what());
    return u0_;
  }

  return u;
}

// ── 참조 궤적 생성 ─────────────────────────────────────────────────────────────
std::vector<VehicleState> MpcUFRWSController::generateReferenceTrajectory(
  const VehicleState & current_state,
  std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  if (poses.empty()) {
    throw std::runtime_error("Global plan is empty in generateReferenceTrajectory.");
  }

  // 로봇으로부터 일정 거리 내에 있는 범위만 탐색
  double max_search_dist = 2.0;
  auto search_upper_bound = poses.begin();
  double integrated_dist = 0.0;
  
  for (auto it = poses.begin(); it != poses.end(); ++it) {
    search_upper_bound = it;
    if (it + 1 != poses.end()) {
      integrated_dist += std::hypot(
        (it + 1)->pose.position.x - it->pose.position.x,
        (it + 1)->pose.position.y - it->pose.position.y);
    }
    if (integrated_dist > max_search_dist) {
      break;
    }
  }

  // 가장 가까운 경로점 탐색
  auto closest_pose_it = poses.begin();
  double min_dist = std::numeric_limits<double>::max();

  for (auto it = poses.begin(); it != search_upper_bound; ++it) {
    const double dx = it->pose.position.x - current_state.x;
    const double dy = it->pose.position.y - current_state.y;
    const double d  = std::hypot(dx, dy);
    
    if (d < min_dist) {
      min_dist = d;
      closest_pose_it = it;
    }
  }

  // 지나온 경로는 잘라내기
  if (closest_pose_it != poses.begin()) {
    poses.erase(poses.begin(), closest_pose_it);
  }

  const size_t n_poses = poses.size();
  size_t closest_idx = 0;

  std::vector<VehicleState> ref_seq;
  ref_seq.reserve(static_cast<size_t>(N_));
  const double step_dist = lookahead_dist_ / static_cast<double>(N_);

  for (int i = 0; i < N_; ++i) {
    const double target_dist = step_dist * static_cast<double>(i + 1);
    double accumulated = 0.0;
    size_t sel_idx     = closest_idx;

    for (size_t k = closest_idx; k + 1 < n_poses; ++k) {
      const double seg_dx = poses[k + 1].pose.position.x - poses[k].pose.position.x;
      const double seg_dy = poses[k + 1].pose.position.y - poses[k].pose.position.y;
      accumulated += std::hypot(seg_dx, seg_dy);
      sel_idx = k + 1;
      if (accumulated >= target_dist) {
        break;
      }
    }

    VehicleState ref;
    ref.x = poses[sel_idx].pose.position.x;
    ref.y = poses[sel_idx].pose.position.y;

    // ref theta 계산 (경로의 접선 방향)
    if (sel_idx + 1 < n_poses) {
      const double dx = poses[sel_idx + 1].pose.position.x - poses[sel_idx].pose.position.x;
      const double dy = poses[sel_idx + 1].pose.position.y - poses[sel_idx].pose.position.y;
      ref.theta = std::atan2(dy, dx);
    } else if (sel_idx > 0) {
      const double dx = poses[sel_idx].pose.position.x - poses[sel_idx - 1].pose.position.x;
      const double dy = poses[sel_idx].pose.position.y - poses[sel_idx - 1].pose.position.y;
      ref.theta = std::atan2(dy, dx);
    } else {
      const auto & ori = poses[sel_idx].pose.orientation;
      ref.theta = std::atan2(
        2.0 * (ori.w * ori.z + ori.x * ori.y),
        1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z));
    }

    if (is_reversing_) {
      ref.theta = normalizeAngle(ref.theta + M_PI);
    }

    // crab walking 테스트하려면 아래 주석 해제하고 ref.theta = 0 으로 지정 
    // (heading angle 항상 정면으로 고정)
    // ref.theta = 0;

    ref_seq.push_back(ref);
  }

  return ref_seq;
}

// ── 경로 추종 모드 설정 ────────────────────────────────────────────────
bool MpcUFRWSController::orientationModes(
  const geometry_msgs::msg::PoseStamped & pose,
  geometry_msgs::msg::TwistStamped & cmd_vel)
{
  current_v_ref_ = V_ref_;
  is_reversing_ = false;

  if (global_plan_.poses.size() < 2) return false;

  geometry_msgs::msg::PoseStamped robot_in_map;
  if (!transformPose(tf_, global_plan_.header.frame_id, pose,
        robot_in_map, transform_tolerance_)) {
    return false;
  }

  const double rx = robot_in_map.pose.position.x;
  const double ry = robot_in_map.pose.position.y;
  const auto & rq = robot_in_map.pose.orientation;
  const double robot_yaw = std::atan2(
    2.0 * (rq.w * rq.z + rq.x * rq.y),
    1.0 - 2.0 * (rq.y * rq.y + rq.z * rq.z));

  size_t lookahead_idx = std::min(global_plan_.poses.size() - 1, static_cast<size_t>(10));
  double dx = global_plan_.poses[lookahead_idx].pose.position.x - rx;
  double dy = global_plan_.poses[lookahead_idx].pose.position.y - ry;
  double path_angle = std::atan2(dy, dx);

  // 로봇의 현재 heading과 경로 방향의 오차 계산
  double angle_diff = normalizeAngle(path_angle - robot_yaw);

  // 1. 제자리 회전 모드
  if (point_turning_mode_) {
    if (!is_point_turning_ && std::abs(angle_diff) > M_PI_4) {  // 45deg 이상 오차
      is_point_turning_ = true;
      RCLCPP_INFO(logger_, "Heading angle error: %.1f deg. Start point turn.", angle_diff * 180.0 / M_PI);
    } else if (is_point_turning_ && std::abs(angle_diff) < 0.08) {  // 약 5도 이내 오차
      is_point_turning_ = false;
      RCLCPP_INFO(logger_, "Align completed.");
    }

    if (is_point_turning_) {
       // 제자리 회전 각속도 (+: CCW, -: CW)
      double target_yaw_rate = (angle_diff > 0) ? desired_angular_vel_ : -desired_angular_vel_;

      // cmd_vel 계산
      cmd_vel.header.frame_id = pose.header.frame_id;
      cmd_vel.header.stamp    = clock_->now();
      cmd_vel.twist.linear.x  = 0.0;
      cmd_vel.twist.linear.y  = 0.0;
      cmd_vel.twist.angular.z = target_yaw_rate;

      return true;
    }
  }

  // 2. 후진 모드
  if (reversing_mode_) {
    if (std::abs(angle_diff) > M_PI_2) {
      is_reversing_ = true;
      current_v_ref_ = -V_ref_;
    }
  }

  return false;
}

// ── 조향각 계산 ─────────────────────────────────────────────────────────────
WheelAngles MpcUFRWSController::computeWheelAngles(double delta_f, double delta_r) const
{
  const double tan_df  = std::tan(delta_f);
  const double tan_dr  = std::tan(delta_r);
  const double denom   = (W_ / (2.0 * L_)) * (tan_df - tan_dr);  // 공통 분모 항

  auto safe_atan = [&](double tan_val, double d) -> double {
      // 분모가 너무 작으면 조향각 0으로 안전 처리
      return (std::abs(d) > 1e-5) ? std::atan(tan_val / d) : 0.0;
    };

  WheelAngles w;
  w.fl = safe_atan(tan_df, 1.0 - denom);  // Front Left
  w.fr = safe_atan(tan_df, 1.0 + denom);  // Front Right
  w.rl = safe_atan(tan_dr, 1.0 - denom);  // Rear Left
  w.rr = safe_atan(tan_dr, 1.0 + denom);  // Rear Right

  // 안전 클리핑
  auto clip = [&](double v) {
      return std::max(-max_steer_, std::min(max_steer_, v));
    };
  w.fl = clip(w.fl);
  w.fr = clip(w.fr);
  w.rl = clip(w.rl);
  w.rr = clip(w.rr);

  return w;
}

// ── 4WS 모델의 ICR을 이용한 개별 바퀴 속도 ────────────────────────────────────────
WheelVelocities MpcUFRWSController::computeWheelVelocities(double delta_f, double delta_r) const
{
  const double tan_df = std::tan(delta_f);
  const double tan_dr = std::tan(delta_r);
  const double diff   = tan_df - tan_dr;

  WheelVelocities wv;

  // 직진 또는 완벽한 크랩 주행 (ICR이 무한대인 경우)
  if (std::abs(diff) < 1e-5)
  {
    const double default_rad_speed = current_v_ref_ / wheel_radius_;
    wv.fl = default_rad_speed;
    wv.fr = default_rad_speed;
    wv.rl = default_rad_speed;
    wv.rr = default_rad_speed;
  }
  else
  {
    // 로컬 좌표계 기준 ICR (회전 중심) 좌표 계산
    const double Y_c = L_ / diff;
    const double X_c = (-Lf_ * tan_dr - Lr_ * tan_df) / diff;

    // 로봇 중심(M)에서 ICR까지의 거리
    const double R_M = std::hypot(X_c, Y_c);

    // 각 바퀴의 장착 위치에서 ICR까지의 거리 계산
    const double R_fl = std::hypot(Lf_ - X_c,  W_ / 2.0 - Y_c); // Front-Left
    const double R_fr = std::hypot(Lf_ - X_c, -W_ / 2.0 - Y_c); // Front-Right
    const double R_rl = std::hypot(-Lr_ - X_c, W_ / 2.0 - Y_c); // Rear-Left
    const double R_rr = std::hypot(-Lr_ - X_c, -W_ / 2.0 - Y_c); // Rear-Right

    // 선속도를 각속도[rad/s]로 변환하여 구조체에 저장 (v_i = V_ref * R_i / R_M)
    wv.fl = (current_v_ref_ * (R_fl / R_M)) / wheel_radius_;
    wv.fr = (current_v_ref_ * (R_fr / R_M)) / wheel_radius_;
    wv.rl = (current_v_ref_ * (R_rl / R_M)) / wheel_radius_;
    wv.rr = (current_v_ref_ * (R_rr / R_M)) / wheel_radius_;
  }

  return wv;
}

// ── cmd_vel 계산 ─────────────────────────────────────────────────────────────────
void MpcUFRWSController::wheelToCmdVel(
  const WheelAngles & steer, const WheelVelocities & vel,
  double & vx, double & vy, double & wz) const
{
  const std::array<double, 4> rx = {Lf_, Lf_, -Lr_, -Lr_};
  const std::array<double, 4> ry = {W_/2.0, -W_/2.0, W_/2.0, -W_/2.0};

  const std::array<double, 4> steer_rad = {steer.fl, steer.fr, steer.rl, steer.rr};
  const std::array<double, 4> wheel_vel_rad = {vel.fl, vel.fr, vel.rl, vel.rr};

  double sum_vx = 0.0;
  double sum_vy = 0.0;
  double sum_wz = 0.0;
  const double r2 = Lf_ * Lf_ + (W_/2.0) * (W_/2.0);

  for (int i = 0; i < 4; ++i) {
    double v_linear = wheel_vel_rad[i] * wheel_radius_;   // [m/s]
    double v_xi = v_linear * std::cos(steer_rad[i]);
    double v_yi = v_linear * std::sin(steer_rad[i]);

    sum_vx += v_xi;
    sum_vy += v_yi;
    sum_wz += (v_yi * rx[i] - v_xi * ry[i]) / r2;
  }

  vx = sum_vx / 4.0;
  vy = sum_vy / 4.0;
  wz = sum_wz / 4.0;
}

// ── TF 좌표 변환 ──────────────────────────────────────────────────────────────
bool MpcUFRWSController::transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException &) {
    auto transform = tf->lookupTransform(
      frame, in_pose.header.frame_id, tf2::TimePointZero);

    if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      RCLCPP_ERROR(logger_,
        "Transform data too old: %s -> %s",
        in_pose.header.frame_id.c_str(), frame.c_str());
      return false;
    }

    tf2::doTransform(in_pose, out_pose, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "transformPose exception: %s", ex.what());
    return false;
  }
}

}  // namespace mpc_ufrws_controller

// ── Nav2 플러그인 등록 ─────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  mpc_ufrws_controller::MpcUFRWSController, nav2_core::Controller)
