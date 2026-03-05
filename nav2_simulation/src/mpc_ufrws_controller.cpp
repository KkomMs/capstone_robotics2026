/*
 * MPC-UFRWS (Model Predictive Control - Unconstrained Front Rear Wheel Steering)
 * Nav2 Custom Controller Plugin — NLopt 실시간 최적화 버전 구현
 *
 * Based on: "Optimal Control Method of Path Tracking for Four-Wheel Steering Vehicles"
 * Tan, X.; Liu, D.; Xiong, H. Actuators 2022, 11, 61.
 * https://doi.org/10.3390/act11020061
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  최적화 교체 요약                                                │
 * │                                                                  │
 * │  Python (SciPy)          →  C++ (NLopt)                         │
 * │  ─────────────────────────────────────────────────────────────  │
 * │  scipy.optimize.minimize     nlopt::opt (LD_SLSQP)              │
 * │  method='SLSQP'          →  nlopt::LD_SLSQP (동일 알고리즘)    │
 * │  bounds=[(-ms,ms)]*2N    →  set_lower/upper_bounds()            │
 * │  options={'maxiter':20}  →  set_maxeval()                       │
 * │  warm start (u0 shift)   →  u0_ 벡터 rotate + 포인터 재사용    │
 * │                                                                  │
 * │  실시간 성능 최적화:                                             │
 * │  - nlopt::opt 객체를 configure()에서 1회 생성 후 재사용         │
 * │    → 매 제어 주기 동적 할당/해제 비용 제거                      │
 * │  - set_min_objective()로 콜백 데이터 포인터만 교체              │
 * │    → std::vector deepcopy 없음                                   │
 * │  - roundoff_limited 예외 → 현재까지 최적해 사용 (실시간 안전)   │
 * └─────────────────────────────────────────────────────────────────┘
 */

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
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_simulation/mpc_ufrws_controller.hpp"

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

// ── NLopt 목적 함수 콜백 (static) ────────────────────────────────────────────
//
// NLopt은 LD_SLSQP 알고리즘에서 기울기를 요구한다.
// grad 벡터가 비어 있지 않으면 전진 유한 차분으로 수치 기울기를 채운다.
//
// 수치 기울기 비용: mpcCost를 (2N + 1)번 호출
//   → N=10 기준 21번 / NLopt 반복당
//   → SLSQP는 QP 서브문제를 효율적으로 풀기 때문에
//     반복 횟수 자체가 Armijo 경사 하강보다 훨씬 적음
//
double MpcUFRWSController::nloptObjectiveCb(
  const std::vector<double> & u,
  std::vector<double> & grad,
  void * data)
{
  auto * d = static_cast<NloptCallbackData *>(data);

  // 현재 점에서 비용 계산
  const double cost = d->controller->mpcCost(u, *d->current_state, *d->target_seq);

  // 기울기가 필요할 때만 유한 차분 계산 (LN 계열 알고리즘이면 grad가 비어 있음)
  if (!grad.empty()) {
    std::vector<double> u_plus = u;           // 한 번만 복사, 이후 원소별 교란
    const double eps = d->eps;

    for (size_t i = 0; i < u.size(); ++i) {
      u_plus[i] += eps;
      grad[i] = (d->controller->mpcCost(u_plus, *d->current_state, *d->target_seq) - cost) / eps;
      u_plus[i] = u[i];                       // 원상 복구 (재할당 없음)
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
    rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(node, plugin_name_ + ".control_period",
    rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".prediction_horizon",
    rclcpp::ParameterValue(10));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_steer_angle",
    rclcpp::ParameterValue(180.0));  // [deg]

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
  //   opt_max_eval  : 실시간 보장을 위한 함수 평가 횟수 상한
  //                   N=10일 때 함수 1회 평가 = mpcCost 호출 1회 (기울기 포함 시 21회)
  //                   maxeval=100 → SLSQP 약 4~5회 반복 (매우 빠름)
  //   opt_ftol_rel  : 비용 함수 상대 수렴 허용치
  //   opt_xtol_rel  : 변수 상대 수렴 허용치
  //   opt_grad_eps  : 유한 차분 스텝 크기 (너무 작으면 수치 오차, 너무 크면 부정확)
  declare_parameter_if_not_declared(node, plugin_name_ + ".opt_max_eval",
    rclcpp::ParameterValue(100));
  declare_parameter_if_not_declared(node, plugin_name_ + ".opt_ftol_rel",
    rclcpp::ParameterValue(1e-6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".opt_xtol_rel",
    rclcpp::ParameterValue(1e-6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".opt_grad_eps",
    rclcpp::ParameterValue(1e-5));

  // ── TF ───────────────────────────────────────────────────────────────────────
  declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance",
    rclcpp::ParameterValue(0.1));

  // ── 파라미터 읽기 ────────────────────────────────────────────────────────────
  node->get_parameter(plugin_name_ + ".wheelbase_front", Lf_);
  node->get_parameter(plugin_name_ + ".wheelbase_rear",  Lr_);
  node->get_parameter(plugin_name_ + ".wheel_track",     W_);
  node->get_parameter(plugin_name_ + ".wheel_radius",    wheel_radius_);
  L_ = Lf_ + Lr_;

  node->get_parameter(plugin_name_ + ".desired_linear_vel", V_ref_);
  node->get_parameter(plugin_name_ + ".control_period",     dt_);
  node->get_parameter(plugin_name_ + ".prediction_horizon", N_);

  double max_steer_deg{60.0};
  node->get_parameter(plugin_name_ + ".max_steer_angle", max_steer_deg);
  max_steer_ = max_steer_deg * M_PI / 180.0;

  node->get_parameter(plugin_name_ + ".Q_y",    Q_y_);
  node->get_parameter(plugin_name_ + ".Q_phi",  Q_phi_);
  node->get_parameter(plugin_name_ + ".R_u",    R_u_);
  node->get_parameter(plugin_name_ + ".R_delta", R_delta_);

  node->get_parameter(plugin_name_ + ".opt_max_eval",  opt_max_eval_);
  node->get_parameter(plugin_name_ + ".opt_ftol_rel",  opt_ftol_rel_);
  node->get_parameter(plugin_name_ + ".opt_xtol_rel",  opt_xtol_rel_);
  node->get_parameter(plugin_name_ + ".opt_grad_eps",  opt_grad_eps_);

  double transform_tolerance{0.1};
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  // ── 워밍 스타트 초기화 ───────────────────────────────────────────────────────
  const size_t n_ctrl = static_cast<size_t>(N_) * 2;  // 2N 최적화 변수
  u0_.assign(n_ctrl, 0.0);

  // ── NLopt 옵티마이저 초기화 (1회, 이후 재사용) ───────────────────────────────
  //
  // 알고리즘: LD_SLSQP
  //   - Sequential Least-Squares Quadratic Programming
  //   - SciPy method='SLSQP'과 동일 알고리즘
  //   - 박스 제약 + 비선형 목적 함수에 최적
  //   - 기울기 정보를 사용하므로 LN(미분 불필요) 계열보다 수렴이 빠름
  //
  // 대안 알고리즘 (필요 시 교체):
  //   nlopt::LD_LBFGS  → 제약 없는 문제에서 더 빠름 (박스 제약은 투영 처리)
  //   nlopt::LD_MMA    → 복잡한 비선형 제약이 추가될 때 유리
  //
  nlopt_opt_ = std::make_unique<nlopt::opt>(nlopt::LD_SLSQP, n_ctrl);

  // 박스 제약 설정 (configure에서 1회 설정 → 재사용 시 재설정 불필요)
  std::vector<double> lb(n_ctrl, -max_steer_);
  std::vector<double> ub(n_ctrl,  max_steer_);
  nlopt_opt_->set_lower_bounds(lb);
  nlopt_opt_->set_upper_bounds(ub);

  // 수렴 기준 설정
  nlopt_opt_->set_ftol_rel(opt_ftol_rel_);
  nlopt_opt_->set_xtol_rel(opt_xtol_rel_);
  nlopt_opt_->set_maxeval(opt_max_eval_);

  // 콜백 데이터 초기화 (포인터는 optimizeMPC()에서 갱신)
  nlopt_cb_data_.controller   = this;
  nlopt_cb_data_.eps          = opt_grad_eps_;

  // ── 퍼블리셔 생성 ────────────────────────────────────────────────────────────
  pub_steer_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_position_controller/commands", rclcpp::SystemDefaultsQoS());
  pub_drive_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_velocity_controller/commands", rclcpp::SystemDefaultsQoS());
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "received_global_plan", 1);

  RCLCPP_INFO(logger_,
    "[MpcUFRWSController] Configured. "
    "L=%.3f m, W=%.3f m, V_ref=%.2f m/s, N=%d, dt=%.2f s, "
    "max_steer=%.1f deg, optimizer=NLopt::LD_SLSQP, max_eval=%d",
    L_, W_, V_ref_, N_, dt_, max_steer_deg, opt_max_eval_);
}

void MpcUFRWSController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up MpcUFRWSController: %s", plugin_name_.c_str());
  nlopt_opt_.reset();
  global_pub_.reset();
  pub_steer_.reset();
  pub_drive_.reset();
}

void MpcUFRWSController::activate()
{
  RCLCPP_INFO(logger_, "Activating MpcUFRWSController: %s", plugin_name_.c_str());
  global_pub_->on_activate();
  pub_steer_->on_activate();
  pub_drive_->on_activate();
}

void MpcUFRWSController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating MpcUFRWSController: %s", plugin_name_.c_str());
  global_pub_->on_deactivate();
  pub_steer_->on_deactivate();
  pub_drive_->on_deactivate();
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
  u0_.assign(static_cast<size_t>(N_) * 2, 0.0);  // 새 경로 → 워밍 스타트 초기화
  RCLCPP_INFO(logger_, "Global plan set: %zu poses.", path.poses.size());
}

// ── 메인 제어 루프 ─────────────────────────────────────────────────────────────

geometry_msgs::msg::TwistStamped MpcUFRWSController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("MpcUFRWSController: Global plan is empty.");
  }

  // ── 1. 현재 상태 추출 ─────────────────────────────────────────────────────
  VehicleState current_state;
  current_state.x = pose.pose.position.x;
  current_state.y = pose.pose.position.y;

  const auto & q = pose.pose.orientation;
  current_state.theta = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // ── 2. N 스텝 참조 궤적 생성 ──────────────────────────────────────────────
  std::vector<VehicleState> target_seq;
  try {
    target_seq = generateReferenceTrajectory(current_state);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Reference trajectory failed: %s", e.what());
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header = pose.header;
    return stop_cmd;
  }

  // ── 3. NLopt 최적화 ────────────────────────────────────────────────────────
  const std::vector<double> optimal_u = optimizeMPC(current_state, target_seq);

  // Receding Horizon: 첫 번째 스텝의 제어 입력만 실제로 인가
  const double delta_f = optimal_u[0];
  const double delta_r = optimal_u[1];

  // ── 4. 워밍 스타트 갱신 ───────────────────────────────────────────────────
  // 최적 시퀀스를 한 스텝 앞으로 시프트 (Python의 np.roll + 0 패딩과 동일)
  // 동시에 NLopt 결과로 u0_ 갱신 (이미 optimal_u로 업데이트됨)
  for (size_t i = 0; i < u0_.size() - 2; ++i) {
    u0_[i] = optimal_u[i + 2];
  }
  u0_[u0_.size() - 2] = 0.0;
  u0_[u0_.size() - 1] = 0.0;

  // ── 5. Ackermann 기하학: 개별 바퀴 조향각 계산 (논문 Eq.1) ──────────────
  const WheelAngles wheels = computeWheelAngles(delta_f, delta_r);

  // ── 6. 바퀴 명령 발행 ────────────────────────────────────────────────────
  publishWheelCommands(wheels, V_ref_);

  // ── 7. Nav2 표준 TwistStamped 반환 ───────────────────────────────────────
  // 등가 yaw rate: ω = V·cos((δf+δr)/2) / L · (δf - δr)  (논문 Eq.6에서 유도)
  const double avg_steer = (delta_f + delta_r) / 2.0;
  const double yaw_rate  = V_ref_ * std::cos(avg_steer) / L_ * (delta_f - delta_r);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp    = clock_->now();
  cmd_vel.twist.linear.x  = V_ref_;
  cmd_vel.twist.angular.z = yaw_rate;

  return cmd_vel;
}

// ── UFRWS 기구학 모델 (논문 Eq.6) ────────────────────────────────────────────

VehicleState MpcUFRWSController::ufrwsModel(
  const VehicleState & state,
  double delta_f,
  double delta_r) const
{
  const double avg = (delta_f + delta_r) / 2.0;

  VehicleState next;
  next.x     = state.x     + V_ref_ * std::cos(state.theta + avg) * dt_;
  next.y     = state.y     + V_ref_ * std::sin(state.theta + avg) * dt_;
  next.theta = normalizeAngle(
    state.theta + (V_ref_ * std::cos(avg) / L_) * (delta_f - delta_r) * dt_);

  return next;
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

    // 전역 오차 → 차체 로컬 좌표계 변환
    const double dx    = ref.x - state.x;
    const double dy    = ref.y - state.y;
    const double e_y   = -std::sin(state.theta) * dx + std::cos(state.theta) * dy;
    const double e_phi = normalizeAngle(ref.theta - state.theta);

    // 상태 비용
    cost += Q_y_  * e_y   * e_y;
    cost += Q_phi_ * e_phi * e_phi;

    // 제어량 크기 비용 (조향각이 불필요하게 커지는 것 억제)
    cost += R_u_ * (delta_f * delta_f + delta_r * delta_r);

    // 제어 증분 비용 (급격한 조향 변화 억제 → 부드러운 주행)
    cost += R_delta_ * (
      (delta_f - prev_df) * (delta_f - prev_df) +
      (delta_r - prev_dr) * (delta_r - prev_dr));

    prev_df = delta_f;
    prev_dr = delta_r;
  }

  return cost;
}

// ── NLopt 최적화 (핵심: 실시간 최적화 루프) ──────────────────────────────────
//
// 실행 흐름:
//   1. 콜백 데이터 포인터 갱신 (O(1), 복사 없음)
//   2. 목적 함수 재등록 (옵티마이저 객체 재사용)
//   3. 초기값 u = u0_ (워밍 스타트)
//   4. NLopt::optimize() 호출 → LD_SLSQP 수렴
//   5. 예외 처리: roundoff_limited → 부분 최적해 사용 (실시간 안전)
//
std::vector<double> MpcUFRWSController::optimizeMPC(
  const VehicleState & current_state,
  const std::vector<VehicleState> & target_seq)
{
  // ── 1. 콜백 데이터 갱신 (포인터만 교체, 힙 할당 없음) ──────────────────────
  nlopt_cb_data_.current_state = &current_state;
  nlopt_cb_data_.target_seq   = &target_seq;

  // ── 2. 목적 함수 등록 (옵티마이저 재사용, 데이터 포인터만 갱신) ─────────────
  nlopt_opt_->set_min_objective(nloptObjectiveCb, &nlopt_cb_data_);

  // ── 3. 초기값: 워밍 스타트 + 박스 제약 클리핑 ───────────────────────────────
  std::vector<double> u = u0_;
  for (auto & val : u) {
    val = std::max(-max_steer_, std::min(max_steer_, val));
  }

  // ── 4. NLopt 최적화 실행 ─────────────────────────────────────────────────────
  double min_cost = std::numeric_limits<double>::max();

  try {
    nlopt::result result = nlopt_opt_->optimize(u, min_cost);

    // 수렴 결과 로깅 (DEBUG 레벨 → 릴리즈 빌드에서 오버헤드 없음)
    RCLCPP_DEBUG(logger_,
      "NLopt result: %d (1=success, 3=ftol, 4=xtol, 5=maxeval), cost=%.6f",
      static_cast<int>(result), min_cost);

  } catch (const nlopt::roundoff_limited &) {
    // 수치 정밀도 한계에 도달 → 현재까지의 최적해를 안전하게 사용
    // 실시간 시스템에서는 정지보다 부분 최적해가 훨씬 안전
    RCLCPP_DEBUG(logger_,
      "NLopt: roundoff_limited, using best solution found (cost=%.6f).", min_cost);

  } catch (const nlopt::forced_stop &) {
    RCLCPP_WARN(logger_, "NLopt: forced_stop triggered.");

  } catch (const std::exception & e) {
    // 복구 불가 예외 → 워밍 스타트(이전 해)를 폴백으로 반환
    RCLCPP_ERROR(logger_,
      "NLopt exception: %s. Falling back to warm-start solution.", e.what());
    return u0_;
  }

  return u;
}

// ── 참조 궤적 생성 ─────────────────────────────────────────────────────────────

std::vector<VehicleState> MpcUFRWSController::generateReferenceTrajectory(
  const VehicleState & current_state) const
{
  const auto & poses   = global_plan_.poses;
  const size_t n_poses = poses.size();

  if (n_poses == 0) {
    throw std::runtime_error("Global plan is empty in generateReferenceTrajectory.");
  }

  // ── 1. 현재 위치에서 가장 가까운 경로점 탐색 ─────────────────────────────
  size_t closest_idx = 0;
  double min_dist    = std::numeric_limits<double>::max();

  for (size_t i = 0; i < n_poses; ++i) {
    const double dx = poses[i].pose.position.x - current_state.x;
    const double dy = poses[i].pose.position.y - current_state.y;
    const double d  = std::hypot(dx, dy);
    if (d < min_dist) {
      min_dist    = d;
      closest_idx = i;
    }
  }

  // ── 2. V_ref·dt 간격으로 N 스텝 참조점 샘플링 ────────────────────────────
  std::vector<VehicleState> ref_seq;
  ref_seq.reserve(static_cast<size_t>(N_));

  const double step_dist = V_ref_ * dt_;  ///< 1 스텝당 이동 거리 [m]

  for (int i = 0; i < N_; ++i) {
    const double target_dist = step_dist * static_cast<double>(i + 1);
    double accumulated = 0.0;
    size_t sel_idx     = closest_idx;

    // 누적 거리가 target_dist를 넘는 경로점 탐색
    for (size_t k = closest_idx; k + 1 < n_poses; ++k) {
      const double seg_dx = poses[k + 1].pose.position.x - poses[k].pose.position.x;
      const double seg_dy = poses[k + 1].pose.position.y - poses[k].pose.position.y;
      accumulated += std::hypot(seg_dx, seg_dy);
      sel_idx = k + 1;
      if (accumulated >= target_dist) {
        break;
      }
    }

    // 참조 상태 구성
    VehicleState ref;
    ref.x = poses[sel_idx].pose.position.x;
    ref.y = poses[sel_idx].pose.position.y;

    // 헤딩: 쿼터니언에서 추출 (경로 생성기가 설정한 값)
    const auto & q = poses[sel_idx].pose.orientation;
    ref.theta = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // 쿼터니언 헤딩이 설정되지 않은 경우 인접점으로 보정
    if (sel_idx + 1 < n_poses) {
      const double fw_dx = poses[sel_idx + 1].pose.position.x - poses[sel_idx].pose.position.x;
      const double fw_dy = poses[sel_idx + 1].pose.position.y - poses[sel_idx].pose.position.y;
      if (std::hypot(fw_dx, fw_dy) > 1e-5) {
        ref.theta = std::atan2(fw_dy, fw_dx);
      }
    } else if (sel_idx > 0) {
      const double bk_dx = poses[sel_idx].pose.position.x - poses[sel_idx - 1].pose.position.x;
      const double bk_dy = poses[sel_idx].pose.position.y - poses[sel_idx - 1].pose.position.y;
      if (std::hypot(bk_dx, bk_dy) > 1e-5) {
        ref.theta = std::atan2(bk_dy, bk_dx);
      }
    }

    ref_seq.push_back(ref);
  }

  return ref_seq;
}

// ── Ackermann 기하학: 등가 조향각 → 개별 바퀴 조향각 (논문 Eq.1) ────────────

WheelAngles MpcUFRWSController::computeWheelAngles(
  double delta_f, double delta_r) const
{
  const double tan_df  = std::tan(delta_f);
  const double tan_dr  = std::tan(delta_r);
  const double denom   = (W_ / (2.0 * L_)) * (tan_df - tan_dr);  // 공통 분모 항

  auto safe_atan = [&](double tan_val, double d) -> double {
      // 분모가 너무 작으면 조향각 0으로 안전 처리
      return (std::abs(d) > 1e-5) ? std::atan(tan_val / d) : 0.0;
    };

  WheelAngles w;
  w.fl = safe_atan(tan_df, 1.0 - denom);  // Front-Left
  w.fr = safe_atan(tan_df, 1.0 + denom);  // Front-Right
  w.rl = safe_atan(tan_dr, 1.0 - denom);  // Rear-Left
  w.rr = safe_atan(tan_dr, 1.0 + denom);  // Rear-Right

  // 안전 클리핑 (actuator 한계)
  auto clip = [&](double v) {
      return std::max(-max_steer_, std::min(max_steer_, v));
    };
  w.fl = clip(w.fl);
  w.fr = clip(w.fr);
  w.rl = clip(w.rl);
  w.rr = clip(w.rr);

  return w;
}

// ── 바퀴 명령 발행 ─────────────────────────────────────────────────────────────

void MpcUFRWSController::publishWheelCommands(
  const WheelAngles & wheels, double v_ref)
{
  const double rad_speed = v_ref / wheel_radius_;  // [rad/s]

  // 조향각 위치 명령: [FL, FR, RL, RR]
  std_msgs::msg::Float64MultiArray steer_msg;
  steer_msg.data = {wheels.fl, wheels.fr, wheels.rl, wheels.rr};
  pub_steer_->publish(steer_msg);

  // 구동 속도 명령: [FL, FR, RL, RR] — 등속 단순 가정
  // (정밀 제어 필요 시 각 바퀴의 ICR 기반 반경 계산으로 확장 가능)
  std_msgs::msg::Float64MultiArray drive_msg;
  drive_msg.data = {rad_speed, rad_speed, rad_speed, rad_speed};
  pub_drive_->publish(drive_msg);
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
  mpc_ufrws_controller::MpcUFRWSController,
  nav2_core::Controller)
