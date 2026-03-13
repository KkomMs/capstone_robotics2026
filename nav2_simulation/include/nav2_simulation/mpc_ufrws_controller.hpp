/*
 * MPC-UFRWS (Model Predictive Control - Unconstrained Front Rear Wheel Steering)
 * Nav2 Custom Controller Plugin — NLopt 실시간 최적화 버전
 *
 * Based on: "Optimal Control Method of Path Tracking for Four-Wheel Steering Vehicles"
 * Tan, X.; Liu, D.; Xiong, H. Actuators 2022, 11, 61.
 * https://doi.org/10.3390/act11020061
 *
 * 최적화: SciPy SLSQP → NLopt LD_SLSQP
 *   - 동일 알고리즘 (Sequential Least-Squares Quadratic Programming)
 *   - 제어 주기마다 옵티마이저 객체 재사용으로 오버헤드 최소화
 *   - 박스 제약 (-max_steer ≤ δ ≤ max_steer) NLopt 네이티브 처리
 *
 * 의존성:
 *   sudo apt install libnlopt-cxx-dev
 *   CMakeLists.txt: find_package(NLopt REQUIRED)
 *                   target_link_libraries(... NLopt::nlopt)
 */

#ifndef MPC_UFRWS_CONTROLLER__MPC_UFRWS_CONTROLLER_HPP_
#define MPC_UFRWS_CONTROLLER__MPC_UFRWS_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <limits>

#include <nlopt.hpp>  // NLopt C++ 인터페이스

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace mpc_ufrws_controller
{

// ── 데이터 구조체 ──────────────────────────────────────────────────────────────

/**
 * @brief 차량 상태 벡터: 전역 좌표계 기준 [X, Y, theta(yaw)]
 */
struct VehicleState
{
  double x     = 0.0;  ///< 전역 X 좌표 [m]
  double y     = 0.0;  ///< 전역 Y 좌표 [m]
  double theta = 0.0;  ///< 헤딩 각도 (yaw) [rad]
};

/**
 * @brief 4WS 차량의 개별 바퀴 조향각 (Ackermann 기하학 적용 결과)
 * Convention: FL=Front-Left, FR=Front-Right, RL=Rear-Left, RR=Rear-Right
 */
struct WheelAngles
{
  double fl = 0.0;  ///< Front-Left  조향각 [rad]
  double fr = 0.0;  ///< Front-Right 조향각 [rad]
  double rl = 0.0;  ///< Rear-Left   조향각 [rad]
  double rr = 0.0;  ///< Rear-Right  조향각 [rad]
};

/**
 * @brief 4WS 차량의 개별 바퀴 속도
 */
struct WheelVelocities
{
  double fl = 0.0;
  double fr = 0.0;
  double rl = 0.0;
  double rr = 0.0;
};

// ── 컨트롤러 클래스 ────────────────────────────────────────────────────────────

/**
 * @brief MPC-UFRWS Nav2 컨트롤러 플러그인 (NLopt 실시간 최적화)
 *
 * UFRWS(Unconstrained Front-Rear Wheel Steering) 기구학 모델을 예측 모델로 사용하는
 * MPC 경로 추종 컨트롤러. 전륜/후륜 조향각을 독립적으로 최적화하여 4WS 차량의
 * 조향 자유도를 최대한 활용한다.
 *
 * 제어 입력: [δf_0, δr_0, δf_1, δr_1, ..., δf_{N-1}, δr_{N-1}]  (2N 변수)
 * 상태 벡터: [X, Y, θ]
 *
 * 발행 토픽:
 *   - /forward_position_controller/commands  (바퀴별 조향각 [rad], FL FR RL RR)
 *   - /forward_velocity_controller/commands  (바퀴별 각속도 [rad/s], FL FR RL RR)
 */
class MpcUFRWSController : public nav2_core::Controller
{
public:
  MpcUFRWSController() = default;
  ~MpcUFRWSController() override = default;

  // ── Nav2 Controller 인터페이스 ──────────────────────────────────────────────

  /**
   * @brief 플러그인 초기화: 파라미터 로드 + NLopt 옵티마이저 설정
   *
   * NLopt 객체를 configure() 시점에 1회만 생성하고 이후 제어 주기에서
   * 재사용함으로써 매 주기의 할당 오버헤드를 제거한다.
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief 현재 포즈에서 속도 명령 계산 (NLopt MPC 최적화 수행)
   *
   * 처리 순서:
   *   1. 현재 상태 추출 (quaternion → yaw)
   *   2. N 스텝 참조 궤적 생성
   *   3. NLopt LD_SLSQP 최적화 → 최적 [δf, δr] 획득
   *   4. Ackermann 기하학으로 개별 바퀴 조향각 계산
   *   5. 조향/구동 명령 발행
   *   6. Nav2 표준 TwistStamped 반환
   *
   * @return TwistStamped (linear.x=V_ref, angular.z=등가 yaw rate)
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 전역 경로 설정 (워밍 스타트 초기화 포함)
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  // ── NLopt 콜백 ──────────────────────────────────────────────────────────────

  /**
   * @brief NLopt 목적 함수 콜백 (static: NLopt C++ 함수 포인터 인터페이스)
   *
   * NLopt C++ 시그니처:
   *   double f(const std::vector<double>& x, std::vector<double>& grad, void* data)
   *
   * grad가 비어 있지 않으면 수치 기울기(전진 유한 차분)를 채운다.
   *
   * @param u    현재 제어 시퀀스 (최적화 변수)
   * @param grad 기울기 벡터 (LD 알고리즘일 때 NLopt이 채우도록 요청)
   * @param data NloptCallbackData 포인터
   */
  static double nloptObjectiveCb(
    const std::vector<double> & u,
    std::vector<double> & grad,
    void * data);

protected:
  // ── 내부 데이터 구조 ─────────────────────────────────────────────────────────

  /**
   * @brief NLopt 콜백에 전달되는 컨텍스트 데이터
   *
   * optimizeMPC() 호출 시 포인터만 갱신 → 힙 할당 없음
   */
  struct NloptCallbackData
  {
    const MpcUFRWSController * controller   = nullptr;  ///< mpcCost() 호출용
    const VehicleState       * current_state = nullptr;  ///< 현재 상태 참조
    const std::vector<VehicleState> * target_seq = nullptr;  ///< 참조 궤적 참조
    double eps = 1e-5;  ///< 수치 기울기 유한 차분 스텝 크기
  };

  // ── UFRWS 기구학 모델 ───────────────────────────────────────────────────────

  /**
   * @brief UFRWS 단일 스텝 예측 (논문 Eq.6)
   *
   * Ẋ = V·cos(θ + (δf+δr)/2)
   * Ẏ = V·sin(θ + (δf+δr)/2)
   * θ̇ = V·cos((δf+δr)/2) / L · (δf - δr)
   */
  VehicleState ufrwsModel(
    const VehicleState & state,
    double delta_f,
    double delta_r) const;

  // ── MPC 비용 함수 ───────────────────────────────────────────────────────────

  /**
   * @brief MPC 비용 함수
   *
   * J = Σᵢ [ Q_y·eᵧ² + Q_φ·eφ² + R_u·(δf²+δr²) + R_Δ·(Δδf²+Δδr²) ]
   *
   * eᵧ: 차체 로컬 횡방향 오차 (전역 오차를 차체 방향으로 투영)
   * eφ: 헤딩 각도 오차 ([-π, π] 정규화)
   *
   * @param u_seq        제어 시퀀스 [δf₀,δr₀, δf₁,δr₁, ...]  크기: 2N
   * @param current_state 현재 차량 상태
   * @param target_seq   N 스텝 참조 상태 시퀀스 (전역 좌표계)
   * @return 총 비용값
   */
  double mpcCost(
    const std::vector<double> & u_seq,
    const VehicleState & current_state,
    const std::vector<VehicleState> & target_seq) const;

  // ── NLopt MPC 최적화 ────────────────────────────────────────────────────────

  /**
   * @brief NLopt LD_SLSQP 기반 MPC 최적화
   *
   * 실시간 성능 최적화 포인트:
   *   - nlopt_opt_ 객체 재사용 (configure()에서 1회 생성)
   *   - 콜백 데이터 포인터만 갱신 (deepcopy 없음)
   *   - u0_ 워밍 스타트로 초기 수렴 속도 향상
   *   - roundoff_limited 예외 → 부분 최적해 사용 (실시간 안전)
   *
   * @param current_state 현재 상태
   * @param target_seq    N 스텝 참조 시퀀스
   * @return 최적 제어 시퀀스 (크기: 2N)
   */
  std::vector<double> optimizeMPC(
    const VehicleState & current_state,
    const std::vector<VehicleState> & target_seq);

  // ── 참조 궤적 생성 ──────────────────────────────────────────────────────────

  /**
   * @brief 전역 경로에서 N 스텝 참조 상태 시퀀스 생성
   *
   * 현재 위치에서 가장 가까운 경로점을 찾고, V_ref·dt 간격으로
   * 앞 경로점을 샘플링하여 N개의 참조 상태를 구성한다.
   */
  std::vector<VehicleState> generateReferenceTrajectory(
    const VehicleState & current_state);

  // ── 4WS Ackermann 기하학 ────────────────────────────────────────────────────

  /**
   * @brief 전/후륜 조향각 및 바퀴 속도 계산
   *
   * UFRWS 모델의 개별 조향각 및 ICR 개념을 이용한 바퀴 속도 계산
   */
  WheelAngles computeWheelAngles(double delta_f, double delta_r) const;
  WheelVelocities computeWheelVelocities(double delta_f, double delta_r) const;

  // ── 발행 ────────────────────────────────────────────────────────────────────

  void publishStopCommands();
  
  // ── 발행 ────────────────────────────────────────────────────────────────────

  void publishWheelCommands(const WheelAngles & steer, const WheelVelocities & vel);

  // ── 좌표 변환 ───────────────────────────────────────────────────────────────

  bool transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string & frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance) const;

  // ── 유틸 ────────────────────────────────────────────────────────────────────

  static double normalizeAngle(double angle);

  // ── ROS2 인터페이스 멤버 ────────────────────────────────────────────────────

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("MpcUFRWSController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>      global_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
    std_msgs::msg::Float64MultiArray>> pub_steer_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
    std_msgs::msg::Float64MultiArray>> pub_drive_;

  // ── 차량 파라미터 ────────────────────────────────────────────────────────────

  double Lf_;           ///< 차량 중심~전축 거리 [m]
  double Lr_;           ///< 차량 중심~후축 거리 [m]
  double L_;            ///< 축간 거리 L = Lf + Lr [m]
  double W_;            ///< 윤거 (wheel track) [m]
  double wheel_radius_; ///< 바퀴 반지름 [m]

  // ── MPC 파라미터 ─────────────────────────────────────────────────────────────

  double V_ref_;            ///< 목표 주행 속도 [m/s]
  double lookahead_dist_;   ///< 예측 구간 총 거리 [m]
  double dt_;               ///< 제어 주기 [s]
  int    N_;                ///< 예측 구간 스텝 수
  double max_steer_;        ///< 최대 조향각 [rad]

  // ── 비용 함수 가중치 ─────────────────────────────────────────────────────────

  double Q_y_;     ///< 횡방향 오차 가중치
  double Q_phi_;   ///< 헤딩 각도 오차 가중치
  double R_u_;     ///< 조향각 크기 가중치
  double R_delta_; ///< 조향각 변화량 가중치

  // ── NLopt 최적화 멤버 ────────────────────────────────────────────────────────

  std::unique_ptr<nlopt::opt> nlopt_opt_;  ///< 재사용 NLopt 옵티마이저 (1회 생성)
  NloptCallbackData nlopt_cb_data_;        ///< 콜백 데이터 (매 주기 포인터만 갱신)

  int    opt_max_eval_;   ///< 최대 함수 평가 횟수 (≒ SLSQP 반복)
  double opt_ftol_rel_;   ///< 함수값 상대 수렴 허용치 (e.g. 1e-6)
  double opt_xtol_rel_;   ///< 변수값 상대 수렴 허용치 (e.g. 1e-6)
  double opt_grad_eps_;   ///< 수치 기울기 유한 차분 스텝 크기 (e.g. 1e-5)

  // ── 워밍 스타트 ──────────────────────────────────────────────────────────────

  std::vector<double> u0_;  ///< 이전 최적해 (다음 주기 초기값)

  // ── 전역 경로 ────────────────────────────────────────────────────────────────

  nav_msgs::msg::Path global_plan_;
  rclcpp::Duration transform_tolerance_{0, 0};

  // ── 타이머 ───────────────────────────────────────────────────────────────────

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_time_;
  void watchdogCallback();
};

}  // namespace mpc_ufrws_controller

#endif  // MPC_UFRWS_CONTROLLER__MPC_UFRWS_CONTROLLER_HPP_