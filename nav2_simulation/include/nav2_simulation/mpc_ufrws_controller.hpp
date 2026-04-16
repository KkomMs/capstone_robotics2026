#ifndef MPC_UFRWS_CONTROLLER__MPC_UFRWS_CONTROLLER_HPP_
#define MPC_UFRWS_CONTROLLER__MPC_UFRWS_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <limits>

#include <nlopt.hpp>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
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
 */
struct WheelAngles
{
  double fl = 0.0;  ///< Front Left   [rad]
  double fr = 0.0;  ///< Front Right  [rad]
  double rl = 0.0;  ///< Rear Left    [rad]
  double rr = 0.0;  ///< Rear Right   [rad]
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
 * @brief MPC-UFRWS Nav2 컨트롤러
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
   * 재사용함으로써 매 주기의 할당 오버헤드 제거.
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
   * @brief 현재 포즈에서 속도 명령 계산
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 전역 경로 설정
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  // ── NLopt 콜백 ──────────────────────────────────────────────────────────────

  /**
   * @brief NLopt 목적 함수 콜백
   *
   *
   * @param u    현재 제어 시퀀스
   * @param grad 기울기 벡터
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
   * optimizeMPC() 호출 시 포인터만 갱신
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
   * @brief UFRWS 기구학 모델 (논문 Eq.6)
   */
  VehicleState ufrwsModel(
    const VehicleState & state,
    double delta_f,
    double delta_r) const;

  // ── MPC 비용 함수 ───────────────────────────────────────────────────────────

  /**
   * @brief MPC 비용 함수
   *
   * @param u_seq        제어 시퀀스
   * @param current_state 현재 차량 상태
   * @param target_seq   N 스텝 참조 상태 시퀀스
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
   * @param current_state 현재 상태
   * @param target_seq    N 스텝 참조 시퀀스
   * @return 최적 제어 시퀀스
   */
  std::vector<double> optimizeMPC(
    const VehicleState & current_state,
    const std::vector<VehicleState> & target_seq);

  // ── 참조 궤적 생성 ──────────────────────────────────────────────────────────

  /**
   * @brief 전역 경로에서 N 스텝 참조 상태 시퀀스 생성
   */
  std::vector<VehicleState> generateReferenceTrajectory(
    const VehicleState & current_state);

  // ── 4WS Ackermann 기하학 ────────────────────────────────────────────────────

  /**
   * @brief 전/후륜 조향각 및 바퀴 속도 계산, 제자리 회전 속도 계산
   *
   * UFRWS 모델의 개별 조향각 및 ICR 개념을 이용한 바퀴 속도 계산
   */
  WheelAngles computeWheelAngles(double delta_f, double delta_r) const;
  WheelVelocities computeWheelVelocities(double delta_f, double delta_r) const;
  void computePointTurnCommands(double target_yaw_rate, WheelAngles & steer, WheelVelocities & vel) const;

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

  // ── 유틸 ──────────────────────────────────────────────────────────────────

  static double normalizeAngle(double angle);

  // ── ROS2 인터페이스 ────────────────────────────────────────────────────

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("MpcUFRWSController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>      global_pub_;
  std::array<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr, 4>    motor_steer_pubs_;
  std::array<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr, 4>    motor_inwheel_pubs_;

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

  // ── 주행 모드 파라미터 ───────────────────────────────────────────────────────
  bool reversing_mode_;       ///< 후진 모드 여부
  bool point_turning_mode_;   ///< 제자리 회전 모드 여부
  bool is_reversing_;         ///< 후진 여부
  bool is_point_turning_;     ///< 제자리 회전 여부
  double current_v_ref_;      ///< 현재 적용할 동적 속도 [m/s]

  // ── 마지막 조향각 저장 ───────────────────────────────────────────────────────
  WheelAngles last_steer_angles_;     ///< 마지막 조향각 (정지 명령 시 유지용)
  
  // ── 비용 함수 가중치 ─────────────────────────────────────────────────────────

  double Q_y_;     ///< 횡방향 오차 가중치
  double Q_phi_;   ///< 헤딩 각도 오차 가중치
  double R_u_;     ///< 조향각 크기 가중치
  double R_delta_; ///< 조향각 변화량 가중치

  // ── NLopt 최적화 멤버 ────────────────────────────────────────────────────────

  std::unique_ptr<nlopt::opt> nlopt_opt_;  ///< 재사용 NLopt 옵티마이저
  NloptCallbackData nlopt_cb_data_;        ///< 콜백 데이터

  int    opt_max_eval_;   ///< 최대 함수 평가 횟수
  double opt_ftol_rel_;   ///< 함수값 상대 수렴 허용치
  double opt_xtol_rel_;   ///< 변수값 상대 수렴 허용치
  double opt_grad_eps_;   ///< 수치 기울기 유한 차분 스텝 크기

  // ── 워밍 스타트 ──────────────────────────────────────────────────────────────

  std::vector<double> u0_;  ///< 이전 최적해 (다음 주기 초기값)

  // ── 전역 경로 ────────────────────────────────────────────────────────────────

  nav_msgs::msg::Path global_plan_;
  rclcpp::Duration transform_tolerance_{0, 0};

  // ── 타이머 ───────────────────────────────────────────────────────────────────

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_time_;
  void watchdogCallback();

  // ── 모드 처리 함수 ─────────────────────────────────────────────────────────────
  bool orientationModes(
    const VehicleState current_state,
    const geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::TwistStamped & cmd_vel);
};

}  // namespace mpc_ufrws_controller

#endif  // MPC_UFRWS_CONTROLLER__MPC_UFRWS_CONTROLLER_HPP_