#!/usr/bin/env python3
import rclpy
import math
import time
import threading
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String, Bool
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
 
 
def yaw2quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q
 
def quaternion2yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
 
def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
 
def make_pose(navigator: BasicNavigator,
              x: float, y: float, yaw: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = navigator.get_clock().now().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation = yaw2quaternion(yaw)
    return ps
 
def waypoints(navigator: BasicNavigator) -> list:
    half_turn = 1.5708      # 90도 회전
    a_round = 3.14           # 180도 회전
 
    poses = [
        # (x, y, yaw)
        ( 1.98,  2.99,  0.0 ),
        ( 3.5,  -2.20,  -half_turn ),
        ( -3.48,  0.07,  half_turn ),
        ( -3.48,  0.07,  0.0),
        ( 3.03,  0.07, 0.0),
        (-2.97,  3.0,  0.0),
    ]
 
    return [make_pose(navigator, x, y, yaw) for x, y, yaw in poses]
 
 
# ─────────────────────────────────────────────────────────────────────────────
#  WaypointManager Node
# ─────────────────────────────────────────────────────────────────────────────
class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
 
        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('xy_same_threshold', 0.05)       # [m]
        self.declare_parameter('yaw_diff_threshold', 5.0)       # [deg]
        self.declare_parameter('general_checker_name', 'general_goal_checker')
        self.declare_parameter('precise_checker_name', 'precise_goal_checker')
        self.declare_parameter('goal_checker_selector_topic', 'goal_checker_selector')
        self.declare_parameter('frame_id', 'map')
 
        self.xy_threshold = self.get_parameter('xy_same_threshold').value
        self.yaw_threshold = self.get_parameter('yaw_diff_threshold').value * math.pi / 180.0
        self.general_checker = self.get_parameter('general_checker_name').value
        self.precise_checker = self.get_parameter('precise_checker_name').value
        self.gc_topic = self.get_parameter('goal_checker_selector_topic').value
        self.frame_id = self.get_parameter('frame_id').value
 
        # ── Publisher (RELIABLE + TRANSIENT_LOCAL) ─────────────────────
        gc_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.gc_selector_pub = self.create_publisher(String, self.gc_topic, gc_qos)
        self.heading_only_pub = self.create_publisher(Bool, '/heading_only_mode', qos)
 
        # ── 반복 publish 제어 ────────────────────────────────────────
        self._current_checker = self.general_checker
        self._publish_active = False
        self._publish_thread = None
 
        # ── GetParameters Service Client ─────────────────────────────
        self.get_params_client = self.create_client(
            GetParameters, '/controller_server/get_parameters')
 
        self.get_logger().info(
            f'WaypointManager 초기화 완료. '
            f'xy_threshold={self.xy_threshold:.3f}m, '
            f'yaw_threshold={math.degrees(self.yaw_threshold):.1f}°')
 
    # ── Determine Goal Checker ────────────────────────────────────────
    def determine_goal_checker(self, prev_wp: PoseStamped, curr_wp: PoseStamped) -> str:
        dx = curr_wp.pose.position.x - prev_wp.pose.position.x
        dy = curr_wp.pose.position.y - prev_wp.pose.position.y
        xy_dist = math.sqrt(dx * dx + dy * dy)
 
        prev_yaw = quaternion2yaw(prev_wp.pose.orientation)
        curr_yaw = quaternion2yaw(curr_wp.pose.orientation)
        yaw_diff = abs(normalize_angle(curr_yaw - prev_yaw))
 
        xy_same = xy_dist < self.xy_threshold
        yaw_different = yaw_diff > self.yaw_threshold
 
        if xy_same and yaw_different:
            self.get_logger().info(
                f'  Heading-only 전환 감지 '
                f'(xy={xy_dist:.4f}m, Δyaw={math.degrees(yaw_diff):.1f}°) '
                f'→ {self.precise_checker}')
            return self.precise_checker
        else:
            self.get_logger().info(
                f'  일반 주행 '
                f'(xy={xy_dist:.4f}m, Δyaw={math.degrees(yaw_diff):.1f}°) '
                f'→ {self.general_checker}')
            return self.general_checker
        
    # ── heading-only 모드 발행 ──────────────
    def publish_heading_only(self, value: Bool):
        msg = Bool()
        msg.data = value
        self.heading_only_pub.publish(msg)
        self.get_logger().info(
            f'[MissionBridge] /heading_only_mode = {value}')
 
    # ── GoalChecker 반복 publish (BT 생성 후 수신 보장) ──────────────
    def _publish_loop(self):
        """
        BT 내부의 GoalCheckerSelector가 subscription을 생성한 뒤
        메시지를 확실히 수신할 수 있도록, 짧은 간격으로 반복 publish.
        goToPose() 직후 시작, navigation 진행 중 계속 실행.
        """
        while self._publish_active:
            msg = String()
            msg.data = self._current_checker
            self.gc_selector_pub.publish(msg)
            time.sleep(0.2)  # 200ms 간격으로 반복
 
    def start_publishing(self, checker_name: str):
        """반복 publish 시작"""
        self._current_checker = checker_name
        self._publish_active = True
        self._publish_thread = threading.Thread(
            target=self._publish_loop, daemon=True)
        self._publish_thread.start()
        self.get_logger().info(f'GoalChecker 반복 publish 시작 → {checker_name}')
 
    def stop_publishing(self):
        """반복 publish 중지"""
        self._publish_active = False
        if self._publish_thread is not None:
            self._publish_thread.join(timeout=1.0)
            self._publish_thread = None
 
    # ── GetParameters ─────────────────────────────────────────
    def print_goal_checker_params(self):
        if not self.get_params_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                '/controller_server/get_parameters 서비스를 찾을 수 없습니다.')
            return
 
        param_names = [
            f'{self.general_checker}.xy_goal_tolerance',
            f'{self.general_checker}.yaw_goal_tolerance',
            f'{self.precise_checker}.xy_goal_tolerance',
            f'{self.precise_checker}.yaw_goal_tolerance',
        ]
 
        request = GetParameters.Request()
        request.names = param_names
 
        future = self.get_params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
 
        if future.result() is None:
            self.get_logger().warn('GoalChecker 파라미터 조회 실패.')
            return
 
        values = future.result().values
        if len(values) != len(param_names):
            self.get_logger().warn(
                f'파라미터 응답 불일치: 요청={len(param_names)}, 응답={len(values)}')
            return
 
        def get_double(pv):
            if pv.type == 3:
                return pv.double_value
            elif pv.type == 2:
                return float(pv.integer_value)
            return None
 
        g_xy  = get_double(values[0])
        g_yaw = get_double(values[1])
        p_xy  = get_double(values[2])
        p_yaw = get_double(values[3])
 
        print()
        print('=' * 56)
        print('  GoalChecker Parameters (from controller_server)')
        print('=' * 56)
        if g_xy is not None and g_yaw is not None:
            print(f'  [{self.general_checker}]')
            print(f'    xy_goal_tolerance  : {g_xy:.3f} m')
            print(f'    yaw_goal_tolerance : {g_yaw:.3f} rad  ({math.degrees(g_yaw):.1f}°)')
        else:
            print(f'  [{self.general_checker}] 파라미터 조회 실패')
        print('-' * 56)
        if p_xy is not None and p_yaw is not None:
            print(f'  [{self.precise_checker}]')
            print(f'    xy_goal_tolerance  : {p_xy:.3f} m')
            print(f'    yaw_goal_tolerance : {p_yaw:.3f} rad  ({math.degrees(p_yaw):.1f}°)')
        else:
            print(f'  [{self.precise_checker}] 파라미터 조회 실패')
        print('=' * 56)
        print()
 
 
# ─────────────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
 
    navigator = BasicNavigator()
    navigator.set_parameters(
        [Parameter('use_sim_time', Parameter.Type.BOOL, True)])
 
    manager = WaypointManager()
 
    # Nav2 시스템이 켜질 때까지 대기
    navigator.waitUntilNav2Active()
    time.sleep(3.0)
 
    # 파라미터 출력
    manager.print_goal_checker_params()
 
    # 웨이포인트 리스트 생성
    all_waypoints = waypoints(navigator)
    total = len(all_waypoints)
    print(f'[Waypoints] 총 {total}개의 waypoints 생성')
 
    # ── waypoint 순차 실행 ──────────────────────────────────────────
    for i, wp in enumerate(all_waypoints):
 
        # 1. GoalChecker 결정
        if i == 0:
            checker = manager.general_checker
            manager.get_logger().info(f'  첫 번째 waypoint → {checker}')
        else:
            checker = manager.determine_goal_checker(all_waypoints[i - 1], wp)

        # 2. heading-only 모드 판정 & 발행
        is_heading_only = (checker == manager.precise_checker)
        manager.publish_heading_only(is_heading_only)
 
        # 3. timestamp 갱신
        wp.header.stamp = navigator.get_clock().now().to_msg()
 
        yaw_deg = math.degrees(quaternion2yaw(wp.pose.orientation))
        print(f'\n[WP {i + 1}/{total}] 이동 시작: '
              f'({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}, '
              f'yaw={yaw_deg:.1f}°) [{checker}]')
 
        # 4. goToPose 호출 → BT 생성됨
        navigator.goToPose(wp)
 
        # 5. goToPose 직후, BT의 GoalCheckerSelector가 subscription 생성될 때까지
        #    반복적으로 publish (BT가 수신할 수 있도록)
        manager.start_publishing(checker)
 
        # 6. 주행 모니터링
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                remaining = feedback.distance_remaining
            time.sleep(0.5)
 
        # 7. 반복 publish 중지
        manager.stop_publishing()
        manager.publish_heading_only(False)
 
        # 8. 결과 확인
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'  [WP {i + 1}/{total}] 도달 성공.')
        elif result == TaskResult.CANCELED:
            print(f'  [WP {i + 1}/{total}] 취소됨.')
            break
        elif result == TaskResult.FAILED:
            print(f'  [WP {i + 1}/{total}] 실패! 다음 waypoint로 진행...')
            continue
 
    print('\n[Waypoints] ★★★ 전체 주행 완료 ★★★')
    manager.stop_publishing()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
