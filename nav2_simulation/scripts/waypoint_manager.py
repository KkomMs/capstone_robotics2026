#!/usr/bin/env python3
import rclpy
import math
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from typing import List


def yaw2quaternion(yaw: float) -> Quaternion:
    # z축 회전 각도(yaw) -> quaternion 변환
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def quaternion2yaw(q: Quaternion) -> float:
    # quaternion -> yaw 변환
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < math.pi:
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
    a_round = 3.14          # 180도 회전

    poses = [
        # (x, y, yaw)
        # ( 3.4,  0.0,  -half_turn ),     # 1. 랙 중앙으로 진입
        # ( 3.4,  0.0,  -a_round ),       # 2. 제자리 회전
        # ( -3.26,  0.0,  -a_round ),     # 3. 랙 끝까지 주행
        # 테스트용
        ( 1.98,  2.99,  0.0 ),
        ( 3.5,  -2.03,  -half_turn ),
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
        self.yaw_threshold = self.get_parameter('yaw_diff_threshold').value * math.pi / 180.0   # [rad]
        self.general_checker = self.get_parameter('general_checker_name').value
        self.precise_checker = self.get_parameter('precise_checker_name').value
        self.gc_topic = self.get_parameter('goal_checker_selector_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # ── Publisher ──────────────────────────────────────────────────
        gc_qos = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.gc_selector_pub = self.create_publisher(String, self.gc_topic, gc_qos)
        # ── Nav2 Client ──────────────────────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(
            f'WaypointManager 초기화 완료. '
            f'xy_threshold={self.xy_threshold:.3f}m, '
            f'yaw_threshold={math.degrees(self.yaw_threshold):.1f}°')
        
    # ── Determine Goal Checker ────────────────────────────────────────────
    def determine_goal_checker(self, prev_wp: PoseStamped, curr_wp: PoseStamped) -> str:
        """
        직전/현재 waypoint를 비교하여 적절한 GoalChecker 반환.
        """
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
        
    # ── GoalChecker publish ─────────────────────────────────────────
    def publish_goal_checker(self, checker_name: str):
        msg = String()
        msg.data = checker_name
        self.gc_selector_pub.publish(msg)
        self.get_logger().info(f'GoalChecker 선택 → {checker_name}')
        

# ─────────────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    manager = WaypointManager()

    # Nav2 시스템이 켜질 때까지 대기
    navigator.waitUntilNav2Active()
    time.sleep(3.0)

    # 웨이포인트 리스트 생성
    all_waypoints = waypoints(navigator)
    total = len(all_waypoints)
    print(f'[Waypoints] 총 {total}개의 waypoints 생성')

    # ── waypoint 순차 실행 (NavigateToPose) ─────────────────────────
    for i, wp in enumerate(all_waypoints):

        # 1. Goalchecker 결정 및 publish
        if i == 0:
            checker = manager.general_checker
            manager.get_logger().info(f'  첫 번째 waypoint → {checker}')
        else:
            checker = manager.determine_goal_checker(all_waypoints[i - 1], wp)

        manager.publish_goal_checker(checker)
        time.sleep(0.3)

        # 2. timestamp 갱신 및 navigation 시작
        wp.header.stamp = navigator.get_clock().now().to_msg()

        yaw_deg = math.degrees(quaternion2yaw(wp.pose.orientation))
        print(f'\n[WP {i + 1}/{total}] 이동 시작: '
              f'({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}, '
              f'yaw={yaw_deg:.1f}°) [{checker}]')

        navigator.goToPose(wp)

        # 3. 주행 모니터링
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                remaining = feedback.distance_remaining
            time.sleep(0.5)
        
        # 4. 결과 확인
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
    rclpy.shutdown()

if __name__ == '__main__':
    main()