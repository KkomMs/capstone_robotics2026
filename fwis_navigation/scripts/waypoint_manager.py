#!/usr/bin/env python3
"""
waypoint_manager.py

역할:
  - waypoint 리스트 정의
  - Goalchecker 동적 선택
  - nav2 FollowWaypoints로 이동
  - 이동 중 /aruco_detected=true 수신 시:
      cancel → 정렬 완료(/alignment_done) 대기 → 스캔 완료(/scan_done) 대기
      → /mobile_robot_pause=false publish → 남은 waypoint로 재개
  - 전체 완료

토픽:
  구독: /aruco_detected (Bool) - aruco_aligner_node가 마커 감지 시 publish
        /alignment_done  (Bool) - aruco_aligner_node가 정렬 완료 시 publish
        /scan_done       (Bool) - dynamixel_scan_node가 스캔 완료 시 publish
  발행: /mobile_robot_pause (Bool) - 스캔 완료 후 false publish해서 주행 재개
"""

import math
import threading
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped  # [수정] PoseWithCovarianceStamped 추가
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# ─────────────────────────────────────────────────────────────────────────────
#  [추가] 위치 보정 설정
#  teleop으로 측정한 각 마커 정렬 완료 시점의 로봇 map 좌표
# ─────────────────────────────────────────────────────────────────────────────
MARKER_MAP_POSES = {
    0: (0.472, -2.72),   # 마커 1+2
    1: (1.1,   -2.72),   # 마커 3+4
    2: (1.79,  -2.73),   # 마커 5+6
    3: (2.36,  -2.75),   # 마커 7+8
    4: (2.92,  -2.82),   # 마커 9+10
}

# ─────────────────────────────────────────────────────────────────────────────
#  유틸
# ─────────────────────────────────────────────────────────────────────────────
def yaw2quaternion(yaw: float) -> Quaternion:
    """yaw(rad) → Quaternion"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def quaternion2yaw(q: Quaternion) -> float:
    """Quaternion -> yaw(rad)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def make_pose(navigator: BasicNavigator,
              x: float, y: float, yaw: float) -> PoseStamped:
    """PoseStamped 생성 헬퍼"""
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = navigator.get_clock().now().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation = yaw2quaternion(yaw)
    return ps

def normalize_angle(angle: float) -> float:
    """각도 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# ─────────────────────────────────────────────────────────────────────────────
#  Determine GoalChecker
# ─────────────────────────────────────────────────────────────────────────────
XY_SAME_THRESHOLD = 0.05
YAW_DIFF_THRESHOLD_DEG = 5.0
YAW_DIFF_THRESHOLD = YAW_DIFF_THRESHOLD_DEG * math.pi / 180.0

GENERAL_CHECKER_NAME = "general_goal_checker"
PRECISE_CHECKER_NAME = "precise_goal_checker"

def determine_goal_checker(prev_wp: PoseStamped, curr_wp: PoseStamped) -> str:
    dx = curr_wp.pose.position.x - prev_wp.pose.position.x
    dy = curr_wp.pose.position.y - prev_wp.pose.position.y
    xy_dist = math.sqrt(dx * dx + dy * dy)

    prev_yaw = quaternion2yaw(prev_wp.pose.orientation)
    curr_yaw = quaternion2yaw(curr_wp.pose.orientation)
    yaw_diff = abs(normalize_angle(curr_yaw - prev_yaw))

    xy_same = xy_dist < XY_SAME_THRESHOLD
    yaw_different = yaw_diff > YAW_DIFF_THRESHOLD

    if xy_same and yaw_different:
        print(f'  [GoalChecker] Heading-only 전환 감지 '
              f'(xy_dist={xy_dist:.4f}m, yaw_diff={math.degrees(yaw_diff):.1f}°) '
              f'→ {PRECISE_CHECKER_NAME}')
        return PRECISE_CHECKER_NAME
    else:
        print(f'  [GoalChecker] 일반 주행 '
              f'(xy_dist={xy_dist:.4f}m, yaw_diff={math.degrees(yaw_diff):.1f}°) '
              f'→ {GENERAL_CHECKER_NAME}')
        return GENERAL_CHECKER_NAME

# ─────────────────────────────────────────────────────────────────────────────
#  토픽 수신 노드
# ─────────────────────────────────────────────────────────────────────────────
class MissionBridge(Node):
    """
    /aruco_detected, /alignment_done, /scan_done 구독
    /mobile_robot_pause 발행
    /goal_checker_selector 발행
    """

    def __init__(self):
        super().__init__('mission_bridge')

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        # ── Publishers ─────────────────────────────────────────────────────
        self.pause_pub = self.create_publisher(Bool, '/mobile_robot_pause', qos)

        # [추가] initialpose publisher
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        gc_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.gc_selector_pub = self.create_publisher(
            String, '/goal_checker_selector', gc_qos)

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            Bool, '/aruco_detected', self._on_aruco, qos)
        self.create_subscription(
            Bool, '/alignment_done', self._on_alignment_done, qos)
        self.create_subscription(
            Bool, '/scan_done', self._on_scan_done, qos)

        # [추가] amcl_pose 구독
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)

        # ── 내부 플래그 ────────────────────────────────────────────────────
        self.aruco_detected = threading.Event()
        self.alignment_done = threading.Event()
        self.scan_done      = threading.Event()

        # [추가] amcl_pose 저장용
        self.amcl_lock = threading.Lock()
        self.amcl_x   = None
        self.amcl_y   = None
        self.amcl_yaw = None

        # [추가] 마커쌍 카운터
        self.marker_pair_idx = 0

        self._last_aruco_time = 0.0
        self._aruco_cooldown  = 5.0

    # ── 콜백 ───────────────────────────────────────────────────────────────

    # [추가] amcl_pose 콜백
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        o = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (o.w * o.z + o.x * o.y),
            1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        with self.amcl_lock:
            self.amcl_x   = msg.pose.pose.position.x
            self.amcl_y   = msg.pose.pose.position.y
            self.amcl_yaw = yaw

    def _on_aruco(self, msg: Bool):
        if not msg.data:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_aruco_time < self._aruco_cooldown:
            return
        self._last_aruco_time = now
        self.get_logger().info('[MissionBridge] 마커 감지!')
        self.aruco_detected.set()

    def _on_alignment_done(self, msg: Bool):
        if msg.data:
            self.get_logger().info('[MissionBridge] 정렬 완료 ✓')
            self.alignment_done.set()

    def _on_scan_done(self, msg: Bool):
        if msg.data:
            self.get_logger().info('[MissionBridge] 스캔 완료 ✓')
            self.scan_done.set()

    # ── 헬퍼 ───────────────────────────────────────────────────────────────

    # [추가] initialpose 보정 메서드
    def correct_pose(self):
        idx = self.marker_pair_idx
        if idx not in MARKER_MAP_POSES:
            self.get_logger().info(
                f'[Correct] 마커쌍 {idx} 보정 없음 (좌표 미등록)')
            self.marker_pair_idx += 1
            return

        mx, my = MARKER_MAP_POSES[idx]

        with self.amcl_lock:
            yaw = self.amcl_yaw if self.amcl_yaw is not None else 0.0

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = mx
        msg.pose.pose.position.y = my
        msg.pose.pose.orientation = yaw2quaternion(yaw)
        msg.pose.covariance[0]  = 0.01
        msg.pose.covariance[7]  = 0.01
        msg.pose.covariance[35] = 0.005

        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f'[Correct] 마커쌍 {idx} → initialpose ({mx:.3f}, {my:.3f})')
        self.marker_pair_idx += 1

    def publish_pause(self, value: bool):
        msg = Bool()
        msg.data = value
        self.pause_pub.publish(msg)
        self.get_logger().info(
            f'[MissionBridge] /mobile_robot_pause = {value}')

    def publish_goal_checker(self, checker_name: str):
        msg = String()
        msg.data = checker_name
        self.gc_selector_pub.publish(msg)
        self.get_logger().info(
            f'[MissionBridge] /goal_checker_selector = {checker_name}')

    def reset_flags(self):
        self.aruco_detected.clear()
        self.alignment_done.clear()
        self.scan_done.clear()

    def wait_for_alignment(self):
        self.alignment_done.wait()

    def wait_for_scan(self):
        self.scan_done.wait()


# ─────────────────────────────────────────────────────────────────────────────
#  Goalchecker 선택
# ─────────────────────────────────────────────────────────────────────────────
def publish_checker_for_waypoints(bridge: MissionBridge,
                                  all_waypoints: list,
                                  start_idx: int):
    has_heading_only = False
    for i in range(start_idx, len(all_waypoints)):
        if i == 0:
            continue
        checker = determine_goal_checker(all_waypoints[i - 1], all_waypoints[i])
        if checker == PRECISE_CHECKER_NAME:
            has_heading_only = True
            break

    if has_heading_only:
        bridge.publish_goal_checker(PRECISE_CHECKER_NAME)
    else:
        bridge.publish_goal_checker(GENERAL_CHECKER_NAME)

def split_waypoints_by_checker(all_waypoints: list, start_idx: int) -> list:
    if start_idx >= len(all_waypoints):
        return []

    segments = []
    current_checker = GENERAL_CHECKER_NAME
    current_segment = []

    for i in range(start_idx, len(all_waypoints)):
        if i == 0 or i == start_idx:
            checker = GENERAL_CHECKER_NAME
        else:
            checker = determine_goal_checker(all_waypoints[i - 1], all_waypoints[i])

        if checker != current_checker and current_segment:
            segments.append((current_checker, current_segment))
            current_segment = []
            current_checker = checker

        current_segment.append(all_waypoints[i])
        current_checker = checker

    if current_segment:
        segments.append((current_checker, current_segment))

    return segments


# ─────────────────────────────────────────────────────────────────────────────
#  waypoint 정의
# ─────────────────────────────────────────────────────────────────────────────
def build_waypoints(navigator: BasicNavigator) -> list:
    half_turn = 1.5708
    a_round   = 3.14

    coords = [
        # (x,     y,     yaw)
        # ( 2.8,  0.0,  -half_turn),
        # ( 3.76, -4.99, -half_turn),
        # (-0.2,  -2.7,  half_turn),
        # (-0.2,  -2.7,  0.0),
        # ( 3.37, -2.7,  0.0),
        # ( 0.1,  0.0, 0.0),
        #### 자세정렬 테스트용
        (3.4, -2.5, 0.0),
    ]

    return [make_pose(navigator, x, y, yaw) for x, y, yaw in coords]


# ─────────────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()

    navigator = BasicNavigator()
    bridge    = MissionBridge()

    navigator.waitUntilNav2Active()
    time.sleep(1.0)

    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    executor.add_node(bridge)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    bridge.reset_flags()

    all_waypoints = build_waypoints(navigator)
    next_idx      = 0

    print(f'[Mission] 임무 시작: 총 {len(all_waypoints)}개 waypoint')

    while next_idx < len(all_waypoints):

        segments = split_waypoints_by_checker(all_waypoints, next_idx)

        if not segments:
            break

        for seg_idx, (checker_name, seg_waypoints) in enumerate(segments):
            seg_global_start = next_idx
            for prev_seg in segments[:seg_idx]:
                seg_global_start += len(prev_seg[1])

            print(f'\n[Mission] ── 구간 {seg_idx + 1}/{len(segments)} ──')
            print(f'  GoalChecker: {checker_name}')
            print(f'  Waypoints: {seg_global_start + 1}~'
                  f'{seg_global_start + len(seg_waypoints)}/{len(all_waypoints)}')

            bridge.publish_goal_checker(checker_name)
            time.sleep(0.3)

            for wp in seg_waypoints:
                wp.header.stamp = navigator.get_clock().now().to_msg()

            bridge.reset_flags()

            if len(seg_waypoints) == 1:
                navigator.goToPose(seg_waypoints[0])
            else:
                navigator.followWaypoints(seg_waypoints)

            last_wp_idx = -1
            mission_interrupted = False

            while not navigator.isTaskComplete():

                feedback = navigator.getFeedback()
                if feedback is not None:
                    if hasattr(feedback, 'current_waypoint'):
                        cur_wp_in_seg = int(feedback.current_waypoint)
                        global_wp_idx = seg_global_start + cur_wp_in_seg

                        if cur_wp_in_seg > last_wp_idx and last_wp_idx >= 0:
                            arrived_global = seg_global_start + last_wp_idx
                            print(f'[Waypoints] wp{arrived_global + 1}/'
                                  f'{len(all_waypoints)} 도착!')

                        if cur_wp_in_seg != last_wp_idx:
                            print(f'[Waypoints] 현재 진행 중: '
                                  f'wp{global_wp_idx + 1}/{len(all_waypoints)} '
                                  f'(좌표: x={all_waypoints[global_wp_idx].pose.position.x:.2f}, '
                                  f'y={all_waypoints[global_wp_idx].pose.position.y:.2f})')
                            last_wp_idx = cur_wp_in_seg

                if bridge.aruco_detected.is_set():
                    print('[Mission] ★ 마커 감지! → cancel 후 정렬/스캔 시작')

                    bridge.publish_pause(True)

                    feedback = navigator.getFeedback()
                    if feedback is not None and hasattr(feedback, 'current_waypoint'):
                        next_idx = seg_global_start + int(feedback.current_waypoint)
                    else:
                        next_idx = seg_global_start

                    navigator.cancelTask()
                    while not navigator.isTaskComplete():
                        pass

                    print('[Mission] 정렬 완료 대기 중...')
                    bridge.wait_for_alignment()

                    # [추가] 정렬 완료 시점 위치 보정
                    bridge.correct_pose()
                    time.sleep(0.5)

                    print('[Mission] 스캔 완료 대기 중...')
                    bridge.wait_for_scan()

                    bridge.publish_pause(False)

                    print(f'[Mission] 주행 재개 → wp{next_idx + 1}번 부터')
                    mission_interrupted = True
                    break

            if mission_interrupted:
                break

            if bridge.aruco_detected.is_set():
                print('[Mission] ★ (task 완료 후) 마커 감지! → 정렬/스캔 시작')
                navigator.cancelTask()
                while not navigator.isTaskComplete():
                    time.sleep(0.05)

                print('[Mission] 정렬 완료 대기 중...')
                bridge.wait_for_alignment()

                # [추가] 정렬 완료 시점 위치 보정
                bridge.correct_pose()
                time.sleep(0.5)

                print('[Mission] 스캔 완료 대기 중...')
                bridge.wait_for_scan()
                bridge.publish_pause(False)
                mission_interrupted = True
                break

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'[Mission] 구간 {seg_idx + 1} 완료 ✓')
            elif result == TaskResult.FAILED:
                global_wp_idx = seg_global_start
                print(f'[Failed] wp{global_wp_idx + 1}/{len(all_waypoints)} 실패! → 재시도...')
                next_idx = seg_global_start
                mission_interrupted = True
                break
            elif result == TaskResult.CANCELED:
                pass

        else:
            print('[Mission] 모든 waypoint 완료!')
            next_idx = len(all_waypoints)

        if mission_interrupted:
            continue

    print('[Mission] ★★★ 전체 임무 완료 ★★★')

    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()