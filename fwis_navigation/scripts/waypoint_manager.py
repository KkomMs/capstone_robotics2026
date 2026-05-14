#!/usr/bin/env python3
"""
waypoint_manager.py

역할:
  - waypoint 리스트 정의
  - Goalchecker 동적 선택
  - nav2 goToPose로 waypoint 순차 이동
  - 이동 중 /aruco_detected=true 수신 시:
      cancel → 정렬 완료(/alignment_done) 대기 → 스캔 완료(/scan_done) 대기
      → /mobile_robot_pause=false publish → 현재 waypoint 재시도
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
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
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
#  waypoint 정의
# ─────────────────────────────────────────────────────────────────────────────
def build_waypoints(navigator: BasicNavigator) -> list:
    half_turn = 1.5708
    a_round   = 3.14

    coords = [
        # (x,     y,     yaw)
        ( 2.8,  0.0,  0.0),
        ( 3.76, -4.99, -half_turn),
        (-0.2,  -2.7,  half_turn),
        (-0.2,  -2.7,  0.0),
        ( 3.54, -2.7,  0.0),
        ( 0.23, -0.05, 0.0),
        #### 자세정렬 테스트용
        # (3.4, -2.7, 0.0),
    ]

    return [make_pose(navigator, x, y, yaw) for x, y, yaw in coords]

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
#  토픽 수신 노드 (aruco/정렬/스캔/위치보정 — 기존 코드 그대로)
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

        # initialpose publisher
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        gc_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        # goal checker selector
        self.gc_selector_pub = self.create_publisher(
            String, '/goal_checker_selector', gc_qos)
        # heading-only mode
        self.heading_only_pub = self.create_publisher(
            Bool, '/heading_only_mode', qos)

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            Bool, '/aruco_detected', self._on_aruco, qos)
        self.create_subscription(
            Bool, '/alignment_done', self._on_alignment_done, qos)
        self.create_subscription(
            Bool, '/scan_done', self._on_scan_done, qos)
        self.create_subscription(
            Bool, '/cancel_assisted_teleop',
            self._on_cancel_teleop, qos)

        # [추가] amcl_pose 구독
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)

        # ── 내부 플래그 ────────────────────────────────────────────────────
        self.aruco_detected = threading.Event()
        self.alignment_done = threading.Event()
        self.scan_done      = threading.Event()
        self.cancel_teleop_requested = threading.Event()

        # [추가] amcl_pose 저장용
        self.amcl_lock = threading.Lock()
        self.amcl_x   = None
        self.amcl_y   = None
        self.amcl_yaw = None

        # [추가] 마커쌍 카운터
        self.marker_pair_idx = 0

        self._last_aruco_time = 0.0
        self._aruco_cooldown  = 5.0

        # ── GoalChecker 반복 publish 제어 ──────────────────────────────────
        self._current_checker = GENERAL_CHECKER_NAME
        self._publish_active = False
        self._publish_thread = None

        # heading-only mode
        self._heading_only_mode = False

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

    def _on_cancel_teleop(self, msg: Bool):
        if msg.data:
            self.get_logger().info('[MissionBridge] AssistedTeleop 취소 요청 수신')
            self.cancel_teleop_requested.set()

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
        
    def publish_heading_only(self, value: bool):
        msg = Bool()
        msg.data = value
        self.heading_only_pub.publish(msg)
        self.get_logger().info(
            f'[MissionBridge] /heading_only_mode = {value}')

    def reset_flags(self):
        self.aruco_detected.clear()
        self.alignment_done.clear()
        self.scan_done.clear()

    def wait_for_alignment(self):
        self.alignment_done.wait()

    def wait_for_scan(self):
        self.scan_done.wait()

    # ── GoalChecker 반복 publish (BT 생성 후 수신 보장) ────────────────────
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

            heading_msg = Bool()
            heading_msg.data = self._heading_only_mode
            self.heading_only_pub.publish(heading_msg)

            time.sleep(0.2)

    def start_publishing(self, checker_name: str, heading_only: bool = False):
        """반복 publish 시작"""
        self.stop_publishing()
        self._current_checker = checker_name
        self._heading_only_mode = heading_only
        self._publish_active = True
        self._publish_thread = threading.Thread(
            target=self._publish_loop, daemon=True)
        self._publish_thread.start()
        self.get_logger().info(
            f'GoalChecker 반복 publish 시작 → {checker_name}')

    def stop_publishing(self):
        """반복 publish 중지"""
        self._publish_active = False
        if self._publish_thread is not None:
            self._publish_thread.join(timeout=1.0)
            self._publish_thread = None

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

    all_waypoints = build_waypoints(navigator)
    total = len(all_waypoints)

    print(f'[Mission] 임무 시작: 총 {total}개 waypoint')

    # ── waypoint 순차 실행 (goToPose 개별 호출) ─────────────────────────
    i = 0
    while i < total:
        wp = all_waypoints[i]

        # 1) GoalChecker 결정
        if i == 0:
            checker = GENERAL_CHECKER_NAME
            print(f'  [GoalChecker] 첫 번째 waypoint → {checker}')
        else:
            checker = determine_goal_checker(all_waypoints[i - 1], wp)

        # 2) heading-only 모드 판정
        is_heading_only = (checker == PRECISE_CHECKER_NAME)

        # 3) timestamp 갱신
        wp.header.stamp = navigator.get_clock().now().to_msg()

        yaw_deg = math.degrees(quaternion2yaw(wp.pose.orientation))
        print(f'\n[WP {i + 1}/{total}] 이동 시작: '
              f'({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}, '
              f'yaw={yaw_deg:.1f}°) [{checker}]')

        # 4) 플래그 초기화 & goToPose 호출
        bridge.reset_flags()
        navigator.goToPose(wp)

        # 5) goToPose 직후 GoalChecker 반복 publish 시작
        #    BT의 GoalCheckerSelector가 subscription을 생성한 뒤 수신 보장
        bridge.start_publishing(checker, is_heading_only)

        # 6) 주행 모니터링
        wp_interrupted = False
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and hasattr(feedback, 'distance_remaining'):
                pass  # distance_remaining 활용 가능

            # aruco 감지 체크
            if bridge.aruco_detected.is_set():
                print('[Mission] ★ 마커 감지! → cancel 후 정렬/스캔 시작')

                bridge.stop_publishing()
                bridge.publish_pause(True)

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

                print(f'[Mission] 주행 재개 → WP{i + 1}번 재시도')
                wp_interrupted = True
                break

            time.sleep(0.3)

        # 7) 반복 publish 중지
        bridge.stop_publishing()

        # 8) aruco 인터럽트 → 현재 waypoint 재시도
        if wp_interrupted:
            bridge.reset_flags()
            continue

        # 9) task 완료 직후에도 aruco 체크
        if bridge.aruco_detected.is_set():
            print('[Mission] ★ (task 완료 후) 마커 감지! → 정렬/스캔 시작')

            print('[Mission] 정렬 완료 대기 중...')
            bridge.wait_for_alignment()

            # [추가] 정렬 완료 시점 위치 보정
            bridge.correct_pose()
            time.sleep(0.5)

            print('[Mission] 스캔 완료 대기 중...')
            bridge.wait_for_scan()
            bridge.publish_pause(False)

            # 도착은 했으므로 다음 waypoint로 진행
            bridge.reset_flags()
            i += 1
            continue

        # 10) 결과 확인
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'  [WP {i + 1}/{total}] 도달 성공 ✓')
            i += 1
        elif result == TaskResult.CANCELED:
            print(f'  [WP {i + 1}/{total}] 취소됨.')
            i += 1
        elif result == TaskResult.FAILED:
            print(f'  [WP {i + 1}/{total}] 실패! → 재시도...')
            time.sleep(1.0)
            # i를 증가시키지 않아 동일 waypoint 재시도

    print('\n[Mission] ★★★ 전체 waypoint 주행 완료 ★★★')

    bridge.stop_publishing()

    # ── AssistedTeleop 모드 진입 ─────────────────────────
    print('[Mission] AssistedTeleop 모드 진입 - 조이스틱으로 조종 가능')
    print('[Mission] 종료하려면 /cancel_assisted_teleop topic에 True publish.')

    # 조이스틱 조종 허용 시간
    TELEOP_TIME_ALLOWANCE = 300     # 초

    try:
        navigator.assistedTeleop(TELEOP_TIME_ALLOWANCE)

        while not navigator.isTaskComplete():
            # cancel 토픽 체크
            if bridge.cancel_teleop_requested.is_set():
                print('[Mission] AssistedTeleop 취소 요청 수신')
                navigator.cancelTask()
                break
            time.sleep(0.5)
        
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('[Mission] AssistedTeleop 정상 종료')
        elif result == TaskResult.CANCELED:
            print('[Mission] AssistedTeleop 취소 완료')
        elif result == TaskResult.FAILED:
            print('[Mission] AssistedTeleop 실패')
    
    except Exception as e:
        print(f'[Mission] AssistedTeleop exception: {e}')

    print('\n[Mission] ★★★ waypoint manager 종료 ★★★')
    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()