#!/usr/bin/env python3
"""
waypoint_manager.py

마커 감지 흐름:
  1. aruco_aligner: 마커 감지 → /aruco_detected publish → 대기
  2. waypoint_manager: pause → cancel → heading 맞춤 → /aruco_start publish
  3. aruco_aligner: /aruco_start 수신 → pause → 정렬 시작 (YAW→X→Z→완료)
  4. waypoint_manager: /alignment_done 대기 → 스캔 대기 → pause 해제 → 재주행
"""

import math
import threading
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

MARKER_MAP_POSES = {
    0: (0.472, -2.72),
    1: (1.1,   -2.72),
    2: (1.79,  -2.73),
    3: (2.36,  -2.75),
    4: (2.92,  -2.82),
}

RACK_DRIVE_WP_IDX = 2       # 직접 구동 시작하는 wp index
LAST_MARKER_IDX = 4         # 마지막 마커 index. 해당 마커 스캔 후 nav2 복귀
DIRECT_DRIVE_SPEED = 0.20   # 랙 사이 직진 속도
DIRECT_DRIVE_HZ = 20        # cmd_vel publish 주기

def yaw2quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0; q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def quaternion2yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def make_pose(navigator, x: float, y: float, yaw: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = navigator.get_clock().now().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation = yaw2quaternion(yaw)
    return ps

def normalize_angle(angle: float) -> float:
    while angle > math.pi:  angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def build_waypoints(navigator) -> list:
    """
    - 'goal': (x, y, yaw) - 최종 목표점
    - 'via': [(x, y, yaw), ...] - 경유점 리스트
    'via'가 비어있음: goToPose()
    'via'가 있음: goThroughPoses()
    """
    half_turn = 1.5708
    a_round   = 3.14
    coords = [
        # wp1: 랙 입구
        {
            'goal': (-0.2, -2.55, half_turn),
            'via': [
                (2.8, 0.0, 0.0),             # 시작 지점 복도
                (3.0, -5.0, -a_round),       # 복도
                (1.0, -5.0, -a_round),       # 마지막 복도
            ],
        },
        # wp2: 랙 입구 정렬
        {
            'goal': (-0.2, -2.55, 0.0),
            'via': [],
        },
        # wp3: 랙 출구
        {
            'goal': (3.54, -2.75, 0.0),
            'via': [],
        },
        # wp4: 초기 위치
        {
            'goal': (0.23, -0.05, -a_round),
            'via': [
                (2.8, 0.0, -a_round),
            ],
        },
        # wp5: 초기 위치 정렬
        {
            'goal': (0.23, -0.05, 0.0),
            'via': [],
        },
    ]
    return coords

XY_SAME_THRESHOLD      = 0.05
YAW_DIFF_THRESHOLD_DEG = 5.0
YAW_DIFF_THRESHOLD     = YAW_DIFF_THRESHOLD_DEG * math.pi / 180.0
GENERAL_CHECKER_NAME   = "general_goal_checker"
PRECISE_CHECKER_NAME   = "precise_goal_checker"

def determine_goal_checker(prev_goal: tuple, curr_goal: tuple) -> str:
    px, py, pyaw = prev_goal
    cx, cy, cyaw = curr_goal

    dx = cx - px
    dy = cy - py
    xy_dist = math.sqrt(dx * dx + dy * dy)
    yaw_diff = abs(normalize_angle(cyaw - pyaw))

    if xy_dist < XY_SAME_THRESHOLD and yaw_diff > YAW_DIFF_THRESHOLD:
        print(f'  [GoalChecker] Heading-only → {PRECISE_CHECKER_NAME}')
        return PRECISE_CHECKER_NAME
    else:
        print(f'  [GoalChecker] 일반 주행 → {GENERAL_CHECKER_NAME}')
        return GENERAL_CHECKER_NAME


class MissionBridge(Node):
    def __init__(self):
        super().__init__('mission_bridge')
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        self.pause_pub        = self.create_publisher(Bool, '/mobile_robot_pause', qos)
        self.initialpose_pub  = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.aruco_start_pub  = self.create_publisher(Bool, '/aruco_start', qos)

        gc_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self.gc_selector_pub  = self.create_publisher(String, '/goal_checker_selector', gc_qos)
        self.heading_only_pub = self.create_publisher(Bool, '/heading_only_mode', qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Bool, '/aruco_detected',         self._on_aruco,          qos)
        self.create_subscription(Bool, '/alignment_done',         self._on_alignment_done, qos)
        self.create_subscription(Bool, '/scan_done',              self._on_scan_done,      qos)
        self.create_subscription(Bool, '/cancel_assisted_teleop', self._on_cancel_teleop,  qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)

        self.aruco_detected          = threading.Event()
        self.alignment_done          = threading.Event()
        self.scan_done               = threading.Event()
        self.cancel_teleop_requested = threading.Event()

        self.amcl_lock = threading.Lock()
        self.amcl_x = None; self.amcl_y = None; self.amcl_yaw = None

        self.marker_pair_idx  = 0
        self._last_aruco_time = 0.0
        self._aruco_cooldown  = 2.0  # ★ 수정: 5.0 → 2.0 (마커 재감지 응답속도 향상)

        self._current_checker   = GENERAL_CHECKER_NAME
        self._publish_active    = False
        self._publish_thread    = None
        self._heading_only_mode = False

        # cmd_vel 직접 구동 상태
        self._direct_drive_active = False
        self._direct_drive_thread = None

    def _on_amcl_pose(self, msg):
        o = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(o.w*o.z+o.x*o.y), 1.0-2.0*(o.y*o.y+o.z*o.z))
        with self.amcl_lock:
            self.amcl_x = msg.pose.pose.position.x
            self.amcl_y = msg.pose.pose.position.y
            self.amcl_yaw = yaw

    def _on_aruco(self, msg):
        if not msg.data: return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_aruco_time < self._aruco_cooldown: return
        self._last_aruco_time = now
        self.get_logger().info('[MissionBridge] 마커 감지!')
        self.aruco_detected.set()

    def _on_alignment_done(self, msg):
        if msg.data:
            self.get_logger().info('[MissionBridge] 정렬 완료 ✓')
            self.alignment_done.set()

    def _on_scan_done(self, msg):
        if msg.data:
            self.get_logger().info('[MissionBridge] 스캔 완료 ✓')
            self.scan_done.set()

    def _on_cancel_teleop(self, msg):
        if msg.data:
            self.get_logger().info('[MissionBridge] AssistedTeleop 취소 요청')
            self.cancel_teleop_requested.set()

    def correct_pose(self):
        idx = self.marker_pair_idx
        if idx not in MARKER_MAP_POSES:
            self.get_logger().info(f'[Correct] 마커쌍 {idx} 보정 없음')
            self.marker_pair_idx += 1
            return
        mx, my = MARKER_MAP_POSES[idx]
        with self.amcl_lock:
            yaw = self.amcl_yaw if self.amcl_yaw is not None else 0.0
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x  = mx
        msg.pose.pose.position.y  = my
        msg.pose.pose.orientation = yaw2quaternion(yaw)
        msg.pose.covariance[0]  = 0.01
        msg.pose.covariance[7]  = 0.01
        msg.pose.covariance[35] = 0.005
        self.initialpose_pub.publish(msg)
        self.get_logger().info(f'[Correct] 마커쌍 {idx} → initialpose ({mx:.3f}, {my:.3f})')
        self.marker_pair_idx += 1

    def publish_pause(self, value: bool):
        msg = Bool(); msg.data = value
        self.pause_pub.publish(msg)
        self.get_logger().info(f'[MissionBridge] /mobile_robot_pause = {value}')

    def publish_aruco_start(self):
        msg = Bool(); msg.data = True
        self.aruco_start_pub.publish(msg)
        self.get_logger().info('[MissionBridge] /aruco_start publish')

    def publish_heading_only(self, value: bool):
        msg = Bool(); msg.data = value
        self.heading_only_pub.publish(msg)
        self.get_logger().info(f'[MissionBridge] /heading_only_mode = {value}')

    def reset_flags(self):
        self.aruco_detected.clear()
        self.alignment_done.clear()
        self.scan_done.clear()

    def wait_for_alignment(self): self.alignment_done.wait()
    def wait_for_scan(self):      self.scan_done.wait()

    def _publish_loop(self):
        while self._publish_active:
            msg = String(); msg.data = self._current_checker
            self.gc_selector_pub.publish(msg)
            hm = Bool(); hm.data = self._heading_only_mode
            self.heading_only_pub.publish(hm)
            time.sleep(0.2)

    def start_publishing(self, checker_name: str, heading_only: bool = False):
        self.stop_publishing()
        self._current_checker   = checker_name
        self._heading_only_mode = heading_only
        self._publish_active    = True
        self._publish_thread    = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()
        self.get_logger().info(f'GoalChecker 반복 publish 시작 → {checker_name}')

    def stop_publishing(self):
        self._publish_active = False
        if self._publish_thread is not None:
            self._publish_thread.join(timeout=1.0)
            self._publish_thread = None

    def _direct_drive_loop(self):
        period = 1.0 / DIRECT_DRIVE_HZ
        while self._direct_drive_active:
            msg = Twist()
            msg.linear.x = DIRECT_DRIVE_SPEED
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            time.sleep(period)
    
    def start_direct_drive(self):
        self.stop_direct_drive()
        self._direct_drive_active = True
        self._direct_drive_thread = threading.Thread(
            target=self._direct_drive_loop, daemon=True)
        self._direct_drive_thread.start()
        self.get_logger().info(
            f'[MissionBridge] ★ 직접 구동 시작 (v={DIRECT_DRIVE_SPEED} m/s)')

    def stop_direct_drive(self):
        self._direct_drive_active = False
        if self._direct_drive_thread is not None:
            self._direct_drive_thread.join(timeout=1.0)
            self._direct_drive_thread = None
        # 정지 명령 전송
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info('[MissionBridge] cmd_vel 직접 구동 정지')


# def align_heading(navigator, bridge: MissionBridge, wp: PoseStamped):
#     """
#     현재 위치에서 yaw=0°로 heading 맞추기
#     완료 후 pause → /aruco_start publish → aruco_aligner가 정렬 시작

#     수정사항:
#       - cancel 직후 amcl 안정화 대기 (0.5s)
#       - target_yaw: waypoint의 yaw 사용
#       - yaw 차이 0.5도 미만이면 heading 스킵
#       - heading_only_mode 먼저 publish → 0.5s 대기 → pause 해제 (BT 수신 확보)
#       - heading 완료 후 pause → aruco_start 순서
#     """
#     # cancel 직후 amcl 업데이트 대기
#     time.sleep(0.5)

#     with bridge.amcl_lock:
#         cur_x   = bridge.amcl_x
#         cur_y   = bridge.amcl_y
#         cur_yaw = bridge.amcl_yaw

#     if cur_x is None or cur_y is None:
#         print('[Mission] amcl_pose 없음 → heading 스킵, aruco_start 바로 publish')
#         bridge.publish_aruco_start()
#         return

#     target_yaw = quaternion2yaw(wp.pose.orientation)
#     yaw_diff = abs(normalize_angle(target_yaw - (cur_yaw or 0.0)))

#     print(f'[Mission] heading: 현재={math.degrees(cur_yaw or 0.0):.1f}° '
#           f'목표={math.degrees(target_yaw):.1f}° 차이={math.degrees(yaw_diff):.1f}°')

#     # yaw 차이 0.5도 미만이면 스킵
#     if yaw_diff < math.radians(0.5):
#         print('[Mission] heading 차이 작음 → 스킵, aruco_start publish')
#         bridge.publish_aruco_start()
#         time.sleep(0.2)
#         return

#     heading_wp = make_pose(navigator, cur_x, cur_y, target_yaw)

#     # ★ heading_only_mode 먼저 publish → BT가 수신한 뒤 pause 해제
#     bridge.start_publishing(PRECISE_CHECKER_NAME, heading_only=True)
#     time.sleep(0.5)  # BT가 heading_only_mode 수신할 시간 확보

#     # pause 해제 → nav2/mobile_robot_node 활성화
#     bridge.publish_pause(False)
#     time.sleep(0.2)

#     navigator.goToPose(heading_wp)

#     while not navigator.isTaskComplete():
#         time.sleep(0.1)

#     bridge.stop_publishing()
#     bridge.publish_heading_only(False)

#     result = navigator.getResult()
#     print(f'[Mission] heading 완료: {result}')

#     # heading 완료 → pause → aruco_start
#     bridge.publish_pause(True)
#     time.sleep(0.3)

#     bridge.publish_aruco_start()
#     time.sleep(0.2)

def align_heading(navigator, bridge: MissionBridge, wp: PoseStamped):
    """
    0.5초 대기 후 aruco_start publish
    """
    time.sleep(0.5)  # cancel 직후 안정화 대기
    bridge.publish_aruco_start()
    time.sleep(0.2)

def run_direct_drive(navigator, bridge: MissionBridge, goal_pose: PoseStamped):
    """
    랙 입구 정렬 도달 후 호출, 첫 마커 감지 후 cmd_vel로 직접 주행
    Nav2 controller 대신 cmd_vel.linear.x로 직선 주행
    마지막 마커 스캔 완료 시 Nav2 복귀
    """
    print('[Mission] ══════════════════════════════════════')
    print('[Mission] ★ 랙 사이 주행 모드 진입')
    print(f'[Mission]   Phase 1: Nav2로 첫 마커까지 주행')
    print(f'[Mission]   Phase 2: cmd_vel 직진 (v={DIRECT_DRIVE_SPEED} m/s)')
    print(f'[Mission]   마지막 마커: {LAST_MARKER_IDX}')
    print('[Mission] ══════════════════════════════════════')

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    #  Phase 1: Nav2로 wp3 방향 주행하면서 첫 번째 마커 대기
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    print('[Mission] [Phase 1] Nav2로 첫 마커까지 주행 시작')
    bridge.reset_flags()

    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(goal_pose)
    bridge.start_publishing(GENERAL_CHECKER_NAME, heading_only=False)

    first_marker_found = False
    while not navigator.isTaskComplete():
        if bridge.aruco_detected.is_set():
            print('[Mission] [Phase 1] ★ 첫 번째 마커 감지!')
            bridge.stop_publishing()

            # pause → cancel
            bridge.publish_pause(True)
            navigator.cancelTask()
            while not navigator.isTaskComplete():
                time.sleep(0.05)

            first_marker_found = True
            break

        time.sleep(0.05)

    bridge.stop_publishing()

    if not first_marker_found:
        # Nav2가 wp3까지 도달했는데 마커를 못 찾음 -> 그냥 완료 처리
        result = navigator.getResult()
        print(f'[Mission] [Phase 1] Nav2 완료 (마커 미감지): {result}')
        return True

    # ── 첫 번째 마커 정렬 + 스캔 ──
    current_marker = bridge.marker_pair_idx
    print(f'[Mission] [Phase 1] 첫 마커 처리 (marker_pair_idx={current_marker})')

    align_heading(navigator, bridge, goal_pose)

    print('[Mission] 정렬 완료 대기 중...')
    bridge.wait_for_alignment()

    bridge.correct_pose()
    time.sleep(0.5)

    print('[Mission] 스캔 완료 대기 중...')
    bridge.wait_for_scan()

    # 첫 마커가 마지막 마커였을 경우
    if bridge.marker_pair_idx > LAST_MARKER_IDX:
        print('[Mission] ★ 마지막 마커 스캔 완료 → Nav2 복귀 준비')
        bridge.publish_pause(False)
        bridge.reset_flags()
        return True

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    #  Phase 2: cmd_vel 직진으로 나머지 마커 처리
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    print(f'[Mission] [Phase 2] cmd_vel 직진 모드 시작 (다음 idx={bridge.marker_pair_idx})')
    bridge.reset_flags()
    # pause 해제 후 직접 구동 시작
    bridge.publish_pause(False)
    time.sleep(0.2)
    bridge.start_direct_drive()

    while True:
        # 마커 감지 대기 (50ms 주기 폴링)
        if bridge.aruco_detected.is_set():
            current_marker = bridge.marker_pair_idx
            print(f'[Mission] [Phase 2] ★ 마커 감지! (marker_pair_idx={current_marker})')

            # ── 1) 직진 정지 → pause ──
            bridge.stop_direct_drive()
            time.sleep(0.1)
            bridge.publish_pause(True)

            # ── 2) heading 정렬 (Nav2 goToPose 사용) ──
            align_heading(navigator, bridge, goal_pose)

            # ── 3) 정렬 완료 대기 ──
            print('[Mission] 정렬 완료 대기 중...')
            bridge.wait_for_alignment()

            # ── 4) 위치 보정 ──
            bridge.correct_pose()
            time.sleep(0.5)

            # ── 5) 스캔 완료 대기 ──
            print('[Mission] 스캔 완료 대기 중...')
            bridge.wait_for_scan()

            # ── 6) 마지막 마커 확인 ──
            if bridge.marker_pair_idx > LAST_MARKER_IDX:
                print('[Mission] ★ 마지막 마커 스캔 완료 → Nav2 복귀 준비')
                bridge.publish_pause(False)
                bridge.reset_flags()
                return True

            # ── 7) 아직 마커 남음 → 직진 재개 ──
            print(f'[Mission] [Phase 2] 다음 마커 대기 (다음 idx={bridge.marker_pair_idx})')
            bridge.reset_flags()
            bridge.publish_pause(False)
            time.sleep(0.2)
            bridge.start_direct_drive()

        time.sleep(0.05)

# ══════════════════════════════════════════════════════════════
#  main()
# ══════════════════════════════════════════════════════════════
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

    i = 0
    while i < total:
        wp_info = all_waypoints[i]
        goal_x, goal_y, goal_yaw = wp_info['goal']
        via_list = wp_info.get('via', [])

        # 1) GoalChecker 결정
        goal_pose = make_pose(navigator, goal_x, goal_y, goal_yaw)

        if i == 0:
            checker = GENERAL_CHECKER_NAME
            print(f'  [GoalChecker] 첫 번째 waypoint → {checker}')
        else:
            prev_goal = all_waypoints[i - 1]['goal']
            checker = determine_goal_checker(prev_goal, wp_info['goal'])

        is_heading_only = (checker == PRECISE_CHECKER_NAME)

        yaw_deg = math.degrees(goal_yaw)
        print(f'\n[WP {i + 1}/{total}] 이동 시작: '
              f'({goal_x:.2f}, {goal_y:.2f}, yaw={yaw_deg:.1f}°) [{checker}]'
              f'{" (via " + str(len(via_list)) + " points)" if via_list else ""}')

        # ───────────── 랙 진입 정렬 후 직접 구동 모드 ─────────────
        if i == RACK_DRIVE_WP_IDX:
            # 직접 구동
            run_direct_drive(navigator, bridge, goal_pose)

            # 직접 구동 완료 -> nav2로 랙 출구까지 남은 거리 주행
            print(f'[Mission] Nav2로 WP{i+1} 까지 잔여 주행 재개')
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            navigator.goToPose(goal_pose)
            bridge.start_publishing(GENERAL_CHECKER_NAME, heading_only=False)

            while not navigator.isTaskComplete():
                time.sleep(0.05)

            bridge.stop_publishing()
            result = navigator.getResult()
            print(f'  [WP {i+1}/{total}] Nav2 결과: {result}')

            if result == TaskResult.SUCCEEDED:
                print(f'  [WP {i+1}/{total}] 도달 성공 ✓')
                i += 1
            elif result == TaskResult.CANCELED:
                print(f'  [WP {i+1}/{total}] 취소됨.')
                i += 1
            elif result == TaskResult.FAILED:
                print(f'  [WP {i+1}/{total}] 실패! → 재시도...')
                time.sleep(1.0)
            continue
        
        # ───────────── 일반 주행 (wp1, wp2, wp4, wp5) ─────────────
        # 2) 플래그 초기화
        bridge.reset_flags()

        # 3) timestamp 갱신 및 navigation 호출
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()

        if via_list:
            through_poses = []
            for vx, vy, vyaw in via_list:
                through_poses.append(make_pose(navigator, vx, vy, vyaw))
            through_poses.append(goal_pose)
            navigator.goThroughPoses(through_poses)
            print(f'  → goThroughPoses ({len(through_poses)} poses)')
        else:
            navigator.goToPose(goal_pose)

        # 4) GoalChecker 반복 publish
        bridge.start_publishing(checker, is_heading_only)

        # 5) 주행 모니터링
        wp_interrupted = False
        while not navigator.isTaskComplete():

            if bridge.aruco_detected.is_set():
                print('[Mission] ★ 마커 감지!')
                bridge.stop_publishing()

                # pause → cancel
                bridge.publish_pause(True)
                navigator.cancelTask()
                while not navigator.isTaskComplete():
                    time.sleep(0.05)

                # heading 맞추기 (yaw=0° 고정) → pause → aruco_start publish
                align_heading(navigator, bridge, goal_pose)

                # 정렬 완료 대기
                print('[Mission] 정렬 완료 대기 중...')
                bridge.wait_for_alignment()

                bridge.correct_pose()
                time.sleep(0.5)

                # 스캔 완료 대기
                print('[Mission] 스캔 완료 대기 중...')
                bridge.wait_for_scan()

                bridge.publish_pause(False)
                print(f'[Mission] 주행 재개 → WP{i+1}번 재시도')
                wp_interrupted = True
                break

            time.sleep(0.05)  # ★ 수정: 0.3 → 0.05 (마커 감지 즉시 반응)

        bridge.stop_publishing()

        if wp_interrupted:
            bridge.reset_flags()
            continue

        if bridge.aruco_detected.is_set():
            print('[Mission] ★ (task 완료 후) 마커 감지!')
            bridge.publish_pause(True)
            align_heading(navigator, bridge, goal_pose)

            print('[Mission] 정렬 완료 대기 중...')
            bridge.wait_for_alignment()

            bridge.correct_pose()
            time.sleep(0.5)

            print('[Mission] 스캔 완료 대기 중...')
            bridge.wait_for_scan()
            bridge.publish_pause(False)

            bridge.reset_flags()
            i += 1
            continue

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'  [WP {i+1}/{total}] 도달 성공 ✓')
            i += 1
        elif result == TaskResult.CANCELED:
            print(f'  [WP {i+1}/{total}] 취소됨.')
            i += 1
        elif result == TaskResult.FAILED:
            print(f'  [WP {i+1}/{total}] 실패! → 재시도...')
            time.sleep(1.0)

    print('\n[Mission] ★★★ 전체 waypoint 완료 ★★★')
    bridge.stop_publishing()

    print('[Mission] AssistedTeleop 모드 진입')
    TELEOP_TIME_ALLOWANCE = 300
    try:
        navigator.assistedTeleop(TELEOP_TIME_ALLOWANCE)
        while not navigator.isTaskComplete():
            if bridge.cancel_teleop_requested.is_set():
                navigator.cancelTask()
                break
            time.sleep(0.5)
        result = navigator.getResult()
        print(f'[Mission] AssistedTeleop 종료: {result}')
    except Exception as e:
        print(f'[Mission] AssistedTeleop exception: {e}')

    print('\n[Mission] ★★★ waypoint manager 종료 ★★★')
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()