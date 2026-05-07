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
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped

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
XY_SAME_THRESHOLD = 0.05        # 같은 위치라고 판단하는 거리 [m]
YAW_DIFF_THRESHOLD_DEG = 5.0        # yaw가 다르다고 판단하는 차이 [deg]
YAW_DIFF_THRESHOLD = YAW_DIFF_THRESHOLD_DEG * math.pi / 180.0

GENERAL_CHECKER_NAME = "general_goal_checker"
PRECISE_CHECKER_NAME = "precise_goal_checker"

def determine_goal_checker(prev_wp: PoseStamped, curr_wp: PoseStamped) -> str:
    """
    직전 waypoint와 현재 waypoint를 비교하여 적절한 Goalchecker 반환
    """
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
#  (BasicNavigator 가 rclpy.spin 을 내부적으로 쓰기 때문에
#   별도 Node 로 분리해서 MultiThreadedExecutor 로 같이 돌림)
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

        # self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)
        # self.current_pose = None

        gc_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.gc_selector_pub = self.create_publisher(
            String, '/goal_checker_selector', gc_qos
        )

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            Bool, '/aruco_detected', self._on_aruco, qos)
        self.create_subscription(
            Bool, '/alignment_done', self._on_alignment_done, qos)
        self.create_subscription(
            Bool, '/scan_done', self._on_scan_done, qos)

        # ── 내부 플래그 ────────────────────────────────────────────────────
        # 외부(main)에서 읽고 쓰는 플래그들 — threading.Event 사용
        self.aruco_detected   = threading.Event()  # 마커 감지됨
        self.alignment_done   = threading.Event()  # 정렬 완료
        self.scan_done        = threading.Event()  # 스캔 완료

        # 마커 감지 후 일정 시간 중복 트리거 방지
        self._last_aruco_time = 0.0
        self._aruco_cooldown  = 5.0  # [s]

    # ── 콜백 ───────────────────────────────────────────────────────────────

    # def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
    #     self.current_pose = msg.pose.pose

    # def publish_initialpose(self, pose):
    #     msg = PoseWithCovarianceStamped()
    #     msg.header.frame_id = 'map'
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.pose.pose = pose
    #     msg.pose.covariance[0]  = 0.25
    #     msg.pose.covariance[7]  = 0.25
    #     msg.pose.covariance[35] = 0.1
    #     self.initialpose_pub.publish(msg)


    def _on_aruco(self, msg: Bool):
        if not msg.data:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_aruco_time < self._aruco_cooldown:
            return  # 쿨다운 중이면 무시
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
    def publish_pause(self, value: bool):
        msg = Bool()
        msg.data = value
        self.pause_pub.publish(msg)
        self.get_logger().info(
            f'[MissionBridge] /mobile_robot_pause = {value}')
    
    def publish_goal_checker(self, checker_name: str):
        """GoalChecker 이름 publish"""
        msg = String()
        msg.data = checker_name
        self.gc_selector_pub.publish(msg)
        self.get_logger().info(
            f'[MissionBridge] /goal_checker_selector = {checker_name}')

    def reset_flags(self):
        """정렬/스캔 시퀀스 시작 전 플래그 초기화"""
        self.aruco_detected.clear()
        self.alignment_done.clear()
        self.scan_done.clear()

    def wait_for_alignment(self):
        """/alignment_done=true 올 때까지 블로킹 대기"""
        self.alignment_done.wait()

    def wait_for_scan(self):
        """/scan_done=true 올 때까지 블로킹 대기"""
        self.scan_done.wait()

# ─────────────────────────────────────────────────────────────────────────────
#  Goalchecker 선택
# ─────────────────────────────────────────────────────────────────────────────
def publish_checker_for_waypoints(bridge: MissionBridge,
                                  all_waypoints: list,
                                  start_idx: int):
    # 전송할 구간 중 heading-only 전환이 있는지 확인
    has_heading_only = False
    for i in range(start_idx, len(all_waypoints)):
        if i == 0:
            continue
        checker = determine_goal_checker(all_waypoints[i - 1], all_waypoints[i])
        if checker == PRECISE_CHECKER_NAME:
            has_heading_only = True
            break

    # heading-only가 포함된 구간이면 precise, 아니면 general
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
        # 첫 waypoint 또는 이전 waypoint가 없는 경우
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
    """
    주행할 waypoint 리스트 반환.
    좌표/방향은 실제 환경에 맞게 수정할 것.
    """
    half_turn = 1.5708   # 90도
    a_round   = 3.14     # 180도

    coords = [
        # (x,     y,     yaw)
        ( 2.8,  0.0,  0.0),            # wp1: 출발지점에서 직진
        ( 3.76, -4.99, -half_turn),     # wp2: 복도 끝
        (-0.2,  -2.7,  half_turn),    # wp3: 랙 중앙
        (-0.2,  -2.7,  0.0),          # wp4: 랙 사이 바라보기
        ( 3.37, -2.7,  0.0),           # wp5: 1번 랙 끝
        ( 0.1,  0.0, 0.0),            # wp6: 초기위치
        #### 자세정렬 테스트용
        # (3.4, -2.5, 0.0),
    ]

    return [make_pose(navigator, x, y, yaw) for x, y, yaw in coords]


# ─────────────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()

    navigator = BasicNavigator()
    bridge    = MissionBridge()

    # nav2 준비 대기
    navigator.waitUntilNav2Active()
    time.sleep(1.0)

    # BasicNavigator + MissionBridge 를 멀티스레드로 동시에 spin
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    executor.add_node(bridge)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    bridge.reset_flags()

    # ── 전체 waypoint 리스트 ────────────────────────────────────────────────
    all_waypoints = build_waypoints(navigator)
    next_idx      = 0

    print(f'[Mission] 임무 시작: 총 {len(all_waypoints)}개 waypoint')

    # ── 메인 루프 ───────────────────────────────────────────────────────────
    while next_idx < len(all_waypoints):

        # ────────────────────────────────────────────────────────────────────
        # [추가] GoalChecker 구간 분할 및 순차 실행
        # heading-only waypoint가 포함된 경우, 구간을 분할하여
        # 각 구간마다 적절한 GoalChecker를 적용합니다.
        # ────────────────────────────────────────────────────────────────────
        segments = split_waypoints_by_checker(all_waypoints, next_idx)

        if not segments:
            break

        for seg_idx, (checker_name, seg_waypoints) in enumerate(segments):
            # 현재 구간의 global 시작 인덱스 계산
            seg_global_start = next_idx
            for prev_seg in segments[:seg_idx]:
                seg_global_start += len(prev_seg[1])

            print(f'\n[Mission] ── 구간 {seg_idx + 1}/{len(segments)} ──')
            print(f'  GoalChecker: {checker_name}')
            print(f'  Waypoints: {seg_global_start + 1}~'
                  f'{seg_global_start + len(seg_waypoints)}/{len(all_waypoints)}')

            # GoalChecker publish
            bridge.publish_goal_checker(checker_name)
            # BT 노드가 토픽을 수신할 시간 확보
            time.sleep(0.3)

            # timestamp 갱신
            for wp in seg_waypoints:
                wp.header.stamp = navigator.get_clock().now().to_msg()

            bridge.reset_flags()

            # 구간 waypoint 전송
            if len(seg_waypoints) == 1:
                # 단일 waypoint → goToPose 사용 (heading-only에 적합)
                navigator.goToPose(seg_waypoints[0])
            else:
                # 복수 waypoint → followWaypoints 사용
                navigator.followWaypoints(seg_waypoints)

            # ── 주행 모니터링 루프 ──────────────────────────────────────────
            last_wp_idx = -1
            mission_interrupted = False

            while not navigator.isTaskComplete():

                # waypoint 진행 상황 출력
                feedback = navigator.getFeedback()
                if feedback is not None:
                    # followWaypoints 피드백 (current_waypoint 존재)
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

                # 마커 감지 여부 체크 (non-blocking)
                if bridge.aruco_detected.is_set():
                    print('[Mission] ★ 마커 감지! → cancel 후 정렬/스캔 시작')

                    bridge.publish_pause(True)

                    # 현재 진행 중인 global index 추적
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

                    print('[Mission] 스캔 완료 대기 중...')
                    bridge.wait_for_scan()

                    bridge.publish_pause(False)

                    print(f'[Mission] 주행 재개 → wp{next_idx + 1}번 부터')
                    mission_interrupted = True
                    break

            if mission_interrupted:
                break  # segments 루프 탈출 → while next_idx 루프에서 재시작

            # 마커 감지가 task 완료 후에 일어난 경우
            if bridge.aruco_detected.is_set():
                print('[Mission] ★ (task 완료 후) 마커 감지! → 정렬/스캔 시작')
                navigator.cancelTask()
                while not navigator.isTaskComplete():
                    time.sleep(0.05)
                print('[Mission] 정렬 완료 대기 중...')
                bridge.wait_for_alignment()
                print('[Mission] 스캔 완료 대기 중...')
                bridge.wait_for_scan()
                bridge.publish_pause(False)
                mission_interrupted = True
                break

            # 구간 정상 완료 확인
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'[Mission] 구간 {seg_idx + 1} 완료 ✓')
                # 다음 구간으로 계속
            elif result == TaskResult.FAILED:
                global_wp_idx = seg_global_start
                print(f'[Failed] wp{global_wp_idx + 1}/{len(all_waypoints)} 실패! → 재시도...')
                next_idx = seg_global_start
                mission_interrupted = True
                break
            elif result == TaskResult.CANCELED:
                pass

        else:
            # 모든 구간이 정상 완료된 경우
            print('[Mission] 모든 waypoint 완료!')
            next_idx = len(all_waypoints)

        if mission_interrupted:
            continue  # while next_idx 루프 재시작

    print('[Mission] ★★★ 전체 임무 완료 ★★★')

    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()