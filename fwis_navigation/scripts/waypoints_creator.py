#!/usr/bin/env python3
"""
waypoint_script.py

배치 위치: fwis_navigation/scripts/waypoint_script.py

역할:
  - waypoint 리스트 정의
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
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy


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


# ─────────────────────────────────────────────────────────────────────────────
#  토픽 수신 노드
#  (BasicNavigator 가 rclpy.spin 을 내부적으로 쓰기 때문에
#   별도 Node 로 분리해서 MultiThreadedExecutor 로 같이 돌림)
# ─────────────────────────────────────────────────────────────────────────────
class MissionBridge(Node):
    """
    /aruco_detected, /alignment_done, /scan_done 구독
    /mobile_robot_pause 발행
    """

    def __init__(self):
        super().__init__('mission_bridge')

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        # ── Publishers ─────────────────────────────────────────────────────
        self.pause_pub = self.create_publisher(Bool, '/mobile_robot_pause', qos)

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
        # ( 2.68,  0.10,  0.0),           # wp1: 시작점
        # ( 3.50, -4.25, -half_turn),     # wp2: 1번 랙
        # (-0.7,  -2.5,  0.0),            # wp3: 랙 시작
        # ( 3.02, -2.44,  0.0),           # wp4: 1번 랙 끝
        # ( 0.00,  0.30, -a_round),       # wp5: 좌회전
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

    # BasicNavigator + MissionBridge 를 멀티스레드로 동시에 spin
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    executor.add_node(bridge)

    # executor 를 별도 스레드에서 돌림 (main 스레드는 임무 로직 담당)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # nav2 준비 대기
    navigator.waitUntilNav2Active()

    time.sleep(1.0)
    bridge.reset_flags()

    # ── 전체 waypoint 리스트 ────────────────────────────────────────────────
    all_waypoints = build_waypoints(navigator)
    next_idx      = 0   # 다음에 전송할 waypoint 시작 인덱스

    print(f'[Mission] 임무 시작: 총 {len(all_waypoints)}개 waypoint')

    # ── 메인 루프 ───────────────────────────────────────────────────────────
    while next_idx < len(all_waypoints):

        remaining = all_waypoints[next_idx:]
        print(f'[Mission] FollowWaypoints 시작: {next_idx}번부터 {len(remaining)}개')

        for wp in all_waypoints:
            wp.header.stamp = navigator.get_clock().now().to_msg()

        bridge.reset_flags()
        navigator.followWaypoints(remaining)

        # ── 주행 모니터링 루프 ──────────────────────────────────────────────
        last_wp_idx = -1
        while not navigator.isTaskComplete():

            # waypoint 진행 상황 출력
            feedback = navigator.getFeedback()
            if feedback is not None:
                cur_wp_in_remaining = int(feedback.current_waypoint)
                global_wp_idx = next_idx + cur_wp_in_remaining  # 전체 기준 인덱스

                if cur_wp_in_remaining > last_wp_idx and last_wp_idx >= 0:
                    arrived_global = next_idx + last_wp_idx
                    print(f'[Waypoints] wp{arrived_global + 1}/{len(all_waypoints)} 도착!')

                if cur_wp_in_remaining != last_wp_idx:
                    print(f'[Waypoints] 현재 진행 중: wp{global_wp_idx + 1}/{len(all_waypoints)} '
                            f'(좌표: x={all_waypoints[global_wp_idx].pose.position.x:.2f}, '
                            f'y={all_waypoints[global_wp_idx].pose.position.y:.2f})')
                    last_wp_idx = cur_wp_in_remaining

            # 마커 감지 여부 체크 (non-blocking)
            if bridge.aruco_detected.is_set():
                print('[Mission] ★ 마커 감지! → cancel 후 정렬/스캔 시작')

                feedback = navigator.getFeedback()
                if feedback is not None:
                    next_idx = next_idx + int(feedback.current_waypoint)
                else:
                    pass

                # nav2 cancel
                navigator.cancelTask()

                # nav2가 완전히 멈출 때까지 대기
                while not navigator.isTaskComplete():
                    pass

                # 정렬 완료 대기: /alignment_done=true 올 때까지
                # (aruco_aligner_node가 pause=true도 직접 보냄)
                print('[Mission] 정렬 완료 대기 중...')
                bridge.wait_for_alignment()

                # 스캔 완료 대기: /scan_done=true 올 때까지
                print('[Mission] 스캔 완료 대기 중...')
                bridge.wait_for_scan()

                # pause 해제
                bridge.publish_pause(False)

                print(f'[Mission] 주행 재개 → wp{next_idx + 1}번 부터')
                break  # while not isTaskComplete() 빠져나가서 재시작

        else:
            if bridge.aruco_detected.is_set():
                print('[Mission] ★ (else) 마커 감지! → 정렬/스캔 시작')
                navigator.cancelTask()
                while not navigator.isTaskComplete():
                    time.sleep(0.05)
                print('[Mission] 정렬 완료 대기 중...')
                bridge.wait_for_alignment()
                print('[Mission] 스캔 완료 대기 중...')
                bridge.wait_for_scan()
                bridge.publish_pause(False)
            # isTaskComplete() 가 True 가 된 경우 (마커 감지 없이 정상 완료)
            result = navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                print('[Mission] 모든 waypoint 완료!')
                next_idx = len(all_waypoints)  # 루프 종료

            elif result == TaskResult.CANCELED:
                # cancelTask() 후 여기 올 수 있음 → 위 break로 처리됐어야 하지만 안전장치
                pass

            elif result == TaskResult.FAILED:
                print(f'[Failed] wp{global_wp_idx + 1}/{len(all_waypoints)} 실패! '
                      f'(좌표: x={all_waypoints[global_wp_idx].pose.position.x:.2f}, '
                      f'y={all_waypoints[global_wp_idx].pose.position.y:.2f}) → 재시도...')
                # 실패한 waypoint 그대로 재시도

    print('[Mission] ★★★ 전체 임무 완료 ★★★')

    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()