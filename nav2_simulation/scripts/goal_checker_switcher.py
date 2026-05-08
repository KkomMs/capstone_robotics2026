#!/usr/bin/env python3
"""
goal_checker_switcher.py
────────────────────────────────────────────────────────────────
Nav2의 GoalCheckerSelector BT 노드에 토픽을 발행하여
상황에 따라 Goal Checker를 동적으로 전환하는 예시 노드.

사용 방법:
  1) params_nav2.yaml에 goal_checker_plugins를 복수로 등록
  2) BT XML에 GoalCheckerSelector 노드 추가
  3) 이 노드를 실행하여 동적 전환

동작 모드:
  - "general_goal_checker"   : 일반 주행 (xy=0.25m, yaw=0.35rad)
  - "precise_goal_checker"   : 랙 진입점 정렬 (xy=0.05m, yaw=0.10rad)
  - "corridor_goal_checker"  : 복도 통과 (xy=0.15m, yaw=0.50rad)
────────────────────────────────────────────────────────────────
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import math


class GoalCheckerSwitcher(Node):
    """상황 기반으로 Goal Checker를 동적 전환하는 노드."""

    # ── 전환할 Goal Checker ID (params_nav2.yaml과 일치해야 함) ──
    GENERAL  = "general_goal_checker"
    PRECISE  = "precise_goal_checker"
    CORRIDOR = "corridor_goal_checker"

    def __init__(self):
        super().__init__("goal_checker_switcher")

        # ── 파라미터 ──
        self.declare_parameter("default_checker", self.GENERAL)
        self.declare_parameter("rack_zone_threshold", 1.0)   # [m] 랙 근처 판정 거리

        self.current_checker_ = self.get_parameter("default_checker").value
        self.rack_threshold_  = self.get_parameter("rack_zone_threshold").value

        # ── QoS: GoalCheckerSelector가 요구하는 reliable + transient_local ──
        selector_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Publisher: GoalCheckerSelector가 구독하는 토픽 ──
        self.checker_pub_ = self.create_publisher(
            String,
            "goal_checker_selector",   # BT XML의 topic_name과 일치
            selector_qos,
        )

        # ── 예시 1: 외부 명령으로 수동 전환 ──
        self.manual_sub_ = self.create_subscription(
            String,
            "goal_checker_command",
            self._manual_cb,
            10,
        )

        # ── 예시 2: 네비게이션 목표에 따라 자동 전환 ──
        self.goal_sub_ = self.create_subscription(
            PoseStamped,
            "goal_pose",
            self._goal_cb,
            10,
        )

        # ── 랙 위치 목록 (데이터센터 환경 예시) ──
        # 실제로는 파라미터/파일에서 로드하거나 서비스로 받을 수 있음
        self.rack_positions_ = [
            (3.0, 1.5),   # 랙 A 입구
            (3.0, 4.0),   # 랙 B 입구
            (6.0, 1.5),   # 랙 C 입구
            (6.0, 4.0),   # 랙 D 입구
        ]

        # 초기 상태 발행
        self._publish_checker(self.current_checker_)
        self.get_logger().info(
            f"[GoalCheckerSwitcher] Started. default={self.current_checker_}"
        )

    # ──────────────────────────────────────────────────────────
    #  발행 헬퍼
    # ──────────────────────────────────────────────────────────
    def _publish_checker(self, checker_id: str):
        """Goal Checker ID를 토픽으로 발행."""
        if checker_id != self.current_checker_:
            self.get_logger().info(
                f"[GoalCheckerSwitcher] Switching: {self.current_checker_} → {checker_id}"
            )
        self.current_checker_ = checker_id
        msg = String()
        msg.data = checker_id
        self.checker_pub_.publish(msg)

    # ──────────────────────────────────────────────────────────
    #  방법 1: 수동 전환 (토픽 명령)
    # ──────────────────────────────────────────────────────────
    def _manual_cb(self, msg: String):
        """
        외부에서 직접 전환 명령을 보내는 경우.
        
        사용 예:
          ros2 topic pub --once /goal_checker_command std_msgs/msg/String \
            "data: 'precise_goal_checker'"
        """
        requested = msg.data.strip()
        valid_ids = [self.GENERAL, self.PRECISE, self.CORRIDOR]
        if requested in valid_ids:
            self._publish_checker(requested)
        else:
            self.get_logger().warn(
                f"[GoalCheckerSwitcher] Unknown checker: '{requested}'. "
                f"Valid: {valid_ids}"
            )

    # ──────────────────────────────────────────────────────────
    #  방법 2: 목표 위치 기반 자동 전환
    # ──────────────────────────────────────────────────────────
    def _goal_cb(self, msg: PoseStamped):
        """
        네비게이션 목표가 설정될 때 위치를 보고 적절한 checker를 선택.

        로직:
          - 목표가 랙 입구 근처 → precise (정밀 정렬 필요)
          - 그 외 일반 → general
        """
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        # 랙 입구 근처인지 판단
        near_rack = False
        for (rx, ry) in self.rack_positions_:
            dist = math.hypot(gx - rx, gy - ry)
            if dist < self.rack_threshold_:
                near_rack = True
                break

        if near_rack:
            self._publish_checker(self.PRECISE)
        else:
            self._publish_checker(self.GENERAL)

    # ──────────────────────────────────────────────────────────
    #  방법 3: 프로그래밍 방식 전환 (다른 노드에서 호출)
    # ──────────────────────────────────────────────────────────
    def switch_to_corridor_mode(self):
        """복도 진입 시 호출. 예: crab 전환 상태 머신에서 사용."""
        self._publish_checker(self.CORRIDOR)

    def switch_to_precise_mode(self):
        """랙 앞 정렬 시 호출."""
        self._publish_checker(self.PRECISE)

    def switch_to_general_mode(self):
        """일반 주행 복귀."""
        self._publish_checker(self.GENERAL)


def main(args=None):
    rclpy.init(args=args)
    node = GoalCheckerSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
