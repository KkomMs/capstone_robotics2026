#!/usr/bin/env python3
"""
SideArucoAligner - 3단계 정렬
==============================
Phase 1 (CENTER)  : linear.x 로 마커를 카메라 중앙(mx→0)
Phase 2 (YAW)     : angular.z 로 랙과 평행(qz→0)
Phase 3 (APPROACH): linear.y 크랩으로 거리(mz→target) 조절

각 Phase 완료 시 잠깐 정지 후 다음 단계 진입.
"""

import math
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros2_aruco_interfaces.msg import ArucoMarkers


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def apply_min_abs(x, min_abs):
    if 0 < abs(x) < min_abs:
        return math.copysign(min_abs, x)
    return x


def lpf(prev, new, alpha):
    return alpha * new + (1.0 - alpha) * prev


class Phase:
    CENTER = "CENTER"  # 1단계: 마커 카메라 중앙
    YAW = "YAW"  # 2단계: 랙과 평행
    APPROACH = "APPROACH"  # 3단계: 거리 조절


class State:
    IDLE = "IDLE"
    ALIGNING = "ALIGNING"
    DONE = "DONE"


class SideArucoAligner(Node):

    def __init__(self):
        super().__init__("side_aruco_aligner")

        # ====================================================
        # 파라미터
        # ====================================================
        self.target_distance: float = 0.90  # [m] 목표 거리 (mz)
        self.marker_timeout: float = 2.0  # [s]
        self.lpf_alpha: float = 0.5
        self.stop_duration: float = 1.0  # [s] 각 Phase 전환 정지 시간

        # Phase 1: 가로 중앙 정렬 (linear.x)
        # mx > 0 → 마커가 오른쪽 → 후진으로 중앙 이동
        self.mx_threshold: float = 0.04  # [m]
        self.kp_x: float = 0.5
        self.max_x: float = 0.10
        self.min_x: float = 0.02

        # Phase 2: 평행 정렬 (angular.z)
        # 실측: 완벽히 평행 시 qz≈0.014 ≈ 0
        self.yaw_threshold: float = 0.02  # qz 단위 (더 엄격하게)
        self.kp_w: float = 1.5
        self.max_w: float = 0.30
        self.min_w: float = 0.03

        # Phase 3: 거리 조절 (linear.y)
        self.mz_threshold: float = 0.05  # [m]
        self.kp_y: float = 0.5
        self.max_y: float = 0.15
        self.min_y: float = 0.02

        self.required_stable: int = 12
        self.target_marker_id = None

        self.camera_side: str = "left"
        self.rack_sign = {"left": +1.0, "right": -1.0}
        self.cmd_topic: str = "/cmd_vel"

        # ====================================================
        # 내부 상태
        # ====================================================
        self.state: str = State.IDLE
        self.phase: str = Phase.CENTER
        self.latest = None
        self.last_stamp = None
        self.cnt = {"mx": 0, "yaw": 0, "mz": 0}
        self.stop_until = None

        # ====================================================
        # ROS I/O
        # ====================================================
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(ArucoMarkers, "/aruco_markers", self._cb, 10)
        self.create_timer(0.05, self._loop)

        self.get_logger().info(
            f"SideArucoAligner 시작 | cmd={self.cmd_topic} | "
            f"target_dist={self.target_distance}m"
        )

    # ====================================================
    # Nav2 cancel
    # ====================================================
    def _cancel_nav(self):
        try:
            from action_msgs.srv import CancelGoal

            cancel_client = self.create_client(
                CancelGoal, "/navigate_to_pose/_action/cancel_goal"
            )
            if cancel_client.service_is_ready():
                future = cancel_client.call_async(CancelGoal.Request())
                future.add_done_callback(
                    lambda f: self.get_logger().info("Nav2 goal cancel 완료")
                )
            else:
                self.get_logger().warn("Nav2 cancel 서비스 미준비 - 스킵")
        except Exception as e:
            self.get_logger().warn(f"Nav2 cancel 실패 (무시): {e}")

    # ====================================================
    # 콜백
    # ====================================================
    def _cb(self, msg: ArucoMarkers):
        parsed = self._parse(msg)
        if parsed is None:
            return

        raw_mx, raw_mz, qx, qy, qz, qw, mid = parsed

        if self.latest is None:
            mx_f, mz_f = raw_mx, raw_mz
        else:
            mx_f = lpf(self.latest[0], raw_mx, self.lpf_alpha)
            mz_f = lpf(self.latest[1], raw_mz, self.lpf_alpha)

        self.latest = (mx_f, mz_f, qx, qy, qz, qw, mid)
        self.last_stamp = self.get_clock().now()

        if self.state == State.IDLE:
            self.get_logger().info(f"마커 ID {mid} 감지! Nav2 cancel → 정렬 시작")
            self._cancel_nav()
            self._stop()
            self._reset_cnt()
            self.phase = Phase.CENTER
            self.state = State.ALIGNING

    def _parse(self, msg: ArucoMarkers):
        if len(msg.marker_ids) == 0:
            return None
        idx = 0
        if self.target_marker_id is not None:
            for i, mid in enumerate(msg.marker_ids):
                if int(mid) == self.target_marker_id:
                    idx = i
                    break
            else:
                return None
        p = msg.poses[idx]
        return (
            float(p.position.x),
            float(p.position.z),
            float(p.orientation.x),
            float(p.orientation.y),
            float(p.orientation.z),
            float(p.orientation.w),
            int(msg.marker_ids[idx]),
        )

    # ====================================================
    # 메인 루프
    # ====================================================
    def _loop(self):
        if self.state in (State.IDLE, State.DONE):
            return
        if self.last_stamp is None or self.latest is None:
            return

        # Phase 전환 정지 대기
        if self.stop_until is not None:
            remaining = (self.stop_until - self.get_clock().now()).nanoseconds / 1e9
            if remaining > 0:
                self._stop()
                self.get_logger().info(
                    f"[{self.phase}] 대기 {remaining:.1f}s...",
                    throttle_duration_sec=0.5,
                )
                return
            else:
                self.stop_until = None

        # 타임아웃
        dt = (self.get_clock().now() - self.last_stamp).nanoseconds / 1e9
        if dt > self.marker_timeout:
            self.get_logger().warn("마커 타임아웃 → IDLE", throttle_duration_sec=1.0)
            self._stop()
            self._reset_cnt()
            self.state = State.IDLE
            return

        mx, mz, qx, qy, qz, qw, mid = self.latest

        mx_err = mx
        yaw_err = float(qz)  # 평행 시 ≈0
        mz_err = mz - self.target_distance

        mx_ok = abs(mx_err) < self.mx_threshold
        yaw_ok = abs(yaw_err) < self.yaw_threshold
        mz_ok = abs(mz_err) < self.mz_threshold

        self.cnt["mx"] = self.cnt["mx"] + 1 if mx_ok else 0
        self.cnt["yaw"] = self.cnt["yaw"] + 1 if yaw_ok else 0
        self.cnt["mz"] = self.cnt["mz"] + 1 if mz_ok else 0

        mx_stable = self.cnt["mx"] >= self.required_stable
        yaw_stable = self.cnt["yaw"] >= self.required_stable
        mz_stable = self.cnt["mz"] >= self.required_stable

        self.get_logger().info(
            f"[{self.phase}][ID {mid}] "
            f"mx={mx:+.3f} mz={mz:.3f} qz={qz:+.4f} | "
            f"mx({'check' if mx_stable  else 'rotation'})({self.cnt['mx']}) "
            f"yaw({'check' if yaw_stable else 'rotation'})({self.cnt['yaw']}) "
            f"mz({'check' if mz_stable  else 'rotation'})({self.cnt['mz']})",
            throttle_duration_sec=0.3,
        )

        # -----------------------------------------------
        # Phase 전환
        # -----------------------------------------------
        if self.phase == Phase.CENTER and mx_stable:
            self.get_logger().info(
                "Phase 1 완료! 마커 중앙 정렬 → Phase 2 (평행 정렬)"
            )
            self._transition(Phase.YAW)
            return

        if self.phase == Phase.YAW and yaw_stable:
            self.get_logger().info("Phase 2 완료! 평행 정렬 → Phase 3 (거리 조절)")
            self._transition(Phase.APPROACH)
            return

        if self.phase == Phase.APPROACH and mz_stable:
            self.get_logger().info("정렬 완료!")
            self._stop()
            self._reset_cnt()
            self.state = State.DONE
            return

        # -----------------------------------------------
        # 제어 명령
        # -----------------------------------------------
        twist = Twist()

        if self.phase == Phase.CENTER:
            # linear.x 만 사용
            # mx > 0 (마커 오른쪽) → 전진으로 중앙 이동
            if not mx_stable:
                vx = clamp(self.kp_x * mx_err, -self.max_x, self.max_x)
                twist.linear.x = apply_min_abs(vx, self.min_x)

        elif self.phase == Phase.YAW:
            # angular.z 만 사용
            if not yaw_stable:
                w = clamp(-self.kp_w * yaw_err, -self.max_w, self.max_w)
                twist.angular.z = apply_min_abs(w, self.min_w)

        elif self.phase == Phase.APPROACH:
            # linear.y 로 거리 조절 + linear.x 로 mx 중앙 유지 동시에
            if not mz_stable:
                vy = clamp(self.kp_y * mz_err, -self.max_y, self.max_y)
                twist.linear.y = self.rack_sign[self.camera_side] * apply_min_abs(
                    vy, self.min_y
                )
            if abs(mx_err) > self.mx_threshold:
                vx = clamp(self.kp_x * mx_err, -self.max_x, self.max_x)
                twist.linear.x = apply_min_abs(vx, self.min_x)

        self.cmd_pub.publish(twist)

    # ====================================================
    # 헬퍼
    # ====================================================
    def _transition(self, next_phase: str):
        """Phase 전환 + 정지 대기."""
        self._stop()
        self._reset_cnt()
        self.phase = next_phase
        self.stop_until = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.stop_duration
        )

    def _reset_cnt(self):
        self.cnt = {"mx": 0, "yaw": 0, "mz": 0}

    def _stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SideArucoAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()