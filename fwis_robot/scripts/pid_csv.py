#!/usr/bin/env python3
"""
wheel_vel_logger.py
─────────────────────────────────────────────────────────────────
/wheel_vel/front, /wheel_vel/rear 토픽을 CSV로 저장
/motor_N/inwheel 목표속도도 같이 저장
/filtered_vel_N 필터링된 속도도 같이 저장
─────────────────────────────────────────────────────────────────
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

import csv
import time
from datetime import datetime


class WheelVelLogger(Node):
    def __init__(self):
        super().__init__('wheel_vel_logger')

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f'wheel_vel_log_{timestamp}.csv'
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            'time_s',
            'target_FL', 'target_FR', 'target_RL', 'target_RR',
            'actual_FL', 'actual_FR', 'actual_RL', 'actual_RR',
            'filtered_FL', 'filtered_FR', 'filtered_RL', 'filtered_RR'
        ])

        self.start_time = time.monotonic()

        self.target   = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.actual   = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.filtered = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        qos = 10

        # 목표속도 구독
        self.create_subscription(Float32, '/motor_1/inwheel',
            lambda msg: self._update_target('FL', msg), qos)
        self.create_subscription(Float32, '/motor_2/inwheel',
            lambda msg: self._update_target('FR', msg), qos)
        self.create_subscription(Float32, '/motor_3/inwheel',
            lambda msg: self._update_target('RL', msg), qos)
        self.create_subscription(Float32, '/motor_4/inwheel',
            lambda msg: self._update_target('RR', msg), qos)

        # 실제속도 구독
        self.create_subscription(Float32MultiArray, '/wheel_vel/front',
            self._update_actual_front, qos)
        self.create_subscription(Float32MultiArray, '/wheel_vel/rear',
            self._update_actual_rear, qos)

        # 필터링된 속도 구독 (InWheelMotorId = {1,2,3,4} = {FL,FR,RL,RR})
        self.create_subscription(Float32, '/filtered_vel_1',
            lambda msg: self._update_filtered('FL', msg), qos)
        self.create_subscription(Float32, '/filtered_vel_2',
            lambda msg: self._update_filtered('FR', msg), qos)
        self.create_subscription(Float32, '/filtered_vel_3',
            lambda msg: self._update_filtered('RL', msg), qos)
        self.create_subscription(Float32, '/filtered_vel_4',
            lambda msg: self._update_filtered('RR', msg), qos)

        self.create_timer(0.1, self._log)
        self.get_logger().info(f'로깅 시작: {self.filename}')

    def _update_target(self, wheel: str, msg: Float32):
        self.target[wheel] = float(msg.data)

    def _update_actual_front(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.actual['FL'] = msg.data[0]
            self.actual['FR'] = msg.data[1]

    def _update_actual_rear(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.actual['RL'] = msg.data[0]
            self.actual['RR'] = msg.data[1]

    def _update_filtered(self, wheel: str, msg: Float32):
        self.filtered[wheel] = float(msg.data)

    def _log(self):
        t = time.monotonic() - self.start_time
        self.writer.writerow([
            f'{t:.3f}',
            f'{self.target["FL"]:.4f}',   f'{self.target["FR"]:.4f}',
            f'{self.target["RL"]:.4f}',   f'{self.target["RR"]:.4f}',
            f'{self.actual["FL"]:.4f}',   f'{self.actual["FR"]:.4f}',
            f'{self.actual["RL"]:.4f}',   f'{self.actual["RR"]:.4f}',
            f'{self.filtered["FL"]:.4f}', f'{self.filtered["FR"]:.4f}',
            f'{self.filtered["RL"]:.4f}', f'{self.filtered["RR"]:.4f}',
        ])
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        self.get_logger().info(f'저장 완료: {self.filename}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelVelLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()