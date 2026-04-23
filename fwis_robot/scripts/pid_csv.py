#!/usr/bin/env python3
"""
wheel_vel_logger.py
─────────────────────────────────────────────────────────────────
/wheel_vel/front, /wheel_vel/rear 토픽을 CSV로 저장
/motor_N/inwheel 목표속도도 같이 저장
/filtered_vel_N 필터링된 속도도 같이 저장
/odom, /wheel_odom pose, twist 데이터 새로운 CSV로 저장
─────────────────────────────────────────────────────────────────
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry

import csv
import time
from datetime import datetime


class WheelVelLogger(Node):
    def __init__(self):
        super().__init__('wheel_vel_logger')

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # 바퀴 속도 로깅용 CSV 설정
        self.vel_filename = f'wheel_vel_log_{timestamp}.csv'
        self.vel_file = open(self.vel_filename, 'w', newline='')
        self.vel_writer = csv.writer(self.vel_file)

        self.vel_writer.writerow([
            'time_s',
            'target_FL', 'target_FR', 'target_RL', 'target_RR',
            'actual_FL', 'actual_FR', 'actual_RL', 'actual_RR',
            'filtered_FL', 'filtered_FR', 'filtered_RL', 'filtered_RR'
        ])

        # odom 로깅용 CSV 설정
        self.odom_filename = f'odom_log_{timestamp}.csv'
        self.odom_file = open(self.odom_filename, 'w', newline='')
        self.odom_writer = csv.writer(self.odom_file)

        self.odom_writer.writerow([
            'time_s',
            # /odom 토픽 데이터
            'odom_pose_x', 'odom_pose_y', 'odom_pose_z',
            'odom_ori_x', 'odom_ori_y', 'odom_ori_z', 'odom_ori_w',
            'odom_lin_x', 'odom_lin_y', 'odom_lin_z',
            'odom_ang_x', 'odom_ang_y', 'odom_ang_z',
            # /wheel_odom 토픽 데이터
            'wheel_pose_x', 'wheel_pose_y', 'wheel_pose_z',
            'wheel_ori_x', 'wheel_ori_y', 'wheel_ori_z', 'wheel_ori_w',
            'wheel_lin_x', 'wheel_lin_y', 'wheel_lin_z',
            'wheel_ang_x', 'wheel_ang_y', 'wheel_ang_z'
        ])

        self.start_time = time.monotonic()

        # 바퀴 데이터 저장용 딕셔너리
        self.target   = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.actual   = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.filtered = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        # odom 데이터 저장용 딕셔너리
        self.odom_data = self._init_odom_dict()
        self.wheel_odom_data = self._init_odom_dict()

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
        
        # odom 구독
        self.create_subscription(Odometry, '/odom', self._update_odom, qos)
        self.create_subscription(Odometry, 'wheel_odom', self._update_wheel_odom, qos)

        self.create_timer(0.1, self._log)
        self.get_logger().info(f'속도 로깅 시작: {self.vel_filename}')
        self.get_logger().info(f'odom 로깅 시작: {self.odom_filename}')

    def _init_odom_dict(self):
        """Odometry 데이터를 저장할 딕셔너리 구조 초기화"""
        return {
            'px': 0.0, 'py': 0.0, 'pz': 0.0,
            'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0,
            'lx': 0.0, 'ly': 0.0, 'lz': 0.0,
            'ax': 0.0, 'ay': 0.0, 'az': 0.0
        }
    
    def _extract_odom_data(self, msg: Odometry, target_dict: dict):
        """공분산을 제외한 pose, twist 데이터만 추출"""
        target_dict['px'] = msg.pose.pose.position.x
        target_dict['py'] = msg.pose.pose.position.y
        target_dict['pz'] = msg.pose.pose.position.z
        target_dict['ox'] = msg.pose.pose.orientation.x
        target_dict['oy'] = msg.pose.pose.orientation.y
        target_dict['oz'] = msg.pose.pose.orientation.z
        target_dict['ow'] = msg.pose.pose.orientation.w
        
        target_dict['lx'] = msg.twist.twist.linear.x
        target_dict['ly'] = msg.twist.twist.linear.y
        target_dict['lz'] = msg.twist.twist.linear.z
        target_dict['ax'] = msg.twist.twist.angular.x
        target_dict['ay'] = msg.twist.twist.angular.y
        target_dict['az'] = msg.twist.twist.angular.z

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

    def _update_odom(self, msg: Odometry):
        self._extract_odom_data(msg, self.odom_data)

    def _update_wheel_odom(self, msg: Odometry):
        self._extract_odom_data(msg, self.wheel_odom_data)

    def _log(self):
        t = time.monotonic() - self.start_time

        # 바퀴 속도 데이터 기록
        self.vel_writer.writerow([
            f'{t:.3f}',
            f'{self.target["FL"]:.4f}',   f'{self.target["FR"]:.4f}',
            f'{self.target["RL"]:.4f}',   f'{self.target["RR"]:.4f}',
            f'{self.actual["FL"]:.4f}',   f'{self.actual["FR"]:.4f}',
            f'{self.actual["RL"]:.4f}',   f'{self.actual["RR"]:.4f}',
            f'{self.filtered["FL"]:.4f}', f'{self.filtered["FR"]:.4f}',
            f'{self.filtered["RL"]:.4f}', f'{self.filtered["RR"]:.4f}',
        ])
        self.vel_file.flush()

        # odom 데이터 기록
        self.odom_writer.writerow([
            f'{t:.3f}',
            # /odom
            f'{self.odom_data["px"]:.4f}', f'{self.odom_data["py"]:.4f}', f'{self.odom_data["pz"]:.4f}',
            f'{self.odom_data["ox"]:.4f}', f'{self.odom_data["oy"]:.4f}', f'{self.odom_data["oz"]:.4f}', f'{self.odom_data["ow"]:.4f}',
            f'{self.odom_data["lx"]:.4f}', f'{self.odom_data["ly"]:.4f}', f'{self.odom_data["lz"]:.4f}',
            f'{self.odom_data["ax"]:.4f}', f'{self.odom_data["ay"]:.4f}', f'{self.odom_data["az"]:.4f}',
            # /wheel_odom
            f'{self.wheel_odom_data["px"]:.4f}', f'{self.wheel_odom_data["py"]:.4f}', f'{self.wheel_odom_data["pz"]:.4f}',
            f'{self.wheel_odom_data["ox"]:.4f}', f'{self.wheel_odom_data["oy"]:.4f}', f'{self.wheel_odom_data["oz"]:.4f}', f'{self.wheel_odom_data["ow"]:.4f}',
            f'{self.wheel_odom_data["lx"]:.4f}', f'{self.wheel_odom_data["ly"]:.4f}', f'{self.wheel_odom_data["lz"]:.4f}',
            f'{self.wheel_odom_data["ax"]:.4f}', f'{self.wheel_odom_data["ay"]:.4f}', f'{self.wheel_odom_data["az"]:.4f}'
        ])
        self.odom_file.flush()

    def destroy_node(self):
        self.vel_file.close()
        self.odom_file.close()
        self.get_logger().info(f'저장 완료: {self.vel_filename}')
        self.get_logger().info(f'저장 완료: {self.odom_filename}')
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