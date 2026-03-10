#!/usr/bin/env python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ScannerKeyboardController(Node):
    def __init__(self):
        super().__init__('scanner_keyboard_controller')
        
        self.pub_pos = self.create_publisher(Float64MultiArray, '/scanner_position_controller/commands', 10)

        self.target_pos = np.zeros(2, float)
        self.current_pos = np.zeros(2, float)

        # 모터 속도   
        self.motor_vel = 10.0       # [deg/s]
        self.motor_vel_rad = math.radians(self.motor_vel)

        self.control_loop_hz = 100.0    # [Hz]
        self.dt = 1.0 / self.control_loop_hz
        self.step_size = self.motor_vel_rad * self.dt

        # Timer
        self.timer = self.create_timer(1.0 / self.control_loop_hz, self.update_motor_physics)
        
        self.get_logger().info("Scanner Controller Ready.")
        self.get_logger().info("제한 범위: -40 ~ 90 (Degree)")
        self.get_logger().info(f"모터 속도: {self.motor_vel}deg/s")
        self.get_logger().info("입력 방식: 두 모터의 각도를 띄어쓰기로 구분하여 입력 (예: '30 45')")
        self.get_logger().info("종료하려면 'q'를 입력하세요.")
        
        # 백그라운드 스레드
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input()
                if user_input.lower() == 'q':
                    self.get_logger().info("프로그램을 종료합니다.")
                    rclpy.shutdown()
                    break
                
                # 입력값 파싱
                parts = user_input.split()
                if len(parts) != 2:
                    self.get_logger().warn("두 개의 숫자를 입력하세요.")
                    continue
                
                deg1, deg2 = float(parts[0]), float(parts[1])
                
                if not (-40.0 <= deg1 <= 90.0) or not (-40.0 <= deg2 <= 90.0):
                    self.get_logger().warn("입력 각도가 제한 범위(-40 ~ 180)를 벗어났습니다.")
                    continue
                
                # Degree -> Radian
                rad1 = math.radians(deg1)
                rad2 = math.radians(deg2)
                
                self.target_pos[0] = math.radians(deg1)
                self.target_pos[1] = math.radians(deg2)
                
                self.get_logger().info(f"Scanner1: {deg1}° ({rad1:.4f} rad) | Scanner2: {deg2}° ({rad2:.4f} rad)")
                
            except ValueError:
                self.get_logger().warn("숫자만 입력해주세요.")
            except EOFError:
                break

    def update_motor_physics(self):
        for i in range(2):
            diff = self.target_pos[i] - self.current_pos[i]

            if abs(diff) > self.step_size:
                self.current_pos[i] += math.copysign(self.step_size, diff)
            else:
                self.current_pos[i] = self.target_pos[i]

        msg = Float64MultiArray()
        msg.data = [self.current_pos[0], self.current_pos[1]]
        self.pub_pos.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScannerKeyboardController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()