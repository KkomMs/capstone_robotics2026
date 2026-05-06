#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

class FourWSKeyboardController(Node):

    def __init__(self):
        super().__init__('four_ws_keyboard_controller')
            
        self.wheel_x_offset = 0.215
        self.wheel_y_offset = 0.215
        self.wheel_radius = 0.0695 
        
        # Order must match YAML 'joints' [FL, FR, RL, RR]
        self.wheel_positions = [
            (self.wheel_x_offset, self.wheel_y_offset), # 1: FL
            (self.wheel_x_offset, -self.wheel_y_offset),  # 2: FR
            (-self.wheel_x_offset, self.wheel_y_offset), # 3: RL
            (-self.wheel_x_offset, -self.wheel_y_offset) # 4: RR
        ]

        # Command Storage
        self.pos_cmd = np.zeros(4, float) # Steering Angles (rad)
        self.rad_cmd = np.zeros(4, float) # Wheel Velocity (rad/s)

        # Current State
        self.current_pos = np.zeros(4, float)
        self.current_rad = np.zeros(4, float)

        # cmd_vel input storage for logging
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        # Tuning Parameters (LPF, 0~1. 값이 클수록 반응 빠름)
        self.steering_response = 0.2 
        self.driving_response = 0.5
        self.control_loop_hz = 100.0 # [Hz]

        # Publishers
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        
        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.control_loop_hz, self.update_motor_physics)

        self.get_logger().info("4WIS Controller Ready. Waiting for /cmd_vel...")

    def cmd_vel_callback(self, msg):
        # Store for logging
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Deadzone logic
        if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(wz) < 0.01:
            self.rad_cmd[:] = 0.0
            return

        # Kinematics Calculation
        for i, (rx, ry) in enumerate(self.wheel_positions):
            # V_wheel = V_robot + Omega x R_vector
            wheel_vx = vx - wz * ry
            wheel_vy = vy + wz * rx

            target_speed = math.sqrt(wheel_vx**2 + wheel_vy**2)
            target_angle = math.atan2(wheel_vy, wheel_vx)       # CW: - / CCW: + (origianl)
            #target_angle = math.atan2(-wheel_vy, wheel_vx)     # CW: + / CCW: - (ours)

            # Optimization (Swerve Logic): Keep steering within -90 ~ +90 degrees
            current_angle = self.current_pos[i]
            
            # Normalize angle diff
            ang_diff = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

            if abs(ang_diff) > math.pi / 2:
                if ang_diff > 0:
                    ang_diff -= math.pi
                else:
                    ang_diff += math.pi
                target_speed *= -1.0
            
            final_angle = current_angle + ang_diff

            if final_angle > math.pi or final_angle < -math.pi:
                if final_angle > math.pi:
                    final_angle -= math.pi
                else:
                    final_angle += math.pi
                target_speed *= -1.0
            
            final_angle = max(-math.pi, min(math.pi, final_angle))
            
            self.rad_cmd[i] = target_speed / self.wheel_radius
            self.pos_cmd[i] = final_angle

    def update_motor_physics(self):
        # Low Pass Filter for smooth movement simulation
        self.current_pos += self.steering_response * (self.pos_cmd - self.current_pos)
        self.current_rad += self.driving_response * (self.rad_cmd - self.current_rad)

        self.publish_commands()

    def publish_commands(self):
        pos_msg = Float64MultiArray(data=self.current_pos.tolist())
        vel_msg = Float64MultiArray(data=self.current_rad.tolist())
        
        self.pub_pos.publish(pos_msg)
        self.pub_vel.publish(vel_msg)

        # Logging (Throttled)
        self.get_logger().info(
            "\n"
            "  [motor cmd] steer(deg) / inwheel(m/s)\n"
            f"  FL(1): steer={(self.pos_cmd[0] * 180.0 / math.pi):6.2f}  inwheel={(self.rad_cmd[0] * self.wheel_radius):6.3f}\n"
            f"  FR(2): steer={(self.pos_cmd[1] * 180.0 / math.pi):6.2f}  inwheel={(self.rad_cmd[1] * self.wheel_radius):6.3f}\n"
            f"  RL(3): steer={(self.pos_cmd[2] * 180.0 / math.pi):6.2f}  inwheel={(self.rad_cmd[2] * self.wheel_radius):6.3f}\n"
            f"  RR(4): steer={(self.pos_cmd[3] * 180.0 / math.pi):6.2f}  inwheel={(self.rad_cmd[3] * self.wheel_radius):6.3f}",
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = FourWSKeyboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping robot...")
        # Emergency Stop Command
        zero_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        node.pub_vel.publish(zero_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()