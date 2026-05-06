#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

class FourWSKeyboardController(Node):

    def __init__(self):
        super().__init__('lyapunov_kinematics_node')
            
        self.wheel_x_offset = 0.215
        self.wheel_y_offset = 0.215
        self.width = 0.43
        self.wheel_radius = 0.0695 
        
        # Order must match YAML 'joints' [FL, FR, RR, RL]
        self.wheel_positions = [
            (self.wheel_x_offset, self.wheel_y_offset), # 1: FR
            (self.wheel_x_offset, -self.wheel_y_offset),  # 2: FL
            (-self.wheel_x_offset, -self.wheel_y_offset), # 3: RL
            (-self.wheel_x_offset, self.wheel_y_offset) # 4: RR
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

        self.get_logger().info("Lyapunov Kinematics Node Ready. Waiting for nav2 /cmd_vel...")

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
        
        # Handling singularity when v_A(linear.x) is close to 0
        if abs(vx) < 1e-3:
            delta_f = 0.0
        else:
            delta_f = math.atan((wz * self.wheel_x_offset) / vx)

        # Saturate delta_f if needed
        max_steer = math.radians(60)
        delta_f = max(min(delta_f, max_steer), -max_steer)

        # Calculate ICR Radius R
        # Handle singularity when delta_f is 0 (Straight motion)
        if abs(delta_f) < 1e-3:
            R = float('inf')
        else:
            R = self.wheel_x_offset / math.tan(delta_f)

        # Calculate Individual Steering Angles
        # delta_1 (FL), delta_2 (FR), delta_3 (RL), delta_4 (RR)
        if R == float('inf'):
            d1 = d2 = d3 = d4 = 0.0
        else:
            d1 = math.atan(self.wheel_x_offset / (R - self.width / 2.0))
            d2 = math.atan(self.wheel_x_offset / (R + self.width / 2.0))
            d3 = -math.atan(self.wheel_y_offset / (R + self.width / 2.0))
            d4 = -math.atan(self.wheel_y_offset / (R - self.width / 2.0))
        
        # Calculate Individual Wheel Velocities
        if R == float('inf'):
            v1 = v2 = v3 = v4 = vx
        else:
            v1 = vx * (R - self.width / 2.0) / (R * math.cos(d1))
            v2 = vx * (R + self.width / 2.0) / (R * math.cos(d2))
            v3 = vx * (R + self.width / 2.0) / (R * math.cos(d3))
            v4 = vx * (R - self.width / 2.0) / (R * math.cos(d4))
        
        # Linear velocity to Angular velocity
        rad1 = v1 / self.wheel_radius
        rad2 = v2 / self.wheel_radius
        rad3 = v3 / self.wheel_radius
        rad4 = v4 / self.wheel_radius

        self.pos_cmd = np.array([d1, d2, d3, d4])
        self.rad_cmd = np.array([rad1, rad2, rad3, rad4])

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
            f"CMD Input -> Vx: {self.cmd_vx:.2f} m/s | Vy: {self.cmd_vy:.2f} m/s | Wz: {self.cmd_wz:.2f} rad/s",
            throttle_duration_sec=2.0)

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