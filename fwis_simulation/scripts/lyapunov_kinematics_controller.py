#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import numpy as np
import math

class LyapunovKinematicsNode(Node):
    def __init__(self):
        super().__init__('lyapunov_kinematics_ctrl')
        
        self.Lf = 0.215 # Front length [m]
        self.Lr = 0.215 # Rear length [m]
        self.W = 0.43   # Width [m]
        self.radius = 0.0695    # Wheel Radius [m]
        
        # Control gains
        self.kx = 0.6
        self.ky = 0.5
        self.k_theta = 1.0
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_actual = 0.0 # 현재 선속도
        
        # # --- 4. Waypoints / Trajectory (Example: Straight Line) ---
        # # 간단한 테스트를 위해 기준 속도와 경로 설정
        # self.v_ref = 0.5  # Reference velocity [m/s]
        # self.omega_ref = 0.0 # Reference yaw rate [rad/s]

        # --- Trajectory Parameters ---
        self.v_ref = 0.15          # 주행 속도 [m/s]
        self.lane_length = 2.0    # 한 레인의 길이 [m]
        self.row_width = 3.2      # 레인 간 간격 (U-turn 지름) [m]
        self.turn_radius = self.row_width / 2.0
        
        self.dt = 0.01  # 100 [Hz]
        
        # Publishers
        self.pub_steer = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_drive = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        
        # Subscriber
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("Lyapunov Kinematics Controller Started.")

    def odom_callback(self, msg):
        # Position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Orientation (Quaternion to Euler Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Linear Velocity (Local Frame)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.v_actual = math.sqrt(vx**2 + vy**2)

    def get_reference_state(self, t):
        """
        'ㄹ'자 (Zigzag) 직선 궤적 생성 함수 (Rectangular Path)
        Pattern: East -> North -> West -> North -> Repeat
        곡선 회전(U-Turn) 없이 직선으로만 연결된 경로입니다.
        """
        # 각 구간별 소요 시간 계산
        # 가로 이동 시간 (East/West)
        t_horz = self.lane_length / self.v_ref
        # 세로 이동 시간 (North - 다음 레인으로 이동)
        t_vert = self.row_width / self.v_ref
        
        # 한 사이클(우측 이동 -> 위로 -> 좌측 이동 -> 위로)의 총 시간
        cycle_time = 2 * t_horz + 2 * t_vert
        
        # 현재 사이클 횟수 및 사이클 내 시간
        cycle_count = int(t / cycle_time)
        t_cycle = t % cycle_time
        
        # 기본 Y 오프셋 (사이클마다 2 * row_width 만큼 위로 이동해 있음)
        y_offset_base = cycle_count * (2 * self.row_width)
        
        x_r = 0.0
        y_r = 0.0
        theta_r = 0.0
        
        # Phase 1: 동쪽 직진 (East, +X)
        if t_cycle < t_horz:
            x_r = self.v_ref * t_cycle
            y_r = 0.0
            theta_r = 0.0
            
        # Phase 2: 위로 직진 (North, +Y) - 오른쪽 끝에서 위로 이동
        elif t_cycle < (t_horz + t_vert):
            dt = t_cycle - t_horz
            x_r = self.lane_length
            y_r = self.v_ref * dt
            theta_r = math.pi / 2.0  # 90도 (위쪽을 바라봄)
            
        # Phase 3: 서쪽 직진 (West, -X)
        elif t_cycle < (2 * t_horz + t_vert):
            dt = t_cycle - (t_horz + t_vert)
            x_r = self.lane_length - self.v_ref * dt
            y_r = self.row_width
            theta_r = math.pi  # 180도 (서쪽을 바라봄)
            
        # Phase 4: 위로 직진 (North, +Y) - 왼쪽 끝에서 위로 이동
        else:
            dt = t_cycle - (2 * t_horz + t_vert)
            x_r = 0.0
            y_r = self.row_width + self.v_ref * dt
            theta_r = math.pi / 2.0  # 90도 (위쪽을 바라봄)

        # 전체 Y 오프셋 적용
        y_r += y_offset_base
        
        return x_r, y_r, theta_r

    def control_loop(self):
        if not hasattr(self, 'start_time'):
            self.start_time = self.get_clock().now()
            
        # 1. Reference Generation
        # (실제로는 Waypoint 로직을 구현해야 하지만, 테스트용으로 동적 타겟 생성)
        now_sec = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        x_r, y_r, theta_r = self.get_reference_state(now_sec)
        
        # 2. Position Error Calculation (Eq 21)
        # Global frame error -> Local frame error transformation
        dx = x_r - self.x
        dy = y_r - self.y
        dtheta = theta_r - self.theta
        
        # Wrap theta error to [-pi, pi]
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        
        xe = math.cos(self.theta) * dx + math.sin(self.theta) * dy
        ye = -math.sin(self.theta) * dx + math.cos(self.theta) * dy
        theta_e = dtheta
        
        # 3. Global Control Law (Eq 25, 26)
        # v_A: Robot Linear Velocity Command
        # omega_A: Robot Angular Velocity Command
        
        v_A = self.v_ref * math.cos(theta_e) + self.kx * xe
        # omega_A = self.omega_ref + (1.0/self.ky) * ye * self.v_ref + self.k_theta * math.sin(theta_e)

        omega_R = 0.0 # Reference yaw rate is approximated as 0 or derivative of theta_r
        # 회전 구간에서 Feedforward 항 추가 (omega_ref)
        # 간단한 구현을 위해 여기서는 피드백 항에 의존하거나, 
        # get_reference_state에서 omega_r도 리턴하여 사용하는 것이 더 정확함.
        # 현 코드에서는 위치 에러 기반 보정만으로 추종.
        
        omega_A = (1.0/self.ky) * ye * self.v_ref + self.k_theta * math.sin(theta_e) + omega_R
        
        # 4. Inverse Kinematics (Mapping to 4 Wheels)
        # Calculate Front Steering Angle (Eq 8)
        # Handling singularity when v_A is close to 0
        if abs(v_A) < 1e-3:
            delta_f = 0.0
        else:
            delta_f = math.atan((omega_A * self.Lf) / v_A)
            
        # Saturate delta_f if needed (Paper limits to +/- 60 deg)
        max_steer = math.radians(60)
        delta_f = max(min(delta_f, max_steer), -max_steer)
        
        # Calculate ICR Radius R (Eq 10)
        # Handle singularity when delta_f is 0 (Straight motion)
        if abs(delta_f) < 1e-3:
            R = float('inf')
        else:
            R = self.Lf / math.tan(delta_f)
            
        # Calculate Individual Steering Angles (Eq 11-14)
        # delta_1 (FL), delta_2 (FR), delta_3 (RL), delta_4 (RR)
        if R == float('inf'):
            d1 = d2 = d3 = d4 = 0.0
        else:
            d1 = math.atan(self.Lf / (R - self.W/2.0))
            d2 = math.atan(self.Lf / (R + self.W/2.0))
            d3 = -math.atan(self.Lr / (R + self.W/2.0))
            d4 = -math.atan(self.Lr / (R - self.W/2.0))
            
        # Calculate Individual Wheel Velocities (Eq 15-18 -> 20)
        if R == float('inf'):
            v1 = v2 = v3 = v4 = v_A
        else:
            # cos(delta)가 0이 되는 경우는 기구적으로 제한되므로 안전하다고 가정
            v1 = v_A * (R - self.W/2.0) / (R * math.cos(d1))
            v2 = v_A * (R + self.W/2.0) / (R * math.cos(d2))
            v3 = v_A * (R + self.W/2.0) / (R * math.cos(d3))
            v4 = v_A * (R - self.W/2.0) / (R * math.cos(d4))

        # Linear velocity to Angular velocity
        rad1 = v1 / self.radius
        rad2 = v2 / self.radius
        rad3 = v3 / self.radius
        rad4 = v4 / self.radius

        # 5. Publish Commands
        # Drive (Velocity)
        msg_drive = Float64MultiArray()
        msg_drive.data = [float(rad1), float(rad2), float(rad3), float(rad4)]
        self.pub_drive.publish(msg_drive)
        
        # Steering (Position)
        msg_steer = Float64MultiArray()
        msg_steer.data = [float(d1), float(d2), float(d3), float(d4)]
        self.pub_steer.publish(msg_steer)

def main(args=None):
    rclpy.init(args=args)
    node = LyapunovKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()