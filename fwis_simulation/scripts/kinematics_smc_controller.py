#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.time import Time

import numpy as np
import math

# Calculate dynamic matrices
class DynamicsCalculator:
    def __init__(self):
        # Parameters
        self.a = 0.215  # coordinate [m]
        self.b = 0.215  # coordinate [m]
        self.r = 0.0695 # wheel radius [m]
        self.mb = 13.0  # robot body mass[kg]
        self.mw = 0.66  # wheel mass [kg]
        self.Iz = 0.007   # vehicle inertia moment [kgm^2]
        self.I_phi = 0.001  # wheel rolling inertia moment [kgm^2]
        self.I_delta = 0.001 # wheel steering inertia moment [kgm^2]
        
        self.m = self.mb + 4 * self.mw
        self.K = 4 * (self.a**2 + self.b**2)
        self.I = self.Iz + 4 * self.mw * (self.a**2 + self.b**2)
        
        # wheel coordinate
        self.wheel_pos = [
            (self.a, self.b),  # 1: FR
            (-self.a, self.b),   # 2: FL
            (-self.a, -self.b),  # 3: RL
            (self.a, -self.b)   # 4: RR
        ]

    def compute_matrices(self, v_curr, delta_curr):
        # v_curr: [v1~4, d_delta1~4]
        v_wheels = v_curr[:4]
        d_deltas = v_curr[4:]
        
        c = np.cos(delta_curr)
        s = np.sin(delta_curr)
        
        W = np.zeros(4)
        W_dot = np.zeros(4)
        
        for i in range(4):
            x_i, y_i = self.wheel_pos[i]
            W[i] = (-y_i * c[i] + x_i * s[i]) / self.K
            W_dot[i] = d_deltas[i] * (y_i * s[i] + x_i * c[i]) / self.K
            
        omega = np.sum(v_wheels * W)
        
        M_bar = np.zeros((8, 8))
        Vm_bar = np.zeros((8, 8))
        N_bar = np.zeros((8, 8))
        
        # M_bar Calculation
        for i in range(4):
            for j in range(4):
                if i == j:
                    M_bar[i, j] = self.I * (W[i]**2) + (self.m / 16.0) + (self.I_phi / self.r**2)
                else:
                    cos_diff = np.cos(delta_curr[j] - delta_curr[i])
                    M_bar[i, j] = self.I * W[j] * W[i] + (self.m / 16.0) * cos_diff
        for i in range(4, 8):
            M_bar[i, i] = self.I_delta

        # Vm_bar Calculation
        for i in range(4):
            for j in range(4):
                sin_diff = np.sin(delta_curr[j] - delta_curr[i])
                term2 = (self.m / 16.0) * sin_diff * (d_deltas[i] + omega)
                Vm_bar[i, j] = self.I * W[j] * W_dot[i] + term2

        # N_bar Calculation
        for i in range(4):
            for j in range(4):
                x_j, y_j = self.wheel_pos[j]
                term1 = (W[i] * (y_j * c[j] + x_j * s[j])) / self.r
                if i == j:
                    N_bar[i, j] = term1 + (1.0 / (4.0 * self.r)) + (1.0/self.r)
                else:
                    # 계산 식 다시 확인해보기
                    cos_diff = np.cos(delta_curr[i] - delta_curr[j])
                    N_bar[i, j] = term1 + (cos_diff / (4.0 * self.r))
        for i in range(4, 8):
            N_bar[i, i] = 1.0

        return M_bar, Vm_bar, N_bar

# Controller node
class KinematicSMCNode(Node):
    def __init__(self):
        super().__init__('trajectory_ctrl')
        
        # Control Gains
        self.kx = 2.0
        self.ky = 2.0
        self.k_theta = 1.5
        self.k_delta = 2.5
        self.lam = np.diag([2.5] * 8)
        self.K_smc = np.diag([5.0] * 8)
        self.epsilon = 0.1
        self.dt = 0.01  # 100Hz

        # State variables
        self.robot_q = np.zeros(3)  # [x, y, theta]
        self.robot_v = np.zeros(8)  # [v1~4, d_delta1~4]
        self.robot_delta = np.zeros(4) # [delta1~4]
        
        self.prev_v_cmd = np.zeros(8)
        self.error_int = np.zeros(8)
        self.start_time = self.get_clock().now()

        # 도구 초기화
        self.dynamics = DynamicsCalculator()

        # Publishers
        self.pub_torque = self.create_publisher(Float64MultiArray, '/forward_effort_controller/commands', 10)

        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_joint = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Kinematic-SMC Controller Started.")

    def odom_callback(self, msg):
        # Quaternion -> Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_q = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def joint_callback(self, msg):

        user_map = {
            'drive': ['motor1_axis_joint', 'motor2_axis_joint', 'motor3_axis_joint', 'motor4_axis_joint'],
            'steer': ['drive1_axis_joint', 'drive2_axis_joint', 'drive3_axis_joint', 'drive4_axis_joint']
        }

        # Map User Index (0:FR, 1:FL, 2:RL, 3:RR) to Internal Index (0:FL, 1:RL, 2:RR, 3:FR)
        # FR (User 0) -> Internal 3
        # FL (User 1) -> Internal 0
        # RL (User 2) -> Internal 1
        # RR (User 3) -> Internal 2
        map_u2i = {0: 3, 1: 0, 2: 1, 3: 2}

        try:
            for user_i in range(4):
                internal_i = map_u2i[user_i]
                
                # Steering Angle
                idx_s = msg.name.index(user_map['steer'][user_i])
                self.robot_delta[internal_i] = msg.position[idx_s]
                
                # Steering Rate
                self.robot_v[4+internal_i] = msg.velocity[idx_s]
                
                # Wheel Velocity
                idx_d = msg.name.index(user_map['drive'][user_i])
                self.robot_v[internal_i] = msg.velocity[idx_d] * self.dynamics.r
        except ValueError:
            pass

    def get_trajectory(self, t):
        # 원형 궤적
        # Xd = 3 cos(0.5t), Yd = 3 sin(0.5t)
        omega = 0.5
        xd = 3.0 * math.cos(omega * t)
        yd = 3.0 * math.sin(omega * t)
        
        vxd = -1.5 * math.sin(omega * t)
        vyd = 1.5 * math.cos(omega * t)
        
        theta_d = math.atan2(vyd, vxd)
        # theta_d의 미분값(omega_d) 계산 필요 (여기서는 근사 혹은 수식 미분)
        omega_d = omega # 원운동이므로 각속도 일정
        
        return np.array([xd, yd, theta_d]), np.array([vxd, vyd, omega_d])

    def saturate(self, s):
        return np.array([x/self.epsilon if abs(x) <= self.epsilon else np.sign(x) for x in s])

    def control_loop(self):
        # Create Timer and Trajectory
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        q_d, vel_d = self.get_trajectory(t) # Desired
        
        # Kinematic Control
        # Control Law
        # 1. wheel velocity and steer angle of ith wheel
        # - v_ic = z1 * sqrt(a_i^2 + b_i^2)
        # - delta_ic = z2 + arctan2(b_i, a_i)
        # 2. steering yaw rate of ith wheel
        # - delta_i_dot = delta_ic_dot + k_delta * (delta_ic - delta_i)
        # Tracking Errors
        xe = np.cos(self.robot_q[2]) * (q_d[0] - self.robot_q[0]) + np.sin(self.robot_q[2]) * (q_d[1] - self.robot_q[1])
        ye = -np.sin(self.robot_q[2]) * (q_d[0] - self.robot_q[0]) + np.cos(self.robot_q[2]) * (q_d[1] - self.robot_q[1])
        theta_e = q_d[2] - self.robot_q[2]
        # Angle wrapping
        theta_e = (theta_e + np.pi) % (2 * np.pi) - np.pi

        v_ic_list = []
        delta_target_list = []
        
        for i in range(4):
            xw, yw = self.dynamics.wheel_pos[i]
            vx_id = vel_d[0] - yw * vel_d[2]
            vy_id = vel_d[1] + xw * vel_d[2]
            
            v_id = math.sqrt(vx_id**2 + vy_id**2)
            delta_id = math.atan2(vy_id, vx_id)
            
            # Kinematic Control Laws
            a_i = v_id * math.cos(delta_id) + self.kx * xe - self.k_theta * yw * theta_e
            b_i = v_id * math.sin(delta_id) + self.ky * ye + self.k_theta * xw * theta_e
            
            current_delta = self.robot_delta[i]
            diff = delta_id - current_delta
            diff = (diff + np.pi) % (2 * np.pi) - np.pi
            
            if abs(diff) <= np.pi / 2:
                z1, z2 = 1.0, 0.0
            else:
                z1, z2 = -1.0, np.pi
                
            v_ic = z1 * math.sqrt(a_i**2 + b_i**2)
            delta_ic = z2 + math.atan2(b_i, a_i)
            
            v_ic_list.append(v_ic)
            delta_target_list.append(delta_ic)

        # Command Vector
        delta_dot_cmds = []
        for i in range(4):
            err = delta_target_list[i] - self.robot_delta[i]
            err = (err + np.pi) % (2 * np.pi) - np.pi # wrap
            cmd = self.k_delta * err
            delta_dot_cmds.append(cmd)
            
        v_cmd = np.array(v_ic_list + delta_dot_cmds)

        # Calculate acceleration and error
        dv_cmd = (v_cmd - self.prev_v_cmd) / self.dt
        self.prev_v_cmd = v_cmd
        
        error = v_cmd - self.robot_v
        self.error_int += error * self.dt
        self.error_int = np.clip(self.error_int, -2.0, 2.0) # Anti-windup

        # SMC
        M_bar, Vm_bar, N_bar = self.dynamics.compute_matrices(self.robot_v, self.robot_delta)
        
        # A, B Matrices
        try:
            M_inv = np.linalg.inv(M_bar)
        except:
            M_inv = np.linalg.pinv(M_bar)
            
        A = np.dot(M_inv, Vm_bar)
        B = np.dot(M_inv, N_bar)
        
        # Sliding Surface
        S = error + np.dot(self.lam, self.error_int)
        
        # Torque Calculation
        # tau = B^-1 * (dv_cmd + A*v + lambda*e + K*sat(S))
        term_dyn = dv_cmd + np.dot(A, self.robot_v) + np.dot(self.lam, error) + np.dot(self.K_smc, self.saturate(S))
        
        try:
            tau = np.linalg.solve(B, term_dyn)
        except:
            tau = np.dot(np.linalg.pinv(B), term_dyn)

        # --- Output Mapping (Internal -> User Order) ---
        # Internal Order: [FL, RL, RR, FR] (indices 0, 1, 2, 3)
        # User Order:     [FR, FL, RL, RR]
        # tau structure:  [drive_0..3, steer_0..3]

        # Drive Torques
        t_d = tau[:4]
        user_drive = [t_d[3], t_d[0], t_d[1], t_d[2]] # FR, FL, RL, RR
        
        # Steering Torques
        t_s = tau[4:]
        user_steer = [t_s[3], t_s[0], t_s[1], t_s[2]] # FR, FL, RL, RR
        
        # Combine (Order must match Controller YAML joints list)
        # Assuming YAML list is: [motor1, motor2, motor3, motor4, drive1, drive2, drive3, drive4]
        # which corresponds to [Drive_FR, Drive_FL, Drive_RL, Drive_RR, Steer_FR, Steer_FL, Steer_RL, Steer_RR]
        
        final_cmd = user_drive + user_steer
        
        msg = Float64MultiArray()
        msg.data = [float(x) for x in final_cmd]
        self.pub_torque.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicSMCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()