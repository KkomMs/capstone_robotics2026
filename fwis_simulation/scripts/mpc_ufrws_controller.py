#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import math
from scipy.optimize import minimize

class MpcUFRWSNode(Node):
    def __init__(self):
        super().__init__('mpc_ufrws_ctrl')

        # --- Robot Parameters ---
        self.Lf = 0.215         # Front length [m]
        self.Lr = 0.215         # Rear length [m]
        self.L = self.Lf + self.Lr
        self.W = 0.43           # Width [m]
        self.radius = 0.0695    # Wheel Radius [m]

        # --- MPC Parameters ---
        self.V_ref = 0.3    # 목표 주행 속도 [m/s]
        self.dt = 0.1       # 제어 주기 [10Hz]
        self.N = 10         # 예측 구간
        self.max_steer = math.radians(180)   # 최대 조향각 [radian]

        # Tuning Parameters
        self.Q_y = 200.0        # 횡방향 오차 가중치
        self.Q_phi = 5.0       # heading angle 오차 가중치
        self.R_u = 0.1          # 조향각 크기 가중치
        self.R_delta = 5.0     # 조향각 변화량(제어 증분) 가중치

        # --- State Parameters ---
        self.state = np.array([0.0, 0.0, 0.0])      # [x, y, theta]
        self.u0 = np.zeros(self.N * 2)              # 최적화 솔버 초기값

        # --- Trajectory Parameters ---
        self.lane_length = 2.0
        self.row_width = 3.2

        # --- ROS2 Interfaces ---
        # Publishers
        self.pub_steer = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_drive = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        
        # Subscriber
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.start_time = None
        self.get_logger().info("MPC-UFRWS Controller Started.")

    def odom_callback(self, msg):
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Orientation (Quaternion to Euler Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        self.state = np.array([x, y, theta])

    def get_ref(self, t):
        # 기존 지그재그 경로 생성 로직
        t_horz = self.lane_length / self.V_ref
        t_vert = self.row_width / self.V_ref
        cycle_time = 2 * t_horz + 2 * t_vert
        
        cycle_count = int(t / cycle_time)
        t_cycle = t % cycle_time
        y_offset_base = cycle_count * (2 * self.row_width)
        
        if t_cycle < t_horz: 
            x_r, y_r, theta_r = self.V_ref * t_cycle, 0.0, 0.0
        elif t_cycle < (t_horz + t_vert):
            x_r, y_r, theta_r = self.lane_length, self.V_ref * (t_cycle - t_horz), math.pi / 2.0
        elif t_cycle < (2 * t_horz + t_vert):
            x_r, y_r, theta_r = self.lane_length - self.V_ref * (t_cycle - (t_horz + t_vert)), self.row_width, math.pi
        else:
            x_r, y_r, theta_r = 0.0, self.row_width + self.V_ref * (t_cycle - (2 * t_horz + t_vert)), math.pi / 2.0

        return x_r, y_r + y_offset_base, theta_r
    
    def ufrws_model(self, state, u):
        # Eq [6]
        X, Y, phi = state       # state vector
        delta_f, delta_r = u    # control input

        X_next = X + self.V_ref * math.cos(phi + (delta_f + delta_r) / 2.0) * self.dt
        Y_next = Y + self.V_ref * math.sin(phi + (delta_f + delta_r) / 2.0) * self.dt
        phi_next = phi + (self.V_ref * math.cos((delta_f + delta_r) / 2.0) / (self.Lf + self.Lr)) * (delta_f - delta_r) * self.dt

        return np.array([X_next, Y_next, phi_next])
    
    def mpc_cost(self, u_seq, current_state, target_seq):
        cost = 0.0
        state = current_state
        u_seq = u_seq.reshape((self.N, 2))
        prev_u = np.array([0.0, 0.0])

        for i in range(self.N):
            u = u_seq[i]
            state = self.ufrws_model(state, u)

            target_x, target_y, target_phi = target_seq[i]

            # Global frame error -> Local frame error transformation
            dx = target_x - state[0]
            dy = target_y - state[1]

            e_y = -math.sin(state[2]) * dx + math.cos(state[2]) * dy
            e_phi = target_phi - state[2]
            # 각도 오차 정규화 (-pi ~ pi)
            e_phi = (e_phi + math.pi) % (2 * math.pi) - math.pi

            cost += self.Q_y * e_y**2 + self.Q_phi * e_phi**2
            cost += self.R_u * (u[0]**2 + u[1]**2)
            cost += self.R_delta * ((u[0] - prev_u[0])**2 + (u[1] - prev_u[1])**2)

            prev_u = u

        return cost
    
    def control_loop(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            return
        
        now_sec = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # 1. 미래 N 스텝의 타겟 시퀀스 생성
        target_seq = []
        #x_pred = self.state[0] # 로봇의 현재 X 좌표에서 출발

        for i in range(self.N):
            pred_time = now_sec + (i * self.dt)
            target_seq.append(self.get_ref(pred_time))

        # 2. MPC 최적화 수행
        bounds = [(-self.max_steer, self.max_steer)] * (self.N * 2)

        # SciPy minimize를 이용한 최적화 (SLSQP 알고리즘 사용)
        res = minimize(self.mpc_cost, self.u0, args=(self.state, target_seq), bounds=bounds, method='SLSQP', options={'maxiter': 20})

        # 최적화된 제어 입력 중 첫 번째 스텝의 값만 사용
        optimal_u = res.x.reshape((self.N, 2))[0]
        
        # 다음 스텝의 최적화 연산 속도 향상을 위해 초기값 시프트 (Warm start)
        self.u0 = np.roll(res.x, -2)
        self.u0[-2:] = 0.0

        delta_f, delta_r = optimal_u[0], optimal_u[1]

        # 3. UFRWS 모델 기구학
        tan_df = math.tan(delta_f)
        tan_dr = math.tan(delta_r)

        denom_diff = (self.W / (2.0 * self.L)) * (tan_df - tan_dr)

        # 각 바퀴 조향각 [FL, FR, RL, RR]
        d1 = math.atan(tan_df / (1.0 - denom_diff)) if abs(1.0 - denom_diff) > 1e-5 else 0.0
        d2 = math.atan(tan_df / (1.0 + denom_diff)) if abs(1.0 + denom_diff) > 1e-5 else 0.0
        d3 = math.atan(tan_dr / (1.0 - denom_diff)) if abs(1.0 - denom_diff) > 1e-5 else 0.0
        d4 = math.atan(tan_dr / (1.0 + denom_diff)) if abs(1.0 + denom_diff) > 1e-5 else 0.0

        # 4. 각 바퀴 속도 (모든 바퀴가 근사적으로 목표 속도 v_ref 낸다고 가정)
        # 회전 반경 R을 구해 개별 바퀴 속도 제어 가능
        rad_speed = self.V_ref / self.radius

        # 5. Publish Commands
        # Drive (Velocity)
        msg_drive = Float64MultiArray()
        msg_drive.data = [float(rad_speed)] * 4
        self.pub_drive.publish(msg_drive)
        
        # Steering (Position)
        msg_steer = Float64MultiArray()
        msg_steer.data = [float(d1), float(d2), float(d3), float(d4)]
        self.pub_steer.publish(msg_steer)

def main(args=None):
    rclpy.init(args=args)
    node = MpcUFRWSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()