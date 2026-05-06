#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

class FourWSOdometry(Node):
    def __init__(self):
        super().__init__('fws_odometry',
                         parameter_overrides=[
                             Parameter('use_sim_time', Parameter.Type.BOOL, True)
                         ])

        self.L_half = 0.215
        self.W_half = 0.215
        self.wheel_radius = 0.0695

        # === 변수 초기화 ===
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.steering_names = [
            'drive1_axis_joint', 'drive2_axis_joint', 
            'drive3_axis_joint', 'drive4_axis_joint'
        ]
        self.driving_names = [
            'motor1_axis_joint', 'motor2_axis_joint', 
            'motor3_axis_joint', 'motor4_axis_joint'
        ]
        
        # 현재 바퀴 상태 저장용 [FL, FR, RL, RR]
        self.steering_angles = [0.0] * 4
        self.wheel_velocities = [0.0] * 4 

        # 바퀴 위치 좌표 [FL, FR, RL, RR]
        self.wheel_positions = [
            ( self.L_half, self.W_half),
            ( self.L_half, -self.W_half),
            (-self.L_half, self.W_half),
            (-self.L_half, -self.W_half)
        ]

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        self.get_logger().info("4WS Odometry Node Started")

    def joint_state_callback(self, msg):
        # 1. Joint State 메시지 파싱 (안전장치 추가)
        for i in range(4):
            # --- (1) Steering Angle (Position) 읽기 ---
            try:
                # 해당 조인트 이름이 메시지에 있는지 확인
                if self.steering_names[i] in msg.name:
                    idx_s = msg.name.index(self.steering_names[i])
                    # 인덱스가 실제 데이터 배열 길이보다 작은지 확인 (IndexError 방지)
                    if idx_s < len(msg.position):
                        val = msg.position[idx_s]
                        # nan 체크
                        self.steering_angles[i] = val if not math.isnan(val) else 0.0
            except ValueError:
                pass 

            # --- (2) Wheel Velocity (Velocity, [rad/s]) 읽기 ---
            try:
                if self.driving_names[i] in msg.name:
                    idx_d = msg.name.index(self.driving_names[i])
                    # [핵심 수정] velocity 배열 길이가 인덱스보다 큰지 확인
                    if idx_d < len(msg.velocity):
                        vel = msg.velocity[idx_d]
                        # nan 체크
                        self.wheel_velocities[i] = (vel if not math.isnan(vel) else 0.0) * self.wheel_radius
                    else:
                        # 이름은 있는데 속도 데이터가 없는 경우 (다른 노드가 보낸 메시지 무시)
                        pass
            except ValueError:
                pass

        # 2. Forward Kinematics 계산
        vx_robot, vy_robot, wz_robot = self.calculate_body_velocity()

        # 3. Odometry 적분 및 발행
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # 시간이 너무 짧으면(0초) 계산 건너뛰기
        if dt < 0.0001:
            return
            
        self.last_time = current_time

        # 로봇 좌표계 속도 -> 월드 좌표계 변환
        delta_x = (vx_robot * math.cos(self.th) - vy_robot * math.sin(self.th)) * dt
        delta_y = (vx_robot * math.sin(self.th) + vy_robot * math.cos(self.th)) * dt
        delta_th = wz_robot * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 4. Publish
        self.publish_odometry(current_time, vx_robot, vy_robot, wz_robot)


    def calculate_body_velocity(self):
        v_x_sum = 0.0
        v_y_sum = 0.0
        
        valid_wheels = 0

        for i in range(4):
            v_wheel = self.wheel_velocities[i]
            delta = self.steering_angles[i]
            rx, ry = self.wheel_positions[i]

            # 바퀴의 속도 벡터 분해 (Global이 아닌 Robot Frame 기준)
            vx_i = v_wheel * math.cos(delta)
            vy_i = v_wheel * math.sin(delta)

            # 로봇 전체 속도 기여분 합산
            v_x_sum += vx_i
            v_y_sum += vy_i
            
            # 각속도 추정: v_tangential / radius
            # v_y_i = v_robot_y + w * rx
            # v_x_i = v_robot_x - w * ry
            # 이를 이용하여 w 성분을 역산해서 평균냄 (단순화된 모델)
            # 여기서는 각각의 기여도를 더하는 방식보다는
            # vx, vy 평균을 먼저 구하고 나머지를 회전으로 간주하는 것이 안정적임
            valid_wheels += 1

        if valid_wheels == 0:
            return 0.0, 0.0, 0.0

        # 평균 선속도
        vx_robot = v_x_sum / 4.0
        vy_robot = v_y_sum / 4.0

        # 각속도 별도 계산 (평균 선속도를 뺀 나머지 성분으로 계산)
        w_estimates = 0.0
        for i in range(4):
            v_wheel = self.wheel_velocities[i]
            delta = self.steering_angles[i]
            rx, ry = self.wheel_positions[i]
            
            vx_i = v_wheel * math.cos(delta)
            vy_i = v_wheel * math.sin(delta)
            
            # (rx, ry) 위치에서의 접선 속도 성분을 이용해 w 추정
            # w = (vy_local * rx - vx_local * ry) / (rx^2 + ry^2)
            # where vx_local = vx_i - vx_robot, vy_local = vy_i - vy_robot
            
            vx_local = vx_i - vx_robot
            vy_local = vy_i - vy_robot
            
            r2 = rx**2 + ry**2
            if r2 > 0.001:
                w_estimates += (vy_local * rx - vx_local * ry) / r2

        wz_robot = w_estimates / 4.0

        return vx_robot, vy_robot, wz_robot

    def publish_odometry(self, current_time, vx, vy, wz):
        # Quaternion 변환
        q = self.euler_to_quaternion(0, 0, self.th)

        # 1. TF Broadcast (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint' # 혹은 base_link
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 2. Odom Message Publish
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint' # 혹은 base_link
        
        # 위치 (Pose)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # 속도 (Twist) - base_link 기준
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = FourWSOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()