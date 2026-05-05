#!/usr/bin/env python3
import rclpy
import math
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.parameter import Parameter


def yaw2quaternion(yaw):
    # z축 회전 각도(yaw) -> quaternion 변환
    q = Quaternion()

    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)

    return q

def make_pose(navigator: BasicNavigator,
              x: float, y: float, yaw: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = navigator.get_clock().now().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation = yaw2quaternion(yaw)
    
    return ps

def waypoints(navigator: BasicNavigator) -> list:
    half_turn = 1.5708      # 90도 회전
    a_round = 3.14          # 180도 회전

    poses = [
        # (x, y, yaw)
        # ( 3.4,  0.0,  -half_turn ),     # 1. 랙 중앙으로 진입
        # ( 3.4,  0.0,  -a_round ),       # 2. 제자리 회전
        # ( -3.26,  0.0,  -a_round ),     # 3. 랙 끝까지 주행
        # 테스트용
        ( -3.4,  0.0,  -half_turn ),
        ( -3.4,  0.0,  0.0 ),
        ( 3.3,  0.0,  0.0 ),
    ]

    return [make_pose(navigator, x, y, yaw) for x, y, yaw in poses]


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    # Nav2 시스템이 켜질 때까지 대기
    navigator.waitUntilNav2Active()
    time.sleep(3.0)

    # 웨이포인트 리스트 생성
    all_waypoints = waypoints(navigator)
    total_waypoints = len(all_waypoints)
    print(f'[Waypoints] 총 {total_waypoints}개의 waypoints 생성')

    # 주행 명령 전송
    navigator.followWaypoints(all_waypoints)

    prev_idx = -1

    # 주행 상태 모니터링
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()

        if feedback:
            current_idx = feedback.current_waypoint

            if prev_idx != -1 and current_idx > prev_idx:
                print(f"  -> [Waypoint {prev_idx + 1}] 도달 성공.")

            if current_idx < total_waypoints:
                current_pose = all_waypoints[current_idx]
                target_x = current_pose.pose.position.x
                target_y = current_pose.pose.position.y

                print(f"현재 {current_idx + 1}번째 waypoint로 이동 중. "
                      f"목표 좌표: (x={target_x:.2f}, y={target_y:.2f})")
            
            prev_idx = current_idx
            
        time.sleep(0.5)
    
    # 최종 결과 확인
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("[Waypoints] SUCCEEDED: 모든 waypoints 주행 완료.")
    elif result == TaskResult.CANCELED:
        print("[Waypoints] CANCELED: waypoints 주행 취소.")
    elif result == TaskResult.FAILED:
        print("[Waypoints] FAILED: waypoints 주행 실패.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()