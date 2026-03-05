#!/usr/bin/env python3
import rclpy
import math
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion


def yaw2quaternion(yaw):
    # z축 회전 각도(yaw) -> quaternion 변환
    q = Quaternion()

    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)

    return q


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Nav2 시스템이 켜질 때까지 대기
    navigator.waitUntilNav2Active()

    # 'ㄹ'자 웨이포인트 리스트 생성
    waypoints = []

    # heading 방향
    half_turn = 1.5708
    a_round = 3.14

    ## ============ waypoint ============ ##
    # 웨이포인트 1: initial point
    # wp1 = PoseStamped()
    # wp1.header.frame_id = 'map'
    # wp1.header.stamp = navigator.get_clock().now().to_msg()
    # wp1.pose.position.x = 3.6
    # wp1.pose.position.y = 3.3
    # wp1.pose.orientation = yaw2quaternion(-half_turn)
    # waypoints.append(wp1)

    # 웨이포인트 2: 1st rack
    wp2 = PoseStamped()
    wp2.header.frame_id = 'map'
    wp2.header.stamp = navigator.get_clock().now().to_msg()
    wp2.pose.position.x = 3.6
    wp2.pose.position.y = 0.2
    wp2.pose.orientation = yaw2quaternion(-half_turn)
    waypoints.append(wp2)

    # 웨이포인트 3: turn right
    # wp3 = PoseStamped()
    # wp3.header.frame_id = 'map'
    # wp3.header.stamp = navigator.get_clock().now().to_msg()
    # wp3.pose.position.x = 3.6
    # wp3.pose.position.y = 0.0
    # wp3.pose.orientation = yaw2quaternion(-a_round)
    # waypoints.append(wp3)

    # 웨이포인트 4: 1st rack end
    wp4 = PoseStamped()
    wp4.header.frame_id = 'map'
    wp4.header.stamp = navigator.get_clock().now().to_msg()
    wp4.pose.position.x = -3.2
    wp4.pose.position.y = 0.0
    wp4.pose.orientation = yaw2quaternion(-a_round)
    waypoints.append(wp4)

    # 웨이포인트 5: turn left
    # wp5 = PoseStamped()
    # wp5.header.frame_id = 'map'
    # wp5.header.stamp = navigator.get_clock().now().to_msg()
    # wp5.pose.position.x = -3.4
    # wp5.pose.position.y = 0.0
    # wp5.pose.orientation = yaw2quaternion(-half_turn)
    # waypoints.append(wp5)

    # 웨이포인트 6: 2nd rack
    wp6 = PoseStamped()
    wp6.header.frame_id = 'map'
    wp6.header.stamp = navigator.get_clock().now().to_msg()
    wp6.pose.position.x = -3.4
    wp6.pose.position.y = -2.4
    wp6.pose.orientation = yaw2quaternion(-half_turn)
    waypoints.append(wp6)

    # 웨이포인트 7: turn left
    # wp7 = PoseStamped()
    # wp7.header.frame_id = 'map'
    # wp7.header.stamp = navigator.get_clock().now().to_msg()
    # wp7.pose.position.x = -3.4
    # wp7.pose.position.y = -2.8
    # wp7.pose.orientation = yaw2quaternion(0.0)
    # waypoints.append(wp7)

    # 웨이포인트 8: 2nd rack end
    wp8 = PoseStamped()
    wp8.header.frame_id = 'map'
    wp8.header.stamp = navigator.get_clock().now().to_msg()
    wp8.pose.position.x = 3.2
    wp8.pose.position.y = -2.8
    wp8.pose.orientation = yaw2quaternion(0.0)
    waypoints.append(wp8)

    # 웨이포인트 9: turn left
    # wp9 = PoseStamped()
    # wp9.header.frame_id = 'map'
    # wp9.header.stamp = navigator.get_clock().now().to_msg()
    # wp9.pose.position.x = -3.4
    # wp9.pose.position.y = -2.8
    # wp9.pose.orientation = yaw2quaternion(half_turn)
    # waypoints.append(wp9)

    # 웨이포인트 10: go back to initial point
    wp10 = PoseStamped()
    wp10.header.frame_id = 'map'
    wp10.header.stamp = navigator.get_clock().now().to_msg()
    wp10.pose.position.x = 3.6
    wp10.pose.position.y = 3.3
    wp10.pose.orientation = yaw2quaternion(half_turn)
    waypoints.append(wp10)
    ## ================================== ##
    
    # FollowWaypoints 액션 실행 (웨이포인트 리스트 전송)
    print("Start Following Waypoints Node.")
    navigator.followWaypoints(waypoints)

    # 주행 상태 모니터링
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Currently moving to the {feedback.current_waypoint}th waypoint.")

    # 최종 결과 확인
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("All waypoints have been passed.")
    elif result == TaskResult.CANCELED:
        print("Waypoints following has been cancelled.")
    elif result == TaskResult.FAILED:
        print("Waypoints following failure.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()