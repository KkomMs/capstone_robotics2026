# stm32 serial bridge + mobile robot node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import xacro
# ydlidar_launch 실행에 필요한 것들
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_launch_setup),
    ])

def _launch_setup(context, *args, **kwargs):
    pkg_share   = FindPackageShare('fwis_robot')
    config      = PathJoinSubstitution([pkg_share, 'config'])
    urdf        = PathJoinSubstitution([pkg_share, 'urdf'])
    params_yaml = PathJoinSubstitution([config, 'robot_params.yaml'])
    urdf_path   = PathJoinSubstitution([urdf, 'fwis_position_velocity.xacro.urdf'])
    urdf_str    = urdf_path.perform(context)
    urdf_xml    = xacro.process_file(urdf_str).toxml()
    rviz_cfg    = PathJoinSubstitution([config, 'robot_rviz.rviz'])
    ekf_params_yaml = PathJoinSubstitution([config, 'ekf.yaml'])
    joy_params_yaml = PathJoinSubstitution([config, 'joystick.yaml'])

    # ─────────────────────────────────────────────────────────────────────────────
    # parameters
    # ─────────────────────────────────────────────────────────────────────────────
    params_path = params_yaml.perform(context)
    with open(params_path, 'r') as f:
        y = yaml.safe_load(f) or {}

    mobile_params = (y.get('mobile_robot_node') or {}).get('ros__parameters') or {}
    if not mobile_params:
        raise RuntimeError(f"[robot_bringup.launch] '{params_path}'에 "
                           f"mobile_robot_node.ros__parameters가 없습니다.")
    
    # camera_params = (y.get('camera_node') or {}).get('ros__parameters') or {}
    # if not camera_params:
    #     raise RuntimeError(f"[robot_bringup.launch] '{params_path}'에 "
    #                        f"camera_node.ros__parameters가 없습니다.")

    imu_params = (y.get('imu_node') or {}).get('ros__parameters') or {}
    if not imu_params:
        raise RuntimeError(f"[robot_bringup.launch] '{params_path}'에 "
                           f"imu_node.ross__parameters가 없습니다.")
    
    # ─────────────────────────────────────────────────────────────────────────────
    # ydlidar_launch
    # ─────────────────────────────────────────────────────────────────────────────
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ydlidar_ros2_driver'),
            '/launch/ydlidar_launch.py'
        ])
    )
    # ─────────────────────────────────────────────────────────────────────────────
    # STM32 serial bridge
    # ─────────────────────────────────────────────────────────────────────────────
    stm32_bridge = Node(
        package='fwis_robot',
        executable='stm32_bridge',
        name='stm32_bridge',
        output='screen',
    )
    # ─────────────────────────────────────────────────────────────────────────────
    # Controller
    # ─────────────────────────────────────────────────────────────────────────────
    mobile_node = Node(
        package="fwis_robot",
        executable="mobile_robot_node",
        name="mobile_robot_node",
        output="screen",
        parameters=[mobile_params],
        remappings=[('odom', 'wheel_odom')],
    )
    # ─────────────────────────────────────────────────────────────────────────────
    # Cameras
    # ─────────────────────────────────────────────────────────────────────────────
    # camera_node = Node(
    #     package='fwis_robot',
    #     executable="camera_node",
    #     name="camera_node",
    #     output="screen",
    #     parameters=[camera_params],
    # )
    # ─────────────────────────────────────────────────────────────────────────────
    # IMU
    # ─────────────────────────────────────────────────────────────────────────────
    imu_node = Node(
        package="fwis_robot",
        executable="imu_node",
        name="imu_node",
        output='screen',
        parameters=[imu_params],
    )
    # ─────────────────────────────────────────────────────────────────────────────
    # Robot state Publisher Node (base_footprint -> 각 joints TF)
    # ─────────────────────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_xml,
            'publish_frequency': 50.0,
        }],
    )
    # ─────────────────────────────────────────────────────────────────────────────
    # localization
    # ─────────────────────────────────────────────────────────────────────────────
    ekf_robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_yaml],
        remappings=[('odometry/filtered', 'odom')]
    )
    # ─────────────────────────────────────────────────────────────────────────────
    # Joystick
    # ─────────────────────────────────────────────────────────────────────────────
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 1.0,
            'autorepeat_rate': 50.0
        }],
    )
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_params_yaml],
    )
    
    # ─────────────────────────────────────────────────────────────────────────────
    # rviz
    # ─────────────────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    return [ydlidar_launch,
            stm32_bridge,
            mobile_node,
            #camera_node,
            imu_node,
            robot_state_publisher,
            ekf_robot_localization_node,
            #joy_node,
            #teleop_twist_joy_node,
            rviz,
            ]