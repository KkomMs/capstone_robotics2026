from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')
    pkg_share = FindPackageShare('fwis_simulation')
    rviz_config_path = PathJoinSubstitution([
        pkg_share, 
        'rviz', 
        'display.rviz'
    ])
    world_path = PathJoinSubstitution([
        FindPackageShare('fwis_simulation'),
        'worlds',
        'sim_room.sdf'
    ])

    set_gazebo_resource = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_share, 'models'])
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('fwis_simulation'),
                 'urdf', 'fwis_effort.xacro.urdf']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('fwis_simulation'),
            'config',
            'fwis_controller_effort.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'fwis',
                #    '-x', '3.6',
                #    '-y', '3.3',
                   '-z', '0.2',
                #    '-Y', '-1.5708',
                   '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    forward_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_effort_controller',
            '--param-file',
            robot_controllers,
            ],
    )
    scanner_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'scanner_position_controller',
            '--param-file',
            robot_controllers,
            ],
    )
    odom_node = Node(
        package='fwis_simulation',
        executable='odom_publisher.py',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            "/scanner1/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/scanner1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/scanner2/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/scanner2/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            ],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_resource,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, ' -r -v 1 ', world_path, '', gz_args])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[forward_effort_controller_spawner,
                         scanner_position_controller_spawner],
            )
        ),
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        odom_node,
        #rviz_node,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])