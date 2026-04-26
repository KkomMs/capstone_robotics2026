import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('fwis_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            pkg_dir,
            'map',
            'test_room.yaml'))      # 이름 바꾸기

    param_file_name = 'params_nav2.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            pkg_dir,
            'param',
            param_file_name))
    
    bt_xml_file_name = 'my_nav2_replanning_and_recovery.xml'
    bt_xml_dir = LaunchConfiguration(
        'default_bt_xml_filename',
        default=os.path.join(
            pkg_dir,
            'config',
            bt_xml_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('fwis_navigation'),
        'rviz',
        'rviz_nav2.rviz')
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=bt_xml_dir,
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'default_bt_xml_filename': bt_xml_dir}.items(),
        ),
        rviz_node,
    ])