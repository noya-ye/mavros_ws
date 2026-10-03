import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    offboard_share = get_package_share_directory('offboard_core_pkg')
    door_share = get_package_share_directory('door_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='door_navigation',
            executable='door_node',
            name='door_navigation',
            output='screen',
            parameters=[
                os.path.join(door_share, 'config', 'preview.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
        Node(
            package='offboard_core_pkg',
            executable='door_navigation_node',
            name='door_navigation_node',
            output='screen',
            parameters=[
                os.path.join(offboard_share, 'config', 'door_navigation.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
