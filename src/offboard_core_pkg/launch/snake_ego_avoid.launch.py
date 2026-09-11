import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('offboard_core_pkg'),
        'config',
        'snake_ego_avoid.yaml',
    )

    return LaunchDescription([
        Node(
            package='offboard_core_pkg',
            executable='snake_ego_avoid_node',
            name='snake_ego_avoid_node',
            output='screen',
            parameters=[config],
        ),
    ])
