import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('offboard_core_pkg'),
        'config',
        'align_drop_snake_ego.yaml',
    )

    return LaunchDescription([
        Node(
            package='offboard_core_pkg',
            executable='align_drop_snake_ego_node',
            name='align_drop_snake_ego_node',
            output='screen',
            parameters=[config],
        ),
    ])
