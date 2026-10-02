from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('offboard_core_pkg'), 'config', 'offboard_core.yaml')
    return LaunchDescription([
        Node(
            package='offboard_core_pkg',
            executable='offboard_core_node',
            name='offboard_core_node',
            output='screen',
            parameters=[config],
        )
    ])
