from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('offboard_core_pkg'),
        'config', 'corridor_door.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        Node(
            package='offboard_core_pkg',
            executable='corridor_door_node',
            name='corridor_door_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]),
    ])
