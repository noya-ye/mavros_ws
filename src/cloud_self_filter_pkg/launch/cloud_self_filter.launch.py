from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('cloud_self_filter_pkg')
    cfg = os.path.join(pkg_share, 'config', 'cloud_self_filter.yaml')

    return LaunchDescription([
        Node(
            package='cloud_self_filter_pkg',
            executable='cloud_self_filter_node',
            name='cloud_self_filter_node',
            output='screen',
            parameters=[cfg]
        )
    ])
