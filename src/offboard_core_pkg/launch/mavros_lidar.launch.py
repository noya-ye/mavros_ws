import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    livox_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py'],
        output='screen',
    )

    fastlio2_launch_file = os.path.join(
        get_package_share_directory('fastlio2'), 'launch', 'lio_launch.py')
    fastlio2_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(fastlio2_launch_file)),
        ],
    )

    lidar_to_px4_bridge = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='offboard_core_pkg',
                executable='lidar_to_px4_bridge',
                name='lidar_to_px4_bridge',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        livox_launch,
        fastlio2_launch,
        lidar_to_px4_bridge,
    ])
