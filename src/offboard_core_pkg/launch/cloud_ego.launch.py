import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)


def generate_launch_description():
    cloud_filter_launch = os.path.join(
        get_package_share_directory('cloud_self_filter_pkg'),
        'launch',
        'cloud_self_filter.launch.py',
    )
    ego_planner_launch = os.path.join(
        get_package_share_directory('ego_2d_planner_pkg'),
        'launch',
        'ego_2d_planner.launch.xml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(
                get_package_share_directory('ego_2d_planner_pkg'),
                'config',
                'ego_2d_planner.yaml',
            ),
            description='Path to ego_2d_planner parameter file',
        ),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_traj_server', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('ego_2d_planner_pkg'),
                'rviz',
                'ego_2d_planner.rviz',
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cloud_filter_launch),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(ego_planner_launch),
            launch_arguments={
                'config': LaunchConfiguration('config'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_traj_server': LaunchConfiguration('use_traj_server'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items(),
        ),
    ])
