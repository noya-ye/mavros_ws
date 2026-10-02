from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="vision_pkg",
            executable="camera_center_node",
            name="camera_center_node",
            output="screen",
            parameters=[{
                "camera_device": "/dev/video0",
                "frame_width": 1280,
                "frame_height": 720,
                "camera_fps": 30.0,

                # 是否本机弹出 OpenCV 窗口
                "display": False,

                # 是否发布 Debug 图像
                "publish_debug": True,

                # 中心点话题
                "circle_topic": "/target/circle_center",
                "contour_topic": "/target/contour_center",

                # Debug 图像话题
                "debug_topic": "/target/debug_image",
            }],
        )
    ])