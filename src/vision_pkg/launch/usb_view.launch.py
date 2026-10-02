from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="usb_camera",
            output="screen",
            parameters=[{
                "video_device": "/dev/video0",   # 按实际设备号修改
                "image_size": [1280, 720],       # [宽, 高]
                "pixel_format": "YUYV",          # 或 "MJPG"（更高帧率时用）
                "time_per_frame": [1, 30],       # 30 FPS
                "camera_frame_id": "camera_link",
            }],
            remappings=[
                ("image_raw", "/camera/image_raw"),
                ("camera_info", "/camera/camera_info"),
            ],
        )
    ])