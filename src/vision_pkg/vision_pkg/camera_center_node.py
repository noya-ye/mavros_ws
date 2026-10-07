import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

from vision_pkg.detector import detect_frame, draw_detection


class CameraCenterNode(Node):
    def __init__(self):
        super().__init__("camera_center_node")

        # =========================
        # 参数
        # =========================
        self.declare_parameter("camera_device", "/dev/video2")
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("camera_fps", 10.0)

        # 是否在本机使用 OpenCV 窗口显示
        self.declare_parameter("display", False)

        # 是否发布 debug 图
        self.declare_parameter("publish_debug", False)

        # 中心点话题
        self.declare_parameter("circle_topic", "/target/circle_center")
        self.declare_parameter("contour_topic", "/target/contour_center")
        self.declare_parameter("red_cross_topic", "/target/red_cross_center")

        # Debug 图话题
        self.declare_parameter("debug_topic", "/target/debug_image")

        # =========================
        # 读取参数
        # =========================
        camera_device = self.get_parameter("camera_device").value
        frame_width = self.get_parameter("frame_width").value
        frame_height = self.get_parameter("frame_height").value
        camera_fps = self.get_parameter("camera_fps").value

        circle_topic = self.get_parameter("circle_topic").value
        contour_topic = self.get_parameter("contour_topic").value
        red_cross_topic = self.get_parameter("red_cross_topic").value
        debug_topic = self.get_parameter("debug_topic").value

        self.display = self.get_parameter("display").value
        self.publish_debug = self.get_parameter("publish_debug").value

        # =========================
        # 打开摄像头
        # =========================
        backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY

        self.capture = cv2.VideoCapture(camera_device, backend)

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.capture.set(cv2.CAP_PROP_FPS, camera_fps)
        self.capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG")
        )

        if not self.capture.isOpened():
            raise RuntimeError(f"无法打开摄像头: {camera_device}")

        # =========================
        # 中心点 Publisher
        # =========================
        self.circle_publisher = self.create_publisher(
            Point,
            circle_topic,
            10
        )

        self.contour_publisher = self.create_publisher(
            Point,
            contour_topic,
            10
        )

        self.red_cross_publisher = self.create_publisher(
            Point,
            red_cross_topic,
            10
        )

        # =========================
        # Debug 图 Publisher
        # =========================
        self.bridge = CvBridge()

        self.debug_publisher = None

        if self.publish_debug:
            self.debug_publisher = self.create_publisher(
                Image,
                debug_topic,
                10
            )

        # =========================
        # Timer
        # =========================
        timer_period = 1.0 / camera_fps if camera_fps > 0 else 0.1

        self.timer = self.create_timer(
            timer_period,
            self.process_frame
        )

        self.failed_reads = 0

        self.get_logger().info(
            f"camera={camera_device}, "
            f"circle_topic={circle_topic}, "
            f"contour_topic={contour_topic}, "
            f"publish_debug={self.publish_debug}, "
            f"debug_topic={debug_topic}"
        )

    @staticmethod
    def make_point(center, image_shape):
        """
        将图像像素坐标转换为以图像中心为原点的坐标。

        图像坐标:
            x -> 右
            y -> 下

        输出坐标:
            message.x -> 上为正
            message.y -> 左为正
        """

        height, width = image_shape[:2]

        message = Point()

        message.x = float(
            height / 2.0 - center[1]
        )

        message.y = float(
            width / 2.0 - center[0]
        )

        message.z = 0.0

        return message

    def process_frame(self):
        # =========================
        # 读取摄像头
        # =========================
        ret, frame = self.capture.read()

        if not ret:
            self.failed_reads += 1

            if self.failed_reads == 1 or self.failed_reads % 30 == 0:
                self.get_logger().warning(
                    f"摄像头读取失败，连续失败次数: {self.failed_reads}"
                )

            return

        self.failed_reads = 0

        # =========================
        # 图像检测
        # =========================
        detection = detect_frame(frame)

        # =========================
        # 发布外轮廓中心
        # =========================
        if detection.contour_center is not None:

            contour_point = self.make_point(
                detection.contour_center,
                frame.shape
            )

            self.contour_publisher.publish(
                contour_point
            )

        # =========================
        # 发布圆心
        # =========================
        if detection.circle_center is not None:

            circle_point = self.make_point(
                detection.circle_center,
                frame.shape
            )

            self.circle_publisher.publish(
                circle_point
            )

        if detection.red_cross_center is not None:
            red_cross_point = self.make_point(
                detection.red_cross_center,
                frame.shape
            )
            self.red_cross_publisher.publish(red_cross_point)

        # =========================
        # Debug 图
        # =========================
        #
        # 只有以下任意一个条件成立时才生成 debug 图：
        #
        # 1. publish_debug = True
        # 2. display = True
        #
        # 避免无意义地每帧画图，减少 CPU 开销。
        #
        debug_image = None

        if self.publish_debug or self.display:
            debug_image = draw_detection(
                frame,
                detection
            )

        # =========================
        # 发布 Debug ROS Image
        # =========================
        if self.publish_debug and self.debug_publisher is not None:

            debug_msg = self.bridge.cv2_to_imgmsg(
                debug_image,
                encoding="bgr8"
            )

            debug_msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            debug_msg.header.frame_id = "camera"

            self.debug_publisher.publish(
                debug_msg
            )

        # =========================
        # OpenCV 本地显示
        # =========================
        if self.display:

            cv2.imshow(
                "ROS 2 Camera Center Detection",
                debug_image
            )

            cv2.waitKey(1)

    def destroy_node(self):
        # 释放摄像头
        self.capture.release()

        # 关闭 OpenCV 窗口
        if self.display:
            cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = CameraCenterNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
