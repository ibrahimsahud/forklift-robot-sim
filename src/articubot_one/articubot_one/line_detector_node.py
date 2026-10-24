#!/usr/bin/env python3

import math

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from articubot_one.line_detection import detect_line


class LineDetector(Node):
    def __init__(self) -> None:
        super().__init__("line_detector")

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("line_offset_topic", "/line_offset")
        self.declare_parameter("line_angle_topic", "/line_angle")
        self.declare_parameter("debug_image_topic", "/line_detection/debug_image")
        self.declare_parameter("dark_threshold", 90)
        self.declare_parameter("min_area", 250.0)
        self.declare_parameter("publish_nan_when_lost", True)

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.line_offset_topic = str(self.get_parameter("line_offset_topic").value)
        self.line_angle_topic = str(self.get_parameter("line_angle_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.dark_threshold = int(self.get_parameter("dark_threshold").value)
        self.min_area = float(self.get_parameter("min_area").value)
        self.publish_nan_when_lost = bool(
            self.get_parameter("publish_nan_when_lost").value
        )

        self.bridge = CvBridge()
        self.offset_pub = self.create_publisher(Float32, self.line_offset_topic, 10)
        self.angle_pub = self.create_publisher(Float32, self.line_angle_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Line detector subscribed to {self.camera_topic}; publishing "
            f"{self.line_offset_topic}, {self.line_angle_topic}, and {self.debug_image_topic}"
        )

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert bottom camera image: {exc}")
            return

        detection, debug = detect_line(
            frame,
            dark_threshold=self.dark_threshold,
            min_area=self.min_area,
        )

        if detection.detected:
            self.publish_float(self.offset_pub, detection.offset)
            self.publish_float(self.angle_pub, detection.angle)
        elif self.publish_nan_when_lost:
            self.publish_float(self.offset_pub, math.nan)
            self.publish_float(self.angle_pub, math.nan)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert line debug image: {exc}")
            return

        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

    @staticmethod
    def publish_float(publisher, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
