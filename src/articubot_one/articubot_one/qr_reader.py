#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from articubot_one.qr_detection import detect_qr_codes, draw_debug_image

VALID_QR_VALUES = {
    "START",
    "A1",
    "A2",
    "A3",
    "GATE_OUT",
    "GATE_IN",
    "B1",
    "B2",
    "B3",
}

class QRReader(Node):
    def __init__(self):
        super().__init__("qr_reader")

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("qr_data_topic", "/qr_code")
        self.declare_parameter("legacy_qr_data_topic", "/qr_data")
        self.declare_parameter("debug_image_topic", "/qr_detection/debug_image")
        self.declare_parameter("legacy_debug_image_topic", "/qr_debug_image")
        self.declare_parameter("min_qr_width_px", 20)
        self.declare_parameter("max_qr_width_px", 1000)
        self.declare_parameter("publish_duplicates_period_sec", 1.0)
        self.declare_parameter("enable_preprocessing", True)
        self.declare_parameter("debug_logging", False)
        self.declare_parameter("debug_invalid_qr", False)

        self.camera_topic = self.get_parameter("camera_topic").value
        self.qr_data_topic = self.get_parameter("qr_data_topic").value
        self.legacy_qr_data_topic = self.get_parameter("legacy_qr_data_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.legacy_debug_image_topic = self.get_parameter("legacy_debug_image_topic").value
        self.min_qr_width_px = float(self.get_parameter("min_qr_width_px").value)
        self.max_qr_width_px = float(self.get_parameter("max_qr_width_px").value)
        self.publish_duplicates_period_sec = float(
            self.get_parameter("publish_duplicates_period_sec").value
        )
        self.enable_preprocessing = bool(self.get_parameter("enable_preprocessing").value)
        self.debug_logging = bool(self.get_parameter("debug_logging").value)
        self.debug_invalid_qr = bool(self.get_parameter("debug_invalid_qr").value)

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        qr_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.qr_data_publisher = self.create_publisher(String, self.qr_data_topic, qr_qos)
        self.legacy_qr_data_publisher = None
        if self.legacy_qr_data_topic:
            self.legacy_qr_data_publisher = self.create_publisher(
                String,
                self.legacy_qr_data_topic,
                qr_qos,
            )
        self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic, 10)
        self.legacy_debug_image_publisher = None
        if self.legacy_debug_image_topic:
            self.legacy_debug_image_publisher = self.create_publisher(
                Image,
                self.legacy_debug_image_topic,
                10,
            )
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.last_data = None
        self.last_publish_time = None
        self.logged_resolution = False

        self.get_logger().info(
            f"QR reader subscribed to {self.camera_topic}, publishing "
            f"{self.qr_data_topic} and {self.debug_image_topic}"
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert camera image: {exc}")
            return

        height, width = frame.shape[:2]
        if not self.logged_resolution:
            self.get_logger().info(f"Camera image resolution: {width}x{height}")
            self.logged_resolution = True

        detections = detect_qr_codes(
            frame,
            enable_preprocessing=self.enable_preprocessing,
            detector=self.detector,
        )
        debug_frame = draw_debug_image(
            frame,
            detections,
            self.min_qr_width_px,
            self.max_qr_width_px,
        )

        for detection in detections:
            valid_width = (
                self.min_qr_width_px <= detection.width_px <= self.max_qr_width_px
            )
            valid_value = detection.data in VALID_QR_VALUES
            error_x = detection.error_x(width)

            if not valid_width:
                if self.debug_logging:
                    self.get_logger().info(
                        f"Ignoring QR outside width filter: {detection.data}, "
                        f"width px: {detection.width_px:.1f}"
                )
                continue

            if not valid_value:
                if self.debug_invalid_qr or self.debug_logging:
                    self.get_logger().warn(
                        f"Ignoring invalid QR value: {detection.data}"
                    )
                continue

            if self.debug_logging:
                self.get_logger().info(f"Detected QR: {detection.data}")
                self.get_logger().info(
                    f"center pixel: {detection.cx:.1f}, {detection.cy:.1f}"
                )
                self.get_logger().info(f"width px: {detection.width_px:.1f}")
                self.get_logger().info(f"normalized error_x: {error_x:.3f}")

            self.publish_qr_data(detection.data)

        self.publish_debug_image(debug_frame, msg.header)

    def publish_qr_data(self, data):
        now = self.get_clock().now()
        should_publish = data != self.last_data

        if self.last_publish_time is None:
            should_publish = True
        elif self.publish_duplicates_period_sec <= 0.0:
            should_publish = True
        else:
            elapsed_ns = (now - self.last_publish_time).nanoseconds
            period_ns = int(self.publish_duplicates_period_sec * 1_000_000_000)
            if elapsed_ns >= period_ns:
                should_publish = True

        if not should_publish:
            return

        qr_msg = String()
        qr_msg.data = data
        self.qr_data_publisher.publish(qr_msg)
        if self.legacy_qr_data_publisher is not None:
            self.legacy_qr_data_publisher.publish(qr_msg)

        self.last_data = data
        self.last_publish_time = now

    def publish_debug_image(self, frame, header):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert debug image: {exc}")
            return

        debug_msg.header = header
        self.debug_image_publisher.publish(debug_msg)
        if self.legacy_debug_image_publisher is not None:
            self.legacy_debug_image_publisher.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QRReader()
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
