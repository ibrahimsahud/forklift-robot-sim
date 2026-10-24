#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class SaveCameraFrame(Node):
    def __init__(self):
        super().__init__("save_camera_frame")
        self.camera_topic = "/camera/image_raw"
        self.output_path = "/tmp/qr_test.png"
        self.bridge = CvBridge()
        self.saved = False

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Waiting for one frame on {self.camera_topic}")

    def image_callback(self, msg):
        if self.saved:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert camera image: {exc}")
            return

        if not cv2.imwrite(self.output_path, frame):
            self.get_logger().error(f"Failed to save frame to {self.output_path}")
            return

        self.saved = True
        self.get_logger().info(f"Saved camera frame to {self.output_path}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SaveCameraFrame()
        while rclpy.ok() and not node.saved:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
