#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_relay")
        self.publisher = self.create_publisher(Odometry, "/odom", 50)
        self.subscription = self.create_subscription(
            Odometry, "/diff_cont/odom", self.odom_cb, 50
        )
        self.get_logger().info("Relaying /diff_cont/odom -> /odom")

    def odom_cb(self, msg: Odometry) -> None:
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
