#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class PointRecorder(Node):
    def __init__(self) -> None:
        super().__init__('point_recorder')
        self.create_subscription(PointStamped, '/clicked_point', self.on_point, 10)
        self.get_logger().info('Listening for RViz Publish Point clicks on /clicked_point')

    def on_point(self, msg: PointStamped) -> None:
        point = msg.point
        print(
            f'name: TODO, x: {point.x:.3f}, y: {point.y:.3f}, z: {point.z:.3f}',
            flush=True,
        )


def main() -> None:
    rclpy.init()
    node = PointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
