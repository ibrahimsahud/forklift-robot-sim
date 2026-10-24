#!/usr/bin/env python3
"""
Node to republish /scan with corrected frame_id
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('target_frame', 'laser_frame')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10)

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)

        self.get_logger().info(f'Republishing {input_topic} to {output_topic} with frame_id={self.target_frame}')

    def scan_callback(self, msg):
        # Create new message with corrected frame_id
        new_msg = LaserScan()
        new_msg.header.stamp = msg.header.stamp
        new_msg.header.frame_id = self.target_frame
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = msg.ranges
        new_msg.intensities = msg.intensities

        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
