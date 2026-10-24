#!/usr/bin/env python3

import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SimpleScanExplorer(Node):
    def __init__(self) -> None:
        super().__init__("simple_scan_explorer")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_explore")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("scan_timeout", 0.8)
        self.declare_parameter("forward_speed", 0.16)
        self.declare_parameter("turn_speed", 0.65)
        self.declare_parameter("steer_gain", 0.7)
        self.declare_parameter("stop_distance", 0.42)
        self.declare_parameter("clear_distance", 0.75)
        self.declare_parameter("front_angle_deg", 28.0)
        self.declare_parameter("side_angle_deg", 70.0)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.steer_gain = float(self.get_parameter("steer_gain").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.clear_distance = float(self.get_parameter("clear_distance").value)
        self.front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.side_angle = math.radians(float(self.get_parameter("side_angle_deg").value))

        rate = float(self.get_parameter("publish_rate").value)
        self.latest_scan: LaserScan | None = None
        self.latest_scan_time = self.get_clock().now()

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_timer(1.0 / max(rate, 1.0), self.tick)

        self.get_logger().info(
            f"Exploring from {self.scan_topic} to {self.cmd_vel_topic}; "
            "keep a hand near the stop script."
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

    @staticmethod
    def valid_ranges(values: Iterable[float], range_min: float, range_max: float) -> list[float]:
        valid = []
        for value in values:
            if math.isfinite(value) and range_min < value < range_max:
                valid.append(float(value))
        return valid

    def sector_min(self, scan: LaserScan, start_angle: float, end_angle: float) -> float:
        if scan.angle_increment == 0.0:
            return scan.range_max

        start = max(min(start_angle, end_angle), scan.angle_min)
        end = min(max(start_angle, end_angle), scan.angle_max)
        if start >= end:
            return scan.range_max

        first = max(0, int((start - scan.angle_min) / scan.angle_increment))
        last = min(len(scan.ranges), int((end - scan.angle_min) / scan.angle_increment) + 1)
        values = self.valid_ranges(scan.ranges[first:last], scan.range_min, scan.range_max)
        if not values:
            return scan.range_max
        return min(values)

    def make_command(self, scan: LaserScan) -> Twist:
        front = self.sector_min(scan, -self.front_angle, self.front_angle)
        left = self.sector_min(scan, self.front_angle, self.side_angle)
        right = self.sector_min(scan, -self.side_angle, -self.front_angle)

        cmd = Twist()
        if front < self.stop_distance:
            cmd.angular.z = self.turn_speed if left >= right else -self.turn_speed
            return cmd

        cmd.linear.x = self.forward_speed
        if front < self.clear_distance:
            cmd.linear.x *= 0.55

        balance = max(min(left - right, 1.0), -1.0)
        cmd.angular.z = max(min(self.steer_gain * balance, self.turn_speed), -self.turn_speed)
        return cmd

    def tick(self) -> None:
        cmd = Twist()
        if self.latest_scan is not None:
            age = (self.get_clock().now() - self.latest_scan_time).nanoseconds / 1e9
            if age <= self.scan_timeout:
                cmd = self.make_command(self.latest_scan)
        self.publisher.publish(cmd)

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())


def main() -> None:
    rclpy.init()
    node = SimpleScanExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
