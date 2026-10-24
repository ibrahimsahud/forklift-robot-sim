#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


DEFAULT_TOPICS = [
    "/cmd_vel_keyboard",
    "/cmd_vel_explore",
    "/cmd_vel_joy",
    "/cmd_vel_tracker",
    "/cmd_vel",
    "/diff_cont/cmd_vel_unstamped",
]


class StopRobot(Node):
    def __init__(self) -> None:
        super().__init__("stop_robot")
        self.declare_parameter("duration", 1.5)
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("topics", DEFAULT_TOPICS)

        self.duration = float(self.get_parameter("duration").value)
        self.rate = float(self.get_parameter("rate").value)
        self.topics = list(self.get_parameter("topics").value)
        self.publishers = [self.create_publisher(Twist, topic, 10) for topic in self.topics]
        self.stop_msg = Twist()

    def run(self) -> None:
        end_time = time.monotonic() + max(self.duration, 0.1)
        sleep_sec = 1.0 / max(self.rate, 1.0)
        self.get_logger().info(f"Publishing stop commands for {self.duration:.1f}s")
        while rclpy.ok() and time.monotonic() < end_time:
            for publisher in self.publishers:
                publisher.publish(self.stop_msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(sleep_sec)

        for publisher in self.publishers:
            publisher.publish(self.stop_msg)
        self.get_logger().info("Stop command sent.")


def main() -> None:
    rclpy.init()
    node = StopRobot()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
