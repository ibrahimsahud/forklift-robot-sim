#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Iterable

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class LidarObstacleGuard(Node):
    def __init__(self) -> None:
        super().__init__("lidar_obstacle_guard")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("safety_cmd_topic", "/cmd_vel_safety")
        self.declare_parameter("stop_topic", "/forklift_safety/stop")
        self.declare_parameter("status_topic", "/forklift_safety/status")
        self.declare_parameter("mission_state_topic", "/forklift_mission/state")
        self.declare_parameter("pickup_success_topic", "/pickup_success")
        self.declare_parameter(
            "safety_bypass_states",
            [
                "VERIFY_FORK_INSERTION",
                "PICK_LOAD",
                "VERIFY_PICKUP_SUCCESS",
                "PASS_GATE_CENTER",
                "GATE_CLEAR_AFTER",
            ],
        )
        self.declare_parameter(
            "carried_load_bypass_states",
            [
                "PICK_LOAD",
                "VERIFY_PICKUP_SUCCESS",
                "BACK_OUT_SHORT",
                "TURN_180_TO_NAV",
                "NAV_TO_GATE_WAIT",
                "WAIT_GATE_PERMISSION",
                "PASS_GATE_CENTER",
                "GATE_CLEAR_AFTER",
                "NAV_TO_DROPOFF_PREPOINT",
                "QR_DROPOFF_CONFIRM",
                "LINE_FOLLOW_DROPOFF",
                "TURN_180_TO_DROPOFF",
                "LINE_OR_ODOM_APPROACH_DROPOFF",
                "FINAL_ALIGN_DROPOFF",
                "DROP_LOAD",
                "VERIFY_DROPOFF_RELEASE",
                "BACK_OUT_DROPOFF",
                "TURN_180_TO_RETURN",
            ],
        )
        self.declare_parameter("front_angle_deg", 28.0)
        self.declare_parameter("stop_distance_m", 0.70)
        self.declare_parameter("clear_distance_m", 0.90)
        self.declare_parameter("min_obstacle_range_m", 0.25)
        self.declare_parameter("scan_timeout_sec", 0.8)
        self.declare_parameter("control_hz", 20.0)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.stop_distance = float(self.get_parameter("stop_distance_m").value)
        self.clear_distance = float(self.get_parameter("clear_distance_m").value)
        self.min_obstacle_range = float(self.get_parameter("min_obstacle_range_m").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self.safety_bypass_states = {
            str(state).strip()
            for state in self.get_parameter("safety_bypass_states").value
            if str(state).strip()
        }
        self.carried_load_bypass_states = {
            str(state).strip()
            for state in self.get_parameter("carried_load_bypass_states").value
            if str(state).strip()
        }

        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("safety_cmd_topic").value),
            10,
        )
        self.stop_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("stop_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            self.mission_state_cb,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("pickup_success_topic").value),
            self.pickup_success_cb,
            10,
        )

        control_hz = float(self.get_parameter("control_hz").value)
        self.create_timer(1.0 / max(control_hz, 1.0), self.control_loop)

        self.front_min = math.inf
        self.last_scan_time = None
        self.blocked = False
        self.last_status = ""
        self.current_mission_state = ""
        self.carrying_load = False

        self.get_logger().info(
            "LiDAR obstacle guard active: "
            f"front=+/-{math.degrees(self.front_angle):.1f} deg, "
            f"stop={self.stop_distance:.2f} m, clear={self.clear_distance:.2f} m"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def scan_cb(self, msg: LaserScan) -> None:
        self.front_min = self.sector_min(msg, -self.front_angle, self.front_angle)
        self.last_scan_time = self.now_sec()

    def mission_state_cb(self, msg: String) -> None:
        self.current_mission_state = msg.data.split(maxsplit=1)[0] if msg.data else ""

    def pickup_success_cb(self, msg: Bool) -> None:
        self.carrying_load = bool(msg.data)

    def sector_min(self, scan: LaserScan, start_angle: float, end_angle: float) -> float:
        if scan.angle_increment == 0.0:
            return math.inf

        start = max(min(start_angle, end_angle), scan.angle_min)
        end = min(max(start_angle, end_angle), scan.angle_max)
        if end <= start:
            return math.inf

        first = max(0, int((start - scan.angle_min) / scan.angle_increment))
        last = min(len(scan.ranges), int((end - scan.angle_min) / scan.angle_increment) + 1)
        values = self.valid_ranges(scan.ranges[first:last], scan.range_min, scan.range_max)
        return min(values) if values else math.inf

    def valid_ranges(
        self,
        ranges: Iterable[float],
        range_min: float,
        range_max: float,
    ) -> list[float]:
        lower = max(range_min, self.min_obstacle_range)
        return [
            float(value)
            for value in ranges
            if math.isfinite(value) and lower <= value <= range_max
        ]

    def control_loop(self) -> None:
        bypassed = self.current_mission_state in self.safety_bypass_states or (
            self.carrying_load
            and self.current_mission_state in self.carried_load_bypass_states
        )
        if bypassed:
            self.set_blocked(False, f"BYPASS: mission_state={self.current_mission_state}")
            return

        if self.last_scan_time is None or self.now_sec() - self.last_scan_time > self.scan_timeout:
            self.set_blocked(True, "STOP: scan stale")
            return

        if self.blocked:
            if self.front_min >= self.clear_distance:
                self.set_blocked(False, f"CLEAR: front_min={self.front_min:.2f} m")
            else:
                self.publish_stop(True, f"STOP: front_min={self.front_min:.2f} m")
            return

        if self.front_min <= self.stop_distance:
            self.set_blocked(True, f"STOP: front_min={self.front_min:.2f} m")
        else:
            self.publish_stop(False, f"CLEAR: front_min={self.front_min:.2f} m")

    def set_blocked(self, blocked: bool, status: str) -> None:
        self.blocked = blocked
        self.publish_stop(blocked, status)

    def publish_stop(self, stop: bool, status: str) -> None:
        stop_msg = Bool()
        stop_msg.data = bool(stop)
        self.stop_pub.publish(stop_msg)

        if stop:
            self.cmd_pub.publish(Twist())

        if status != self.last_status:
            self.last_status = status
            status_msg = String()
            status_msg.data = status
            self.status_pub.publish(status_msg)
            self.get_logger().info(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarObstacleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
