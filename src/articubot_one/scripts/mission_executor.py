#!/usr/bin/env python3
"""Deprecated simple Nav2 mission runner.

The active TEKNOFEST scenario path is now:
    ros2 run articubot_one route_executor --route A1_TO_B3 --execute

This file is kept only as historical reference for the old saved-map point
runner and is intentionally no longer installed by CMake.
"""

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    import yaml
except ImportError:
    yaml = None


REQUIRED_FIELDS = ("x", "y", "z", "yaw")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a pickup/gate/drop Nav2 mission.")
    parser.add_argument("--pickup", required=True, type=int, choices=(1, 2, 3))
    parser.add_argument("--drop", required=True, type=int, choices=(1, 2, 3))
    return parser.parse_args()


def mission_points_path() -> Path:
    return Path(get_package_share_directory("articubot_one")) / "config" / "mission_points.yaml"


def parse_simple_points(text: str) -> dict[str, dict[str, float]]:
    points: dict[str, dict[str, float]] = {}
    current_name: str | None = None

    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if not line.startswith(" ") and line.endswith(":"):
            current_name = line[:-1].strip()
            points[current_name] = {}
            continue
        if current_name is None or ":" not in line:
            continue
        key, value = line.strip().split(":", 1)
        points[current_name][key.strip()] = float(value.strip())

    return points


def load_points(path: Path) -> dict[str, dict[str, float]]:
    text = path.read_text(encoding="utf-8")
    if yaml is not None:
        loaded = yaml.safe_load(text) or {}
    else:
        loaded = parse_simple_points(text)

    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a mapping of point names")

    points: dict[str, dict[str, float]] = {}
    for name, value in loaded.items():
        if not isinstance(value, dict):
            raise ValueError(f"Point '{name}' must be a mapping")
        missing = [field for field in REQUIRED_FIELDS if field not in value]
        if missing:
            raise ValueError(f"Point '{name}' is missing fields: {', '.join(missing)}")
        points[str(name)] = {field: float(value[field]) for field in REQUIRED_FIELDS}

    return points


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    half_yaw = yaw * 0.5
    return math.sin(half_yaw), math.cos(half_yaw)


class MissionExecutor(Node):
    def __init__(self, points: dict[str, dict[str, float]]) -> None:
        super().__init__("mission_executor")
        self.points = points
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def wait_for_nav2(self) -> None:
        while rclpy.ok() and not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for /navigate_to_pose action server...")

    def pose_from_point(self, point_name: str) -> PoseStamped:
        point = self.points[point_name]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point["x"]
        pose.pose.position.y = point["y"]
        pose.pose.position.z = point["z"]
        pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(point["yaw"])
        return pose

    def go_to(self, point_name: str) -> bool:
        pose = self.pose_from_point(point_name)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f"Navigating to {point_name}: "
            f"x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, "
            f"z={pose.pose.position.z:.3f}, yaw={self.points[point_name]['yaw']:.3f}"
        )

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected: {point_name}")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error(f"No result returned for goal: {point_name}")
            return False

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"Goal failed: {point_name}, status={result.status}")
            return False

        self.get_logger().info(f"Goal reached: {point_name}")
        return True


def run_mission(args: argparse.Namespace, points: dict[str, dict[str, float]]) -> int:
    sequence: list[tuple[str, int, str | None]] = [
        (f"pickup_{args.pickup}", 5, "PICKUP_DONE"),
        ("gate", 10, "GATE_PERMISSION_GRANTED"),
        (f"drop_{args.drop}", 5, "DROP_DONE"),
        ("start", 0, "MISSION_DONE"),
    ]

    required_names = [name for name, _, _ in sequence]
    missing = [name for name in required_names if name not in points]
    if missing:
        raise ValueError(f"Missing mission points: {', '.join(missing)}")

    rclpy.init()
    node = MissionExecutor(points)
    try:
        node.wait_for_nav2()
        for point_name, wait_seconds, done_text in sequence:
            if not node.go_to(point_name):
                node.get_logger().error("Mission stopped after failed goal.")
                return 1
            if wait_seconds > 0:
                time.sleep(wait_seconds)
            if done_text:
                print(done_text, flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    args = parse_args()
    path = mission_points_path()
    try:
        points = load_points(path)
        print(f"Loaded mission points: {path}", flush=True)
        return run_mission(args, points)
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError) as exc:
        print(f"mission_executor error: {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
