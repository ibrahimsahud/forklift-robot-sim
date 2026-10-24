#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from articubot_one.route_config import config_path, load_route_waypoints

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("PyYAML is required for gazebo_pose_line_controller") from exc


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GazeboPoseLineController(Node):
    def __init__(self) -> None:
        super().__init__(
            "gazebo_pose_line_controller",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.declare_parameter("line_segments_file", str(config_path("line_segments.yaml")))
        self.declare_parameter("route_waypoints_file", str(config_path("route_waypoints.yaml")))
        self.declare_parameter("approach_linear_speed", 0.10)
        self.declare_parameter("reverse_approach_linear_speed", 0.06)
        self.declare_parameter("dock_linear_speed", 0.06)
        self.declare_parameter("approach_max_angular", 1.00)
        self.declare_parameter("reverse_approach_max_angular", 0.80)
        self.declare_parameter("dock_max_angular", 0.80)
        self.declare_parameter("cross_track_gain", 1.60)
        self.declare_parameter("heading_gain", 1.10)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("segment_end_tolerance_m", 0.10)
        self.declare_parameter("dock_completion_distance_tolerance_m", 0.02)
        self.declare_parameter("dock_completion_cross_track_tolerance_m", 0.04)
        self.declare_parameter("dock_completion_heading_tolerance_rad", 0.05)
        self.declare_parameter("dock_completion_min_progress_ratio", 0.95)
        self.declare_parameter("line_approach_completion_distance_tolerance_m", 0.12)
        self.declare_parameter("line_approach_cross_track_tolerance_m", 0.15)
        self.declare_parameter("line_approach_heading_tolerance_rad", 0.20)
        self.declare_parameter("line_approach_min_progress_ratio", 0.95)
        self.declare_parameter(
            "debug_log_dir",
            os.path.join("/tmp", "tekno_robot_artifacts", "pose_line_logs"),
        )

        self.enabled = False
        self.mode = "OFF"
        self.active_segment = ""
        self.status_seq = 0
        self.debug_log_dir = Path(str(self.get_parameter("debug_log_dir").value))
        self.debug_log_dir.mkdir(parents=True, exist_ok=True)
        self.current_log_segment = ""
        self.current_log_path: Path | None = None
        self.segment_config_path = Path(self.get_parameter("line_segments_file").value)
        self.last_invalid_segment_report = ""

        self.waypoints = load_route_waypoints(Path(self.get_parameter("route_waypoints_file").value))["waypoints"]
        self.segments = self._load_segments(self.segment_config_path)
        self.log_loaded_segments()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_pose_line", 10)
        self.status_pub = self.create_publisher(String, "/pose_line/status", 10)
        self.create_subscription(Bool, "/pose_line/enable", self.enable_cb, 10)
        self.create_subscription(String, "/pose_line/mode", self.mode_cb, 10)
        self.create_subscription(String, "/pose_line/segment", self.segment_cb, 10)

        period = 1.0 / max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(period, self.control_tick)

    def _load_segments(self, path: Path) -> dict[str, dict[str, dict[str, str]]]:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(loaded, dict):
            raise ValueError(f"{path} must contain a top-level mapping")
        result: dict[str, dict[str, dict[str, str]]] = {
            "road_segments": {},
            "approach_segments": {},
            "dock_segments": {},
        }
        for section in result:
            raw_section = loaded.get(section, {})
            if not isinstance(raw_section, dict):
                raise ValueError(f"{path} field '{section}' must be a mapping")
            for name, segment in raw_section.items():
                if not isinstance(segment, dict) or "start" not in segment or "end" not in segment:
                    raise ValueError(f"Segment '{name}' in '{section}' must define start/end")
                start = str(segment["start"]).upper()
                end = str(segment["end"]).upper()
                if start not in self.waypoints or end not in self.waypoints:
                    raise ValueError(f"Segment '{name}' references unknown waypoint(s): {start}, {end}")
                result[section][str(name).upper()] = {"start": start, "end": end}
        return result

    def log_loaded_segments(self) -> None:
        for section_name in ("road_segments", "approach_segments", "dock_segments"):
            names = sorted(self.segments.get(section_name, {}))
            self.get_logger().info(
                f"Loaded {section_name} from {self.segment_config_path}: "
                + (", ".join(names) if names else "(none)")
            )

    def available_segment_names(self) -> list[str]:
        names: list[str] = []
        for section_name in ("road_segments", "approach_segments", "dock_segments"):
            names.extend(self.segments.get(section_name, {}).keys())
        return sorted(set(names))

    def report_invalid_segment(self) -> None:
        signature = f"{self.mode}:{self.active_segment}"
        if signature == self.last_invalid_segment_report:
            return
        self.last_invalid_segment_report = signature
        self.get_logger().error(
            "Invalid pose_line segment request: "
            f"requested_segment={self.active_segment or '(empty)'} "
            f"mode={self.mode} "
            f"config_path={self.segment_config_path} "
            f"available_segments={', '.join(self.available_segment_names()) or '(none)'}"
        )

    def enable_cb(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def mode_cb(self, msg: String) -> None:
        self.mode = msg.data.strip().upper() or "OFF"

    def segment_cb(self, msg: String) -> None:
        self.active_segment = msg.data.strip().upper()

    def reset_segment_log(self) -> None:
        self.current_log_segment = ""
        self.current_log_path = None

    def ensure_segment_log(self, segment_name: str) -> None:
        if self.current_log_segment == segment_name and self.current_log_path is not None:
            return
        self.current_log_segment = segment_name
        self.current_log_path = self.debug_log_dir / f"{segment_name}.csv"
        self.current_log_path.write_text(
            "time_sec,x,y,yaw,cross_track_error,heading_error,progress,remaining,linear_cmd,angular_cmd\n",
            encoding="utf-8",
        )

    def append_segment_log(
        self,
        *,
        segment_name: str,
        pose: tuple[float, float, float],
        cross_track_error: float,
        heading_error: float,
        progress: float,
        remaining: float,
        linear_cmd: float,
        angular_cmd: float,
    ) -> None:
        self.ensure_segment_log(segment_name)
        if self.current_log_path is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        with self.current_log_path.open("a", encoding="utf-8") as handle:
            handle.write(
                f"{now:.6f},{pose[0]:.6f},{pose[1]:.6f},{pose[2]:.6f},"
                f"{cross_track_error:.6f},{heading_error:.6f},{progress:.6f},{remaining:.6f},"
                f"{linear_cmd:.6f},{angular_cmd:.6f}\n"
            )

    def current_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        except Exception:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        return (float(t.x), float(t.y), quaternion_to_yaw(q.x, q.y, q.z, q.w))

    def lookup_segment(self) -> tuple[str, tuple[float, float, float], tuple[float, float, float]] | None:
        if self.mode in {"LINE_APPROACH", "LINE_REVERSE"}:
            segment = self.segments["approach_segments"].get(self.active_segment)
            if segment is None:
                segment = self.segments["road_segments"].get(self.active_segment)
        elif self.mode == "DOCKING":
            segment = self.segments["dock_segments"].get(self.active_segment)
        else:
            segment = None
        if segment is None:
            return None
        start = self.waypoints[segment["start"]]
        end = self.waypoints[segment["end"]]
        return (
            self.active_segment,
            (start["x"], start["y"], start["yaw"]),
            (end["x"], end["y"], end["yaw"]),
        )

    def publish_status(self, payload: dict[str, Any]) -> None:
        self.status_seq += 1
        payload["seq"] = self.status_seq
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.status_pub.publish(msg)

    def control_tick(self) -> None:
        twist = Twist()
        pose = self.current_pose()
        segment_info = self.lookup_segment()
        status: dict[str, Any] = {
            "enabled": self.enabled,
            "mode": self.mode,
            "segment": self.active_segment,
            "controller_source": "pose_line",
            "segment_valid": bool(segment_info is not None),
            "pose_available": bool(pose is not None),
            "cross_track_error_m": None,
            "heading_error_rad": None,
            "progress_m": None,
            "remaining_m": None,
            "progress_ratio": None,
            "segment_length_m": None,
            "linear_cmd": 0.0,
            "angular_cmd": 0.0,
            "completion_flag": False,
            "stop_reason": "inactive",
            "active": False,
        }

        if not self.enabled:
            status["stop_reason"] = "disabled"
            self.reset_segment_log()
            self.publish_status(status)
            return

        if self.mode == "OFF":
            status["stop_reason"] = "mode_off"
            self.reset_segment_log()
            self.publish_status(status)
            return

        if pose is None:
            status["stop_reason"] = "pose_unavailable"
            self.reset_segment_log()
            self.publish_status(status)
            return

        if segment_info is None:
            status["stop_reason"] = "invalid_segment"
            self.report_invalid_segment()
            self.reset_segment_log()
            self.publish_status(status)
            return
        self.last_invalid_segment_report = ""

        _, start_pose, end_pose = segment_info
        dx = end_pose[0] - start_pose[0]
        dy = end_pose[1] - start_pose[1]
        seg_len = math.hypot(dx, dy)
        if seg_len <= 1e-6:
            status["error"] = "degenerate_segment"
            self.publish_status(status)
            return

        ux = dx / seg_len
        uy = dy / seg_len
        px = pose[0] - start_pose[0]
        py = pose[1] - start_pose[1]
        along = px * ux + py * uy
        cross = px * (-uy) + py * ux
        seg_heading = math.atan2(dy, dx)
        desired_heading = seg_heading
        if self.mode == "LINE_REVERSE":
            desired_heading = normalize_angle(seg_heading + math.pi)
        if self.mode == "DOCKING" and math.isfinite(end_pose[2]):
            # Docking waypoints encode the base yaw we want to hold while following the line.
            desired_heading = end_pose[2]
        heading_error = normalize_angle(desired_heading - pose[2])
        remaining = math.hypot(end_pose[0] - pose[0], end_pose[1] - pose[1])
        progress_ratio = along / seg_len

        if self.mode == "DOCKING":
            # Forks trail the base_link x axis on this robot, so docking must drive in reverse
            # while holding the dock waypoint yaw.
            linear_speed = -abs(float(self.get_parameter("dock_linear_speed").value))
            max_angular = float(self.get_parameter("dock_max_angular").value)
        elif self.mode == "LINE_REVERSE":
            # Carry the pallet through narrow spaces without forcing a 180-degree turn in place.
            linear_speed = -abs(float(self.get_parameter("reverse_approach_linear_speed").value))
            max_angular = float(self.get_parameter("reverse_approach_max_angular").value)
        else:
            linear_speed = float(self.get_parameter("approach_linear_speed").value)
            max_angular = float(self.get_parameter("approach_max_angular").value)

        k_cross = float(self.get_parameter("cross_track_gain").value)
        k_heading = float(self.get_parameter("heading_gain").value)
        end_tol = float(self.get_parameter("segment_end_tolerance_m").value)
        dock_end_tol = float(self.get_parameter("dock_completion_distance_tolerance_m").value)
        dock_cross_tol = float(self.get_parameter("dock_completion_cross_track_tolerance_m").value)
        dock_heading_tol = float(self.get_parameter("dock_completion_heading_tolerance_rad").value)
        dock_min_progress = float(self.get_parameter("dock_completion_min_progress_ratio").value)
        line_approach_end_tol = float(self.get_parameter("line_approach_completion_distance_tolerance_m").value)
        line_approach_cross_tol = float(self.get_parameter("line_approach_cross_track_tolerance_m").value)
        line_approach_heading_tol = float(self.get_parameter("line_approach_heading_tolerance_rad").value)
        line_approach_min_progress = float(self.get_parameter("line_approach_min_progress_ratio").value)

        # Positive cross means the robot sits left of the segment heading; steer right to rejoin.
        # Negative cross means the robot sits right of the segment heading; steer left to rejoin.
        angular = (-k_cross * cross) + (k_heading * heading_error)
        angular = max(-max_angular, min(max_angular, angular))

        line_approach_complete = bool(
            self.mode in {"LINE_APPROACH", "LINE_REVERSE"}
            and progress_ratio >= line_approach_min_progress
            and abs(cross) <= line_approach_cross_tol
            and abs(heading_error) <= line_approach_heading_tol
            and remaining <= line_approach_end_tol
        )
        docking_complete = bool(
            self.mode == "DOCKING"
            and progress_ratio >= dock_min_progress
            and abs(cross) <= dock_cross_tol
            and abs(heading_error) <= dock_heading_tol
            and remaining <= dock_end_tol
        )

        if line_approach_complete or docking_complete:
            linear_speed = 0.0
            angular = 0.0
        elif abs(cross) > 0.45 and abs(heading_error) > 0.70:
            linear_speed *= 0.35
        elif abs(cross) > 0.25 or abs(heading_error) > 0.45:
            linear_speed *= 0.60

        twist.linear.x = linear_speed
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

        status.update(
            {
                "active": True,
                "segment_start": {"x": start_pose[0], "y": start_pose[1], "yaw": start_pose[2]},
                "segment_end": {"x": end_pose[0], "y": end_pose[1], "yaw": end_pose[2]},
                "pose_x": pose[0],
                "pose_y": pose[1],
                "pose_yaw": pose[2],
                "cross_track_error_m": cross,
                "heading_error_rad": heading_error,
                "desired_heading_rad": desired_heading,
                "progress_m": along,
                "remaining_m": remaining,
                "segment_length_m": seg_len,
                "progress_ratio": progress_ratio,
                "linear_cmd": twist.linear.x,
                "angular_cmd": twist.angular.z,
                "completion_flag": line_approach_complete or docking_complete,
                "stop_reason": (
                    "line_approach_complete"
                    if line_approach_complete
                    else "dock_geometry_complete"
                    if docking_complete
                    else "tracking_segment"
                ),
            }
        )
        self.append_segment_log(
            segment_name=self.active_segment,
            pose=pose,
            cross_track_error=cross,
            heading_error=heading_error,
            progress=along,
            remaining=remaining,
            linear_cmd=twist.linear.x,
            angular_cmd=twist.angular.z,
        )
        self.publish_status(status)


def main() -> int:
    rclpy.init()
    node = GazeboPoseLineController()
    try:
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
