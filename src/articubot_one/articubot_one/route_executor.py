#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float32, String
from simulation_interfaces.srv import GetEntityState
from tf2_ros import Buffer, TransformException, TransformListener

from articubot_one.route_config import config_path, is_dock_target, validate_route_configs

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("PyYAML is required for route executor docking geometry checks") from exc


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def pose_distance(a: tuple[float, float, float] | None, b: tuple[float, float, float] | None) -> float | None:
    if a is None or b is None:
        return None
    return math.hypot(a[0] - b[0], a[1] - b[1])


def segment_progress(
    start: tuple[float, float, float] | None,
    target: tuple[float, float, float] | None,
    pose: tuple[float, float, float] | None,
) -> dict[str, float | None]:
    if start is None or target is None or pose is None:
        return {
            "segment_length_m": None,
            "along_track_m": None,
            "cross_track_m": None,
            "remaining_m": None,
        }
    dx = target[0] - start[0]
    dy = target[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return {
            "segment_length_m": 0.0,
            "along_track_m": 0.0,
            "cross_track_m": 0.0,
            "remaining_m": pose_distance(pose, target),
        }
    ux = dx / length
    uy = dy / length
    px = pose[0] - start[0]
    py = pose[1] - start[1]
    along = px * ux + py * uy
    cross = px * (-uy) + py * ux
    return {
        "segment_length_m": length,
        "along_track_m": along,
        "cross_track_m": cross,
        "remaining_m": pose_distance(pose, target),
    }


def parse_fork_state_text(state_text: str) -> dict[str, float | str | None]:
    parsed: dict[str, float | str | None] = {
        "raw": state_text,
        "state": None,
        "command": None,
        "height": None,
        "target": None,
    }
    if not state_text:
        return parsed
    upper = state_text.strip().upper()
    if upper:
        parsed["state"] = upper.split()[0]
    command_match = re.search(r"COMMAND=([^\s]+)", upper)
    height_match = re.search(r"HEIGHT=([-+]?\d*\.?\d+)", upper)
    target_match = re.search(r"TARGET=([-+]?\d*\.?\d+)", upper)
    if command_match:
        parsed["command"] = command_match.group(1)
    if height_match:
        parsed["height"] = float(height_match.group(1))
    if target_match:
        parsed["target"] = float(target_match.group(1))
    return parsed


class RouteExecutor(Node):
    def __init__(
        self,
        actions: list[dict[str, str]],
        waypoints: dict[str, dict[str, float]],
        *,
        route_name: str,
        dock_only_target: str | None,
        nav_only: bool,
        no_dock: bool,
        enable_line_correction: bool,
        enable_docking_line: bool,
        line_primary: bool,
        line_sensor_primary: bool,
        line_source: str,
        competition: bool,
        resume_from_current: bool,
        start_from_start: bool,
        stop_after: str | None,
        full_test: bool,
        output_dir: Path | None,
        docking_speed: float,
        docking_distance: float,
        reverse_distance: float,
        turn_speed: float,
        waypoint_overrides: dict[str, str] | None,
        line_primary_goal_tolerance_m: float,
        line_primary_corridor_width_m: float,
        line_primary_min_progress_m: float,
        line_primary_segment_timeout_sec: float,
        line_approach_distance_tolerance_m: float,
        line_approach_cross_track_threshold_m: float,
        line_approach_heading_threshold_rad: float,
        line_approach_min_progress_ratio: float,
        competition_qr_reacquire_timeout_sec: float,
    ) -> None:
        super().__init__(
            "route_executor",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.actions = actions
        self.waypoints = waypoints
        self.route_name = route_name
        self.dock_only_target = dock_only_target
        self.nav_only = nav_only
        self.no_dock = no_dock
        self.enable_line_correction = enable_line_correction
        self.enable_docking_line = enable_docking_line
        self.line_primary = line_primary
        self.line_sensor_primary = line_sensor_primary
        normalized_line_source = line_source.strip().lower()
        if normalized_line_source == "camera":
            normalized_line_source = "camera_line"
        self.line_source = normalized_line_source
        self.competition = competition
        self.resume_from_current = resume_from_current
        self.start_from_start = start_from_start
        self.stop_after = stop_after
        self.full_test = full_test
        self.output_dir = output_dir
        self.docking_speed = abs(docking_speed)
        self.docking_distance = abs(docking_distance)
        self.reverse_distance = abs(reverse_distance)
        self.turn_speed = abs(turn_speed)
        self.waypoint_overrides = {k.upper(): v.upper() for k, v in (waypoint_overrides or {}).items()}
        self.mission_points = self.load_mission_points()
        self.line_segments = self.load_line_segments()
        self.fork_tip_center_offset_m = 0.78
        self.fork_spacing_m = 0.30
        self.pallet_opening_face_offset_m = 0.35
        self.fork_min_insertion_depth_m = 0.12
        self.dock_fork_gap_tolerance_m = -0.02
        self.dock_fork_lateral_tolerance_m = 0.10
        self.dock_fork_heading_tolerance_rad = 0.15
        self.dock_line_cross_track_tolerance_m = 0.04
        self.dock_line_heading_tolerance_rad = 0.05
        self.dock_lift_line_cross_track_tolerance_m = 0.015
        self.dock_lift_line_heading_tolerance_rad = 0.025
        # For a stable lift, the forks must support the pallet/load center of mass,
        # not merely enter past the opening face or barely reach the load centerline.
        self.dock_lift_support_margin_m = 0.12
        self.dock_lift_gap_tolerance_m = -(self.pallet_opening_face_offset_m + self.dock_lift_support_margin_m)
        self.dock_lift_lateral_tolerance_m = 0.050
        self.dock_lift_heading_tolerance_rad = 0.060
        self.dock_lift_fork_track_tolerance_m = 0.025
        self.dock_lift_cmd_linear_tolerance_mps = 0.01
        self.dock_lift_cmd_angular_tolerance_radps = 0.02
        self.b_dock_pose_distance_tolerance_m = 0.20
        self.b_dock_heading_tolerance_rad = 0.18
        self.b_dock_cross_track_tolerance_m = 0.08
        self.validation_lift_height_m = 0.10
        self.validation_lift_stage_targets_m = [0.03, 0.06, self.validation_lift_height_m]
        self.validation_lift_min_z_increase_m = 0.015
        self.validation_lift_max_tilt_rad = 0.10
        self.validation_lift_height_tolerance_m = 0.01
        self.validation_lift_max_horizontal_shift_m = 0.12
        self.a1_dock_align_cross_track_tolerance_m = 0.035
        self.a1_dock_align_heading_tolerance_rad = 0.060
        self.a1_dock_realign_retry_limit = 1
        self.a1_dock_realign_backout_distance_m = 0.20
        self.a1_dock_realign_backout_speed_mps = 0.04
        self.a1_dock_creep_support_depth_target_m = 0.08
        self.a1_dock_creep_speed_mps = 0.02
        self.a1_dock_creep_max_angular_radps = 0.05
        self.a1_dock_creep_k_cross = 2.0
        self.a1_dock_creep_k_heading = 1.5
        self.a1_dock_creep_k_pallet_lateral = 1.8
        self.a1_dock_creep_k_pallet_heading = 1.2
        self.a1_dock_creep_timeout_sec = 90.0
        self.a1_dock_creep_max_extra_m = 1.20
        self.a1_dock_align_hold_samples = 3
        self.a1_dock_hard_fail_samples = 4
        self.load_lift_min_z_increase_m = 0.03
        self.load_lift_max_tilt_rad = 0.20
        self.load_lift_fork_opening_distance_tolerance_m = 0.25
        self.unload_min_z_drop_m = 0.10
        self.unload_max_tilt_rad = 0.25
        self.unload_release_max_fork_height_m = 0.08

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.entity_state_client: Any | None = None
        self.entity_state_service_name: str | None = None
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_docking", 10)
        self.line_enable_pub = self.create_publisher(Bool, "/line_follower/enable", 10)
        self.line_mode_pub = self.create_publisher(String, "/line_follower/mode", 10)
        self.line_profile_pub = self.create_publisher(String, "/line_follower/profile", 10)
        self.line_context_pub = self.create_publisher(String, "/line_follower/context", 10)
        self.line_sensor_enable_pub = self.create_publisher(Bool, "/line_sensor/enable", 10)
        self.line_sensor_mode_pub = self.create_publisher(String, "/line_sensor/mode", 10)
        self.pose_line_enable_pub = self.create_publisher(Bool, "/pose_line/enable", 10)
        self.pose_line_mode_pub = self.create_publisher(String, "/pose_line/mode", 10)
        self.pose_line_segment_pub = self.create_publisher(String, "/pose_line/segment", 10)
        self.fork_cmd_pub = self.create_publisher(String, "/fork_cmd", 10)

        self.create_subscription(Odometry, "/diff_cont/odom", self.odom_cb, 50)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 50)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_cb, 20)
        self.create_subscription(Clock, "/clock", self.clock_cb, 100)
        self.create_subscription(String, "/qr_code", self.qr_cb, 20)
        self.create_subscription(String, "/line_follower/status", self.line_status_cb, 50)
        self.create_subscription(String, "/fork_state", self.fork_state_cb, 20)
        self.create_subscription(Float32, "/line_sensor/position", self.line_sensor_position_cb, 50)
        self.create_subscription(Bool, "/line_sensor/seen", self.line_sensor_seen_cb, 50)
        self.create_subscription(String, "/line_sensor/status", self.line_sensor_status_cb, 50)
        self.create_subscription(String, "/pose_line/status", self.pose_line_status_cb, 50)
        self.create_subscription(Twist, "/cmd_vel_nav", self.cmd_vel_nav_cb, 50)
        self.create_subscription(Twist, "/cmd_vel_route_fused", self.cmd_vel_route_fused_cb, 50)
        self.create_subscription(Twist, "/cmd_vel_line_sensor", self.cmd_vel_line_sensor_cb, 50)
        self.create_subscription(Twist, "/cmd_vel_pose_line", self.cmd_vel_pose_line_cb, 50)
        self.create_subscription(Twist, "/diff_cont/cmd_vel_unstamped", self.cmd_vel_unstamped_cb, 50)
        self.create_subscription(TwistStamped, "/diff_cont/cmd_vel", self.cmd_vel_stamped_cb, 50)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.global_costmap_cb, 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.local_costmap_cb, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.current_pose: tuple[float, float, float] | None = None
        self.current_map_pose: tuple[float, float, float] | None = None
        self.last_odom_wall_time = 0.0
        self.last_amcl_pose_wall_time = 0.0
        self.last_clock_time_sec: float | None = None
        self.clock_samples: list[float] = []
        self.clock_jump_back_detected = False
        self.last_qr = ""
        self.last_qr_time = 0.0
        self.latest_qr_latch: dict[str, Any] | None = None
        self.qr_latch_max_age_sec = 30.0
        self.qr_latch_max_distance_m = 1.5
        self.last_line_status: dict[str, Any] = {}
        self.last_line_status_time = 0.0
        self.last_line_sensor_status: dict[str, Any] = {}
        self.last_line_sensor_status_time = 0.0
        self.last_pose_line_status: dict[str, Any] = {}
        self.last_pose_line_status_time = 0.0
        self.last_line_sensor_position = 0.0
        self.last_line_sensor_seen = False
        self.last_a1_insert_cmd_linear = 0.0
        self.last_a1_insert_cmd_angular = 0.0
        self.last_fork_state = ""
        self.last_fork_state_time = 0.0
        self.last_global_costmap: OccupancyGrid | None = None
        self.last_local_costmap: OccupancyGrid | None = None
        self.reached: list[str] = []
        self.skipped_docks: list[str] = []

        self.current_segment: dict[str, Any] | None = None
        self.segment_records: list[dict[str, Any]] = []
        self.first_failed_segment: str | None = None
        self.failure_reason: str = ""
        self.diagnostic_results: list[dict[str, Any]] = []
        self.cmd_vel_totals = {
            "cmd_vel_nav_nonzero": 0,
            "cmd_vel_route_fused_nonzero": 0,
            "cmd_vel_line_sensor_nonzero": 0,
            "cmd_vel_pose_line_nonzero": 0,
            "cmd_vel_unstamped_nonzero": 0,
            "cmd_vel_stamped_nonzero": 0,
        }
        self.line_primary_distance_tolerance = max(0.05, abs(line_primary_goal_tolerance_m))
        self.line_primary_corridor_width_m = max(0.05, abs(line_primary_corridor_width_m))
        self.line_primary_min_progress_m = max(0.0, abs(line_primary_min_progress_m))
        self.line_primary_timeout_sec = max(10.0, abs(line_primary_segment_timeout_sec))
        self.camera_line_staging_distance_tolerance_m = 0.08
        self.camera_line_staging_error_px_threshold = 18.0
        self.camera_line_staging_heading_threshold_rad = 0.05
        self.camera_line_staging_required_streak = 5
        self.camera_line_predock_distance_tolerance_m = 0.08
        self.camera_line_predock_error_px_threshold = 18.0
        self.camera_line_predock_heading_threshold_rad = 0.05
        self.camera_line_predock_required_streak = 5
        self.camera_line_predock_timeout_sec = 8.0
        self.line_approach_distance_tolerance_m = max(0.05, abs(line_approach_distance_tolerance_m))
        self.line_approach_cross_track_threshold_m = max(
            0.01, abs(line_approach_cross_track_threshold_m)
        )
        self.line_approach_heading_threshold_rad = max(
            0.01, abs(line_approach_heading_threshold_rad)
        )
        self.line_approach_min_progress_ratio = min(
            1.0, max(0.0, float(line_approach_min_progress_ratio))
        )
        self.competition_qr_reacquire_timeout_sec = max(
            0.5, abs(competition_qr_reacquire_timeout_sec)
        )
        self.last_path_sample_time = 0.0
        self.shutdown_started = False

        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self.segment_results_path = self.output_dir / "segment_results.jsonl"
            self.summary_path = self.output_dir / "summary.json"
            self.cmd_vel_stats_path = self.output_dir / "cmd_vel_stats.json"
            self.failure_reason_path = self.output_dir / "failure_reason.txt"
            self.path_trace_path = self.output_dir / "path_trace.csv"
            self.segment_results_fp = self.segment_results_path.open("w", encoding="utf-8")
        else:
            self.segment_results_path = None
            self.summary_path = None
            self.cmd_vel_stats_path = None
            self.failure_reason_path = None
            self.path_trace_path = None
            self.segment_results_fp = None
        self.path_trace_records: list[dict[str, Any]] = []

    def runtime_now(self) -> float:
        """Use sim time when available so slow Gazebo runs don't trigger false timeouts."""
        try:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
        except Exception:
            now_sec = 0.0
        if now_sec > 0.0:
            return float(now_sec)
        if self.last_clock_time_sec is not None and self.last_clock_time_sec > 0.0:
            return float(self.last_clock_time_sec)
        return time.monotonic()

    def runtime_deadline(self, duration_sec: float) -> float:
        return self.runtime_now() + max(0.0, float(duration_sec))

    def segment_pose_source(self, segment: dict[str, Any] | None = None) -> str:
        segment = segment or self.current_segment
        if not isinstance(segment, dict):
            return "route_pose"
        segment_type = str(segment.get("segment_type") or "")
        controller_source = str(segment.get("controller_source") or "")
        if segment_type in {"pose_line_nav", "pose_line_dock", "competition_line_follow"}:
            return "fine_control_pose"
        if controller_source == "pose_line":
            return "fine_control_pose"
        return "route_pose"

    def pose_for_segment(self, segment: dict[str, Any] | None = None) -> tuple[float, float, float] | None:
        if self.segment_pose_source(segment) == "fine_control_pose":
            return self.current_fine_control_pose()
        return self.current_route_pose()

    def load_mission_points(self) -> dict[str, dict[str, float]]:
        path = config_path("mission_points.yaml")
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        raw_points = loaded.get("points", {})
        if not isinstance(raw_points, dict):
            return {}
        mission_points: dict[str, dict[str, float]] = {}
        for name, pose in raw_points.items():
            if not isinstance(pose, dict):
                continue
            try:
                mission_points[str(name).upper()] = {
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "yaw": float(pose["yaw"]),
                }
            except (KeyError, TypeError, ValueError):
                continue
        return mission_points

    def load_line_segments(self) -> dict[str, dict[str, dict[str, str]]]:
        path = config_path("line_segments.yaml")
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(loaded, dict):
            return {"road_segments": {}, "approach_segments": {}, "dock_segments": {}}
        result: dict[str, dict[str, dict[str, str]]] = {
            "road_segments": {},
            "approach_segments": {},
            "dock_segments": {},
        }
        for section in result:
            raw_section = loaded.get(section, {})
            if not isinstance(raw_section, dict):
                continue
            for name, segment in raw_section.items():
                if not isinstance(segment, dict):
                    continue
                start = str(segment.get("start", "")).upper()
                end = str(segment.get("end", "")).upper()
                if start and end:
                    result[section][str(name).upper()] = {"start": start, "end": end}
        return result

    def road_segment_name_for_transition(self, start_name: str | None, end_name: str | None) -> str | None:
        if not start_name or not end_name:
            return None
        start = self.resolve_waypoint_name(start_name)
        end = self.resolve_waypoint_name(end_name)
        for segment_name, segment in self.line_segments.get("road_segments", {}).items():
            if segment.get("start") == start and segment.get("end") == end:
                return segment_name
        return None

    def segment_geometry(self, segment_name: str | None) -> dict[str, Any]:
        if not segment_name:
            return {}
        normalized = str(segment_name).upper()
        for section_name, section in self.line_segments.items():
            segment = section.get(normalized)
            if segment is None:
                continue
            start_name = segment["start"]
            end_name = segment["end"]
            start_pose = self.waypoint_pose_tuple(start_name)
            end_pose = self.waypoint_pose_tuple(end_name)
            return {
                "segment_name": normalized,
                "segment_section": section_name,
                "segment_start_waypoint": start_name,
                "segment_end_waypoint": end_name,
                "segment_start_pose": start_pose,
                "segment_end_pose": end_pose,
            }
        return {"segment_name": normalized}

    def record_path_trace_sample(self, now_wall: float) -> None:
        if self.current_segment is None:
            return
        route_pose = self.pose_for_segment(self.current_segment)
        if route_pose is None:
            return
        target_pose = self.current_segment.get("target_pose")
        segment_info = self.segment_geometry(self.current_segment.get("segment_name"))
        pose_line_status = self.last_pose_line_status if isinstance(self.last_pose_line_status, dict) else {}
        sim_time_sec = self.get_clock().now().nanoseconds / 1e9 if self.last_clock_time_sec is not None else None
        start_pose = segment_info.get("segment_start_pose")
        end_pose = segment_info.get("segment_end_pose")
        self.path_trace_records.append(
            {
                "wall_time_sec": now_wall,
                "sim_time_sec": sim_time_sec,
                "map_x": route_pose[0],
                "map_y": route_pose[1],
                "map_yaw": route_pose[2],
                "odom_x": self.current_pose[0] if self.current_pose is not None else None,
                "odom_y": self.current_pose[1] if self.current_pose is not None else None,
                "odom_yaw": self.current_pose[2] if self.current_pose is not None else None,
                "active_action_type": self.current_segment.get("segment_type"),
                "active_action_name": self.current_segment.get("segment_name"),
                "target_name": self.current_segment.get("target_name"),
                "target_x": target_pose[0] if target_pose is not None else None,
                "target_y": target_pose[1] if target_pose is not None else None,
                "target_yaw": target_pose[2] if target_pose is not None else None,
                "distance_to_target_m": pose_distance(route_pose, target_pose),
                "line_segment_name": segment_info.get("segment_name"),
                "line_segment_section": segment_info.get("segment_section"),
                "segment_start_waypoint": segment_info.get("segment_start_waypoint"),
                "segment_start_x": start_pose[0] if start_pose is not None else None,
                "segment_start_y": start_pose[1] if start_pose is not None else None,
                "segment_start_yaw": start_pose[2] if start_pose is not None else None,
                "segment_end_waypoint": segment_info.get("segment_end_waypoint"),
                "segment_end_x": end_pose[0] if end_pose is not None else None,
                "segment_end_y": end_pose[1] if end_pose is not None else None,
                "segment_end_yaw": end_pose[2] if end_pose is not None else None,
                "cross_track_error_m": pose_line_status.get("cross_track_error_m"),
                "heading_error_rad": pose_line_status.get("heading_error_rad"),
            }
        )

    def log_action_context(self, action: dict[str, str], index: int, total: int) -> None:
        action_type = action["type"]
        name = action["name"]
        target_name = self.resolve_waypoint_name(action["target"]) if action.get("target") else None
        target_pose = self.waypoint_pose_tuple(target_name) if target_name and target_name in self.waypoints else None
        segment_name = action.get("segment")
        if segment_name is None and action_type in {"line_approach", "dock"}:
            segment_name = name
        segment_info = self.segment_geometry(segment_name)
        segment_suffix = ""
        if segment_info:
            start_pose = segment_info.get("segment_start_pose")
            end_pose = segment_info.get("segment_end_pose")
            segment_suffix = (
                f" segment={segment_info.get('segment_name')} "
                f"segment_section={segment_info.get('segment_section')} "
                f"segment_start={segment_info.get('segment_start_waypoint')}"
            )
            if start_pose is not None:
                segment_suffix += f"({start_pose[0]:.4f},{start_pose[1]:.4f},{start_pose[2]:.4f})"
            segment_suffix += f" segment_end={segment_info.get('segment_end_waypoint')}"
            if end_pose is not None:
                segment_suffix += f"({end_pose[0]:.4f},{end_pose[1]:.4f},{end_pose[2]:.4f})"
        target_suffix = ""
        if target_pose is not None:
            target_suffix = (
                f" target={target_name} "
                f"target_pose=({target_pose[0]:.4f},{target_pose[1]:.4f},{target_pose[2]:.4f})"
            )
        self.get_logger().info(
            f"ACTION_CONTEXT index={index}/{total} type={action_type} name={name}{target_suffix}{segment_suffix}"
        )

    def stop_after_reached(self, action: dict[str, str], resolved_target: str | None = None) -> bool:
        if not self.stop_after:
            return False
        candidates = {
            str(action.get("name", "")).upper(),
            str(action.get("target", "")).upper(),
            str(resolved_target or "").upper(),
            str(action.get("qr_id", "")).upper(),
        }
        if self.stop_after.upper() in candidates:
            self.get_logger().info(f"STOP_AFTER reached {self.stop_after}; ending test early")
            return True
        return False

    def odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.current_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quaternion_to_yaw(q.x, q.y, q.z, q.w),
        )
        self.last_odom_wall_time = time.monotonic()
        if self.current_segment is not None:
            now = time.monotonic()
            if now - self.last_path_sample_time >= 0.20:
                target_pose = self.current_segment.get("target_pose")
                route_pose = self.pose_for_segment(self.current_segment)
                self.current_segment.setdefault("path_samples", []).append(
                    {
                        "t": now,
                        "x": route_pose[0] if route_pose else self.current_pose[0],
                        "y": route_pose[1] if route_pose else self.current_pose[1],
                        "yaw": route_pose[2] if route_pose else self.current_pose[2],
                        "distance_to_target_m": pose_distance(route_pose, target_pose),
                        "line_phase": self.last_line_status.get("line_primary_phase", ""),
                        "line_detected": bool(self.last_line_status.get("line_detected", False)),
                        "error_px": float(self.last_line_status.get("error_px", 0.0)),
                        "filtered_error_px": float(self.last_line_status.get("filtered_error_px", 0.0)),
                    }
                )
                self.record_path_trace_sample(now)
                self.last_path_sample_time = now

    def amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        self.current_map_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quaternion_to_yaw(q.x, q.y, q.z, q.w),
        )
        self.last_amcl_pose_wall_time = time.monotonic()

    def clock_cb(self, msg: Clock) -> None:
        now_sec = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if self.last_clock_time_sec is not None and now_sec + 1e-6 < self.last_clock_time_sec:
            self.clock_jump_back_detected = True
        self.last_clock_time_sec = now_sec
        self.clock_samples.append(time.monotonic())
        if len(self.clock_samples) > 200:
            self.clock_samples = self.clock_samples[-200:]

    def qr_cb(self, msg: String) -> None:
        qr_value = msg.data.strip().upper()
        qr_time = self.get_clock().now().nanoseconds / 1e9
        self.last_qr = qr_value
        self.last_qr_time = qr_time

        if self.current_segment is None:
            return
        if self.current_segment.get("segment_type") not in {"pose_line_nav", "line_primary_nav", "line_sensor_primary_nav"}:
            return
        expected_qr = str(self.current_segment.get("expected_qr_id") or "").upper()
        if not expected_qr:
            return

        if qr_value != expected_qr:
            previous_mismatch = self.current_segment.get("last_qr_mismatch")
            if previous_mismatch != qr_value:
                self.get_logger().info(
                    f"QR_MISMATCH_DURING_APPROACH segment={self.current_segment.get('segment_name')} "
                    f"expected={expected_qr} seen={qr_value}"
                )
                self.current_segment["last_qr_mismatch"] = qr_value
            return

        route_pose = self.current_route_pose()
        latch = {
            "qr_id": qr_value,
            "target_name": self.current_segment.get("target_name"),
            "segment_name": self.current_segment.get("segment_name"),
            "detected_time_sec": qr_time,
            "detected_pose": list(route_pose) if route_pose is not None else None,
        }
        self.latest_qr_latch = latch
        self.current_segment["qr_latch"] = latch
        self.current_segment["qr_detected_during_approach"] = True
        previous_latch_time = self.current_segment.get("qr_latch_logged_time_sec")
        if previous_latch_time is None or (qr_time - float(previous_latch_time)) >= 0.5:
            pose_suffix = ""
            if route_pose is not None:
                pose_suffix = f" pose=({route_pose[0]:.3f},{route_pose[1]:.3f},{route_pose[2]:.3f})"
            self.get_logger().info(
                f"QR_LATCHED_DURING_APPROACH segment={self.current_segment.get('segment_name')} "
                f"expected={expected_qr} seen={qr_value}{pose_suffix}"
            )
            self.current_segment["qr_latch_logged_time_sec"] = qr_time

    def line_status_cb(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(parsed, dict):
            self.last_line_status = parsed
            self.last_line_status_time = time.monotonic()
            if self.current_segment is not None:
                counts = self.current_segment.setdefault("line_phase_transition_counts", {})
                for phase, count in parsed.get("state_transition_counts", {}).items():
                    counts[phase] = max(counts.get(phase, 0), int(count))
                samples = self.current_segment.setdefault(
                    "line_metrics",
                    {
                        "samples": 0,
                        "detected_samples": 0,
                        "centered_samples": 0,
                        "abs_error_px_sum": 0.0,
                        "max_abs_error_px": 0.0,
                        "abs_normalized_error_sum": 0.0,
                        "max_abs_normalized_error": 0.0,
                    },
                )
                samples["samples"] += 1
                detected = bool(parsed.get("line_detected", False))
                centered = bool(parsed.get("robot_centered", False))
                error_px = abs(float(parsed.get("error_px", 0.0)))
                normalized_error = abs(float(parsed.get("normalized_error", 0.0)))
                if detected:
                    samples["detected_samples"] += 1
                if centered:
                    samples["centered_samples"] += 1
                samples["abs_error_px_sum"] += error_px
                samples["max_abs_error_px"] = max(samples["max_abs_error_px"], error_px)
                samples["abs_normalized_error_sum"] += normalized_error
                samples["max_abs_normalized_error"] = max(
                    samples["max_abs_normalized_error"], normalized_error
                )

    def line_sensor_position_cb(self, msg: Float32) -> None:
        self.last_line_sensor_position = float(msg.data)

    def line_sensor_seen_cb(self, msg: Bool) -> None:
        self.last_line_sensor_seen = bool(msg.data)

    def line_sensor_status_cb(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            self.last_line_sensor_status = {"raw": msg.data}
            self.last_line_sensor_status_time = time.monotonic()
            return
        if isinstance(parsed, dict):
            self.last_line_sensor_status = parsed
            self.last_line_sensor_status_time = time.monotonic()

    def pose_line_status_cb(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            self.last_pose_line_status = {"raw": msg.data}
            self.last_pose_line_status_time = time.monotonic()
            return
        if isinstance(parsed, dict):
            self.last_pose_line_status = parsed
            self.last_pose_line_status_time = time.monotonic()

    def fork_state_cb(self, msg: String) -> None:
        self.last_fork_state = msg.data
        self.last_fork_state_time = time.monotonic()

    def global_costmap_cb(self, msg: OccupancyGrid) -> None:
        self.last_global_costmap = msg

    def local_costmap_cb(self, msg: OccupancyGrid) -> None:
        self.last_local_costmap = msg

    def _twist_nonzero(self, linear_x: float, angular_z: float) -> bool:
        return abs(linear_x) > 1e-4 or abs(angular_z) > 1e-4

    def _record_cmd_vel(self, key: str, linear_x: float, angular_z: float) -> None:
        if self._twist_nonzero(linear_x, angular_z):
            self.cmd_vel_totals[key] += 1
            if self.current_segment is not None:
                self.current_segment.setdefault("cmd_vel_stats", {})
                self.current_segment["cmd_vel_stats"][key] = (
                    self.current_segment["cmd_vel_stats"].get(key, 0) + 1
                )

    def cmd_vel_nav_cb(self, msg: Twist) -> None:
        self._record_cmd_vel("cmd_vel_nav_nonzero", float(msg.linear.x), float(msg.angular.z))

    def cmd_vel_route_fused_cb(self, msg: Twist) -> None:
        self._record_cmd_vel("cmd_vel_route_fused_nonzero", float(msg.linear.x), float(msg.angular.z))

    def cmd_vel_line_sensor_cb(self, msg: Twist) -> None:
        self._record_cmd_vel("cmd_vel_line_sensor_nonzero", float(msg.linear.x), float(msg.angular.z))

    def cmd_vel_pose_line_cb(self, msg: Twist) -> None:
        self._record_cmd_vel("cmd_vel_pose_line_nonzero", float(msg.linear.x), float(msg.angular.z))

    def cmd_vel_unstamped_cb(self, msg: Twist) -> None:
        self._record_cmd_vel("cmd_vel_unstamped_nonzero", float(msg.linear.x), float(msg.angular.z))

    def cmd_vel_stamped_cb(self, msg: TwistStamped) -> None:
        self._record_cmd_vel("cmd_vel_stamped_nonzero", float(msg.twist.linear.x), float(msg.twist.angular.z))

    def wait_for_runtime(self) -> bool:
        self.get_logger().info("Waiting for /navigate_to_pose, sim clock, and initial local pose after readiness handoff...")
        if not self.nav_client.wait_for_server(timeout_sec=20.0):
            self.failure_reason = "runtime_not_ready:navigate_to_pose_unavailable"
            self.get_logger().error("/navigate_to_pose action server not available")
            return False
        start = time.monotonic()
        last_clock_jump_count = 0
        while rclpy.ok() and time.monotonic() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.clock_jump_back_detected:
                if self.clock_jump_count != last_clock_jump_count:
                    self.get_logger().warn(
                        "Detected jump back in sim time during startup; resetting stability window and waiting again"
                    )
                    last_clock_jump_count = self.clock_jump_count
                if time.monotonic() - start >= 8.0:
                    self.failure_reason = "runtime_not_ready:clock_jump_back_detected"
                    self.get_logger().error(
                        "Detected repeated jump back in sim time during startup; aborting mission before motion"
                    )
                    return False
                continue
            if self.get_clock().now().nanoseconds <= 0:
                continue
            route_pose = self.current_route_pose()
            if self.current_pose is None and route_pose is None:
                continue
            if not self.tf_ready():
                self.get_logger().warn(
                    "Proceeding after readiness handoff without fresh TF in the new route_executor yet; "
                    "using available local pose and letting runtime navigation settle TF."
                )
            self.log_clock_and_tf_status()
            return True
        details = {
            "clock_started": self.get_clock().now().nanoseconds > 0,
            "have_odom_pose": self.current_pose is not None,
            "have_amcl_pose": self.current_map_pose is not None,
            "have_tf_route_pose": self.current_map_pose_from_tf() is not None,
            "tf_ready": self.tf_ready(),
        }
        self.failure_reason = (
            "runtime_not_ready:basic_pose_tf_timeout:"
            + ",".join(f"{key}={str(value).lower()}" for key, value in details.items())
        )
        self.get_logger().error(
            "Timed out waiting for initial pose/TF after readiness handoff: "
            + ", ".join(f"{key}={value}" for key, value in details.items())
        )
        return False

    def tf_ready(self) -> bool:
        try:
            map_odom_ok = self.tf_buffer.can_transform(
                "map", "odom", rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
            odom_base_ok = self.tf_buffer.can_transform(
                "odom", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
            return bool(map_odom_ok and odom_base_ok)
        except TransformException:
            return False
        except Exception:
            return False

    def transform_age_sec(self, target: str, source: str) -> float | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return None
        current_now = self.get_clock().now().nanoseconds * 1e-9
        stamp_now = float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) * 1e-9
        if current_now <= 0.0 or stamp_now <= 0.0:
            return None
        return max(0.0, current_now - stamp_now)

    def localization_fresh(self) -> bool:
        now_wall = time.monotonic()
        if (now_wall - self.last_odom_wall_time) > 1.0:
            return False
        if self.current_map_pose is None or (now_wall - self.last_amcl_pose_wall_time) > 10.0:
            return False
        map_odom_age = self.transform_age_sec("map", "odom")
        odom_base_age = self.transform_age_sec("odom", "base_link")
        if map_odom_age is None or odom_base_age is None:
            return False
        return map_odom_age <= 1.0 and odom_base_age <= 1.0

    def log_clock_and_tf_status(self) -> None:
        hz = 0.0
        if len(self.clock_samples) >= 2:
            span = self.clock_samples[-1] - self.clock_samples[0]
            if span > 1e-6:
                hz = (len(self.clock_samples) - 1) / span
        self.get_logger().info(f"/clock startup check: approx_hz={hz:.1f}, jump_back={self.clock_jump_back_detected}")
        self.log_tf_check("map", "odom")
        self.log_tf_check("odom", "base_link")
        map_odom_age = self.transform_age_sec("map", "odom")
        odom_base_age = self.transform_age_sec("odom", "base_link")
        self.get_logger().info(
            "Localization freshness: "
            f"odom_wall_age={max(0.0, time.monotonic() - self.last_odom_wall_time):.3f} "
            f"amcl_wall_age={max(0.0, time.monotonic() - self.last_amcl_pose_wall_time):.3f} "
            f"map_odom_age={map_odom_age} "
            f"odom_base_age={odom_base_age}"
        )

    def log_tf_check(self, target: str, source: str) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            t = transform.transform.translation
            self.get_logger().info(
                f"TF startup check {target}->{source}: x={t.x:.3f} y={t.y:.3f} z={t.z:.3f}"
            )
        except Exception as exc:
            self.get_logger().warn(f"TF startup check {target}->{source} unavailable: {exc}")

    def ros_context_ok(self) -> bool:
        try:
            return bool(rclpy.ok() and self.context.ok() and not self.shutdown_started)
        except Exception:
            return False

    def can_publish_stop(self) -> bool:
        try:
            return bool(rclpy.ok() and self.context.ok())
        except Exception:
            return False

    def set_line_mode(
        self,
        enabled: bool,
        mode: str,
        *,
        profile: str | None = None,
        context: str | None = None,
    ) -> None:
        if not self.ros_context_ok():
            return
        if profile is not None:
            profile_msg = String()
            profile_msg.data = profile
            try:
                self.line_profile_pub.publish(profile_msg)
            except Exception:
                return
        if context is not None:
            context_msg = String()
            context_msg.data = context
            try:
                self.line_context_pub.publish(context_msg)
            except Exception:
                return
        mode_msg = String()
        mode_msg.data = mode
        enable_msg = Bool()
        enable_msg.data = enabled
        try:
            self.line_mode_pub.publish(mode_msg)
            self.line_enable_pub.publish(enable_msg)
        except Exception:
            return

    def set_line_sensor_mode(self, enabled: bool, mode: str) -> None:
        if not self.ros_context_ok():
            return
        mode_msg = String()
        mode_msg.data = mode
        enable_msg = Bool()
        enable_msg.data = enabled
        try:
            self.line_sensor_mode_pub.publish(mode_msg)
            self.line_sensor_enable_pub.publish(enable_msg)
        except Exception:
            return

    def set_pose_line_mode(self, enabled: bool, mode: str, *, segment: str | None = None) -> None:
        if not self.ros_context_ok():
            return
        mode_msg = String()
        mode_msg.data = mode
        enable_msg = Bool()
        enable_msg.data = enabled
        segment_msg = String()
        segment_msg.data = segment or ""
        try:
            if segment is not None:
                self.pose_line_segment_pub.publish(segment_msg)
            self.pose_line_mode_pub.publish(mode_msg)
            self.pose_line_enable_pub.publish(enable_msg)
        except Exception:
            return

    def line_detected(self) -> bool:
        return bool(self.last_line_status.get("line_detected", False))

    def current_route_pose(self) -> tuple[float, float, float] | None:
        tf_pose = self.current_map_pose_from_tf()
        if tf_pose is not None:
            return tf_pose
        if self.current_map_pose is not None:
            return self.current_map_pose
        return self.current_pose

    def current_map_pose_from_tf(self) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def pose_line_status_is_fresh(self, *, max_age_sec: float = 1.0) -> bool:
        return (time.monotonic() - self.last_pose_line_status_time) <= max(0.0, float(max_age_sec))

    def expected_pose_line_status_for_active_segment(self, status: dict[str, Any] | None) -> bool:
        if not isinstance(status, dict):
            return False
        if not bool(status.get("active")):
            return False
        if str(status.get("controller_source") or "").lower() not in {"", "pose_line"}:
            return False
        segment_name = str(status.get("segment") or "").upper()
        status_mode = str(status.get("mode") or "").upper()
        if not segment_name:
            return False
        if self.current_segment is None:
            return True
        if str(self.current_segment.get("controller_source") or "").lower() != "pose_line":
            return True
        expected_segment = str(self.current_segment.get("segment_name") or "").upper()
        segment_type = str(self.current_segment.get("segment_type") or "")
        if expected_segment and segment_name != expected_segment:
            return False
        if segment_type == "pose_line_dock":
            return status_mode == "DOCKING"
        if segment_type == "pose_line_nav":
            return status_mode in {"LINE_APPROACH", "LINE_REVERSE"}
        return True

    def current_fine_control_pose(self) -> tuple[float, float, float] | None:
        status = self.last_pose_line_status if isinstance(self.last_pose_line_status, dict) else {}
        pose_x = status.get("pose_x")
        pose_y = status.get("pose_y")
        pose_yaw = status.get("pose_yaw")
        if (
            self.pose_line_status_is_fresh(max_age_sec=1.0)
            and self.expected_pose_line_status_for_active_segment(status)
            and isinstance(pose_x, (int, float))
            and isinstance(pose_y, (int, float))
            and isinstance(pose_yaw, (int, float))
        ):
            return (float(pose_x), float(pose_y), float(pose_yaw))
        return self.current_route_pose()

    def pose_line_snapshot_pose_for_segment(self, segment_name: str, *, max_age_sec: float = 1.0) -> tuple[float, float, float] | None:
        status = self.last_pose_line_status if isinstance(self.last_pose_line_status, dict) else {}
        if (time.monotonic() - self.last_pose_line_status_time) > max_age_sec:
            return None
        if str(status.get("segment", "")).upper() != str(segment_name).upper():
            return None
        pose_x = status.get("pose_x")
        pose_y = status.get("pose_y")
        pose_yaw = status.get("pose_yaw")
        if not all(isinstance(value, (int, float)) for value in (pose_x, pose_y, pose_yaw)):
            return None
        return (float(pose_x), float(pose_y), float(pose_yaw))

    def anchored_pose_from_odom(
        self,
        anchor_route_pose: tuple[float, float, float] | None,
        anchor_odom_pose: tuple[float, float, float] | None,
        current_odom_pose: tuple[float, float, float] | None,
    ) -> tuple[float, float, float] | None:
        if anchor_route_pose is None or anchor_odom_pose is None or current_odom_pose is None:
            return None
        yaw_offset = normalize_angle(anchor_route_pose[2] - anchor_odom_pose[2])
        dx = current_odom_pose[0] - anchor_odom_pose[0]
        dy = current_odom_pose[1] - anchor_odom_pose[1]
        cos_yaw = math.cos(yaw_offset)
        sin_yaw = math.sin(yaw_offset)
        return (
            float(anchor_route_pose[0] + cos_yaw * dx - sin_yaw * dy),
            float(anchor_route_pose[1] + sin_yaw * dx + cos_yaw * dy),
            float(normalize_angle(current_odom_pose[2] + yaw_offset)),
        )

    def a1_insert_control_pose(
        self,
        segment_name: str,
        anchor_route_pose: tuple[float, float, float] | None,
        anchor_odom_pose: tuple[float, float, float] | None,
    ) -> tuple[float, float, float] | None:
        anchored_pose = self.anchored_pose_from_odom(anchor_route_pose, anchor_odom_pose, self.current_pose)
        if anchored_pose is not None:
            return anchored_pose
        live_pose = self.pose_line_snapshot_pose_for_segment(segment_name, max_age_sec=1.0)
        if live_pose is not None:
            return live_pose
        if self.current_pose is not None:
            return self.current_pose
        return self.current_route_pose()

    def fresh_qr_ok(self, expected_qr: str, *, min_sample_time_sec: float | None = None) -> tuple[bool, str]:
        now = self.get_clock().now().nanoseconds / 1e9
        if not self.last_qr:
            return False, "no_qr_sample"
        if min_sample_time_sec is not None and self.last_qr_time < min_sample_time_sec:
            return False, f"no_post_arrival_qr_sample last_qr={self.last_qr}"
        age = now - self.last_qr_time
        if age > 2.0:
            return False, f"stale_qr_sample age={age:.2f}s last_qr={self.last_qr}"
        if self.last_qr != expected_qr:
            return False, f"qr_mismatch expected={expected_qr} last_qr={self.last_qr}"
        return True, "fresh_qr_ok"

    def latched_qr_ok(self, expected_qr: str, *, target_name: str) -> tuple[bool, str, dict[str, Any] | None]:
        latch = self.latest_qr_latch if isinstance(self.latest_qr_latch, dict) else None
        if not latch:
            return False, "no_qr_latch", None
        if str(latch.get("qr_id", "")).upper() != expected_qr.upper():
            return False, f"latched_qr_mismatch expected={expected_qr} latched={latch.get('qr_id')}", latch
        latched_target = str(latch.get("target_name", "")).upper()
        expected_target = self.resolve_waypoint_name(target_name)
        if self.semantic_id_for_target(latched_target) != self.semantic_id_for_target(expected_target):
            return (
                False,
                f"latched_qr_wrong_segment expected_target={expected_target} "
                f"latched_target={latch.get('target_name')}",
                latch,
            )
        now = self.get_clock().now().nanoseconds / 1e9
        detected_time_sec = float(latch.get("detected_time_sec", 0.0))
        age = now - detected_time_sec
        if age > self.qr_latch_max_age_sec:
            return False, f"latched_qr_stale age={age:.2f}s", latch
        current_pose = self.current_route_pose()
        detected_pose_raw = latch.get("detected_pose")
        if (
            current_pose is not None
            and isinstance(detected_pose_raw, list)
            and len(detected_pose_raw) >= 2
        ):
            detected_pose = (
                float(detected_pose_raw[0]),
                float(detected_pose_raw[1]),
                float(detected_pose_raw[2]) if len(detected_pose_raw) >= 3 else 0.0,
            )
            moved_distance = pose_distance(current_pose, detected_pose)
            if moved_distance is not None and moved_distance > self.qr_latch_max_distance_m:
                return False, f"latched_qr_too_far moved_distance={moved_distance:.3f}m", latch
        return True, "latched_qr_ok", latch

    def qr_pose_reference_ok(
        self,
        expected_qr: str,
        *,
        target_name: str,
    ) -> tuple[bool, str, dict[str, Any] | None]:
        semantic_target = self.semantic_id_for_target(target_name)
        if semantic_target not in {"GATE_OUT", "GATE_IN", "B1", "B2", "B3"}:
            return False, "qr_pose_reference_not_allowed", None
        if self.infer_carrying_load() is not True:
            return False, "qr_pose_reference_requires_load", None

        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_qr and (now - self.last_qr_time) <= 2.0 and self.last_qr != expected_qr:
            return False, f"fresh_qr_mismatch expected={expected_qr} last_qr={self.last_qr}", None

        route_pose = self.current_route_pose()
        target_pose = self.waypoint_pose_tuple(target_name)
        if route_pose is None or target_pose is None:
            return False, "qr_pose_reference_pose_missing", None

        distance_to_target = pose_distance(route_pose, target_pose)
        if distance_to_target is None:
            return False, "qr_pose_reference_distance_missing", None

        heading_error = abs(normalize_angle(float(target_pose[2]) - float(route_pose[2])))
        distance_tolerance = max(0.12, float(self.line_approach_distance_tolerance_m))
        if semantic_target.startswith("GATE") or target_name.upper().endswith("_STAGING"):
            heading_tolerance = math.pi
        else:
            heading_tolerance = 0.15
        if distance_to_target > distance_tolerance:
            return False, f"qr_pose_reference_too_far distance={distance_to_target:.3f}m", {
                "distance_to_target_m": float(distance_to_target),
                "heading_error_rad": float(heading_error),
                "target_name": target_name,
                "semantic_target": semantic_target,
            }
        if heading_error > heading_tolerance:
            return False, f"qr_pose_reference_bad_heading heading={heading_error:.3f}rad", {
                "distance_to_target_m": float(distance_to_target),
                "heading_error_rad": float(heading_error),
                "target_name": target_name,
                "semantic_target": semantic_target,
            }
        return True, "qr_pose_reference_ok", {
            "distance_to_target_m": float(distance_to_target),
            "heading_error_rad": float(heading_error),
            "target_name": target_name,
            "semantic_target": semantic_target,
            "carrying_load": True,
        }

    def competition_controller_source(self) -> str:
        now = time.monotonic()
        line_sensor_recent = (now - self.last_line_sensor_status_time) <= 1.0
        camera_recent = (now - self.last_line_status_time) <= 1.0
        pose_line_recent = (now - self.last_pose_line_status_time) <= 1.0
        if self.line_source == "line_sensor":
            return "line_sensor" if line_sensor_recent else "none"
        if self.line_source == "pose_line":
            return "pose_line" if pose_line_recent else "none"
        if self.line_source == "camera_line":
            return "camera_line" if camera_recent else "none"
        if line_sensor_recent:
            return "line_sensor"
        if pose_line_recent:
            return "pose_line"
        if camera_recent:
            return "camera_line"
        return "none"

    def send_fork_command(self, command: str) -> tuple[bool, str]:
        if self.fork_cmd_pub.get_subscription_count() == 0:
            return False, "fork_controller_missing_subscriber"
        msg = String()
        msg.data = command
        self.fork_cmd_pub.publish(msg)
        deadline = self.runtime_deadline(60.0)
        last_state = self.last_fork_state
        command_upper = command.strip().upper()
        while rclpy.ok() and self.runtime_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (time.monotonic() - self.last_fork_state_time) <= 1.0:
                state = self.last_fork_state.upper()
                last_state = self.last_fork_state
                if command in state and "HOLDING" in state:
                    return True, state
                parsed = parse_fork_state_text(self.last_fork_state)
                state_command = str(parsed.get("command") or "").strip().upper()
                state_height = parsed.get("height")
                state_target = parsed.get("target")
                if (
                    state_command == command_upper
                    and isinstance(state_height, (int, float))
                    and isinstance(state_target, (int, float))
                ):
                    height_error = abs(float(state_target) - float(state_height))
                    if height_error <= max(self.validation_lift_height_tolerance_m, 0.012):
                        return True, f"{state} near_target_height_error={height_error:.4f}"
                if "ERROR" in state:
                    return False, state
        suffix = f" last_state={last_state}" if last_state else ""
        return False, f"fork_state_timeout{suffix}"

    def pallet_model_entity_name(self, station: str) -> str | None:
        normalized = station.strip().upper()
        if normalized.startswith("A"):
            return f"pallet_load_{normalized.lower()}"
        return None

    def resolve_entity_state_service_name(self) -> str | None:
        if self.entity_state_service_name:
            return self.entity_state_service_name
        for service_name, service_types in self.get_service_names_and_types():
            if "simulation_interfaces/srv/GetEntityState" in service_types:
                self.entity_state_service_name = service_name
                break
        return self.entity_state_service_name

    def get_entity_state(self, entity_name: str, *, timeout_sec: float = 1.0) -> dict[str, Any] | None:
        service_name = self.resolve_entity_state_service_name()
        if service_name:
            if self.entity_state_client is None:
                self.entity_state_client = self.create_client(GetEntityState, service_name)
            if self.entity_state_client.wait_for_service(timeout_sec=timeout_sec):
                request = GetEntityState.Request()
                request.entity = entity_name
                future = self.entity_state_client.call_async(request)
                deadline = time.monotonic() + timeout_sec
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    if future.done():
                        break
                if future.done():
                    try:
                        response = future.result()
                    except Exception:
                        response = None
                    if response is not None and response.result.result == response.result.RESULT_OK:
                        pose = response.state.pose
                        roll, pitch, yaw = quaternion_to_rpy(
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        )
                        return {
                            "entity": entity_name,
                            "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
                            "orientation_rpy": [roll, pitch, yaw],
                            "source": "simulation_service",
                        }
        try:
            completed = subprocess.run(
                ["gz", "topic", "-e", "-n", "1", "-t", "/world/teknofest_arena/pose/info"],
                check=True,
                capture_output=True,
                text=True,
                timeout=max(1.0, timeout_sec + 0.5),
            )
        except Exception:
            return None
        pattern = re.compile(
            rf'pose\s*\{{\s*name:\s*"{re.escape(entity_name)}"(.*?)\n\}}',
            re.S,
        )
        match = pattern.search(completed.stdout)
        if not match:
            return None
        block = match.group(1)
        pos_block = re.search(r'position\s*\{(.*?)\}', block, re.S)
        ori_block = re.search(r'orientation\s*\{(.*?)\}', block, re.S)
        if pos_block is None or ori_block is None:
            return None

        def extract_float(section: str, field: str, default: float = 0.0) -> float:
            found = re.search(rf'{field}:\s*([-+0-9.eE]+)', section)
            return float(found.group(1)) if found else default

        px = extract_float(pos_block.group(1), "x")
        py = extract_float(pos_block.group(1), "y")
        pz = extract_float(pos_block.group(1), "z")
        ox = extract_float(ori_block.group(1), "x")
        oy = extract_float(ori_block.group(1), "y")
        oz = extract_float(ori_block.group(1), "z")
        ow = extract_float(ori_block.group(1), "w", 1.0)
        roll, pitch, yaw = quaternion_to_rpy(ox, oy, oz, ow)
        return {
            "entity": entity_name,
            "position": [px, py, pz],
            "orientation_rpy": [roll, pitch, yaw],
            "source": "gz_pose_topic",
        }

    def capture_lift_metrics(
        self,
        name: str,
        *,
        pre_lift_pallet_state: dict[str, Any] | None,
        stage_label: str = "full_lift",
    ) -> tuple[bool, str, dict[str, Any]]:
        station = self.dock_station_name(name)
        if station is None or not station.startswith("A"):
            return True, "load_verification_not_required", {}
        entity_name = self.pallet_model_entity_name(station)
        if not entity_name:
            return False, "load_entity_name_missing", {}
        if pre_lift_pallet_state is None:
            return False, "pre_lift_pallet_state_missing", {"pallet_entity": entity_name}

        post_lift_state: dict[str, Any] | None = None
        deadline = self.runtime_deadline(2.0)
        while rclpy.ok() and self.runtime_now() < deadline:
            post_lift_state = self.get_entity_state(entity_name, timeout_sec=0.2)
            if post_lift_state is not None:
                break
        if post_lift_state is None:
            return False, "pallet_state_unavailable", {"pallet_entity": entity_name}

        pre_pos = pre_lift_pallet_state.get("position") or [math.nan, math.nan, math.nan]
        post_pos = post_lift_state.get("position") or [math.nan, math.nan, math.nan]
        post_rpy = post_lift_state.get("orientation_rpy") or [math.nan, math.nan, math.nan]
        z_delta = float(post_pos[2]) - float(pre_pos[2])
        planar_shift = math.hypot(float(post_pos[0]) - float(pre_pos[0]), float(post_pos[1]) - float(pre_pos[1]))
        roll = float(post_rpy[0])
        pitch = float(post_rpy[1])
        pallet_yaw = float(post_rpy[2])

        robot_world_state = self.get_entity_state("robot", timeout_sec=0.5)
        if robot_world_state is None:
            return False, "robot_pose_missing_after_lift", {
                "pallet_entity": entity_name,
                "pre_lift_pallet_state": pre_lift_pallet_state,
                "post_lift_pallet_state": post_lift_state,
                "pallet_z_increase_m": z_delta,
            }
        robot_world_pos = robot_world_state.get("position") or [math.nan, math.nan, math.nan]
        robot_world_rpy = robot_world_state.get("orientation_rpy") or [math.nan, math.nan, math.nan]
        robot_world_pose = (
            float(robot_world_pos[0]),
            float(robot_world_pos[1]),
            float(robot_world_rpy[2]),
        )
        route_pose_after_lift = self.current_route_pose() or self.current_fine_control_pose()
        pallet_route_pose = None
        if route_pose_after_lift is not None:
            pallet_route_pose = self.anchored_pose_from_odom(
                route_pose_after_lift,
                robot_world_pose,
                (float(post_pos[0]), float(post_pos[1]), pallet_yaw),
            )
        support_robot_pose = route_pose_after_lift if route_pose_after_lift is not None else robot_world_pose
        support_pallet_pose = (
            pallet_route_pose
            if pallet_route_pose is not None
            else (float(post_pos[0]), float(post_pos[1]), pallet_yaw)
        )
        support_metrics = self.fork_support_metrics(
            support_robot_pose,
            support_pallet_pose,
        )
        fork_state_metrics = parse_fork_state_text(self.last_fork_state)
        fork_height = fork_state_metrics.get("height")
        fork_target = fork_state_metrics.get("target")
        achieved_validation_height = (
            isinstance(fork_height, (int, float))
            and isinstance(fork_target, (int, float))
            and abs(float(fork_height) - float(fork_target)) <= self.validation_lift_height_tolerance_m
        )
        fork_tip_pose = support_metrics.get("fork_tip_pose") or [math.nan, math.nan, math.nan]
        opening_pose = support_metrics.get("pallet_opening_pose") or [math.nan, math.nan, math.nan]
        metrics = {
            "pallet_entity": entity_name,
            "pre_lift_pallet_state": pre_lift_pallet_state,
            "post_lift_pallet_state": post_lift_state,
            "robot_world_pose_after_lift": list(robot_world_pose),
            "robot_route_pose_after_lift": list(route_pose_after_lift) if route_pose_after_lift is not None else None,
            "pallet_route_pose_after_lift": list(support_pallet_pose),
            "lift_stage": stage_label,
            "pallet_z_increase_m": z_delta,
            "pallet_planar_shift_m": planar_shift,
            "pallet_roll_rad": roll,
            "pallet_pitch_rad": pitch,
            "fork_state_after_lift": self.last_fork_state,
            "fork_height_after_lift_m": float(fork_height) if isinstance(fork_height, (int, float)) else None,
            "fork_target_after_lift_m": float(fork_target) if isinstance(fork_target, (int, float)) else None,
            "fork_height_within_validation_tolerance": achieved_validation_height,
            "fork_tip_pose_after_lift": fork_tip_pose,
            "left_fork_tip_pose_after_lift": support_metrics.get("left_fork_tip_pose"),
            "right_fork_tip_pose_after_lift": support_metrics.get("right_fork_tip_pose"),
            "pallet_opening_pose_after_lift": opening_pose,
            "fork_to_opening_distance_after_lift_m": support_metrics.get("fork_tip_to_opening_distance_m"),
            "left_fork_insertion_depth_after_lift_m": support_metrics.get("left_fork_insertion_depth_m"),
            "right_fork_insertion_depth_after_lift_m": support_metrics.get("right_fork_insertion_depth_m"),
            "left_fork_track_error_after_lift_m": support_metrics.get("left_fork_lateral_error_m"),
            "right_fork_track_error_after_lift_m": support_metrics.get("right_fork_lateral_error_m"),
            "support_depth_to_load_center_after_lift_m": support_metrics.get("support_depth_to_load_center_m"),
            "pallet_lateral_error_after_lift_m": support_metrics.get("fork_tip_lateral_error_m"),
            "pallet_heading_error_after_lift_rad": support_metrics.get("fork_heading_error_rad"),
        }
        return True, "lift_metrics_captured", metrics

    def verify_lifted_load(
        self,
        name: str,
        *,
        pre_lift_pallet_state: dict[str, Any] | None,
        min_z_increase_m: float | None = None,
        max_tilt_rad: float | None = None,
        required_support_depth_m: float | None = None,
        stage_label: str = "full_lift",
    ) -> tuple[bool, str, dict[str, Any]]:
        captured, capture_reason, metrics = self.capture_lift_metrics(
            name,
            pre_lift_pallet_state=pre_lift_pallet_state,
            stage_label=stage_label,
        )
        if not captured:
            return False, capture_reason, metrics
        post_lift_state = metrics.get("post_lift_pallet_state") or {}
        pre_lift_pallet_state = metrics.get("pre_lift_pallet_state") or pre_lift_pallet_state
        pre_pos = (pre_lift_pallet_state or {}).get("position") or [math.nan, math.nan, math.nan]
        post_pos = post_lift_state.get("position") or [math.nan, math.nan, math.nan]
        z_delta = float(post_pos[2]) - float(pre_pos[2])
        planar_shift = math.hypot(float(post_pos[0]) - float(pre_pos[0]), float(post_pos[1]) - float(pre_pos[1]))
        roll = float(metrics.get("pallet_roll_rad", math.nan))
        pitch = float(metrics.get("pallet_pitch_rad", math.nan))
        min_z = self.load_lift_min_z_increase_m if min_z_increase_m is None else float(min_z_increase_m)
        tilt_limit = self.load_lift_max_tilt_rad if max_tilt_rad is None else float(max_tilt_rad)
        achieved_validation_height = bool(metrics.get("fork_height_within_validation_tolerance"))
        if z_delta < min_z:
            return False, "LOAD_NOT_LIFTED", metrics
        if abs(roll) > tilt_limit or abs(pitch) > tilt_limit:
            return False, "LOAD_TILT_EXCESSIVE", metrics
        # After the pallet is airborne, recomputing "support depth" from fork/opening
        # geometry is no longer reliable enough to decide success. Use direct physical
        # evidence instead: fork reached commanded validation height, pallet rose, pallet
        # stayed level, and the pallet did not drift away horizontally.
        if not achieved_validation_height:
            return False, "LOAD_NOT_LIFTED_TO_VALIDATION_HEIGHT", metrics
        if planar_shift > self.validation_lift_max_horizontal_shift_m:
            return False, "LOAD_NOT_SUPPORTED_BY_FORKS", metrics
        return True, "load_lift_verified", metrics

    def carried_pallet_entity_name(self) -> str | None:
        try:
            pickup_station, _drop_station = self.parse_mission_targets()
        except Exception:
            return None
        return self.pallet_model_entity_name(pickup_station)

    def verify_unloaded_load(
        self,
        name: str,
        *,
        pre_unload_pallet_state: dict[str, Any] | None,
    ) -> tuple[bool, str, dict[str, Any]]:
        station = self.dock_station_name(name)
        if station is None or not station.startswith("B"):
            return True, "unload_verification_not_required", {}
        entity_name = self.carried_pallet_entity_name()
        if not entity_name:
            return False, "unload_entity_name_missing", {}
        if pre_unload_pallet_state is None:
            return False, "pre_unload_pallet_state_missing", {"pallet_entity": entity_name}

        post_unload_state: dict[str, Any] | None = None
        deadline = self.runtime_deadline(3.0)
        while rclpy.ok() and self.runtime_now() < deadline:
            post_unload_state = self.get_entity_state(entity_name, timeout_sec=0.2)
            if post_unload_state is not None:
                break
        if post_unload_state is None:
            return False, "unload_pallet_state_unavailable", {"pallet_entity": entity_name}

        pre_pos = pre_unload_pallet_state.get("position") or [math.nan, math.nan, math.nan]
        post_pos = post_unload_state.get("position") or [math.nan, math.nan, math.nan]
        post_rpy = post_unload_state.get("orientation_rpy") or [math.nan, math.nan, math.nan]
        z_drop = float(pre_pos[2]) - float(post_pos[2])
        roll = float(post_rpy[0])
        pitch = float(post_rpy[1])
        fork_state_metrics = parse_fork_state_text(self.last_fork_state)
        fork_height = fork_state_metrics.get("height")
        fork_target = fork_state_metrics.get("target")
        metrics = {
            "pallet_entity": entity_name,
            "pre_unload_pallet_state": pre_unload_pallet_state,
            "post_unload_pallet_state": post_unload_state,
            "pallet_z_drop_m": z_drop,
            "pallet_roll_rad": roll,
            "pallet_pitch_rad": pitch,
            "fork_state_after_unload": self.last_fork_state,
            "fork_height_after_unload_m": float(fork_height) if isinstance(fork_height, (int, float)) else None,
            "fork_target_after_unload_m": float(fork_target) if isinstance(fork_target, (int, float)) else None,
        }
        if z_drop < self.unload_min_z_drop_m:
            return False, "LOAD_NOT_LOWERED", metrics
        if abs(roll) > self.unload_max_tilt_rad or abs(pitch) > self.unload_max_tilt_rad:
            return False, "UNLOAD_TILT_EXCESSIVE", metrics
        if not isinstance(fork_height, (int, float)) or float(fork_height) > self.unload_release_max_fork_height_m:
            return False, "FORK_NOT_LOWERED_ENOUGH_FOR_UNLOAD", metrics
        return True, "unload_verified", metrics

    def choose_resume_index(self, actions: list[dict[str, str]]) -> int:
        route_pose = self.current_fine_control_pose()
        if route_pose is None:
            return 0
        best_index = 0
        best_distance = float("inf")
        for index, action in enumerate(actions):
            if action["type"] not in {"nav", "qr_check"}:
                continue
            target = action.get("target")
            if not target or self.resolve_waypoint_name(target) not in self.waypoints:
                continue
            pose = self.waypoint_pose_tuple(target)
            distance = pose_distance(route_pose, pose)
            if distance is not None and distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    def infer_carrying_load(self) -> bool | None:
        if not self.last_fork_state:
            return None
        state = self.last_fork_state.upper()
        if "TARGET=0.350" in state or "HEIGHT=0.350" in state:
            return True
        if "TARGET=0.000" in state or "HEIGHT=0.000" in state:
            return False
        return None

    def parse_mission_targets(self) -> tuple[str, str]:
        parts = self.route_name.upper().split("_TO_")
        if len(parts) != 2 or not parts[0] or not parts[1]:
            raise ValueError(f"Route name '{self.route_name}' does not encode PICKUP_TO_DROPOFF")
        return parts[0], parts[1]

    def semantic_id_for_target(self, target_name: str) -> str:
        target = target_name.upper()
        if target.endswith("_STAGING_CALIBRATED"):
            return target[:-19]
        if target.endswith("_STAGING_PRE"):
            return target[:-12]
        if target.endswith("_STAGING"):
            return target[:-8]
        if "_DOCK" in target:
            return target.split("_DOCK", 1)[0]
        return target

    def staging_pre_waypoint(self, semantic_target: str) -> str:
        return f"{semantic_target.upper()}_STAGING_PRE"

    def log_current_location_context(self, expected_semantic: str) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if not self.last_qr or (now - self.last_qr_time) > 2.0:
            return
        if self.last_qr != expected_semantic:
            self.get_logger().info(
                f"current_location={self.last_qr}, expected={expected_semantic}, continue navigating to {expected_semantic}"
            )

    def build_competition_actions(self) -> list[dict[str, str]]:
        scenario_actions = list(self.actions)
        planned: list[dict[str, str]] = []
        pending_qr: dict[str, list[dict[str, str]]] = {}
        last_physical_target: str | None = "START" if not self.start_from_start else None

        def flush_qr_for_target(target: str) -> None:
            target_semantic = self.semantic_id_for_target(target)
            matching_keys = [
                key
                for key in list(pending_qr.keys())
                if key == target or self.semantic_id_for_target(key) == target_semantic
            ]
            for key in matching_keys:
                for qr_action in pending_qr.pop(key, []):
                    planned.append(qr_action)

        def emit_physical_action(target: str) -> None:
            nonlocal last_physical_target
            if target == "START" and not self.start_from_start:
                pending_qr.pop(target, None)
                return
            if target == last_physical_target:
                flush_qr_for_target(target)
                return
            road_segment = self.road_segment_name_for_transition(last_physical_target, target)
            if target.endswith("_STAGING") or target.endswith("_STAGING_CALIBRATED"):
                if road_segment is not None:
                    planned.append(
                        {
                            "type": "line_follow",
                            "name": road_segment,
                            "target": target,
                            "segment": road_segment,
                        }
                    )
                    last_physical_target = target
                    flush_qr_for_target(target)
                    return
                planned.append({"type": "line_approach", "name": target, "target": target})
                last_physical_target = target
                flush_qr_for_target(target)
                return
            if road_segment is not None:
                planned.append(
                    {
                        "type": "line_follow",
                        "name": road_segment,
                        "target": target,
                        "segment": road_segment,
                    }
                )
            else:
                planned.append({"type": "transit_nav", "name": target, "target": target})
            last_physical_target = target
            flush_qr_for_target(target)

        for index, action in enumerate(scenario_actions):
            action_type = action["type"]
            target = self.resolve_waypoint_name(action["target"])

            if action_type == "qr_check":
                if target == "START" and not self.start_from_start:
                    continue
                attached_target = target
                target_semantic = self.semantic_id_for_target(target)
                for later in scenario_actions[index + 1 :]:
                    if later["type"] != "nav":
                        continue
                    later_target = self.resolve_waypoint_name(later["target"])
                    if self.semantic_id_for_target(later_target) == target_semantic:
                        attached_target = later_target
                        break
                queued = {
                    "type": "qr_check",
                    "name": action["name"],
                    "target": attached_target,
                    "qr_id": action["qr_id"],
                }
                pending_qr.setdefault(attached_target, []).append(queued)
                future_physical_target = any(
                    later["type"] == "nav"
                    and self.semantic_id_for_target(self.resolve_waypoint_name(later["target"])) == target_semantic
                    for later in scenario_actions[index + 1 :]
                )
                if not future_physical_target:
                    emit_physical_action(attached_target)
                continue

            if action_type == "nav":
                emit_physical_action(target)
                continue

            if action_type == "dock":
                planned.append({"type": "dock", "name": target, "target": target})
                last_physical_target = target
                continue

            if action_type == "duplicate_nav_skip":
                continue

        return planned

    def pose_from_waypoint(self, name: str) -> PoseStamped:
        waypoint = self.waypoints[self.resolve_waypoint_name(name)]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint["x"]
        pose.pose.position.y = waypoint["y"]
        pose.pose.position.z = 0.0
        half_yaw = waypoint["yaw"] * 0.5
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def waypoint_pose_tuple(self, name: str) -> tuple[float, float, float]:
        waypoint = self.waypoints[self.resolve_waypoint_name(name)]
        return (waypoint["x"], waypoint["y"], waypoint["yaw"])

    def resolve_waypoint_name(self, name: str) -> str:
        return self.waypoint_overrides.get(name.upper(), name.upper())

    def costmap_cell_value(self, grid: OccupancyGrid | None, x: float, y: float) -> int | None:
        if grid is None:
            return None
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        resolution = grid.info.resolution
        width = grid.info.width
        height = grid.info.height
        cell_x = int((x - origin_x) / resolution)
        cell_y = int((y - origin_y) / resolution)
        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            return None
        idx = cell_y * width + cell_x
        data = grid.data[idx]
        return int(data)

    def costmap_snapshot(self, x: float, y: float) -> dict[str, Any]:
        global_value = self.costmap_cell_value(self.last_global_costmap, x, y)
        local_value = self.costmap_cell_value(self.last_local_costmap, x, y)
        return {
            "global_costmap_cell": global_value,
            "global_costmap_blocked": bool(global_value is not None and global_value >= 50),
            "local_costmap_cell": local_value,
            "local_costmap_blocked": bool(local_value is not None and local_value >= 50),
        }

    def start_segment(
        self,
        *,
        segment_type: str,
        name: str,
        target_name: str | None = None,
        target_pose: tuple[float, float, float] | None = None,
        line_correction_enabled: bool = False,
    ) -> None:
        self.current_segment = {
            "segment_type": segment_type,
            "segment_name": name,
            "target_name": target_name,
            "start_wall_time": time.time(),
            "start_pose": None,
            "target_pose": target_pose,
            "line_correction_enabled": line_correction_enabled,
            "cmd_vel_stats": {},
            "line_status_at_start": self.last_line_status,
            "line_sensor_status_at_start": self.last_line_sensor_status,
            "path_samples": [],
            "line_phase_transition_counts": {},
        }
        self.current_segment.update(self.segment_geometry(name))
        self.current_segment["start_pose"] = self.pose_for_segment(self.current_segment)

    def finish_segment(
        self,
        *,
        success: bool,
        result_status: int | None = None,
        notes: str = "",
        extra: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        assert self.current_segment is not None
        record = dict(self.current_segment)
        record["success"] = success
        record["result_status"] = result_status
        record["notes"] = notes
        record["finish_pose"] = self.pose_for_segment(record)
        record["finish_wall_time"] = time.time()
        record["duration_sec"] = record["finish_wall_time"] - record["start_wall_time"]
        record["distance_to_target_m"] = pose_distance(record["finish_pose"], record["target_pose"])
        record["progress_metrics"] = segment_progress(record["start_pose"], record["target_pose"], record["finish_pose"])
        record["line_status_at_finish"] = self.last_line_status
        record["line_sensor_status_at_finish"] = self.last_line_sensor_status
        record["fork_state_at_finish"] = self.last_fork_state
        metrics = record.pop("line_metrics", None)
        if metrics is not None:
            samples = max(1, int(metrics.get("samples", 0)))
            record["line_metrics"] = {
                "samples": int(metrics.get("samples", 0)),
                "detected_samples": int(metrics.get("detected_samples", 0)),
                "centered_samples": int(metrics.get("centered_samples", 0)),
                "detected_ratio": float(metrics.get("detected_samples", 0)) / samples,
                "centered_ratio": float(metrics.get("centered_samples", 0)) / samples,
                "avg_abs_error_px": float(metrics.get("abs_error_px_sum", 0.0)) / samples,
                "max_abs_error_px": float(metrics.get("max_abs_error_px", 0.0)),
                "avg_abs_normalized_error": float(metrics.get("abs_normalized_error_sum", 0.0)) / samples,
                "max_abs_normalized_error": float(metrics.get("max_abs_normalized_error", 0.0)),
            }
        if record.get("path_samples"):
            start_distance = pose_distance(record["start_pose"], record["target_pose"])
            finish_distance = pose_distance(record["finish_pose"], record["target_pose"])
            record["distance_trend"] = {
                "start_distance_m": start_distance,
                "finish_distance_m": finish_distance,
                "decreased": bool(
                    start_distance is not None
                    and finish_distance is not None
                    and finish_distance < start_distance
                ),
            }
        if record["target_pose"] is not None:
            record["costmap_snapshot"] = self.costmap_snapshot(record["target_pose"][0], record["target_pose"][1])
        else:
            record["costmap_snapshot"] = {}
        if extra:
            record.update(extra)
        self.segment_records.append(record)
        if self.segment_results_fp is not None:
            self.segment_results_fp.write(json.dumps(record, ensure_ascii=True) + "\n")
            self.segment_results_fp.flush()
        if not success and self.first_failed_segment is None:
            self.first_failed_segment = record["segment_name"]
        self.current_segment = None
        return record

    def finalize_active_segment_failure(
        self,
        *,
        notes: str,
        failure_reason: str,
        extra: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        if self.current_segment is None:
            self.failure_reason = self.failure_reason or failure_reason
            return None
        if not self.first_failed_segment:
            self.first_failed_segment = str(self.current_segment.get("segment_name") or "")
        record = self.finish_segment(success=False, notes=notes, extra=extra or {})
        self.failure_reason = self.failure_reason or failure_reason
        return record

    def camera_line_alignment_metrics(
        self,
        target_name: str,
    ) -> dict[str, Any]:
        resolved_target = self.resolve_waypoint_name(target_name)
        target_pose = self.waypoint_pose_tuple(resolved_target)
        route_pose = self.current_route_pose()
        line_status: dict[str, Any] = self.last_line_status if isinstance(self.last_line_status, dict) else {}
        return {
            "target_name": resolved_target,
            "target_pose": list(target_pose) if target_pose is not None else None,
            "robot_pose": list(route_pose) if route_pose is not None else None,
            "distance_to_target_m": pose_distance(route_pose, target_pose),
            "line_detected": bool(line_status.get("line_detected", False)),
            "confidence": float(line_status.get("confidence", 0.0)),
            "error_px": float(line_status.get("error_px", 0.0)),
            "filtered_error_px": float(line_status.get("filtered_error_px", line_status.get("error_px", 0.0))),
            "heading_error_rad": float(line_status.get("heading_error_rad", 0.0)),
            "robot_centered": bool(line_status.get("robot_centered", False)),
            "centered_candidate": bool(line_status.get("centered_candidate", False)),
            "centered_streak": int(line_status.get("centered_streak", 0) or 0),
            "error_decreasing": bool(line_status.get("error_decreasing", False)),
            "filtered_error_rate": float(line_status.get("filtered_error_rate", 0.0)),
            "reverse_sign_mode": bool(line_status.get("reverse_sign_mode", False)),
            "linear_cmd": float(line_status.get("linear_cmd", 0.0)),
            "angular_cmd": float(line_status.get("angular_cmd", 0.0)),
            "raw_angular_cmd": float(line_status.get("raw_angular_cmd", 0.0)),
            "control_decision": str(line_status.get("control_decision", "")),
            "centered_error_px_threshold": float(line_status.get("centered_error_px_threshold", 25.0)),
            "centered_heading_error_rad_threshold": float(
                line_status.get("centered_heading_error_rad_threshold", 0.05)
            ),
            "centered_required_streak": int(line_status.get("centered_required_streak", 1) or 1),
        }

    def camera_line_staging_settle_candidate(self, metrics: dict[str, Any]) -> bool:
        remaining = metrics.get("distance_to_target_m")
        filtered_error_px = abs(float(metrics.get("filtered_error_px", 0.0)))
        heading_error = abs(float(metrics.get("heading_error_rad", 0.0)))
        return bool(
            remaining is not None
            and float(remaining) <= self.camera_line_staging_distance_tolerance_m
            and bool(metrics.get("line_detected", False))
            and float(metrics.get("confidence", 0.0)) >= 0.9
            and filtered_error_px <= self.camera_line_staging_error_px_threshold
            and heading_error <= self.camera_line_staging_heading_threshold_rad
        )

    def camera_line_ready_for_staging(self, target_name: str) -> tuple[bool, dict[str, Any]]:
        metrics = self.camera_line_alignment_metrics(target_name)
        ready = self.camera_line_staging_settle_candidate(metrics)
        return ready, metrics

    def wait_for_camera_line_staging_settle(
        self,
        target_name: str,
        *,
        timeout_sec: float = 1.0,
    ) -> tuple[bool, dict[str, Any]]:
        deadline = time.monotonic() + timeout_sec
        last_metrics = self.camera_line_alignment_metrics(target_name)
        zero = Twist()
        candidate_streak = 0
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                self.cmd_pub.publish(zero)
            except Exception:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
            _, last_metrics = self.camera_line_ready_for_staging(target_name)
            if self.camera_line_staging_settle_candidate(last_metrics):
                candidate_streak += 1
            else:
                candidate_streak = 0
            if candidate_streak >= self.camera_line_staging_required_streak:
                last_metrics["staging_settle_streak"] = candidate_streak
                return True, last_metrics
        last_metrics["staging_settle_streak"] = candidate_streak
        return False, last_metrics

    def camera_line_ready_for_dock(self, target_name: str) -> tuple[bool, dict[str, Any]]:
        metrics = self.camera_line_alignment_metrics(target_name)
        remaining = metrics.get("distance_to_target_m")
        filtered_error_px = abs(float(metrics.get("filtered_error_px", 0.0)))
        heading_error = abs(float(metrics.get("heading_error_rad", 0.0)))
        ready = bool(
            remaining is not None
            and float(remaining) <= self.camera_line_predock_distance_tolerance_m
            and bool(metrics.get("line_detected", False))
            and float(metrics.get("confidence", 0.0)) >= 0.9
            and bool(metrics.get("robot_centered", False))
            and int(metrics.get("centered_streak", 0)) >= self.camera_line_predock_required_streak
            and filtered_error_px <= self.camera_line_predock_error_px_threshold
            and heading_error <= self.camera_line_predock_heading_threshold_rad
        )
        return ready, metrics

    def wait_for_camera_line_dock_alignment(
        self,
        dock_name: str,
        staging_target: str,
    ) -> tuple[bool, dict[str, Any]]:
        self.set_line_mode(True, "LINE_PRIMARY_ROUTE", profile="route_line", context=staging_target)
        deadline = time.monotonic() + self.camera_line_predock_timeout_sec
        last_metrics = self.camera_line_alignment_metrics(staging_target)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            ready, last_metrics = self.camera_line_ready_for_dock(staging_target)
            if ready:
                last_metrics["debug_image_topic"] = "/camera_line/debug_image"
                self.write_debug_artifact(
                    f"{dock_name.lower()}_camera_line_predock_alignment_ok.json",
                    last_metrics,
                )
                self.set_line_mode(False, "OFF", profile="route_line", context=staging_target)
                return True, last_metrics
        last_metrics["debug_image_topic"] = "/camera_line/debug_image"
        self.write_debug_artifact(
            f"{dock_name.lower()}_camera_line_predock_alignment_failed.json",
            last_metrics,
        )
        self.set_line_mode(False, "OFF", profile="route_line", context=staging_target)
        return False, last_metrics

    def line_primary_completion_reason(
        self,
        start_pose: tuple[float, float, float] | None,
        target_pose: tuple[float, float, float] | None,
        current_pose: tuple[float, float, float] | None,
    ) -> tuple[bool, str, dict[str, float | None]]:
        metrics = segment_progress(start_pose, target_pose, current_pose)
        remaining = metrics["remaining_m"]
        along = metrics["along_track_m"]
        cross = metrics["cross_track_m"]
        segment_length = metrics["segment_length_m"]
        current_target = ""
        line_status: dict[str, Any] = self.last_line_status if isinstance(self.last_line_status, dict) else {}
        if self.current_segment is not None:
            current_target = str(self.current_segment.get("target_name") or "").upper()
        camera_staging_target = (
            self.line_source == "camera_line"
            and current_target.endswith("_STAGING_CALIBRATED")
        )
        if camera_staging_target:
            ready, strict_metrics = self.camera_line_ready_for_staging(current_target)
            metrics.update({k: v for k, v in strict_metrics.items() if k not in metrics})
            if ready:
                return True, "camera_line_staging_strict", metrics
            return False, "", metrics

        if remaining is not None and remaining <= self.line_primary_distance_tolerance:
            return True, "distance_tolerance", metrics

        if along is None or cross is None or segment_length is None:
            return False, "", metrics

        corridor_progress_threshold = max(
            self.line_primary_min_progress_m,
            max(0.0, segment_length - self.line_primary_distance_tolerance),
        )
        in_corridor = abs(cross) <= self.line_primary_corridor_width_m

        if in_corridor and along >= corridor_progress_threshold:
            return True, "target_corridor", metrics

        if in_corridor and along >= segment_length:
            return True, "passed_waypoint", metrics

        return False, "", metrics

    def line_sensor_ready_for_staging(self, target_name: str) -> tuple[bool, dict[str, Any]]:
        route_pose = self.current_route_pose()
        target_pose = self.waypoint_pose_tuple(target_name)
        distance_to_target = pose_distance(route_pose, target_pose)
        status = self.last_line_sensor_status if isinstance(self.last_line_sensor_status, dict) else {}
        seen = bool(self.last_line_sensor_seen)
        position = status.get("position", self.last_line_sensor_position)
        error = status.get("error", -self.last_line_sensor_position)
        angular_cmd = status.get("angular_cmd")
        linear_cmd = status.get("linear_cmd")
        deadband = 2.0
        if isinstance(status, dict):
            try:
                deadband = abs(float(status.get("position_deadband", deadband)))
            except Exception:
                deadband = 2.0
        distance_tolerance_m = min(self.line_approach_distance_tolerance_m, 0.08)
        angular_cmd_limit = 0.12
        metrics = {
            "distance_to_target_m": distance_to_target,
            "strict_distance_tolerance_m": distance_tolerance_m,
            "line_sensor_seen": seen,
            "line_sensor_position": float(position) if isinstance(position, (int, float)) else None,
            "line_sensor_error": float(error) if isinstance(error, (int, float)) else None,
            "line_sensor_deadband": float(deadband),
            "line_sensor_angular_cmd": float(angular_cmd) if isinstance(angular_cmd, (int, float)) else None,
            "line_sensor_linear_cmd": float(linear_cmd) if isinstance(linear_cmd, (int, float)) else None,
            "line_sensor_angular_cmd_limit": angular_cmd_limit,
        }
        angular_ok = True
        if isinstance(angular_cmd, (int, float)):
            angular_ok = abs(float(angular_cmd)) <= angular_cmd_limit
        ready = bool(
            distance_to_target is not None
            and distance_to_target <= distance_tolerance_m
            and seen
            and isinstance(position, (int, float))
            and abs(float(position)) <= max(deadband, 3.0)
            and angular_ok
        )
        return ready, metrics

    def strict_line_approach_evaluation(
        self,
        progress_metrics: dict[str, float | None],
    ) -> tuple[bool, dict[str, float | None]]:
        status = self.last_pose_line_status if isinstance(self.last_pose_line_status, dict) else {}
        current_target = ""
        if self.current_segment is not None:
            current_target = str(self.current_segment.get("target_name") or "").upper()
        calibrated_staging = current_target.endswith("_STAGING_CALIBRATED")
        executor_remaining = progress_metrics.get("remaining_m")
        executor_along = progress_metrics.get("along_track_m")
        executor_segment_length = progress_metrics.get("segment_length_m")
        cross_track = status.get("cross_track_error_m")
        heading_error = status.get("heading_error_rad")
        controller_progress_m = status.get("progress_m")
        controller_progress_ratio = status.get("progress_ratio")
        controller_remaining = status.get("remaining_m")

        if controller_progress_ratio is None and controller_progress_m is not None:
            segment_length_for_ratio = status.get("segment_length_m")
            if segment_length_for_ratio not in (None, 0.0):
                controller_progress_ratio = float(controller_progress_m) / float(segment_length_for_ratio)

        executor_progress_ratio = None
        if executor_along is not None and executor_segment_length not in (None, 0.0):
            executor_progress_ratio = executor_along / executor_segment_length

        distance_tolerance_m = self.line_approach_distance_tolerance_m
        cross_track_tolerance_m = self.line_approach_cross_track_threshold_m
        heading_tolerance_rad = self.line_approach_heading_threshold_rad
        if calibrated_staging:
            distance_tolerance_m = min(distance_tolerance_m, 0.08)
            cross_track_tolerance_m = min(cross_track_tolerance_m, 0.04)
            heading_tolerance_rad = min(heading_tolerance_rad, 0.05)
        completion_flag = bool(status.get("completion_flag"))
        stop_reason = str(status.get("stop_reason") or "")

        criteria = {
            "distance_to_target_m": float(controller_remaining)
            if controller_remaining is not None
            else executor_remaining,
            "cross_track_error_m": float(cross_track) if cross_track is not None else None,
            "heading_error_rad": float(heading_error) if heading_error is not None else None,
            "progress_m": float(controller_progress_m) if controller_progress_m is not None else executor_along,
            "segment_length_m": status.get("segment_length_m")
            if status.get("segment_length_m") is not None
            else executor_segment_length,
            "progress_ratio": float(controller_progress_ratio)
            if controller_progress_ratio is not None
            else None,
            "executor_progress_m": executor_along,
            "executor_progress_ratio": executor_progress_ratio,
            "executor_distance_to_target_m": executor_remaining,
            "controller_progress_m": float(controller_progress_m) if controller_progress_m is not None else None,
            "controller_progress_ratio": float(controller_progress_ratio)
            if controller_progress_ratio is not None
            else None,
            "controller_remaining_m": float(controller_remaining)
            if controller_remaining is not None
            else None,
            "distance_ok": bool(
                (
                    controller_remaining is not None
                    and float(controller_remaining) <= distance_tolerance_m
                )
                or (
                    controller_remaining is None
                    and executor_remaining is not None
                    and executor_remaining <= distance_tolerance_m
                )
            ),
            "cross_track_ok": bool(
                cross_track is not None
                and abs(float(cross_track)) <= cross_track_tolerance_m
            ),
            "heading_ok": bool(
                heading_error is not None
                and abs(float(heading_error)) <= heading_tolerance_rad
            ),
            "progress_ok": bool(
                controller_progress_ratio is not None
                and float(controller_progress_ratio) >= self.line_approach_min_progress_ratio
            ),
            "strict_distance_tolerance_m": distance_tolerance_m,
            "strict_cross_track_tolerance_m": cross_track_tolerance_m,
            "strict_heading_tolerance_rad": heading_tolerance_rad,
            "completion_flag": completion_flag,
            "stop_reason": stop_reason,
        }
        strict_ok = bool(
            criteria["distance_ok"]
            and criteria["cross_track_ok"]
            and criteria["heading_ok"]
            and criteria["progress_ok"]
        )
        if (
            not strict_ok
            and calibrated_staging
            and completion_flag
            and stop_reason == "line_approach_complete"
            and criteria["distance_ok"]
            and heading_error is not None
            and abs(float(heading_error)) <= 0.07
            and cross_track is not None
            and abs(float(cross_track)) <= 0.05
        ):
            criteria["cross_track_ok"] = True
            criteria["strict_cross_track_tolerance_m"] = 0.05
            criteria["heading_ok"] = True
            criteria["strict_heading_tolerance_rad"] = 0.07
            strict_ok = True
        return strict_ok, criteria

    def line_approach_completion_reason(
        self,
        progress_metrics: dict[str, float | None],
    ) -> tuple[bool, str, dict[str, float | None]]:
        strict_ok, strict_metrics = self.strict_line_approach_evaluation(progress_metrics)
        if strict_ok:
            return True, "strict_line_approach", strict_metrics
        return False, "", strict_metrics

    def stop(self, duration_sec: float = 0.4) -> None:
        if not self.can_publish_stop():
            return
        msg = Twist()
        end = self.runtime_deadline(duration_sec)
        while rclpy.ok() and self.runtime_now() < end:
            try:
                self.cmd_pub.publish(msg)
            except Exception:
                return
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until_robot_stopped(
        self,
        *,
        stable_duration_sec: float = 0.5,
        timeout_sec: float = 2.0,
    ) -> tuple[bool, dict[str, float | None]]:
        stable_since: float | None = None
        last_pose: tuple[float, float, float] | None = None
        last_linear_cmd = math.nan
        last_angular_cmd = math.nan
        deadline = self.runtime_deadline(timeout_sec)
        while rclpy.ok() and self.runtime_now() < deadline:
            self.stop(duration_sec=0.05)
            rclpy.spin_once(self, timeout_sec=0.05)
            route_pose = self.current_fine_control_pose()
            linear_cmd = self.last_pose_line_status.get("linear_cmd")
            angular_cmd = self.last_pose_line_status.get("angular_cmd")
            last_linear_cmd = float(linear_cmd) if isinstance(linear_cmd, (int, float)) else math.nan
            last_angular_cmd = float(angular_cmd) if isinstance(angular_cmd, (int, float)) else math.nan
            cmd_stopped = (
                isinstance(linear_cmd, (int, float))
                and abs(float(linear_cmd)) <= self.dock_lift_cmd_linear_tolerance_mps
                and isinstance(angular_cmd, (int, float))
                and abs(float(angular_cmd)) <= self.dock_lift_cmd_angular_tolerance_radps
            )
            pose_stopped = False
            if route_pose is not None and last_pose is not None:
                dx = route_pose[0] - last_pose[0]
                dy = route_pose[1] - last_pose[1]
                dyaw = normalize_angle(route_pose[2] - last_pose[2])
                pose_stopped = math.hypot(dx, dy) <= 0.003 and abs(dyaw) <= 0.01
            last_pose = route_pose
            if pose_stopped:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif (time.monotonic() - stable_since) >= stable_duration_sec:
                    return True, {
                        "linear_cmd": last_linear_cmd,
                        "angular_cmd": last_angular_cmd,
                    }
            else:
                stable_since = None
        return False, {
            "linear_cmd": last_linear_cmd,
            "angular_cmd": last_angular_cmd,
        }

    def drive_distance(self, distance: float, speed: float) -> bool:
        if self.current_pose is None:
            self.get_logger().error("Cannot drive: no odom pose")
            return False
        start_x, start_y, _ = self.current_pose
        command = Twist()
        command.linear.x = speed
        deadline = self.runtime_deadline(max(abs(distance / max(abs(speed), 0.01)) * 3.0, 8.0))
        while rclpy.ok() and self.runtime_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.current_pose is None:
                continue
            dx = self.current_pose[0] - start_x
            dy = self.current_pose[1] - start_y
            if math.hypot(dx, dy) >= abs(distance):
                self.stop()
                return True
            self.cmd_pub.publish(command)
        self.stop()
        self.get_logger().error(f"Controlled drive timeout: target_distance={distance:.2f}")
        return False

    def wait_distance_without_publishing(self, distance: float, timeout_sec: float) -> bool:
        if self.current_pose is None:
            self.get_logger().error("Cannot monitor distance: no odom pose")
            return False
        start_x, start_y, _ = self.current_pose
        deadline = self.runtime_deadline(timeout_sec)
        while rclpy.ok() and self.runtime_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_pose is None:
                continue
            dx = self.current_pose[0] - start_x
            dy = self.current_pose[1] - start_y
            if math.hypot(dx, dy) >= abs(distance):
                return True
        self.get_logger().error(f"Distance monitor timeout: target_distance={distance:.2f}")
        return False

    def dock_line_error_metrics(self, name: str) -> dict[str, float]:
        metrics: dict[str, float] = {
            "cross_track_error_m": math.nan,
            "heading_error_rad": math.nan,
            "progress_ratio": math.nan,
        }
        if (
            isinstance(self.last_pose_line_status, dict)
            and str(self.last_pose_line_status.get("mode", "")).upper() == "DOCKING"
            and str(self.last_pose_line_status.get("segment", "")).upper() == str(name).upper()
        ):
            for key, source_key in (
                ("cross_track_error_m", "cross_track_error_m"),
                ("heading_error_rad", "heading_error_rad"),
                ("progress_ratio", "progress_ratio"),
            ):
                value = self.last_pose_line_status.get(source_key)
                if isinstance(value, (int, float)):
                    metrics[key] = float(value)
            return metrics
        route_pose = self.current_fine_control_pose()
        tracking = self.segment_tracking_metrics(name, route_pose)
        if tracking is not None:
            cross_track = tracking.get("cross_track_m")
            progress_ratio = tracking.get("progress_ratio")
            if isinstance(cross_track, (int, float)):
                metrics["cross_track_error_m"] = float(cross_track)
            if isinstance(progress_ratio, (int, float)):
                metrics["progress_ratio"] = float(progress_ratio)
        segment_info = self.segment_geometry(name)
        end_pose = segment_info.get("segment_end_pose")
        if route_pose is not None and end_pose is not None:
            metrics["heading_error_rad"] = normalize_angle(float(end_pose[2]) - float(route_pose[2]))
        return metrics

    def a1_dock_trace_sample(
        self,
        name: str,
        *,
        route_pose_override: tuple[float, float, float] | None = None,
    ) -> dict[str, Any]:
        route_pose = route_pose_override or self.current_fine_control_pose()
        line_metrics = self.dock_line_error_metrics(name)
        tracking = self.segment_tracking_metrics(name, route_pose) if route_pose is not None else None
        segment_info = self.segment_geometry(name)
        segment_end_pose = segment_info.get("segment_end_pose")
        if tracking is not None:
            cross_track = tracking.get("cross_track_m")
            progress_ratio = tracking.get("progress_ratio")
            if isinstance(cross_track, (int, float)):
                line_metrics["cross_track_error_m"] = float(cross_track)
            if isinstance(progress_ratio, (int, float)):
                line_metrics["progress_ratio"] = float(progress_ratio)
        if route_pose is not None and segment_end_pose is not None:
            line_metrics["heading_error_rad"] = normalize_angle(float(segment_end_pose[2]) - float(route_pose[2]))
        pose_line_status = dict(self.last_pose_line_status) if isinstance(self.last_pose_line_status, dict) else {}
        if route_pose is not None:
            pose_line_status.update(
                {
                    "mode": "DOCKING",
                    "segment": name,
                    "pose_x": float(route_pose[0]),
                    "pose_y": float(route_pose[1]),
                    "pose_yaw": float(route_pose[2]),
                    "cross_track_error_m": line_metrics.get("cross_track_error_m"),
                    "heading_error_rad": line_metrics.get("heading_error_rad"),
                    "progress_ratio": line_metrics.get("progress_ratio"),
                    "linear_cmd": 0.0,
                    "angular_cmd": 0.0,
                }
            )
        dock_ok, dock_reason, dock_metrics = self.evaluate_dock_geometry(
            name,
            pose_line_status_override=pose_line_status,
            require_controller_complete=False,
        )
        pallet_state = self.get_entity_state(
            self.pallet_model_entity_name(self.dock_station_name(name) or "") or "",
            timeout_sec=0.2,
        )
        pallet_rpy = (
            pallet_state.get("orientation_rpy")
            if isinstance(pallet_state, dict)
            else [math.nan, math.nan, math.nan]
        )
        pallet_pos = (
            pallet_state.get("position")
            if isinstance(pallet_state, dict)
            else [math.nan, math.nan, math.nan]
        )
        cmd = {
            "linear_x": float(self.last_a1_insert_cmd_linear),
            "angular_z": float(self.last_a1_insert_cmd_angular),
        }
        return {
            "t": time.monotonic(),
            "state": "",
            "robot_pose": list(route_pose) if route_pose is not None else None,
            "cmd_vel": cmd,
            "cross_track_error_m": line_metrics.get("cross_track_error_m"),
            "heading_error_rad": line_metrics.get("heading_error_rad"),
            "progress_ratio": line_metrics.get("progress_ratio"),
            "left_fork_insertion_depth_m": dock_metrics.get("left_fork_insertion_depth_m"),
            "right_fork_insertion_depth_m": dock_metrics.get("right_fork_insertion_depth_m"),
            "support_depth_to_load_center_m": dock_metrics.get("support_depth_to_load_center_m"),
            "pallet_lateral_error_m": dock_metrics.get("fork_tip_lateral_error_m"),
            "pallet_heading_error_rad": dock_metrics.get("fork_heading_error_rad"),
            "left_fork_tip_pose": dock_metrics.get("left_fork_tip_pose"),
            "right_fork_tip_pose": dock_metrics.get("right_fork_tip_pose"),
            "pallet_pose": dock_metrics.get("pallet_pose"),
            "pallet_roll_rad": float(pallet_rpy[0]) if len(pallet_rpy) >= 1 else math.nan,
            "pallet_pitch_rad": float(pallet_rpy[1]) if len(pallet_rpy) >= 2 else math.nan,
            "pallet_z_m": float(pallet_pos[2]) if len(pallet_pos) >= 3 else math.nan,
            "dock_stop_reason": pose_line_status.get("stop_reason"),
            "dock_geometry_ok": dock_ok,
            "dock_geometry_reason": dock_reason,
        }

    def log_a1_dock_trace_sample(self, name: str, sample: dict[str, Any], state: str) -> None:
        sample["state"] = state
        if self.current_segment is not None:
            self.current_segment.setdefault("a1_dock_samples", []).append(sample)
        robot_pose = sample.get("robot_pose") or [math.nan, math.nan, math.nan]
        cmd = sample.get("cmd_vel") or {}
        self.get_logger().info(
            "A1_DOCK_TRACE "
            f"{name}: state={state} "
            f"robot_pose=({float(robot_pose[0]):.4f},{float(robot_pose[1]):.4f},{float(robot_pose[2]):.4f}) "
            f"cmd_linear={float(cmd.get('linear_x', math.nan)):.4f} "
            f"cmd_angular={float(cmd.get('angular_z', math.nan)):.4f} "
            f"cross_track_error_m={float(sample.get('cross_track_error_m', math.nan)):.4f} "
            f"heading_error_rad={float(sample.get('heading_error_rad', math.nan)):.4f} "
            f"left_fork_insertion_depth_m={float(sample.get('left_fork_insertion_depth_m', math.nan)):.4f} "
            f"right_fork_insertion_depth_m={float(sample.get('right_fork_insertion_depth_m', math.nan)):.4f} "
            f"support_depth_to_load_center_m={float(sample.get('support_depth_to_load_center_m', math.nan)):.4f} "
            f"pallet_lateral_error_m={float(sample.get('pallet_lateral_error_m', math.nan)):.4f} "
            f"pallet_heading_error_rad={float(sample.get('pallet_heading_error_rad', math.nan)):.4f} "
            f"pallet_roll_rad={float(sample.get('pallet_roll_rad', math.nan)):.4f} "
            f"pallet_pitch_rad={float(sample.get('pallet_pitch_rad', math.nan)):.4f} "
            f"stop_reason={sample.get('dock_stop_reason')}"
        )

    def camera_dock_trace_sample(self, name: str) -> dict[str, Any]:
        route_pose = self.current_fine_control_pose()
        line_status = dict(self.last_line_status) if isinstance(self.last_line_status, dict) else {}
        pose_line_status = dict(self.last_pose_line_status) if isinstance(self.last_pose_line_status, dict) else {}
        dock_ok, dock_reason, dock_metrics = self.evaluate_dock_geometry(
            name,
            require_controller_complete=False,
        )
        return {
            "t": time.monotonic(),
            "robot_pose": list(route_pose) if route_pose is not None else None,
            "fork_tip_pose": dock_metrics.get("fork_tip_pose"),
            "left_fork_tip_pose": dock_metrics.get("left_fork_tip_pose"),
            "right_fork_tip_pose": dock_metrics.get("right_fork_tip_pose"),
            "pallet_opening_pose": dock_metrics.get("pallet_opening_pose"),
            "fork_tip_gap_m": dock_metrics.get("fork_tip_forward_gap_m"),
            "support_depth_to_load_center_m": dock_metrics.get("support_depth_to_load_center_m"),
            "left_fork_insertion_depth_m": dock_metrics.get("left_fork_insertion_depth_m"),
            "right_fork_insertion_depth_m": dock_metrics.get("right_fork_insertion_depth_m"),
            "pallet_lateral_error_m": dock_metrics.get("fork_tip_lateral_error_m"),
            "pallet_heading_error_rad": dock_metrics.get("fork_heading_error_rad"),
            "commanded_linear_velocity_mps": float(line_status.get("linear_cmd", 0.0)),
            "commanded_angular_velocity_radps": float(line_status.get("angular_cmd", 0.0)),
            "active_controller_source": "camera_line",
            "camera_line_status": line_status,
            "pose_line_status": pose_line_status,
            "dock_geometry_ok": dock_ok,
            "dock_geometry_reason": dock_reason,
        }

    def log_camera_dock_trace_sample(self, name: str, sample: dict[str, Any], state: str) -> None:
        sample["state"] = state
        if self.current_segment is not None:
            self.current_segment.setdefault("camera_dock_samples", []).append(sample)
        robot_pose = sample.get("robot_pose") or [math.nan, math.nan, math.nan]
        fork_tip_pose = sample.get("fork_tip_pose") or [math.nan, math.nan, math.nan]
        opening_pose = sample.get("pallet_opening_pose") or [math.nan, math.nan, math.nan]
        self.get_logger().info(
            "CAMERA_DOCK_TRACE "
            f"{name}: state={state} "
            f"robot_pose=({float(robot_pose[0]):.4f},{float(robot_pose[1]):.4f},{float(robot_pose[2]):.4f}) "
            f"fork_tip_pose=({float(fork_tip_pose[0]):.4f},{float(fork_tip_pose[1]):.4f},{float(fork_tip_pose[2]):.4f}) "
            f"opening_pose=({float(opening_pose[0]):.4f},{float(opening_pose[1]):.4f},{float(opening_pose[2]):.4f}) "
            f"fork_tip_gap_m={float(sample.get('fork_tip_gap_m', math.nan)):.4f} "
            f"support_depth_to_load_center_m={float(sample.get('support_depth_to_load_center_m', math.nan)):.4f} "
            f"cmd_linear={float(sample.get('commanded_linear_velocity_mps', math.nan)):.4f} "
            f"cmd_angular={float(sample.get('commanded_angular_velocity_radps', math.nan)):.4f} "
            f"controller={sample.get('active_controller_source')} "
            f"camera_error_px={float((sample.get('camera_line_status') or {}).get('filtered_error_px', math.nan)):.4f} "
            f"camera_heading_error={float((sample.get('camera_line_status') or {}).get('heading_error_rad', math.nan)):.4f} "
            f"dock_reason={sample.get('dock_geometry_reason')}"
        )

    def wait_for_camera_dock_progress(
        self,
        name: str,
        timeout_sec: float,
    ) -> tuple[bool, str, dict[str, Any]]:
        deadline = self.runtime_deadline(timeout_sec)
        start_gap = None
        best_gap = None
        best_support_depth = None
        last_sample: dict[str, Any] = {}
        last_log_time = 0.0
        worsening_samples = 0
        while rclpy.ok() and self.runtime_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            sample = self.camera_dock_trace_sample(name)
            last_sample = sample
            gap = sample.get("fork_tip_gap_m")
            support_depth = sample.get("support_depth_to_load_center_m")
            if isinstance(gap, (int, float)):
                gap = float(gap)
                if start_gap is None:
                    start_gap = gap
                if best_gap is None or gap < best_gap:
                    best_gap = gap
                    worsening_samples = 0
                elif best_gap is not None and gap > best_gap + 0.06:
                    worsening_samples += 1
            if isinstance(support_depth, (int, float)):
                support_depth = float(support_depth)
                if best_support_depth is None or support_depth > best_support_depth:
                    best_support_depth = support_depth
                elif best_support_depth is not None and support_depth < best_support_depth - 0.05:
                    worsening_samples += 1
            if (time.monotonic() - last_log_time) >= 0.2:
                self.log_camera_dock_trace_sample(name, dict(sample), "DOCKING")
                last_log_time = time.monotonic()
            if worsening_samples >= 3:
                sample["start_gap_m"] = start_gap
                sample["best_gap_m"] = best_gap
                sample["best_support_depth_m"] = best_support_depth
                self.write_debug_artifact(f"{name.lower()}_camera_dock_trace_failed.json", sample)
                return False, "DOCK_MOVING_AWAY_FROM_PALLET", sample
            if isinstance(best_support_depth, (int, float)) and float(best_support_depth) >= self.dock_lift_support_margin_m:
                sample["start_gap_m"] = start_gap
                sample["best_gap_m"] = best_gap
                sample["best_support_depth_m"] = best_support_depth
                self.write_debug_artifact(f"{name.lower()}_camera_dock_trace_ok.json", sample)
                return True, "dock_support_progress_ok", sample
        if last_sample:
            last_sample["start_gap_m"] = start_gap
            last_sample["best_gap_m"] = best_gap
            last_sample["best_support_depth_m"] = best_support_depth
            self.write_debug_artifact(f"{name.lower()}_camera_dock_trace_timeout.json", last_sample)
        return False, "dock_distance_timeout", last_sample

    def a1_insertion_metrics_ok(self, sample: dict[str, Any]) -> bool:
        cross_track = sample.get("cross_track_error_m")
        heading_error = sample.get("heading_error_rad")
        left_depth = sample.get("left_fork_insertion_depth_m")
        right_depth = sample.get("right_fork_insertion_depth_m")
        support_depth = sample.get("support_depth_to_load_center_m")
        pallet_lateral = sample.get("pallet_lateral_error_m")
        pallet_heading = sample.get("pallet_heading_error_rad")
        required_support_depth = max(
            float(self.a1_dock_creep_support_depth_target_m),
            float(self.dock_lift_support_margin_m),
        )
        min_insertion_depth = self.fork_min_insertion_depth_m
        return bool(
            isinstance(cross_track, (int, float))
            and abs(float(cross_track)) <= self.a1_dock_align_cross_track_tolerance_m
            and isinstance(heading_error, (int, float))
            and abs(float(heading_error)) <= self.a1_dock_align_heading_tolerance_rad
            and isinstance(left_depth, (int, float))
            and float(left_depth) >= min_insertion_depth
            and isinstance(right_depth, (int, float))
            and float(right_depth) >= min_insertion_depth
            and isinstance(support_depth, (int, float))
            and float(support_depth) >= required_support_depth
            and isinstance(pallet_lateral, (int, float))
            and abs(float(pallet_lateral)) <= self.dock_lift_lateral_tolerance_m
            and isinstance(pallet_heading, (int, float))
            and abs(float(pallet_heading)) <= self.dock_lift_heading_tolerance_rad
        )

    def a1_pre_lift_line_metrics_ok(self, sample: dict[str, Any]) -> bool:
        cross_track = sample.get("cross_track_error_m")
        heading_error = sample.get("heading_error_rad")
        return bool(
            isinstance(cross_track, (int, float))
            and abs(float(cross_track)) <= self.dock_lift_line_cross_track_tolerance_m
            and isinstance(heading_error, (int, float))
            and abs(float(heading_error)) <= self.dock_lift_line_heading_tolerance_rad
        )

    def a1_alignment_metrics_ok(self, sample: dict[str, Any]) -> bool:
        cross_track = sample.get("cross_track_error_m")
        heading_error = sample.get("heading_error_rad")
        pallet_lateral = sample.get("pallet_lateral_error_m")
        pallet_heading = sample.get("pallet_heading_error_rad")
        return bool(
            isinstance(cross_track, (int, float))
            and abs(float(cross_track)) <= self.a1_dock_align_cross_track_tolerance_m
            and isinstance(heading_error, (int, float))
            and abs(float(heading_error)) <= self.a1_dock_align_heading_tolerance_rad
            and isinstance(pallet_lateral, (int, float))
            and abs(float(pallet_lateral)) <= self.dock_lift_lateral_tolerance_m
            and isinstance(pallet_heading, (int, float))
            and abs(float(pallet_heading)) <= self.dock_lift_heading_tolerance_rad
        )

    def a1_alignment_recoverable(self, sample: dict[str, Any]) -> bool:
        cross_track = sample.get("cross_track_error_m")
        heading_error = sample.get("heading_error_rad")
        pallet_lateral = sample.get("pallet_lateral_error_m")
        pallet_heading = sample.get("pallet_heading_error_rad")
        if not isinstance(cross_track, (int, float)):
            return False
        if abs(float(cross_track)) > self.dock_line_cross_track_tolerance_m:
            return False
        if not isinstance(heading_error, (int, float)):
            return False
        if abs(float(heading_error)) > self.dock_line_heading_tolerance_rad:
            return False

        # During pallet insertion, pallet-referenced metrics can briefly disappear
        # or jitter even while the robot remains well-centered on the dock line.
        # Treat missing pallet metrics as recoverable so the controller does not
        # bounce back into a full realignment cycle on every transient.
        if isinstance(pallet_lateral, (int, float)) and abs(float(pallet_lateral)) > 0.08:
            return False
        if isinstance(pallet_heading, (int, float)) and abs(float(pallet_heading)) > 0.08:
            return False

        return True

    def a1_predock_line_recoverable(self, sample: dict[str, Any]) -> bool:
        cross_track = sample.get("cross_track_error_m")
        heading_error = sample.get("heading_error_rad")
        if not isinstance(cross_track, (int, float)):
            return False
        if abs(float(cross_track)) > 0.15:
            return False
        if not isinstance(heading_error, (int, float)):
            return False
        if abs(float(heading_error)) > 0.15:
            return False
        return True

    def a1_predock_insertable(self, sample: dict[str, Any]) -> bool:
        """Permit insertion when the dock approach is close enough to recover in-place.

        A-station predock realign is only worth attempting when the robot is clearly
        too skewed to start a controlled insertion. When the robot is already close
        to the dock line, forcing a full predock realign tends to oscillate between
        two nearby poses and never actually begin insertion.
        """

        cross_track = sample.get("cross_track_error_m")
        heading_error = sample.get("heading_error_rad")
        pallet_lateral = sample.get("pallet_lateral_error_m")
        pallet_heading = sample.get("pallet_heading_error_rad")
        return bool(
            isinstance(cross_track, (int, float))
            and abs(float(cross_track)) <= 0.05
            and isinstance(heading_error, (int, float))
            and abs(float(heading_error)) <= 0.06
            and isinstance(pallet_lateral, (int, float))
            and abs(float(pallet_lateral)) <= 0.065
            and isinstance(pallet_heading, (int, float))
            and abs(float(pallet_heading)) <= 0.06
        )

    def a1_inserted_lift_alignment_ok(self, metrics: dict[str, Any]) -> bool:
        support_depth = metrics.get("support_depth_to_load_center_m")
        lateral = metrics.get("fork_tip_lateral_error_m")
        heading_error = metrics.get("fork_heading_error_rad")
        dock_cross_track = metrics.get("dock_cross_track_error_m")
        dock_heading = metrics.get("dock_heading_error_rad")
        left_insertion_depth = metrics.get("left_fork_insertion_depth_m")
        right_insertion_depth = metrics.get("right_fork_insertion_depth_m")
        min_insertion_depth = self.pallet_opening_face_offset_m + self.dock_lift_support_margin_m
        return bool(
            isinstance(support_depth, (int, float))
            and float(support_depth) >= self.dock_lift_support_margin_m
            and isinstance(lateral, (int, float))
            and abs(float(lateral)) <= self.dock_lift_lateral_tolerance_m
            and isinstance(heading_error, (int, float))
            and abs(float(heading_error)) <= self.dock_lift_heading_tolerance_rad
            and isinstance(dock_cross_track, (int, float))
            and abs(float(dock_cross_track)) <= self.a1_dock_align_cross_track_tolerance_m
            and isinstance(dock_heading, (int, float))
            and abs(float(dock_heading)) <= self.a1_dock_align_heading_tolerance_rad
            and isinstance(left_insertion_depth, (int, float))
            and float(left_insertion_depth) >= min_insertion_depth
            and isinstance(right_insertion_depth, (int, float))
            and float(right_insertion_depth) >= min_insertion_depth
        )

    def a1_inserted_retry_not_needed(self, sample: dict[str, Any]) -> bool:
        """Allow lift-gate evaluation to decide once insertion is physically safe.

        After the forks are already inserted with adequate support, retrying the full
        alignment cycle tends to make the robot back out of a usable pose. This helper
        intentionally accepts a slightly looser dock-line envelope than the strict
        pre-lift line gate, while still requiring safe fork/pallet geometry.
        """

        if not self.a1_insertion_metrics_ok(sample):
            return False
        cross_track = sample.get("cross_track_error_m")
        heading_error = sample.get("heading_error_rad")
        if not isinstance(cross_track, (int, float)):
            return False
        if abs(float(cross_track)) > self.a1_dock_align_cross_track_tolerance_m:
            return False
        if not isinstance(heading_error, (int, float)):
            return False
        if abs(float(heading_error)) > self.a1_dock_align_heading_tolerance_rad:
            return False
        return True

    def a1_creep_insertion(self, name: str) -> tuple[bool, str, dict[str, Any]]:
        deadline = self.runtime_deadline(self.a1_dock_creep_timeout_sec)
        state = "ALIGN_ON_GREEN_LINE"
        last_sample: dict[str, Any] = {}
        last_log_time = 0.0
        insert_start_pose: tuple[float, float, float] | None = None
        alignment_hold_samples = 0
        hard_alignment_samples = 0
        segment_info = self.segment_geometry(name)
        start_pose = segment_info.get("segment_start_pose")
        end_pose = segment_info.get("segment_end_pose")
        if start_pose is None or end_pose is None:
            return False, "dock_segment_geometry_missing", {}
        dx = float(end_pose[0]) - float(start_pose[0])
        dy = float(end_pose[1]) - float(start_pose[1])
        seg_len = math.hypot(dx, dy)
        if seg_len <= 1e-6:
            return False, "dock_segment_length_invalid", {}
        ux = dx / seg_len
        uy = dy / seg_len
        initial_pose = self.pose_line_snapshot_pose_for_segment(name, max_age_sec=1.0) or self.current_fine_control_pose()
        initial_odom_pose = self.current_pose
        if initial_pose is None:
            return False, "robot_pose_missing", {}
        if initial_odom_pose is None:
            return False, "odom_pose_missing", {}
        forward_dot = math.cos(initial_pose[2]) * ux + math.sin(initial_pose[2]) * uy
        insertion_sign = 1.0 if forward_dot >= 0.0 else -1.0
        while rclpy.ok() and self.runtime_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            route_pose = self.a1_insert_control_pose(name, initial_pose, initial_odom_pose)
            last_sample = self.a1_dock_trace_sample(name, route_pose_override=route_pose)
            cross_track = last_sample.get("cross_track_error_m")
            heading_error = last_sample.get("heading_error_rad")
            pallet_lateral_error = last_sample.get("pallet_lateral_error_m")
            pallet_heading_error = last_sample.get("pallet_heading_error_rad")
            support_depth = last_sample.get("support_depth_to_load_center_m")
            if route_pose is None:
                state = "ROBOT_POSE_MISSING"
                continue
            if self.a1_insertion_metrics_ok(last_sample):
                state = "STOP_AND_VERIFY_SUPPORT"
            alignment_ok = self.a1_alignment_metrics_ok(last_sample)
            recoverable_alignment = self.a1_alignment_recoverable(last_sample)
            support_insufficient = (
                not isinstance(support_depth, (int, float))
                or float(support_depth) < max(
                    float(self.a1_dock_creep_support_depth_target_m),
                    float(self.dock_lift_support_margin_m),
                )
            )
            hard_alignment_failure = bool(
                (
                    isinstance(cross_track, (int, float))
                    and abs(float(cross_track)) > 0.08
                )
                or (
                    isinstance(heading_error, (int, float))
                    and abs(float(heading_error)) > 0.10
                )
                or (
                    isinstance(pallet_lateral_error, (int, float))
                    and abs(float(pallet_lateral_error)) > 0.08
                )
                or (
                    isinstance(pallet_heading_error, (int, float))
                    and abs(float(pallet_heading_error)) > 0.10
                )
            )
            if hard_alignment_failure:
                hard_alignment_samples += 1
            else:
                hard_alignment_samples = 0
            if hard_alignment_samples >= self.a1_dock_hard_fail_samples:
                self.stop()
                self.last_a1_insert_cmd_linear = 0.0
                self.last_a1_insert_cmd_angular = 0.0
                return False, "INSERTION_ALIGNMENT_FAILED", last_sample
            extra_creep_m = 0.0
            if insert_start_pose is not None:
                extra_creep_m = max(
                    0.0,
                    insertion_sign
                    * (
                        (route_pose[0] - insert_start_pose[0]) * ux
                        + (route_pose[1] - insert_start_pose[1]) * uy
                    ),
                )
            if extra_creep_m >= self.a1_dock_creep_max_extra_m and support_insufficient:
                self.stop()
                self.last_a1_insert_cmd_linear = 0.0
                self.last_a1_insert_cmd_angular = 0.0
                return False, "INSUFFICIENT_FORK_INSERTION", last_sample
            heading_term = 0.0
            if isinstance(heading_error, (int, float)):
                heading_term = self.a1_dock_creep_k_heading * float(heading_error)
            cross_term = 0.0
            if isinstance(cross_track, (int, float)):
                cross_term = -self.a1_dock_creep_k_cross * float(cross_track)
            pallet_lateral_term = 0.0
            if isinstance(pallet_lateral_error, (int, float)):
                pallet_lateral_term = -self.a1_dock_creep_k_pallet_lateral * float(pallet_lateral_error)
            pallet_heading_term = 0.0
            if isinstance(pallet_heading_error, (int, float)):
                pallet_heading_term = -self.a1_dock_creep_k_pallet_heading * float(pallet_heading_error)
            angular_cmd = max(
                -self.a1_dock_creep_max_angular_radps,
                min(
                    self.a1_dock_creep_max_angular_radps,
                    heading_term + cross_term + pallet_lateral_term + pallet_heading_term,
                ),
            )
            command = Twist()
            if state == "ALIGN_ON_GREEN_LINE":
                if alignment_ok:
                    alignment_hold_samples += 1
                    if alignment_hold_samples >= self.a1_dock_align_hold_samples:
                        state = "INSERT_STRAIGHT_UNTIL_SUPPORT"
                        insert_start_pose = route_pose
                        alignment_hold_samples = 0
                else:
                    alignment_hold_samples = 0
                command.linear.x = 0.0
                command.angular.z = max(-0.02, min(0.02, angular_cmd))
            elif state == "INSERT_STRAIGHT_UNTIL_SUPPORT":
                if not recoverable_alignment:
                    state = "ALIGN_ON_GREEN_LINE"
                    alignment_hold_samples = 0
                    command.linear.x = 0.0
                    command.angular.z = 0.0
                elif support_insufficient:
                    # Keep creeping while alignment is still recoverable. Dropping back to
                    # full realignment on every small heading drift causes the robot to
                    # wag left/right near the pallet instead of completing insertion.
                    command.linear.x = insertion_sign * min(self.a1_dock_creep_speed_mps, 0.015)
                    command.angular.z = max(-0.01, min(0.01, angular_cmd))
                else:
                    state = "STOP_AND_VERIFY_SUPPORT"
                    command.linear.x = 0.0
                    command.angular.z = 0.0
            else:
                command.linear.x = 0.0
                command.angular.z = 0.0
            if hard_alignment_failure:
                command.linear.x = 0.0
            if (time.monotonic() - last_log_time) >= 0.2:
                self.log_a1_dock_trace_sample(name, dict(last_sample), state)
                last_log_time = time.monotonic()
            if state == "STOP_AND_VERIFY_SUPPORT":
                self.stop()
                self.last_a1_insert_cmd_linear = 0.0
                self.last_a1_insert_cmd_angular = 0.0
                stopped_ok, stop_metrics = self.wait_until_robot_stopped()
                verify_pose = self.a1_insert_control_pose(name, initial_pose, initial_odom_pose)
                verify_sample = self.a1_dock_trace_sample(name, route_pose_override=verify_pose)
                verify_sample["final_stop_linear_cmd_mps"] = stop_metrics.get("linear_cmd")
                verify_sample["final_stop_angular_cmd_radps"] = stop_metrics.get("angular_cmd")
                verify_sample["robot_stopped_after_insertion_ok"] = stopped_ok
                self.log_a1_dock_trace_sample(name, dict(verify_sample), "STOP_AND_VERIFY_SUPPORT")
                if stopped_ok and self.a1_insertion_metrics_ok(verify_sample):
                    if self.a1_pre_lift_line_metrics_ok(verify_sample):
                        verify_sample["state"] = "STOP_AND_VERIFY_SUPPORT"
                        return True, "INSERTION_OK", verify_sample
                    if self.a1_inserted_retry_not_needed(verify_sample):
                        verify_sample["state"] = "STOP_AND_VERIFY_SUPPORT"
                        verify_sample["near_prelift_line_gate_override"] = True
                        return True, "INSERTION_OK_NEAR_PRELIFT_GATE", verify_sample
                    verify_sample["state"] = "STOP_AND_VERIFY_SUPPORT"
                    verify_sample["realign_retry_stage"] = "strict_pre_lift_line_gate_failed"
                    return False, "INSERTION_REQUIRES_REALIGN_RETRY", verify_sample
                if not self.a1_alignment_recoverable(verify_sample):
                    return False, "INSERTION_ALIGNMENT_FAILED", verify_sample
                state = "ALIGN_ON_GREEN_LINE"
                alignment_hold_samples = 0
                last_sample = verify_sample
                continue
            self.last_a1_insert_cmd_linear = float(command.linear.x)
            self.last_a1_insert_cmd_angular = float(command.angular.z)
            self.cmd_pub.publish(command)
        self.stop()
        self.last_a1_insert_cmd_linear = 0.0
        self.last_a1_insert_cmd_angular = 0.0
        if last_sample:
            if not self.a1_insertion_metrics_ok(last_sample):
                if isinstance(last_sample.get("support_depth_to_load_center_m"), (int, float)):
                    if float(last_sample["support_depth_to_load_center_m"]) < max(
                        float(self.a1_dock_creep_support_depth_target_m),
                        float(self.dock_lift_support_margin_m),
                    ):
                        return False, "INSUFFICIENT_FORK_INSERTION", last_sample
                if not self.a1_alignment_metrics_ok(last_sample):
                    return False, "INSERTION_ALIGNMENT_FAILED", last_sample
                return False, "INSERTION_ALIGNMENT_FAILED", last_sample
        return False, "dock_creep_timeout", last_sample

    def dock_station_name(self, name: str) -> str | None:
        dock_name = str(name).upper()
        if not is_dock_target(dock_name):
            return None
        return dock_name.split("_DOCK", 1)[0]

    def fork_tip_center_pose(self, pose: tuple[float, float, float]) -> tuple[float, float]:
        x, y, yaw = pose
        return (
            x + math.cos(yaw) * (-self.fork_tip_center_offset_m),
            y + math.sin(yaw) * (-self.fork_tip_center_offset_m),
        )

    def fork_tip_pair_pose(
        self,
        pose: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
        center_x, center_y = self.fork_tip_center_pose(pose)
        fork_heading = normalize_angle(pose[2] + math.pi)
        lateral_x = -math.sin(fork_heading)
        lateral_y = math.cos(fork_heading)
        half_spacing = self.fork_spacing_m / 2.0
        center_tip = (center_x, center_y, fork_heading)
        left_tip = (
            center_x + lateral_x * half_spacing,
            center_y + lateral_y * half_spacing,
            fork_heading,
        )
        right_tip = (
            center_x - lateral_x * half_spacing,
            center_y - lateral_y * half_spacing,
            fork_heading,
        )
        return center_tip, left_tip, right_tip

    def fork_support_metrics(
        self,
        pose: tuple[float, float, float],
        pallet_pose: tuple[float, float, float],
    ) -> dict[str, Any]:
        pallet_x, pallet_y, pallet_yaw = pallet_pose
        center_tip, left_tip, right_tip = self.fork_tip_pair_pose(pose)
        opening_x = pallet_x - math.cos(pallet_yaw) * self.pallet_opening_face_offset_m
        opening_y = pallet_y - math.sin(pallet_yaw) * self.pallet_opening_face_offset_m

        def tip_metrics(tip: tuple[float, float, float], expected_lateral: float) -> dict[str, float]:
            dx_open = tip[0] - opening_x
            dy_open = tip[1] - opening_y
            along_open = dx_open * math.cos(pallet_yaw) + dy_open * math.sin(pallet_yaw)
            dx_center = tip[0] - pallet_x
            dy_center = tip[1] - pallet_y
            lateral_center = -dx_center * math.sin(pallet_yaw) + dy_center * math.cos(pallet_yaw)
            forward_gap = -along_open
            return {
                "forward_gap_m": forward_gap,
                "support_depth_to_load_center_m": -forward_gap - self.pallet_opening_face_offset_m,
                "lateral_center_m": lateral_center,
                "track_error_m": lateral_center - expected_lateral,
            }

        center_metrics = tip_metrics(center_tip, 0.0)
        left_metrics = tip_metrics(left_tip, self.fork_spacing_m / 2.0)
        right_metrics = tip_metrics(right_tip, -(self.fork_spacing_m / 2.0))
        fork_heading = center_tip[2]
        heading_error = normalize_angle(fork_heading - pallet_yaw)
        return {
            "fork_tip_pose": [center_tip[0], center_tip[1], center_tip[2]],
            "left_fork_tip_pose": [left_tip[0], left_tip[1], left_tip[2]],
            "right_fork_tip_pose": [right_tip[0], right_tip[1], right_tip[2]],
            "pallet_opening_pose": [opening_x, opening_y, pallet_yaw],
            "fork_tip_forward_gap_m": center_metrics["forward_gap_m"],
            "support_depth_to_load_center_m": min(
                left_metrics["support_depth_to_load_center_m"],
                right_metrics["support_depth_to_load_center_m"],
            ),
            "fork_tip_lateral_error_m": center_metrics["lateral_center_m"],
            "fork_heading_error_rad": heading_error,
            "fork_tip_to_opening_distance_m": math.hypot(
                center_metrics["forward_gap_m"],
                center_metrics["lateral_center_m"],
            ),
            "left_fork_insertion_depth_m": -left_metrics["forward_gap_m"],
            "right_fork_insertion_depth_m": -right_metrics["forward_gap_m"],
            "left_support_depth_to_load_center_m": left_metrics["support_depth_to_load_center_m"],
            "right_support_depth_to_load_center_m": right_metrics["support_depth_to_load_center_m"],
            "left_fork_lateral_error_m": left_metrics["track_error_m"],
            "right_fork_lateral_error_m": right_metrics["track_error_m"],
            "left_fork_lateral_center_m": left_metrics["lateral_center_m"],
            "right_fork_lateral_center_m": right_metrics["lateral_center_m"],
        }

    def write_debug_artifact(self, filename: str, payload: dict[str, Any]) -> str | None:
        if self.output_dir is None:
            return None
        path = self.output_dir / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
        return str(path)

    def segment_tracking_metrics(
        self,
        segment_name: str | None,
        pose: tuple[float, float, float] | None = None,
    ) -> dict[str, float] | None:
        segment_info = self.segment_geometry(segment_name)
        start_pose = segment_info.get("segment_start_pose")
        end_pose = segment_info.get("segment_end_pose")
        route_pose = pose or self.current_route_pose()
        if route_pose is None or start_pose is None or end_pose is None:
            return None
        dx = end_pose[0] - start_pose[0]
        dy = end_pose[1] - start_pose[1]
        seg_len = math.hypot(dx, dy)
        if seg_len <= 1e-6:
            return None
        ux = dx / seg_len
        uy = dy / seg_len
        px = route_pose[0] - start_pose[0]
        py = route_pose[1] - start_pose[1]
        along = px * ux + py * uy
        cross = px * (-uy) + py * ux
        return {
            "segment_length_m": seg_len,
            "along_track_m": along,
            "cross_track_m": cross,
            "progress_ratio": along / seg_len,
        }

    def dock_pose_line_snapshot_pose(
        self,
        name: str,
        pose_line_status: dict[str, Any] | None,
        *,
        require_recent: bool,
    ) -> tuple[float, float, float] | None:
        if not isinstance(pose_line_status, dict):
            return None
        if require_recent and (time.monotonic() - self.last_pose_line_status_time) > 1.0:
            return None
        if str(pose_line_status.get("mode", "")).upper() != "DOCKING":
            return None
        if str(pose_line_status.get("segment", "")).upper() != str(name).upper():
            return None
        status_pose = (
            pose_line_status.get("pose_x"),
            pose_line_status.get("pose_y"),
            pose_line_status.get("pose_yaw"),
        )
        if not all(isinstance(value, (int, float)) for value in status_pose):
            return None
        return (float(status_pose[0]), float(status_pose[1]), float(status_pose[2]))

    def evaluate_dock_geometry(
        self,
        name: str,
        *,
        pose_line_status_override: dict[str, Any] | None = None,
        require_controller_complete: bool = True,
    ) -> tuple[bool, str, dict[str, Any]]:
        station = self.dock_station_name(name)
        if station is None:
            return True, "dock_geometry_not_required", {}
        if station.startswith("B"):
            pose = self.current_fine_control_pose()
            status_pose = self.dock_pose_line_snapshot_pose(
                name,
                pose_line_status_override,
                require_recent=False,
            )
            if status_pose is None:
                status_pose = self.dock_pose_line_snapshot_pose(
                    name,
                    self.last_pose_line_status,
                    require_recent=True,
                )
            if status_pose is not None:
                pose = status_pose
            target_pose = self.waypoint_pose_tuple(name)
            tracking = self.segment_tracking_metrics(name, pose=pose) if pose is not None else None
            if pose is None or target_pose is None or tracking is None:
                return False, "dock_pose_missing", {}
            distance_to_target = pose_distance(pose, target_pose)
            heading_error = normalize_angle(float(target_pose[2]) - float(pose[2]))
            metrics = {
                "dock_cross_track_error_m": float(tracking.get("cross_track_m", math.nan)),
                "dock_heading_error_rad": float(heading_error),
                "distance_to_target_m": float(distance_to_target) if distance_to_target is not None else math.nan,
                "progress_ratio": float(tracking.get("progress_ratio", math.nan)),
                "robot_pose": list(pose),
                "target_pose": list(target_pose),
            }
            if require_controller_complete:
                completion_flag = bool((pose_line_status_override or {}).get("completion_flag"))
                if not completion_flag:
                    return False, "dock_controller_not_complete", metrics
            if distance_to_target is None or float(distance_to_target) > self.b_dock_pose_distance_tolerance_m:
                return False, "B_DOCK_DISTANCE_TOO_LARGE", metrics
            if abs(float(tracking.get("cross_track_m", math.nan))) > self.b_dock_cross_track_tolerance_m:
                return False, "B_DOCK_CROSSTRACK_TOO_LARGE", metrics
            if abs(float(heading_error)) > self.b_dock_heading_tolerance_rad:
                return False, "B_DOCK_HEADING_TOO_LARGE", metrics
            return True, "b_dock_pose_ready", metrics
        if not station.startswith("A"):
            return True, "dock_geometry_not_required", {}
        pallet = self.mission_points.get(f"PALLET_{station}")
        pose = self.current_fine_control_pose()
        status_pose = self.dock_pose_line_snapshot_pose(
            name,
            pose_line_status_override,
            require_recent=False,
        )
        if status_pose is None:
            status_pose = self.dock_pose_line_snapshot_pose(
                name,
                self.last_pose_line_status,
                require_recent=True,
            )
        if status_pose is not None:
            pose = status_pose
        if pallet is None:
            return False, "pallet_pose_missing", {}
        if pose is None:
            return False, "robot_pose_missing", {}
        pallet_yaw = float(pallet["yaw"])
        support_metrics = self.fork_support_metrics(
            pose,
            (float(pallet["x"]), float(pallet["y"]), pallet_yaw),
        )
        forward_gap = float(support_metrics["fork_tip_forward_gap_m"])
        lateral = float(support_metrics["fork_tip_lateral_error_m"])
        heading_error = float(support_metrics["fork_heading_error_rad"])
        metrics = {
            "robot_pose": [pose[0], pose[1], pose[2]],
            "pallet_pose": [float(pallet["x"]), float(pallet["y"]), pallet_yaw],
        }
        metrics.update(support_metrics)
        pose_line_status = (
            pose_line_status_override
            if isinstance(pose_line_status_override, dict)
            else self.last_pose_line_status
            if isinstance(self.last_pose_line_status, dict)
            else {}
        )
        if (
            self.dock_pose_line_snapshot_pose(
                name,
                pose_line_status,
                require_recent=pose_line_status is not pose_line_status_override,
            )
            is not None
        ):
            metrics["dock_cross_track_error_m"] = pose_line_status.get("cross_track_error_m")
            metrics["dock_heading_error_rad"] = pose_line_status.get("heading_error_rad")
            metrics["dock_progress_ratio"] = pose_line_status.get("progress_ratio")
            metrics["dock_completion_flag"] = pose_line_status.get("completion_flag")
            metrics["dock_stop_reason"] = pose_line_status.get("stop_reason")
            metrics["dock_linear_cmd_mps"] = pose_line_status.get("linear_cmd")
            metrics["dock_angular_cmd_radps"] = pose_line_status.get("angular_cmd")
            cross_track = pose_line_status.get("cross_track_error_m")
            dock_heading = pose_line_status.get("heading_error_rad")
            progress_ratio = pose_line_status.get("progress_ratio")
            completion_flag = bool(pose_line_status.get("completion_flag"))
            stop_reason = str(pose_line_status.get("stop_reason", ""))
            if isinstance(cross_track, (int, float)) and abs(float(cross_track)) > self.dock_line_cross_track_tolerance_m:
                return False, "dock_cross_track_too_large", metrics
            if isinstance(dock_heading, (int, float)) and abs(float(dock_heading)) > self.dock_line_heading_tolerance_rad:
                return False, "dock_heading_error_too_large", metrics
            if isinstance(progress_ratio, (int, float)) and float(progress_ratio) < 0.95:
                return False, "dock_progress_incomplete", metrics
            if require_controller_complete and (not completion_flag or stop_reason != "dock_geometry_complete"):
                return False, "dock_controller_not_complete", metrics
        if forward_gap > self.dock_fork_gap_tolerance_m:
            return False, "fork_tip_gap_too_large", metrics
        if abs(lateral) > self.dock_fork_lateral_tolerance_m:
            return False, "fork_tip_lateral_error_too_large", metrics
        if abs(heading_error) > self.dock_fork_heading_tolerance_rad:
            return False, "fork_heading_error_too_large", metrics
        return True, "dock_geometry_ok", metrics

    def evaluate_lift_ready_geometry(
        self,
        name: str,
        *,
        pose_line_status_override: dict[str, Any] | None = None,
        require_controller_complete: bool = True,
    ) -> tuple[bool, str, dict[str, Any]]:
        ok, reason, metrics = self.evaluate_dock_geometry(
            name,
            pose_line_status_override=pose_line_status_override,
            require_controller_complete=require_controller_complete,
        )
        if not ok:
            return False, "DOCK_ALIGNMENT_NOT_SAFE", metrics
        forward_gap = metrics.get("fork_tip_forward_gap_m")
        support_depth = metrics.get("support_depth_to_load_center_m")
        lateral = metrics.get("fork_tip_lateral_error_m")
        heading_error = metrics.get("fork_heading_error_rad")
        dock_cross_track = metrics.get("dock_cross_track_error_m")
        dock_heading = metrics.get("dock_heading_error_rad")
        left_insertion_depth = metrics.get("left_fork_insertion_depth_m")
        right_insertion_depth = metrics.get("right_fork_insertion_depth_m")
        left_track_error = metrics.get("left_fork_lateral_error_m")
        right_track_error = metrics.get("right_fork_lateral_error_m")
        linear_cmd = metrics.get("dock_linear_cmd_mps")
        angular_cmd = metrics.get("dock_angular_cmd_radps")
        if (
            not isinstance(dock_cross_track, (int, float))
            or abs(float(dock_cross_track)) > self.dock_lift_line_cross_track_tolerance_m
            or not isinstance(dock_heading, (int, float))
            or abs(float(dock_heading)) > self.dock_lift_line_heading_tolerance_rad
        ):
            return False, "LINE_ALIGNMENT_FAILED", metrics
        if (
            not isinstance(forward_gap, (int, float))
            or float(forward_gap) > self.dock_lift_gap_tolerance_m
            or not isinstance(support_depth, (int, float))
            or float(support_depth) < self.dock_lift_support_margin_m
        ):
            return False, "INSUFFICIENT_FORK_INSERTION", metrics
        min_insertion_depth = self.pallet_opening_face_offset_m + self.dock_lift_support_margin_m
        if (
            not isinstance(left_insertion_depth, (int, float))
            or float(left_insertion_depth) < min_insertion_depth
            or not isinstance(right_insertion_depth, (int, float))
            or float(right_insertion_depth) < min_insertion_depth
        ):
            return False, "INSUFFICIENT_FORK_INSERTION", metrics
        if not isinstance(lateral, (int, float)) or abs(float(lateral)) > self.dock_lift_lateral_tolerance_m:
            return False, "DOCK_ALIGNMENT_NOT_SAFE", metrics
        if not isinstance(heading_error, (int, float)) or abs(float(heading_error)) > self.dock_lift_heading_tolerance_rad:
            return False, "DOCK_ALIGNMENT_NOT_SAFE", metrics
        if (
            not isinstance(left_track_error, (int, float))
            or abs(float(left_track_error)) > self.dock_lift_fork_track_tolerance_m
            or not isinstance(right_track_error, (int, float))
            or abs(float(right_track_error)) > self.dock_lift_fork_track_tolerance_m
        ):
            return False, "DOCK_ALIGNMENT_NOT_SAFE", metrics
        if (
            not isinstance(linear_cmd, (int, float))
            or abs(float(linear_cmd)) > self.dock_lift_cmd_linear_tolerance_mps
            or not isinstance(angular_cmd, (int, float))
            or abs(float(angular_cmd)) > self.dock_lift_cmd_angular_tolerance_radps
        ):
            return False, "DOCK_NOT_STOPPED", metrics
        return True, "dock_lift_ready", metrics

    def dock_diagnostic_snapshot(
        self,
        name: str,
        *,
        geometry_ok: bool | None = None,
        geometry_reason: str | None = None,
        geometry: dict[str, Any] | None = None,
        pose_line_status: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        if geometry_ok is None or geometry_reason is None or geometry is None:
            geometry_ok, geometry_reason, geometry = self.evaluate_dock_geometry(name)
        if pose_line_status is None:
            pose_line_status = self.last_pose_line_status if isinstance(self.last_pose_line_status, dict) else {}
        segment_name = str(name).upper()
        segment_info = {}
        if segment_name in self.line_segments.get("dock_segments", {}):
            segment = self.line_segments["dock_segments"][segment_name]
            start_name = segment["start"]
            end_name = segment["end"]
            start_pose = self.waypoint_pose_tuple(start_name)
            end_pose = self.waypoint_pose_tuple(end_name)
            segment_info = {
                "segment_name": segment_name,
                "segment_start_waypoint": start_name,
                "segment_end_waypoint": end_name,
                "segment_start_pose": list(start_pose),
                "segment_end_pose": list(end_pose),
            }
        return {
            "dock_geometry_ok": geometry_ok,
            "dock_geometry_reason": geometry_reason,
            "dock_geometry": geometry,
            "pose_line_status": pose_line_status,
            "active_cmd_vel": {
                "linear": pose_line_status.get("linear_cmd"),
                "angular": pose_line_status.get("angular_cmd"),
            },
            "stop_reason": pose_line_status.get("stop_reason"),
            **segment_info,
        }

    def log_dock_preflight(self, name: str) -> None:
        route_pose = self.current_fine_control_pose()
        segment_info = self.segment_geometry(name)
        dock_ok, dock_reason, dock_metrics = self.evaluate_dock_geometry(name)
        tracking = self.segment_tracking_metrics(name, route_pose) or {}
        start_pose = segment_info.get("segment_start_pose")
        end_pose = segment_info.get("segment_end_pose")
        if route_pose is None or start_pose is None or end_pose is None:
            self.get_logger().info(
                f"DOCK_PREFLIGHT {name}: missing_pose_or_segment "
                f"robot_pose={route_pose} segment={segment_info}"
            )
            return
        fork_tip_pose = dock_metrics.get("fork_tip_pose") or [math.nan, math.nan, math.nan]
        pallet_pose = dock_metrics.get("pallet_pose") or [math.nan, math.nan, math.nan]
        opening_pose = dock_metrics.get("pallet_opening_pose") or [math.nan, math.nan, math.nan]
        self.get_logger().info(
            "DOCK_PREFLIGHT "
            f"{name}: segment={segment_info.get('segment_name')} "
            f"start={segment_info.get('segment_start_waypoint')}({start_pose[0]:.4f},{start_pose[1]:.4f},{start_pose[2]:.4f}) "
            f"end={segment_info.get('segment_end_waypoint')}({end_pose[0]:.4f},{end_pose[1]:.4f},{end_pose[2]:.4f}) "
            f"robot_pose=({route_pose[0]:.4f},{route_pose[1]:.4f},{route_pose[2]:.4f}) "
            f"fork_tip=({float(fork_tip_pose[0]):.4f},{float(fork_tip_pose[1]):.4f},{float(fork_tip_pose[2]):.4f}) "
            f"pallet_pose=({float(pallet_pose[0]):.4f},{float(pallet_pose[1]):.4f},{float(pallet_pose[2]):.4f}) "
            f"opening_pose=({float(opening_pose[0]):.4f},{float(opening_pose[1]):.4f},{float(opening_pose[2]):.4f}) "
            f"green_line_center_error_m={float(tracking.get('cross_track_m', math.nan)):.4f} "
            f"green_line_progress={float(tracking.get('progress_ratio', math.nan)):.4f} "
            f"fork_to_opening_m={float(dock_metrics.get('fork_tip_to_opening_distance_m', math.nan)):.4f} "
            f"fork_forward_gap_m={float(dock_metrics.get('fork_tip_forward_gap_m', math.nan)):.4f} "
            f"fork_lateral_error_m={float(dock_metrics.get('fork_tip_lateral_error_m', math.nan)):.4f} "
            f"fork_heading_error_rad={float(dock_metrics.get('fork_heading_error_rad', math.nan)):.4f} "
            f"geometry_ok={dock_ok} reason={dock_reason}"
        )

    def log_lift_gate(
        self,
        name: str,
        *,
        dock_geometry_reason: str,
        lift_reason: str,
        lift_metrics: dict[str, Any],
        pose_line_status: dict[str, Any] | None,
        pre_lift_pallet_state: dict[str, Any] | None = None,
    ) -> None:
        pose_line_status = pose_line_status if isinstance(pose_line_status, dict) else {}
        robot_pose = lift_metrics.get("robot_pose") or [math.nan, math.nan, math.nan]
        fork_tip_pose = lift_metrics.get("fork_tip_pose") or [math.nan, math.nan, math.nan]
        left_fork_tip_pose = lift_metrics.get("left_fork_tip_pose") or [math.nan, math.nan, math.nan]
        right_fork_tip_pose = lift_metrics.get("right_fork_tip_pose") or [math.nan, math.nan, math.nan]
        opening_pose = lift_metrics.get("pallet_opening_pose") or [math.nan, math.nan, math.nan]
        pallet_pose = lift_metrics.get("pallet_pose") or [math.nan, math.nan, math.nan]
        pallet_pos = (
            pre_lift_pallet_state.get("position")
            if isinstance(pre_lift_pallet_state, dict)
            else [math.nan, math.nan, math.nan]
        )
        debug_payload = {
            "segment": name,
            "dock_geometry_reason": dock_geometry_reason,
            "lift_ready_reason": lift_reason,
            "pose_line_status": pose_line_status,
            "lift_metrics": lift_metrics,
            "fork_state": self.last_fork_state,
            "pre_lift_pallet_state": pre_lift_pallet_state,
        }
        debug_path = self.write_debug_artifact(f"{name.lower()}_pre_lift_debug.json", debug_payload)
        if debug_path is not None:
            lift_metrics["pre_lift_debug_artifact"] = debug_path
        self.get_logger().info(
            "A1_LIFT_GATE "
            f"{name}: dock_geometry_reason={dock_geometry_reason} "
            f"lift_ready_reason={lift_reason} "
            f"pose_line_stop_reason={pose_line_status.get('stop_reason')} "
            f"pose_line_completion_flag={pose_line_status.get('completion_flag')} "
            f"fork_tip_forward_gap_m={float(lift_metrics.get('fork_tip_forward_gap_m', math.nan)):.4f} "
            f"support_depth_to_load_center_m={float(lift_metrics.get('support_depth_to_load_center_m', math.nan)):.4f} "
            f"left_fork_insertion_depth_m={float(lift_metrics.get('left_fork_insertion_depth_m', math.nan)):.4f} "
            f"right_fork_insertion_depth_m={float(lift_metrics.get('right_fork_insertion_depth_m', math.nan)):.4f} "
            f"left_fork_track_error_m={float(lift_metrics.get('left_fork_lateral_error_m', math.nan)):.4f} "
            f"right_fork_track_error_m={float(lift_metrics.get('right_fork_lateral_error_m', math.nan)):.4f} "
            f"lateral_error_m={float(lift_metrics.get('fork_tip_lateral_error_m', math.nan)):.4f} "
            f"heading_error_rad={float(lift_metrics.get('fork_heading_error_rad', math.nan)):.4f} "
            f"cross_track_error_m={float(pose_line_status.get('cross_track_error_m', math.nan)):.4f} "
            f"pose_line_heading_error_rad={float(pose_line_status.get('heading_error_rad', math.nan)):.4f} "
            f"linear_cmd_mps={float(pose_line_status.get('linear_cmd', math.nan)):.4f} "
            f"angular_cmd_radps={float(pose_line_status.get('angular_cmd', math.nan)):.4f} "
            f"robot_pose=({float(robot_pose[0]):.4f},{float(robot_pose[1]):.4f},{float(robot_pose[2]):.4f}) "
            f"fork_tip_pose=({float(fork_tip_pose[0]):.4f},{float(fork_tip_pose[1]):.4f},{float(fork_tip_pose[2]):.4f}) "
            f"left_fork_tip_pose=({float(left_fork_tip_pose[0]):.4f},{float(left_fork_tip_pose[1]):.4f},{float(left_fork_tip_pose[2]):.4f}) "
            f"right_fork_tip_pose=({float(right_fork_tip_pose[0]):.4f},{float(right_fork_tip_pose[1]):.4f},{float(right_fork_tip_pose[2]):.4f}) "
            f"pallet_pose=({float(pallet_pose[0]):.4f},{float(pallet_pose[1]):.4f},{float(pallet_pose[2]):.4f}) "
            f"opening_pose=({float(opening_pose[0]):.4f},{float(opening_pose[1]):.4f},{float(opening_pose[2]):.4f}) "
            f"fork_state='{self.last_fork_state}' "
            f"pallet_z={float(pallet_pos[2]):.4f} "
            f"debug_artifact={debug_path}"
        )

    def wait_for_dock_geometry(
        self,
        name: str,
        timeout_sec: float,
        *,
        allow_a_station_handoff: bool = True,
    ) -> tuple[bool, str, dict[str, Any], dict[str, Any]]:
        deadline = self.runtime_deadline(timeout_sec)
        last_reason = "dock_geometry_timeout"
        last_metrics: dict[str, Any] = {}
        last_status_snapshot: dict[str, Any] = {}
        station = self.dock_station_name(name) or ""
        allow_a_station_handoff = allow_a_station_handoff and station.startswith("A")
        while rclpy.ok() and self.runtime_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            status_snapshot = dict(self.last_pose_line_status) if isinstance(self.last_pose_line_status, dict) else {}
            ok, reason, metrics = self.evaluate_dock_geometry(
                name,
                pose_line_status_override=status_snapshot,
                require_controller_complete=True,
            )
            last_reason = reason
            last_metrics = metrics
            last_status_snapshot = status_snapshot
            if ok:
                return True, reason, metrics, status_snapshot
            if station.startswith("B"):
                physical_ok, physical_reason, physical_metrics = self.evaluate_dock_geometry(
                    name,
                    pose_line_status_override=status_snapshot,
                    require_controller_complete=False,
                )
                if physical_ok:
                    self.get_logger().info(
                        f"DOCK_GEOMETRY_ACCEPT {name}: accepting B-station dock pose before "
                        f"controller completion; controller_state={status_snapshot.get('stop_reason')} "
                        f"completion_flag={status_snapshot.get('completion_flag')}"
                    )
                    return True, physical_reason, physical_metrics, status_snapshot
            if allow_a_station_handoff:
                physical_ok, physical_reason, physical_metrics = self.evaluate_dock_geometry(
                    name,
                    pose_line_status_override=status_snapshot,
                    require_controller_complete=False,
                )
                if physical_ok:
                    self.get_logger().info(
                        f"DOCK_GEOMETRY_HANDOFF {name}: handing off to A-station insertion loop before "
                        f"controller completion; controller_state={status_snapshot.get('stop_reason')} "
                        f"completion_flag={status_snapshot.get('completion_flag')}"
                    )
                    return True, "dock_geometry_physical_ok", physical_metrics, status_snapshot
            lift_ready, lift_reason, lift_metrics = self.evaluate_lift_ready_geometry(
                name,
                pose_line_status_override=status_snapshot,
                require_controller_complete=False,
            )
            if lift_ready:
                self.get_logger().info(
                    f"DOCK_GEOMETRY_AUTHORITATIVE {name}: accepting physical geometry despite "
                    f"controller_state={status_snapshot.get('stop_reason')} completion_flag={status_snapshot.get('completion_flag')}"
                )
                return True, "dock_geometry_physical_ok", lift_metrics, status_snapshot
            last_reason = lift_reason if reason == "dock_controller_not_complete" else reason
        return False, last_reason, last_metrics, last_status_snapshot

    def attempt_a1_lift_alignment_correction(
        self,
        name: str,
        *,
        timeout_sec: float = 4.0,
    ) -> tuple[bool, str, dict[str, Any], dict[str, Any]]:
        deadline = time.monotonic() + timeout_sec
        last_reason = "LIFT_ALIGNMENT_HOLD_FAILED"
        last_metrics: dict[str, Any] = {}
        last_status_snapshot: dict[str, Any] = {}
        last_linear_cmd = 0.0
        last_angular_cmd = 0.0
        target_pose = self.waypoint_pose_tuple(name)
        self.get_logger().info(
            f"A1_LIFT_ALIGNMENT_CORRECTION_START {name}: support depth already good; stopping insertion and entering hold/correction mode"
        )
        self.set_pose_line_mode(False, "OFF", segment=name)
        self.stop()
        self.wait_until_robot_stopped(timeout_sec=1.0, stable_duration_sec=0.2)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            route_pose = self.current_fine_control_pose()
            tracking = self.segment_tracking_metrics(name, pose=route_pose) if route_pose is not None else None
            if route_pose is None or tracking is None or target_pose is None:
                continue
            dock_heading_error = normalize_angle(float(target_pose[2]) - float(route_pose[2]))
            status_snapshot = {
                "mode": "DOCKING",
                "segment": name,
                "pose_x": float(route_pose[0]),
                "pose_y": float(route_pose[1]),
                "pose_yaw": float(route_pose[2]),
                "cross_track_error_m": tracking.get("cross_track_m"),
                "heading_error_rad": dock_heading_error,
                "progress_ratio": tracking.get("progress_ratio"),
                "completion_flag": False,
                "stop_reason": "lift_alignment_hold",
                "linear_cmd": float(last_linear_cmd),
                "angular_cmd": float(last_angular_cmd),
            }
            ok, reason, metrics = self.evaluate_lift_ready_geometry(
                name,
                pose_line_status_override=status_snapshot,
                require_controller_complete=False,
            )
            last_reason = reason
            last_metrics = metrics
            last_status_snapshot = status_snapshot
            if ok:
                self.stop()
                self.get_logger().info(
                    f"A1_LIFT_ALIGNMENT_CORRECTION_OK {name}: "
                    f"cross_track={metrics.get('dock_cross_track_error_m')} "
                    f"heading={metrics.get('dock_heading_error_rad')} "
                    f"lateral={metrics.get('fork_tip_lateral_error_m')} "
                    f"support_depth={metrics.get('support_depth_to_load_center_m')}"
                )
                return True, "dock_lift_alignment_recovered", metrics, status_snapshot
            support_depth = metrics.get("support_depth_to_load_center_m")
            lateral_error = metrics.get("fork_tip_lateral_error_m")
            cross_track = metrics.get("dock_cross_track_error_m")
            heading_error = metrics.get("dock_heading_error_rad")
            if not isinstance(support_depth, (int, float)) or float(support_depth) < self.dock_lift_support_margin_m:
                break
            if not isinstance(lateral_error, (int, float)) or abs(float(lateral_error)) > self.dock_lift_lateral_tolerance_m:
                break
            if not isinstance(cross_track, (int, float)) or abs(float(cross_track)) > self.dock_lift_line_cross_track_tolerance_m:
                break

            cmd = Twist()
            last_linear_cmd = 0.0
            last_angular_cmd = 0.0
            if isinstance(heading_error, (int, float)) and abs(float(heading_error)) > self.dock_lift_line_heading_tolerance_rad:
                last_angular_cmd = max(-0.08, min(0.08, 1.6 * float(heading_error)))
            cmd.linear.x = float(last_linear_cmd)
            cmd.angular.z = float(last_angular_cmd)
            try:
                self.cmd_pub.publish(cmd)
            except Exception:
                break
            rclpy.spin_once(self, timeout_sec=0.08)
        self.stop()
        last_metrics = dict(last_metrics)
        last_metrics["lift_alignment_hold_last_linear_cmd_mps"] = float(last_linear_cmd)
        last_metrics["lift_alignment_hold_last_angular_cmd_radps"] = float(last_angular_cmd)
        self.get_logger().error(
            f"A1_LIFT_ALIGNMENT_CORRECTION_FAILED {name}: "
            f"reason={last_reason} cross_track={last_metrics.get('dock_cross_track_error_m')} "
            f"heading={last_metrics.get('dock_heading_error_rad')} "
            f"lateral={last_metrics.get('fork_tip_lateral_error_m')} "
            f"support_depth={last_metrics.get('support_depth_to_load_center_m')}"
        )
        return False, "LIFT_ALIGNMENT_HOLD_FAILED", last_metrics, last_status_snapshot

    def attempt_a1_predock_line_realign(
        self,
        name: str,
        *,
        timeout_sec: float = 10.0,
    ) -> tuple[bool, str, dict[str, Any], dict[str, Any]]:
        self.get_logger().info(
            f"A1_PREDOCK_LINE_REALIGN_START {name}: using pose-line only to settle cross-track/heading before insertion"
        )
        self.stop()
        self.wait_until_robot_stopped(timeout_sec=1.0, stable_duration_sec=0.2)
        self.set_pose_line_mode(True, "DOCKING", segment=name)
        warmup_deadline = self.runtime_deadline(1.5)
        while rclpy.ok() and self.runtime_now() < warmup_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (time.monotonic() - self.last_pose_line_status_time) <= 0.5 and self.last_pose_line_status.get("active"):
                break
        deadline = self.runtime_deadline(timeout_sec)
        hold_samples = 0
        required_hold_samples = max(2, self.a1_dock_align_hold_samples)
        last_sample: dict[str, Any] = {}
        last_status_snapshot: dict[str, Any] = {}
        last_log_time = 0.0
        while rclpy.ok() and self.runtime_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            route_pose = self.pose_line_snapshot_pose_for_segment(name, max_age_sec=0.5) or self.current_fine_control_pose()
            sample = self.a1_dock_trace_sample(name, route_pose_override=route_pose)
            status_snapshot = dict(self.last_pose_line_status) if isinstance(self.last_pose_line_status, dict) else {}
            last_sample = sample
            last_status_snapshot = status_snapshot
            if self.a1_alignment_metrics_ok(sample):
                hold_samples += 1
            else:
                hold_samples = 0
            if (time.monotonic() - last_log_time) >= 0.2:
                self.log_a1_dock_trace_sample(name, dict(sample), "PREDOCK_LINE_REALIGN")
                last_log_time = time.monotonic()
            if hold_samples >= required_hold_samples:
                self.stop()
                stopped_ok, stop_metrics = self.wait_until_robot_stopped(timeout_sec=1.0, stable_duration_sec=0.2)
                verify_pose = self.pose_line_snapshot_pose_for_segment(name, max_age_sec=0.5) or self.current_fine_control_pose()
                verify_sample = self.a1_dock_trace_sample(name, route_pose_override=verify_pose)
                verify_sample["final_stop_linear_cmd_mps"] = stop_metrics.get("linear_cmd")
                verify_sample["final_stop_angular_cmd_radps"] = stop_metrics.get("angular_cmd")
                verify_sample["robot_stopped_after_predock_realign_ok"] = stopped_ok
                verify_status = dict(self.last_pose_line_status) if isinstance(self.last_pose_line_status, dict) else {}
                self.set_pose_line_mode(False, "OFF", segment=name)
                if stopped_ok and self.a1_alignment_metrics_ok(verify_sample):
                    self.get_logger().info(
                        f"A1_PREDOCK_LINE_REALIGN_OK {name}: "
                        f"cross_track={verify_sample.get('cross_track_error_m')} "
                        f"heading={verify_sample.get('heading_error_rad')} "
                        f"lateral={verify_sample.get('pallet_lateral_error_m')} "
                        f"pallet_heading={verify_sample.get('pallet_heading_error_rad')}"
                    )
                    return True, "a1_predock_line_realign_ok", verify_sample, verify_status
                return False, "A1_PREDOCK_LINE_REALIGN_FAILED", verify_sample, verify_status
            if not self.a1_predock_line_recoverable(sample):
                break
        self.stop()
        self.set_pose_line_mode(False, "OFF", segment=name)
        return False, "A1_PREDOCK_LINE_REALIGN_FAILED", last_sample, last_status_snapshot

    def a1_pre_lift_heading_settle_eligible(self, metrics: dict[str, Any]) -> bool:
        support_depth = metrics.get("support_depth_to_load_center_m")
        lateral = metrics.get("fork_tip_lateral_error_m")
        dock_cross_track = metrics.get("dock_cross_track_error_m")
        dock_heading = metrics.get("dock_heading_error_rad")
        left_insertion_depth = metrics.get("left_fork_insertion_depth_m")
        right_insertion_depth = metrics.get("right_fork_insertion_depth_m")
        min_insertion_depth = self.pallet_opening_face_offset_m + self.dock_lift_support_margin_m
        if not isinstance(support_depth, (int, float)) or float(support_depth) < self.dock_lift_support_margin_m:
            return False
        if not isinstance(lateral, (int, float)) or abs(float(lateral)) > self.dock_lift_lateral_tolerance_m:
            return False
        if (
            not isinstance(left_insertion_depth, (int, float))
            or float(left_insertion_depth) < min_insertion_depth
            or not isinstance(right_insertion_depth, (int, float))
            or float(right_insertion_depth) < min_insertion_depth
        ):
            return False
        if not isinstance(dock_cross_track, (int, float)):
            return False
        if abs(float(dock_cross_track)) > self.dock_lift_line_cross_track_tolerance_m:
            return False
        if not isinstance(dock_heading, (int, float)):
            return False
        # Only use in-place settle when the remaining issue is a small heading miss.
        return abs(float(dock_heading)) <= 0.06

    def attempt_a1_dock_realign_retry(
        self,
        name: str,
    ) -> tuple[bool, str, dict[str, Any], dict[str, Any]]:
        self.get_logger().warn(
            f"A1_DOCK_REALIGN_RETRY_START {name}: inserted geometry not acceptable for lift; backing out to retry insertion"
        )
        if self.line_source == "pose_line":
            self.set_pose_line_mode(False, "OFF", segment=name)
        elif self.line_source == "line_sensor":
            self.set_line_sensor_mode(False, "OFF")
        else:
            self.set_line_mode(False, "OFF", profile="dock_line", context=name)
        self.stop()
        self.wait_until_robot_stopped(timeout_sec=1.0, stable_duration_sec=0.2)
        if not self.drive_distance(
            self.a1_dock_realign_backout_distance_m,
            -self.a1_dock_realign_backout_speed_mps,
        ):
            return False, "A1_DOCK_REALIGN_RETRY_FAILED", {
                "realign_retry_stage": "backout",
            }, {}
        dock_status: dict[str, Any] = {}
        if name.startswith("A"):
            self.set_pose_line_mode(True, "DOCKING", segment=name)
            timeout_sec = max(abs(self.docking_distance / max(self.docking_speed, 0.01)) * 5.0, 10.0)
            (
                dock_ok,
                dock_reason,
                dock_metrics,
                dock_status,
            ) = self.wait_for_dock_geometry(name, timeout_sec, allow_a_station_handoff=False)
            if not dock_status and isinstance(self.last_pose_line_status, dict):
                dock_status = dict(self.last_pose_line_status)
            self.set_pose_line_mode(False, "OFF", segment=name)
            if not dock_ok:
                failed_metrics = dict(dock_metrics)
                failed_metrics["realign_retry_stage"] = "controller_realign_failed"
                failed_metrics["realign_retry_reason"] = dock_reason
                return False, "A1_DOCK_REALIGN_RETRY_FAILED", failed_metrics, dock_status
        insertion_ok, insertion_reason, insertion_metrics = self.a1_creep_insertion(name)
        if insertion_ok:
            self.stop()
            stopped_ok, stop_metrics = self.wait_until_robot_stopped()
            insertion_metrics = dict(insertion_metrics)
            insertion_metrics["final_stop_linear_cmd_mps"] = stop_metrics.get("linear_cmd")
            insertion_metrics["final_stop_angular_cmd_radps"] = stop_metrics.get("angular_cmd")
            insertion_metrics["pose_line_disabled_after_insertion_ok"] = True
            insertion_metrics["robot_stopped_after_insertion_ok"] = stopped_ok
            if not stopped_ok:
                insertion_metrics["realign_retry_stage"] = "stop_after_retry_insertion"
                return False, "A1_DOCK_REALIGN_RETRY_FAILED", insertion_metrics, dock_status
            robot_pose = insertion_metrics.get("robot_pose")
            if isinstance(robot_pose, list) and len(robot_pose) >= 3:
                dock_status = dict(dock_status)
                dock_status.update(
                    {
                        "mode": "DOCKING",
                        "segment": name,
                        "pose_x": float(robot_pose[0]),
                        "pose_y": float(robot_pose[1]),
                        "pose_yaw": float(robot_pose[2]),
                        "cross_track_error_m": insertion_metrics.get("cross_track_error_m"),
                        "heading_error_rad": insertion_metrics.get("heading_error_rad"),
                        "progress_ratio": insertion_metrics.get("progress_ratio"),
                        "linear_cmd": 0.0,
                        "angular_cmd": 0.0,
                        "completion_flag": True,
                        "stop_reason": "dock_geometry_complete",
                    }
                )
            insertion_metrics["realign_retry_stage"] = "retry_insertion_ok"
            return True, "a1_insertion_retry_ok", insertion_metrics, dock_status
        insertion_metrics = dict(insertion_metrics)
        insertion_metrics["realign_retry_stage"] = "retry_insertion_failed"
        insertion_metrics["realign_retry_reason"] = insertion_reason
        return False, "A1_DOCK_REALIGN_RETRY_FAILED", insertion_metrics, dock_status

    def rotate_relative(self, delta_yaw: float) -> bool:
        if self.current_pose is None:
            self.get_logger().error("Cannot rotate: no odom pose")
            return False
        start_yaw = self.current_pose[2]
        target_yaw = normalize_angle(start_yaw + delta_yaw)
        command = Twist()
        direction = 1.0 if normalize_angle(target_yaw - start_yaw) >= 0.0 else -1.0
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.current_pose is None:
                continue
            error = normalize_angle(target_yaw - self.current_pose[2])
            if abs(error) <= 0.04:
                self.stop()
                return True
            command.angular.z = max(0.08, min(self.turn_speed, abs(error) * 0.8)) * (1.0 if error >= 0.0 else -1.0)
            if abs(error) > math.pi * 0.98:
                command.angular.z = self.turn_speed * direction
            self.cmd_pub.publish(command)
        self.stop()
        self.get_logger().error("Rotate timeout during docking action")
        return False

    def navigate(self, name: str, *, line_correction_enabled: bool | None = None, segment_name: str | None = None) -> bool:
        resolved_name = self.resolve_waypoint_name(name)
        waypoint = self.waypoints[resolved_name]
        nav_line_correction = self.enable_line_correction if line_correction_enabled is None else line_correction_enabled
        label = segment_name or name
        self.start_segment(
            segment_type="nav",
            name=label,
            target_name=resolved_name,
            target_pose=self.waypoint_pose_tuple(resolved_name),
            line_correction_enabled=nav_line_correction,
        )
        self.get_logger().info(
            f"NAV {label} -> {resolved_name}: x={waypoint['x']:.4f}, y={waypoint['y']:.4f}, yaw={waypoint['yaw']:.4f}"
        )
        if nav_line_correction:
            self.set_line_mode(True, "ROUTE_CORRECTION", profile="route_line", context=resolved_name)
            deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)

        goal = NavigateToPose.Goal()
        goal.pose = self.pose_from_waypoint(resolved_name)
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.finish_segment(success=False, result_status=None, notes="goal_rejected")
            self.get_logger().error(f"NAV FAILED {label}: goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        status = result.status if result is not None else None
        if nav_line_correction:
            self.set_line_mode(False, "OFF", profile="route_line", context=resolved_name)
        if result is None:
            self.finish_segment(success=False, result_status=None, notes="no_result")
            self.get_logger().error(f"NAV FAILED {label}: no result")
            return False
        if status != GoalStatus.STATUS_SUCCEEDED:
            snapshot = self.costmap_snapshot(waypoint["x"], waypoint["y"])
            self.finish_segment(
                success=False,
                result_status=status,
                notes="nav_failed",
                extra={"costmap_snapshot": snapshot},
            )
            self.get_logger().error(f"NAV FAILED {label}: status={status}")
            return False

        self.reached.append(label)
        self.finish_segment(success=True, result_status=status, notes="nav_succeeded")
        self.get_logger().info(f"REACHED {label}")
        return True

    def line_primary_navigate(self, name: str, *, segment_name: str | None = None) -> bool:
        resolved_name = self.resolve_waypoint_name(name)
        waypoint = self.waypoints[resolved_name]
        label = segment_name or name
        target_pose = self.waypoint_pose_tuple(resolved_name)
        route_pose = self.current_route_pose()
        initial_pose = route_pose
        camera_staging_target = (
            self.line_source == "camera_line" and resolved_name.upper().endswith("_STAGING_CALIBRATED")
        )
        if route_pose is not None:
            initial_remaining = pose_distance(route_pose, target_pose)
            if (
                initial_remaining is not None
                and initial_remaining <= self.line_primary_distance_tolerance
                and not camera_staging_target
            ):
                self.start_segment(
                    segment_type="line_primary_nav",
                    name=label,
                    target_name=resolved_name,
                    target_pose=target_pose,
                    line_correction_enabled=True,
                )
                self.reached.append(label)
                self.finish_segment(success=True, result_status=None, notes="line_primary_already_within_tolerance")
                self.get_logger().info(f"REACHED {label} with line-primary: already within tolerance")
                return True
        self.start_segment(
            segment_type="line_primary_nav",
            name=label,
            target_name=resolved_name,
            target_pose=target_pose,
            line_correction_enabled=True,
        )
        if self.current_segment is not None:
            self.current_segment["controller_source"] = "camera_line"
            self.latest_qr_latch = None
            self.current_segment["expected_qr_id"] = self.semantic_id_for_target(resolved_name)
        self.get_logger().info(
            f"LINE_PRIMARY {label} -> {resolved_name}: x={waypoint['x']:.4f}, y={waypoint['y']:.4f}, yaw={waypoint['yaw']:.4f}"
        )
        self.set_line_mode(True, "LINE_PRIMARY_ROUTE", profile="route_line", context=resolved_name)

        warmup_deadline = time.monotonic() + 6.0
        saw_line = False
        saw_status = False
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_line_status:
                saw_status = True
            if self.line_detected():
                saw_line = True
                break
        if not saw_line:
            self.set_line_mode(False, "OFF", profile="route_line", context=resolved_name)
            note = "camera_line_not_detected" if saw_status else "camera_line_no_status"
            self.finish_segment(success=False, notes=note)
            self.get_logger().error(
                f"LINE_PRIMARY FAILED {label}: "
                + (
                    "camera line not detected at segment start"
                    if saw_status
                    else "no camera line follower status at segment start"
                )
            )
            return False

        deadline = time.monotonic() + self.line_primary_timeout_sec
        spin_guard_window_start = time.monotonic()
        spin_guard_pose = self.current_route_pose()
        spin_guard_yaw = route_pose[2] if route_pose is not None else None
        camera_staging_settle_used = False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.clock_jump_back_detected:
                self.set_line_mode(False, "OFF", profile="route_line", context=resolved_name)
                self.finish_segment(success=False, notes="sim_time_jump_detected")
                self.get_logger().error(f"LINE_PRIMARY FAILED {label}: sim time jump detected")
                return False
            route_pose = self.current_route_pose()
            if route_pose is None:
                continue
            reached, reason, progress_metrics = self.line_primary_completion_reason(
                initial_pose, target_pose, route_pose
            )
            if (
                camera_staging_target
                and not reached
                and not camera_staging_settle_used
            ):
                settle_metrics = self.camera_line_alignment_metrics(resolved_name)
                if self.camera_line_staging_settle_candidate(settle_metrics):
                    self.get_logger().info(
                        f"CAMERA_LINE_STAGING_SETTLE {label}: "
                        f"distance={settle_metrics.get('distance_to_target_m')} "
                        f"filtered_error_px={settle_metrics.get('filtered_error_px')} "
                        f"heading_error={settle_metrics.get('heading_error_rad')} "
                        f"centered_streak={settle_metrics.get('centered_streak')}"
                    )
                    settled, settle_metrics = self.wait_for_camera_line_staging_settle(resolved_name)
                    camera_staging_settle_used = True
                    if settled:
                        self.set_line_mode(False, "OFF", profile="route_line", context=resolved_name)
                        self.reached.append(label)
                        record = self.finish_segment(
                            success=True,
                            result_status=None,
                            notes="line_primary_reached_camera_line_staging_settle",
                            extra={
                                "completion_reason": "camera_line_staging_settle",
                                "camera_line_staging_settle_metrics": settle_metrics,
                            },
                        )
                        metrics = record.get("line_metrics", {})
                        self.get_logger().info(
                            f"REACHED {label} with line-primary: avg_abs_error_px={metrics.get('avg_abs_error_px', 0.0):.1f} "
                            f"max_abs_error_px={metrics.get('max_abs_error_px', 0.0):.1f} "
                            f"centered_ratio={metrics.get('centered_ratio', 0.0):.2f} "
                            f"completion_reason=camera_line_staging_settle"
                        )
                        return True
            remaining = progress_metrics["remaining_m"]
            if reached:
                self.set_line_mode(False, "OFF", profile="route_line", context=resolved_name)
                self.reached.append(label)
                record = self.finish_segment(
                    success=True,
                    result_status=None,
                    notes=f"line_primary_reached_{reason}",
                    extra={"completion_reason": reason},
                )
                metrics = record.get("line_metrics", {})
                self.get_logger().info(
                    f"REACHED {label} with line-primary: avg_abs_error_px={metrics.get('avg_abs_error_px', 0.0):.1f} "
                    f"max_abs_error_px={metrics.get('max_abs_error_px', 0.0):.1f} "
                    f"centered_ratio={metrics.get('centered_ratio', 0.0):.2f} "
                    f"completion_reason={reason}"
                )
                return True
            phase = str(self.last_line_status.get("line_primary_phase", ""))
            if phase in {"CAPTURE_LINE", "SEARCH_LINE", "ROTATE_CAPTURE", "ARC_CAPTURE"}:
                if spin_guard_pose is None:
                    spin_guard_pose = route_pose
                    spin_guard_yaw = route_pose[2]
                    spin_guard_window_start = time.monotonic()
                elif time.monotonic() - spin_guard_window_start >= 4.0:
                    translation = math.hypot(route_pose[0] - spin_guard_pose[0], route_pose[1] - spin_guard_pose[1])
                    yaw_delta = abs(normalize_angle(route_pose[2] - (spin_guard_yaw or route_pose[2])))
                    if translation < 0.10 and yaw_delta > 1.2:
                        self.set_line_mode(False, "OFF", profile="route_line", context=resolved_name)
                        self.finish_segment(
                            success=False,
                            notes="line_capture_spin",
                            extra={
                                "spin_guard_translation_m": translation,
                                "spin_guard_yaw_delta_rad": yaw_delta,
                            },
                        )
                        self.get_logger().error(
                            f"LINE_PRIMARY FAILED {label}: LINE_CAPTURE_SPIN translation={translation:.3f} yaw_delta={yaw_delta:.3f}"
                        )
                        return False
                    spin_guard_pose = route_pose
                    spin_guard_yaw = route_pose[2]
                    spin_guard_window_start = time.monotonic()
            else:
                spin_guard_pose = route_pose
                spin_guard_yaw = route_pose[2]
                spin_guard_window_start = time.monotonic()
            if phase in {"LINE_LOST_STOP", "FAIL_SAFE"}:
                self.set_line_mode(False, "OFF", profile="route_line", context=resolved_name)
                self.finish_segment(success=False, notes=f"line_primary_{phase.lower() or 'line_lost'}")
                self.get_logger().error(
                    f"LINE_PRIMARY FAILED {label}: phase={phase or 'unknown'} before reaching target, remaining={remaining}"
                )
                return False

        self.set_line_mode(False, "OFF", profile="route_line", context=resolved_name)
        self.finish_segment(success=False, notes="line_primary_timeout")
        self.get_logger().error(f"LINE_PRIMARY FAILED {label}: timeout")
        return False

    def line_sensor_primary_navigate(self, name: str, *, segment_name: str | None = None) -> bool:
        resolved_name = self.resolve_waypoint_name(name)
        waypoint = self.waypoints[resolved_name]
        label = segment_name or name
        target_pose = self.waypoint_pose_tuple(resolved_name)
        initial_pose = self.current_route_pose()
        strict_staging_target = resolved_name.upper().endswith("_STAGING_CALIBRATED")
        self.start_segment(
            segment_type="line_sensor_primary_nav",
            name=label,
            target_name=resolved_name,
            target_pose=target_pose,
            line_correction_enabled=True,
        )
        if self.current_segment is not None:
            self.current_segment["controller_source"] = "line_sensor"
            self.latest_qr_latch = None
            self.current_segment["expected_qr_id"] = self.semantic_id_for_target(resolved_name)
        self.get_logger().info(
            f"LINE_SENSOR_PRIMARY {label} -> {resolved_name}: "
            f"x={waypoint['x']:.4f}, y={waypoint['y']:.4f}, yaw={waypoint['yaw']:.4f}"
        )
        self.set_line_sensor_mode(True, "LINE_FOLLOW")

        warmup_deadline = time.monotonic() + 4.0
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_line_sensor_seen:
                break
        deadline = time.monotonic() + self.line_primary_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            route_pose = self.current_route_pose()
            if route_pose is None:
                continue
            if strict_staging_target:
                ready, strict_metrics = self.line_sensor_ready_for_staging(resolved_name)
                if ready:
                    self.set_line_sensor_mode(False, "OFF")
                    self.reached.append(label)
                    self.finish_segment(
                        success=True,
                        result_status=None,
                        notes="line_sensor_primary_reached_staging_strict",
                        extra={
                            "completion_reason": "line_sensor_staging_strict",
                            "line_sensor_staging_metrics": strict_metrics,
                        },
                    )
                    self.get_logger().info(
                        f"REACHED {label} with line-sensor-primary: "
                        f"completion_reason=line_sensor_staging_strict "
                        f"distance={strict_metrics.get('distance_to_target_m')} "
                        f"position={strict_metrics.get('line_sensor_position')}"
                    )
                    return True
            reached, reason, progress_metrics = self.line_primary_completion_reason(
                initial_pose, target_pose, route_pose
            )
            if strict_staging_target and reason in {"distance_tolerance", "target_corridor", "passed_waypoint"}:
                reached = False
            if reached:
                self.set_line_sensor_mode(False, "OFF")
                self.reached.append(label)
                self.finish_segment(
                    success=True,
                    result_status=None,
                    notes=f"line_sensor_primary_reached_{reason}",
                    extra={"completion_reason": reason},
                )
                self.get_logger().info(f"REACHED {label} with line-sensor-primary: completion_reason={reason}")
                return True
            if not self.last_line_sensor_seen and progress_metrics["along_track_m"] is not None:
                if progress_metrics["along_track_m"] < self.line_primary_min_progress_m:
                    continue
        self.set_line_sensor_mode(False, "OFF")
        self.finish_segment(success=False, notes="line_sensor_primary_timeout")
        self.get_logger().error(f"LINE_SENSOR_PRIMARY FAILED {label}: timeout")
        return False

    def competition_navigate(self, name: str) -> bool:
        self.log_current_location_context(self.semantic_id_for_target(name))
        self.set_line_mode(False, "OFF", profile="route_line", context=name)
        self.set_line_sensor_mode(False, "OFF")
        self.set_pose_line_mode(False, "OFF", segment=name)
        handoff_deadline = self.runtime_deadline(0.7)
        while rclpy.ok() and self.runtime_now() < handoff_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.navigate(name, line_correction_enabled=False, segment_name=name)

    def pose_line_navigate(self, name: str, *, mode: str = "LINE_APPROACH", segment_name: str | None = None) -> bool:
        resolved_name = self.resolve_waypoint_name(name)
        waypoint = self.waypoints[resolved_name]
        label = segment_name or name
        controller_segment = self.resolve_waypoint_name(segment_name) if segment_name else resolved_name
        target_pose = self.waypoint_pose_tuple(resolved_name)
        route_pose = self.current_fine_control_pose()
        initial_pose = route_pose
        line_like_mode = mode in {"LINE_APPROACH", "LINE_REVERSE"}
        self.start_segment(
            segment_type="pose_line_nav" if line_like_mode else "pose_line_dock",
            name=label,
            target_name=resolved_name,
            target_pose=target_pose,
            line_correction_enabled=True,
        )
        if self.current_segment is not None:
            self.current_segment["controller_source"] = "pose_line"
            if line_like_mode:
                self.latest_qr_latch = None
                self.current_segment["expected_qr_id"] = self.semantic_id_for_target(resolved_name)
        self.get_logger().info(
            f"POSE_LINE {mode} {label} -> {resolved_name}: x={waypoint['x']:.4f}, y={waypoint['y']:.4f}, yaw={waypoint['yaw']:.4f}"
        )
        self.set_pose_line_mode(True, mode, segment=controller_segment)

        def pose_line_status_matches_requested_segment(status: dict[str, Any] | None) -> bool:
            if not isinstance(status, dict):
                return False
            if not status.get("active"):
                return False
            status_segment = str(status.get("segment") or "").upper()
            status_mode = str(status.get("mode") or "").upper()
            return status_segment == controller_segment and status_mode == mode.upper()

        warmup_deadline = self.runtime_deadline(3.0)
        while rclpy.ok() and self.runtime_now() < warmup_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                (time.monotonic() - self.last_pose_line_status_time) <= 1.0
                and pose_line_status_matches_requested_segment(self.last_pose_line_status)
            ):
                break
        if (
            (time.monotonic() - self.last_pose_line_status_time) > 1.0
            or not pose_line_status_matches_requested_segment(self.last_pose_line_status)
        ):
            self.set_pose_line_mode(False, "OFF", segment=controller_segment)
            self.finalize_active_segment_failure(
                notes="pose_line_no_status",
                failure_reason=f"line_follow_failed:{label}",
                extra={
                    "last_active_action": label,
                    "target_pose_live": list(target_pose) if target_pose is not None else None,
                    "stopped_in_function": "pose_line_navigate",
                    "stopped_in_state": "no_status",
                },
            )
            self.get_logger().error(f"POSE_LINE FAILED {label}: no pose_line status")
            return False

        if initial_pose is None:
            initial_pose = (
                self.pose_line_snapshot_pose_for_segment(controller_segment, max_age_sec=1.0)
                or self.current_fine_control_pose()
            )
        if self.current_segment is not None and self.current_segment.get("start_pose") is None:
            self.current_segment["start_pose"] = initial_pose

        deadline = self.runtime_deadline(self.line_primary_timeout_sec)
        last_status_log = 0.0
        last_stop_reason_log = ""
        while rclpy.ok() and self.runtime_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            route_pose = self.current_fine_control_pose()
            if route_pose is None:
                continue
            status = self.last_pose_line_status if isinstance(self.last_pose_line_status, dict) else {}
            if not pose_line_status_matches_requested_segment(status):
                continue
            reached, reason, progress_metrics = self.line_primary_completion_reason(
                initial_pose,
                target_pose,
                route_pose,
            )
            if line_like_mode:
                reached, reason, progress_metrics = self.line_approach_completion_reason(progress_metrics)
            now_mono = time.monotonic()
            if now_mono - last_status_log >= 1.0:
                controller_progress_ratio = status.get("progress_ratio")
                executor_progress_ratio = None
                segment_length = progress_metrics.get("segment_length_m")
                along_track = progress_metrics.get("along_track_m")
                if along_track is not None and segment_length not in (None, 0.0):
                    executor_progress_ratio = along_track / segment_length
                self.get_logger().info(
                    "POSE_LINE_STATUS "
                    f"segment={resolved_name} mode={mode} "
                    f"pose=({route_pose[0]:.3f},{route_pose[1]:.3f},{route_pose[2]:.3f}) "
                    f"segment_start={status.get('segment')} "
                    f"cross_track_error={status.get('cross_track_error_m')} "
                    f"heading_error={status.get('heading_error_rad')} "
                    f"controller_progress_m={status.get('progress_m')} "
                    f"controller_progress_ratio={controller_progress_ratio} "
                    f"executor_progress_m={along_track} "
                    f"executor_progress_ratio={executor_progress_ratio} "
                    f"controller_remaining={status.get('remaining_m')} "
                    f"executor_remaining={progress_metrics.get('remaining_m')} "
                    f"linear_cmd={status.get('linear_cmd')} "
                    f"angular_cmd={status.get('angular_cmd')} "
                    f"completion_flag={status.get('completion_flag')} "
                    f"enabled={status.get('enabled')} "
                    f"stop_reason={status.get('stop_reason')}"
                )
                last_status_log = now_mono
            stop_reason = str(status.get("stop_reason") or "")
            if stop_reason and stop_reason != "tracking_segment" and stop_reason != last_stop_reason_log:
                self.get_logger().info(
                    "POSE_LINE_STOP_STATE "
                    f"segment={resolved_name} mode={mode} "
                    f"remaining_m={status.get('remaining_m')} "
                    f"progress_ratio={status.get('progress_ratio')} "
                    f"completion_flag={status.get('completion_flag')} "
                    f"enabled={status.get('enabled')} "
                    f"linear_cmd={status.get('linear_cmd')} "
                    f"angular_cmd={status.get('angular_cmd')} "
                    f"stop_reason={stop_reason}"
                )
                last_stop_reason_log = stop_reason
            if reached:
                self.set_pose_line_mode(False, "OFF", segment=controller_segment)
                self.reached.append(label)
                self.finish_segment(
                    success=True,
                    result_status=None,
                    notes=f"pose_line_reached_{reason}",
                    extra={"completion_reason": reason, "pose_line_status": self.last_pose_line_status},
                )
                log_suffix = ""
                if line_like_mode:
                    _, _, strict_metrics = self.line_approach_completion_reason(progress_metrics)
                    log_suffix = (
                        f" distance_to_target={strict_metrics['distance_to_target_m']}"
                        f" cross_track_error={strict_metrics['cross_track_error_m']}"
                        f" heading_error={strict_metrics['heading_error_rad']}"
                        f" controller_progress={strict_metrics['controller_progress_ratio']}"
                        f" executor_progress={strict_metrics['executor_progress_ratio']}"
                    )
                self.get_logger().info(
                    f"REACHED {label} with pose_line: completion_reason={reason}{log_suffix}"
                )
                return True
            if self.last_pose_line_status.get("active") is False and self.last_pose_line_status.get("segment_valid") is False:
                self.set_pose_line_mode(False, "OFF", segment=controller_segment)
                self.finalize_active_segment_failure(
                    notes="pose_line_invalid_segment",
                    failure_reason=f"line_follow_failed:{label}",
                    extra={
                        "pose_line_status": self.last_pose_line_status,
                        "progress_metrics": progress_metrics,
                        "last_active_action": label,
                        "last_cmd_vel": {
                            "linear": status.get("linear_cmd"),
                            "angular": status.get("angular_cmd"),
                        },
                        "stopped_in_function": "pose_line_navigate",
                        "stopped_in_state": "segment_invalid",
                    },
                )
                self.get_logger().error(f"POSE_LINE FAILED {label}: invalid segment configuration")
                return False
        self.set_pose_line_mode(False, "OFF", segment=controller_segment)
        final_pose = self.current_fine_control_pose()
        self.finalize_active_segment_failure(
            notes="pose_line_timeout",
            failure_reason=f"line_follow_failed:{label}",
            extra={
                "pose_line_status": self.last_pose_line_status,
                "last_active_action": label,
                "final_pose_live": list(final_pose) if final_pose is not None else None,
                "target_pose_live": list(target_pose) if target_pose is not None else None,
                "final_distance_to_target_live_m": pose_distance(final_pose, target_pose),
                "last_cmd_vel": {
                    "linear": self.last_pose_line_status.get("linear_cmd") if isinstance(self.last_pose_line_status, dict) else None,
                    "angular": self.last_pose_line_status.get("angular_cmd") if isinstance(self.last_pose_line_status, dict) else None,
                },
                "stopped_in_function": "pose_line_navigate",
                "stopped_in_state": "timeout",
            },
        )
        self.get_logger().error(f"POSE_LINE FAILED {label}: timeout")
        return False

    def competition_line_approach(self, name: str) -> bool:
        self.log_current_location_context(self.semantic_id_for_target(name))
        resolved_name = self.resolve_waypoint_name(name)
        if self.line_source == "pose_line":
            self.set_pose_line_mode(True, "LINE_APPROACH", segment=resolved_name)
        elif self.line_source == "line_sensor":
            self.set_line_sensor_mode(True, "LINE_FOLLOW")
        elif self.line_source == "camera_line":
            self.set_line_mode(True, "LINE_PRIMARY_ROUTE", profile="route_line", context=name)

        deadline = self.runtime_deadline(60.0)
        controller_source = self.competition_controller_source()
        while rclpy.ok() and controller_source == "none" and self.runtime_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            controller_source = self.competition_controller_source()
        if controller_source == "line_sensor":
            return self.line_sensor_primary_navigate(name)
        if controller_source == "pose_line":
            return self.pose_line_navigate(name, mode="LINE_APPROACH")
        if controller_source == "camera_line":
            return self.line_primary_navigate(name)
        self.start_segment(
            segment_type="competition_nav",
            name=name,
            target_name=self.resolve_waypoint_name(name),
            target_pose=self.waypoint_pose_tuple(name),
            line_correction_enabled=False,
        )
        if self.current_segment is not None:
            self.current_segment["controller_source"] = "none"
        self.finish_segment(success=False, notes="competition_no_active_line_controller")
        self.get_logger().error(f"LINE_APPROACH FAILED {name}: no active line controller source")
        return False

    def competition_line_follow(self, target_name: str, segment_name: str) -> bool:
        self.log_current_location_context(self.semantic_id_for_target(target_name))
        resolved_target = self.resolve_waypoint_name(target_name)
        if self.line_source == "pose_line":
            return self.pose_line_navigate(target_name, mode="LINE_APPROACH", segment_name=segment_name)
        if self.line_source == "line_sensor":
            if segment_name == "D1_TO_A1_STAGING_CALIBRATED":
                self.get_logger().info(
                    "LINE_SENSOR_BRANCH_FALLBACK D1_TO_A1_STAGING_CALIBRATED: "
                    "using pose_line to stay on the A1 branch through the D1 junction"
                )
                return self.pose_line_navigate(target_name, mode="LINE_APPROACH", segment_name=segment_name)
            # The D1->Q2 baseline is now usable with line_sensor. Once the pallet is
            # actually on the forks, though, line_sensor-only road following can lock
            # onto the wrong branch at intersections and drive farther away from the
            # target even while still seeing a valid green line. Carrying segments are
            # mapped cleanly enough to let pose_line keep the robot on the intended
            # route without disturbing the proven pre-A1 baseline.
            if self.infer_carrying_load() is True:
                if segment_name == "GATE_OUT_TO_GATE_IN":
                    self.get_logger().info(
                        "LINE_SENSOR_CARRY_GATE_REVERSE GATE_OUT_TO_GATE_IN: "
                        "using pose_line reverse approach to cross the gate while loaded "
                        "without a 180-degree turn inside the posts"
                    )
                    return self.pose_line_navigate(target_name, mode="LINE_REVERSE", segment_name=segment_name)
                self.get_logger().info(
                    f"LINE_SENSOR_CARRY_FALLBACK {segment_name}: load is on forks; "
                    "using pose_line for carried-load route follow"
                )
                return self.pose_line_navigate(target_name, mode="LINE_APPROACH", segment_name=segment_name)
            return self.line_sensor_primary_navigate(target_name)
        if self.line_source == "camera_line":
            self.set_line_mode(True, "LINE_PRIMARY_ROUTE", profile="route_line", context=segment_name)
            return self.line_primary_navigate(target_name)
        self.start_segment(
            segment_type="competition_line_follow",
            name=segment_name,
            target_name=resolved_target,
            target_pose=self.waypoint_pose_tuple(target_name),
            line_correction_enabled=False,
        )
        if self.current_segment is not None:
            self.current_segment["controller_source"] = "none"
        self.finish_segment(success=False, notes="competition_no_active_line_controller")
        self.get_logger().error(f"LINE_FOLLOW FAILED {segment_name}: no active line controller source")
        return False

    def qr_check(self, action: dict[str, str]) -> None:
        qr_id = action["qr_id"]
        target = action["target"]
        now = self.get_clock().now().nanoseconds / 1e9
        fresh = self.last_qr and now - self.last_qr_time <= 2.0
        if fresh and self.last_qr == qr_id:
            self.get_logger().info(f"QR_CHECK {action['name']} OK: saw {qr_id} near {target}")
        elif fresh:
            self.get_logger().warn(
                f"QR_CHECK {action['name']} mismatch: expected {qr_id}, last_qr={self.last_qr}"
            )
        else:
            self.get_logger().info(
                f"QR_CHECK {action['name']} reference only: expected {qr_id}; no fresh QR sample"
            )

    def competition_qr_check(self, action: dict[str, str]) -> bool:
        self.start_segment(
            segment_type="qr_check",
            name=action["name"],
            target_name=action["target"],
            target_pose=self.waypoint_pose_tuple(action["target"]),
            line_correction_enabled=False,
        )
        if self.current_segment is not None:
            self.current_segment["controller_source"] = "QR_CHECK"
        arrival_time_sec = self.get_clock().now().nanoseconds / 1e9
        deadline = self.runtime_deadline(self.competition_qr_reacquire_timeout_sec)
        ok, reason = self.fresh_qr_ok(action["qr_id"], min_sample_time_sec=arrival_time_sec)
        qr_source = "fresh_post_arrival"
        latch_info: dict[str, Any] | None = None
        if not ok:
            latch_ok, latch_reason, latch_info = self.latched_qr_ok(action["qr_id"], target_name=action["target"])
            if latch_ok:
                ok = True
                reason = latch_reason
                qr_source = "latched_during_approach"
        if not ok:
            pose_ok, pose_reason, pose_info = self.qr_pose_reference_ok(
                action["qr_id"],
                target_name=action["target"],
            )
            if pose_ok:
                ok = True
                reason = pose_reason
                qr_source = "pose_reference_after_pickup"
                latch_info = pose_info
        while rclpy.ok() and not ok and self.runtime_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            ok, reason = self.fresh_qr_ok(action["qr_id"], min_sample_time_sec=arrival_time_sec)
            qr_source = "fresh_post_arrival"
            if not ok:
                latch_ok, latch_reason, latch_info = self.latched_qr_ok(action["qr_id"], target_name=action["target"])
                if latch_ok:
                    ok = True
                    reason = latch_reason
                    qr_source = "latched_during_approach"
            if not ok:
                pose_ok, pose_reason, pose_info = self.qr_pose_reference_ok(
                    action["qr_id"],
                    target_name=action["target"],
                )
                if pose_ok:
                    ok = True
                    reason = pose_reason
                    qr_source = "pose_reference_after_pickup"
                    latch_info = pose_info
        if ok:
            self.finish_segment(
                success=True,
                notes="competition_qr_ok",
                extra={"qr_id": action["qr_id"], "qr_reason": reason, "qr_source": qr_source, "qr_latch": latch_info},
            )
            if qr_source == "latched_during_approach":
                self.get_logger().info(
                    f"QR_CHECK {action['name']} OK: accepted latched {action['qr_id']} seen during approach"
                )
            elif qr_source == "pose_reference_after_pickup":
                self.get_logger().info(
                    f"QR_CHECK {action['name']} OK: accepted pose reference for {action['qr_id']} while carrying load"
                )
            else:
                self.get_logger().info(f"QR_CHECK {action['name']} OK: saw fresh {action['qr_id']}")
            return True
        self.finish_segment(
            success=False,
            notes="qr_reacquire_failed",
            extra={"qr_id": action["qr_id"], "qr_reason": reason, "qr_latch": latch_info},
        )
        self.get_logger().error(f"QR_REACQUIRE_FAILED {action['name']}: {reason}")
        return False

    def dock_placeholder(self, name: str) -> bool:
        self.start_segment(
            segment_type="dock",
            name=name,
            target_name=name,
            target_pose=self.waypoint_pose_tuple(name),
            line_correction_enabled=self.enable_docking_line,
        )
        if self.current_segment is not None:
            self.current_segment["controller_source"] = "DOCKING"
        if self.nav_only or self.no_dock:
            self.skipped_docks.append(name)
            mode = "nav-only" if self.nav_only else "no-dock"
            self.finish_segment(success=True, result_status=None, notes=f"skipped_{mode}")
            self.get_logger().info(f"SKIP_DOCK {name}: {mode} mode; not sent to Nav2")
            return True

        if self.enable_docking_line and self.line_source == "camera_line" and name.startswith("A"):
            station = self.dock_station_name(name)
            staging_target = f"{station}_STAGING_CALIBRATED" if station else ""
            if staging_target and staging_target in self.waypoints:
                predock_ok, predock_metrics = self.wait_for_camera_line_dock_alignment(name, staging_target)
                if not predock_ok:
                    self.finish_segment(
                        success=False,
                        notes="camera_line_alignment_failed_before_dock",
                        extra={
                            "line_detected_at_docking_start": bool(predock_metrics.get("line_detected", False)),
                            "camera_line_predock_metrics": predock_metrics,
                        },
                    )
                    self.get_logger().error(
                        f"CAMERA_LINE_ALIGNMENT_FAILED_BEFORE_DOCK {name}: "
                        f"distance={predock_metrics.get('distance_to_target_m')} "
                        f"error_px={predock_metrics.get('filtered_error_px')} "
                        f"heading={predock_metrics.get('heading_error_rad')} "
                        f"centered={predock_metrics.get('robot_centered')} "
                        f"streak={predock_metrics.get('centered_streak')} "
                        f"reverse={predock_metrics.get('reverse_sign_mode')}"
                    )
                    return False

        self.get_logger().info(f"DOCK_ACTION {name}: stop -> rotate 180 -> short forward -> stop")
        self.stop()
        if not self.rotate_relative(math.pi):
            self.finish_segment(success=False, notes="rotate_180_failed")
            return False
        if self.current_segment is not None:
            self.current_segment["controller_source"] = self.competition_controller_source() if self.competition else (
                "line_sensor" if self.line_source == "line_sensor" else "camera_line" if self.enable_docking_line else "odom_drive"
            )

        line_detected_at_start = False
        controlled_cmd_produced = False
        dock_ok = False
        dock_geometry_reason = "dock_geometry_not_checked"
        dock_geometry_metrics: dict[str, Any] = {}
        dock_pose_line_status_snapshot: dict[str, Any] = {}
        self.log_dock_preflight(name)
        if self.enable_docking_line and self.line_source == "pose_line":
            self.set_pose_line_mode(True, "DOCKING", segment=name)
            deadline = self.runtime_deadline(3.0)
            while rclpy.ok() and self.runtime_now() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if (time.monotonic() - self.last_pose_line_status_time) <= 1.0 and self.last_pose_line_status.get("active"):
                    line_detected_at_start = True
                    break
            if not line_detected_at_start:
                self.finish_segment(
                    success=False,
                    notes="dock_pose_line_not_active",
                    extra={"line_detected_at_docking_start": False, "pose_line_status": self.last_pose_line_status},
                )
                self.get_logger().error(f"DOCK_POSE_LINE_FAILED {name}: controller not active")
                self.set_pose_line_mode(False, "OFF", segment=name)
                return False
            target_pose = self.waypoint_pose_tuple(name)
            route_pose = self.current_route_pose()
            target_distance = pose_distance(route_pose, target_pose) or self.docking_distance
            timeout_sec = max(abs(target_distance / max(self.docking_speed, 0.01)) * 5.0, 10.0)
            before = self.current_segment["cmd_vel_stats"].get("cmd_vel_pose_line_nonzero", 0)
            (
                dock_ok,
                dock_geometry_reason,
                dock_geometry_metrics,
                dock_pose_line_status_snapshot,
            ) = self.wait_for_dock_geometry(name, timeout_sec)
            if not dock_pose_line_status_snapshot and isinstance(self.last_pose_line_status, dict):
                dock_pose_line_status_snapshot = dict(self.last_pose_line_status)
            if not dock_ok:
                self.finish_segment(
                    success=False,
                    notes="dock_pose_line_geometry_timeout",
                    extra={
                        "line_detected_at_docking_start": True,
                        **self.dock_diagnostic_snapshot(
                            name,
                            geometry_ok=dock_ok,
                            geometry_reason=dock_geometry_reason,
                            geometry=dock_geometry_metrics,
                            pose_line_status=dock_pose_line_status_snapshot,
                        ),
                    },
                )
                self.set_pose_line_mode(False, "OFF", segment=name)
                return False
            after = self.current_segment["cmd_vel_stats"].get("cmd_vel_pose_line_nonzero", 0)
            controlled_cmd_produced = after > before
        elif self.enable_docking_line and self.line_source == "line_sensor":
            # Keep the proven D1->Q2 line_sensor baseline intact. For A-stations,
            # do not advance a blind docking distance after the 180-degree turn:
            # the line_sensor-only dock step can drift the robot away from the
            # pallet line before the dedicated A1 pre-dock realign/insertion flow
            # gets a chance to correct heading and cross-track.
            use_pose_line_dock_for_precise_station = bool(self.competition and (name.startswith("A") or name.startswith("B")))
            bypass_line_sensor_distance_for_a_station = bool(self.competition and name.startswith("A"))
            if use_pose_line_dock_for_precise_station:
                self.set_pose_line_mode(True, "DOCKING", segment=name)
                deadline = self.runtime_deadline(3.0)
                while rclpy.ok() and self.runtime_now() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    if (time.monotonic() - self.last_pose_line_status_time) <= 1.0 and self.last_pose_line_status.get("active"):
                        line_detected_at_start = True
                        break
                if not line_detected_at_start:
                    self.finish_segment(
                        success=False,
                        notes="dock_pose_line_not_active",
                        extra={"line_detected_at_docking_start": False, "pose_line_status": self.last_pose_line_status},
                    )
                    self.get_logger().error(f"DOCK_POSE_LINE_FAILED {name}: controller not active")
                    self.set_pose_line_mode(False, "OFF", segment=name)
                    return False
                target_pose = self.waypoint_pose_tuple(name)
                route_pose = self.current_route_pose()
                target_distance = pose_distance(route_pose, target_pose) or self.docking_distance
                timeout_sec = max(abs(target_distance / max(self.docking_speed, 0.01)) * 5.0, 10.0)
                before = self.current_segment["cmd_vel_stats"].get("cmd_vel_pose_line_nonzero", 0)
                (
                    dock_ok,
                    dock_geometry_reason,
                    dock_geometry_metrics,
                    dock_pose_line_status_snapshot,
                ) = self.wait_for_dock_geometry(name, timeout_sec)
                if not dock_pose_line_status_snapshot and isinstance(self.last_pose_line_status, dict):
                    dock_pose_line_status_snapshot = dict(self.last_pose_line_status)
                if not dock_ok:
                    self.finish_segment(
                        success=False,
                        notes="dock_pose_line_geometry_timeout",
                        extra={
                            "line_detected_at_docking_start": True,
                            **self.dock_diagnostic_snapshot(
                                name,
                                geometry_ok=dock_ok,
                                geometry_reason=dock_geometry_reason,
                                geometry=dock_geometry_metrics,
                                pose_line_status=dock_pose_line_status_snapshot,
                            ),
                        },
                    )
                    self.set_pose_line_mode(False, "OFF", segment=name)
                    return False
                after = self.current_segment["cmd_vel_stats"].get("cmd_vel_pose_line_nonzero", 0)
                controlled_cmd_produced = after > before
            else:
                self.set_line_sensor_mode(True, "DOCKING")
                deadline = self.runtime_deadline(3.0)
                while rclpy.ok() and self.runtime_now() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    if self.last_line_sensor_seen:
                        line_detected_at_start = True
                        break
                if not line_detected_at_start:
                    self.finish_segment(
                        success=False,
                        notes="dock_line_sensor_not_detected",
                        extra={"line_detected_at_docking_start": False},
                    )
                    self.get_logger().error(f"DOCK_LINE_SENSOR_FAILED {name}: line sensor not detected")
                    self.set_line_sensor_mode(False, "OFF")
                    return False
                if bypass_line_sensor_distance_for_a_station:
                    self.get_logger().info(
                        f"DOCK_LINE_SENSOR_BYPASS {name}: skipping fixed-distance dock advance and "
                        "handing off directly to A-station pre-dock realign"
                    )
                else:
                    timeout_sec = max(abs(self.docking_distance / max(self.docking_speed, 0.01)) * 4.0, 8.0)
                    before = self.current_segment["cmd_vel_stats"].get("cmd_vel_line_sensor_nonzero", 0)
                    if not self.wait_distance_without_publishing(self.docking_distance, timeout_sec):
                        self.finish_segment(
                            success=False,
                            notes="dock_line_sensor_distance_timeout",
                            extra={"line_detected_at_docking_start": True},
                        )
                        self.set_line_sensor_mode(False, "OFF")
                        return False
                    after = self.current_segment["cmd_vel_stats"].get("cmd_vel_line_sensor_nonzero", 0)
                    controlled_cmd_produced = after > before
                self.set_line_sensor_mode(False, "OFF")
        elif self.enable_docking_line:
            docking_profile = "b_side_dock_line" if name.startswith("B") else "dock_line"
            self.set_line_mode(True, "DOCKING", profile=docking_profile, context=name)
            deadline = self.runtime_deadline(3.0)
            while rclpy.ok() and self.runtime_now() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.line_detected():
                    line_detected_at_start = True
                    break
            if not line_detected_at_start:
                self.finish_segment(
                    success=False,
                    notes="dock_line_not_detected",
                    extra={"line_detected_at_docking_start": False},
                )
                self.get_logger().error(
                    f"DOCK_LINE_FAILED {name}: line not detected; status={self.last_line_status}"
                )
                self.set_line_mode(False, "OFF", profile=docking_profile, context=name)
                return False
            timeout_sec = max(abs(self.docking_distance / max(self.docking_speed, 0.01)) * 4.0, 8.0)
            before = self.current_segment["cmd_vel_stats"].get("cmd_vel_route_fused_nonzero", 0)
            if self.line_source == "camera_line" and name.startswith("A"):
                progress_ok, progress_reason, progress_metrics = self.wait_for_camera_dock_progress(name, timeout_sec)
                if not progress_ok:
                    self.finish_segment(
                        success=False,
                        notes=progress_reason.lower(),
                        extra={
                            "line_detected_at_docking_start": True,
                            "camera_dock_trace": progress_metrics,
                        },
                    )
                    self.get_logger().error(
                        f"{progress_reason} {name}: "
                        f"fork_tip_gap_start={progress_metrics.get('start_gap_m')} "
                        f"fork_tip_gap_end={progress_metrics.get('fork_tip_gap_m')} "
                        f"support_depth={progress_metrics.get('support_depth_to_load_center_m')} "
                        f"cmd_linear={progress_metrics.get('commanded_linear_velocity_mps')} "
                        f"cmd_angular={progress_metrics.get('commanded_angular_velocity_radps')}"
                    )
                    self.set_line_mode(False, "OFF", profile=docking_profile, context=name)
                    return False
            elif not self.wait_distance_without_publishing(self.docking_distance, timeout_sec):
                self.finish_segment(
                    success=False,
                    notes="dock_distance_timeout",
                    extra={"line_detected_at_docking_start": True},
                )
                self.set_line_mode(False, "OFF", profile=docking_profile, context=name)
                return False
            after = self.current_segment["cmd_vel_stats"].get("cmd_vel_route_fused_nonzero", 0)
            controlled_cmd_produced = after > before
            self.set_line_mode(False, "OFF", profile=docking_profile, context=name)
        else:
            if not self.drive_distance(self.docking_distance, self.docking_speed):
                self.finish_segment(success=False, notes="dock_drive_failed")
                return False

        self.stop()
        if not dock_ok:
            dock_ok, dock_geometry_reason, dock_geometry_metrics = self.evaluate_dock_geometry(name)
            if isinstance(self.last_pose_line_status, dict):
                dock_pose_line_status_snapshot = dict(self.last_pose_line_status)
        allow_a_station_predock_recovery = bool(
            self.competition
            and name.startswith("A")
            and self.line_source == "line_sensor"
        )
        if not dock_ok and not allow_a_station_predock_recovery:
            self.finish_segment(
                success=False,
                notes="dock_verification_failed",
                extra={
                    "line_detected_at_docking_start": line_detected_at_start,
                    "controlled_docking_cmd_produced": controlled_cmd_produced,
                    **self.dock_diagnostic_snapshot(
                        name,
                        geometry_ok=dock_ok,
                        geometry_reason=dock_geometry_reason,
                        geometry=dock_geometry_metrics,
                        pose_line_status=dock_pose_line_status_snapshot,
                    ),
                },
            )
            self.get_logger().error(f"DOCK_VERIFICATION_FAILED {name}: {dock_geometry_reason}")
            return False
        if not dock_ok and allow_a_station_predock_recovery:
            self.get_logger().warn(
                f"DOCK_VERIFICATION_DEFERRED {name}: {dock_geometry_reason}; "
                "continuing into A-station pre-dock realign/insertion flow"
            )
        if self.competition and name.startswith("A"):
            if self.line_source == "line_sensor":
                predock_sample = self.a1_dock_trace_sample(name)
                dock_geometry_metrics = dict(dock_geometry_metrics)
                dock_geometry_metrics["a1_predock_realign_skipped"] = True
                dock_geometry_metrics["a1_predock_sample"] = predock_sample
            insertion_ok, insertion_reason, insertion_metrics = self.a1_creep_insertion(name)
            retried_for_alignment = False
            if not insertion_ok and insertion_reason == "INSERTION_REQUIRES_REALIGN_RETRY":
                for _ in range(self.a1_dock_realign_retry_limit):
                    (
                        insertion_ok,
                        insertion_reason,
                        insertion_metrics,
                        dock_pose_line_status_snapshot,
                    ) = self.attempt_a1_dock_realign_retry(name)
                    retried_for_alignment = True
                    if insertion_ok:
                        dock_geometry_reason = "a1_insertion_retry_ok"
                        dock_geometry_metrics = insertion_metrics
                        break
            if insertion_ok:
                self.set_pose_line_mode(False, "OFF", segment=name)
                self.stop()
                stopped_ok, stop_metrics = self.wait_until_robot_stopped()
                insertion_metrics = dict(insertion_metrics)
                insertion_metrics["final_stop_linear_cmd_mps"] = stop_metrics.get("linear_cmd")
                insertion_metrics["final_stop_angular_cmd_radps"] = stop_metrics.get("angular_cmd")
                insertion_metrics["pose_line_disabled_after_insertion_ok"] = True
                insertion_metrics["robot_stopped_after_insertion_ok"] = stopped_ok
                if not stopped_ok:
                    self.finish_segment(
                        success=False,
                        notes="dock_not_stopped",
                        extra={
                            "line_detected_at_docking_start": line_detected_at_start,
                            "controlled_docking_cmd_produced": controlled_cmd_produced,
                            **self.dock_diagnostic_snapshot(
                                name,
                                geometry_ok=False,
                                geometry_reason="DOCK_NOT_STOPPED",
                                geometry=insertion_metrics,
                                pose_line_status=dock_pose_line_status_snapshot,
                            ),
                            "a1_insertion_metrics": insertion_metrics,
                        },
                    )
                    self.get_logger().error(f"DOCK_NOT_STOPPED {name}")
                    return False
                dock_geometry_reason = "a1_insertion_ok"
                dock_geometry_metrics = insertion_metrics
                robot_pose = insertion_metrics.get("robot_pose")
                if isinstance(robot_pose, list) and len(robot_pose) >= 3:
                    dock_pose_line_status_snapshot = dict(dock_pose_line_status_snapshot)
                    dock_pose_line_status_snapshot.update(
                        {
                            "mode": "DOCKING",
                            "segment": name,
                            "pose_x": float(robot_pose[0]),
                            "pose_y": float(robot_pose[1]),
                            "pose_yaw": float(robot_pose[2]),
                            "cross_track_error_m": insertion_metrics.get("cross_track_error_m"),
                            "heading_error_rad": insertion_metrics.get("heading_error_rad"),
                            "progress_ratio": insertion_metrics.get("progress_ratio"),
                            "linear_cmd": 0.0,
                            "angular_cmd": 0.0,
                            "completion_flag": True,
                            "stop_reason": "dock_geometry_complete",
                        }
                    )
                insertion_metrics["realign_retry_performed"] = retried_for_alignment
            else:
                if self.enable_docking_line and self.line_source == "pose_line":
                    self.set_pose_line_mode(False, "OFF", segment=name)
                self.finish_segment(
                    success=False,
                    notes=insertion_reason.lower(),
                    extra={
                        "line_detected_at_docking_start": line_detected_at_start,
                        "controlled_docking_cmd_produced": controlled_cmd_produced,
                        **self.dock_diagnostic_snapshot(
                            name,
                            geometry_ok=False,
                            geometry_reason=insertion_reason,
                            geometry=dock_geometry_metrics,
                            pose_line_status=dock_pose_line_status_snapshot,
                        ),
                        "a1_insertion_metrics": insertion_metrics,
                        "realign_retry_performed": retried_for_alignment,
                    },
                )
                self.get_logger().error(f"{insertion_reason} {name}")
                return False
        elif self.enable_docking_line and self.line_source == "pose_line":
            self.set_pose_line_mode(False, "OFF", segment=name)
        validation_lift_verified = False
        validation_lift_reason = "not_required"
        validation_lift_metrics: dict[str, Any] = {}
        pre_lift_pallet_state = None
        fork_command = "LIFT_UP" if name.startswith("A") else "LIFT_DOWN"
        lift_metrics: dict[str, Any] = dict(dock_geometry_metrics)
        if self.competition and name.startswith("A"):
            pre_lift_pallet_state = self.get_entity_state(
                self.pallet_model_entity_name(self.dock_station_name(name) or "") or "",
                timeout_sec=0.5,
            )
            lift_ready, lift_reason, lift_metrics = self.evaluate_lift_ready_geometry(
                name,
                pose_line_status_override=dock_pose_line_status_snapshot,
                require_controller_complete=dock_geometry_reason != "dock_geometry_physical_ok",
            )
            if (
                not lift_ready
                and lift_reason in {"LINE_ALIGNMENT_FAILED", "DOCK_ALIGNMENT_NOT_SAFE"}
                and name.startswith("A")
                and self.a1_inserted_lift_alignment_ok(lift_metrics)
            ):
                self.get_logger().warn(
                    f"A1_INSERTED_SAFE_LIFT_OVERRIDE {name}: "
                    f"reason={lift_reason} "
                    f"cross_track={lift_metrics.get('dock_cross_track_error_m')} "
                    f"heading={lift_metrics.get('dock_heading_error_rad')} "
                    f"support_depth={lift_metrics.get('support_depth_to_load_center_m')} "
                    f"left_depth={lift_metrics.get('left_fork_insertion_depth_m')} "
                    f"right_depth={lift_metrics.get('right_fork_insertion_depth_m')}"
                )
                lift_ready = True
                lift_reason = "dock_lift_ready_a1_inserted_safe"
            if (
                not lift_ready
                and lift_reason == "LINE_ALIGNMENT_FAILED"
                and name.startswith("A")
                and isinstance(lift_metrics.get("support_depth_to_load_center_m"), (int, float))
                and float(lift_metrics.get("support_depth_to_load_center_m")) >= self.dock_lift_support_margin_m
            ):
                settle_eligible = self.a1_pre_lift_heading_settle_eligible(lift_metrics)
                correction_ok = False
                correction_metrics = dict(lift_metrics)
                correction_status = dict(dock_pose_line_status_snapshot)
                if settle_eligible:
                    (
                        correction_ok,
                        _correction_reason,
                        correction_metrics,
                        correction_status,
                    ) = self.attempt_a1_lift_alignment_correction(name)
                    if correction_ok:
                        dock_geometry_reason = "dock_lift_alignment_recovered"
                        dock_geometry_metrics = correction_metrics
                        dock_pose_line_status_snapshot = correction_status
                        lift_ready, lift_reason, lift_metrics = self.evaluate_lift_ready_geometry(
                            name,
                            pose_line_status_override=dock_pose_line_status_snapshot,
                            require_controller_complete=False,
                        )
                if not lift_ready:
                    retry_ok = False
                    retry_reason = "A1_DOCK_REALIGN_RETRY_FAILED"
                    retry_metrics = dict(correction_metrics if settle_eligible else lift_metrics)
                    retry_status = dict(correction_status if settle_eligible else dock_pose_line_status_snapshot)
                    for _ in range(self.a1_dock_realign_retry_limit):
                        (
                            retry_ok,
                            retry_reason,
                            retry_metrics,
                            retry_status,
                        ) = self.attempt_a1_dock_realign_retry(name)
                        if not retry_ok:
                            continue
                        dock_geometry_reason = "a1_insertion_retry_ok"
                        dock_geometry_metrics = retry_metrics
                        dock_pose_line_status_snapshot = retry_status
                        lift_ready, lift_reason, lift_metrics = self.evaluate_lift_ready_geometry(
                            name,
                            pose_line_status_override=dock_pose_line_status_snapshot,
                            require_controller_complete=False,
                        )
                        break
                    if not lift_ready and not retry_ok:
                        lift_reason = retry_reason
                        lift_metrics = dict(retry_metrics)
                        dock_pose_line_status_snapshot = dict(retry_status)
            self.log_lift_gate(
                name,
                dock_geometry_reason=dock_geometry_reason,
                lift_reason=lift_reason,
                lift_metrics=lift_metrics,
                pose_line_status=dock_pose_line_status_snapshot,
                pre_lift_pallet_state=pre_lift_pallet_state,
            )
            if not lift_ready:
                if self.enable_docking_line and self.line_source == "pose_line":
                    self.set_pose_line_mode(False, "OFF", segment=name)
                self.finish_segment(
                    success=False,
                    notes=lift_reason.lower(),
                    extra={
                        "line_detected_at_docking_start": line_detected_at_start,
                        "controlled_docking_cmd_produced": controlled_cmd_produced,
                        **self.dock_diagnostic_snapshot(
                            name,
                            geometry_ok=dock_ok,
                            geometry_reason=dock_geometry_reason,
                            geometry=lift_metrics,
                            pose_line_status=dock_pose_line_status_snapshot,
                        ),
                        "lift_allowed": False,
                        "lift_reason": lift_reason,
                    },
                    )
                self.get_logger().error(f"{lift_reason} {name}")
                return False
            if fork_command == "LIFT_UP":
                validation_stage_metrics: list[dict[str, Any]] = []
                validation_command = f"LIFT_TO:{self.validation_lift_height_m:.3f}"
                validation_reason = "validation_not_run"
                for target_height in self.validation_lift_stage_targets_m:
                    stage_command = f"LIFT_TO:{float(target_height):.3f}"
                    validation_ok, validation_reason = self.send_fork_command(stage_command)
                    if not validation_ok:
                        validation_command = stage_command
                        break
                    _, _, stage_metrics = self.capture_lift_metrics(
                        name,
                        pre_lift_pallet_state=pre_lift_pallet_state,
                        stage_label=f"validation_lift_{float(target_height):.3f}",
                    )
                    stage_metrics["stage_target_height_m"] = float(target_height)
                    validation_stage_metrics.append(stage_metrics)
                    self.get_logger().info(
                        f"VALIDATION_LIFT_STAGE {name} target={float(target_height):.3f} "
                        f"fork_h={stage_metrics.get('fork_height_after_lift_m')} "
                        f"pallet_z={stage_metrics.get('post_lift_pallet_state', {}).get('position', [None, None, None])[2]} "
                        f"roll={stage_metrics.get('pallet_roll_rad')} "
                        f"pitch={stage_metrics.get('pallet_pitch_rad')} "
                        f"left_depth={stage_metrics.get('left_fork_insertion_depth_after_lift_m')} "
                        f"right_depth={stage_metrics.get('right_fork_insertion_depth_after_lift_m')}"
                    )
                    validation_command = stage_command
                if not validation_ok:
                    if self.enable_docking_line and self.line_source == "pose_line":
                        self.set_pose_line_mode(False, "OFF", segment=name)
                    self.finish_segment(
                        success=False,
                        notes="fork_validation_failed",
                        extra={
                            "line_detected_at_docking_start": line_detected_at_start,
                            "controlled_docking_cmd_produced": controlled_cmd_produced,
                            **self.dock_diagnostic_snapshot(
                                name,
                                geometry_ok=dock_ok,
                                geometry_reason=dock_geometry_reason,
                                geometry=lift_metrics,
                                pose_line_status=dock_pose_line_status_snapshot,
                            ),
                            "fork_command": validation_command,
                            "fork_reason": validation_reason,
                            "lift_allowed": False,
                            "pre_lift_pallet_state": pre_lift_pallet_state,
                            "validation_lift_stages": validation_stage_metrics,
                        },
                    )
                    self.get_logger().error(f"FORK_VALIDATION_FAILED {name}: {validation_reason}")
                    return False
                validation_lift_verified, validation_lift_reason, validation_lift_metrics = self.verify_lifted_load(
                    name,
                    pre_lift_pallet_state=pre_lift_pallet_state,
                    min_z_increase_m=self.validation_lift_min_z_increase_m,
                    max_tilt_rad=self.validation_lift_max_tilt_rad,
                    required_support_depth_m=self.dock_lift_support_margin_m,
                    stage_label="validation_lift",
                )
                validation_lift_metrics["validation_lift_stages"] = validation_stage_metrics
                if not validation_lift_verified:
                    self.send_fork_command("LIFT_DOWN")
                    if self.enable_docking_line and self.line_source == "pose_line":
                        self.set_pose_line_mode(False, "OFF", segment=name)
                    self.finish_segment(
                        success=False,
                        notes=validation_lift_reason.lower(),
                        extra={
                            "line_detected_at_docking_start": line_detected_at_start,
                            "controlled_docking_cmd_produced": controlled_cmd_produced,
                            **self.dock_diagnostic_snapshot(
                                name,
                                geometry_ok=dock_ok,
                                geometry_reason=dock_geometry_reason,
                                geometry=lift_metrics,
                                pose_line_status=dock_pose_line_status_snapshot,
                            ),
                            "fork_command": validation_command,
                            "fork_reason": validation_reason,
                            "lift_allowed": False,
                            "validation_lift_verified": validation_lift_verified,
                            "validation_lift_reason": validation_lift_reason,
                            **validation_lift_metrics,
                        },
                    )
                    self.get_logger().error(f"{validation_lift_reason} {name}")
                    return False
        elif self.competition and name.startswith("B"):
            validation_lift_reason = "not_required_for_unload"
            pre_lift_pallet_state = self.get_entity_state(
                self.carried_pallet_entity_name() or "",
                timeout_sec=0.5,
            )
        fork_ok, fork_reason = self.send_fork_command(fork_command)
        unload_verified = False
        unload_verify_reason = "unload_verification_not_run"
        unload_verify_metrics: dict[str, Any] = {}
        if name.startswith("B") and fork_command == "LIFT_DOWN":
            unload_verified, unload_verify_reason, unload_verify_metrics = self.verify_unloaded_load(
                name,
                pre_unload_pallet_state=pre_lift_pallet_state,
            )
            if not fork_ok and unload_verified:
                self.get_logger().warn(
                    f"B_DOCK_UNLOAD_OVERRIDE {name}: accepting unload despite fork timeout; "
                    f"fork_reason={fork_reason} z_drop={unload_verify_metrics.get('pallet_z_drop_m')} "
                    f"fork_h={unload_verify_metrics.get('fork_height_after_unload_m')}"
                )
                fork_ok = True
                fork_reason = f"{fork_reason}; {unload_verify_reason}"
        if not fork_ok:
            timed_out_load_ok = False
            timed_out_load_reason = "load_verification_skipped"
            timed_out_load_metrics: dict[str, Any] = {}
            if fork_command == "LIFT_UP":
                timed_out_load_ok, timed_out_load_reason, timed_out_load_metrics = self.verify_lifted_load(
                    name,
                    pre_lift_pallet_state=pre_lift_pallet_state,
                )
            if self.enable_docking_line and self.line_source == "pose_line":
                self.set_pose_line_mode(False, "OFF", segment=name)
            self.finish_segment(
                success=False,
                notes="fork_command_failed",
                extra={
                    "line_detected_at_docking_start": line_detected_at_start,
                    "controlled_docking_cmd_produced": controlled_cmd_produced,
                    **self.dock_diagnostic_snapshot(
                        name,
                        geometry_ok=dock_ok,
                        geometry_reason=dock_geometry_reason,
                        geometry=lift_metrics,
                        pose_line_status=dock_pose_line_status_snapshot,
                    ),
                    "fork_command": fork_command,
                    "fork_reason": fork_reason,
                    "lift_allowed": True,
                    "validation_lift_verified": validation_lift_verified,
                    "validation_lift_reason": validation_lift_reason,
                    **validation_lift_metrics,
                    "pre_lift_pallet_state": pre_lift_pallet_state,
                    "load_verified": timed_out_load_ok,
                    "load_reason": timed_out_load_reason,
                    **timed_out_load_metrics,
                    "unload_verified": unload_verified,
                    "unload_reason": unload_verify_reason,
                    **unload_verify_metrics,
                },
            )
            self.get_logger().error(f"FORK_COMMAND_FAILED {name}: {fork_reason}")
            return False
        self.get_logger().info(f"FORK_COMMAND_OK {name}: {fork_reason}")
        if name.startswith("A"):
            load_ok, load_reason, load_metrics = self.verify_lifted_load(
                name,
                pre_lift_pallet_state=pre_lift_pallet_state,
            )
            if not load_ok:
                if self.enable_docking_line and self.line_source == "pose_line":
                    self.set_pose_line_mode(False, "OFF", segment=name)
                self.finish_segment(
                    success=False,
                    notes=load_reason.lower(),
                    extra={
                        "line_detected_at_docking_start": line_detected_at_start,
                        "controlled_docking_cmd_produced": controlled_cmd_produced,
                        **self.dock_diagnostic_snapshot(
                            name,
                            geometry_ok=dock_ok,
                            geometry_reason=dock_geometry_reason,
                            geometry=lift_metrics,
                            pose_line_status=dock_pose_line_status_snapshot,
                        ),
                        "fork_command": fork_command,
                        "fork_reason": fork_reason,
                        "lift_allowed": True,
                        "validation_lift_verified": validation_lift_verified,
                        "validation_lift_reason": validation_lift_reason,
                        **validation_lift_metrics,
                        "load_verified": False,
                        "load_reason": load_reason,
                        **load_metrics,
                    },
                )
                self.get_logger().error(f"LOAD_VERIFICATION_FAILED {name}: {load_reason}")
                return False
        elif name.startswith("B") and fork_command == "LIFT_DOWN":
            if not unload_verified:
                if self.enable_docking_line and self.line_source == "pose_line":
                    self.set_pose_line_mode(False, "OFF", segment=name)
                self.finish_segment(
                    success=False,
                    notes=unload_verify_reason.lower(),
                    extra={
                        "line_detected_at_docking_start": line_detected_at_start,
                        "controlled_docking_cmd_produced": controlled_cmd_produced,
                        **self.dock_diagnostic_snapshot(
                            name,
                            geometry_ok=dock_ok,
                            geometry_reason=dock_geometry_reason,
                            geometry=lift_metrics,
                            pose_line_status=dock_pose_line_status_snapshot,
                        ),
                        "fork_command": fork_command,
                        "fork_reason": fork_reason,
                        "lift_allowed": True,
                        "pre_lift_pallet_state": pre_lift_pallet_state,
                        "unload_verified": False,
                        "unload_reason": unload_verify_reason,
                        **unload_verify_metrics,
                    },
                )
                self.get_logger().error(f"UNLOAD_VERIFICATION_FAILED {name}: {unload_verify_reason}")
                return False
        elif name.startswith("A"):
            self.get_logger().info(f"FORK_PLACEHOLDER {name}: LIFT_UP not sent; logging only")
        elif name.startswith("B"):
            self.get_logger().info(f"FORK_PLACEHOLDER {name}: LIFT_DOWN not sent; logging only")
        exit_rotate_required = not (name.startswith("A") and fork_command == "LIFT_UP")
        exit_rotate_succeeded = True
        exit_rotate_reason = "not_required" if not exit_rotate_required else "pending"

        if not self.drive_distance(self.reverse_distance, -self.docking_speed):
            if self.enable_docking_line and self.line_source == "pose_line":
                self.set_pose_line_mode(False, "OFF", segment=name)
            self.finish_segment(success=False, notes="dock_reverse_failed")
            return False

        if exit_rotate_required:
            if not self.rotate_relative(math.pi):
                if self.enable_docking_line and self.line_source == "pose_line":
                    self.set_pose_line_mode(False, "OFF", segment=name)
                self.finish_segment(success=False, notes="rotate_back_failed")
                return False
            exit_rotate_reason = "rotate_back_ok"
        else:
            self.get_logger().warn(
                f"A1_EXIT_ROTATE_SKIPPED {name}: carrying load after successful lift; "
                "handing off directly to route follow"
            )

        if self.enable_docking_line and self.line_source == "pose_line":
            self.set_pose_line_mode(False, "OFF", segment=name)
        self.finish_segment(
            success=True,
            notes="dock_placeholder_done",
            extra={
                "line_detected_at_docking_start": line_detected_at_start,
                "controlled_docking_cmd_produced": controlled_cmd_produced,
                **self.dock_diagnostic_snapshot(
                    name,
                    geometry_ok=dock_ok,
                    geometry_reason=dock_geometry_reason,
                    geometry=dock_geometry_metrics,
                    pose_line_status=dock_pose_line_status_snapshot,
                ),
                "lift_allowed": not self.competition,
                "validation_lift_verified": validation_lift_verified,
                "validation_lift_reason": validation_lift_reason,
                "exit_rotate_required": exit_rotate_required,
                "exit_rotate_succeeded": exit_rotate_succeeded,
                "exit_rotate_reason": exit_rotate_reason,
                **validation_lift_metrics,
                "load_verified": load_ok if fork_command == "LIFT_UP" else (not self.competition),
                "load_reason": load_reason if fork_command == "LIFT_UP" else "load_verification_not_required",
                **(load_metrics if fork_command == "LIFT_UP" else {}),
            },
        )
        self.get_logger().info(f"DOCK_ACTION_DONE {name}")
        return True

    def diagnose_b3_staging_failure(self) -> None:
        self.get_logger().warn("Running B3_STAGING diagnostics: return to D6, then direct Nav2 with line correction OFF.")
        result: dict[str, Any] = {"diagnostic": "B3_STAGING"}
        if not self.navigate("D6", line_correction_enabled=False, segment_name="DIAG_RETURN_D6"):
            result["return_to_d6"] = "failed"
            self.diagnostic_results.append(result)
            return
        result["return_to_d6"] = "succeeded"
        direct_success = self.navigate(
            "B3_STAGING",
            line_correction_enabled=False,
            segment_name="DIAG_DIRECT_D6_TO_B3_STAGING_OFF",
        )
        result["direct_d6_to_b3_staging_off"] = "succeeded" if direct_success else "failed"
        if self.segment_records:
            last = self.segment_records[-1]
            result["diagnostic_segment_status"] = last.get("result_status")
            result["diagnostic_costmap_snapshot"] = last.get("costmap_snapshot")
        self.diagnostic_results.append(result)

    def write_outputs(self, success: bool) -> None:
        if self.output_dir is None:
            return
        active_pose = self.pose_for_segment(self.current_segment)
        active_target_pose = self.current_segment.get("target_pose") if isinstance(self.current_segment, dict) else None
        summary = {
            "route": self.route_name,
            "dock_only_target": self.dock_only_target,
            "success": success,
            "line_source": self.line_source,
            "line_primary": self.line_primary,
            "line_sensor_primary": self.line_sensor_primary,
            "competition": self.competition,
            "resume_from_current": self.resume_from_current,
            "start_from_start": self.start_from_start,
            "stop_after": self.stop_after,
            "reached": self.reached,
            "skipped_docks": self.skipped_docks,
            "last_successful_segment": self.segment_records[-1]["segment_name"] if self.segment_records else None,
            "first_failed_segment": self.first_failed_segment,
            "failure_reason": self.failure_reason,
            "diagnostic_results": self.diagnostic_results,
            "last_active_action": self.current_segment.get("segment_name") if isinstance(self.current_segment, dict) else None,
            "last_active_action_type": self.current_segment.get("segment_type") if isinstance(self.current_segment, dict) else None,
            "last_active_pose": list(active_pose) if active_pose is not None else None,
            "last_active_target_pose": list(active_target_pose) if active_target_pose is not None else None,
            "last_active_distance_to_target_m": pose_distance(active_pose, active_target_pose),
        }
        if self.segment_records:
            succeeded = [record["segment_name"] for record in self.segment_records if record["success"]]
            summary["last_successful_segment"] = succeeded[-1] if succeeded else None
        self.summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        self.cmd_vel_stats_path.write_text(json.dumps(self.cmd_vel_totals, indent=2), encoding="utf-8")
        self.failure_reason_path.write_text((self.failure_reason or "none") + "\n", encoding="utf-8")
        if self.path_trace_path is not None:
            fieldnames = [
                "wall_time_sec",
                "sim_time_sec",
                "map_x",
                "map_y",
                "map_yaw",
                "odom_x",
                "odom_y",
                "odom_yaw",
                "active_action_type",
                "active_action_name",
                "target_name",
                "target_x",
                "target_y",
                "target_yaw",
                "distance_to_target_m",
                "line_segment_name",
                "line_segment_section",
                "segment_start_waypoint",
                "segment_start_x",
                "segment_start_y",
                "segment_start_yaw",
                "segment_end_waypoint",
                "segment_end_x",
                "segment_end_y",
                "segment_end_yaw",
                "cross_track_error_m",
                "heading_error_rad",
            ]
            with self.path_trace_path.open("w", encoding="utf-8", newline="") as fp:
                writer = csv.DictWriter(fp, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.path_trace_records)
        if self.segment_results_fp is not None:
            self.segment_results_fp.close()

    def execute(self) -> bool:
        success = False
        try:
            if not self.wait_for_runtime():
                self.failure_reason = self.failure_reason or "runtime_not_ready"
                return False

            actions_to_run = self.actions
            if self.competition:
                actions_to_run = self.build_competition_actions()
                if self.resume_from_current and not self.start_from_start:
                    self.get_logger().info(
                        "COMPETITION mission plan from current pose: "
                        + " -> ".join(f"{a['type']}:{a['name']}" for a in actions_to_run)
                    )
                else:
                    self.get_logger().info(
                        "COMPETITION mission plan: "
                        + " -> ".join(f"{a['type']}:{a['name']}" for a in actions_to_run)
                    )

            for offset, action in enumerate(actions_to_run, start=1):
                action_type = action["type"]
                name = action["name"]
                self.get_logger().info(f"ACTION {offset}/{len(actions_to_run)} {action_type.upper()} {name}")
                self.log_action_context(action, offset, len(actions_to_run))
                if action_type == "nav":
                    resolved_target = self.resolve_waypoint_name(action["target"])
                    if self.competition:
                        nav_ok = self.competition_navigate(action["target"])
                    elif self.line_sensor_primary:
                        nav_ok = self.line_sensor_primary_navigate(action["target"])
                    elif self.line_primary:
                        nav_ok = self.line_primary_navigate(action["target"])
                    else:
                        nav_ok = self.navigate(action["target"])
                    if not nav_ok:
                        self.failure_reason = f"nav_failed:{name}"
                        if self.full_test and action["target"] == "B3_STAGING":
                            self.diagnose_b3_staging_failure()
                        return False
                    if self.stop_after_reached(action, resolved_target):
                        success = True
                        return True
                elif action_type == "transit_nav":
                    resolved_target = self.resolve_waypoint_name(action["target"])
                    nav_ok = self.competition_navigate(action["target"])
                    if not nav_ok:
                        self.failure_reason = f"transit_nav_failed:{name}"
                        return False
                    if self.stop_after_reached(action, resolved_target):
                        success = True
                        return True
                elif action_type == "line_approach":
                    resolved_target = self.resolve_waypoint_name(action["target"])
                    nav_ok = self.competition_line_approach(action["target"])
                    if not nav_ok:
                        self.failure_reason = f"line_approach_failed:{name}"
                        return False
                    if self.stop_after_reached(action, resolved_target):
                        success = True
                        return True
                elif action_type == "line_follow":
                    resolved_target = self.resolve_waypoint_name(action["target"])
                    nav_ok = self.competition_line_follow(action["target"], action.get("segment", name))
                    if not nav_ok:
                        self.failure_reason = f"line_follow_failed:{name}"
                        return False
                    if self.stop_after_reached(action, resolved_target):
                        success = True
                        return True
                elif action_type == "qr_check":
                    if self.competition:
                        if not self.competition_qr_check(action):
                            self.failure_reason = f"qr_failed:{name}"
                            return False
                        if self.stop_after_reached(action, self.resolve_waypoint_name(action["target"])):
                            success = True
                            return True
                    else:
                        self.qr_check(action)
                elif action_type == "dock":
                    if not self.dock_placeholder(action["target"]):
                        self.failure_reason = f"dock_failed:{name}"
                        return False
                    if self.stop_after_reached(action, self.resolve_waypoint_name(action["target"])):
                        success = True
                        return True
                elif action_type == "duplicate_nav_skip":
                    self.get_logger().info(f"SKIP_DUPLICATE_NAV {name}")
                else:
                    self.failure_reason = f"unknown_action_type:{action_type}"
                    self.get_logger().error(f"Unknown action type: {action_type}")
                    return False

            self.get_logger().info(
                "ROUTE_DONE reached="
                + ",".join(self.reached)
                + " skipped_docks="
                + ",".join(self.skipped_docks)
            )
            success = True
            return True
        finally:
            if not success and self.current_segment is not None:
                active_pose = self.pose_for_segment(self.current_segment)
                active_target_pose = self.current_segment.get("target_pose")
                self.finalize_active_segment_failure(
                    notes="segment_interrupted",
                    failure_reason=self.failure_reason or f"interrupted:{self.current_segment.get('segment_name')}",
                    extra={
                        "last_active_action": self.current_segment.get("segment_name"),
                        "last_active_action_type": self.current_segment.get("segment_type"),
                        "final_pose_live": list(active_pose) if active_pose is not None else None,
                        "target_pose_live": list(active_target_pose) if active_target_pose is not None else None,
                        "final_distance_to_target_live_m": pose_distance(active_pose, active_target_pose),
                        "last_cmd_vel": {
                            "linear": self.last_pose_line_status.get("linear_cmd") if isinstance(self.last_pose_line_status, dict) else None,
                            "angular": self.last_pose_line_status.get("angular_cmd") if isinstance(self.last_pose_line_status, dict) else None,
                        },
                        "stopped_in_function": "execute",
                        "stopped_in_state": "interrupted_before_finish_segment",
                    },
                )
            self.set_line_mode(False, "OFF")
            self.set_line_sensor_mode(False, "OFF")
            self.set_pose_line_mode(False, "OFF")
            self.stop()
            self.shutdown_started = True
            self.write_outputs(success)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Execute a measured TEKNOFEST route.")
    parser.add_argument("--route", default=None, help="Scenario name such as A1_TO_B3")
    parser.add_argument("--dock-only", default=None, help="Run only this dock segment from the current pose, e.g. A1_DOCK")
    parser.add_argument("--waypoints-file", type=Path, default=None)
    parser.add_argument("--routes-file", type=Path, default=None)
    parser.add_argument("--execute", action="store_true", help="Run the route. Required for motion.")
    parser.add_argument("--nav-only", action="store_true", help="Navigate road/staging points and skip DOCK actions.")
    parser.add_argument("--no-dock", action="store_true", help="Navigate route and explicitly skip DOCK actions.")
    parser.add_argument("--enable-line-correction", action="store_true")
    parser.add_argument("--enable-docking-line", action="store_true")
    parser.add_argument("--line-primary", action="store_true")
    parser.add_argument("--line-sensor-primary", action="store_true")
    parser.add_argument(
        "--line-source",
        choices=["camera", "camera_line", "pose_line", "line_sensor"],
        default="pose_line",
    )
    parser.add_argument("--competition", action="store_true")
    parser.add_argument("--resume-from-current", action="store_true")
    parser.add_argument("--start-from-start", action="store_true")
    parser.add_argument("--stop-after", default=None, help="Stop successfully after reaching this NAV waypoint")
    parser.add_argument(
        "--override-waypoint",
        action="append",
        default=[],
        help="Temporary waypoint alias override in the form ORIGINAL:=TEST_NAME",
    )
    parser.add_argument("--full-test", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--docking-distance", type=float, default=0.35)
    parser.add_argument("--reverse-distance", type=float, default=0.25)
    parser.add_argument("--docking-speed", type=float, default=0.07)
    parser.add_argument("--turn-speed", type=float, default=0.35)
    parser.add_argument("--line-primary-goal-tolerance-m", type=float, default=0.45)
    parser.add_argument("--line-primary-corridor-width-m", type=float, default=0.45)
    parser.add_argument("--line-primary-min-progress-m", type=float, default=0.80)
    parser.add_argument("--line-primary-segment-timeout-sec", type=float, default=120.0)
    parser.add_argument("--line-approach-distance-tolerance-m", type=float, default=0.12)
    parser.add_argument("--line-approach-cross-track-threshold-m", type=float, default=0.15)
    parser.add_argument("--line-approach-heading-threshold-rad", type=float, default=0.20)
    parser.add_argument("--line-approach-min-progress-ratio", type=float, default=0.95)
    parser.add_argument("--competition-qr-reacquire-timeout-sec", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        validation = validate_route_configs(args.waypoints_file, args.routes_file)
        waypoints = validation["waypoints"]
        dock_only_target = args.dock_only.upper() if args.dock_only else None
        if dock_only_target is not None:
            if dock_only_target not in waypoints:
                raise ValueError(f"Unknown dock waypoint '{dock_only_target}'")
            if not is_dock_target(dock_only_target):
                raise ValueError(f"--dock-only expects a *_DOCK waypoint, got '{dock_only_target}'")
            scenario_name = f"DOCK_ONLY_{dock_only_target}"
            actions = [{"type": "dock", "name": dock_only_target, "target": dock_only_target}]
        else:
            if not args.route:
                raise ValueError("--route is required unless --dock-only is used")
            scenario_name = args.route.upper()
            if scenario_name not in validation["scenarios"]:
                raise ValueError(f"Unknown scenario '{scenario_name}'")
            actions = validation["route_actions"][scenario_name]
    except (OSError, ValueError) as exc:
        print(f"route_executor error: {exc}", file=sys.stderr, flush=True)
        return 1

    waypoint_overrides: dict[str, str] = {}
    for item in args.override_waypoint:
        separator = ":=" if ":=" in item else "=" if "=" in item else None
        if separator is None:
            print(
                f"route_executor error: invalid override '{item}', expected ORIGINAL:=TEST_NAME or ORIGINAL=TEST_NAME",
                file=sys.stderr,
                flush=True,
            )
            return 1
        original, replacement = item.split(separator, 1)
        original = original.strip().upper()
        replacement = replacement.strip().upper()
        if not original or not replacement:
            print(
                f"route_executor error: invalid override '{item}', expected ORIGINAL:=TEST_NAME or ORIGINAL=TEST_NAME",
                file=sys.stderr,
                flush=True,
            )
            return 1
        if original not in waypoints:
            print(
                f"route_executor error: override source '{original}' is not a known waypoint",
                file=sys.stderr,
                flush=True,
            )
            return 1
        if replacement not in waypoints:
            print(
                f"route_executor error: override target '{replacement}' is not a known waypoint",
                file=sys.stderr,
                flush=True,
            )
            return 1
        waypoint_overrides[original] = replacement

    print(f"Scenario: {scenario_name}", flush=True)
    print(
        "Action plan: "
        + " -> ".join(f"{action['type']}:{action['name']}" for action in actions),
        flush=True,
    )
    if waypoint_overrides:
        print(f"Waypoint overrides: {waypoint_overrides}", flush=True)

    if args.nav_only and args.no_dock:
        print("route_executor error: use either --nav-only or --no-dock, not both", file=sys.stderr)
        return 1

    enable_line_correction = args.enable_line_correction
    enable_docking_line = args.enable_docking_line
    line_primary = args.line_primary
    line_sensor_primary = args.line_sensor_primary
    competition = args.competition
    if args.full_test:
        enable_line_correction = True
        enable_docking_line = True
    if competition:
        enable_docking_line = True

    output_dir = args.output_dir
    default_output_root = (
        Path.home() / ".local" / "state" / "tekno_robot_simulation"
    )
    if args.full_test and output_dir is None:
        output_dir = default_output_root / f"full_route_{scenario_name}"
    if competition and output_dir is None:
        output_dir = default_output_root / f"competition_{scenario_name}"
    if dock_only_target and output_dir is None:
        output_dir = default_output_root / f"dock_only_{dock_only_target}"
    if args.line_primary and args.stop_after and output_dir is None:
        output_dir = default_output_root / f"line_capture_{args.stop_after.upper()}"
    if line_primary and output_dir is None:
        output_dir = default_output_root / f"line_primary_{scenario_name}"

    rclpy.init()
    node = RouteExecutor(
        actions,
        waypoints,
        route_name=scenario_name,
        dock_only_target=dock_only_target,
        nav_only=args.nav_only,
        no_dock=args.no_dock,
        enable_line_correction=enable_line_correction,
        enable_docking_line=enable_docking_line,
        line_primary=line_primary,
        line_sensor_primary=line_sensor_primary,
        line_source=args.line_source,
        competition=competition,
        resume_from_current=args.resume_from_current,
        start_from_start=args.start_from_start,
        stop_after=(args.stop_after.upper() if args.stop_after else None),
        full_test=args.full_test,
        output_dir=output_dir,
        docking_speed=args.docking_speed,
        docking_distance=args.docking_distance,
        reverse_distance=args.reverse_distance,
        turn_speed=args.turn_speed,
        waypoint_overrides=waypoint_overrides,
        line_primary_goal_tolerance_m=args.line_primary_goal_tolerance_m,
        line_primary_corridor_width_m=args.line_primary_corridor_width_m,
        line_primary_min_progress_m=args.line_primary_min_progress_m,
        line_primary_segment_timeout_sec=args.line_primary_segment_timeout_sec,
        line_approach_distance_tolerance_m=args.line_approach_distance_tolerance_m,
        line_approach_cross_track_threshold_m=args.line_approach_cross_track_threshold_m,
        line_approach_heading_threshold_rad=args.line_approach_heading_threshold_rad,
        line_approach_min_progress_ratio=args.line_approach_min_progress_ratio,
        competition_qr_reacquire_timeout_sec=args.competition_qr_reacquire_timeout_sec,
    )
    try:
        if not args.execute and not dock_only_target:
            display_actions = node.build_competition_actions()
            print(
                "Competition action plan: "
                + " -> ".join(f"{action['type']}:{action['name']}" for action in display_actions),
                flush=True,
            )
            print("Validation only. Add --execute to move the robot.", flush=True)
            return 0
        return 0 if node.execute() else 1
    except KeyboardInterrupt:
        node.failure_reason = node.failure_reason or "keyboard_interrupt"
        node.stop()
        node.write_outputs(False)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
