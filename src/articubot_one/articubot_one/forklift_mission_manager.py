#!/usr/bin/env python3
"""Deprecated experimental forklift FSM.

The active measured-route execution path is route_executor. This node remains
installed for reference and future FSM work, but it does not consume
mission_routes.yaml waypoint-by-waypoint and is not used by the current
validated scenario command.
"""

import math
import re
import subprocess
import time
from enum import Enum
from pathlib import Path
from typing import Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String
import tf2_ros
from tf2_ros import TransformException

try:
    import yaml
except ImportError:
    yaml = None


class MissionState(Enum):
    IDLE = "IDLE"
    TASK_RECEIVED = "TASK_RECEIVED"
    NAV_TO_PICKUP_PREPOINT = "NAV_TO_PICKUP_PREPOINT"
    QR_PICKUP_CONFIRM = "QR_PICKUP_CONFIRM"
    TURN_180_TO_FORKS = "TURN_180_TO_FORKS"
    LINE_OR_ODOM_APPROACH_PICKUP = "LINE_OR_ODOM_APPROACH_PICKUP"
    FINAL_ALIGN_PICKUP = "FINAL_ALIGN_PICKUP"
    VERIFY_FORK_INSERTION = "VERIFY_FORK_INSERTION"
    PICK_LOAD = "PICK_LOAD"
    VERIFY_PICKUP_SUCCESS = "VERIFY_PICKUP_SUCCESS"
    BACK_OUT_SHORT = "BACK_OUT_SHORT"
    TURN_180_TO_NAV = "TURN_180_TO_NAV"
    NAV_TO_GATE_OUT = "NAV_TO_GATE_OUT"
    NAV_TO_GATE_WAIT = "NAV_TO_GATE_WAIT"
    WAIT_GATE_PERMISSION = "WAIT_GATE_PERMISSION"
    PASS_GATE = "PASS_GATE"
    PASS_GATE_CENTER = "PASS_GATE_CENTER"
    GATE_CLEAR_AFTER = "GATE_CLEAR_AFTER"
    NAV_TO_DROPOFF_PREPOINT = "NAV_TO_DROPOFF_PREPOINT"
    QR_DROPOFF_CONFIRM = "QR_DROPOFF_CONFIRM"
    LINE_FOLLOW_DROPOFF = "LINE_FOLLOW_DROPOFF"
    TURN_180_TO_DROPOFF = "TURN_180_TO_DROPOFF"
    LINE_OR_ODOM_APPROACH_DROPOFF = "LINE_OR_ODOM_APPROACH_DROPOFF"
    FINAL_ALIGN_DROPOFF = "FINAL_ALIGN_DROPOFF"
    DROP_LOAD = "DROP_LOAD"
    VERIFY_DROPOFF_RELEASE = "VERIFY_DROPOFF_RELEASE"
    BACK_OUT_DROPOFF = "BACK_OUT_DROPOFF"
    TURN_180_TO_RETURN = "TURN_180_TO_RETURN"
    NAV_TO_GATE_IN = "NAV_TO_GATE_IN"
    WAIT_GATE_PERMISSION_RETURN = "WAIT_GATE_PERMISSION_RETURN"
    RETURN_TO_START = "RETURN_TO_START"
    TASK_DONE = "TASK_DONE"
    ERROR = "ERROR"
    EMERGENCY_STOP = "EMERGENCY_STOP"


PICKUP_STATIONS = {"A1", "A2", "A3"}
DROPOFF_STATIONS = {"B1", "B2", "B3"}
NAV_STATES = {
    MissionState.NAV_TO_PICKUP_PREPOINT,
    MissionState.NAV_TO_GATE_OUT,
    MissionState.NAV_TO_GATE_WAIT,
    MissionState.PASS_GATE,
    MissionState.NAV_TO_DROPOFF_PREPOINT,
    MissionState.NAV_TO_GATE_IN,
    MissionState.RETURN_TO_START,
}
VERIFIED_LOAD_HANDLING_STATES = {
    MissionState.PICK_LOAD,
    MissionState.VERIFY_PICKUP_SUCCESS,
    MissionState.BACK_OUT_SHORT,
    MissionState.TURN_180_TO_NAV,
}
STATIONARY_PICKUP_SAFETY_BYPASS_STATES = {
    MissionState.VERIFY_FORK_INSERTION,
    MissionState.PICK_LOAD,
    MissionState.VERIFY_PICKUP_SUCCESS,
}
CARRIED_LOAD_SAFETY_BYPASS_STATES = VERIFIED_LOAD_HANDLING_STATES | {
    MissionState.NAV_TO_GATE_WAIT,
    MissionState.WAIT_GATE_PERMISSION,
    MissionState.PASS_GATE_CENTER,
    MissionState.GATE_CLEAR_AFTER,
    MissionState.NAV_TO_DROPOFF_PREPOINT,
    MissionState.QR_DROPOFF_CONFIRM,
    MissionState.LINE_FOLLOW_DROPOFF,
    MissionState.TURN_180_TO_DROPOFF,
    MissionState.LINE_OR_ODOM_APPROACH_DROPOFF,
    MissionState.FINAL_ALIGN_DROPOFF,
    MissionState.DROP_LOAD,
    MissionState.VERIFY_DROPOFF_RELEASE,
    MissionState.BACK_OUT_DROPOFF,
    MissionState.TURN_180_TO_RETURN,
}
GATE_CROSSING_STATES = {
    MissionState.PASS_GATE_CENTER,
    MissionState.GATE_CLEAR_AFTER,
}
PICKUP_PREPOINTS = {
    "A1": "A1_QR_APPROACH",
    "A2": "A2_QR_APPROACH",
    "A3": "A3_QR_APPROACH",
}
DROPOFF_PREPOINTS = {
    "B1": "drop_1",
    "B2": "drop_2",
    "B3": "drop_3",
}
RETURN_ROUTE_POINTS = [
    "RETURN_GATE_CLEAR",
    "RETURN_MAIN_ROUTE",
    "START",
]


def yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    half_yaw = yaw * 0.5
    return math.sin(half_yaw), math.cos(half_yaw)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def mission_points_path() -> Path:
    return Path(get_package_share_directory("articubot_one")) / "config" / "mission_points.yaml"


def load_points(path: Path) -> Dict[str, Dict[str, float]]:
    text = path.read_text(encoding="utf-8")
    if yaml is None:
        loaded = _parse_simple_yaml_points(text)
    else:
        loaded = yaml.safe_load(text) or {}

    if isinstance(loaded, dict) and "points" in loaded and isinstance(loaded["points"], dict):
        loaded = loaded["points"]

    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a point mapping")

    points: Dict[str, Dict[str, float]] = {}
    for name, value in loaded.items():
        if not isinstance(value, dict):
            continue
        x_value = value.get("x")
        y_value = value.get("y")
        yaw_value = value.get("yaw", 0.0)
        z_value = value.get("z", 0.0)
        if x_value is None or y_value is None:
            continue
        points[str(name)] = {
            "x": float(x_value),
            "y": float(y_value),
            "z": float(z_value),
            "yaw": float(yaw_value),
        }

    aliases = {
        "START": "start",
        "GATE_OUT": "gate",
        "A1": "pickup_1",
        "A2": "pickup_2",
        "A3": "pickup_3",
        "B1": "drop_1",
        "B2": "drop_2",
        "B3": "drop_3",
    }
    for alias, existing in aliases.items():
        if alias not in points and existing in points:
            points[alias] = dict(points[existing])

    return points


def _parse_simple_yaml_points(text: str) -> Dict[str, Dict[str, float]]:
    points: Dict[str, Dict[str, float]] = {}
    current_name: Optional[str] = None
    in_points = False

    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if line.strip() == "points:":
            in_points = True
            continue
        if not in_points:
            continue
        if line.startswith("  ") and not line.startswith("    ") and line.endswith(":"):
            current_name = line.strip()[:-1]
            points[current_name] = {}
            continue
        if current_name and ":" in line:
            key, value = line.strip().split(":", 1)
            points[current_name][key.strip()] = float(value.strip())

    return points


class ForkliftMissionManager(Node):
    def __init__(self) -> None:
        super().__init__("forklift_mission_manager")

        self.declare_parameter("pickup", "A1")
        self.declare_parameter("dropoff", "B1")
        self.declare_parameter("autostart", False)
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_docking")
        self.declare_parameter("qr_topic", "/qr_code")
        self.declare_parameter("line_offset_topic", "/line_offset")
        self.declare_parameter("line_angle_topic", "/line_angle")
        self.declare_parameter("fork_cmd_topic", "/fork_cmd")
        self.declare_parameter("fork_state_topic", "/fork_state")
        self.declare_parameter("pickup_success_topic", "/pickup_success")
        self.declare_parameter("mission_command_topic", "/forklift_mission/command")
        self.declare_parameter("state_topic", "/forklift_mission/state")
        self.declare_parameter("status_topic", "/forklift_mission/status")
        self.declare_parameter("safety_stop_topic", "/forklift_safety/stop")
        self.declare_parameter("odom_topic", "/diff_cont/odom")
        self.declare_parameter("qr_timeout_sec", 8.0)
        self.declare_parameter("line_timeout_sec", 2.0)
        self.declare_parameter("state_timeout_sec", 45.0)
        self.declare_parameter("nav_timeout_sec", 180.0)
        self.declare_parameter("gate_wait_sec", 1.0)
        self.declare_parameter("gate_wait_point_out", "GATE_OUT")
        self.declare_parameter("gate_wait_point_return", "GATE_IN")
        self.declare_parameter("gate_clear_point_out", "GATE_IN")
        self.declare_parameter("gate_clear_point_return", "RETURN_GATE_CLEAR")
        self.declare_parameter("gate_drive_speed", 0.12)
        self.declare_parameter("gate_drive_min_speed", 0.05)
        self.declare_parameter("gate_drive_slow_radius", 0.50)
        self.declare_parameter("gate_drive_goal_tolerance", 0.12)
        self.declare_parameter("gate_drive_lateral_tolerance", 0.30)
        self.declare_parameter("loaded_gate_lateral_tolerance", 0.60)
        self.declare_parameter("gate_drive_lateral_gain", 0.9)
        self.declare_parameter("gate_drive_yaw_gain", 0.35)
        self.declare_parameter("gate_drive_max_angular", 0.18)
        self.declare_parameter("loaded_gate_lane_x_offset", -0.15)
        self.declare_parameter("loaded_gate_pass_lane_x_offset", -0.20)
        self.declare_parameter("loaded_gate_pass_y_offset", 0.75)
        self.declare_parameter("loaded_gate_wait_forward_tolerance", 1.30)
        self.declare_parameter("loaded_gate_wait_lateral_tolerance", 0.95)
        self.declare_parameter("loaded_gate_entry_yaw_tolerance", 0.14)
        self.declare_parameter("loaded_gate_heading_gain", 0.28)
        self.declare_parameter("loaded_gate_yaw_gain", 0.38)
        self.declare_parameter("loaded_gate_max_angular", 0.09)
        self.declare_parameter("loaded_gate_pass_heading_gain", 1.10)
        self.declare_parameter("loaded_gate_pass_yaw_gain", 0.80)
        self.declare_parameter("loaded_gate_pass_max_angular", 0.16)
        self.declare_parameter("loaded_gate_pass_goal_tolerance", 0.35)
        self.declare_parameter("loaded_route_drive_speed", 0.12)
        self.declare_parameter("loaded_route_heading_gain", 0.45)
        self.declare_parameter("loaded_route_yaw_gain", 0.18)
        self.declare_parameter("loaded_route_max_angular", 0.09)
        self.declare_parameter("loaded_route_heading_deadband", 0.035)
        self.declare_parameter("loaded_route_yaw_deadband", 0.04)
        self.declare_parameter("loaded_route_smoothing_alpha", 0.30)
        self.declare_parameter("loaded_route_max_angular_step", 0.025)
        self.declare_parameter("loaded_route_turn_angular_speed", 0.28)
        self.declare_parameter("fork_action_sec", 3.0)
        self.declare_parameter("line_follow_distance", 1.15)
        self.declare_parameter("line_follow_speed", 0.08)
        self.declare_parameter("pickup_qr_require_line", True)
        self.declare_parameter("pickup_qr_line_offset_tolerance", 0.25)
        self.declare_parameter("pickup_qr_line_angle_tolerance", 0.12)
        self.declare_parameter("pickup_qr_align_max_angular", 0.18)
        self.declare_parameter("pickup_qr_yaw_tolerance", 0.015)
        self.declare_parameter("pickup_qr_yaw_gain", 1.0)
        self.declare_parameter("pickup_blind_approach_min_distance", 1.00)
        self.declare_parameter("pickup_blind_approach_max_distance", 1.50)
        self.declare_parameter("pickup_blind_approach_speed", 0.085)
        self.declare_parameter("pickup_blind_heading_gain", 0.80)
        self.declare_parameter("pickup_blind_max_angular", 0.08)
        self.declare_parameter("stop_after_pickup_test", True)
        self.declare_parameter("final_align_distance", 0.35)
        self.declare_parameter("final_align_speed", 0.035)
        self.declare_parameter("final_line_offset_tolerance", 0.25)
        self.declare_parameter("final_line_angle_tolerance", 0.60)
        self.declare_parameter("dropoff_final_centered_offset_tolerance", 0.06)
        self.declare_parameter("back_out_distance", 0.35)
        self.declare_parameter("back_out_speed", 0.08)
        self.declare_parameter("dropoff_final_start_forward", 0.45)
        self.declare_parameter("dropoff_verify_forward_tolerance", 0.12)
        self.declare_parameter("dropoff_verify_lateral_tolerance", 0.16)
        self.declare_parameter("dropoff_verify_yaw_tolerance", 0.25)
        self.declare_parameter("dropoff_align_extra_distance", 0.60)
        self.declare_parameter("dropoff_release_xy_tolerance", 0.35)
        self.declare_parameter("dropoff_release_min_z_drop", 0.025)
        self.declare_parameter("dropoff_release_max_z", 0.120)
        self.declare_parameter("dropoff_verify_timeout_sec", 5.0)
        self.declare_parameter("fork_lower_success_height", 0.05)
        self.declare_parameter("dropoff_back_out_distance", 0.35)
        self.declare_parameter("dropoff_back_out_speed", 0.08)
        self.declare_parameter("turn_angular_speed", 0.45)
        self.declare_parameter("turn_tolerance", 0.015)
        self.declare_parameter("turn_min_angular_speed", 0.08)
        self.declare_parameter("turn_slowdown_gain", 1.4)
        self.declare_parameter("line_offset_tolerance", 0.85)
        self.declare_parameter("line_angle_tolerance", 2.0)
        self.declare_parameter("line_acquire_offset_tolerance", 1.15)
        self.declare_parameter("line_acquire_speed", 0.04)
        self.declare_parameter("angular_offset_gain", 0.8)
        self.declare_parameter("angular_angle_gain", 0.35)
        self.declare_parameter("line_offset_deadband", 0.025)
        self.declare_parameter("line_heading_deadband", 0.04)
        self.declare_parameter("line_angular_smoothing_alpha", 0.35)
        self.declare_parameter("line_max_angular_step", 0.05)
        self.declare_parameter("pass_gate_point", "GATE_PASS_OUT")
        self.declare_parameter("pass_gate_nav_retries", 1)
        self.declare_parameter("pickup_verify_forward_min", 0.45)
        self.declare_parameter("pickup_verify_forward_max", 0.85)
        self.declare_parameter("pickup_verify_lateral_tolerance", 0.10)
        self.declare_parameter("pickup_verify_yaw_tolerance", 0.20)
        self.declare_parameter("pickup_verify_line_offset_tolerance", 0.18)
        self.declare_parameter("pickup_verify_line_angle_tolerance", 0.45)
        self.declare_parameter("pickup_verify_timeout_sec", 5.0)
        self.declare_parameter("pickup_insertion_hold_sec", 0.5)
        self.declare_parameter("load_frame_parent", "world")
        self.declare_parameter("load_frame_template", "pallet_load_{station}")
        self.declare_parameter("load_pose_max_age_sec", 0.5)
        self.declare_parameter("gazebo_pose_topic", "/world/teknofest_arena/pose/info")
        self.declare_parameter("gazebo_pose_timeout_sec", 1.0)
        self.declare_parameter("gazebo_world_origin_x", -5.80)
        self.declare_parameter("gazebo_world_origin_y", -3.05)
        self.declare_parameter("gazebo_world_origin_yaw", 1.5708)
        self.declare_parameter("robot_model_frame", "robot")
        self.declare_parameter("load_lift_min_delta", 0.03)
        self.declare_parameter("load_follow_min_delta", 0.05)
        self.declare_parameter("load_follow_robot_ratio", 0.35)
        self.declare_parameter("load_follow_extra_backout_distance", 0.20)
        self.declare_parameter("load_follow_verify_period", 0.5)
        self.declare_parameter("require_load_follow_verification", True)
        self.declare_parameter("enable_carried_load_pose_lock", True)
        self.declare_parameter("carried_load_pose_update_period", 0.5)
        self.declare_parameter("loaded_route_pose_sync_enabled", True)
        self.declare_parameter("robot_pose_sync_period", 5.0)
        self.declare_parameter(
            "gazebo_set_pose_service",
            "/world/teknofest_arena/set_pose",
        )
        self.declare_parameter("fork_lift_success_height", 0.20)
        self.declare_parameter("pickup_align_extra_distance", 0.45)
        self.declare_parameter("pickup_align_lateral_gain", 1.4)
        self.declare_parameter("pickup_align_yaw_gain", 0.8)
        self.declare_parameter("pickup_align_line_gain", 0.2)
        self.declare_parameter("pickup_align_max_angular", 0.25)
        self.declare_parameter("pickup_align_lateral_deadband", 0.015)
        self.declare_parameter("pickup_align_yaw_deadband", 0.025)
        self.declare_parameter("pickup_align_angular_deadband", 0.015)
        self.declare_parameter("pickup_align_smoothing_alpha", 0.35)
        self.declare_parameter("pickup_align_max_angular_step", 0.04)
        self.declare_parameter("back_out_accel_step", 0.02)
        self.declare_parameter("back_out_yaw_gain", 0.9)
        self.declare_parameter("back_out_max_angular", 0.12)

        self.pickup = str(self.get_parameter("pickup").value).upper()
        self.dropoff = str(self.get_parameter("dropoff").value).upper()
        self.autostart = bool(self.get_parameter("autostart").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.qr_timeout_sec = float(self.get_parameter("qr_timeout_sec").value)
        self.line_timeout_sec = float(self.get_parameter("line_timeout_sec").value)
        self.state_timeout_sec = float(self.get_parameter("state_timeout_sec").value)
        self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)
        self.gate_wait_sec = float(self.get_parameter("gate_wait_sec").value)
        self.gate_wait_point_out = str(
            self.get_parameter("gate_wait_point_out").value
        ).upper()
        self.gate_wait_point_return = str(
            self.get_parameter("gate_wait_point_return").value
        ).upper()
        self.gate_clear_point_out = str(
            self.get_parameter("gate_clear_point_out").value
        ).upper()
        self.gate_clear_point_return = str(
            self.get_parameter("gate_clear_point_return").value
        ).upper()
        self.gate_drive_speed = float(self.get_parameter("gate_drive_speed").value)
        self.gate_drive_min_speed = float(
            self.get_parameter("gate_drive_min_speed").value
        )
        self.gate_drive_slow_radius = float(
            self.get_parameter("gate_drive_slow_radius").value
        )
        self.gate_drive_goal_tolerance = float(
            self.get_parameter("gate_drive_goal_tolerance").value
        )
        self.gate_drive_lateral_tolerance = float(
            self.get_parameter("gate_drive_lateral_tolerance").value
        )
        self.loaded_gate_lateral_tolerance = float(
            self.get_parameter("loaded_gate_lateral_tolerance").value
        )
        self.gate_drive_lateral_gain = float(
            self.get_parameter("gate_drive_lateral_gain").value
        )
        self.gate_drive_yaw_gain = float(self.get_parameter("gate_drive_yaw_gain").value)
        self.gate_drive_max_angular = float(
            self.get_parameter("gate_drive_max_angular").value
        )
        self.loaded_gate_lane_x_offset = float(
            self.get_parameter("loaded_gate_lane_x_offset").value
        )
        self.loaded_gate_pass_lane_x_offset = float(
            self.get_parameter("loaded_gate_pass_lane_x_offset").value
        )
        self.loaded_gate_pass_y_offset = float(
            self.get_parameter("loaded_gate_pass_y_offset").value
        )
        self.loaded_gate_wait_forward_tolerance = float(
            self.get_parameter("loaded_gate_wait_forward_tolerance").value
        )
        self.loaded_gate_wait_lateral_tolerance = float(
            self.get_parameter("loaded_gate_wait_lateral_tolerance").value
        )
        self.loaded_gate_entry_yaw_tolerance = float(
            self.get_parameter("loaded_gate_entry_yaw_tolerance").value
        )
        self.loaded_gate_heading_gain = float(
            self.get_parameter("loaded_gate_heading_gain").value
        )
        self.loaded_gate_yaw_gain = float(
            self.get_parameter("loaded_gate_yaw_gain").value
        )
        self.loaded_gate_max_angular = float(
            self.get_parameter("loaded_gate_max_angular").value
        )
        self.loaded_gate_pass_heading_gain = float(
            self.get_parameter("loaded_gate_pass_heading_gain").value
        )
        self.loaded_gate_pass_yaw_gain = float(
            self.get_parameter("loaded_gate_pass_yaw_gain").value
        )
        self.loaded_gate_pass_max_angular = float(
            self.get_parameter("loaded_gate_pass_max_angular").value
        )
        self.loaded_gate_pass_goal_tolerance = float(
            self.get_parameter("loaded_gate_pass_goal_tolerance").value
        )
        self.loaded_route_drive_speed = float(
            self.get_parameter("loaded_route_drive_speed").value
        )
        self.loaded_route_heading_gain = float(
            self.get_parameter("loaded_route_heading_gain").value
        )
        self.loaded_route_yaw_gain = float(
            self.get_parameter("loaded_route_yaw_gain").value
        )
        self.loaded_route_max_angular = float(
            self.get_parameter("loaded_route_max_angular").value
        )
        self.loaded_route_heading_deadband = float(
            self.get_parameter("loaded_route_heading_deadband").value
        )
        self.loaded_route_yaw_deadband = float(
            self.get_parameter("loaded_route_yaw_deadband").value
        )
        self.loaded_route_smoothing_alpha = float(
            self.get_parameter("loaded_route_smoothing_alpha").value
        )
        self.loaded_route_max_angular_step = float(
            self.get_parameter("loaded_route_max_angular_step").value
        )
        self.loaded_route_turn_angular_speed = float(
            self.get_parameter("loaded_route_turn_angular_speed").value
        )
        self.fork_action_sec = float(self.get_parameter("fork_action_sec").value)
        self.line_follow_distance = float(self.get_parameter("line_follow_distance").value)
        self.line_follow_speed = float(self.get_parameter("line_follow_speed").value)
        self.pickup_qr_require_line = bool(
            self.get_parameter("pickup_qr_require_line").value
        )
        self.pickup_qr_line_offset_tolerance = float(
            self.get_parameter("pickup_qr_line_offset_tolerance").value
        )
        self.pickup_qr_line_angle_tolerance = float(
            self.get_parameter("pickup_qr_line_angle_tolerance").value
        )
        self.pickup_qr_align_max_angular = float(
            self.get_parameter("pickup_qr_align_max_angular").value
        )
        self.pickup_qr_yaw_tolerance = float(
            self.get_parameter("pickup_qr_yaw_tolerance").value
        )
        self.pickup_qr_yaw_gain = float(
            self.get_parameter("pickup_qr_yaw_gain").value
        )
        self.pickup_blind_approach_min_distance = float(
            self.get_parameter("pickup_blind_approach_min_distance").value
        )
        self.pickup_blind_approach_max_distance = float(
            self.get_parameter("pickup_blind_approach_max_distance").value
        )
        self.pickup_blind_approach_speed = float(
            self.get_parameter("pickup_blind_approach_speed").value
        )
        self.pickup_blind_heading_gain = float(
            self.get_parameter("pickup_blind_heading_gain").value
        )
        self.pickup_blind_max_angular = float(
            self.get_parameter("pickup_blind_max_angular").value
        )
        self.stop_after_pickup_test = bool(
            self.get_parameter("stop_after_pickup_test").value
        )
        self.final_align_distance = float(self.get_parameter("final_align_distance").value)
        self.final_align_speed = float(self.get_parameter("final_align_speed").value)
        self.final_line_offset_tolerance = float(
            self.get_parameter("final_line_offset_tolerance").value
        )
        self.final_line_angle_tolerance = float(
            self.get_parameter("final_line_angle_tolerance").value
        )
        self.dropoff_final_centered_offset_tolerance = float(
            self.get_parameter("dropoff_final_centered_offset_tolerance").value
        )
        self.back_out_distance = float(self.get_parameter("back_out_distance").value)
        self.back_out_speed = float(self.get_parameter("back_out_speed").value)
        self.dropoff_final_start_forward = float(
            self.get_parameter("dropoff_final_start_forward").value
        )
        self.dropoff_verify_forward_tolerance = float(
            self.get_parameter("dropoff_verify_forward_tolerance").value
        )
        self.dropoff_verify_lateral_tolerance = float(
            self.get_parameter("dropoff_verify_lateral_tolerance").value
        )
        self.dropoff_verify_yaw_tolerance = float(
            self.get_parameter("dropoff_verify_yaw_tolerance").value
        )
        self.dropoff_align_extra_distance = float(
            self.get_parameter("dropoff_align_extra_distance").value
        )
        self.dropoff_release_xy_tolerance = float(
            self.get_parameter("dropoff_release_xy_tolerance").value
        )
        self.dropoff_release_min_z_drop = float(
            self.get_parameter("dropoff_release_min_z_drop").value
        )
        self.dropoff_release_max_z = float(
            self.get_parameter("dropoff_release_max_z").value
        )
        self.dropoff_verify_timeout_sec = float(
            self.get_parameter("dropoff_verify_timeout_sec").value
        )
        self.fork_lower_success_height = float(
            self.get_parameter("fork_lower_success_height").value
        )
        self.dropoff_back_out_distance = float(
            self.get_parameter("dropoff_back_out_distance").value
        )
        self.dropoff_back_out_speed = float(
            self.get_parameter("dropoff_back_out_speed").value
        )
        self.turn_angular_speed = float(self.get_parameter("turn_angular_speed").value)
        self.turn_tolerance = float(self.get_parameter("turn_tolerance").value)
        self.turn_min_angular_speed = float(
            self.get_parameter("turn_min_angular_speed").value
        )
        self.turn_slowdown_gain = float(
            self.get_parameter("turn_slowdown_gain").value
        )
        self.line_offset_tolerance = float(self.get_parameter("line_offset_tolerance").value)
        self.line_angle_tolerance = float(self.get_parameter("line_angle_tolerance").value)
        self.line_acquire_offset_tolerance = float(
            self.get_parameter("line_acquire_offset_tolerance").value
        )
        self.line_acquire_speed = float(self.get_parameter("line_acquire_speed").value)
        self.angular_offset_gain = float(self.get_parameter("angular_offset_gain").value)
        self.angular_angle_gain = float(self.get_parameter("angular_angle_gain").value)
        self.line_offset_deadband = float(
            self.get_parameter("line_offset_deadband").value
        )
        self.line_heading_deadband = float(
            self.get_parameter("line_heading_deadband").value
        )
        self.line_angular_smoothing_alpha = float(
            self.get_parameter("line_angular_smoothing_alpha").value
        )
        self.line_max_angular_step = float(
            self.get_parameter("line_max_angular_step").value
        )
        self.pass_gate_point = str(self.get_parameter("pass_gate_point").value).upper()
        self.pass_gate_nav_retries = int(self.get_parameter("pass_gate_nav_retries").value)
        self.pickup_verify_forward_min = float(
            self.get_parameter("pickup_verify_forward_min").value
        )
        self.pickup_verify_forward_max = float(
            self.get_parameter("pickup_verify_forward_max").value
        )
        self.pickup_verify_lateral_tolerance = float(
            self.get_parameter("pickup_verify_lateral_tolerance").value
        )
        self.pickup_verify_yaw_tolerance = float(
            self.get_parameter("pickup_verify_yaw_tolerance").value
        )
        self.pickup_verify_line_offset_tolerance = float(
            self.get_parameter("pickup_verify_line_offset_tolerance").value
        )
        self.pickup_verify_line_angle_tolerance = float(
            self.get_parameter("pickup_verify_line_angle_tolerance").value
        )
        self.pickup_verify_timeout_sec = float(
            self.get_parameter("pickup_verify_timeout_sec").value
        )
        self.pickup_insertion_hold_sec = float(
            self.get_parameter("pickup_insertion_hold_sec").value
        )
        self.load_frame_parent = str(self.get_parameter("load_frame_parent").value)
        self.load_frame_template = str(self.get_parameter("load_frame_template").value)
        self.load_pose_max_age_sec = float(
            self.get_parameter("load_pose_max_age_sec").value
        )
        self.gazebo_pose_topic = str(self.get_parameter("gazebo_pose_topic").value)
        self.gazebo_pose_timeout_sec = float(
            self.get_parameter("gazebo_pose_timeout_sec").value
        )
        self.gazebo_world_origin_x = float(
            self.get_parameter("gazebo_world_origin_x").value
        )
        self.gazebo_world_origin_y = float(
            self.get_parameter("gazebo_world_origin_y").value
        )
        self.gazebo_world_origin_yaw = float(
            self.get_parameter("gazebo_world_origin_yaw").value
        )
        self.robot_model_frame = str(self.get_parameter("robot_model_frame").value)
        self.load_lift_min_delta = float(
            self.get_parameter("load_lift_min_delta").value
        )
        self.load_follow_min_delta = float(
            self.get_parameter("load_follow_min_delta").value
        )
        self.load_follow_robot_ratio = float(
            self.get_parameter("load_follow_robot_ratio").value
        )
        self.load_follow_extra_backout_distance = float(
            self.get_parameter("load_follow_extra_backout_distance").value
        )
        self.load_follow_verify_period = float(
            self.get_parameter("load_follow_verify_period").value
        )
        self.require_load_follow_verification = bool(
            self.get_parameter("require_load_follow_verification").value
        )
        self.enable_carried_load_pose_lock = bool(
            self.get_parameter("enable_carried_load_pose_lock").value
        )
        self.carried_load_pose_update_period = float(
            self.get_parameter("carried_load_pose_update_period").value
        )
        self.loaded_route_pose_sync_enabled = bool(
            self.get_parameter("loaded_route_pose_sync_enabled").value
        )
        self.robot_pose_sync_period = float(
            self.get_parameter("robot_pose_sync_period").value
        )
        self.gazebo_set_pose_service = str(
            self.get_parameter("gazebo_set_pose_service").value
        )
        self.fork_lift_success_height = float(
            self.get_parameter("fork_lift_success_height").value
        )
        self.pickup_align_extra_distance = float(
            self.get_parameter("pickup_align_extra_distance").value
        )
        self.pickup_align_lateral_gain = float(
            self.get_parameter("pickup_align_lateral_gain").value
        )
        self.pickup_align_yaw_gain = float(self.get_parameter("pickup_align_yaw_gain").value)
        self.pickup_align_line_gain = float(self.get_parameter("pickup_align_line_gain").value)
        self.pickup_align_max_angular = float(
            self.get_parameter("pickup_align_max_angular").value
        )
        self.pickup_align_lateral_deadband = float(
            self.get_parameter("pickup_align_lateral_deadband").value
        )
        self.pickup_align_yaw_deadband = float(
            self.get_parameter("pickup_align_yaw_deadband").value
        )
        self.pickup_align_angular_deadband = float(
            self.get_parameter("pickup_align_angular_deadband").value
        )
        self.pickup_align_smoothing_alpha = float(
            self.get_parameter("pickup_align_smoothing_alpha").value
        )
        self.pickup_align_max_angular_step = float(
            self.get_parameter("pickup_align_max_angular_step").value
        )
        self.back_out_accel_step = float(self.get_parameter("back_out_accel_step").value)
        self.back_out_yaw_gain = float(self.get_parameter("back_out_yaw_gain").value)
        self.back_out_max_angular = float(
            self.get_parameter("back_out_max_angular").value
        )

        self.points = load_points(mission_points_path())

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.fork_cmd_pub = self.create_publisher(
            String,
            str(self.get_parameter("fork_cmd_topic").value),
            10,
        )
        self.pickup_success_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("pickup_success_topic").value),
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )

        self.create_subscription(String, str(self.get_parameter("qr_topic").value), self.qr_cb, 10)
        self.create_subscription(
            Float32,
            str(self.get_parameter("line_offset_topic").value),
            self.line_offset_cb,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("line_angle_topic").value),
            self.line_angle_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_command_topic").value),
            self.command_cb,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("safety_stop_topic").value),
            self.safety_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("fork_state_topic").value),
            self.fork_state_cb,
            10,
        )

        self.state = MissionState.IDLE
        self.state_enter_time = self.now_sec()
        self.active_goal = None
        self.nav_goal_pending = False
        self.nav_result_status: Optional[int] = None
        self.nav_target = ""
        self.nav_retry_counts: Dict[str, int] = {}
        self.return_route_index = 0
        self.last_qr = ""
        self.last_qr_time = -1.0
        self.qr_seen_times: Dict[str, float] = {}
        self.line_offset = math.nan
        self.line_angle = math.nan
        self.last_line_time = -1.0
        self.safety_stop = False
        self.error_reason = ""
        self.fork_command_sent = False
        self.pickup_verified = False
        self.carrying_load = False
        self.fork_state_text = ""
        self.fork_mode = ""
        self.fork_height = math.nan
        self.last_pickup_verify_detail = ""
        self.pickup_insertion_start_time: Optional[float] = None
        self.load_pose_before_lift: Optional[Tuple[float, float, float]] = None
        self.load_pose_after_lift: Optional[Tuple[float, float, float]] = None
        self.robot_pose_after_lift: Optional[Tuple[float, float, float]] = None
        self.drop_load_pose_before_lower: Optional[Tuple[float, float, float]] = None
        self.drop_load_pose_after_release: Optional[Tuple[float, float, float]] = None
        self.carried_load_offset: Optional[Tuple[float, float, float]] = None
        self.last_carried_load_pose_update = -1.0
        self.load_pose_before_lift_time = -1.0
        self.last_gazebo_pose: Optional[Tuple[str, Tuple[float, float, float], float]] = None
        self.last_gazebo_pose_with_yaw: Optional[
            Tuple[str, Tuple[float, float, float, float], float]
        ] = None
        self.last_robot_pose_sync_wall = -1.0
        self.last_load_follow_verify_wall = -1.0
        self.carried_load_pose_process: Optional[subprocess.Popen] = None
        self.motion_start_xy: Optional[Tuple[float, float]] = None
        self.motion_start_yaw: Optional[float] = None
        self.last_line_angular_cmd = 0.0
        self.last_pickup_angular_cmd = 0.0
        self.last_loaded_route_angular_cmd = 0.0
        self.back_out_speed_cmd = 0.0
        self.loaded_dropoff_route_index = 0
        self.qr_scan_direction = 1.0
        self.raw_current_x = 0.0
        self.raw_current_y = 0.0
        self.raw_current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False
        self.have_raw_odom = False
        self.odom_pose_correction_active = False
        self.odom_pose_correction_x = 0.0
        self.odom_pose_correction_y = 0.0
        self.odom_pose_correction_yaw = 0.0

        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_state)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"Mission manager ready. pickup={self.pickup}, dropoff={self.dropoff}, "
            f"autostart={self.autostart}"
        )
        if self.autostart:
            self.start_mission(self.pickup, self.dropoff)

    def command_cb(self, msg: String) -> None:
        text = msg.data.strip().upper()
        if text == "STOP":
            self.cancel_nav_goal()
            self.stop_robot()
            self.transition(MissionState.IDLE, "STOP command")
            return

        match = re.search(r"(A[123]).*(B[123])", text)
        if match:
            self.start_mission(match.group(1), match.group(2))
            return

        if text == "START":
            self.start_mission(self.pickup, self.dropoff)
            return

        self.get_logger().warn(
            "Mission command ignored. Use 'START', 'STOP', or for example 'START A2 B3'."
        )

    def start_mission(self, pickup: str, dropoff: str) -> None:
        pickup = pickup.upper()
        dropoff = dropoff.upper()
        if pickup not in PICKUP_STATIONS or dropoff not in DROPOFF_STATIONS:
            self.fail(f"invalid mission pickup={pickup} dropoff={dropoff}")
            return

        pickup_prepoint = self.pickup_prepoint(pickup)
        dropoff_prepoint = self.dropoff_prepoint(dropoff)
        pickup_target = self.pickup_target_point(pickup)
        dropoff_target = self.dropoff_target_point(dropoff)
        required = [
            "START",
            pickup_prepoint,
            pickup,
            pickup_target,
            self.gate_wait_point_out,
            self.pass_gate_point,
            self.gate_clear_point_out,
            dropoff_prepoint,
            dropoff,
            dropoff_target,
            self.gate_wait_point_return,
            self.gate_clear_point_return,
            *RETURN_ROUTE_POINTS,
        ]
        missing = [name for name in required if name not in self.points]
        if missing:
            self.fail(f"missing mission points: {', '.join(missing)}")
            return

        self.cancel_nav_goal()
        self.pickup = pickup
        self.dropoff = dropoff
        self.error_reason = ""
        self.last_qr = ""
        self.last_qr_time = -1.0
        self.qr_seen_times = {}
        self.nav_retry_counts = {}
        self.return_route_index = 0
        self.fork_command_sent = False
        self.pickup_verified = False
        self.carrying_load = False
        self.load_pose_before_lift = None
        self.load_pose_after_lift = None
        self.robot_pose_after_lift = None
        self.drop_load_pose_before_lower = None
        self.drop_load_pose_after_release = None
        self.carried_load_offset = None
        self.last_carried_load_pose_update = -1.0
        self.carried_load_pose_process = None
        self.load_pose_before_lift_time = -1.0
        self.last_gazebo_pose = None
        self.last_gazebo_pose_with_yaw = None
        self.last_robot_pose_sync_wall = -1.0
        self.last_load_follow_verify_wall = -1.0
        self.odom_pose_correction_active = False
        self.odom_pose_correction_x = 0.0
        self.odom_pose_correction_y = 0.0
        self.odom_pose_correction_yaw = 0.0
        self.publish_pickup_success(False)
        self.motion_start_xy = None
        self.motion_start_yaw = None
        self.loaded_dropoff_route_index = 0
        self.transition(MissionState.TASK_RECEIVED, f"task {pickup}->{dropoff}")
        self.transition(
            MissionState.NAV_TO_PICKUP_PREPOINT,
            "dispatch Nav2 pickup prepoint goal",
        )

    def tick(self) -> None:
        if self.state in (
            MissionState.IDLE,
            MissionState.TASK_DONE,
            MissionState.ERROR,
            MissionState.EMERGENCY_STOP,
        ):
            return

        safety_bypassed = (
            self.state in GATE_CROSSING_STATES
            or self.state in STATIONARY_PICKUP_SAFETY_BYPASS_STATES
        ) or (
            self.carrying_load and self.state in CARRIED_LOAD_SAFETY_BYPASS_STATES
        )
        if self.safety_stop and not safety_bypassed:
            self.stop_robot()
            self.publish_status("Safety stop active; holding mission state.")
            return

        elapsed = self.now_sec() - self.state_enter_time
        timeout = (
            self.nav_timeout_sec
            if self.state in NAV_STATES or self.state in GATE_CROSSING_STATES
            else self.state_timeout_sec
        )
        if elapsed > timeout:
            self.fail(f"state timeout in {self.state.value}")
            return

        if self.state == MissionState.TASK_RECEIVED:
            self.transition(
                MissionState.NAV_TO_PICKUP_PREPOINT,
                "dispatch Nav2 pickup prepoint goal",
            )
        elif self.state == MissionState.NAV_TO_PICKUP_PREPOINT:
            self.handle_nav_state(self.pickup_prepoint(self.pickup), MissionState.QR_PICKUP_CONFIRM)
        elif self.state == MissionState.QR_PICKUP_CONFIRM:
            self.handle_qr_confirm(self.pickup, MissionState.TURN_180_TO_FORKS)
        elif self.state == MissionState.TURN_180_TO_FORKS:
            self.handle_turn_180(MissionState.LINE_OR_ODOM_APPROACH_PICKUP)
        elif self.state == MissionState.LINE_OR_ODOM_APPROACH_PICKUP:
            self.handle_odom_approach_pickup()
        elif self.state == MissionState.FINAL_ALIGN_PICKUP:
            self.handle_final_align_pickup()
        elif self.state == MissionState.VERIFY_FORK_INSERTION:
            self.handle_verify_fork_insertion()
        elif self.state == MissionState.PICK_LOAD:
            self.handle_pick_load()
        elif self.state == MissionState.VERIFY_PICKUP_SUCCESS:
            self.handle_verify_pickup_success()
        elif self.state == MissionState.BACK_OUT_SHORT:
            self.handle_back_out()
        elif self.state == MissionState.TURN_180_TO_NAV:
            next_state = (
                MissionState.TASK_DONE
                if self.stop_after_pickup_test
                else MissionState.NAV_TO_GATE_WAIT
            )
            self.handle_turn_180(next_state)
        elif self.state in (MissionState.NAV_TO_GATE_OUT, MissionState.NAV_TO_GATE_WAIT):
            self.handle_gate_drive(
                self.gate_wait_point(),
                MissionState.WAIT_GATE_PERMISSION,
                "gate wait point reached",
            )
        elif self.state == MissionState.WAIT_GATE_PERMISSION:
            self.handle_gate_wait(MissionState.PASS_GATE_CENTER, "PLC permission granted")
        elif self.state in (MissionState.PASS_GATE, MissionState.PASS_GATE_CENTER):
            self.handle_gate_drive(
                self.pass_gate_point,
                MissionState.GATE_CLEAR_AFTER,
                "gate center reached",
            )
        elif self.state == MissionState.GATE_CLEAR_AFTER:
            self.handle_gate_drive(
                self.gate_clear_point(),
                self.after_gate_clear_state(),
                "gate clear after crossing",
            )
        elif self.state == MissionState.NAV_TO_DROPOFF_PREPOINT:
            if self.carrying_load:
                self.handle_loaded_dropoff_prepoint_drive()
            else:
                self.handle_nav_state(
                    self.dropoff_prepoint(self.dropoff),
                    MissionState.QR_DROPOFF_CONFIRM,
                )
        elif self.state == MissionState.QR_DROPOFF_CONFIRM:
            self.handle_qr_confirm(self.dropoff, MissionState.TURN_180_TO_DROPOFF)
        elif self.state == MissionState.TURN_180_TO_DROPOFF:
            self.handle_turn_180(MissionState.LINE_OR_ODOM_APPROACH_DROPOFF)
        elif self.state == MissionState.LINE_OR_ODOM_APPROACH_DROPOFF:
            self.handle_odom_approach_dropoff()
        elif self.state == MissionState.LINE_FOLLOW_DROPOFF:
            self.handle_line_follow(MissionState.FINAL_ALIGN_DROPOFF)
        elif self.state == MissionState.FINAL_ALIGN_DROPOFF:
            self.handle_final_align_dropoff()
        elif self.state == MissionState.DROP_LOAD:
            self.handle_drop_load()
        elif self.state == MissionState.VERIFY_DROPOFF_RELEASE:
            self.handle_verify_dropoff_release()
        elif self.state == MissionState.BACK_OUT_DROPOFF:
            self.handle_dropoff_back_out()
        elif self.state == MissionState.TURN_180_TO_RETURN:
            self.handle_turn_180(MissionState.NAV_TO_GATE_WAIT)
        elif self.state == MissionState.NAV_TO_GATE_IN:
            self.handle_nav_state(self.gate_wait_point(), MissionState.WAIT_GATE_PERMISSION)
        elif self.state == MissionState.WAIT_GATE_PERMISSION_RETURN:
            self.handle_gate_wait(
                MissionState.PASS_GATE_CENTER,
                "PLC return permission granted",
            )
        elif self.state == MissionState.RETURN_TO_START:
            self.handle_return_route()

    def handle_nav_state(self, point_name: str, next_state: MissionState) -> None:
        if not self.nav_goal_pending and self.active_goal is None:
            self.send_nav_goal(point_name)
            return
        if self.nav_result_status is None:
            return

        status = self.nav_result_status
        self.clear_nav_tracking()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.stop_robot()
            self.transition(next_state, f"reached {point_name}")
        elif self.should_retry_nav_goal(point_name, status):
            self.stop_robot()
            self.publish_status(
                f"Nav2 goal {point_name} failed with status {status}; retrying once."
            )
        else:
            self.fail(f"Nav2 goal {point_name} failed with status {status}")

    def handle_return_route(self) -> None:
        if self.return_route_index >= len(RETURN_ROUTE_POINTS):
            self.transition(MissionState.TASK_DONE, "returned to START")
            return

        point_name = RETURN_ROUTE_POINTS[self.return_route_index]
        if not self.nav_goal_pending and self.active_goal is None:
            self.send_nav_goal(point_name)
            return
        if self.nav_result_status is None:
            return

        status = self.nav_result_status
        self.clear_nav_tracking()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.stop_robot()
            self.return_route_index += 1
            if self.return_route_index >= len(RETURN_ROUTE_POINTS):
                self.transition(MissionState.TASK_DONE, "returned to START")
            else:
                self.publish_status(f"Reached {point_name}; continuing return route.")
        else:
            self.fail(f"Nav2 goal {point_name} failed with status {status}")

    def handle_qr_confirm(self, expected_qr: str, next_state: MissionState) -> None:
        if self.has_recent_qr(expected_qr):
            if self.state == MissionState.QR_PICKUP_CONFIRM:
                if self.align_pickup_rear_camera_before_turn(expected_qr):
                    return

            self.stop_robot()
            self.reset_qr_buffer(expected_qr)
            if self.state == MissionState.QR_PICKUP_CONFIRM:
                self.transition(
                    next_state,
                    f"confirmed QR {expected_qr}; rear camera alignment complete",
                )
            else:
                self.transition(next_state, f"confirmed QR {expected_qr}")
        else:
            if self.state == MissionState.QR_DROPOFF_CONFIRM and self.carrying_load:
                self.scan_for_dropoff_qr(expected_qr)
                return
            self.stop_robot()
            self.publish_status(f"Waiting for QR {expected_qr}.")

    def align_pickup_rear_camera_before_turn(self, expected_qr: str) -> bool:
        if not self.pickup_qr_require_line:
            return False

        if not self.line_is_fresh():
            self.stop_robot()
            self.publish_status(
                f"QR {expected_qr} confirmed; waiting for rear-camera green line "
                "alignment before 180 turn."
            )
            return True

        if not self.have_odom:
            self.stop_robot()
            self.publish_status(
                f"QR {expected_qr} confirmed; waiting for odometry before "
                "rear-camera yaw alignment."
            )
            return True

        approach_name = self.pickup_prepoint(self.pickup)
        approach = self.points.get(approach_name)
        if approach is not None:
            yaw_error = normalize_angle(approach["yaw"] - self.current_yaw)
            if abs(yaw_error) > self.pickup_qr_yaw_tolerance:
                cmd = Twist()
                cmd.angular.z = max(
                    min(
                        self.pickup_qr_yaw_gain * yaw_error,
                        self.pickup_qr_align_max_angular,
                    ),
                    -self.pickup_qr_align_max_angular,
                )
                self.cmd_pub.publish(cmd)
                self.publish_status(
                    f"QR {expected_qr} confirmed; aligning rear-camera yaw "
                    f"before 180: target_yaw={approach['yaw']:.3f} "
                    f"yaw_error={yaw_error:.3f}/"
                    f"{self.pickup_qr_yaw_tolerance:.3f} "
                    f"wz={cmd.angular.z:.3f}"
                )
                return True

        line_heading = self.line_heading_error()
        if (
            abs(self.line_offset) <= self.pickup_qr_line_offset_tolerance
            and abs(line_heading) <= self.pickup_qr_line_angle_tolerance
        ):
            return False

        cmd = Twist()
        cmd.angular.z = max(
            min(self.angular_correction(), self.pickup_qr_align_max_angular),
            -self.pickup_qr_align_max_angular,
        )
        self.cmd_pub.publish(cmd)
        self.publish_status(
            f"QR {expected_qr} confirmed; aligning rear camera to green line "
            f"before 180: offset={self.line_offset:.3f}/"
            f"{self.pickup_qr_line_offset_tolerance:.3f} "
            f"angle={line_heading:.3f}/"
            f"{self.pickup_qr_line_angle_tolerance:.3f} "
            f"wz={cmd.angular.z:.3f}"
        )
        return True

    def scan_for_dropoff_qr(self, expected_qr: str) -> None:
        if not self.have_odom:
            self.stop_robot()
            self.publish_status(f"Waiting for odometry before QR {expected_qr} scan.")
            return
        if self.motion_start_yaw is None:
            self.motion_start_yaw = self.current_yaw

        yaw_delta = normalize_angle(self.current_yaw - self.motion_start_yaw)
        scan_limit = 0.30
        if yaw_delta >= scan_limit:
            self.qr_scan_direction = -1.0
        elif yaw_delta <= -scan_limit:
            self.qr_scan_direction = 1.0

        cmd = Twist()
        cmd.angular.z = 0.12 * self.qr_scan_direction
        self.cmd_pub.publish(cmd)
        self.publish_status(
            f"Scanning for QR {expected_qr} before dropoff rotation: "
            f"yaw_delta={yaw_delta:.3f} wz={cmd.angular.z:.3f}"
        )

    def handle_line_follow(self, next_state: MissionState) -> None:
        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before line following.")
            return
        if self.motion_start_xy is None:
            self.motion_start_xy = (self.current_x, self.current_y)

        if not self.line_is_fresh():
            self.stop_robot()
            self.publish_status("Waiting for fresh precision line detection.")
            return

        line_heading = self.line_heading_error()
        if abs(self.line_offset) > self.line_offset_tolerance or abs(
            line_heading
        ) > self.line_angle_tolerance:
            if self.can_acquire_visible_line(line_heading):
                cmd = Twist()
                cmd.linear.x = self.line_acquire_speed
                cmd.angular.z = self.angular_correction()
                self.cmd_pub.publish(cmd)
                self.publish_status(
                    f"Acquiring precision line: offset={self.line_offset:.3f}, "
                    f"angle={line_heading:.3f} raw_angle={self.line_angle:.3f}"
                )
                return

            self.stop_robot()
            self.publish_status(
                f"Line offset too high: offset={self.line_offset:.3f}, "
                f"angle={line_heading:.3f} raw_angle={self.line_angle:.3f}"
            )
            return

        if self.distance_from_motion_start() >= self.line_follow_distance:
            self.stop_robot()
            self.transition(next_state, "line follow distance complete")
            return

        cmd = Twist()
        cmd.linear.x = self.line_follow_speed
        cmd.angular.z = self.angular_correction()
        self.cmd_pub.publish(cmd)

    def handle_final_align(self, next_state: MissionState) -> None:
        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before final alignment.")
            return
        if self.motion_start_xy is None:
            self.motion_start_xy = (self.current_x, self.current_y)

        if not self.line_is_fresh():
            self.stop_robot()
            self.publish_status("Waiting for fresh precision line detection.")
            return

        line_heading = self.line_heading_error()
        heading_ok = abs(line_heading) <= self.final_line_angle_tolerance
        centered_near_field = (
            abs(self.line_offset) <= self.dropoff_final_centered_offset_tolerance
        )
        if abs(self.line_offset) > self.final_line_offset_tolerance or (
            not heading_ok and not centered_near_field
        ):
            self.stop_robot()
            self.publish_status(
                f"Final align error: offset={self.line_offset:.3f}, "
                f"angle={line_heading:.3f} raw_angle={self.line_angle:.3f}"
            )
            return

        if self.distance_from_motion_start() >= self.final_align_distance:
            self.stop_robot()
            self.transition(next_state, "final alignment complete")
            return

        cmd = Twist()
        cmd.linear.x = self.final_align_speed
        if heading_ok:
            cmd.angular.z = self.angular_correction()
        else:
            cmd.angular.z = max(
                min(-self.angular_offset_gain * self.line_offset, 0.15),
                -0.15,
            )
            self.publish_status(
                "Final align using centered near-field line; "
                f"offset={self.line_offset:.3f}, raw_angle={self.line_angle:.3f}"
            )
        self.cmd_pub.publish(cmd)

    def handle_odom_approach_pickup(self) -> None:
        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before pickup approach.")
            return
        if self.motion_start_xy is None:
            self.motion_start_xy = (self.current_x, self.current_y)
        if self.motion_start_yaw is None:
            self.motion_start_yaw = self.current_yaw

        target_name = self.pickup_target_point(self.pickup)
        relative = self.pickup_target_relative_to_robot()
        if relative is None:
            self.fail(f"missing pickup target point {target_name}")
            return
        forward, lateral, yaw_error, target_detail = relative
        traveled = self.distance_from_motion_start()
        geometry_ok, geometry_detail = self.verify_pickup_geometry()

        if traveled >= self.pickup_blind_approach_min_distance and geometry_ok:
            self.stop_robot()
            self.transition(
                MissionState.VERIFY_FORK_INSERTION,
                "blind odom pickup approach reached fork insertion window: "
                f"traveled={traveled:.3f} m; {geometry_detail}",
            )
            return

        if traveled >= self.pickup_blind_approach_max_distance:
            self.stop_robot()
            self.fail(
                "Blind pickup approach reached max distance without verified "
                f"fork insertion: traveled={traveled:.3f}/"
                f"{self.pickup_blind_approach_max_distance:.3f}; "
                f"{geometry_detail}"
            )
            return

        if forward <= 0.0:
            self.stop_robot()
            self.fail(
                "Blind pickup approach failed: pallet target is not ahead of forks: "
                f"{target_detail} forward={forward:.3f} lateral={lateral:.3f}"
            )
            return

        yaw_drift = normalize_angle(self.current_yaw - self.motion_start_yaw)
        cmd = Twist()
        cmd.linear.x = self.pickup_blind_approach_speed
        cmd.angular.z = max(
            min(
                -self.pickup_blind_heading_gain * yaw_drift,
                self.pickup_blind_max_angular,
            ),
            -self.pickup_blind_max_angular,
        )
        cmd.angular.z = self.smooth_command(
            cmd.angular.z,
            self.last_pickup_angular_cmd,
            self.pickup_align_smoothing_alpha,
            self.pickup_align_max_angular_step,
        )
        self.last_pickup_angular_cmd = cmd.angular.z
        self.cmd_pub.publish(cmd)
        self.publish_status(
            "Blind odom pickup approach after QR+180; camera disabled: "
            f"traveled={traveled:.3f}/"
            f"{self.pickup_blind_approach_min_distance:.3f}-"
            f"{self.pickup_blind_approach_max_distance:.3f} "
            f"{target_detail} forward={forward:.3f} "
            f"lateral={lateral:.3f} yaw_error={yaw_error:.3f} "
            f"yaw_drift={yaw_drift:.3f} "
            f"vx={cmd.linear.x:.3f} wz={cmd.angular.z:.3f}"
        )

    def handle_final_align_pickup(self) -> None:
        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before pickup final alignment.")
            return
        if self.motion_start_xy is None:
            self.motion_start_xy = (self.current_x, self.current_y)

        traveled = self.distance_from_motion_start()
        ok, detail = self.verify_pickup_geometry()
        if ok:
            self.stop_robot()
            self.transition(
                MissionState.VERIFY_FORK_INSERTION,
                f"pickup odom final alignment verified: {detail}",
            )
            return

        target_name = self.pickup_target_point(self.pickup)
        relative = self.pickup_target_relative_to_robot()
        if relative is None:
            self.fail(f"missing pickup target point {target_name}")
            return
        forward, lateral, yaw_error, _ = relative

        max_travel = self.final_align_distance + self.pickup_align_extra_distance
        if traveled >= max_travel:
            self.stop_robot()
            self.fail(
                f"Pickup final alignment failed after {traveled:.3f} m: "
                f"{detail}"
            )
            return
        if forward <= self.pickup_verify_forward_min:
            self.stop_robot()
            self.fail(
                "Pickup final alignment failed: pallet is too close without valid "
                f"fork alignment: {detail}"
            )
            return

        cmd = Twist()
        cmd.linear.x = self.final_align_speed
        cmd.angular.z = self.pickup_odom_angular_correction(lateral, yaw_error)
        self.cmd_pub.publish(cmd)
        self.publish_status(
            f"Correcting fork alignment: forward={forward:.3f} "
            f"lateral={lateral:.3f} yaw_error={yaw_error:.3f} "
            f"vx={cmd.linear.x:.3f} wz={cmd.angular.z:.3f}"
        )

    def handle_verify_fork_insertion(self) -> None:
        self.stop_robot()
        ok, detail = self.verify_pickup_geometry()
        if not ok:
            self.pickup_insertion_start_time = None
            self.publish_pickup_success(False)
            self.fail(f"Fork insertion verification failed before lift: {detail}")
            return

        now = self.now_sec()
        if self.pickup_insertion_start_time is None:
            self.pickup_insertion_start_time = now
            self.publish_status(f"Fork insertion check started: {detail}")
            return

        if now - self.pickup_insertion_start_time < self.pickup_insertion_hold_sec:
            self.publish_status(
                "Fork insertion holding for confirmation; "
                f"elapsed={now - self.pickup_insertion_start_time:.2f}s"
            )
            return

        self.transition(MissionState.PICK_LOAD, f"fork insertion verified: {detail}")

    def handle_pick_load(self) -> None:
        self.stop_robot()
        if self.load_pose_before_lift is None:
            load_pose = self.get_load_pose()
            if load_pose is None:
                self.publish_status("Waiting for load pose before lift.")
                return
            self.load_pose_before_lift = load_pose
            self.load_pose_before_lift_time = self.now_sec()
            self.publish_status(
                f"Captured load pose before lift: z={load_pose[2]:.3f}"
            )

        if not self.fork_command_sent:
            ok, detail = self.verify_pickup_geometry()
            self.last_pickup_verify_detail = detail
            if not ok:
                self.publish_pickup_success(False)
                self.fail(f"Pickup alignment verification failed before lift: {detail}")
                return
            self.publish_status(
                "Robot stopped; fork insertion verified before LIFT_UP: "
                f"{detail}"
            )
            self.publish_fork_command("LIFT_UP")
            self.fork_command_sent = True

        if self.now_sec() - self.state_enter_time >= self.fork_action_sec:
            self.transition(
                MissionState.VERIFY_PICKUP_SUCCESS,
                "fork lift command complete; verifying pickup",
            )

    def handle_verify_pickup_success(self) -> None:
        self.stop_robot()
        geometry_ok, geometry_detail = self.verify_pickup_geometry()

        if not geometry_ok:
            self.publish_status(
                "Post-lift insertion geometry changed; verifying real load pose: "
                f"{geometry_detail}"
            )

        fork_reached = self.fork_lift_reached()
        load_ok, load_detail = self.verify_load_lifted()
        if not load_ok:
            elapsed = self.now_sec() - self.state_enter_time
            if elapsed < self.pickup_verify_timeout_sec:
                self.publish_status(
                    "Waiting for real load lift confirmation: "
                    f"{load_detail}; fork_height={self.fork_height_text()}"
                )
                return
            self.publish_pickup_success(False)
            self.fail(
                "Pickup success verification failed: "
                f"{load_detail}; fork_height={self.fork_height_text()}"
            )
            return

        if not fork_reached:
            self.publish_status(
                "Fork height telemetry unavailable or below target; "
                "accepting real load lift confirmation: "
                f"{load_detail}; fork_height={self.fork_height_text()}"
            )

        self.pickup_verified = True
        self.carrying_load = True
        self.load_pose_after_lift = self.get_load_pose()
        self.robot_pose_after_lift = self.get_gazebo_model_pose(self.robot_model_frame)
        self.capture_carried_load_offset()
        self.publish_pickup_success(True)
        self.publish_fork_command("LIFT_STOP")
        self.transition(
            MissionState.BACK_OUT_SHORT,
            "pickup success verified: "
            f"{load_detail}, fork_height={self.fork_height_text()}, "
            f"geometry_before_lift={self.last_pickup_verify_detail}, "
            f"post_lift_geometry={geometry_detail}",
        )

    def handle_odom_approach_dropoff(self) -> None:
        if not self.carrying_load:
            self.publish_pickup_success(False)
            self.fail("Cannot approach dropoff: no verified load is being carried.")
            return
        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before dropoff approach.")
            return

        self.update_carried_load_pose()
        relative = self.dropoff_target_relative_to_load()
        if relative is None:
            self.stop_robot()
            self.publish_status("Waiting for carried load pose before dropoff approach.")
            return
        forward, lateral, yaw_error, target_detail = relative

        if forward <= self.dropoff_final_start_forward:
            self.stop_robot()
            self.transition(
                MissionState.FINAL_ALIGN_DROPOFF,
                "dropoff odom approach complete: "
                f"{target_detail} forward={forward:.3f} "
                f"lateral={lateral:.3f} yaw_error={yaw_error:.3f}",
            )
            return

        if forward <= -self.dropoff_verify_forward_tolerance:
            self.stop_robot()
            self.fail(
                "Dropoff odom approach failed: drop target passed before final align: "
                f"{target_detail} forward={forward:.3f} lateral={lateral:.3f}"
            )
            return

        cmd = Twist()
        cmd.linear.x = self.line_follow_speed
        cmd.angular.z = self.pickup_odom_angular_correction(lateral, yaw_error)
        self.cmd_pub.publish(cmd)
        self.publish_status(
            "Odom dropoff approach: "
            f"{target_detail} forward={forward:.3f} "
            f"lateral={lateral:.3f} yaw_error={yaw_error:.3f} "
            f"vx={cmd.linear.x:.3f} wz={cmd.angular.z:.3f}"
        )

    def handle_final_align_dropoff(self) -> None:
        if not self.carrying_load:
            self.publish_pickup_success(False)
            self.fail("Cannot final-align dropoff: no verified load is being carried.")
            return
        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before dropoff final alignment.")
            return
        if self.motion_start_xy is None:
            self.motion_start_xy = (self.current_x, self.current_y)

        self.update_carried_load_pose()
        ok, detail = self.verify_dropoff_alignment()
        if ok:
            self.stop_robot()
            self.transition(MissionState.DROP_LOAD, f"dropoff alignment verified: {detail}")
            return

        relative = self.dropoff_target_relative_to_load()
        if relative is None:
            self.stop_robot()
            self.publish_status("Waiting for carried load pose during dropoff final align.")
            return
        forward, lateral, yaw_error, _ = relative

        max_travel = self.final_align_distance + self.dropoff_align_extra_distance
        if self.distance_from_motion_start() >= max_travel:
            self.stop_robot()
            self.fail(
                f"Dropoff final alignment failed after "
                f"{self.distance_from_motion_start():.3f} m: {detail}"
            )
            return
        if forward <= -self.dropoff_verify_forward_tolerance:
            self.stop_robot()
            self.fail(f"Dropoff final alignment overshot the target: {detail}")
            return

        cmd = Twist()
        cmd.linear.x = self.final_align_speed
        cmd.angular.z = self.pickup_odom_angular_correction(lateral, yaw_error)
        self.cmd_pub.publish(cmd)
        self.publish_status(
            f"Correcting dropoff alignment: forward={forward:.3f} "
            f"lateral={lateral:.3f} yaw_error={yaw_error:.3f} "
            f"vx={cmd.linear.x:.3f} wz={cmd.angular.z:.3f}"
        )

    def handle_drop_load(self) -> None:
        self.stop_robot()
        if not self.carrying_load:
            self.publish_pickup_success(False)
            self.fail("Drop requested but no verified load is being carried.")
            return

        if self.drop_load_pose_before_lower is None:
            self.update_carried_load_pose(force=True)
            load_pose = self.get_load_pose()
            if load_pose is None:
                self.publish_status("Waiting for load pose before lowering.")
                return
            ok, detail = self.verify_dropoff_alignment()
            if not ok:
                self.fail(f"Dropoff alignment verification failed before LIFT_DOWN: {detail}")
                return
            self.drop_load_pose_before_lower = load_pose
            self.publish_status(
                "Robot stopped; dropoff alignment verified before LIFT_DOWN: "
                f"{detail}; load_z={load_pose[2]:.3f}"
            )

        if not self.fork_command_sent:
            self.publish_fork_command("LIFT_DOWN")
            self.carried_load_offset = None
            self.last_carried_load_pose_update = -1.0
            self.publish_status("Released carried load pose lock for physical dropoff verification.")
            self.fork_command_sent = True

        if self.now_sec() - self.state_enter_time >= self.fork_action_sec:
            self.transition(
                MissionState.VERIFY_DROPOFF_RELEASE,
                "fork lower command complete; verifying release",
            )

    def handle_verify_dropoff_release(self) -> None:
        self.stop_robot()
        ok, detail = self.verify_dropoff_release()
        if not ok:
            elapsed = self.now_sec() - self.state_enter_time
            if elapsed < self.dropoff_verify_timeout_sec:
                self.publish_status(f"Waiting for dropoff release confirmation: {detail}")
                return
            self.fail(f"Dropoff release verification failed: {detail}")
            return

        self.carrying_load = False
        self.pickup_verified = False
        self.carried_load_offset = None
        self.last_carried_load_pose_update = -1.0
        self.drop_load_pose_after_release = self.get_load_pose()
        self.publish_pickup_success(False)
        self.transition(MissionState.BACK_OUT_DROPOFF, f"dropoff release verified: {detail}")

    def handle_dropoff_back_out(self) -> None:
        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before dropoff backout.")
            return
        if self.motion_start_xy is None:
            self.motion_start_xy = (self.current_x, self.current_y)
        if self.motion_start_yaw is None:
            self.motion_start_yaw = self.current_yaw

        if self.distance_from_motion_start() >= self.dropoff_back_out_distance:
            self.stop_robot()
            self.transition(MissionState.TURN_180_TO_RETURN, "backed out from dropoff")
            return

        self.back_out_speed_cmd = min(
            abs(self.dropoff_back_out_speed),
            self.back_out_speed_cmd + max(self.back_out_accel_step, 0.0),
        )
        yaw_error = normalize_angle(self.current_yaw - self.motion_start_yaw)
        cmd = Twist()
        cmd.linear.x = -self.back_out_speed_cmd
        cmd.angular.z = max(
            min(-self.back_out_yaw_gain * yaw_error, self.back_out_max_angular),
            -self.back_out_max_angular,
        )
        self.cmd_pub.publish(cmd)


    def handle_fork_action(self, command: str, next_state: MissionState) -> None:
        self.stop_robot()
        if not self.fork_command_sent:
            self.publish_fork_command(command)
            self.fork_command_sent = True

        if self.now_sec() - self.state_enter_time >= self.fork_action_sec:
            self.transition(next_state, f"{command} complete")

    def handle_back_out(self) -> None:
        if not self.pickup_verified:
            self.stop_robot()
            self.publish_pickup_success(False)
            self.fail("Cannot back out: pickup was not physically verified.")
            return
        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before backing out.")
            return
        if self.motion_start_xy is None:
            self.motion_start_xy = (self.current_x, self.current_y)
        if self.motion_start_yaw is None:
            self.motion_start_yaw = self.current_yaw

        self.update_carried_load_pose()

        backout_distance = self.distance_from_motion_start()
        max_backout_distance = (
            self.back_out_distance + self.load_follow_extra_backout_distance
        )
        if backout_distance >= self.back_out_distance:
            verify_now = (
                self.last_load_follow_verify_wall < 0.0
                or time.monotonic() - self.last_load_follow_verify_wall
                >= self.load_follow_verify_period
                or backout_distance >= max_backout_distance
            )
            if verify_now:
                self.last_load_follow_verify_wall = time.monotonic()
                self.stop_robot()
                ok, detail = self.verify_load_followed_robot()
                if ok:
                    self.publish_status(f"Load moved with robot after backout: {detail}")
                    self.transition(MissionState.TURN_180_TO_NAV, "backed out from pallet")
                    return
                if not self.require_load_follow_verification:
                    self.publish_status(f"Load follow verification deferred: {detail}")
                    self.transition(MissionState.TURN_180_TO_NAV, "backed out from pallet")
                    return
                if backout_distance >= max_backout_distance:
                    self.pickup_verified = False
                    self.carrying_load = False
                    self.publish_pickup_success(False)
                    self.fail(f"Load follow verification failed after backout: {detail}")
                    return
                self.publish_status(
                    "Continuing odom backout for load-follow verification: "
                    f"{detail}; backout={backout_distance:.3f}/"
                    f"{max_backout_distance:.3f}"
                )

        self.back_out_speed_cmd = min(
            abs(self.back_out_speed),
            self.back_out_speed_cmd + max(self.back_out_accel_step, 0.0),
        )
        yaw_error = normalize_angle(self.current_yaw - self.motion_start_yaw)
        cmd = Twist()
        cmd.linear.x = -self.back_out_speed_cmd
        cmd.angular.z = max(
            min(-self.back_out_yaw_gain * yaw_error, self.back_out_max_angular),
            -self.back_out_max_angular,
        )
        self.cmd_pub.publish(cmd)
        self.publish_status(
            f"Backing out after pickup: backout={backout_distance:.3f}/"
            f"{self.back_out_distance:.3f} yaw_error={yaw_error:.3f} "
            f"vx={cmd.linear.x:.3f} wz={cmd.angular.z:.3f}"
        )

    def handle_turn_180(self, next_state: MissionState) -> None:
        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before 180 turn.")
            return
        if self.motion_start_yaw is None:
            self.motion_start_yaw = self.current_yaw

        if self.carrying_load:
            self.update_carried_load_pose()

        target_yaw = normalize_angle(self.motion_start_yaw + math.pi)
        yaw_error = normalize_angle(target_yaw - self.current_yaw)
        if abs(yaw_error) <= self.turn_tolerance:
            self.stop_robot()
            self.transition(next_state, "180 degree turn complete")
            return

        angular_speed = min(
            abs(self.turn_angular_speed),
            max(
                abs(self.turn_min_angular_speed),
                abs(self.turn_slowdown_gain * yaw_error),
            ),
        )
        cmd = Twist()
        cmd.angular.z = math.copysign(angular_speed, yaw_error)
        self.cmd_pub.publish(cmd)
        self.publish_status(
            f"Turning 180: target_yaw={target_yaw:.3f} "
            f"yaw_error={yaw_error:.3f} wz={cmd.angular.z:.3f}"
        )

    def handle_gate_wait(self, next_state: MissionState, reason: str) -> None:
        self.stop_robot()
        if self.now_sec() - self.state_enter_time < self.gate_wait_sec:
            return

        if self.carrying_load and next_state == MissionState.PASS_GATE_CENTER:
            desired_yaw = self.gate_drive_yaw()
            yaw_error = normalize_angle(self.current_yaw - desired_yaw)
            if abs(yaw_error) > self.loaded_gate_entry_yaw_tolerance:
                self.turn_to_yaw(desired_yaw, "Loaded gate entry")
                return

        self.transition(next_state, reason)

    def handle_loaded_dropoff_prepoint_drive(self) -> None:
        if not self.carrying_load:
            self.fail("Cannot drive to loaded dropoff prepoint without verified load.")
            return

        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before loaded dropoff prepoint drive.")
            return

        prepoint_name = self.dropoff_prepoint(self.dropoff)
        if prepoint_name not in self.points:
            self.fail(f"missing point {prepoint_name}")
            return
        if self.gate_clear_point_out not in self.points:
            self.fail(f"missing point {self.gate_clear_point_out}")
            return

        prepoint = self.points[prepoint_name]
        gate_clear = self.points[self.gate_clear_point_out]
        route = (
            ("drive_south", gate_clear["x"], prepoint["y"], -math.pi / 2.0),
            ("turn_east", None, None, 0.0),
            ("drive_across", prepoint["x"], prepoint["y"], 0.0),
            ("turn_to_qr", None, None, prepoint["yaw"]),
        )

        if self.loaded_dropoff_route_index >= len(route):
            self.stop_robot()
            self.transition(
                MissionState.QR_DROPOFF_CONFIRM,
                f"loaded dropoff prepoint reached: {prepoint_name}",
            )
            return

        stage_name, target_x, target_y, target_yaw = route[self.loaded_dropoff_route_index]
        if target_x is None or target_y is None:
            if self.turn_to_yaw(target_yaw, f"Loaded dropoff {stage_name}"):
                self.sync_after_loaded_route_stage()
                self.last_loaded_route_angular_cmd = 0.0
                self.loaded_dropoff_route_index += 1
            return

        if self.drive_to_xy_yaw(
            target_x,
            target_y,
            target_yaw,
            f"Loaded dropoff {stage_name}",
        ):
            self.sync_after_loaded_route_stage()
            synced_distance = math.hypot(
                target_x - self.current_x,
                target_y - self.current_y,
            )
            if synced_distance > self.gate_drive_goal_tolerance:
                self.publish_status(
                    "Loaded route physical sync corrected pose; continuing "
                    f"{stage_name}: distance={synced_distance:.3f}"
                )
                return
            self.last_loaded_route_angular_cmd = 0.0
            self.loaded_dropoff_route_index += 1

    def sync_after_loaded_route_stage(self) -> None:
        if not self.loaded_route_pose_sync_enabled:
            return
        if self.sync_current_pose_from_gazebo(force=True):
            self.update_carried_load_pose(force=True)
            self.publish_status(
                "Loaded route physical pose synced: "
                f"correction=({self.odom_pose_correction_x:.3f},"
                f"{self.odom_pose_correction_y:.3f},"
                f"{self.odom_pose_correction_yaw:.3f})"
            )
        else:
            self.publish_status("Loaded route physical pose sync unavailable.")

    def turn_to_yaw(self, target_yaw: float, label: str) -> bool:
        if self.carrying_load:
            self.sync_current_pose_from_gazebo()
            if self.state in GATE_CROSSING_STATES | {MissionState.NAV_TO_GATE_WAIT}:
                self.sync_current_pose_from_gazebo(force=True)
        if self.carrying_load and self.state != MissionState.NAV_TO_DROPOFF_PREPOINT:
            self.update_carried_load_pose()

        yaw_error = normalize_angle(target_yaw - self.current_yaw)
        if abs(yaw_error) <= self.turn_tolerance:
            self.stop_robot()
            self.publish_status(
                f"{label}: yaw aligned target={target_yaw:.3f} "
                f"yaw_error={yaw_error:.3f}"
            )
            return True

        loaded_precise_turn = self.carrying_load and self.state in (
            MissionState.NAV_TO_GATE_WAIT,
            MissionState.WAIT_GATE_PERMISSION,
            MissionState.PASS_GATE_CENTER,
            MissionState.GATE_CLEAR_AFTER,
            MissionState.NAV_TO_DROPOFF_PREPOINT,
        )
        turn_speed = abs(self.turn_angular_speed)
        if loaded_precise_turn:
            turn_speed = min(turn_speed, abs(self.loaded_route_turn_angular_speed))
        angular = math.copysign(turn_speed, yaw_error)
        if loaded_precise_turn:
            angular = self.smooth_loaded_route_angular(angular)

        cmd = Twist()
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        if loaded_precise_turn:
            self.update_carried_load_pose()
        self.publish_status(
            f"{label}: turning target_yaw={target_yaw:.3f} "
            f"yaw_error={yaw_error:.3f} wz={cmd.angular.z:.3f}"
        )
        return False

    def drive_to_xy_yaw(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
        label: str,
    ) -> bool:
        if self.carrying_load:
            self.sync_current_pose_from_gazebo()
        if self.carrying_load and self.state != MissionState.NAV_TO_DROPOFF_PREPOINT:
            self.update_carried_load_pose()

        forward, lateral, yaw_error = self.point_relative_to_robot(
            target_x,
            target_y,
            target_yaw,
        )
        distance = math.hypot(target_x - self.current_x, target_y - self.current_y)
        if distance <= self.gate_drive_goal_tolerance:
            self.stop_robot()
            self.publish_status(
                f"{label}: reached target distance={distance:.3f} "
                f"lateral={lateral:.3f}"
            )
            return True

        heading_error = math.atan2(lateral, forward)
        if abs(heading_error) > 0.55:
            cmd = Twist()
            cmd.angular.z = max(
                min(0.6 * heading_error, self.loaded_route_max_angular),
                -self.loaded_route_max_angular,
            )
            cmd.angular.z = self.smooth_loaded_route_angular(cmd.angular.z)
            self.cmd_pub.publish(cmd)
            if self.carrying_load:
                self.update_carried_load_pose()
            self.publish_status(
                f"{label}: rotating toward target distance={distance:.3f} "
                f"forward={forward:.3f} lateral={lateral:.3f} "
                f"heading_error={heading_error:.3f} wz={cmd.angular.z:.3f}"
            )
            return False

        speed = min(self.gate_drive_speed, self.loaded_route_drive_speed)
        if self.gate_drive_slow_radius > 0.0:
            speed *= min(1.0, max(distance / self.gate_drive_slow_radius, 0.0))
        speed = max(min(speed, self.gate_drive_speed), self.gate_drive_min_speed)

        heading_error = self.apply_deadband(
            heading_error,
            self.loaded_route_heading_deadband,
        )
        yaw_correction = self.apply_deadband(
            normalize_angle(self.current_yaw - target_yaw),
            self.loaded_route_yaw_deadband,
        )
        angular = (
            self.loaded_route_heading_gain * heading_error
            - self.loaded_route_yaw_gain * yaw_correction
        )
        angular = max(
            min(angular, self.loaded_route_max_angular),
            -self.loaded_route_max_angular,
        )
        angular = self.smooth_loaded_route_angular(angular)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        if self.carrying_load:
            self.update_carried_load_pose()
        self.publish_status(
            f"{label}: distance={distance:.3f} forward={forward:.3f} "
            f"lateral={lateral:.3f} yaw_error={yaw_error:.3f} "
            f"vx={cmd.linear.x:.3f} wz={cmd.angular.z:.3f}"
        )
        return False

    def smooth_loaded_route_angular(self, target: float) -> float:
        angular = self.smooth_command(
            target,
            self.last_loaded_route_angular_cmd,
            self.loaded_route_smoothing_alpha,
            self.loaded_route_max_angular_step,
        )
        self.last_loaded_route_angular_cmd = angular
        return angular

    def handle_gate_drive(
        self,
        point_name: str,
        next_state: MissionState,
        reason: str,
    ) -> None:
        if not self.have_odom:
            self.stop_robot()
            self.publish_status("Waiting for odometry before gate crossing.")
            return
        if point_name not in self.points:
            self.fail(f"missing point {point_name}")
            return

        if self.carrying_load:
            self.sync_current_pose_from_gazebo()

        point = self.gate_drive_target(point_name)
        desired_yaw = self.gate_drive_yaw()
        forward, lateral, yaw_error = self.point_relative_to_robot(
            point["x"],
            point["y"],
            desired_yaw,
        )
        distance = math.hypot(point["x"] - self.current_x, point["y"] - self.current_y)
        crossed_target_line = (
            forward <= self.gate_drive_goal_tolerance
            and abs(lateral) <= self.gate_drive_lateral_tolerance
        )
        loaded_lateral_tolerance = max(
            self.gate_drive_lateral_tolerance,
            self.loaded_gate_lateral_tolerance,
        )
        loaded_gate_wait_reached = (
            self.carrying_load
            and point_name == self.gate_wait_point_out
            and forward <= max(
                self.gate_drive_slow_radius,
                self.gate_drive_goal_tolerance,
                self.loaded_gate_wait_forward_tolerance,
            )
            and abs(lateral) <= max(
                loaded_lateral_tolerance,
                self.loaded_gate_wait_lateral_tolerance,
            )
        )
        loaded_gate_pass_reached = (
            self.carrying_load
            and point_name == self.pass_gate_point
            and forward <= max(
                self.gate_drive_goal_tolerance,
                self.loaded_gate_pass_goal_tolerance,
            )
            and abs(lateral) <= loaded_lateral_tolerance
        )
        if (
            distance <= self.gate_drive_goal_tolerance
            or crossed_target_line
            or loaded_gate_wait_reached
            or loaded_gate_pass_reached
        ):
            if self.carrying_load and self.sync_current_pose_from_gazebo(force=True):
                forward, lateral, yaw_error = self.point_relative_to_robot(
                    point["x"],
                    point["y"],
                    desired_yaw,
                )
                distance = math.hypot(
                    point["x"] - self.current_x,
                    point["y"] - self.current_y,
                )
                crossed_target_line = (
                    forward <= self.gate_drive_goal_tolerance
                    and abs(lateral) <= self.gate_drive_lateral_tolerance
                )
                loaded_gate_wait_reached = (
                    point_name == self.gate_wait_point_out
                    and forward <= max(
                        self.gate_drive_slow_radius,
                        self.gate_drive_goal_tolerance,
                        self.loaded_gate_wait_forward_tolerance,
                    )
                    and abs(lateral) <= max(
                        loaded_lateral_tolerance,
                        self.loaded_gate_wait_lateral_tolerance,
                    )
                )
                loaded_gate_pass_reached = (
                    point_name == self.pass_gate_point
                    and forward <= max(
                        self.gate_drive_goal_tolerance,
                        self.loaded_gate_pass_goal_tolerance,
                    )
                    and abs(lateral) <= loaded_lateral_tolerance
                )
                if (
                    distance > self.gate_drive_goal_tolerance
                    and not crossed_target_line
                    and not loaded_gate_wait_reached
                    and not loaded_gate_pass_reached
                ):
                    self.publish_status(
                        f"Loaded gate physical sync corrected {point_name}; "
                        f"continuing distance={distance:.3f} "
                        f"forward={forward:.3f} lateral={lateral:.3f}"
                    )
                    return
            if (
                self.carrying_load
                and point_name in (self.gate_wait_point_out, self.pass_gate_point)
                and not (loaded_gate_wait_reached or loaded_gate_pass_reached)
            ):
                gate_yaw_error = normalize_angle(self.current_yaw - desired_yaw)
                if abs(gate_yaw_error) > self.loaded_gate_entry_yaw_tolerance:
                    self.turn_to_yaw(desired_yaw, f"Loaded gate {point_name}")
                    return
            self.stop_robot()
            self.transition(
                next_state,
                f"{reason}: reached {point_name} "
                f"(distance={distance:.3f}, forward={forward:.3f}, "
                f"lateral={lateral:.3f})",
            )
            return

        heading_error = math.atan2(lateral, forward)
        loaded_heading_gain = self.loaded_gate_heading_gain
        loaded_yaw_gain = self.loaded_gate_yaw_gain
        loaded_angular_limit = self.loaded_gate_max_angular
        if self.carrying_load and self.state == MissionState.PASS_GATE_CENTER:
            loaded_heading_gain = self.loaded_gate_pass_heading_gain
            loaded_yaw_gain = self.loaded_gate_pass_yaw_gain
            loaded_angular_limit = self.loaded_gate_pass_max_angular
            if abs(lateral) > 0.45:
                cmd = Twist()
                cmd.angular.z = max(
                    min(loaded_heading_gain * heading_error, loaded_angular_limit),
                    -loaded_angular_limit,
                )
                cmd.angular.z = self.smooth_loaded_route_angular(cmd.angular.z)
                self.cmd_pub.publish(cmd)
                self.update_carried_load_pose()
                self.publish_status(
                    f"Gate drive {point_name}: centering before pass "
                    f"distance={distance:.3f} forward={forward:.3f} "
                    f"lateral={lateral:.3f} heading_error={heading_error:.3f} "
                    f"wz={cmd.angular.z:.3f}"
                )
                return

        if abs(heading_error) > 0.55:
            cmd = Twist()
            angular_limit = self.gate_drive_max_angular
            angular_gain = 0.8
            if self.carrying_load:
                angular_limit = min(
                    self.gate_drive_max_angular,
                    loaded_angular_limit,
                )
                angular_gain = loaded_heading_gain
            cmd.angular.z = max(
                min(angular_gain * heading_error, angular_limit),
                -angular_limit,
            )
            if self.carrying_load:
                cmd.angular.z = self.smooth_loaded_route_angular(cmd.angular.z)
            self.cmd_pub.publish(cmd)
            if self.carrying_load:
                self.update_carried_load_pose()
            self.publish_status(
                f"Gate drive {point_name}: rotating toward target "
                f"distance={distance:.3f} forward={forward:.3f} "
                f"lateral={lateral:.3f} heading_error={heading_error:.3f} "
                f"wz={cmd.angular.z:.3f}"
            )
            return

        speed = self.gate_drive_speed
        if self.carrying_load:
            speed = min(speed, self.loaded_route_drive_speed)
        if self.gate_drive_slow_radius > 0.0:
            speed *= min(1.0, max(distance / self.gate_drive_slow_radius, 0.0))
        speed = max(min(speed, self.gate_drive_speed), self.gate_drive_min_speed)
        if self.carrying_load and self.state == MissionState.PASS_GATE_CENTER:
            lateral_abs = abs(lateral)
            yaw_abs = abs(yaw_error)
            if lateral_abs > 0.35:
                speed = min(
                    speed,
                    max(self.gate_drive_min_speed, self.loaded_route_drive_speed * 0.50),
                )
            elif lateral_abs > 0.15:
                speed = min(
                    speed,
                    max(self.gate_drive_min_speed, self.loaded_route_drive_speed * 0.70),
                )
            if yaw_abs > 0.50:
                speed = self.gate_drive_min_speed
            elif yaw_abs > 0.30:
                speed = min(
                    speed,
                    max(self.gate_drive_min_speed, self.loaded_route_drive_speed * 0.65),
                )

        if self.carrying_load:
            heading_error = self.apply_deadband(
                heading_error,
                self.loaded_route_heading_deadband,
            )
            yaw_correction = self.apply_deadband(
                yaw_error,
                self.loaded_route_yaw_deadband,
            )
            if self.state == MissionState.PASS_GATE_CENTER:
                if abs(lateral) < 0.35 and abs(yaw_correction) > 0.18:
                    angular = -loaded_yaw_gain * yaw_correction
                else:
                    angular = (
                        loaded_heading_gain * heading_error
                        - 0.25 * loaded_yaw_gain * yaw_correction
                    )
            else:
                angular = (
                    loaded_heading_gain * heading_error
                    - loaded_yaw_gain * yaw_correction
                )
            angular = max(
                min(angular, loaded_angular_limit),
                -loaded_angular_limit,
            )
            angular = self.smooth_loaded_route_angular(angular)
        else:
            angular = (
                self.gate_drive_lateral_gain * self.apply_deadband(lateral, 0.015)
                - self.gate_drive_yaw_gain * self.apply_deadband(yaw_error, 0.025)
            )
            angular = max(
                min(angular, self.gate_drive_max_angular),
                -self.gate_drive_max_angular,
            )

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        if self.carrying_load:
            self.update_carried_load_pose()
        self.publish_status(
            f"Gate drive {point_name}: distance={distance:.3f} "
            f"forward={forward:.3f} lateral={lateral:.3f} "
            f"yaw_error={yaw_error:.3f} vx={cmd.linear.x:.3f} "
            f"wz={cmd.angular.z:.3f}"
        )

    def gate_drive_target(self, point_name: str) -> dict:
        point = dict(self.points[point_name])
        if (
            self.carrying_load
            and point_name == self.pass_gate_point
        ):
            point["x"] = float(point["x"]) + self.loaded_gate_pass_lane_x_offset
            point["y"] = float(point["y"]) + self.loaded_gate_pass_y_offset
        elif (
            self.carrying_load
            and point_name in (self.gate_wait_point_out, self.gate_clear_point_out)
            and self.loaded_gate_lane_x_offset != 0.0
        ):
            point["x"] = float(point["x"]) + self.loaded_gate_lane_x_offset
        return point

    def gate_wait_point(self) -> str:
        return self.gate_wait_point_out if self.carrying_load else self.gate_wait_point_return

    def gate_clear_point(self) -> str:
        return self.gate_clear_point_out if self.carrying_load else self.gate_clear_point_return

    def gate_drive_yaw(self) -> float:
        return -math.pi / 2.0 if self.carrying_load else math.pi / 2.0

    def after_gate_clear_state(self) -> MissionState:
        return MissionState.NAV_TO_DROPOFF_PREPOINT if self.carrying_load else MissionState.RETURN_TO_START

    def send_nav_goal(self, point_name: str) -> None:
        if point_name not in self.points:
            self.fail(f"missing point {point_name}")
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.publish_status("Waiting for /navigate_to_pose action server.")
            return

        pose = self.pose_from_point(point_name)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_goal_pending = True
        self.nav_result_status = None
        self.nav_target = point_name
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_response_cb)
        self.publish_status(f"Sent Nav2 goal {point_name}.")

    def nav_goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.nav_goal_pending = False
            self.fail(f"Nav2 goal response failed: {exc}")
            return

        self.nav_goal_pending = False
        if goal_handle is None or not goal_handle.accepted:
            self.fail("Nav2 rejected goal")
            return

        self.active_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future) -> None:
        try:
            result = future.result()
        except Exception:
            self.nav_result_status = GoalStatus.STATUS_UNKNOWN
            return

        if result is None:
            self.nav_result_status = GoalStatus.STATUS_UNKNOWN
        else:
            self.nav_result_status = result.status

    def cancel_nav_goal(self) -> None:
        if self.active_goal is not None:
            try:
                self.active_goal.cancel_goal_async()
            except Exception:
                pass
        self.clear_nav_tracking()

    def clear_nav_tracking(self) -> None:
        self.active_goal = None
        self.nav_goal_pending = False
        self.nav_result_status = None
        self.nav_target = ""

    def should_retry_nav_goal(self, point_name: str, status: int) -> bool:
        if point_name != self.pass_gate_point:
            return False
        if self.pass_gate_nav_retries <= 0:
            return False

        used = self.nav_retry_counts.get(point_name, 0)
        if used >= self.pass_gate_nav_retries:
            return False

        self.nav_retry_counts[point_name] = used + 1
        self.get_logger().warn(
            f"Nav2 goal {point_name} failed with status {status}; "
            f"retry {used + 1}/{self.pass_gate_nav_retries}"
        )
        return True

    def pose_from_point(self, point_name: str) -> PoseStamped:
        point = self.points[point_name]
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point["x"]
        pose.pose.position.y = point["y"]
        pose.pose.position.z = point.get("z", 0.0)
        pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(point["yaw"])
        return pose

    def transition(self, new_state: MissionState, reason: str) -> None:
        old_state = self.state
        self.state = new_state
        self.state_enter_time = self.now_sec()
        self.fork_command_sent = False
        self.motion_start_xy = None
        self.motion_start_yaw = None
        self.pickup_insertion_start_time = None
        if new_state not in (
            MissionState.LINE_FOLLOW_DROPOFF,
            MissionState.FINAL_ALIGN_DROPOFF,
        ):
            self.last_line_angular_cmd = 0.0
        if new_state not in (
            MissionState.LINE_OR_ODOM_APPROACH_PICKUP,
            MissionState.FINAL_ALIGN_PICKUP,
            MissionState.LINE_OR_ODOM_APPROACH_DROPOFF,
            MissionState.FINAL_ALIGN_DROPOFF,
        ):
            self.last_pickup_angular_cmd = 0.0
        if new_state not in (
            MissionState.BACK_OUT_SHORT,
            MissionState.BACK_OUT_DROPOFF,
        ):
            self.back_out_speed_cmd = 0.0
        if new_state != MissionState.NAV_TO_DROPOFF_PREPOINT:
            self.last_loaded_route_angular_cmd = 0.0
        if new_state == MissionState.BACK_OUT_SHORT:
            self.last_load_follow_verify_wall = -1.0
        if new_state in (
            MissionState.QR_PICKUP_CONFIRM,
            MissionState.QR_DROPOFF_CONFIRM,
        ):
            self.qr_scan_direction = 1.0
        if new_state == MissionState.RETURN_TO_START:
            self.return_route_index = (
                1
                if old_state == MissionState.GATE_CLEAR_AFTER
                and not self.carrying_load
                else 0
            )
        if new_state == MissionState.NAV_TO_DROPOFF_PREPOINT:
            self.loaded_dropoff_route_index = 0
        if new_state in (
            MissionState.WAIT_GATE_PERMISSION,
            MissionState.PASS_GATE_CENTER,
            MissionState.GATE_CLEAR_AFTER,
            MissionState.NAV_TO_DROPOFF_PREPOINT,
        ):
            self.last_loaded_route_angular_cmd = 0.0
        if new_state not in NAV_STATES:
            self.clear_nav_tracking()
        self.get_logger().info(f"{old_state.value} -> {new_state.value}: {reason}")
        self.publish_state()

    def fail(self, reason: str) -> None:
        self.error_reason = reason
        self.cancel_nav_goal()
        self.stop_robot()
        self.state = MissionState.ERROR
        self.state_enter_time = self.now_sec()
        self.get_logger().error(reason)
        self.publish_state()

    def line_is_fresh(self) -> bool:
        if not math.isfinite(self.line_offset) or not math.isfinite(self.line_angle):
            return False
        return self.now_sec() - self.last_line_time <= self.line_timeout_sec

    def load_frame_for_station(self, station: str) -> str:
        station_key = station.lower()
        try:
            return self.load_frame_template.format(station=station_key)
        except Exception:
            return f"pallet_load_{station_key}"

    def get_load_pose(self) -> Optional[Tuple[float, float, float]]:
        frame = self.load_frame_for_station(self.pickup)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.load_frame_parent,
                frame,
                Time(),
            )
        except TransformException:
            return self.get_load_pose_from_gazebo(frame)

        stamp = transform.header.stamp
        pose_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if pose_time > 0.0 and self.load_pose_max_age_sec > 0.0:
            if self.now_sec() - pose_time > self.load_pose_max_age_sec:
                return self.get_load_pose_from_gazebo(frame)

        translation = transform.transform.translation
        return (float(translation.x), float(translation.y), float(translation.z))

    def get_load_pose_from_gazebo(self, frame: str) -> Optional[Tuple[float, float, float]]:
        return self.get_gazebo_model_pose(frame)

    def sync_current_pose_from_gazebo(self, force: bool = False) -> bool:
        now_wall = time.monotonic()
        if (
            not force
            and self.last_robot_pose_sync_wall > 0.0
            and now_wall - self.last_robot_pose_sync_wall
            < self.robot_pose_sync_period
        ):
            return self.have_odom
        self.last_robot_pose_sync_wall = now_wall

        pose = self.get_gazebo_model_pose_with_yaw(self.robot_model_frame)
        if pose is None:
            return False

        world_x, world_y, _, world_yaw = pose
        gazebo_odom_x, gazebo_odom_y = self.gazebo_world_to_odom(world_x, world_y)
        gazebo_odom_yaw = normalize_angle(world_yaw - self.gazebo_world_origin_yaw)
        if self.have_raw_odom:
            self.odom_pose_correction_x = gazebo_odom_x - self.raw_current_x
            self.odom_pose_correction_y = gazebo_odom_y - self.raw_current_y
            self.odom_pose_correction_yaw = 0.0
            self.odom_pose_correction_active = True
            self.current_x = self.raw_current_x + self.odom_pose_correction_x
            self.current_y = self.raw_current_y + self.odom_pose_correction_y
            self.current_yaw = self.raw_current_yaw
        else:
            self.current_x = gazebo_odom_x
            self.current_y = gazebo_odom_y
            self.current_yaw = gazebo_odom_yaw
        self.have_odom = True
        return True

    def get_gazebo_model_pose_with_yaw(
        self,
        frame: str,
    ) -> Optional[Tuple[float, float, float, float]]:
        now = self.now_sec()
        if self.last_gazebo_pose_with_yaw is not None:
            cached_frame, cached_pose, cached_time = self.last_gazebo_pose_with_yaw
            if cached_frame == frame and now - cached_time < 0.25:
                return cached_pose

        pose = self.get_gazebo_model_pose_from_model_command(frame)
        if pose is not None:
            self.last_gazebo_pose_with_yaw = (frame, pose, now)
            self.last_gazebo_pose = (frame, pose[:3], now)
            return pose

        if not self.gazebo_pose_topic:
            return None

        try:
            result = subprocess.run(
                [
                    "gz",
                    "topic",
                    "-e",
                    "-t",
                    self.gazebo_pose_topic,
                    "-n",
                    "1",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=max(self.gazebo_pose_timeout_sec, 0.05),
            )
        except (OSError, subprocess.SubprocessError):
            return None

        if result.returncode != 0 and not result.stdout:
            return None

        pose = self.parse_gazebo_pose_text_with_yaw(result.stdout, frame)
        if pose is not None:
            self.last_gazebo_pose_with_yaw = (frame, pose, now)
            self.last_gazebo_pose = (frame, pose[:3], now)
        return pose

    def get_gazebo_model_pose(self, frame: str) -> Optional[Tuple[float, float, float]]:
        pose = self.get_gazebo_model_pose_with_yaw(frame)
        if pose is not None:
            return pose[:3]
        return None

    def get_gazebo_model_pose_from_model_command(
        self,
        frame: str,
    ) -> Optional[Tuple[float, float, float, float]]:
        try:
            result = subprocess.run(
                ["gz", "model", "-m", frame, "-p"],
                check=False,
                capture_output=True,
                text=True,
                timeout=max(self.gazebo_pose_timeout_sec + 0.5, 0.5),
            )
        except (OSError, subprocess.SubprocessError):
            return None

        if result.returncode != 0 and not result.stdout:
            return None

        return self.parse_gazebo_model_command_pose(result.stdout)

    @staticmethod
    def parse_gazebo_model_command_pose(
        text: str,
    ) -> Optional[Tuple[float, float, float, float]]:
        lines = text.splitlines()
        for index, raw_line in enumerate(lines):
            if "Pose [ XYZ" not in raw_line:
                continue

            arrays = []
            for pose_line in lines[index + 1 : index + 5]:
                match = re.search(r"\[([^\]]+)\]", pose_line)
                if not match:
                    continue
                try:
                    arrays.append([float(value) for value in match.group(1).split()])
                except ValueError:
                    return None
                if len(arrays) >= 2:
                    break

            if len(arrays) < 2 or len(arrays[0]) < 3 or len(arrays[1]) < 3:
                return None
            return (arrays[0][0], arrays[0][1], arrays[0][2], arrays[1][2])

        return None

    @staticmethod
    def parse_gazebo_pose_text(text: str, frame: str) -> Optional[Tuple[float, float, float]]:
        block = []
        depth = 0
        in_pose = False

        for raw_line in text.splitlines():
            stripped = raw_line.strip()
            if not in_pose and stripped == "pose {":
                in_pose = True
                depth = 1
                block = [raw_line]
                continue

            if not in_pose:
                continue

            block.append(raw_line)
            depth += raw_line.count("{")
            depth -= raw_line.count("}")
            if depth <= 0:
                pose = ForkliftMissionManager.parse_gazebo_pose_block(block, frame)
                if pose is not None:
                    return pose
                in_pose = False
                block = []

        return None

    @staticmethod
    def parse_gazebo_pose_block(
        block: list[str],
        frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        name = None
        x = 0.0
        y = 0.0
        z = 0.0
        in_position = False

        for raw_line in block:
            stripped = raw_line.strip()
            name_match = re.match(r'name:\s+"([^"]+)"', stripped)
            if name_match:
                name = name_match.group(1)
                continue

            if stripped == "position {":
                in_position = True
                continue
            if in_position and stripped == "}":
                in_position = False
                continue
            if not in_position or ":" not in stripped:
                continue

            key, value = stripped.split(":", 1)
            try:
                numeric_value = float(value.strip())
            except ValueError:
                continue
            if key == "x":
                x = numeric_value
            elif key == "y":
                y = numeric_value
            elif key == "z":
                z = numeric_value

        if name != frame:
            return None
        return (x, y, z)

    @staticmethod
    def parse_gazebo_pose_text_with_yaw(
        text: str,
        frame: str,
    ) -> Optional[Tuple[float, float, float, float]]:
        block = []
        depth = 0
        in_pose = False

        for raw_line in text.splitlines():
            stripped = raw_line.strip()
            if not in_pose and stripped == "pose {":
                in_pose = True
                depth = 1
                block = [raw_line]
                continue

            if not in_pose:
                continue

            block.append(raw_line)
            depth += raw_line.count("{")
            depth -= raw_line.count("}")
            if depth <= 0:
                pose = ForkliftMissionManager.parse_gazebo_pose_block_with_yaw(
                    block,
                    frame,
                )
                if pose is not None:
                    return pose
                in_pose = False
                block = []

        return None

    @staticmethod
    def parse_gazebo_pose_block_with_yaw(
        block: list[str],
        frame: str,
    ) -> Optional[Tuple[float, float, float, float]]:
        name = None
        x = 0.0
        y = 0.0
        z = 0.0
        qx = 0.0
        qy = 0.0
        qz = 0.0
        qw = 1.0
        section = ""

        for raw_line in block:
            stripped = raw_line.strip()
            name_match = re.match(r'name:\s+"([^"]+)"', stripped)
            if name_match:
                name = name_match.group(1)
                continue

            if stripped == "position {":
                section = "position"
                continue
            if stripped == "orientation {":
                section = "orientation"
                continue
            if stripped == "}":
                section = ""
                continue
            if not section or ":" not in stripped:
                continue

            key, value = stripped.split(":", 1)
            try:
                numeric_value = float(value.strip())
            except ValueError:
                continue
            if section == "position":
                if key == "x":
                    x = numeric_value
                elif key == "y":
                    y = numeric_value
                elif key == "z":
                    z = numeric_value
            elif section == "orientation":
                if key == "x":
                    qx = numeric_value
                elif key == "y":
                    qy = numeric_value
                elif key == "z":
                    qz = numeric_value
                elif key == "w":
                    qw = numeric_value

        if name != frame:
            return None
        return (x, y, z, quaternion_to_yaw(qx, qy, qz, qw))

    def angular_correction(self) -> float:
        line_offset = self.apply_deadband(self.line_offset, self.line_offset_deadband)
        heading_error = self.apply_deadband(
            self.line_heading_error(),
            self.line_heading_deadband,
        )
        correction = (
            -self.angular_offset_gain * line_offset
            -self.angular_angle_gain * heading_error
        )
        correction = max(min(correction, 0.30), -0.30)
        correction = self.smooth_command(
            correction,
            self.last_line_angular_cmd,
            self.line_angular_smoothing_alpha,
            self.line_max_angular_step,
        )
        self.last_line_angular_cmd = correction
        return correction

    def pickup_odom_angular_correction(self, lateral: float, yaw_error: float) -> float:
        lateral = self.apply_deadband(lateral, self.pickup_align_lateral_deadband)
        yaw_error = self.apply_deadband(yaw_error, self.pickup_align_yaw_deadband)
        correction = (
            self.pickup_align_lateral_gain * lateral
            - self.pickup_align_yaw_gain * yaw_error
        )
        correction = max(
            min(correction, self.pickup_align_max_angular),
            -self.pickup_align_max_angular,
        )
        correction = self.apply_deadband(
            correction,
            self.pickup_align_angular_deadband,
        )
        correction = self.smooth_command(
            correction,
            self.last_pickup_angular_cmd,
            self.pickup_align_smoothing_alpha,
            self.pickup_align_max_angular_step,
        )
        self.last_pickup_angular_cmd = correction
        return correction

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        return 0.0 if abs(value) < max(deadband, 0.0) else value

    @staticmethod
    def smooth_command(
        target: float,
        previous: float,
        alpha: float,
        max_step: float,
    ) -> float:
        alpha = max(0.0, min(alpha, 1.0))
        smoothed = previous + alpha * (target - previous)
        step_limit = max(max_step, 0.0)
        if step_limit > 0.0:
            delta = max(min(smoothed - previous, step_limit), -step_limit)
            smoothed = previous + delta
        return smoothed

    def line_heading_error(self) -> float:
        if not math.isfinite(self.line_angle):
            return math.nan
        heading = normalize_angle(self.line_angle)
        if heading > math.pi * 0.5:
            heading -= math.pi
        elif heading < -math.pi * 0.5:
            heading += math.pi
        return heading

    def can_acquire_visible_line(self, line_heading: float) -> bool:
        return (
            math.isfinite(self.line_offset)
            and math.isfinite(line_heading)
            and abs(self.line_offset) <= self.line_acquire_offset_tolerance
            and abs(line_heading) <= self.line_angle_tolerance
        )

    def target_relative_to_robot(self, point_name: str) -> Optional[Tuple[float, float, float]]:
        if point_name not in self.points:
            return None
        target = self.points[point_name]
        return self.point_relative_to_robot(target["x"], target["y"], target["yaw"])

    def point_relative_to_robot(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
    ) -> Tuple[float, float, float]:
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        forward = cos_yaw * dx + sin_yaw * dy
        lateral = -sin_yaw * dx + cos_yaw * dy
        yaw_error = normalize_angle(self.current_yaw - target_yaw)
        return forward, lateral, yaw_error

    def pickup_target_relative_to_robot(self) -> Optional[Tuple[float, float, float, str]]:
        target_name = self.pickup_target_point(self.pickup)
        if target_name not in self.points:
            return None

        target = self.points[target_name]
        target_x = target["x"]
        target_y = target["y"]
        target_yaw = target["yaw"]
        target_detail = f"target={target_name}"

        load_pose = self.get_load_pose()
        if load_pose is not None:
            target_x, target_y = self.gazebo_world_to_odom(load_pose[0], load_pose[1])
            target_detail = (
                f"target={target_name}(gazebo) "
                f"odom=({target_x:.3f},{target_y:.3f})"
            )

        forward, lateral, yaw_error = self.point_relative_to_robot(
            target_x,
            target_y,
            target_yaw,
        )
        return forward, lateral, yaw_error, target_detail

    def gazebo_world_to_odom(self, world_x: float, world_y: float) -> Tuple[float, float]:
        dx = world_x - self.gazebo_world_origin_x
        dy = world_y - self.gazebo_world_origin_y
        cos_yaw = math.cos(self.gazebo_world_origin_yaw)
        sin_yaw = math.sin(self.gazebo_world_origin_yaw)
        odom_x = cos_yaw * dx + sin_yaw * dy
        odom_y = -sin_yaw * dx + cos_yaw * dy
        return odom_x, odom_y

    def odom_to_gazebo_world(self, odom_x: float, odom_y: float) -> Tuple[float, float]:
        cos_yaw = math.cos(self.gazebo_world_origin_yaw)
        sin_yaw = math.sin(self.gazebo_world_origin_yaw)
        world_x = self.gazebo_world_origin_x + cos_yaw * odom_x - sin_yaw * odom_y
        world_y = self.gazebo_world_origin_y + sin_yaw * odom_x + cos_yaw * odom_y
        return world_x, world_y

    def capture_carried_load_offset(self) -> None:
        if not self.enable_carried_load_pose_lock or not self.have_odom:
            return
        if self.load_pose_after_lift is None:
            return

        load_odom_x, load_odom_y = self.gazebo_world_to_odom(
            self.load_pose_after_lift[0],
            self.load_pose_after_lift[1],
        )
        dx = load_odom_x - self.current_x
        dy = load_odom_y - self.current_y
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        forward = cos_yaw * dx + sin_yaw * dy
        lateral = -sin_yaw * dx + cos_yaw * dy
        self.carried_load_offset = (forward, lateral, self.load_pose_after_lift[2])
        self.last_carried_load_pose_update = -1.0

    def update_carried_load_pose(self, force: bool = False) -> None:
        if (
            not self.enable_carried_load_pose_lock
            or not self.carrying_load
            or self.carried_load_offset is None
            or not self.have_odom
        ):
            return

        now = self.now_sec()
        if (
            not force
            and self.last_carried_load_pose_update > 0.0
            and now - self.last_carried_load_pose_update
            < self.carried_load_pose_update_period
        ):
            return

        forward, lateral, z = self.carried_load_offset
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        target_odom_x = self.current_x + cos_yaw * forward - sin_yaw * lateral
        target_odom_y = self.current_y + sin_yaw * forward + cos_yaw * lateral
        world_x, world_y = self.odom_to_gazebo_world(target_odom_x, target_odom_y)
        if self.set_gazebo_model_pose(
            self.load_frame_for_station(self.pickup),
            world_x,
            world_y,
            z,
            normalize_angle(self.current_yaw + self.gazebo_world_origin_yaw),
            wait=force,
        ):
            self.last_carried_load_pose_update = now

    def set_gazebo_model_pose(
        self,
        frame: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        wait: bool = True,
    ) -> bool:
        if not self.gazebo_set_pose_service:
            return False
        quat_z, quat_w = yaw_to_quaternion(yaw)
        request = (
            f'name: "{frame}" '
            f"position {{x: {x:.6f} y: {y:.6f} z: {z:.6f}}} "
            f"orientation {{z: {quat_z:.8f} w: {quat_w:.8f}}}"
        )
        command = [
            "gz",
            "service",
            "-s",
            self.gazebo_set_pose_service,
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(int(max(self.gazebo_pose_timeout_sec, 0.2) * 1000)),
            "--req",
            request,
        ]
        if not wait:
            if self.carried_load_pose_process is not None:
                if self.carried_load_pose_process.poll() is None:
                    return False
                self.carried_load_pose_process = None
            try:
                self.carried_load_pose_process = subprocess.Popen(
                    command,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except OSError:
                return False
            self.last_gazebo_pose = None
            self.last_gazebo_pose_with_yaw = None
            return True

        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=max(self.gazebo_pose_timeout_sec + 0.2, 0.3),
            )
        except (OSError, subprocess.SubprocessError):
            return False

        self.last_gazebo_pose = None
        self.last_gazebo_pose_with_yaw = None
        return result.returncode == 0

    def verify_pickup_geometry(self) -> Tuple[bool, str]:
        if not self.have_odom:
            return False, "no odometry for pickup verification"

        target_name = self.pickup_target_point(self.pickup)
        relative = self.pickup_target_relative_to_robot()
        if relative is None:
            return False, f"missing pickup target point {target_name}"
        forward, lateral, yaw_error, target_detail = relative

        forward_ok = self.pickup_verify_forward_min <= forward <= self.pickup_verify_forward_max
        lateral_ok = abs(lateral) <= self.pickup_verify_lateral_tolerance
        yaw_ok = abs(yaw_error) <= self.pickup_verify_yaw_tolerance
        detail = (
            f"{target_detail} forward={forward:.3f} "
            f"range=[{self.pickup_verify_forward_min:.3f},{self.pickup_verify_forward_max:.3f}] "
            f"lateral={lateral:.3f}/{self.pickup_verify_lateral_tolerance:.3f} "
            f"yaw_error={yaw_error:.3f}/{self.pickup_verify_yaw_tolerance:.3f}"
        )
        if not forward_ok:
            return False, f"fork insertion depth invalid: {detail}"
        if not lateral_ok:
            return False, f"forks are not centered on pallet entry: {detail}"
        if not yaw_ok:
            return False, f"forks are not straight for pallet entry: {detail}"
        return True, detail

    def verify_load_lifted(self) -> Tuple[bool, str]:
        if self.load_pose_before_lift is None:
            return False, "load pose before lift missing"
        load_pose = self.get_load_pose()
        if load_pose is None:
            return False, "load pose unavailable"

        delta_z = load_pose[2] - self.load_pose_before_lift[2]
        if delta_z < self.load_lift_min_delta:
            return (
                False,
                f"load z delta {delta_z:.3f} below {self.load_lift_min_delta:.3f}",
            )
        return True, f"load z delta {delta_z:.3f}"

    def verify_load_followed_robot(self) -> Tuple[bool, str]:
        if self.load_pose_after_lift is None:
            return False, "load pose after lift missing"

        load_pose = self.get_load_pose()
        if load_pose is None:
            return False, "load pose unavailable after backout"

        load_delta_xy = math.hypot(
            load_pose[0] - self.load_pose_after_lift[0],
            load_pose[1] - self.load_pose_after_lift[1],
        )
        load_delta_z = load_pose[2] - self.load_pose_before_lift[2]
        if load_delta_z < self.load_lift_min_delta:
            return (
                False,
                f"load z delta {load_delta_z:.3f} below {self.load_lift_min_delta:.3f}",
            )

        robot_pose = self.get_gazebo_model_pose(self.robot_model_frame)
        if self.robot_pose_after_lift is None or robot_pose is None:
            robot_delta_xy = self.distance_from_motion_start()
            robot_detail = f"odom_backout_delta={robot_delta_xy:.3f}"
        else:
            robot_delta_xy = math.hypot(
                robot_pose[0] - self.robot_pose_after_lift[0],
                robot_pose[1] - self.robot_pose_after_lift[1],
            )
            robot_detail = f"robot_model_delta={robot_delta_xy:.3f}"

        if robot_delta_xy < self.load_follow_min_delta:
            return (
                False,
                f"{robot_detail} below {self.load_follow_min_delta:.3f}; "
                "robot did not physically back out with load",
            )

        required_load_delta = max(
            self.load_follow_min_delta,
            robot_delta_xy * self.load_follow_robot_ratio,
        )
        if load_delta_xy < required_load_delta:
            return (
                False,
                f"load_xy_delta={load_delta_xy:.3f} below {required_load_delta:.3f}; "
                f"{robot_detail}",
            )

        return (
            True,
            f"load_xy_delta={load_delta_xy:.3f}, load_z_delta={load_delta_z:.3f}, "
            f"{robot_detail}",
        )

    def dropoff_target_relative_to_load(
        self,
    ) -> Optional[Tuple[float, float, float, str]]:
        target_name = self.dropoff_target_point(self.dropoff)
        if target_name not in self.points:
            return None

        target = self.points[target_name]
        if self.carried_load_offset is not None and self.have_odom:
            forward_offset, lateral_offset, _ = self.carried_load_offset
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            load_odom_x = (
                self.current_x
                + cos_yaw * forward_offset
                - sin_yaw * lateral_offset
            )
            load_odom_y = (
                self.current_y
                + sin_yaw * forward_offset
                + cos_yaw * lateral_offset
            )
            target_detail = (
                f"target={target_name} load_est=({load_odom_x:.3f},{load_odom_y:.3f})"
            )
        else:
            load_pose = self.get_load_pose()
            if load_pose is None:
                return None
            load_odom_x, load_odom_y = self.gazebo_world_to_odom(
                load_pose[0],
                load_pose[1],
            )
            target_detail = (
                f"target={target_name} load_odom=({load_odom_x:.3f},{load_odom_y:.3f})"
            )

        dx = target["x"] - load_odom_x
        dy = target["y"] - load_odom_y
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        forward = cos_yaw * dx + sin_yaw * dy
        lateral = -sin_yaw * dx + cos_yaw * dy
        yaw_error = normalize_angle(self.current_yaw - target["yaw"])
        return forward, lateral, yaw_error, target_detail

    def verify_dropoff_alignment(self) -> Tuple[bool, str]:
        if not self.have_odom:
            return False, "no odometry for dropoff verification"

        target_name = self.dropoff_target_point(self.dropoff)
        relative = self.dropoff_target_relative_to_load()
        if relative is None:
            return False, f"load pose unavailable for dropoff target {target_name}"
        forward, lateral, yaw_error, target_detail = relative

        forward_ok = abs(forward) <= self.dropoff_verify_forward_tolerance
        lateral_ok = abs(lateral) <= self.dropoff_verify_lateral_tolerance
        yaw_ok = abs(yaw_error) <= self.dropoff_verify_yaw_tolerance
        detail = (
            f"{target_detail} forward={forward:.3f}/"
            f"{self.dropoff_verify_forward_tolerance:.3f} "
            f"lateral={lateral:.3f}/{self.dropoff_verify_lateral_tolerance:.3f} "
            f"yaw_error={yaw_error:.3f}/{self.dropoff_verify_yaw_tolerance:.3f}"
        )
        if not forward_ok:
            return False, f"load is not at dropoff depth: {detail}"
        if not lateral_ok:
            return False, f"load is not centered on dropoff zone: {detail}"
        if not yaw_ok:
            return False, f"robot is not straight for dropoff: {detail}"
        return True, detail

    def verify_dropoff_release(self) -> Tuple[bool, str]:
        if self.drop_load_pose_before_lower is None:
            return False, "load pose before lower missing"

        load_pose = self.get_load_pose()
        if load_pose is None:
            return False, "load pose unavailable after lower"

        target_name = self.dropoff_target_point(self.dropoff)
        if target_name not in self.points:
            return False, f"missing dropoff target point {target_name}"
        target = self.points[target_name]
        load_odom_x, load_odom_y = self.gazebo_world_to_odom(load_pose[0], load_pose[1])
        xy_error = math.hypot(target["x"] - load_odom_x, target["y"] - load_odom_y)
        z_drop = self.drop_load_pose_before_lower[2] - load_pose[2]
        fork_lowered = (
            math.isfinite(self.fork_height)
            and self.fork_height <= self.fork_lower_success_height
        )
        load_lowered = (
            z_drop >= self.dropoff_release_min_z_drop
            or load_pose[2] <= self.dropoff_release_max_z
        )

        detail = (
            f"target={target_name} load_odom=({load_odom_x:.3f},{load_odom_y:.3f}) "
            f"xy_error={xy_error:.3f}/{self.dropoff_release_xy_tolerance:.3f} "
            f"z_drop={z_drop:.3f}/{self.dropoff_release_min_z_drop:.3f} "
            f"load_z={load_pose[2]:.3f}/{self.dropoff_release_max_z:.3f} "
            f"fork_height={self.fork_height_text()}/"
            f"{self.fork_lower_success_height:.3f}"
        )
        if not fork_lowered:
            return False, f"forks not lowered: {detail}"
        if not load_lowered:
            return False, f"load did not lower onto drop zone: {detail}"
        if xy_error > self.dropoff_release_xy_tolerance:
            return False, f"released load is outside drop zone tolerance: {detail}"
        return True, detail

    def fork_lift_reached(self) -> bool:
        return (
            math.isfinite(self.fork_height)
            and self.fork_height >= self.fork_lift_success_height
        )

    def fork_height_text(self) -> str:
        if math.isfinite(self.fork_height):
            return f"{self.fork_height:.3f}"
        return "unknown"

    def has_recent_qr(self, expected_qr: str) -> bool:
        seen_time = self.qr_seen_times.get(expected_qr, -1.0)
        return seen_time > 0.0 and self.now_sec() - seen_time <= self.qr_timeout_sec

    def reset_qr_buffer(self, expected_qr: str = "") -> None:
        if expected_qr:
            self.qr_seen_times[expected_qr] = -1.0
        elif self.last_qr:
            self.qr_seen_times[self.last_qr] = -1.0
        self.last_qr = ""
        self.last_qr_time = -1.0

    def distance_from_motion_start(self) -> float:
        if self.motion_start_xy is None:
            return 0.0
        start_x, start_y = self.motion_start_xy
        return math.hypot(self.current_x - start_x, self.current_y - start_y)

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_fork_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.fork_cmd_pub.publish(msg)
        self.publish_status(f"Published fork command {command}.")

    def publish_pickup_success(self, success: bool) -> None:
        msg = Bool()
        msg.data = success
        self.pickup_success_pub.publish(msg)

    def publish_state(self) -> None:
        msg = String()
        suffix = f" pickup={self.pickup} dropoff={self.dropoff}"
        if self.state == MissionState.ERROR:
            suffix += f" error={self.error_reason}"
        msg.data = self.state.value + suffix
        self.state_pub.publish(msg)

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def qr_cb(self, msg: String) -> None:
        qr_text = msg.data.strip().upper()
        if not qr_text:
            return

        expected = self.expected_qr_for_state()
        if expected is None:
            return

        self.last_qr = qr_text
        self.last_qr_time = self.now_sec()
        self.qr_seen_times[qr_text] = self.last_qr_time
        if self.is_wrong_precision_qr(qr_text):
            return

        if expected == qr_text:
            self.publish_status(f"QR {qr_text} accepted in {self.state.value}.")
        else:
            self.publish_status(f"Ignoring QR {qr_text} in {self.state.value}.")

    def line_offset_cb(self, msg: Float32) -> None:
        self.line_offset = float(msg.data)
        self.last_line_time = self.now_sec()

    def line_angle_cb(self, msg: Float32) -> None:
        self.line_angle = float(msg.data)
        self.last_line_time = self.now_sec()

    def fork_state_cb(self, msg: String) -> None:
        self.fork_state_text = msg.data
        words = msg.data.strip().split()
        self.fork_mode = words[0].upper() if words else ""
        height_match = re.search(r"height=([0-9.+-]+)", msg.data)
        if height_match:
            try:
                self.fork_height = float(height_match.group(1))
            except ValueError:
                self.fork_height = math.nan

    def safety_cb(self, msg: Bool) -> None:
        self.safety_stop = bool(msg.data)

    def odom_cb(self, msg: Odometry) -> None:
        self.raw_current_x = float(msg.pose.pose.position.x)
        self.raw_current_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.raw_current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.have_raw_odom = True
        if self.odom_pose_correction_active:
            self.current_x = self.raw_current_x + self.odom_pose_correction_x
            self.current_y = self.raw_current_y + self.odom_pose_correction_y
            self.current_yaw = normalize_angle(
                self.raw_current_yaw + self.odom_pose_correction_yaw
            )
        else:
            self.current_x = self.raw_current_x
            self.current_y = self.raw_current_y
            self.current_yaw = self.raw_current_yaw
        self.have_odom = True

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def expected_qr_for_state(self) -> Optional[str]:
        if self.state in (
            MissionState.NAV_TO_PICKUP_PREPOINT,
            MissionState.QR_PICKUP_CONFIRM,
        ):
            return self.pickup
        if self.state in (
            MissionState.NAV_TO_DROPOFF_PREPOINT,
            MissionState.QR_DROPOFF_CONFIRM,
        ):
            return self.dropoff
        return None

    def is_wrong_precision_qr(self, qr_text: str) -> bool:
        if self.state in (
            MissionState.LINE_OR_ODOM_APPROACH_PICKUP,
            MissionState.FINAL_ALIGN_PICKUP,
            MissionState.VERIFY_FORK_INSERTION,
            MissionState.PICK_LOAD,
            MissionState.VERIFY_PICKUP_SUCCESS,
        ):
            if qr_text in PICKUP_STATIONS and qr_text != self.pickup:
                self.fail(
                    f"Wrong pickup QR {qr_text} visible during {self.state.value}; "
                    f"expected {self.pickup}."
                )
                return True

        if self.state in (
            MissionState.TURN_180_TO_DROPOFF,
            MissionState.LINE_OR_ODOM_APPROACH_DROPOFF,
            MissionState.LINE_FOLLOW_DROPOFF,
            MissionState.FINAL_ALIGN_DROPOFF,
            MissionState.DROP_LOAD,
            MissionState.VERIFY_DROPOFF_RELEASE,
        ):
            if qr_text in DROPOFF_STATIONS and qr_text != self.dropoff:
                self.fail(
                    f"Wrong dropoff QR {qr_text} visible during {self.state.value}; "
                    f"expected {self.dropoff}."
                )
                return True

        return False

    @staticmethod
    def pickup_prepoint(station: str) -> str:
        return PICKUP_PREPOINTS.get(station, station)

    @staticmethod
    def dropoff_prepoint(station: str) -> str:
        return DROPOFF_PREPOINTS.get(station, station)

    @staticmethod
    def pickup_target_point(station: str) -> str:
        return f"pallet_{station}"

    @staticmethod
    def dropoff_target_point(station: str) -> str:
        return f"drop_{station}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForkliftMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.cancel_nav_goal()
                node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
