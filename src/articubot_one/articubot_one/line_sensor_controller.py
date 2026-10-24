#!/usr/bin/env python3

from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class LineSensorController(Node):
    def __init__(self) -> None:
        super().__init__("line_sensor_controller")
        self.declare_parameter("enabled", False)
        self.declare_parameter("mode", "OFF")
        self.declare_parameter("linear_speed", 0.08)
        self.declare_parameter("max_angular_speed", 0.9)
        self.declare_parameter("kp", 0.015)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.004)
        self.declare_parameter("heading_kp", 0.35)
        self.declare_parameter("lookahead_weight", 0.55)
        self.declare_parameter("position_scale", 30.0)
        self.declare_parameter("heading_capture_threshold_rad", 0.35)
        self.declare_parameter("heading_capture_lateral_weight", 0.25)
        self.declare_parameter("derivative_filter_alpha", 0.25)
        self.declare_parameter("turn_slowdown_position", 10.0)
        self.declare_parameter("turn_slowdown_angle_rad", 0.30)
        self.declare_parameter("turn_hold_position", 18.0)
        self.declare_parameter("turn_hold_angle_rad", 0.65)
        self.declare_parameter("min_turn_linear_scale", 0.18)
        self.declare_parameter("dock_linear_speed", 0.03)
        self.declare_parameter("dock_max_angular_speed", 0.45)
        self.declare_parameter("dock_kp", 0.020)
        self.declare_parameter("dock_ki", 0.0)
        self.declare_parameter("dock_kd", 0.005)
        self.declare_parameter("dock_position_deadband", 1.0)
        self.declare_parameter("dock_alignment_hold_threshold", 6.0)
        self.declare_parameter("dock_centered_slowdown_threshold", 12.0)
        self.declare_parameter("integral_limit", 200.0)
        self.declare_parameter("position_deadband", 2.0)
        self.declare_parameter("stop_on_line_lost", True)
        self.declare_parameter("seen_timeout_sec", 0.30)

        self.create_subscription(Float32, "/line_sensor/position", self.position_cb, 20)
        self.create_subscription(Bool, "/line_sensor/seen", self.seen_cb, 20)
        self.create_subscription(String, "/line_sensor/status", self.status_cb, 20)
        self.create_subscription(Bool, "/line_sensor/enable", self.enable_cb, 10)
        self.create_subscription(String, "/line_sensor/mode", self.mode_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_line_sensor", 10)
        self.status_pub = self.create_publisher(String, "/line_sensor/controller_status", 10)

        self.enabled = self.get_parameter("enabled").get_parameter_value().bool_value
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.last_position = 0.0
        self.last_seen = False
        self.last_seen_time = 0.0
        self.last_angle = 0.0
        self.last_lookahead_position = 0.0
        self.last_cross_track_m: float | None = None
        self.last_source_mode = ""
        self.last_segment = ""
        self.last_requested_segment = ""
        self.last_sensor_status_time = 0.0
        self.last_error = 0.0
        self.filtered_derivative = 0.0
        self.integral = 0.0
        self.last_control_time = time.monotonic()
        self.last_status_time = 0.0
        self.last_cmd_active = False

        self.timer = self.create_timer(0.05, self.control_step)

    def enable_cb(self, msg: Bool) -> None:
        if self.enabled != bool(msg.data):
            self.integral = 0.0
            self.last_error = 0.0
            self.filtered_derivative = 0.0
        self.enabled = bool(msg.data)

    def mode_cb(self, msg: String) -> None:
        requested_mode = msg.data.strip().upper() or "LINE_FOLLOW"
        if requested_mode != self.mode:
            self.integral = 0.0
            self.last_error = 0.0
            self.filtered_derivative = 0.0
        self.mode = requested_mode

    def position_cb(self, msg: Float32) -> None:
        self.last_position = float(msg.data)

    def seen_cb(self, msg: Bool) -> None:
        self.last_seen = bool(msg.data)
        if self.last_seen:
            self.last_seen_time = time.monotonic()

    def status_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.last_sensor_status_time = time.monotonic()
        self.last_source_mode = str(payload.get("source_mode", ""))
        self.last_segment = str(payload.get("segment", ""))
        self.last_requested_segment = str(payload.get("requested_segment", ""))
        cross_track = payload.get("cross_track_m")
        self.last_cross_track_m = float(cross_track) if isinstance(cross_track, (int, float)) else None
        angle = payload.get("angle_rad")
        self.last_angle = float(angle) if isinstance(angle, (int, float)) else 0.0
        lookahead_offset = payload.get("lookahead_offset")
        if isinstance(lookahead_offset, (int, float)):
            scale = float(self.get_parameter("position_scale").value)
            self.last_lookahead_position = float(lookahead_offset) * scale
        else:
            self.last_lookahead_position = self.last_position

    def publish_status(
        self,
        angular_cmd: float,
        linear_cmd: float,
        line_seen: bool,
        *,
        alignment_locked: bool = False,
        linear_scale: float = 0.0,
        control_state: str = "STOPPED",
        control_error: float | None = None,
    ) -> None:
        now = time.monotonic()
        if now - self.last_status_time < 0.1:
            return
        msg = String()
        msg.data = json.dumps(
            {
                "enabled": self.enabled,
                "mode": self.mode,
                "seen": line_seen,
                "position": self.last_position,
                "lookahead_position": self.last_lookahead_position,
                "angle_rad": self.last_angle,
                "cross_track_m": self.last_cross_track_m,
                "source_mode": self.last_source_mode,
                "segment": self.last_segment,
                "requested_segment": self.last_requested_segment,
                "error": -self.last_position,
                "control_error": control_error,
                "integral": self.integral,
                "angular_cmd": angular_cmd,
                "linear_cmd": linear_cmd,
                "alignment_locked": alignment_locked,
                "linear_scale": linear_scale,
                "control_state": control_state,
                "position_deadband": float(self.get_parameter("position_deadband").value),
            },
            ensure_ascii=True,
        )
        self.status_pub.publish(msg)
        self.last_status_time = now

    def stop_robot(self) -> None:
        try:
            self.cmd_pub.publish(Twist())
            self.last_cmd_active = False
        except Exception:
            pass

    def control_step(self) -> None:
        now = time.monotonic()
        dt = max(now - self.last_control_time, 1e-3)
        self.last_control_time = now

        if not self.enabled or self.mode == "OFF":
            self.integral = 0.0
            if self.last_cmd_active:
                self.stop_robot()
            self.publish_status(0.0, 0.0, False, control_state="OFF")
            return

        line_seen = self.last_seen and (now - self.last_seen_time) <= self.get_parameter("seen_timeout_sec").value
        if not line_seen:
            self.integral = 0.0
            if self.get_parameter("stop_on_line_lost").value:
                self.stop_robot()
            self.publish_status(0.0, 0.0, False, control_state="LINE_LOST_STOP")
            return

        docking_mode = self.mode == "DOCKING"
        deadband = float(
            self.get_parameter("dock_position_deadband" if docking_mode else "position_deadband").value
        )
        error = -self.last_position
        lookahead_weight = 0.0 if docking_mode else float(self.get_parameter("lookahead_weight").value)
        if lookahead_weight > 0.0:
            error = (1.0 - lookahead_weight) * error + lookahead_weight * (-self.last_lookahead_position)
        if abs(error) < deadband:
            error = 0.0

        self.integral += error * dt
        limit = float(self.get_parameter("integral_limit").value)
        self.integral = max(-limit, min(limit, self.integral))
        raw_derivative = (error - self.last_error) / dt
        self.last_error = error
        derivative_alpha = max(
            0.0,
            min(1.0, float(self.get_parameter("derivative_filter_alpha").value)),
        )
        self.filtered_derivative += derivative_alpha * (raw_derivative - self.filtered_derivative)

        kp = float(self.get_parameter("dock_kp" if docking_mode else "kp").value)
        ki = float(self.get_parameter("dock_ki" if docking_mode else "ki").value)
        kd = float(self.get_parameter("dock_kd" if docking_mode else "kd").value)
        lateral_angular = kp * error + ki * self.integral + kd * self.filtered_derivative
        heading_capture = bool(
            not docking_mode
            and abs(self.last_angle)
            >= float(self.get_parameter("heading_capture_threshold_rad").value)
        )
        angular = lateral_angular
        if not docking_mode:
            heading_angular = float(self.get_parameter("heading_kp").value) * self.last_angle
            if heading_capture:
                lateral_weight = max(
                    0.0,
                    min(1.0, float(self.get_parameter("heading_capture_lateral_weight").value)),
                )
                angular = lateral_weight * lateral_angular + heading_angular
            else:
                angular += heading_angular
        max_ang = float(
            self.get_parameter("dock_max_angular_speed" if docking_mode else "max_angular_speed").value
        )
        angular = max(-max_ang, min(max_ang, angular))

        linear = float(self.get_parameter("dock_linear_speed" if docking_mode else "linear_speed").value)
        alignment_locked = False
        linear_scale = 1.0
        if docking_mode:
            control_state = "DOCKING_FOLLOW"
            hold_threshold = float(self.get_parameter("dock_alignment_hold_threshold").value)
            slowdown_threshold = max(
                hold_threshold,
                float(self.get_parameter("dock_centered_slowdown_threshold").value),
            )
            abs_error = abs(error)
            if abs_error >= hold_threshold:
                linear = 0.0
                linear_scale = 0.0
                alignment_locked = True
                control_state = "DOCKING_ALIGN"
            elif slowdown_threshold > hold_threshold:
                linear_scale = max(
                    0.15,
                    min(1.0, (slowdown_threshold - abs_error) / max(slowdown_threshold - hold_threshold, 1e-3)),
                )
                linear *= linear_scale
        else:
            control_state = "FOLLOW_LINE"
            abs_error = abs(error)
            abs_angle = abs(self.last_angle)
            hold_position = float(self.get_parameter("turn_hold_position").value)
            hold_angle = float(self.get_parameter("turn_hold_angle_rad").value)
            if heading_capture:
                linear = 0.0
                linear_scale = 0.0
                alignment_locked = True
                control_state = "ALIGN_HEADING"
            elif abs_error >= hold_position or abs_angle >= hold_angle:
                linear = 0.0
                linear_scale = 0.0
                alignment_locked = True
                control_state = "CAPTURE_LINE"
            else:
                slowdown_position = max(
                    deadband,
                    float(self.get_parameter("turn_slowdown_position").value),
                )
                slowdown_angle = max(
                    1e-3,
                    float(self.get_parameter("turn_slowdown_angle_rad").value),
                )
                error_scale = 1.0
                if abs_error > deadband:
                    error_scale = max(
                        float(self.get_parameter("min_turn_linear_scale").value),
                        min(1.0, (slowdown_position - abs_error) / max(slowdown_position - deadband, 1e-3)),
                    )
                angle_scale = max(
                    float(self.get_parameter("min_turn_linear_scale").value),
                    min(1.0, (slowdown_angle - abs_angle) / slowdown_angle),
                )
                linear_scale = min(error_scale, angle_scale)
                linear *= linear_scale
                if linear_scale < 0.999:
                    control_state = "FOLLOW_LINE_SLOW"

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        self.last_cmd_active = True
        self.publish_status(
            angular,
            linear,
            True,
            alignment_locked=alignment_locked,
            linear_scale=linear_scale,
            control_state=control_state,
            control_error=error,
        )


def main() -> None:
    rclpy.init()
    node = LineSensorController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
