#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from articubot_one.line_detection import detect_line_with_mask


VALID_MODES = {"OFF", "ROUTE_CORRECTION", "LINE_PRIMARY_ROUTE", "DOCKING"}
VALID_PROFILES = {"route_line", "dock_line", "b_side_dock_line"}


class LineFollower(Node):
    def __init__(self) -> None:
        super().__init__("line_follower")

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("enable_topic", "/line_follower/enable")
        self.declare_parameter("mode_topic", "/line_follower/mode")
        self.declare_parameter("profile_topic", "/line_follower/profile")
        self.declare_parameter("context_topic", "/line_follower/context")
        self.declare_parameter("nav_cmd_topic", "/cmd_vel_nav_smoothed")
        self.declare_parameter("nav_cmd_fallback_topic", "/cmd_vel_nav")
        self.declare_parameter("line_cmd_topic", "/cmd_vel_line")
        self.declare_parameter("fused_cmd_topic", "/cmd_vel_route_fused")
        self.declare_parameter("status_topic", "/line_follower/status")
        self.declare_parameter("debug_image_topic", "/line_follower/debug_image")
        self.declare_parameter("camera_line_status_topic", "/camera_line/status")
        self.declare_parameter("camera_line_debug_image_topic", "/camera_line/debug_image")
        self.declare_parameter("camera_line_reverse", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("min_area", 250.0)
        self.declare_parameter("line_timeout_sec", 0.5)
        self.declare_parameter("nav_timeout_sec", 0.5)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("route_line_dark_threshold", 90)
        self.declare_parameter("route_line_hsv_lower", [35, 45, 45])
        self.declare_parameter("route_line_hsv_upper", [95, 255, 255])
        self.declare_parameter("dock_line_dark_threshold", 90)
        self.declare_parameter("dock_line_hsv_lower", [35, 45, 45])
        self.declare_parameter("dock_line_hsv_upper", [95, 255, 255])
        self.declare_parameter("b_side_dock_line_dark_threshold", 120)
        self.declare_parameter("b_side_dock_line_hsv_lower", [20, 35, 35])
        self.declare_parameter("b_side_dock_line_hsv_upper", [110, 255, 255])
        self.declare_parameter("route_offset_gain", 0.30)
        self.declare_parameter("route_angle_gain", 0.14)
        self.declare_parameter("route_max_angular_correction", 0.35)
        self.declare_parameter("route_max_total_angular", 1.20)
        self.declare_parameter("route_linear_scale_when_lost", 0.70)
        self.declare_parameter("route_linear_scale_when_detected", 1.00)
        self.declare_parameter("line_primary_linear_speed", 0.08)
        self.declare_parameter("line_primary_offset_gain", 1.05)
        self.declare_parameter("line_primary_angle_gain", 0.75)
        self.declare_parameter("line_primary_derivative_gain", 0.10)
        self.declare_parameter("line_primary_max_angular", 1.00)
        self.declare_parameter("centered_error_px_threshold", 25.0)
        self.declare_parameter("centered_heading_error_rad_threshold", 0.05)
        self.declare_parameter("centered_required_streak", 5)
        self.declare_parameter("centered_max_error_rate", 0.25)
        self.declare_parameter("line_primary_rotate_only_error_px", 90.0)
        self.declare_parameter("line_primary_follow_reenter_error_px", 45.0)
        self.declare_parameter("line_primary_acquire_linear_speed", 0.0)
        self.declare_parameter("line_primary_search_angular_speed", 0.40)
        self.declare_parameter("line_primary_search_timeout_sec", 3.0)
        self.declare_parameter("line_primary_error_stall_timeout_sec", 1.2)
        self.declare_parameter("line_primary_transient_line_lost_sec", 0.25)
        self.declare_parameter("line_primary_capture_good_frames", 4)
        self.declare_parameter("line_primary_follow_bad_frames", 6)
        self.declare_parameter("line_primary_aligned_angle_threshold_rad", 0.18)
        self.declare_parameter("line_primary_arc_capture_linear_speed", 0.025)
        self.declare_parameter("line_primary_reverse_speed", 0.035)
        self.declare_parameter("line_primary_reverse_duration_sec", 0.7)
        self.declare_parameter("line_primary_capture_min_angular", 0.55)
        self.declare_parameter("line_primary_capture_smoothing_alpha", 1.0)
        self.declare_parameter("line_primary_error_ema_alpha", 0.22)
        self.declare_parameter("line_primary_angle_ema_alpha", 0.20)
        self.declare_parameter("line_primary_loss_confirm_frames", 4)
        self.declare_parameter("line_primary_capture_enter_error_px", 85.0)
        self.declare_parameter("line_primary_follow_enter_error_px", 30.0)
        self.declare_parameter("line_primary_follow_exit_error_px", 70.0)
        self.declare_parameter("docking_linear_speed", 0.055)
        self.declare_parameter("docking_offset_gain", 0.50)
        self.declare_parameter("docking_angle_gain", 0.20)
        self.declare_parameter("docking_max_angular", 0.22)
        self.declare_parameter("angular_deadband", 0.015)
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter(
            "debug_save_root",
            str(Path.home() / ".local" / "state" / "tekno_robot_simulation"),
        )

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.enable_topic = str(self.get_parameter("enable_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.profile_topic = str(self.get_parameter("profile_topic").value)
        self.context_topic = str(self.get_parameter("context_topic").value)
        self.nav_cmd_topic = str(self.get_parameter("nav_cmd_topic").value)
        self.nav_cmd_fallback_topic = str(self.get_parameter("nav_cmd_fallback_topic").value)
        self.line_cmd_topic = str(self.get_parameter("line_cmd_topic").value)
        self.fused_cmd_topic = str(self.get_parameter("fused_cmd_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.camera_line_status_topic = str(self.get_parameter("camera_line_status_topic").value)
        self.camera_line_debug_image_topic = str(
            self.get_parameter("camera_line_debug_image_topic").value
        )
        self.camera_line_reverse = bool(self.get_parameter("camera_line_reverse").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.min_area = float(self.get_parameter("min_area").value)
        self.line_timeout_sec = float(self.get_parameter("line_timeout_sec").value)
        self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)
        self.route_offset_gain = float(self.get_parameter("route_offset_gain").value)
        self.route_angle_gain = float(self.get_parameter("route_angle_gain").value)
        self.route_max_angular_correction = abs(
            float(self.get_parameter("route_max_angular_correction").value)
        )
        self.route_max_total_angular = abs(
            float(self.get_parameter("route_max_total_angular").value)
        )
        self.route_linear_scale_when_lost = max(
            0.0, float(self.get_parameter("route_linear_scale_when_lost").value)
        )
        self.route_linear_scale_when_detected = max(
            0.0, float(self.get_parameter("route_linear_scale_when_detected").value)
        )
        self.line_primary_linear_speed = max(
            0.0, float(self.get_parameter("line_primary_linear_speed").value)
        )
        self.line_primary_offset_gain = float(self.get_parameter("line_primary_offset_gain").value)
        self.line_primary_angle_gain = float(self.get_parameter("line_primary_angle_gain").value)
        self.line_primary_derivative_gain = float(
            self.get_parameter("line_primary_derivative_gain").value
        )
        self.line_primary_max_angular = abs(
            float(self.get_parameter("line_primary_max_angular").value)
        )
        self.centered_error_px_threshold = abs(
            float(self.get_parameter("centered_error_px_threshold").value)
        )
        self.centered_heading_error_rad_threshold = abs(
            float(self.get_parameter("centered_heading_error_rad_threshold").value)
        )
        self.centered_required_streak = max(
            1, int(self.get_parameter("centered_required_streak").value)
        )
        self.centered_max_error_rate = abs(
            float(self.get_parameter("centered_max_error_rate").value)
        )
        self.line_primary_rotate_only_error_px = abs(
            float(self.get_parameter("line_primary_rotate_only_error_px").value)
        )
        self.line_primary_follow_reenter_error_px = abs(
            float(self.get_parameter("line_primary_follow_reenter_error_px").value)
        )
        self.line_primary_acquire_linear_speed = float(
            self.get_parameter("line_primary_acquire_linear_speed").value
        )
        self.line_primary_search_angular_speed = abs(
            float(self.get_parameter("line_primary_search_angular_speed").value)
        )
        self.line_primary_search_timeout_sec = max(
            0.1, float(self.get_parameter("line_primary_search_timeout_sec").value)
        )
        self.line_primary_error_stall_timeout_sec = max(
            0.1, float(self.get_parameter("line_primary_error_stall_timeout_sec").value)
        )
        self.line_primary_transient_line_lost_sec = max(
            0.05, float(self.get_parameter("line_primary_transient_line_lost_sec").value)
        )
        self.line_primary_capture_good_frames = max(
            1, int(self.get_parameter("line_primary_capture_good_frames").value)
        )
        self.line_primary_follow_bad_frames = max(
            1, int(self.get_parameter("line_primary_follow_bad_frames").value)
        )
        self.line_primary_aligned_angle_threshold_rad = abs(
            float(self.get_parameter("line_primary_aligned_angle_threshold_rad").value)
        )
        self.line_primary_arc_capture_linear_speed = max(
            0.0, float(self.get_parameter("line_primary_arc_capture_linear_speed").value)
        )
        self.line_primary_reverse_speed = abs(
            float(self.get_parameter("line_primary_reverse_speed").value)
        )
        self.line_primary_reverse_duration_sec = max(
            0.1, float(self.get_parameter("line_primary_reverse_duration_sec").value)
        )
        self.line_primary_capture_min_angular = abs(
            float(self.get_parameter("line_primary_capture_min_angular").value)
        )
        self.line_primary_capture_smoothing_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter("line_primary_capture_smoothing_alpha").value)),
        )
        self.line_primary_error_ema_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter("line_primary_error_ema_alpha").value)),
        )
        self.line_primary_angle_ema_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter("line_primary_angle_ema_alpha").value)),
        )
        self.line_primary_loss_confirm_frames = max(
            1, int(self.get_parameter("line_primary_loss_confirm_frames").value)
        )
        self.line_primary_capture_enter_error_px = abs(
            float(self.get_parameter("line_primary_capture_enter_error_px").value)
        )
        self.line_primary_follow_enter_error_px = abs(
            float(self.get_parameter("line_primary_follow_enter_error_px").value)
        )
        self.line_primary_follow_exit_error_px = abs(
            float(self.get_parameter("line_primary_follow_exit_error_px").value)
        )
        self.docking_linear_speed = float(self.get_parameter("docking_linear_speed").value)
        self.docking_offset_gain = float(self.get_parameter("docking_offset_gain").value)
        self.docking_angle_gain = float(self.get_parameter("docking_angle_gain").value)
        self.docking_max_angular = abs(float(self.get_parameter("docking_max_angular").value))
        self.angular_deadband = abs(float(self.get_parameter("angular_deadband").value))
        self.smoothing_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter("smoothing_alpha").value)),
        )
        self.debug_save_root = Path(str(self.get_parameter("debug_save_root").value))
        self.profile_hsv = {
            "route_line": (
                int(self.get_parameter("route_line_dark_threshold").value),
                [int(v) for v in self.get_parameter("route_line_hsv_lower").value],
                [int(v) for v in self.get_parameter("route_line_hsv_upper").value],
            ),
            "dock_line": (
                int(self.get_parameter("dock_line_dark_threshold").value),
                [int(v) for v in self.get_parameter("dock_line_hsv_lower").value],
                [int(v) for v in self.get_parameter("dock_line_hsv_upper").value],
            ),
            "b_side_dock_line": (
                int(self.get_parameter("b_side_dock_line_dark_threshold").value),
                [int(v) for v in self.get_parameter("b_side_dock_line_hsv_lower").value],
                [int(v) for v in self.get_parameter("b_side_dock_line_hsv_upper").value],
            ),
        }

        self.enabled = False
        self.mode = "OFF"
        self.current_profile = "route_line"
        self.current_context = ""
        self.last_offset = 0.0
        self.last_angle = 0.0
        self.last_detected = False
        self.last_detection_time = None
        self.last_angular_correction = 0.0
        self.last_nav_cmd = Twist()
        self.last_nav_time = None
        self.last_status_json = ""
        self.image_width = 0
        self.image_height = 0
        self.last_error_px = 0.0
        self.last_normalized_error = 0.0
        self.last_error_rate = 0.0
        self.last_error_time = None
        self.line_primary_phase = "IDLE"
        self.last_steering_direction = "none"
        self.last_linear_cmd = 0.0
        self.last_raw_angular_cmd = 0.0
        self.last_control_decision = "idle"
        self.centered_streak = 0
        self.centered_candidate = False
        self.last_timeout_reason = ""
        self.phase_start_time = None
        self.last_abs_error_px = None
        self.last_error_improvement_time = None
        self.capture_good_count = 0
        self.follow_bad_count = 0
        self.loss_count = 0
        self.filtered_error_px = 0.0
        self.filtered_normalized_error = 0.0
        self.filtered_angle = 0.0
        self.filtered_error_rate = 0.0
        self.last_lookahead_offset = 0.0
        self.last_lookahead_x = 0.0
        self.last_lookahead_y = 0.0
        self.search_direction = 1.0
        self.last_debug_image = None
        self.last_mask = None
        self.last_frame_bgr = None
        self.pending_docking_snapshot = False
        self.line_lost_snapshot_saved = False
        self.state_transition_counts: dict[str, int] = {}
        self.bridge = CvBridge()

        self.line_cmd_pub = self.create_publisher(Twist, self.line_cmd_topic, 10)
        self.fused_cmd_pub = self.create_publisher(Twist, self.fused_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.camera_line_status_pub = self.create_publisher(
            String, self.camera_line_status_topic, 10
        )
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.camera_line_debug_pub = self.create_publisher(
            Image, self.camera_line_debug_image_topic, 10
        )

        self.create_subscription(Image, self.camera_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, self.enable_topic, self.enable_cb, 10)
        self.create_subscription(String, self.mode_topic, self.mode_cb, 10)
        self.create_subscription(String, self.profile_topic, self.profile_cb, 10)
        self.create_subscription(String, self.context_topic, self.context_cb, 10)
        self.create_subscription(Twist, self.nav_cmd_topic, self.nav_cmd_cb, 20)
        if self.nav_cmd_fallback_topic != self.nav_cmd_topic:
            self.create_subscription(Twist, self.nav_cmd_fallback_topic, self.nav_cmd_cb, 20)

        control_hz = max(1.0, float(self.get_parameter("control_hz").value))
        self.create_timer(1.0 / control_hz, self.control_loop)
        self.publish_status(reason="initialized")
        self.get_logger().info(
            f"Line follower subscribed to {self.camera_topic}, {self.nav_cmd_topic}, "
            f"{self.nav_cmd_fallback_topic}; publishing fused route commands on {self.fused_cmd_topic}. "
            f"camera_line_reverse={'true' if self.camera_line_reverse else 'false'}"
        )

    def now(self):
        return self.get_clock().now()

    def line_recent(self) -> bool:
        if self.last_detection_time is None:
            return False
        return (self.now() - self.last_detection_time).nanoseconds / 1e9 <= self.line_timeout_sec

    def nav_recent(self) -> bool:
        if self.last_nav_time is None:
            return False
        return (self.now() - self.last_nav_time).nanoseconds / 1e9 <= self.nav_timeout_sec

    def enable_cb(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.publish_zero()
        self.publish_status(reason="enable_changed")

    def mode_cb(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        if mode not in VALID_MODES:
            self.publish_status(reason=f"invalid_mode:{mode}")
            return
        previous_mode = self.mode
        self.mode = mode
        if mode == "OFF":
            self.enabled = False
            self.publish_zero()
        elif mode in {"DOCKING", "LINE_PRIMARY_ROUTE"} and previous_mode != mode:
            self.pending_docking_snapshot = mode == "DOCKING"
            self.line_lost_snapshot_saved = False
            if mode == "LINE_PRIMARY_ROUTE":
                self.reset_line_primary_fsm()
        self.publish_status(reason="mode_changed")

    def profile_cb(self, msg: String) -> None:
        requested = msg.data.strip()
        if requested not in VALID_PROFILES:
            self.publish_status(reason=f"invalid_profile:{requested}")
            return
        self.current_profile = requested
        self.publish_status(reason="profile_changed")

    def context_cb(self, msg: String) -> None:
        self.current_context = msg.data.strip()
        self.publish_status(reason="context_changed")

    def nav_cmd_cb(self, msg: Twist) -> None:
        self.last_nav_cmd = msg
        self.last_nav_time = self.now()

    def active_profile(self) -> tuple[int, list[int], list[int]]:
        return self.profile_hsv.get(self.current_profile, self.profile_hsv["route_line"])

    def image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.publish_status(reason=f"image_convert_error:{exc}")
            return

        self.last_frame_bgr = frame.copy()
        self.image_height, self.image_width = frame.shape[:2]
        dark_threshold, hsv_lower, hsv_upper = self.active_profile()
        detection, debug, mask = detect_line_with_mask(
            frame,
            dark_threshold=dark_threshold,
            min_area=self.min_area,
            color_lower_hsv=hsv_lower,
            color_upper_hsv=hsv_upper,
        )
        self.last_detected = detection.detected
        self.last_mask = mask.copy()
        if detection.detected:
            sign = -1.0 if self.camera_line_reverse else 1.0
            self.last_offset = sign * detection.offset
            self.last_angle = sign * detection.angle
            self.last_lookahead_offset = sign * detection.lookahead_offset
            self.last_lookahead_x = detection.lookahead_x
            self.last_lookahead_y = detection.lookahead_y
            self.last_detection_time = self.now()
            self.line_lost_snapshot_saved = False

        image_center_x = self.image_width / 2.0 if self.image_width > 0 else 0.0
        self.last_error_px = float(self.compute_lookahead_x() - image_center_x) if self.image_width > 0 else 0.0
        self.last_normalized_error = float(self.last_lookahead_offset)
        self.filtered_error_px = (
            self.line_primary_error_ema_alpha * self.last_error_px
            + (1.0 - self.line_primary_error_ema_alpha) * self.filtered_error_px
        )
        self.filtered_normalized_error = (
            self.line_primary_error_ema_alpha * self.last_normalized_error
            + (1.0 - self.line_primary_error_ema_alpha) * self.filtered_normalized_error
        )
        self.filtered_angle = (
            self.line_primary_angle_ema_alpha * self.last_angle
            + (1.0 - self.line_primary_angle_ema_alpha) * self.filtered_angle
        )
        if self.last_error_px > 0.0:
            self.search_direction = -1.0
        elif self.last_error_px < 0.0:
            self.search_direction = 1.0
        now = self.now()
        if self.last_error_time is not None:
            dt = (now - self.last_error_time).nanoseconds / 1e9
            if dt > 1e-6:
                self.last_error_rate = (self.last_normalized_error - getattr(self, "_prev_normalized_error", 0.0)) / dt
                self.filtered_error_rate = (
                    self.filtered_normalized_error - getattr(self, "_prev_filtered_normalized_error", 0.0)
                ) / dt
        self._prev_normalized_error = self.last_normalized_error
        self._prev_filtered_normalized_error = self.filtered_normalized_error
        self.last_error_time = now
        self.update_centering_state()
        self.last_debug_image = self.decorate_debug_image(debug)
        if self.pending_docking_snapshot and self.mode == "DOCKING":
            self.save_debug_bundle("dock_start")
            self.pending_docking_snapshot = False
        if self.publish_debug_image and self.last_debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(self.last_debug_image, encoding="bgr8")
            except CvBridgeError as exc:
                self.publish_status(reason=f"debug_convert_error:{exc}")
                return
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
            self.camera_line_debug_pub.publish(debug_msg)

        self.publish_status(reason="image_update")

    def decorate_debug_image(self, debug):
        if debug is None:
            return None
        overlay = debug.copy()
        if self.image_width <= 0 or self.image_height <= 0:
            return overlay
        roi_y = int(self.image_height * 0.35)
        cv2.rectangle(
            overlay,
            (0, roi_y),
            (self.image_width - 1, self.image_height - 1),
            (255, 255, 0),
            2,
        )
        image_center_x = int(self.image_width / 2.0)
        cv2.line(
            overlay,
            (image_center_x, roi_y),
            (image_center_x, self.image_height - 1),
            (255, 255, 0),
            1,
        )
        cv2.putText(
            overlay,
            f"profile={self.current_profile}",
            (20, 170),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
        if self.current_context:
            cv2.putText(
                overlay,
                f"context={self.current_context}",
                (20, 200),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            overlay,
            f"mode={self.mode}",
            (20, 230),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
        if self.last_detected:
            line_center_x = int(round(self.compute_line_center_x()))
            lookahead_x = int(round(self.compute_lookahead_x()))
            lookahead_y = int(round(self.last_lookahead_y)) if self.last_lookahead_y > 0.0 else self.image_height - 60
            cv2.circle(overlay, (line_center_x, self.image_height - 30), 7, (0, 0, 255), -1)
            cv2.circle(overlay, (lookahead_x, lookahead_y), 7, (0, 128, 255), -1)
            cv2.putText(
                overlay,
                f"line_x={line_center_x} lookahead_x={lookahead_x}",
                (20, 140),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            overlay,
            f"error_px={self.last_error_px:.1f} filt={self.filtered_error_px:.1f} centered={self.robot_centered()}",
            (20, 260),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255) if self.robot_centered() else (0, 140, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            f"phase={self.line_primary_phase} steer={self.last_steering_direction} rate={self.last_error_rate:.2f}",
            (20, 290),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return overlay

    def compute_line_center_x(self) -> float:
        if self.image_width <= 0:
            return 0.0
        image_center_x = self.image_width / 2.0
        return image_center_x + self.last_offset * image_center_x

    def compute_lookahead_x(self) -> float:
        if self.image_width <= 0:
            return 0.0
        image_center_x = self.image_width / 2.0
        return image_center_x + self.last_lookahead_offset * image_center_x

    def robot_centered(self) -> bool:
        return bool(self.centered_streak >= self.centered_required_streak)

    def update_centering_state(self) -> None:
        candidate = bool(
            self.last_detected
            and self.line_recent()
            and abs(self.filtered_error_px) <= self.centered_error_px_threshold
            and abs(self.filtered_angle) <= self.centered_heading_error_rad_threshold
            and abs(self.filtered_error_rate) <= self.centered_max_error_rate
        )
        self.centered_candidate = candidate
        if candidate:
            self.centered_streak += 1
        else:
            self.centered_streak = 0

    def error_decreasing(self) -> bool:
        return bool(self.last_detected and self.line_recent() and self.last_error_rate < -0.02)

    def phase_elapsed(self) -> float:
        if self.phase_start_time is None:
            return 0.0
        return (self.now() - self.phase_start_time).nanoseconds / 1e9

    def reset_line_primary_fsm(self) -> None:
        self.line_primary_phase = "SEARCH_LINE"
        self.phase_start_time = self.now()
        self.last_abs_error_px = None
        self.last_error_improvement_time = self.now()
        self.capture_good_count = 0
        self.follow_bad_count = 0
        self.loss_count = 0
        self.last_timeout_reason = ""
        self.last_steering_direction = "none"
        self.search_direction = 1.0

    def set_line_primary_phase(self, phase: str, *, timeout_reason: str = "") -> None:
        if self.line_primary_phase != phase:
            self.state_transition_counts[phase] = self.state_transition_counts.get(phase, 0) + 1
            self.line_primary_phase = phase
            self.phase_start_time = self.now()
            if phase in {"CAPTURE_LINE", "FOLLOW_LINE"}:
                self.last_error_improvement_time = self.now()
            if phase != "FAIL_SAFE":
                self.last_timeout_reason = timeout_reason
            if self.last_debug_image is not None:
                event = {
                    "SEARCH_LINE": "search_line",
                    "CAPTURE_LINE": "capture_line",
                    "FOLLOW_LINE": "follow_line",
                    "LINE_LOST_STOP": "line_lost_stop",
                    "FAIL_SAFE": "fail_safe",
                }.get(phase)
                if event:
                    self.save_debug_bundle(event)

    def update_error_progress(self) -> None:
        if not self.last_detected or not self.line_recent():
            return
        abs_error = abs(self.filtered_error_px)
        if self.last_abs_error_px is None or abs_error < self.last_abs_error_px - 2.0:
            self.last_error_improvement_time = self.now()
        self.last_abs_error_px = abs_error

    def current_status(self, reason: str = "") -> dict[str, object]:
        image_center_x = self.image_width / 2.0 if self.image_width > 0 else 0.0
        dark_threshold, hsv_lower, hsv_upper = self.active_profile()
        return {
            "enabled": self.enabled,
            "mode": self.mode,
            "profile": self.current_profile,
            "context": self.current_context,
            "line_detected": bool(self.last_detected and self.line_recent()),
            "error_px": self.last_error_px,
            "filtered_error_px": self.filtered_error_px,
            "normalized_error": self.last_normalized_error,
            "filtered_normalized_error": self.filtered_normalized_error,
            "error_decreasing": self.error_decreasing(),
            "line_angle_rad": float(self.last_angle),
            "filtered_line_angle_rad": float(self.filtered_angle),
            "heading_error_rad": float(self.last_angle),
            "confidence": 1.0 if bool(self.last_detected and self.line_recent()) else 0.0,
            "line_center_x": float(self.compute_line_center_x()) if self.image_width > 0 else 0.0,
            "lookahead_x": float(self.compute_lookahead_x()) if self.image_width > 0 else 0.0,
            "lookahead_y": float(self.last_lookahead_y),
            "image_center_x": float(image_center_x),
            "reverse_sign_mode": bool(self.camera_line_reverse),
            "raw_angular_cmd": float(self.last_raw_angular_cmd),
            "angular_cmd": float(self.last_angular_correction),
            "linear_cmd": float(self.last_linear_cmd),
            "control_decision": self.last_control_decision,
            "error_rate": float(self.last_error_rate),
            "filtered_error_rate": float(self.filtered_error_rate),
            "robot_centered": self.robot_centered(),
            "centered_candidate": bool(self.centered_candidate),
            "centered_streak": int(self.centered_streak),
            "centered_required_streak": int(self.centered_required_streak),
            "centered_heading_error_rad_threshold": float(self.centered_heading_error_rad_threshold),
            "centered_max_error_rate": float(self.centered_max_error_rate),
            "state": self.line_primary_phase,
            "line_primary_phase": self.line_primary_phase,
            "steering_direction": self.last_steering_direction,
            "centered_error_px_threshold": self.centered_error_px_threshold,
            "state_transition_counts": dict(self.state_transition_counts),
            "dark_threshold": dark_threshold,
            "hsv_lower": hsv_lower,
            "hsv_upper": hsv_upper,
            "timeout_reason": self.last_timeout_reason,
            "reason": reason,
        }

    def publish_status(self, *, reason: str) -> None:
        msg = String()
        msg.data = json.dumps(self.current_status(reason), separators=(",", ":"))
        self.last_status_json = msg.data
        self.status_pub.publish(msg)
        self.camera_line_status_pub.publish(msg)

    def artifact_dir(self) -> Path:
        context = self.current_context or "unknown"
        if "B3" in context:
            directory = self.debug_save_root / "line_debug_B3"
        else:
            directory = self.debug_save_root / f"line_debug_{context or 'general'}"
        directory.mkdir(parents=True, exist_ok=True)
        return directory

    def save_debug_bundle(self, event: str) -> None:
        if self.last_frame_bgr is None or self.last_mask is None or self.last_debug_image is None:
            return
        timestamp_ns = self.now().nanoseconds
        base = self.artifact_dir() / f"{timestamp_ns}_{event}"
        mask_bgr = cv2.cvtColor(self.last_mask, cv2.COLOR_GRAY2BGR)
        cv2.imwrite(str(base.with_suffix(".raw.png")), self.last_frame_bgr)
        cv2.imwrite(str(base.with_suffix(".mask.png")), mask_bgr)
        cv2.imwrite(str(base.with_suffix(".debug.png")), self.last_debug_image)
        base.with_suffix(".json").write_text(
            json.dumps(self.current_status(event), indent=2),
            encoding="utf-8",
        )

    def correction(self, offset_gain: float, angle_gain: float, max_angular: float) -> float:
        raw = -(offset_gain * self.last_offset + angle_gain * self.last_angle)
        self.last_raw_angular_cmd = raw
        if abs(raw) < self.angular_deadband:
            raw = 0.0
        self.last_control_decision = "deadband_zero" if raw == 0.0 else "normal_correction"
        raw = max(-max_angular, min(max_angular, raw))
        angular = self.smoothing_alpha * raw + (1.0 - self.smoothing_alpha) * self.last_angular_correction
        self.last_angular_correction = angular
        return angular

    def line_primary_correction(
        self,
        *,
        include_angle: bool,
        smoothing_alpha: float | None = None,
        min_magnitude: float = 0.0,
    ) -> float:
        offset_term = -(self.line_primary_offset_gain * self.filtered_normalized_error)
        angle_term = self.line_primary_angle_gain * self.filtered_angle if include_angle else 0.0
        derivative_term = -(self.line_primary_derivative_gain * self.filtered_error_rate)
        raw = offset_term + angle_term + derivative_term
        self.last_raw_angular_cmd = raw
        if abs(raw) < self.angular_deadband:
            if abs(self.filtered_error_px) > self.centered_error_px_threshold:
                raw = 0.05 if offset_term >= 0.0 else -0.05
                self.last_control_decision = "force_recenter_min_angular"
            else:
                raw = 0.0
                self.last_control_decision = "deadband_zero"
        else:
            self.last_control_decision = "normal_correction"
        raw = max(-self.line_primary_max_angular, min(self.line_primary_max_angular, raw))
        if raw != 0.0 and abs(raw) < min_magnitude:
            raw = min_magnitude if raw > 0.0 else -min_magnitude
            self.last_control_decision = "enforce_min_magnitude"
        alpha = self.smoothing_alpha if smoothing_alpha is None else smoothing_alpha
        angular = alpha * raw + (1.0 - alpha) * self.last_angular_correction
        self.last_angular_correction = angular
        if angular > self.angular_deadband:
            self.last_steering_direction = "left"
            self.search_direction = 1.0
        elif angular < -self.angular_deadband:
            self.last_steering_direction = "right"
            self.search_direction = -1.0
        else:
            self.last_steering_direction = "straight"
        return angular

    def line_primary_capture_stalled(self) -> bool:
        if self.last_error_improvement_time is None:
            return False
        return (self.now() - self.last_error_improvement_time).nanoseconds / 1e9 >= self.line_primary_error_stall_timeout_sec

    def line_primary_command(self, *, capture: bool) -> Twist:
        if capture:
            correction = self.line_primary_correction(
                include_angle=False,
                smoothing_alpha=self.line_primary_capture_smoothing_alpha,
                min_magnitude=self.line_primary_capture_min_angular,
            )
            linear_x = 0.0 if abs(self.last_error_px) > self.line_primary_rotate_only_error_px else self.line_primary_arc_capture_linear_speed
        else:
            correction = self.line_primary_correction(include_angle=True)
            if abs(self.last_error_px) > self.line_primary_follow_reenter_error_px:
                linear_x = min(self.line_primary_arc_capture_linear_speed, self.line_primary_linear_speed)
            else:
                linear_x = self.line_primary_linear_speed
        cmd = self.build_line_only_cmd(correction=correction, linear_x=linear_x)
        self.last_linear_cmd = cmd.linear.x
        return cmd

    def handle_line_primary(self) -> None:
        if not self.last_detected or not self.line_recent():
            self.loss_count += 1
            if self.line_primary_phase == "FOLLOW_LINE" and self.loss_count < self.line_primary_loss_confirm_frames:
                cmd = self.build_line_only_cmd(
                    correction=self.last_angular_correction * 0.8,
                    linear_x=self.line_primary_arc_capture_linear_speed,
                )
                self.last_linear_cmd = cmd.linear.x
                self.line_cmd_pub.publish(cmd)
                self.fused_cmd_pub.publish(cmd)
                self.publish_status(reason="line_primary_transient_line_lost")
                return
            if self.line_primary_phase == "SEARCH_LINE":
                if self.phase_elapsed() >= self.line_primary_search_timeout_sec:
                    self.set_line_primary_phase("LINE_LOST_STOP", timeout_reason="search_timeout")
                    self.publish_zero()
                    self.publish_status(reason="line_primary_search_timeout")
                    return
                cmd = self.build_line_only_cmd(
                    correction=self.line_primary_search_angular_speed * self.search_direction,
                    linear_x=0.0,
                )
                self.last_angular_correction = cmd.angular.z
                self.last_linear_cmd = cmd.linear.x
                self.last_steering_direction = "left" if cmd.angular.z > 0.0 else "right"
                self.line_cmd_pub.publish(cmd)
                self.fused_cmd_pub.publish(cmd)
                self.publish_status(reason="line_primary_searching")
                return
            if self.line_primary_phase != "FAIL_SAFE":
                self.set_line_primary_phase("SEARCH_LINE", timeout_reason="line_lost_recovery")
            if not self.line_lost_snapshot_saved:
                self.save_debug_bundle("route_line_lost")
                self.line_lost_snapshot_saved = True
            cmd = self.build_line_only_cmd(
                correction=self.line_primary_search_angular_speed * self.search_direction,
                linear_x=0.0,
            )
            self.last_angular_correction = cmd.angular.z
            self.last_linear_cmd = cmd.linear.x
            self.last_steering_direction = "left" if cmd.angular.z > 0.0 else "right"
            self.line_cmd_pub.publish(cmd)
            self.fused_cmd_pub.publish(cmd)
            self.publish_status(reason="line_primary_recover_search")
            return

        self.line_lost_snapshot_saved = False
        self.loss_count = 0
        self.update_error_progress()
        abs_error_px = abs(self.filtered_error_px)

        if self.line_primary_phase in {"SEARCH_LINE", "LINE_LOST_STOP", "FAIL_SAFE"}:
            self.set_line_primary_phase("CAPTURE_LINE")

        if self.line_primary_phase == "CAPTURE_LINE":
            if abs_error_px <= self.line_primary_follow_enter_error_px:
                self.capture_good_count += 1
            else:
                self.capture_good_count = 0
            cmd = self.line_primary_command(capture=True)
            self.line_cmd_pub.publish(cmd)
            self.fused_cmd_pub.publish(cmd)
            if self.capture_good_count >= self.line_primary_capture_good_frames:
                self.set_line_primary_phase("FOLLOW_LINE")
            elif self.line_primary_capture_stalled():
                self.set_line_primary_phase("SEARCH_LINE", timeout_reason="capture_stall")
            self.publish_status(reason="line_primary_capture")
            return

        if self.line_primary_phase == "FOLLOW_LINE":
            if abs_error_px >= self.line_primary_follow_exit_error_px:
                self.follow_bad_count += 1
            else:
                self.follow_bad_count = 0
            cmd = self.line_primary_command(capture=False)
            self.line_cmd_pub.publish(cmd)
            self.fused_cmd_pub.publish(cmd)
            if self.follow_bad_count >= self.line_primary_follow_bad_frames:
                self.set_line_primary_phase("CAPTURE_LINE", timeout_reason="follow_recenter")
            self.publish_status(reason="line_primary_follow")
            return

    def build_route_fused_cmd(self) -> Twist | None:
        if not self.nav_recent():
            return None
        correction = 0.0
        linear_scale = self.route_linear_scale_when_lost
        if self.last_detected and self.line_recent():
            correction = self.correction(
                self.route_offset_gain,
                self.route_angle_gain,
                self.route_max_angular_correction,
            )
            linear_scale = self.route_linear_scale_when_detected
        else:
            self.last_angular_correction = 0.0

        cmd = Twist()
        cmd.linear.x = self.last_nav_cmd.linear.x * linear_scale
        cmd.linear.y = self.last_nav_cmd.linear.y
        cmd.linear.z = self.last_nav_cmd.linear.z
        cmd.angular.x = self.last_nav_cmd.angular.x
        cmd.angular.y = self.last_nav_cmd.angular.y
        cmd.angular.z = self.last_nav_cmd.angular.z + correction
        cmd.angular.z = max(-self.route_max_total_angular, min(self.route_max_total_angular, cmd.angular.z))
        return cmd

    def build_line_only_cmd(self, *, correction: float, linear_x: float) -> Twist:
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = correction
        return cmd

    def control_loop(self) -> None:
        if not self.enabled or self.mode == "OFF":
            return

        if self.mode == "ROUTE_CORRECTION":
            fused = self.build_route_fused_cmd()
            if fused is None:
                self.last_angular_correction = 0.0
                self.publish_status(reason="route_no_nav_cmd")
                return
            line_only = Twist()
            if self.last_detected and self.line_recent():
                route_correction = fused.angular.z - self.last_nav_cmd.angular.z
                line_only = self.build_line_only_cmd(correction=route_correction, linear_x=0.0)
            else:
                self.last_angular_correction = 0.0
            self.line_cmd_pub.publish(line_only)
            self.fused_cmd_pub.publish(fused)
            self.publish_status(
                reason="route_detected" if self.last_detected and self.line_recent() else "route_line_lost_passthrough"
            )
            return

        if self.mode == "LINE_PRIMARY_ROUTE":
            self.handle_line_primary()
            return

        if self.mode == "DOCKING":
            if not self.last_detected or not self.line_recent():
                if not self.line_lost_snapshot_saved:
                    self.save_debug_bundle("line_lost")
                    self.line_lost_snapshot_saved = True
                self.publish_zero()
                self.publish_status(reason="docking_line_lost_stop")
                return
            correction = self.correction(
                self.docking_offset_gain,
                self.docking_angle_gain,
                self.docking_max_angular,
            )
            # Fork tips are aft of base_link in this robot model, so docking must drive
            # with negative base_link linear.x to move the forks toward the pallet.
            line_cmd = self.build_line_only_cmd(correction=correction, linear_x=-self.docking_linear_speed)
            self.line_cmd_pub.publish(line_cmd)
            self.fused_cmd_pub.publish(line_cmd)
            self.publish_status(reason="docking_detected")
            return

        self.publish_zero()
        self.publish_status(reason="off_zero")

    def publish_zero(self) -> None:
        self.last_angular_correction = 0.0
        self.last_linear_cmd = 0.0
        if self.mode == "OFF":
            self.line_primary_phase = "IDLE"
            self.last_steering_direction = "none"
        zero = Twist()
        self.line_cmd_pub.publish(zero)
        self.fused_cmd_pub.publish(zero)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_zero()
        except Exception:
            pass
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
