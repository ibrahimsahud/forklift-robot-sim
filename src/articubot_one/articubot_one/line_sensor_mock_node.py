#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32MultiArray, String
from tf2_ros import Buffer, TransformListener

from articubot_one.line_detection import detect_line_with_mask
from articubot_one.route_config import config_path, load_route_waypoints


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def anchored_pose_from_odom(
    anchor_map_pose: tuple[float, float, float] | None,
    anchor_odom_pose: tuple[float, float, float] | None,
    current_odom_pose: tuple[float, float, float] | None,
) -> tuple[float, float, float] | None:
    if anchor_map_pose is None or anchor_odom_pose is None or current_odom_pose is None:
        return None
    yaw_offset = normalize_angle(anchor_map_pose[2] - anchor_odom_pose[2])
    dx = current_odom_pose[0] - anchor_odom_pose[0]
    dy = current_odom_pose[1] - anchor_odom_pose[1]
    cos_yaw = math.cos(yaw_offset)
    sin_yaw = math.sin(yaw_offset)
    return (
        float(anchor_map_pose[0] + cos_yaw * dx - sin_yaw * dy),
        float(anchor_map_pose[1] + sin_yaw * dx + cos_yaw * dy),
        float(normalize_angle(current_odom_pose[2] + yaw_offset)),
    )


class LineSensorMockNode(Node):
    def __init__(self) -> None:
        super().__init__("line_sensor_mock_node")
        self.declare_parameter("source_mode", "camera")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("route_waypoints_file", str(config_path("route_waypoints.yaml")))
        self.declare_parameter("line_segments_file", str(config_path("line_segments.yaml")))
        self.declare_parameter("geometry_map_frame", "map")
        self.declare_parameter("geometry_base_frame", "base_link")
        self.declare_parameter("geometry_use_ground_truth", True)
        self.declare_parameter("geometry_ground_truth_odom_topic", "/ground_truth/odom")
        self.declare_parameter("geometry_world_origin_x_m", -5.80)
        self.declare_parameter("geometry_world_origin_y_m", -3.05)
        self.declare_parameter("geometry_world_origin_yaw_rad", 1.5708)
        self.declare_parameter("geometry_sensor_offset_x_m", 0.28)
        self.declare_parameter("geometry_sensor_range_m", 0.15)
        self.declare_parameter("geometry_seen_cross_track_m", 0.22)
        self.declare_parameter("geometry_seen_along_margin_m", 0.60)
        self.declare_parameter("dark_threshold", 90)
        self.declare_parameter("hsv_lower", [35, 45, 45])
        self.declare_parameter("hsv_upper", [95, 255, 255])
        self.declare_parameter("min_area", 250.0)
        self.declare_parameter("sensor_count", 5)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("seen_timeout_sec", 1.0)
        self.declare_parameter("position_amplitude", 30.0)
        self.declare_parameter("position_frequency_hz", 0.08)
        self.declare_parameter("base_value", 120)
        self.declare_parameter("line_value", 850)
        self.declare_parameter("seen", True)

        self.raw_pub = self.create_publisher(Int32MultiArray, "/line_sensor/raw", 10)
        self.position_pub = self.create_publisher(Float32, "/line_sensor/position", 10)
        self.seen_pub = self.create_publisher(Bool, "/line_sensor/seen", 10)
        self.status_pub = self.create_publisher(String, "/line_sensor/status", 10)

        self.source_mode = str(self.get_parameter("source_mode").value).strip().lower()
        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.dark_threshold = int(self.get_parameter("dark_threshold").value)
        self.hsv_lower = [int(v) for v in self.get_parameter("hsv_lower").value]
        self.hsv_upper = [int(v) for v in self.get_parameter("hsv_upper").value]
        self.min_area = float(self.get_parameter("min_area").value)
        self.bridge = CvBridge()
        self.start_time = time.monotonic()
        self.last_camera_position = 0.0
        self.last_camera_seen = False
        self.last_camera_raw: list[int] = []
        self.last_camera_status: dict[str, object] = {}
        self.last_detection_time = 0.0
        self.current_odom_pose: tuple[float, float, float] | None = None
        self.last_odom_wall_time = 0.0
        self.ground_truth_route_pose: tuple[float, float, float] | None = None
        self.last_ground_truth_wall_time = 0.0
        self.ground_truth_ready_logged = False
        self.active_segment = ""
        self.geometry_anchor_segment = ""
        self.geometry_anchor_map_pose: tuple[float, float, float] | None = None
        self.geometry_anchor_odom_pose: tuple[float, float, float] | None = None
        self.geometry_anchor_source = "none"
        self.geometry_continuity_pose: tuple[float, float, float] | None = None
        self.waypoints: dict[str, dict[str, float]] = {}
        self.line_segments: dict[str, tuple[str, str]] = {}
        self.tf_buffer: Buffer | None = None
        self.tf_listener: TransformListener | None = None
        if self.source_mode == "camera":
            self.create_subscription(Image, self.camera_topic, self.image_cb, qos_profile_sensor_data)
        elif self.source_mode == "geometry":
            self.load_geometry()
            self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
            self.create_subscription(Odometry, "/odom", self.odom_cb, 50)
            self.create_subscription(Odometry, "/diff_cont/odom", self.odom_cb, 50)
            if bool(self.get_parameter("geometry_use_ground_truth").value):
                self.create_subscription(
                    Odometry,
                    str(self.get_parameter("geometry_ground_truth_odom_topic").value),
                    self.ground_truth_odom_cb,
                    qos_profile_sensor_data,
                )
            segment_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(String, "/line_sensor/segment", self.segment_cb, segment_qos)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.publish_mock)

    def load_geometry(self) -> None:
        try:
            route_waypoints = load_route_waypoints(
                Path(str(self.get_parameter("route_waypoints_file").value))
            )
            self.waypoints = route_waypoints["waypoints"]
            loaded_segments = yaml.safe_load(
                Path(str(self.get_parameter("line_segments_file").value)).read_text(encoding="utf-8")
            ) or {}
        except Exception as exc:
            self.get_logger().error(f"LINE_SENSOR_GEOMETRY_CONFIG_FAILED: {exc}")
            self.waypoints = {}
            self.line_segments = {}
            return
        segments: dict[str, tuple[str, str]] = {}
        for section_name in ("road_segments", "approach_segments", "dock_segments"):
            section = loaded_segments.get(section_name, {})
            if not isinstance(section, dict):
                continue
            for name, segment in section.items():
                if not isinstance(segment, dict):
                    continue
                start = str(segment.get("start", "")).upper()
                end = str(segment.get("end", "")).upper()
                if start in self.waypoints and end in self.waypoints:
                    segments[str(name).upper()] = (start, end)
        self.line_segments = segments
        self.get_logger().info(
            f"LINE_SENSOR_GEOMETRY loaded {len(self.line_segments)} line segments"
        )

    def odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.current_odom_pose = (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        self.last_odom_wall_time = time.monotonic()

    def ground_truth_odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        world_x = float(pose.position.x)
        world_y = float(pose.position.y)
        world_yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        origin_x = float(self.get_parameter("geometry_world_origin_x_m").value)
        origin_y = float(self.get_parameter("geometry_world_origin_y_m").value)
        origin_yaw = float(self.get_parameter("geometry_world_origin_yaw_rad").value)
        dx = world_x - origin_x
        dy = world_y - origin_y
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        self.ground_truth_route_pose = (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            normalize_angle(world_yaw - origin_yaw),
        )
        self.last_ground_truth_wall_time = time.monotonic()
        if not self.ground_truth_ready_logged:
            self.ground_truth_ready_logged = True
            self.get_logger().info(
                "LINE_SENSOR_GAZEBO_GROUND_TRUTH_READY: virtual sensor is aligned to the physical floor"
            )

    def segment_cb(self, msg: String) -> None:
        segment = msg.data.strip().upper()
        if segment != self.active_segment:
            self.geometry_continuity_pose = anchored_pose_from_odom(
                self.geometry_anchor_map_pose,
                self.geometry_anchor_odom_pose,
                self.current_odom_pose,
            )
            self.geometry_anchor_segment = ""
            self.geometry_anchor_map_pose = None
            self.geometry_anchor_odom_pose = None
            self.geometry_anchor_source = "none"
        self.active_segment = segment

    def current_map_pose(self) -> tuple[float, float, float] | None:
        if self.tf_buffer is None:
            return None
        map_frame = str(self.get_parameter("geometry_map_frame").value)
        base_frame = str(self.get_parameter("geometry_base_frame").value)
        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                Time(),
                timeout=Duration(seconds=0.02),
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

    def current_geometry_pose(
        self,
    ) -> tuple[tuple[float, float, float] | None, str, tuple[float, float, float] | None]:
        map_pose = self.current_map_pose()
        ground_truth_fresh = (
            bool(self.get_parameter("geometry_use_ground_truth").value)
            and self.ground_truth_route_pose is not None
            and (time.monotonic() - self.last_ground_truth_wall_time) <= 1.0
        )
        if ground_truth_fresh:
            return self.ground_truth_route_pose, "gazebo_ground_truth", map_pose
        odom_fresh = (
            self.current_odom_pose is not None
            and (time.monotonic() - self.last_odom_wall_time) <= 1.0
        )
        if not self.active_segment or map_pose is None or not odom_fresh:
            return map_pose, "map_tf", map_pose

        if (
            self.geometry_anchor_segment != self.active_segment
            or self.geometry_anchor_map_pose is None
            or self.geometry_anchor_odom_pose is None
        ):
            anchor_map_pose = map_pose
            anchor_source = "map_tf"
            segment = self.line_segments.get(self.active_segment)
            if self.geometry_continuity_pose is not None and segment is not None:
                start = self.waypoints[segment[0]]
                start_gap = math.hypot(
                    self.geometry_continuity_pose[0] - float(start["x"]),
                    self.geometry_continuity_pose[1] - float(start["y"]),
                )
                if start_gap <= 0.35:
                    anchor_map_pose = self.geometry_continuity_pose
                    anchor_source = "previous_segment_continuity"
            self.geometry_anchor_segment = self.active_segment
            self.geometry_anchor_map_pose = anchor_map_pose
            self.geometry_anchor_odom_pose = self.current_odom_pose
            self.geometry_anchor_source = anchor_source
            self.geometry_continuity_pose = None
            self.get_logger().info(
                f"LINE_SENSOR_ODOM_ANCHOR {self.active_segment}: "
                f"source={anchor_source} "
                f"anchor=({anchor_map_pose[0]:.3f},{anchor_map_pose[1]:.3f},{anchor_map_pose[2]:.3f}) "
                f"live_map=({map_pose[0]:.3f},{map_pose[1]:.3f},{map_pose[2]:.3f}) "
                f"odom=({self.current_odom_pose[0]:.3f},{self.current_odom_pose[1]:.3f},"
                f"{self.current_odom_pose[2]:.3f})"
            )

        anchored_pose = anchored_pose_from_odom(
            self.geometry_anchor_map_pose,
            self.geometry_anchor_odom_pose,
            self.current_odom_pose,
        )
        if anchored_pose is None:
            return map_pose, "map_tf", map_pose
        return anchored_pose, f"odom_segment_anchor:{self.geometry_anchor_source}", map_pose

    def image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return
        detection, _, _ = detect_line_with_mask(
            frame,
            dark_threshold=self.dark_threshold,
            min_area=self.min_area,
            color_lower_hsv=self.hsv_lower,
            color_upper_hsv=self.hsv_upper,
        )
        count = int(self.get_parameter("sensor_count").value)
        amp = float(self.get_parameter("position_amplitude").value)
        base = int(self.get_parameter("base_value").value)
        line = int(self.get_parameter("line_value").value)
        if detection.detected:
            position = max(-amp, min(amp, float(detection.lookahead_offset) * amp))
            raw = self.synthetic_raw_values(position, count=count, amp=amp, base=base, line=line)
            self.last_camera_position = position
            self.last_camera_seen = True
            self.last_camera_raw = raw
            self.last_detection_time = time.monotonic()
            self.last_camera_status = {
                "mock": True,
                "source_mode": "camera",
                "seen": True,
                "position": position,
                "offset": float(detection.offset),
                "lookahead_offset": float(detection.lookahead_offset),
                "angle_rad": float(detection.angle),
                "raw": raw,
            }
        else:
            self.last_camera_status = {
                "mock": True,
                "source_mode": "camera",
                "seen": False,
                "position": self.last_camera_position,
                "raw": self.last_camera_raw,
            }

    def synthetic_raw_values(self, position: float, *, count: int, amp: float, base: int, line: int) -> list[int]:
        values = [base] * count
        if count <= 0:
            return values
        center = (count - 1) / 2.0
        normalized = max(-1.0, min(1.0, position / max(amp, 1.0)))
        line_index = center + normalized * center
        for i in range(count):
            distance = abs(i - line_index)
            strength = max(0.0, 1.0 - 0.7 * distance)
            values[i] = int(base + strength * (line - base))
        return values

    def geometry_line_measurement(self) -> dict[str, object]:
        geometry_pose, pose_source, live_map_pose = self.current_geometry_pose()
        if geometry_pose is None or not self.waypoints or not self.line_segments:
            return {
                "mock": True,
                "source_mode": "geometry",
                "pose_frame": str(self.get_parameter("geometry_map_frame").value),
                "seen": False,
                "reason": "map_pose_or_geometry_not_ready",
                "requested_segment": self.active_segment,
            }

        if self.active_segment and self.active_segment not in self.line_segments:
            return {
                "mock": True,
                "source_mode": "geometry",
                "pose_frame": str(self.get_parameter("geometry_map_frame").value),
                "seen": False,
                "reason": "requested_segment_unknown",
                "requested_segment": self.active_segment,
            }

        candidates: list[tuple[float, str, str, str, float, float, float, float]] = []
        segment_items = (
            [(self.active_segment, self.line_segments[self.active_segment])]
            if self.active_segment in self.line_segments
            else list(self.line_segments.items())
        )
        x, y, yaw = geometry_pose
        sensor_offset_x = float(self.get_parameter("geometry_sensor_offset_x_m").value)
        sensor_x = x + math.cos(yaw) * sensor_offset_x
        sensor_y = y + math.sin(yaw) * sensor_offset_x
        for segment_name, (start_name, end_name) in segment_items:
            start = self.waypoints[start_name]
            end = self.waypoints[end_name]
            dx = float(end["x"]) - float(start["x"])
            dy = float(end["y"]) - float(start["y"])
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                continue
            ux = dx / length
            uy = dy / length
            px = sensor_x - float(start["x"])
            py = sensor_y - float(start["y"])
            along = px * ux + py * uy
            cross = px * (-uy) + py * ux
            heading = math.atan2(dy, dx)
            heading_error = normalize_angle(heading - yaw)
            clamped_along = max(0.0, min(length, along))
            endpoint_error = abs(along - clamped_along)
            score = abs(cross) + 0.25 * endpoint_error
            candidates.append((score, segment_name, start_name, end_name, cross, along, length, heading_error))

        if not candidates:
            return {"mock": True, "source_mode": "geometry", "seen": False, "reason": "no_segments"}
        _, segment_name, start_name, end_name, cross, along, length, heading_error = min(candidates, key=lambda item: item[0])
        seen_cross_track = float(self.get_parameter("geometry_seen_cross_track_m").value)
        along_margin = float(self.get_parameter("geometry_seen_along_margin_m").value)
        seen = abs(cross) <= seen_cross_track and (-along_margin <= along <= length + along_margin)
        amp = float(self.get_parameter("position_amplitude").value)
        sensor_range = max(0.01, float(self.get_parameter("geometry_sensor_range_m").value))
        position = max(-amp, min(amp, (cross / sensor_range) * amp))
        raw = self.synthetic_raw_values(
            position,
            count=int(self.get_parameter("sensor_count").value),
            amp=amp,
            base=int(self.get_parameter("base_value").value),
            line=int(self.get_parameter("line_value").value),
        )
        map_anchor_disagreement_m = None
        if live_map_pose is not None:
            map_anchor_disagreement_m = math.hypot(live_map_pose[0] - x, live_map_pose[1] - y)
        return {
            "mock": True,
            "source_mode": "geometry",
            "pose_frame": str(self.get_parameter("geometry_map_frame").value),
            "pose_source": pose_source,
            "robot_pose": [x, y, yaw],
            "live_map_pose": list(live_map_pose) if live_map_pose is not None else None,
            "map_anchor_disagreement_m": map_anchor_disagreement_m,
            "sensor_pose": [sensor_x, sensor_y, yaw],
            "sensor_offset_x_m": sensor_offset_x,
            "seen": seen,
            "segment": segment_name,
            "requested_segment": self.active_segment,
            "segment_start": start_name,
            "segment_end": end_name,
            "position": position,
            "cross_track_m": cross,
            "along_track_m": along,
            "segment_length_m": length,
            "lookahead_offset": max(-1.0, min(1.0, cross / sensor_range)),
            "angle_rad": heading_error,
            "raw": raw,
        }

    def publish_mock(self) -> None:
        if self.source_mode == "camera":
            self.publish_camera_sensor()
            return
        if self.source_mode == "geometry":
            self.publish_geometry_sensor()
            return
        t = time.monotonic() - self.start_time
        count = int(self.get_parameter("sensor_count").value)
        amp = float(self.get_parameter("position_amplitude").value)
        freq = float(self.get_parameter("position_frequency_hz").value)
        base = int(self.get_parameter("base_value").value)
        line = int(self.get_parameter("line_value").value)
        seen = bool(self.get_parameter("seen").value)

        position = amp * math.sin(2.0 * math.pi * freq * t) if seen else 0.0
        values = [base] * count
        if seen:
            center = (count - 1) / 2.0
            normalized = max(-1.0, min(1.0, position / max(amp, 1.0)))
            line_index = center + normalized * center
            for i in range(count):
                distance = abs(i - line_index)
                strength = max(0.0, 1.0 - 0.7 * distance)
                values[i] = int(base + strength * (line - base))

        raw_msg = Int32MultiArray()
        raw_msg.data = values
        self.raw_pub.publish(raw_msg)

        pos_msg = Float32()
        pos_msg.data = float(position)
        self.position_pub.publish(pos_msg)

        seen_msg = Bool()
        seen_msg.data = seen
        self.seen_pub.publish(seen_msg)

        status = String()
        status.data = json.dumps(
            {"mock": True, "position": position, "seen": seen, "raw": values},
            ensure_ascii=True,
        )
        self.status_pub.publish(status)

    def publish_camera_sensor(self) -> None:
        seen_timeout = float(self.get_parameter("seen_timeout_sec").value)
        seen = bool((time.monotonic() - self.last_detection_time) <= seen_timeout)
        self.last_camera_seen = seen
        position = float(self.last_camera_position)
        values = list(self.last_camera_raw) if self.last_camera_raw else self.synthetic_raw_values(
            position,
            count=int(self.get_parameter("sensor_count").value),
            amp=float(self.get_parameter("position_amplitude").value),
            base=int(self.get_parameter("base_value").value),
            line=int(self.get_parameter("line_value").value),
        )

        raw_msg = Int32MultiArray()
        raw_msg.data = values
        self.raw_pub.publish(raw_msg)

        pos_msg = Float32()
        pos_msg.data = position
        self.position_pub.publish(pos_msg)

        seen_msg = Bool()
        seen_msg.data = seen
        self.seen_pub.publish(seen_msg)

        payload = dict(self.last_camera_status)
        payload.update({"seen": seen, "position": position, "raw": values})
        status = String()
        status.data = json.dumps(payload, ensure_ascii=True)
        self.status_pub.publish(status)

    def publish_geometry_sensor(self) -> None:
        payload = self.geometry_line_measurement()
        position = float(payload.get("position", 0.0)) if isinstance(payload.get("position"), (int, float)) else 0.0
        values = payload.get("raw")
        if not isinstance(values, list):
            values = self.synthetic_raw_values(
                position,
                count=int(self.get_parameter("sensor_count").value),
                amp=float(self.get_parameter("position_amplitude").value),
                base=int(self.get_parameter("base_value").value),
                line=int(self.get_parameter("line_value").value),
            )
            payload["raw"] = values

        raw_msg = Int32MultiArray()
        raw_msg.data = [int(value) for value in values]
        self.raw_pub.publish(raw_msg)

        pos_msg = Float32()
        pos_msg.data = position
        self.position_pub.publish(pos_msg)

        seen_msg = Bool()
        seen_msg.data = bool(payload.get("seen", False))
        self.seen_pub.publish(seen_msg)

        status = String()
        status.data = json.dumps(payload, ensure_ascii=True)
        self.status_pub.publish(status)


def main() -> None:
    rclpy.init()
    node = LineSensorMockNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
