#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from articubot_one.route_config import config_path, load_mission_routes, load_route_waypoints


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def station_from_dock(name: str) -> str | None:
    normalized = name.upper()
    if "_DOCK" not in normalized:
        return None
    return normalized.split("_DOCK", 1)[0]


class MissionPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__(
            "mission_path_publisher",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.declare_parameter("route", "A1_TO_B3")
        self.declare_parameter("routes_file", str(config_path("mission_routes.yaml")))
        self.declare_parameter("route_waypoints_file", str(config_path("route_waypoints.yaml")))
        self.declare_parameter("line_segments_file", str(config_path("line_segments.yaml")))
        self.declare_parameter("planned_path_topic", "/mission/planned_path")
        self.declare_parameter("actual_path_topic", "/mission/actual_path")
        self.declare_parameter("planned_markers_topic", "/mission/planned_markers")
        self.declare_parameter("actual_path_min_distance_m", 0.03)
        self.declare_parameter("actual_path_max_points", 3000)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("use_ground_truth", True)
        self.declare_parameter("ground_truth_odom_topic", "/ground_truth/odom")
        self.declare_parameter("gazebo_world_origin_x_m", -5.80)
        self.declare_parameter("gazebo_world_origin_y_m", -3.05)
        self.declare_parameter("gazebo_world_origin_yaw_rad", 1.5708)

        route_waypoints = load_route_waypoints(Path(str(self.get_parameter("route_waypoints_file").value)))
        self.waypoints: dict[str, dict[str, float]] = route_waypoints["waypoints"]
        self.qr_aliases: dict[str, str] = route_waypoints["qr_aliases"]
        self.frame_id = str((route_waypoints.get("map") or {}).get("frame_id") or "map")
        self.routes = load_mission_routes(Path(str(self.get_parameter("routes_file").value)))
        self.route_name = str(self.get_parameter("route").value).upper()
        self.line_segments = self.load_line_segments(Path(str(self.get_parameter("line_segments_file").value)))

        self.planned_path_pub = self.create_publisher(
            RosPath,
            str(self.get_parameter("planned_path_topic").value),
            1,
        )
        self.actual_path_pub = self.create_publisher(
            RosPath,
            str(self.get_parameter("actual_path_topic").value),
            1,
        )
        self.planned_markers_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("planned_markers_topic").value),
            1,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.last_amcl_pose: tuple[float, float, float] | None = None
        self.last_amcl_wall_time = 0.0
        self.last_odom_pose: tuple[float, float, float] | None = None
        self.last_odom_wall_time = 0.0
        self.last_ground_truth_route_pose: tuple[float, float, float] | None = None
        self.last_ground_truth_wall_time = 0.0
        self.actual_anchor_map_pose: tuple[float, float, float] | None = None
        self.actual_anchor_odom_pose: tuple[float, float, float] | None = None
        self.actual_path = RosPath()
        self.actual_path.header.frame_id = self.frame_id

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_cb, 20)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 50)
        if bool(self.get_parameter("use_ground_truth").value):
            self.create_subscription(
                Odometry,
                str(self.get_parameter("ground_truth_odom_topic").value),
                self.ground_truth_odom_cb,
                50,
            )

        self.planned_route_names = self.route_path_names()
        self.planned_path = self.build_planned_path()
        self.planned_markers = self.build_planned_markers()
        self.get_logger().info(
            f"Publishing mission paths for {self.route_name}: "
            f"planned={self.get_parameter('planned_path_topic').value} "
            f"actual={self.get_parameter('actual_path_topic').value} "
            f"markers={self.get_parameter('planned_markers_topic').value} "
            f"points={len(self.planned_path.poses)}"
        )
        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish_paths)

    def add_path_name(self, names: list[str], name: str) -> None:
        normalized = name.upper()
        if normalized not in self.waypoints:
            return
        if names and names[-1] == normalized:
            return
        names.append(normalized)

    def load_line_segments(self, path: Path) -> dict[str, dict[str, dict[str, str]]]:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        result: dict[str, dict[str, dict[str, str]]] = {
            "road_segments": {},
            "approach_segments": {},
            "dock_segments": {},
        }
        if not isinstance(loaded, dict):
            return result
        for section_name in result:
            raw_section = loaded.get(section_name, {})
            if not isinstance(raw_section, dict):
                continue
            for name, segment in raw_section.items():
                if not isinstance(segment, dict):
                    continue
                start = str(segment.get("start", "")).upper()
                end = str(segment.get("end", "")).upper()
                if start in self.waypoints and end in self.waypoints:
                    result[section_name][str(name).upper()] = {"start": start, "end": end}
        return result

    def waypoint_point(self, name: str, z: float = 0.06) -> Point:
        waypoint = self.waypoints[name]
        point = Point()
        point.x = float(waypoint["x"])
        point.y = float(waypoint["y"])
        point.z = z
        return point

    def segment_name_for_pair(self, start: str, end: str) -> tuple[str, bool] | None:
        for section in self.line_segments.values():
            for segment_name, segment in section.items():
                if segment["start"] == start and segment["end"] == end:
                    return segment_name, False
                if segment["start"] == end and segment["end"] == start:
                    return segment_name, True
        return None

    def interpolated_pair_poses(self, start: str, end: str, *, step_m: float = 0.12) -> list[PoseStamped]:
        start_wp = self.waypoints[start]
        end_wp = self.waypoints[end]
        dx = float(end_wp["x"]) - float(start_wp["x"])
        dy = float(end_wp["y"]) - float(start_wp["y"])
        distance = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(distance / max(step_m, 0.01))))
        heading = math.atan2(dy, dx) if distance > 1e-6 else float(end_wp["yaw"])
        poses: list[PoseStamped] = []
        for index in range(1, steps + 1):
            ratio = index / steps
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(start_wp["x"]) + dx * ratio
            pose.pose.position.y = float(start_wp["y"]) + dy * ratio
            pose.pose.position.z = 0.04
            qx, qy, qz, qw = yaw_to_quaternion(heading)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            poses.append(pose)
        return poses

    def route_path_names(self) -> list[str]:
        raw_route = self.routes.get(self.route_name)
        if not raw_route:
            raise ValueError(f"Unknown route '{self.route_name}'")

        names: list[str] = []
        for raw_name in raw_route:
            name = str(raw_name).upper()
            if name in self.qr_aliases:
                target = self.qr_aliases[name].upper()
                if target.startswith("GATE"):
                    self.add_path_name(names, target)
                continue
            self.add_path_name(names, name)

        expanded: list[str] = []
        for index, name in enumerate(names):
            self.add_path_name(expanded, name)
            station = station_from_dock(name)
            if station is None or index + 1 >= len(names):
                continue
            staging = f"{station}_STAGING_CALIBRATED"
            if staging not in self.waypoints:
                staging = f"{station}_STAGING"
            if staging in self.waypoints and names[index + 1] != staging:
                self.add_path_name(expanded, staging)
        return expanded

    def pose_stamped_from_waypoint(self, name: str) -> PoseStamped:
        waypoint = self.waypoints[name]
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(waypoint["x"])
        pose.pose.position.y = float(waypoint["y"])
        pose.pose.position.z = 0.04
        qx, qy, qz, qw = yaw_to_quaternion(float(waypoint["yaw"]))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def build_planned_path(self) -> RosPath:
        path = RosPath()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        names = self.planned_route_names
        if names:
            path.poses.append(self.pose_stamped_from_waypoint(names[0]))
        for start, end in zip(names, names[1:]):
            if self.segment_name_for_pair(start, end) is None:
                self.get_logger().warn(
                    f"MISSION_PLANNED_PATH_UNLINKED {start}->{end}: no line segment definition; "
                    "showing direct connector only in planned path"
                )
            path.poses.extend(self.interpolated_pair_poses(start, end))
        self.get_logger().info("MISSION_PLANNED_PATH " + " -> ".join(names))
        return path

    def build_planned_markers(self) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 1
        stamp = self.get_clock().now().to_msg()

        for index, (start, end) in enumerate(zip(self.planned_route_names, self.planned_route_names[1:]), start=1):
            segment_ref = self.segment_name_for_pair(start, end)
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "mission_planned_segments"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.055
            marker.color.r = 1.0
            marker.color.g = 0.18 if segment_ref is not None else 0.75
            marker.color.b = 0.05
            marker.color.a = 0.95 if segment_ref is not None else 0.35
            marker.points = [self.waypoint_point(start, 0.07), self.waypoint_point(end, 0.07)]
            markers.markers.append(marker)

        for index, name in enumerate(self.planned_route_names, start=1):
            point_marker = Marker()
            point_marker.header.frame_id = self.frame_id
            point_marker.header.stamp = stamp
            point_marker.ns = "mission_planned_points"
            point_marker.id = marker_id
            marker_id += 1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position = self.waypoint_point(name, 0.11)
            point_marker.scale.x = 0.13
            point_marker.scale.y = 0.13
            point_marker.scale.z = 0.05
            point_marker.color.r = 1.0
            point_marker.color.g = 0.75
            point_marker.color.b = 0.05
            point_marker.color.a = 0.90
            markers.markers.append(point_marker)

            label_marker = Marker()
            label_marker.header.frame_id = self.frame_id
            label_marker.header.stamp = stamp
            label_marker.ns = "mission_planned_labels"
            label_marker.id = marker_id
            marker_id += 1
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.position = self.waypoint_point(name, 0.34)
            label_marker.scale.z = 0.18
            label_marker.color.r = 1.0
            label_marker.color.g = 1.0
            label_marker.color.b = 1.0
            label_marker.color.a = 0.95
            label_marker.text = f"{index}: {name}"
            markers.markers.append(label_marker)
        return markers

    def amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        q = pose.orientation
        self.last_amcl_pose = (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(q.x, q.y, q.z, q.w),
        )
        self.last_amcl_wall_time = time.monotonic()

    def odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = pose.orientation
        self.last_odom_pose = (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(q.x, q.y, q.z, q.w),
        )
        self.last_odom_wall_time = time.monotonic()

    def ground_truth_odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = pose.orientation
        world_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        origin_x = float(self.get_parameter("gazebo_world_origin_x_m").value)
        origin_y = float(self.get_parameter("gazebo_world_origin_y_m").value)
        origin_yaw = float(self.get_parameter("gazebo_world_origin_yaw_rad").value)
        dx = float(pose.position.x) - origin_x
        dy = float(pose.position.y) - origin_y
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        self.last_ground_truth_route_pose = (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            math.atan2(
                math.sin(world_yaw - origin_yaw),
                math.cos(world_yaw - origin_yaw),
            ),
        )
        self.last_ground_truth_wall_time = time.monotonic()

    def current_map_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            transform = None
        except Exception:
            transform = None
        if transform is not None:
            t = transform.transform.translation
            q = transform.transform.rotation
            return (float(t.x), float(t.y), quaternion_to_yaw(q.x, q.y, q.z, q.w))
        if time.monotonic() - self.last_amcl_wall_time <= 1.0:
            return self.last_amcl_pose
        return None

    def current_actual_path_pose(self) -> tuple[float, float, float] | None:
        if (
            self.last_ground_truth_route_pose is not None
            and time.monotonic() - self.last_ground_truth_wall_time <= 1.0
        ):
            return self.last_ground_truth_route_pose
        if self.last_odom_pose is None or time.monotonic() - self.last_odom_wall_time > 1.0:
            return None
        if self.actual_anchor_map_pose is None or self.actual_anchor_odom_pose is None:
            map_pose = self.current_map_pose()
            if map_pose is None:
                return None
            self.actual_anchor_map_pose = map_pose
            self.actual_anchor_odom_pose = self.last_odom_pose
            self.get_logger().info(
                "MISSION_ACTUAL_PATH_ODOM_ANCHOR "
                f"map=({map_pose[0]:.3f},{map_pose[1]:.3f},{map_pose[2]:.3f}) "
                f"odom=({self.last_odom_pose[0]:.3f},{self.last_odom_pose[1]:.3f},"
                f"{self.last_odom_pose[2]:.3f})"
            )

        anchor_map = self.actual_anchor_map_pose
        anchor_odom = self.actual_anchor_odom_pose
        odom = self.last_odom_pose
        yaw_offset = anchor_map[2] - anchor_odom[2]
        dx = odom[0] - anchor_odom[0]
        dy = odom[1] - anchor_odom[1]
        cos_yaw = math.cos(yaw_offset)
        sin_yaw = math.sin(yaw_offset)
        return (
            float(anchor_map[0] + cos_yaw * dx - sin_yaw * dy),
            float(anchor_map[1] + sin_yaw * dx + cos_yaw * dy),
            float(math.atan2(math.sin(odom[2] + yaw_offset), math.cos(odom[2] + yaw_offset))),
        )

    def append_actual_pose(self, pose: tuple[float, float, float]) -> None:
        min_distance = float(self.get_parameter("actual_path_min_distance_m").value)
        if self.actual_path.poses:
            last = self.actual_path.poses[-1].pose.position
            if math.hypot(float(last.x) - pose[0], float(last.y) - pose[1]) < min_distance:
                return
        stamped = PoseStamped()
        stamped.header.frame_id = self.frame_id
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose.position.x = pose[0]
        stamped.pose.position.y = pose[1]
        stamped.pose.position.z = 0.08
        qx, qy, qz, qw = yaw_to_quaternion(pose[2])
        stamped.pose.orientation.x = qx
        stamped.pose.orientation.y = qy
        stamped.pose.orientation.z = qz
        stamped.pose.orientation.w = qw
        self.actual_path.poses.append(stamped)
        max_points = int(self.get_parameter("actual_path_max_points").value)
        if len(self.actual_path.poses) > max_points:
            self.actual_path.poses = self.actual_path.poses[-max_points:]

    def publish_paths(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.planned_path.header.stamp = stamp
        for pose in self.planned_path.poses:
            pose.header.stamp = stamp
        self.planned_path_pub.publish(self.planned_path)
        for marker in self.planned_markers.markers:
            marker.header.stamp = stamp
        self.planned_markers_pub.publish(self.planned_markers)

        pose = self.current_actual_path_pose()
        if pose is not None:
            self.append_actual_pose(pose)
        self.actual_path.header.stamp = stamp
        self.actual_path_pub.publish(self.actual_path)


def main() -> int:
    rclpy.init()
    node = MissionPathPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
