#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import Counter
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformException, TransformListener


class BringupReadyChecker(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(
            "bringup_ready_checker",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.args = args
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.controller_client = self.create_client(ListControllers, "/controller_manager/list_controllers")

        self.clock_samples_wall: list[float] = []
        self.last_clock_time_sec: float | None = None
        self.clock_jump_back_detected = False
        self.clock_jump_count = 0
        self.last_clock_jump_wall_time: float | None = None
        self.have_odom = False
        self.have_amcl_pose = False
        self.localization_pose_samples: list[tuple[float, float, float, float]] = []
        self._start_wall_time = time.monotonic()
        self._clock_jump_grace_sec = 20.0
        self._clock_warmup_timeout_sec = 45.0
        self._critical_duplicate_names = {
            "/amcl",
            "/controller_manager",
            "/controller_server",
            "/fork_controller",
            "/gazebo_pose_line_controller",
            "/map_server",
            "/odom_relay",
            "/qr_reader",
            "/robot_state_publisher",
            "/ros_gz_bridge",
            "/twist_mux",
            "/twist_stamper",
            "/velocity_smoother",
            "/planner_server",
            "/behavior_server",
            "/bt_navigator",
            "/waypoint_follower",
            "/bringup_ready_checker",
            "/route_executor",
        }
        self.last_readiness_snapshot: dict[str, object] = {}
        self.last_failure_details: list[str] = ["startup_waiting"]

        self.create_subscription(Clock, "/clock", self.clock_cb, 100)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 50)
        self.create_subscription(Odometry, "/diff_cont/odom", self.odom_cb, 50)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_cb, 20)

    def clock_cb(self, msg: Clock) -> None:
        now_sec = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if self.last_clock_time_sec is not None and now_sec + 1e-6 < self.last_clock_time_sec:
            self.clock_jump_count += 1
            self.last_clock_jump_wall_time = time.monotonic()
            if time.monotonic() - self._start_wall_time > self._clock_jump_grace_sec:
                self.clock_jump_back_detected = True
        self.last_clock_time_sec = now_sec
        self.clock_samples_wall.append(time.monotonic())
        if len(self.clock_samples_wall) > 200:
            self.clock_samples_wall = self.clock_samples_wall[-200:]

    def odom_cb(self, _msg: Odometry) -> None:
        self.have_odom = True

    def amcl_pose_cb(self, _msg: PoseWithCovarianceStamped) -> None:
        self.have_amcl_pose = True

    def clock_hz(self) -> float:
        if len(self.clock_samples_wall) < 2:
            return 0.0
        span = self.clock_samples_wall[-1] - self.clock_samples_wall[0]
        if span <= 1e-6:
            return 0.0
        return (len(self.clock_samples_wall) - 1) / span

    def tf_available(self, target: str, source: str) -> bool:
        try:
            return bool(
                self.tf_buffer.can_transform(
                    target,
                    source,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
            )
        except TransformException:
            return False
        except Exception:
            return False

    def map_base_pose(self) -> tuple[float, float, float] | None:
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
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return float(translation.x), float(translation.y), math.atan2(siny_cosp, cosy_cosp)

    def localization_stability(self, pose: tuple[float, float, float] | None) -> dict[str, object]:
        now = time.monotonic()
        if pose is not None:
            self.localization_pose_samples.append((now, pose[0], pose[1], pose[2]))
        self.localization_pose_samples = [
            sample for sample in self.localization_pose_samples if now - sample[0] <= 10.0
        ]
        window_sec = max(2.0, float(self.args.stable_sec))
        recent = [sample for sample in self.localization_pose_samples if now - sample[0] <= window_sec]
        if len(recent) < 8:
            return {
                "localization_pose_stable": False,
                "localization_sample_count": len(recent),
                "localization_sample_span_sec": 0.0,
                "localization_max_translation_m": None,
                "localization_max_yaw_delta_rad": None,
            }
        sample_span = recent[-1][0] - recent[0][0]
        reference = recent[-1]
        max_translation = max(
            math.hypot(sample[1] - reference[1], sample[2] - reference[2])
            for sample in recent
        )
        max_yaw_delta = max(
            abs(math.atan2(math.sin(sample[3] - reference[3]), math.cos(sample[3] - reference[3])))
            for sample in recent
        )
        stable = bool(
            sample_span >= window_sec * 0.80
            and max_translation <= 0.05
            and max_yaw_delta <= 0.08
        )
        return {
            "localization_pose_stable": stable,
            "localization_sample_count": len(recent),
            "localization_sample_span_sec": sample_span,
            "localization_max_translation_m": max_translation,
            "localization_max_yaw_delta_rad": max_yaw_delta,
        }

    def active_controllers(self) -> tuple[bool, bool, str]:
        if not self.controller_client.wait_for_service(timeout_sec=0.05):
            return False, False, "controller_manager_unavailable"
        future = self.controller_client.call_async(ListControllers.Request())
        end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return False, False, "controller_manager_timeout"
        result = future.result()
        if result is None:
            return False, False, "controller_manager_no_result"
        diff_active = False
        joint_active = False
        for controller in result.controller:
            name = controller.name
            state = controller.state.lower()
            if name == "diff_cont" and state == "active":
                diff_active = True
            if name == "joint_broad" and state == "active":
                joint_active = True
        return diff_active, joint_active, ""

    def nav_ready(self) -> bool:
        return self.nav_client.wait_for_server(timeout_sec=0.05)

    def clock_publisher_count(self) -> int:
        try:
            infos = self.get_publishers_info_by_topic("/clock")
            return len(infos)
        except Exception:
            return 0

    def odom_publisher_count(self) -> int:
        try:
            infos = self.get_publishers_info_by_topic("/odom")
            return len(infos)
        except Exception:
            return 0

    def joint_states_publisher_count(self) -> int:
        try:
            infos = self.get_publishers_info_by_topic("/joint_states")
            return len(infos)
        except Exception:
            return 0

    def duplicate_critical_nodes(self) -> list[str]:
        try:
            names_and_namespaces = self.get_node_names_and_namespaces()
        except Exception:
            return []
        full_names: list[str] = []
        for name, namespace in names_and_namespaces:
            ns = namespace or "/"
            if not ns.startswith("/"):
                ns = "/" + ns
            if ns != "/" and ns.endswith("/"):
                ns = ns[:-1]
            full_name = f"{ns}/{name}" if ns != "/" else f"/{name}"
            full_names.append(full_name)
        counts = Counter(full_names)
        return sorted(
            name
            for name, count in counts.items()
            if count > 1 and name in self._critical_duplicate_names
        )

    def controller_states(self) -> dict[str, str]:
        if not self.controller_client.wait_for_service(timeout_sec=0.05):
            return {}
        future = self.controller_client.call_async(ListControllers.Request())
        end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return {}
        result = future.result()
        if result is None:
            return {}
        return {controller.name: controller.state.lower() for controller in result.controller}

    def readiness_snapshot(self) -> dict[str, object]:
        diff_active, joint_active, controller_reason = self.active_controllers()
        duplicate_nodes = self.duplicate_critical_nodes()
        clock_pub_count = self.clock_publisher_count()
        last_jump_age_sec = (
            None
            if self.last_clock_jump_wall_time is None
            else max(0.0, time.monotonic() - self.last_clock_jump_wall_time)
        )
        map_base_pose = self.map_base_pose()
        localization_stability = self.localization_stability(map_base_pose)
        return {
            "controller_manager": controller_reason == "",
            "controller_reason": controller_reason,
            "diff_cont_active": diff_active,
            "joint_broad_active": joint_active,
            "joint_states_publishing": self.joint_states_publisher_count() > 0,
            "odom_publishing": self.have_odom,
            "amcl_pose_publishing": self.have_amcl_pose,
            "map_to_odom_tf": self.tf_available("map", "odom"),
            "odom_to_base_tf": self.tf_available("odom", "base_link"),
            "map_to_base_tf": map_base_pose is not None,
            "map_base_pose": list(map_base_pose) if map_base_pose is not None else None,
            "clock_ok": self.last_clock_time_sec is not None and clock_pub_count == 1,
            "clock_hz": self.clock_hz(),
            "clock_publisher_count": clock_pub_count,
            "clock_jump_count": self.clock_jump_count,
            "last_clock_jump_age_sec": last_jump_age_sec,
            "clock_jump_back_detected": self.clock_jump_back_detected,
            "controller_manager_service_found": controller_reason != "controller_manager_unavailable",
            "controller_states": self.controller_states(),
            "odom_publisher_count": self.odom_publisher_count(),
            "joint_states_publisher_count": self.joint_states_publisher_count(),
            "navigate_to_pose_ready": self.nav_ready(),
            "duplicate_critical_nodes": duplicate_nodes,
            "duplicate_critical_nodes_present": bool(duplicate_nodes),
            **localization_stability,
        }

    def failed_dependencies(self, snapshot: dict[str, object]) -> list[str]:
        failed: list[str] = []
        line_source = str(getattr(self.args, "line_source", "") or "").strip().lower()
        geometry_line_sensor = line_source == "line_sensor"
        duplicate_nodes = list(snapshot.get("duplicate_critical_nodes", []))
        duplicate_nodes = [name for name in duplicate_nodes if name != "/controller_manager"]
        if duplicate_nodes:
            duplicates = ",".join(duplicate_nodes)
            failed.append(f"duplicate_critical_nodes:{duplicates}")
        clock_pub_count = int(snapshot.get("clock_publisher_count", 0) or 0)
        if clock_pub_count != 1:
            failed.append(f"duplicate_clock_publishers:{clock_pub_count}")
        if not snapshot["controller_manager"]:
            failed.append(str(snapshot["controller_reason"]))
        if not snapshot["diff_cont_active"]:
            failed.append("diff_cont_inactive")
        if not snapshot["joint_broad_active"] and not snapshot.get("joint_states_publishing", False):
            failed.append("joint_broad_inactive")
        if not snapshot["odom_publishing"]:
            failed.append("odom_not_publishing")
        if not snapshot["amcl_pose_publishing"] and not snapshot["map_to_odom_tf"]:
            failed.append("amcl_pose_not_publishing")
        if not snapshot["odom_to_base_tf"]:
            failed.append("tf_missing_odom_to_base_link")
        if not snapshot["map_to_odom_tf"]:
            failed.append("tf_missing_map_to_odom")
        if geometry_line_sensor and not snapshot.get("map_to_base_tf", False):
            failed.append("tf_missing_map_to_base_link_for_geometry_sensor")
        # AMCL can keep /amcl_pose silent until its first motion update. The
        # geometry sensor consumes the equivalent map->base_link TF directly.
        if geometry_line_sensor and not snapshot.get("localization_pose_stable", False):
            failed.append("localization_pose_not_stable")
        if not snapshot["clock_ok"]:
            failed.append("clock_not_stable")
        if not snapshot["navigate_to_pose_ready"]:
            failed.append("navigate_to_pose_unavailable")
        return failed

    def write_failure_outputs(self, failure_reason: str, snapshot: dict[str, object], failed: list[str]) -> None:
        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        summary = {
            "route": self.args.route,
            "success": False,
            "reached": [],
            "skipped_docks": [],
            "last_successful_segment": None,
            "first_failed_segment": None,
            "failure_reason": failure_reason,
            "diagnostic_results": [],
            "bringup_readiness": {
                "failed_conditions": failed,
                **snapshot,
            },
        }
        (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        (output_dir / "failure_reason.txt").write_text(failure_reason + "\n", encoding="utf-8")

    def wait_until_ready(self) -> tuple[bool, list[str]]:
        deadline = time.monotonic() + self.args.timeout_sec
        stable_since: float | None = None
        last_failed = ["startup_waiting"]
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            snapshot = self.readiness_snapshot()
            self.last_readiness_snapshot = snapshot
            failed = self.failed_dependencies(snapshot)
            jump_age = snapshot.get("last_clock_jump_age_sec")
            if (
                self.clock_jump_back_detected
                and isinstance(jump_age, (int, float))
                and float(jump_age) < self.args.stable_sec
            ):
                failed.append("clock_recent_jump_reset")
            if self.clock_jump_back_detected and (
                time.monotonic() - self._start_wall_time
            ) > self._clock_warmup_timeout_sec:
                failed = ["unstable_clock_timeout"]
            if failed:
                stable_since = None
                last_failed = failed
                self.last_failure_details = failed
                continue
            if stable_since is None:
                stable_since = time.monotonic()
                continue
            if time.monotonic() - stable_since >= self.args.stable_sec:
                self.get_logger().info(
                    "Bringup ready: "
                    f"clock_hz={float(snapshot['clock_hz']):.1f} "
                    f"clock_publishers={int(snapshot['clock_publisher_count'])} "
                    f"clock_jump_count={int(snapshot['clock_jump_count'])} "
                    f"diff_cont=active joint_broad={'active' if snapshot['joint_broad_active'] else 'bridged'} "
                    f"odom={'yes' if snapshot['odom_publishing'] else 'no'} "
                    f"amcl={'yes' if snapshot['amcl_pose_publishing'] else 'tf_fallback'} "
                    f"tf={'yes' if snapshot['map_to_odom_tf'] and snapshot['odom_to_base_tf'] else 'no'} "
                    f"nav2={'yes' if snapshot['navigate_to_pose_ready'] else 'no'}"
                )
                return True, []
        return False, last_failed

    def exec_route_executor(self) -> None:
        cmd = [
            "ros2",
            "run",
            "articubot_one",
            "route_executor",
            "--route",
            self.args.route,
            "--execute",
            "--competition",
            "--output-dir",
            self.args.output_dir,
        ]
        if self.args.resume_from_current:
            cmd.append("--resume-from-current")
        if self.args.start_from_start:
            cmd.append("--start-from-start")
        if self.args.line_source:
            cmd.extend(["--line-source", self.args.line_source])
        cmd.extend(
            [
                "--line-approach-cross-track-threshold-m",
                f"{float(self.args.line_approach_cross_track_threshold_m):.3f}",
            ]
        )
        if self.args.stop_after:
            cmd.extend(["--stop-after", self.args.stop_after])
        self.get_logger().info("Starting route_executor after bringup readiness passes")
        os.execvp(cmd[0], cmd)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Wait for sim/TF/controller bringup, then exec route_executor.")
    parser.add_argument("--route", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--resume-from-current", action="store_true")
    parser.add_argument("--start-from-start", action="store_true")
    parser.add_argument("--line-source", default="pose_line")
    parser.add_argument("--stop-after", default="")
    parser.add_argument("--line-follower-params", default="")
    parser.add_argument("--line-sensor-params", default="")
    parser.add_argument("--line-segments-file", default="")
    parser.add_argument("--route-waypoints-file", default="")
    parser.add_argument("--line-approach-cross-track-threshold-m", type=float, default=0.05)
    parser.add_argument("--stable-sec", type=float, default=3.0)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = BringupReadyChecker(args)
    try:
        ok, failed = node.wait_until_ready()
        if not ok:
            snapshot = dict(node.last_readiness_snapshot)
            failure_reason = "bringup_not_ready:" + ",".join(failed)
            node.write_failure_outputs(failure_reason, snapshot, failed)
            node.get_logger().error(
                "Bringup readiness failed: "
                + ", ".join(failed)
                + " snapshot="
                + json.dumps(snapshot, ensure_ascii=True)
            )
            return 1
        node.exec_route_executor()
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
