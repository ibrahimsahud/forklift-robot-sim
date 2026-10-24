#!/usr/bin/env python3

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import math
import shutil
import tempfile
from typing import Deque, Dict, List, Optional, Tuple

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from articubot_one.qr_detection import QRDetection, detect_qr_codes


VALID_QR_VALUES = {
    "START",
    "A1",
    "A2",
    "A3",
    "GATE_OUT",
    "GATE_IN",
    "B1",
    "B2",
    "B3",
}


def _default_output_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "qr_map_points.yaml")


@dataclass
class PoseData:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass
class Observation:
    stamp_sec: float
    robot_pose: PoseData
    camera_pose: PoseData
    qr_pose: Optional[PoseData]
    quality: float
    width_px: float
    reprojection_error_px: Optional[float]
    camera_frame: str


class QRTrack:
    def __init__(self) -> None:
        self.observations: Deque[Observation] = deque()
        self.seen_count = 0
        self.rejected_count = 0
        self.last_saved_quality = -1.0
        self.last_saved_source = ""


class QRWaypointLogger(Node):
    def __init__(self) -> None:
        super().__init__("qr_waypoint_logger")

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_link_optical")
        self.declare_parameter("qr_size_m", 0.20)
        self.declare_parameter("qr_forward_offset_m", 0.10)
        self.declare_parameter("min_qr_width_px", 20.0)
        self.declare_parameter("min_stable_detections", 5)
        self.declare_parameter("min_pose_detections", 3)
        self.declare_parameter("sample_window_sec", 45.0)
        self.declare_parameter("staging_position_tolerance_m", 0.40)
        self.declare_parameter("qr_position_tolerance_m", 0.35)
        self.declare_parameter("max_reprojection_error_px", 10.0)
        self.declare_parameter("max_qr_range_m", 4.0)
        self.declare_parameter("min_quality_score", 0.25)
        self.declare_parameter("save_path", _default_output_path())
        self.declare_parameter("backup_existing_file", True)
        self.declare_parameter("debug_logging", False)

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.default_camera_frame = str(self.get_parameter("camera_frame").value)
        self.qr_size_m = float(self.get_parameter("qr_size_m").value)
        self.qr_forward_offset_m = float(self.get_parameter("qr_forward_offset_m").value)
        self.min_qr_width_px = float(self.get_parameter("min_qr_width_px").value)
        self.min_stable_detections = int(self.get_parameter("min_stable_detections").value)
        self.min_pose_detections = int(self.get_parameter("min_pose_detections").value)
        self.sample_window_sec = float(self.get_parameter("sample_window_sec").value)
        self.staging_position_tolerance_m = float(
            self.get_parameter("staging_position_tolerance_m").value
        )
        self.qr_position_tolerance_m = float(
            self.get_parameter("qr_position_tolerance_m").value
        )
        self.max_reprojection_error_px = float(
            self.get_parameter("max_reprojection_error_px").value
        )
        self.max_qr_range_m = float(self.get_parameter("max_qr_range_m").value)
        self.min_quality_score = float(self.get_parameter("min_quality_score").value)
        self.save_path = Path(str(self.get_parameter("save_path").value))
        self.backup_existing_file = bool(self.get_parameter("backup_existing_file").value)
        self.debug_logging = bool(self.get_parameter("debug_logging").value)

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_info_msg: Optional[CameraInfo] = None
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.warned_missing_camera_info = False
        self.warned_camera_frame_alias = False
        self.backup_written = False

        self.tracks: Dict[str, QRTrack] = {qr_id: QRTrack() for qr_id in VALID_QR_VALUES}
        self.saved_data = self._load_existing_file()

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"QR waypoint logger subscribed to {self.camera_topic} and "
            f"{self.camera_info_topic}, saving to {self.save_path}"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info_msg = msg
        if len(msg.k) == 9:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if msg.d:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert camera image: {exc}")
            return

        detections = detect_qr_codes(frame, enable_preprocessing=True, detector=self.detector)
        if not detections:
            return

        stamp = Time.from_msg(msg.header.stamp)
        image_height, image_width = frame.shape[:2]
        camera_frame = self._camera_frame_for_image(msg, stamp)

        robot_pose = self._lookup_pose(self.map_frame, self.base_frame, stamp)
        camera_pose = self._lookup_pose(self.map_frame, camera_frame, stamp)
        if robot_pose is None or camera_pose is None:
            return

        for detection in detections:
            qr_id = detection.data.strip().upper()
            if qr_id not in VALID_QR_VALUES:
                continue
            if detection.width_px < self.min_qr_width_px:
                continue

            qr_pose, reprojection_error_px = self._estimate_qr_pose_map(
                detection,
                stamp,
                camera_frame,
            )
            quality = self._compute_quality(
                detection,
                image_width,
                image_height,
                qr_pose is not None,
                reprojection_error_px,
            )
            if quality < self.min_quality_score:
                self.tracks[qr_id].rejected_count += 1
                continue

            observation = Observation(
                stamp_sec=stamp.nanoseconds / 1_000_000_000.0,
                robot_pose=robot_pose,
                camera_pose=camera_pose,
                qr_pose=qr_pose,
                quality=quality,
                width_px=detection.width_px,
                reprojection_error_px=reprojection_error_px,
                camera_frame=camera_frame,
            )
            self._record_observation(qr_id, observation)

    def _camera_frame_for_image(self, msg: Image, stamp: Time) -> str:
        frame_candidates: List[str] = [self.default_camera_frame]
        if self.camera_info_msg and self.camera_info_msg.header.frame_id:
            frame_candidates.append(self.camera_info_msg.header.frame_id)
        if msg.header.frame_id:
            frame_candidates.append(msg.header.frame_id)

        seen = set()
        unique_candidates = []
        for frame in frame_candidates:
            if frame and frame not in seen:
                seen.add(frame)
                unique_candidates.append(frame)

        for frame in unique_candidates:
            if self._lookup_transform(self.map_frame, frame, stamp) is not None:
                if frame != self.default_camera_frame and not self.warned_camera_frame_alias:
                    self.warned_camera_frame_alias = True
                    self.get_logger().info(
                        f"Using camera TF frame '{frame}' from camera messages."
                    )
                return frame

        return self.default_camera_frame

    def _lookup_pose(
        self,
        target_frame: str,
        source_frame: str,
        stamp: Time,
    ) -> Optional[PoseData]:
        transform = self._lookup_transform(target_frame, source_frame, stamp)
        if transform is None:
            return None
        tr = transform.transform.translation
        rot = transform.transform.rotation
        return PoseData(
            x=float(tr.x),
            y=float(tr.y),
            z=float(tr.z),
            qx=float(rot.x),
            qy=float(rot.y),
            qz=float(rot.z),
            qw=float(rot.w),
        )

    def _lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        stamp: Time,
    ):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=0.15),
            )
        except TransformException:
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.15),
                )
            except TransformException as exc:
                if self.debug_logging:
                    self.get_logger().warn(
                        f"TF lookup failed for {target_frame} <- {source_frame}: {exc}"
                    )
                return None

    def _estimate_qr_pose_map(
        self,
        detection: QRDetection,
        stamp: Time,
        camera_frame: str,
    ) -> Tuple[Optional[PoseData], Optional[float]]:
        if self.camera_matrix is None or self.dist_coeffs is None:
            return None, None

        half = self.qr_size_m / 2.0
        object_points = np.array(
            [
                [-half, -half, 0.0],
                [half, -half, 0.0],
                [half, half, 0.0],
                [-half, half, 0.0],
            ],
            dtype=np.float64,
        )
        image_points = detection.points.astype(np.float64)

        methods = [cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE]
        best_pose: Optional[PoseData] = None
        best_error: Optional[float] = None

        map_from_camera = self._lookup_transform(self.map_frame, camera_frame, stamp)
        if map_from_camera is None:
            return None, None
        map_from_camera_mat = transform_to_matrix(map_from_camera)

        for method in methods:
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=method,
                )
            except cv2.error:
                continue

            if not ok:
                continue

            qr_range_m = float(np.linalg.norm(tvec.reshape(3)))
            if qr_range_m <= 0.0 or qr_range_m > self.max_qr_range_m:
                continue

            reprojection_error_px = compute_reprojection_error(
                object_points,
                image_points,
                rvec,
                tvec,
                self.camera_matrix,
                self.dist_coeffs,
            )
            if reprojection_error_px > self.max_reprojection_error_px:
                continue

            camera_from_qr = np.eye(4, dtype=np.float64)
            camera_from_qr[:3, :3], _ = cv2.Rodrigues(rvec)
            camera_from_qr[:3, 3] = tvec.reshape(3)

            map_from_qr = map_from_camera_mat @ camera_from_qr
            pose = matrix_to_pose(map_from_qr)

            if best_error is None or reprojection_error_px < best_error:
                best_pose = pose
                best_error = reprojection_error_px

        return best_pose, best_error

    def _compute_quality(
        self,
        detection: QRDetection,
        image_width: int,
        image_height: int,
        has_qr_pose: bool,
        reprojection_error_px: Optional[float],
    ) -> float:
        width_score = clamp(detection.width_px / 220.0, 0.0, 1.0)
        error_x = abs(detection.error_x(image_width))
        center_x_score = 1.0 - clamp(error_x, 0.0, 1.0)
        center_y = abs((detection.cy - (image_height / 2.0)) / max(image_height / 2.0, 1.0))
        center_y_score = 1.0 - clamp(center_y, 0.0, 1.0)
        reproj_score = 0.0
        if reprojection_error_px is not None:
            reproj_score = 1.0 - clamp(
                reprojection_error_px / max(self.max_reprojection_error_px, 1e-6),
                0.0,
                1.0,
            )
        pose_bonus = 0.15 if has_qr_pose else 0.0
        return clamp(
            0.45 * width_score + 0.25 * center_x_score + 0.15 * center_y_score
            + 0.15 * reproj_score + pose_bonus,
            0.0,
            1.0,
        )

    def _record_observation(self, qr_id: str, observation: Observation) -> None:
        track = self.tracks[qr_id]
        track.seen_count += 1

        now_sec = observation.stamp_sec
        while track.observations and now_sec - track.observations[0].stamp_sec > self.sample_window_sec:
            track.observations.popleft()

        if self._is_outlier(track, observation):
            track.rejected_count += 1
            return

        track.observations.append(observation)
        entry = self._build_entry(qr_id, track)
        if entry is None:
            return

        existing_entry = self.saved_data.setdefault("qrs", {}).get(qr_id)
        if not self._should_replace(existing_entry, entry):
            return

        self.saved_data["qrs"][qr_id] = entry
        self.saved_data["metadata"]["updated_at_ros"] = iso_timestamp(now_sec)
        self.saved_data["metadata"]["updated_at_wall_utc"] = wall_timestamp()
        self._write_yaml_file()

        source = entry["pose_source"]
        self.get_logger().info(
            f"Saved QR {qr_id} using {source} with {entry['accepted_sample_count']} "
            f"inlier detections."
        )

    def _is_outlier(self, track: QRTrack, observation: Observation) -> bool:
        accepted = list(track.observations)
        if len(accepted) < 3:
            return False

        robot_positions = np.array([[obs.robot_pose.x, obs.robot_pose.y] for obs in accepted])
        robot_median = np.median(robot_positions, axis=0)
        robot_distance = float(
            np.linalg.norm(
                np.array([observation.robot_pose.x, observation.robot_pose.y]) - robot_median
            )
        )
        if robot_distance > self.staging_position_tolerance_m:
            return True

        qr_positions = np.array(
            [
                [obs.qr_pose.x, obs.qr_pose.y, obs.qr_pose.z]
                for obs in accepted
                if obs.qr_pose is not None
            ],
            dtype=np.float64,
        )
        if observation.qr_pose is not None and len(qr_positions) >= 3:
            qr_median = np.median(qr_positions, axis=0)
            qr_distance = float(
                np.linalg.norm(
                    np.array([observation.qr_pose.x, observation.qr_pose.y, observation.qr_pose.z])
                    - qr_median
                )
            )
            if qr_distance > self.qr_position_tolerance_m:
                return True

        return False

    def _build_entry(self, qr_id: str, track: QRTrack) -> Optional[dict]:
        observations = list(track.observations)
        if len(observations) < self.min_stable_detections:
            return None

        robot_pose = average_pose([obs.robot_pose for obs in observations])
        camera_pose = average_pose([obs.camera_pose for obs in observations])
        qr_observations = [obs for obs in observations if obs.qr_pose is not None]

        pose_source = "robot_staging_pose_map_only"
        qr_pose_data = None
        if len(qr_observations) >= self.min_pose_detections:
            pose_source = "estimated_qr_pose_map"
            qr_pose_data = average_pose([obs.qr_pose for obs in qr_observations if obs.qr_pose])

        quality_values = [obs.quality for obs in observations]
        width_values = [obs.width_px for obs in observations]
        reproj_values = [
            obs.reprojection_error_px
            for obs in qr_observations
            if obs.reprojection_error_px is not None
        ]
        latest_stamp = max(obs.stamp_sec for obs in observations)

        entry = {
            "qr_id": qr_id,
            "timestamp_ros": iso_timestamp(latest_stamp),
            "timestamp_ros_sec": round(float(latest_stamp), 3),
            "timestamp_wall_utc": wall_timestamp(),
            "seen_count": track.seen_count,
            "accepted_sample_count": len(observations),
            "rejected_outlier_count": track.rejected_count,
            "pose_source": pose_source,
            "camera_calibration_available": self.camera_matrix is not None,
            "quality_mean": round(float(np.mean(quality_values)), 4),
            "quality_last": round(float(quality_values[-1]), 4),
            "width_px_mean": round(float(np.mean(width_values)), 2),
            "robot_staging_pose_map": pose_to_dict(robot_pose),
            "camera_pose_map": pose_to_dict(camera_pose),
            "estimated_qr_pose_map": pose_to_dict(qr_pose_data) if qr_pose_data else None,
            "fallback_qr_forward_offset_m": (
                float(self.qr_forward_offset_m)
                if qr_pose_data is None
                else None
            ),
            "camera_frame": observations[-1].camera_frame,
            "reprojection_error_px_mean": (
                round(float(np.mean(reproj_values)), 4) if reproj_values else None
            ),
        }
        return entry

    def _should_replace(self, existing_entry: Optional[dict], new_entry: dict) -> bool:
        if existing_entry is None:
            return True

        existing_source = str(existing_entry.get("pose_source", ""))
        new_source = str(new_entry.get("pose_source", ""))
        if existing_source != "estimated_qr_pose_map" and new_source == "estimated_qr_pose_map":
            return True
        if existing_source == "estimated_qr_pose_map" and new_source != "estimated_qr_pose_map":
            return False

        old_quality = float(existing_entry.get("quality_mean", 0.0))
        old_samples = int(existing_entry.get("accepted_sample_count", 0))
        new_quality = float(new_entry.get("quality_mean", 0.0))
        new_samples = int(new_entry.get("accepted_sample_count", 0))
        return new_samples >= old_samples or new_quality > old_quality + 0.02

    def _load_existing_file(self) -> dict:
        data = {
            "metadata": {
                "map_frame": self.map_frame,
                "base_frame": self.base_frame,
                "camera_frame": self.default_camera_frame,
                "qr_size_m": self.qr_size_m,
                "qr_forward_offset_m": self.qr_forward_offset_m,
                "generated_by": "qr_waypoint_logger",
                "updated_at_ros": None,
                "updated_at_wall_utc": None,
            },
            "qrs": {},
        }

        if not self.save_path.exists():
            self.save_path.parent.mkdir(parents=True, exist_ok=True)
            return data

        try:
            loaded = yaml.safe_load(self.save_path.read_text()) or {}
        except Exception as exc:
            self.get_logger().warn(f"Failed to read existing {self.save_path}: {exc}")
            return data

        if isinstance(loaded, dict):
            metadata = loaded.get("metadata", {}) or {}
            metadata.pop("updated_at", None)
            data["metadata"].update(metadata)

            loaded_qrs = loaded.get("qrs", {}) or {}
            if isinstance(loaded_qrs, dict):
                for qr_id, entry in loaded_qrs.items():
                    if isinstance(entry, dict):
                        entry.pop("timestamp", None)
                data["qrs"].update(loaded_qrs)
        return data

    def _write_yaml_file(self) -> None:
        self.save_path.parent.mkdir(parents=True, exist_ok=True)

        if self.backup_existing_file and self.save_path.exists() and not self.backup_written:
            backup_name = (
                self.save_path.stem
                + "."
                + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                + ".bak"
                + self.save_path.suffix
            )
            backup_path = self.save_path.with_name(backup_name)
            shutil.copy2(self.save_path, backup_path)
            self.backup_written = True
            self.get_logger().info(f"Backed up existing QR map file to {backup_path}")

        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=str(self.save_path.parent),
            delete=False,
            suffix=".tmp",
        ) as handle:
            yaml.safe_dump(self.saved_data, handle, sort_keys=False, allow_unicode=False)
            temp_path = Path(handle.name)

        temp_path.replace(self.save_path)


def compute_reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    return float(np.mean(np.linalg.norm(projected - image_points, axis=1)))


def transform_to_matrix(transform_stamped) -> np.ndarray:
    tr = transform_stamped.transform.translation
    rot = transform_stamped.transform.rotation
    mat = quaternion_to_matrix(rot.x, rot.y, rot.z, rot.w)
    mat[:3, 3] = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
    return mat


def matrix_to_pose(matrix: np.ndarray) -> PoseData:
    qx, qy, qz, qw = matrix_to_quaternion(matrix[:3, :3])
    return PoseData(
        x=float(matrix[0, 3]),
        y=float(matrix[1, 3]),
        z=float(matrix[2, 3]),
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
    )


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(4, dtype=np.float64)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    matrix = np.eye(4, dtype=np.float64)
    matrix[0, 0] = 1.0 - 2.0 * (yy + zz)
    matrix[0, 1] = 2.0 * (xy - wz)
    matrix[0, 2] = 2.0 * (xz + wy)
    matrix[1, 0] = 2.0 * (xy + wz)
    matrix[1, 1] = 1.0 - 2.0 * (xx + zz)
    matrix[1, 2] = 2.0 * (yz - wx)
    matrix[2, 0] = 2.0 * (xz - wy)
    matrix[2, 1] = 2.0 * (yz + wx)
    matrix[2, 2] = 1.0 - 2.0 * (xx + yy)
    return matrix


def matrix_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    return normalize_quaternion(qx, qy, qz, qw)


def average_pose(poses: List[PoseData]) -> PoseData:
    xyz = np.mean([[pose.x, pose.y, pose.z] for pose in poses], axis=0)
    quats = np.array([[pose.qx, pose.qy, pose.qz, pose.qw] for pose in poses], dtype=np.float64)
    ref = quats[0]
    for idx in range(len(quats)):
        if np.dot(quats[idx], ref) < 0.0:
            quats[idx] *= -1.0
    quat = np.mean(quats, axis=0)
    qx, qy, qz, qw = normalize_quaternion(*quat)
    return PoseData(
        x=float(xyz[0]),
        y=float(xyz[1]),
        z=float(xyz[2]),
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
    )


def normalize_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float, float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def pose_to_dict(pose: Optional[PoseData]) -> Optional[dict]:
    if pose is None:
        return None
    yaw = yaw_from_quaternion(pose.qx, pose.qy, pose.qz, pose.qw)
    return {
        "x": float(round(float(pose.x), 4)),
        "y": float(round(float(pose.y), 4)),
        "z": float(round(float(pose.z), 4)),
        "qx": float(round(float(pose.qx), 6)),
        "qy": float(round(float(pose.qy), 6)),
        "qz": float(round(float(pose.qz), 6)),
        "qw": float(round(float(pose.qw), 6)),
        "yaw": float(round(float(yaw), 6)),
    }


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def iso_timestamp(stamp_sec: float) -> str:
    return datetime.fromtimestamp(stamp_sec, tz=timezone.utc).isoformat()


def wall_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[QRWaypointLogger] = None
    try:
        node = QRWaypointLogger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
