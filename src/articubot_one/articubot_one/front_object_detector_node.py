#!/usr/bin/env python3

import json
from typing import Any, Dict, List, Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


EXPECTED_CLASSES = [
    "person",
    "cell phone",
    "phone",
    "laptop",
    "mouse",
    "pallet",
    "load",
    "obstacle",
]


class FrontObjectDetector(Node):
    def __init__(self) -> None:
        super().__init__("front_object_detector")

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/front_detections")
        self.declare_parameter("debug_image_topic", "/front_detection/debug_image")
        self.declare_parameter("safety_stop_topic", "/forklift_safety/stop")
        self.declare_parameter("safety_status_topic", "/forklift_safety/status")
        self.declare_parameter("safety_cmd_topic", "/cmd_vel_safety")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("safety_confidence_threshold", 0.45)
        self.declare_parameter("safety_bbox_height_ratio", 0.28)
        self.declare_parameter("publish_zero_cmd_on_safety", True)

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self.safety_status_topic = str(self.get_parameter("safety_status_topic").value)
        self.safety_cmd_topic = str(self.get_parameter("safety_cmd_topic").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.safety_confidence_threshold = float(
            self.get_parameter("safety_confidence_threshold").value
        )
        self.safety_bbox_height_ratio = float(
            self.get_parameter("safety_bbox_height_ratio").value
        )
        self.publish_zero_cmd_on_safety = bool(
            self.get_parameter("publish_zero_cmd_on_safety").value
        )

        self.bridge = CvBridge()
        self.detector_backend = "opencv_hog_people"
        self.supported_classes = ["person"]
        self.unsupported_classes = [
            cls for cls in EXPECTED_CLASSES if cls not in self.supported_classes
        ]
        self.yolo_model = self.try_load_yolo(self.model_path)

        if self.yolo_model is None:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        else:
            self.hog = None

        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        self.safety_status_pub = self.create_publisher(String, self.safety_status_topic, 10)
        self.safety_cmd_pub = self.create_publisher(Twist, self.safety_cmd_topic, 10)
        self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Front detector using {self.detector_backend}. Supported classes: "
            f"{', '.join(self.supported_classes)}. Unsupported without a YOLO/custom model: "
            f"{', '.join(self.unsupported_classes)}"
        )

    def try_load_yolo(self, model_path: str) -> Optional[Any]:
        if not model_path:
            return None

        try:
            from ultralytics import YOLO
        except Exception as exc:
            self.get_logger().warn(
                f"YOLO model requested but ultralytics is unavailable: {exc}"
            )
            return None

        try:
            model = YOLO(model_path)
        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLO model '{model_path}': {exc}")
            return None

        names = getattr(model, "names", {})
        supported = set()
        for name in names.values() if isinstance(names, dict) else names:
            label = str(name)
            if label in EXPECTED_CLASSES:
                supported.add(label)
            if label == "cell phone":
                supported.add("phone")

        self.detector_backend = "ultralytics_yolo"
        self.supported_classes = sorted(supported)
        self.unsupported_classes = [
            cls for cls in EXPECTED_CLASSES if cls not in self.supported_classes
        ]
        return model

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert front camera image: {exc}")
            return

        detections = self.detect(frame)
        safety_stop = self.should_stop_for_safety(detections, frame.shape[0])
        debug = self.draw_debug(frame, detections, safety_stop)

        payload = {
            "backend": self.detector_backend,
            "supported_classes": self.supported_classes,
            "unsupported_classes": self.unsupported_classes,
            "detections": detections,
            "safety_stop": safety_stop,
        }
        msg_out = String()
        msg_out.data = json.dumps(payload, separators=(",", ":"))
        self.detections_pub.publish(msg_out)

        stop_msg = Bool()
        stop_msg.data = safety_stop
        self.safety_stop_pub.publish(stop_msg)

        status_msg = String()
        status_msg.data = "STOP" if safety_stop else "CLEAR"
        self.safety_status_pub.publish(status_msg)

        if safety_stop and self.publish_zero_cmd_on_safety:
            self.safety_cmd_pub.publish(Twist())

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert front debug image: {exc}")
            return

        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

    def detect(self, frame) -> List[Dict[str, Any]]:
        if self.yolo_model is not None:
            return self.detect_yolo(frame)
        return self.detect_hog_people(frame)

    def detect_yolo(self, frame) -> List[Dict[str, Any]]:
        results = self.yolo_model.predict(
            frame,
            conf=self.confidence_threshold,
            verbose=False,
        )
        detections: List[Dict[str, Any]] = []
        if not results:
            return detections

        names = getattr(self.yolo_model, "names", {})
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                label = str(names.get(cls_id, cls_id)) if isinstance(names, dict) else str(cls_id)
                confidence = float(box.conf[0])
                if confidence < self.confidence_threshold:
                    continue
                x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
                detections.append(
                    {
                        "class": label,
                        "confidence": confidence,
                        "bbox": [x1, y1, x2, y2],
                    }
                )
        return detections

    def detect_hog_people(self, frame) -> List[Dict[str, Any]]:
        resized = frame
        height, width = frame.shape[:2]
        scale = 1.0
        if width > 640:
            scale = 640.0 / float(width)
            resized = cv2.resize(frame, None, fx=scale, fy=scale)

        rects, weights = self.hog.detectMultiScale(
            resized,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )

        detections: List[Dict[str, Any]] = []
        for rect, weight in zip(rects, weights):
            confidence = float(weight)
            if confidence < self.confidence_threshold:
                continue
            x, y, w, h = [int(v / scale) for v in rect]
            detections.append(
                {
                    "class": "person",
                    "confidence": confidence,
                    "bbox": [x, y, x + w, y + h],
                }
            )
        return detections

    def should_stop_for_safety(self, detections: List[Dict[str, Any]], image_height: int) -> bool:
        for detection in detections:
            label = detection.get("class")
            confidence = float(detection.get("confidence", 0.0))
            if label not in ("person", "obstacle"):
                continue
            if confidence < self.safety_confidence_threshold:
                continue
            _, y1, _, y2 = detection["bbox"]
            bbox_height_ratio = max(0.0, float(y2 - y1) / max(float(image_height), 1.0))
            if bbox_height_ratio >= self.safety_bbox_height_ratio:
                return True
        return False

    def draw_debug(self, frame, detections: List[Dict[str, Any]], safety_stop: bool):
        debug = frame.copy()
        for detection in detections:
            x1, y1, x2, y2 = detection["bbox"]
            color = (0, 0, 255) if safety_stop else (0, 255, 0)
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            label = f"{detection['class']} {float(detection['confidence']):.2f}"
            cv2.putText(
                debug,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        status = "SAFETY STOP" if safety_stop else self.detector_backend
        cv2.putText(
            debug,
            status,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 255) if safety_stop else (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        return debug


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
