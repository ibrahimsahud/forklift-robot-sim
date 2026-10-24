from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Tuple

import cv2
import numpy as np


PointMapper = Callable[[np.ndarray], np.ndarray]


@dataclass
class QRDetection:
    data: str
    points: np.ndarray
    attempt: str
    width_px: float
    cx: float
    cy: float

    def error_x(self, image_width: int) -> float:
        image_center_x = image_width / 2.0
        if image_center_x <= 0.0:
            return 0.0
        return (self.cx - image_center_x) / image_center_x


def detect_qr_codes(
    image_bgr: np.ndarray,
    enable_preprocessing: bool = True,
    detector: Optional[cv2.QRCodeDetector] = None,
) -> List[QRDetection]:
    if detector is None:
        detector = cv2.QRCodeDetector()

    for attempt_name, attempt_image, point_mapper in build_detection_attempts(
        image_bgr,
        enable_preprocessing,
    ):
        detections = _detect_on_image(detector, attempt_image, attempt_name, point_mapper)
        if detections:
            return detections

    return []


def build_detection_attempts(
    image_bgr: np.ndarray,
    enable_preprocessing: bool,
) -> Iterable[Tuple[str, np.ndarray, PointMapper]]:
    height, width = image_bgr.shape[:2]
    identity = lambda points: points

    yield "original_bgr", image_bgr, identity

    if not enable_preprocessing:
        return

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    yield "grayscale", gray, identity

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    yield "blurred_gray", blurred, identity

    _, threshold = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    yield "threshold_otsu", threshold, identity

    adaptive = cv2.adaptiveThreshold(
        blurred,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        31,
        2,
    )
    yield "adaptive_threshold", adaptive, identity

    resized = cv2.resize(
        image_bgr,
        None,
        fx=2.0,
        fy=2.0,
        interpolation=cv2.INTER_CUBIC,
    )
    yield "resized_2x", resized, lambda points: points * 0.5

    yield (
        "rotated_90_cw",
        cv2.rotate(image_bgr, cv2.ROTATE_90_CLOCKWISE),
        lambda points: _map_rotated_points(points, width, height, cv2.ROTATE_90_CLOCKWISE),
    )
    yield (
        "rotated_180",
        cv2.rotate(image_bgr, cv2.ROTATE_180),
        lambda points: _map_rotated_points(points, width, height, cv2.ROTATE_180),
    )
    yield (
        "rotated_90_ccw",
        cv2.rotate(image_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE),
        lambda points: _map_rotated_points(
            points,
            width,
            height,
            cv2.ROTATE_90_COUNTERCLOCKWISE,
        ),
    )

    for index, warped_marker, marker_mapper in _build_floor_marker_warps(image_bgr):
        yield f"floor_marker_warp_{index}", warped_marker, marker_mapper


def draw_debug_image(
    image_bgr: np.ndarray,
    detections: List[QRDetection],
    min_width_px: float,
    max_width_px: float,
) -> np.ndarray:
    debug_image = image_bgr.copy()

    if not detections:
        cv2.putText(
            debug_image,
            "NO QR",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return debug_image

    for detection in detections:
        valid_width = min_width_px <= detection.width_px <= max_width_px
        color = (0, 255, 0) if valid_width else (0, 180, 255)
        points_i = np.round(detection.points).astype(np.int32)
        cv2.polylines(debug_image, [points_i], True, color, 2, cv2.LINE_AA)

        label = detection.data
        if not valid_width:
            label = f"{detection.data} width={detection.width_px:.0f}px"
        text_origin = (
            int(max(0, np.min(detection.points[:, 0]))),
            int(max(20, np.min(detection.points[:, 1]) - 10)),
        )
        cv2.putText(
            debug_image,
            label,
            text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    return debug_image


def _detect_on_image(
    detector: cv2.QRCodeDetector,
    image: np.ndarray,
    attempt_name: str,
    point_mapper: PointMapper,
) -> List[QRDetection]:
    detections = []

    try:
        retval, decoded_info, points, _ = detector.detectAndDecodeMulti(image)
    except cv2.error:
        retval, decoded_info, points = False, [], None

    if retval and points is not None:
        for data, qr_points in zip(decoded_info, points):
            if data:
                detections.append(_make_detection(data, qr_points, attempt_name, point_mapper))

    if detections:
        return detections

    try:
        data, points, _ = detector.detectAndDecode(image)
    except cv2.error:
        return []

    if data and points is not None:
        detections.append(_make_detection(data, points, attempt_name, point_mapper))

    return detections


def _make_detection(
    data: str,
    points: np.ndarray,
    attempt_name: str,
    point_mapper: PointMapper,
) -> QRDetection:
    normalized_points = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    normalized_points = point_mapper(normalized_points)

    center = np.mean(normalized_points, axis=0)
    edge_lengths = [
        float(
            np.linalg.norm(
                normalized_points[(index + 1) % len(normalized_points)]
                - normalized_points[index]
            )
        )
        for index in range(len(normalized_points))
    ]
    width_px = max(edge_lengths) if edge_lengths else 0.0

    return QRDetection(
        data=data,
        points=normalized_points,
        attempt=attempt_name,
        width_px=width_px,
        cx=float(center[0]),
        cy=float(center[1]),
    )


def _map_rotated_points(
    points: np.ndarray,
    original_width: int,
    original_height: int,
    rotation_code: int,
) -> np.ndarray:
    mapped = np.zeros_like(points, dtype=np.float32)
    x = points[:, 0]
    y = points[:, 1]

    if rotation_code == cv2.ROTATE_90_CLOCKWISE:
        mapped[:, 0] = y
        mapped[:, 1] = float(original_height - 1) - x
    elif rotation_code == cv2.ROTATE_180:
        mapped[:, 0] = float(original_width - 1) - x
        mapped[:, 1] = float(original_height - 1) - y
    elif rotation_code == cv2.ROTATE_90_COUNTERCLOCKWISE:
        mapped[:, 0] = float(original_width - 1) - y
        mapped[:, 1] = x
    else:
        mapped = points.astype(np.float32)

    return mapped


def _build_floor_marker_warps(
    image_bgr: np.ndarray,
) -> Iterable[Tuple[int, np.ndarray, PointMapper]]:
    """Find bright floor QR plates and decode them after perspective correction."""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    _, bright = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    height, width = gray.shape[:2]
    image_area = float(width * height)
    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 250.0 or area > image_area * 0.08:
            continue

        x, y, box_w, box_h = cv2.boundingRect(contour)
        if box_w < 25 or box_h < 8:
            continue

        aspect = box_w / float(max(1, box_h))
        if aspect < 1.2 or aspect > 8.0:
            continue

        roi = gray[y : y + box_h, x : x + box_w]
        if roi.size == 0:
            continue

        dark_ratio = float(np.mean(roi < 115))
        if dark_ratio < 0.04 or dark_ratio > 0.70:
            continue

        hull = cv2.convexHull(contour)
        perimeter = cv2.arcLength(hull, True)
        if perimeter <= 0.0:
            continue

        approx = cv2.approxPolyDP(hull, 0.03 * perimeter, True)
        if len(approx) != 4:
            continue

        candidates.append((area, approx.reshape(4, 2).astype(np.float32)))

    candidates.sort(key=lambda item: item[0], reverse=True)

    for index, (_, quad_points) in enumerate(candidates[:5]):
        ordered = _order_quad_points(quad_points)
        output_size = 580
        destination = np.array(
            [
                [0.0, 0.0],
                [output_size - 1.0, 0.0],
                [output_size - 1.0, output_size - 1.0],
                [0.0, output_size - 1.0],
            ],
            dtype=np.float32,
        )
        homography = cv2.getPerspectiveTransform(ordered, destination)
        inverse_homography = cv2.getPerspectiveTransform(destination, ordered)
        warped = cv2.warpPerspective(gray, homography, (output_size, output_size))
        _, thresholded = cv2.threshold(
            warped,
            0,
            255,
            cv2.THRESH_BINARY | cv2.THRESH_OTSU,
        )

        def map_points(points, matrix=inverse_homography):
            mapped = cv2.perspectiveTransform(
                np.asarray(points, dtype=np.float32).reshape(-1, 1, 2),
                matrix,
            )
            return mapped.reshape(-1, 2)

        yield index, thresholded, map_points


def _order_quad_points(points: np.ndarray) -> np.ndarray:
    ordered = np.zeros((4, 2), dtype=np.float32)
    by_y = points[np.argsort(points[:, 1])]
    top = by_y[:2]
    bottom = by_y[2:]

    top = top[np.argsort(top[:, 0])]
    bottom = bottom[np.argsort(bottom[:, 0])]

    ordered[0] = top[0]
    ordered[1] = top[1]
    ordered[2] = bottom[1]
    ordered[3] = bottom[0]
    return ordered
