from dataclasses import dataclass
from typing import Optional, Sequence

import cv2
import numpy as np


@dataclass
class LineDetection:
    offset: float
    angle: float
    contour_area: float
    detected: bool
    lookahead_offset: float = 0.0
    lookahead_x: float = 0.0
    lookahead_y: float = 0.0


def detect_line_with_mask(
    image_bgr: np.ndarray,
    dark_threshold: int = 90,
    min_area: float = 250.0,
    color_lower_hsv: Sequence[int] = (35, 45, 45),
    color_upper_hsv: Sequence[int] = (95, 255, 255),
) -> tuple[LineDetection, np.ndarray, np.ndarray]:
    detection, debug, mask = _detect_line_impl(
        image_bgr,
        dark_threshold=dark_threshold,
        min_area=min_area,
        color_lower_hsv=color_lower_hsv,
        color_upper_hsv=color_upper_hsv,
    )
    return detection, debug, mask


def detect_line(
    image_bgr: np.ndarray,
    dark_threshold: int = 90,
    min_area: float = 250.0,
    color_lower_hsv: Sequence[int] = (35, 45, 45),
    color_upper_hsv: Sequence[int] = (95, 255, 255),
) -> tuple[LineDetection, np.ndarray]:
    detection, debug, _ = _detect_line_impl(
        image_bgr,
        dark_threshold=dark_threshold,
        min_area=min_area,
        color_lower_hsv=color_lower_hsv,
        color_upper_hsv=color_upper_hsv,
    )
    return detection, debug


def _detect_line_impl(
    image_bgr: np.ndarray,
    dark_threshold: int = 90,
    min_area: float = 250.0,
    color_lower_hsv: Sequence[int] = (35, 45, 45),
    color_upper_hsv: Sequence[int] = (95, 255, 255),
) -> tuple[LineDetection, np.ndarray, np.ndarray]:
    debug = image_bgr.copy()
    height, width = image_bgr.shape[:2]
    if height == 0 or width == 0:
        return LineDetection(0.0, 0.0, 0.0, False), debug, np.zeros((0, 0), dtype=np.uint8)

    roi_y = int(height * 0.35)
    roi = image_bgr[roi_y:, :]
    mask, mask_source = _precision_line_mask(
        roi,
        dark_threshold,
        min_area,
        color_lower_hsv=color_lower_hsv,
        color_upper_hsv=color_upper_hsv,
    )

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = _largest_contour(contours, min_area)
    if contour is None:
        cv2.putText(
            debug,
            "NO LINE",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return LineDetection(0.0, 0.0, 0.0, False), debug, mask

    contour[:, 0, 1] += roi_y
    area = float(cv2.contourArea(contour))
    moments = cv2.moments(contour)
    if moments["m00"] == 0.0:
        center_x = width / 2.0
        center_y = height / 2.0
    else:
        center_x = moments["m10"] / moments["m00"]
        center_y = moments["m01"] / moments["m00"]

    offset = (center_x - (width / 2.0)) / (width / 2.0)

    angle, centerline = _centerline_angle(contour, height, width)
    lookahead_x = center_x
    lookahead_y = float(roi_y + (height - roi_y) * 0.72)
    if centerline is None:
        line = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x0, y0 = [float(value) for value in line.flatten()]
        angle = float(np.arctan2(vx, vy if abs(vy) > 1e-6 else 1e-6))

        draw_y1 = height - 1
        draw_y2 = roi_y
        if abs(vy) > 1e-6:
            draw_x1 = int(x0 + (draw_y1 - y0) * vx / vy)
            draw_x2 = int(x0 + (draw_y2 - y0) * vx / vy)
            lookahead_x = float(x0 + (lookahead_y - y0) * vx / vy)
        else:
            draw_x1 = draw_x2 = int(x0)
    else:
        draw_x1, draw_y1, draw_x2, draw_y2 = centerline
        if draw_y1 != draw_y2:
            t = (lookahead_y - draw_y2) / float(draw_y1 - draw_y2)
            t = min(1.0, max(0.0, t))
            lookahead_x = draw_x2 + t * (draw_x1 - draw_x2)
        else:
            lookahead_x = float(draw_x1)

    lookahead_x = float(min(max(lookahead_x, 0.0), width - 1.0))
    lookahead_offset = (lookahead_x - (width / 2.0)) / (width / 2.0)

    cv2.drawContours(debug, [contour], -1, (0, 255, 0), 2)
    cv2.line(debug, (draw_x1, draw_y1), (draw_x2, draw_y2), (255, 0, 0), 2)
    cv2.circle(debug, (int(center_x), int(center_y)), 5, (0, 255, 255), -1)
    cv2.circle(debug, (int(round(lookahead_x)), int(round(lookahead_y))), 6, (0, 128, 255), -1)
    cv2.putText(
        debug,
        f"offset={offset:.2f} lookahead={lookahead_offset:.2f} angle={angle:.2f}",
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        debug,
        mask_source,
        (20, 110),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    return LineDetection(
        offset,
        angle,
        area,
        True,
        lookahead_offset=lookahead_offset,
        lookahead_x=lookahead_x,
        lookahead_y=lookahead_y,
    ), debug, mask


def _precision_line_mask(
    roi_bgr: np.ndarray,
    dark_threshold: int,
    min_area: float,
    color_lower_hsv: Sequence[int],
    color_upper_hsv: Sequence[int],
) -> tuple[np.ndarray, str]:
    kernel = np.ones((5, 5), np.uint8)

    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    color_mask = cv2.inRange(
        hsv,
        tuple(int(v) for v in color_lower_hsv),
        tuple(int(v) for v in color_upper_hsv),
    )
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if _largest_contour(contours, min_area) is not None:
        return color_mask, "COLOR LINE"

    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, dark_mask = cv2.threshold(blurred, dark_threshold, 255, cv2.THRESH_BINARY_INV)
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel)
    return dark_mask, "DARK LINE"


def _largest_contour(contours, min_area: float) -> Optional[np.ndarray]:
    best = None
    best_area = min_area
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > best_area:
            best = contour
            best_area = area
    return best


def _centerline_angle(
    contour: np.ndarray,
    height: int,
    width: int,
) -> tuple[float, Optional[tuple[int, int, int, int]]]:
    contour_mask = np.zeros((height, width), dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
    ys, xs = np.where(contour_mask > 0)
    if len(xs) < 20:
        return 0.0, None

    y_span = float(np.max(ys) - np.min(ys))
    if y_span < 20.0:
        return 0.0, None

    top_limit = np.percentile(ys, 35)
    bottom_limit = np.percentile(ys, 65)
    top_mask = ys <= top_limit
    bottom_mask = ys >= bottom_limit
    if int(np.count_nonzero(top_mask)) < 5 or int(np.count_nonzero(bottom_mask)) < 5:
        return 0.0, None

    top_x = float(np.mean(xs[top_mask]))
    top_y = float(np.mean(ys[top_mask]))
    bottom_x = float(np.mean(xs[bottom_mask]))
    bottom_y = float(np.mean(ys[bottom_mask]))
    dy = bottom_y - top_y
    if abs(dy) < 1e-6:
        return 0.0, None

    dx = bottom_x - top_x
    angle = float(np.arctan2(dx, dy))
    centerline = (
        int(round(bottom_x)),
        int(round(bottom_y)),
        int(round(top_x)),
        int(round(top_y)),
    )
    return angle, centerline
