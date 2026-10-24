#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

import cv2


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from articubot_one.qr_detection import detect_qr_codes, draw_debug_image  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description="Offline QR detector test for saved camera frames.")
    parser.add_argument("image_path", help="Path to an image frame, for example /tmp/qr_test.png")
    parser.add_argument(
        "--debug-output",
        default="qr_test_debug.png",
        help="Path for the annotated debug image.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    image = cv2.imread(args.image_path, cv2.IMREAD_COLOR)
    if image is None:
        print(f"Failed to load image: {args.image_path}")
        return 1

    height, width = image.shape[:2]
    print(f"image size: {width}x{height}")

    detections = detect_qr_codes(image, enable_preprocessing=True)
    debug_image = draw_debug_image(
        image,
        detections,
        min_width_px=0.0,
        max_width_px=float("inf"),
    )
    cv2.imwrite(args.debug_output, debug_image)
    print(f"debug image: {args.debug_output}")

    if not detections:
        print("decoded QR data: NONE")
        print("detected points: NONE")
        print("QR width in pixels: NONE")
        print(
            "No QR detected. Suggested checks: QR may be cropped, too close, too small, "
            "too low resolution, covered by a route line, blurred, or viewed from a poor camera angle."
        )
        return 1

    for index, detection in enumerate(detections, start=1):
        print(f"detection #{index}")
        print(f"decoded QR data: {detection.data}")
        print(f"detection attempt: {detection.attempt}")
        print(f"detected points: {detection.points.tolist()}")
        print(f"QR width in pixels: {detection.width_px:.1f}")
        print(f"center pixel: {detection.cx:.1f}, {detection.cy:.1f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
