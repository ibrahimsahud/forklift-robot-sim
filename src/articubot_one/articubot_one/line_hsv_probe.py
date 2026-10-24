#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def compute_candidate_ranges(image: np.ndarray) -> dict[str, list[int]]:
    height, width = image.shape[:2]
    roi = image[int(height * 0.35) :, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    candidates = hsv[(saturation > 50) & (value > 40)]
    mode = "strict_color"
    if candidates.size == 0:
        candidates = hsv[(saturation > 15) & (value > 40)]
        mode = "low_saturation_fallback"
    if candidates.size == 0:
        raise ValueError("No usable HSV candidates found in ROI")

    hues = candidates[:, 0]
    hist = np.bincount(hues, minlength=180)
    peak_hue = int(np.argmax(hist))
    hue_distance = np.minimum((hues - peak_hue) % 180, (peak_hue - hues) % 180)
    cluster = candidates[hue_distance <= 12]
    if cluster.size == 0:
        cluster = candidates

    h_vals = cluster[:, 0]
    s_vals = cluster[:, 1]
    v_vals = cluster[:, 2]
    lower = [
        max(0, int(np.percentile(h_vals, 5)) - 5),
        max(0, int(np.percentile(s_vals, 10)) - 15),
        max(0, int(np.percentile(v_vals, 10)) - 15),
    ]
    upper = [
        min(179, int(np.percentile(h_vals, 95)) + 5),
        min(255, int(np.percentile(s_vals, 98)) + 10),
        min(255, int(np.percentile(v_vals, 98)) + 10),
    ]
    return {
        "mode": [mode],
        "peak_hue": [peak_hue],
        "candidate_lower": lower,
        "candidate_upper": upper,
        "roi_shape": [int(roi.shape[1]), int(roi.shape[0])],
        "sample_count": [int(cluster.shape[0])],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe candidate HSV ranges from a saved line image.")
    parser.add_argument("--image", required=True, type=Path)
    args = parser.parse_args()

    image = cv2.imread(str(args.image))
    if image is None:
        raise SystemExit(f"Could not read image: {args.image}")

    result = compute_candidate_ranges(image)
    print(f"Image: {args.image}")
    print(f"Probe mode: {result['mode'][0]}")
    print(f"Peak hue: {result['peak_hue'][0]}")
    print(f"Candidate lower: {result['candidate_lower']}")
    print(f"Candidate upper: {result['candidate_upper']}")
    print(f"ROI shape: {result['roi_shape'][0]}x{result['roi_shape'][1]}")
    print(f"Sample count: {result['sample_count'][0]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
