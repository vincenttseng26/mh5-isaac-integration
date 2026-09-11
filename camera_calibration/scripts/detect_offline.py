#!/usr/bin/env python3
"""Offline ChArUco detection on image file(s). No camera or ZED SDK needed.

By default each image is detected inside an isolated subprocess (see
camera_calibration/isolation.py) so that if this OpenCV build segfaults on
detectMarkers/interpolateCornersCharuco, this script reports it per-image
and moves on instead of dying silently. Use --no-isolate to run in-process
(faster, but a crash takes the whole script down with it).

Examples:
    python detect_offline.py --image path/to/frame.png
    python detect_offline.py --dir path/to/frames --pattern "*.jpg"
    python detect_offline.py --image frame.png --camera-matrix intrinsics.yaml
"""
import argparse
import sys
from pathlib import Path

import _bootstrap  # noqa: F401

import cv2

from camera_calibration.config import DEFAULT_CONFIG_PATH, load_config
from camera_calibration.detector import CharucoBoardDetector
from camera_calibration.isolation import run_isolated


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--image", type=Path)
    src.add_argument("--dir", type=Path)
    p.add_argument("--pattern", default="*.png")
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    p.add_argument("--no-isolate", action="store_true")
    p.add_argument("--timeout", type=float, default=30.0)
    return p.parse_args()


def iter_images(args):
    if args.image is not None:
        yield args.image
    else:
        yield from sorted(args.dir.glob(args.pattern))


def detect_isolated(image_path: Path, cfg_dict: dict, timeout: float):
    img = cv2.imread(str(image_path))
    if img is None:
        return None, f"cv2.imread returned None for {image_path}"
    result = run_isolated(
        "camera_calibration.detector", "detect_in_subprocess", args=(img, cfg_dict), timeout=timeout
    )
    return result, None


def main():
    args = parse_args()
    cfg = load_config(args.config)
    exit_code = 0

    if args.no_isolate:
        detector = CharucoBoardDetector(cfg)

    for image_path in iter_images(args):
        if args.no_isolate:
            img = cv2.imread(str(image_path))
            if img is None:
                print(f"{image_path}: FAILED (cv2.imread returned None)")
                exit_code = 1
                continue
            det = detector.detect(img)
            print(f"{image_path}: markers={det.num_markers} charuco_corners={det.num_charuco_corners}")
            continue

        result, err = detect_isolated(image_path, cfg.to_dict(), args.timeout)
        if err:
            print(f"{image_path}: FAILED ({err})")
            exit_code = 1
        elif result.ok:
            v = result.value
            print(f"{image_path}: markers={v['num_markers']} charuco_corners={v['num_charuco_corners']}")
        else:
            print(f"{image_path}: {result.status.upper()} - {result.error}")
            exit_code = 1

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
