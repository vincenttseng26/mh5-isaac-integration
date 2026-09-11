#!/usr/bin/env python3
"""Live ChArUco detection + pose overlay from a generic webcam or a ZED
camera. Requires a display (cv2.imshow); run detect_offline.py instead on a
headless machine.

This script never runs automatically and is not touched by the test suite
-- it only executes when a human explicitly runs it with a camera attached.

Examples:
    python detect_live.py --camera-index 0
    python detect_live.py --zed --camera-matrix intrinsics.yaml
"""
import argparse
from pathlib import Path

import _bootstrap  # noqa: F401

import cv2
import numpy as np
import yaml

from camera_calibration.capture import GenericCameraSource, ZedCameraSource
from camera_calibration.config import DEFAULT_CONFIG_PATH, load_config
from camera_calibration.detector import CharucoBoardDetector


def load_intrinsics(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
    return camera_matrix, dist_coeffs


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    p.add_argument("--zed", action="store_true", help="use a ZED camera instead of a generic webcam")
    p.add_argument("--camera-index", type=int, default=0)
    p.add_argument("--camera-matrix", type=Path, help="YAML with camera_matrix/dist_coeffs; enables pose overlay")
    return p.parse_args()


def main():
    args = parse_args()
    cfg = load_config(args.config)
    detector = CharucoBoardDetector(cfg)

    camera_matrix = dist_coeffs = None
    if args.camera_matrix:
        camera_matrix, dist_coeffs = load_intrinsics(args.camera_matrix)
    elif args.zed:
        # ZED ships factory intrinsics; use them if the user didn't supply
        # a fresher calibration file.
        pass

    if args.zed:
        source = ZedCameraSource()
        if camera_matrix is None:
            camera_matrix, dist_coeffs = source.get_left_intrinsics()
    else:
        source = GenericCameraSource(args.camera_index)

    print("Press 'q' to quit.")
    try:
        while True:
            frame = source.read()
            if frame is None:
                print("No frame received; stopping.")
                break

            detection = detector.detect(frame)
            vis = detector.draw_overlay(frame, detection)

            if camera_matrix is not None:
                pose = detector.estimate_pose(detection, camera_matrix, dist_coeffs)
                if pose is not None:
                    cv2.drawFrameAxes(
                        vis, camera_matrix, dist_coeffs, pose.rvec, pose.tvec, cfg.square_length_m
                    )

            cv2.imshow("ChArUco detection", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        source.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
