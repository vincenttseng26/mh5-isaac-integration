"""ChArUco marker/corner detection and board pose estimation.

Pure OpenCV, no camera/ZED dependency — works on any grayscale/BGR image,
whether it came from a file, a webcam frame, or a ZED frame retrieved
elsewhere. Keeping this hardware-agnostic is what lets the offline test
path and the live capture path share one implementation.
"""
from __future__ import annotations

import dataclasses
from typing import Optional

import cv2
import numpy as np
from cv2 import aruco

from . import compat
from .config import BoardConfig


@dataclasses.dataclass
class DetectionResult:
    marker_corners: list
    marker_ids: Optional[np.ndarray]
    charuco_corners: Optional[np.ndarray]
    charuco_ids: Optional[np.ndarray]

    @property
    def num_markers(self) -> int:
        return 0 if self.marker_ids is None else len(self.marker_ids)

    @property
    def num_charuco_corners(self) -> int:
        return 0 if self.charuco_ids is None else len(self.charuco_ids)


@dataclasses.dataclass
class PoseResult:
    rvec: np.ndarray
    tvec: np.ndarray

    @property
    def rotation_matrix(self) -> np.ndarray:
        R, _ = cv2.Rodrigues(self.rvec)
        return R


class CharucoBoardDetector:
    """Stateless-ish wrapper: build once per BoardConfig, call detect() per frame."""

    def __init__(self, cfg: BoardConfig):
        cfg.validate()
        self.cfg = cfg
        self.dictionary = compat.get_dictionary(cfg.dictionary)
        self.board = compat.make_charuco_board(cfg, self.dictionary)

    @staticmethod
    def _to_gray(image: np.ndarray) -> np.ndarray:
        if image.ndim == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image

    def detect(self, image: np.ndarray) -> DetectionResult:
        gray = self._to_gray(image)
        marker_corners, marker_ids, _rejected = compat.detect_markers(gray, self.dictionary)
        charuco_corners, charuco_ids = compat.interpolate_charuco(
            marker_corners, marker_ids, gray, self.board
        )
        return DetectionResult(
            marker_corners=marker_corners,
            marker_ids=marker_ids,
            charuco_corners=charuco_corners,
            charuco_ids=charuco_ids,
        )

    def estimate_pose(
        self,
        detection: DetectionResult,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        min_corners: int = 6,
    ) -> Optional[PoseResult]:
        if detection.num_charuco_corners < min_corners:
            return None
        ok, rvec, tvec = compat.estimate_pose_charuco(
            detection.charuco_corners,
            detection.charuco_ids,
            self.board,
            camera_matrix,
            dist_coeffs,
        )
        if not ok:
            return None
        return PoseResult(rvec=rvec, tvec=tvec)

    def draw_overlay(self, image: np.ndarray, detection: DetectionResult) -> np.ndarray:
        vis = image.copy()
        if detection.marker_ids is not None and len(detection.marker_ids) > 0:
            aruco.drawDetectedMarkers(vis, detection.marker_corners, detection.marker_ids)
        if detection.charuco_ids is not None and len(detection.charuco_ids) > 0:
            aruco.drawDetectedCornersCharuco(vis, detection.charuco_corners, detection.charuco_ids)
        return vis


# --- module-level functions for isolation.run_isolated ---------------------
# run_isolated re-imports this module by name and calls a named function, so
# the actual "risky" detection call needs a plain function form too (a bound
# method on a fresh object works fine here since everything is re-created in
# the subprocess anyway).


def detect_in_subprocess(image: np.ndarray, cfg_dict: dict) -> dict:
    """Entry point used by isolation.run_isolated. Returns a plain dict
    (must be picklable) summarizing the detection so the parent process
    doesn't need cv2 types to interpret the result."""
    cfg = BoardConfig.from_dict(cfg_dict)
    detector = CharucoBoardDetector(cfg)
    result = detector.detect(image)
    return {
        "num_markers": result.num_markers,
        "num_charuco_corners": result.num_charuco_corners,
        "marker_ids": None if result.marker_ids is None else result.marker_ids.tolist(),
        "charuco_ids": None if result.charuco_ids is None else result.charuco_ids.tolist(),
    }


def detect_pose_and_annotate_in_subprocess(
    image: np.ndarray,
    cfg_dict: dict,
    camera_matrix: list,
    dist_coeffs: list,
    min_markers: int = 1,
    min_corners: int = 6,
) -> dict:
    """Detection + gate check + board pose + reprojection error + overlay,
    all in one isolated subprocess call. Pose estimation goes through the
    same native cv2.aruco/solvePnP code path as detection, so it needs the
    same crash isolation as detect_and_annotate_in_subprocess.

    camera_matrix/dist_coeffs are plain nested lists (not np.ndarray) so the
    call args stay picklable across the spawn boundary. Gate thresholds
    (min_markers/min_corners) are applied here, before pose estimation, so a
    frame that fails the gate never pays for a pose/reprojection attempt.
    """
    cfg = BoardConfig.from_dict(cfg_dict)
    detector = CharucoBoardDetector(cfg)
    result = detector.detect(image)
    overlay = detector.draw_overlay(image, result)

    out = {
        "num_markers": result.num_markers,
        "num_charuco_corners": result.num_charuco_corners,
        "marker_ids": None if result.marker_ids is None else result.marker_ids.reshape(-1).tolist(),
        "charuco_ids": None if result.charuco_ids is None else result.charuco_ids.reshape(-1).tolist(),
        "overlay": overlay,
        "gate_pass": result.num_markers >= min_markers and result.num_charuco_corners >= min_corners,
        "pose_ok": False,
    }
    if not out["gate_pass"]:
        return out

    camera_matrix_np = np.asarray(camera_matrix, dtype=np.float64)
    dist_coeffs_np = np.asarray(dist_coeffs, dtype=np.float64)
    pose = detector.estimate_pose(result, camera_matrix_np, dist_coeffs_np, min_corners=min_corners)
    if pose is None:
        return out

    obj_points_all = compat.get_board_object_points(detector.board)
    ids = result.charuco_ids.reshape(-1)
    obj_pts = obj_points_all[ids].reshape(-1, 1, 3).astype(np.float64)
    proj, _ = cv2.projectPoints(obj_pts, pose.rvec, pose.tvec, camera_matrix_np, dist_coeffs_np)
    proj = proj.reshape(-1, 2)
    detected = result.charuco_corners.reshape(-1, 2)
    errs = np.linalg.norm(proj - detected, axis=1)

    out.update({
        "pose_ok": True,
        "rvec": pose.rvec.reshape(-1).tolist(),
        "tvec": pose.tvec.reshape(-1).tolist(),
        "reprojection_error_px": {
            "rms": float(np.sqrt(np.mean(errs ** 2))),
            "mean": float(np.mean(errs)),
            "max": float(np.max(errs)),
        },
    })
    return out


def detect_and_annotate_in_subprocess(image: np.ndarray, cfg_dict: dict) -> dict:
    """Same as detect_in_subprocess, plus the annotated overlay image.

    Separate from detect_in_subprocess (rather than adding an
    include_overlay flag) so the plain report path never has to pay the
    extra pipe-transfer cost of a full-size image for every call."""
    cfg = BoardConfig.from_dict(cfg_dict)
    detector = CharucoBoardDetector(cfg)
    result = detector.detect(image)
    overlay = detector.draw_overlay(image, result)
    return {
        "num_markers": result.num_markers,
        "num_charuco_corners": result.num_charuco_corners,
        "marker_ids": None if result.marker_ids is None else result.marker_ids.reshape(-1).tolist(),
        "charuco_ids": None if result.charuco_ids is None else result.charuco_ids.reshape(-1).tolist(),
        "overlay": overlay,
    }
