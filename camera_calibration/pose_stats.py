"""Pure-math helpers for aggregating a session of board-pose estimates into
jitter/stability statistics: quaternion conversion + averaging, translation
centroid deviation, rotation angular deviation, reprojection error rollup.

No camera/robot dependency, nothing here talks to hardware -- this is what
makes it directly unit-testable without pyzed/a real board. The capture
script (scripts/capture_a2_extrinsic_stability.py) is the only caller that
feeds it real data.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Sequence

import cv2
import numpy as np


def rvec_tvec_to_4x4(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """cv2 pose estimation (e.g. aruco.estimatePoseCharucoBoard /
    cv2.solvePnP against board object points) returns an (rvec, tvec) that
    maps a point in the BOARD/target frame into the CAMERA frame:
    X_camera = R @ X_board + t. This packs that into the corresponding
    4x4 homogeneous matrix T_camera_board (X_camera_h = T @ X_board_h)."""
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def invert_4x4(T: np.ndarray) -> np.ndarray:
    """Rigid-transform inverse (not a general 4x4 inverse -- assumes the top
    3x3 block is a proper rotation matrix, true for every T this module
    produces)."""
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv = np.eye(4, dtype=np.float64)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


def rotation_matrix_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """(w, x, y, z), unit-norm. Same formula as
    handeye.HandEyeResult.as_quat_xyz, kept independent here so this module
    has no import-time dependency on cv2 being used for hand-eye specifically."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    w4 = max(4 * w, 1e-8)
    x = (R[2, 1] - R[1, 2]) / w4
    y = (R[0, 2] - R[2, 0]) / w4
    z = (R[1, 0] - R[0, 1]) / w4
    q = np.array([w, x, y, z], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("degenerate rotation matrix produced a near-zero quaternion")
    return q / norm


def mean_quaternion(quats: Sequence[np.ndarray]) -> np.ndarray:
    """Sign-aligns each quaternion to the first (quaternions double-cover
    rotations: q and -q represent the same rotation) before averaging and
    renormalizing. Only valid for quaternions expected to already be close
    together (small jitter around one fixed pose) -- not a general-purpose
    quaternion mean."""
    if not quats:
        raise ValueError("mean_quaternion requires at least one quaternion")
    ref = quats[0]
    aligned = [q if np.dot(q, ref) >= 0 else -q for q in quats]
    m = np.mean(aligned, axis=0)
    norm = np.linalg.norm(m)
    if norm < 1e-12:
        raise ValueError("mean of quaternions is degenerate (near-zero norm)")
    return m / norm


def quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    """Angle (degrees) between the rotations represented by two unit
    quaternions, robust to the q/-q double-cover ambiguity."""
    dot = abs(float(np.dot(q1, q2)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2 * math.acos(dot))


@dataclasses.dataclass
class JitterStats:
    mean: float
    std: float
    max: float
    n: int

    @classmethod
    def from_values(cls, values: Sequence[float]) -> "JitterStats":
        if not values:
            raise ValueError("from_values requires at least one value")
        arr = np.asarray(values, dtype=np.float64)
        return cls(mean=float(arr.mean()), std=float(arr.std()), max=float(arr.max()), n=len(arr))

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


def translation_jitter(translations_m: Sequence[np.ndarray]) -> dict:
    """translations_m: sequence of 3-vectors (meters), one per valid frame."""
    arr = np.asarray(translations_m, dtype=np.float64)  # Nx3
    mean_t = arr.mean(axis=0)
    dev_mm = np.linalg.norm(arr - mean_t, axis=1) * 1000.0
    per_axis_std_mm = arr.std(axis=0) * 1000.0
    return {
        "mean_translation_m": mean_t.tolist(),
        "centroid_deviation_mm": JitterStats.from_values(dev_mm.tolist()).to_dict(),
        "per_axis_std_mm": {
            "x": float(per_axis_std_mm[0]),
            "y": float(per_axis_std_mm[1]),
            "z": float(per_axis_std_mm[2]),
        },
    }


def rotation_jitter(quats_wxyz: Sequence[np.ndarray]) -> dict:
    mean_q = mean_quaternion(list(quats_wxyz))
    angles_deg = [quat_angle_deg(q, mean_q) for q in quats_wxyz]
    return {
        "mean_quaternion_wxyz": mean_q.tolist(),
        "angular_deviation_deg": JitterStats.from_values(angles_deg).to_dict(),
    }


def reprojection_stats(rms_values_px: Sequence[float]) -> dict:
    return JitterStats.from_values(list(rms_values_px)).to_dict()
