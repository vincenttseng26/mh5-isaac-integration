"""Hand-eye calibration: turn (robot pose, board pose) sample pairs into a
camera extrinsic transform, for both mounting configurations.

- eye-in-hand: camera rigidly mounted on the robot end-effector, board fixed
  in the world. Solves for T_gripper_cam (camera pose in the gripper frame).
- eye-to-hand: camera fixed in the world (e.g. on a tripod/frame), board
  rigidly mounted on the end-effector. Solves for T_base_cam (camera pose
  in the robot base frame).

cv2.calibrateHandEye always solves the AX=XB problem for
"gripper-to-base, target-to-cam -> cam-to-gripper". The eye-to-hand case is
handled by inverting the robot poses (gripper2base -> base2gripper) before
calling it, which is the standard trick documented in the OpenCV hand-eye
tutorial: with the inputs inverted, the same routine returns cam-to-base
instead of cam-to-gripper. See README.md for the full derivation and the
frame-convention diagram.
"""
from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Literal, Sequence

import cv2
import numpy as np

Mode = Literal["eye_in_hand", "eye_to_hand"]


@dataclasses.dataclass
class PoseSample:
    """One synchronized (robot pose, detected board pose) pair."""

    R_gripper2base: np.ndarray  # 3x3, from robot forward kinematics
    t_gripper2base: np.ndarray  # 3x1, meters
    R_target2cam: np.ndarray  # 3x3, from ChArUco pose estimation (Rodrigues of rvec)
    t_target2cam: np.ndarray  # 3x1, meters


@dataclasses.dataclass
class HandEyeResult:
    R: np.ndarray  # 3x3
    t: np.ndarray  # 3x1, meters
    label: str  # "cam2gripper" or "cam2base"

    def as_4x4(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = self.t.reshape(3)
        return T

    def as_quat_xyz(self) -> dict:
        """Rotation as (w, x, y, z) quaternion + xyz translation, for easy
        export into robot/ROS tooling that doesn't want a raw matrix."""
        R = self.R
        w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
        w4 = max(4 * w, 1e-8)
        x = (R[2, 1] - R[1, 2]) / w4
        y = (R[0, 2] - R[2, 0]) / w4
        z = (R[1, 0] - R[0, 1]) / w4
        t = self.t.reshape(3)
        return {"qw": float(w), "qx": float(x), "qy": float(y), "qz": float(z),
                "x": float(t[0]), "y": float(t[1]), "z": float(t[2])}


def _invert_rt(R: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    R_inv = R.T
    t_inv = -R_inv @ t.reshape(3, 1)
    return R_inv, t_inv


def solve_hand_eye(
    samples: Sequence[PoseSample],
    mode: Mode,
    method: int = cv2.CALIB_HAND_EYE_TSAI,
    min_samples: int = 10,
) -> HandEyeResult:
    if len(samples) < min_samples:
        raise ValueError(
            f"Only {len(samples)} pose samples given; recommend >= {min_samples} "
            "with varied end-effector orientations for a well-conditioned solve "
            "(see README acceptance thresholds)."
        )

    R_g2b = [s.R_gripper2base for s in samples]
    t_g2b = [s.t_gripper2base for s in samples]

    if mode == "eye_to_hand":
        inverted = [_invert_rt(R, t) for R, t in zip(R_g2b, t_g2b)]
        R_in = [r for r, _ in inverted]
        t_in = [t for _, t in inverted]
        label = "cam2base"
    elif mode == "eye_in_hand":
        R_in, t_in = R_g2b, t_g2b
        label = "cam2gripper"
    else:
        raise ValueError(f"mode must be 'eye_in_hand' or 'eye_to_hand', got {mode!r}")

    R_target2cam = [s.R_target2cam for s in samples]
    t_target2cam = [s.t_target2cam for s in samples]

    R_x, t_x = cv2.calibrateHandEye(R_in, t_in, R_target2cam, t_target2cam, method=method)
    return HandEyeResult(R=R_x, t=t_x, label=label)


def save_result_yaml(result: HandEyeResult, mode: Mode, path: Path | str) -> None:
    import yaml

    data = {
        "mode": mode,
        "label": result.label,
        "matrix_4x4": result.as_4x4().tolist(),
        "quaternion_xyz": result.as_quat_xyz(),
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
