"""Pure-math tests for pose_stats.py. No camera/robot/cv2.aruco involved --
these exercise the numpy/quaternion arithmetic directly."""
import math

import cv2
import numpy as np
import pytest

from camera_calibration import pose_stats


def _rot_z(deg: float) -> np.ndarray:
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_rvec_tvec_to_4x4_direction_matches_camera_from_board():
    # 90deg about Z, board origin sitting at (1, 0, 0) in camera coordinates.
    R = _rot_z(90.0)
    rvec, _ = cv2.Rodrigues(R)
    tvec = np.array([1.0, 0.0, 0.0])
    T_camera_board = pose_stats.rvec_tvec_to_4x4(rvec, tvec)

    # A point at the board origin must land exactly at tvec in camera coords.
    board_origin_h = np.array([0.0, 0.0, 0.0, 1.0])
    assert (T_camera_board @ board_origin_h)[:3] == pytest.approx(tvec)

    # A point 1m along the board's own +X axis must land at camera (1, 1, 0):
    # rotating (1,0,0) by 90deg about Z gives (0,1,0), then + translation (1,0,0).
    board_x_h = np.array([1.0, 0.0, 0.0, 1.0])
    assert (T_camera_board @ board_x_h)[:3] == pytest.approx([1.0, 1.0, 0.0], abs=1e-9)


def test_invert_4x4_round_trips_and_reverses_direction():
    R = _rot_z(30.0)
    rvec, _ = cv2.Rodrigues(R)
    tvec = np.array([0.2, -0.1, 0.5])
    T_camera_board = pose_stats.rvec_tvec_to_4x4(rvec, tvec)
    T_board_camera = pose_stats.invert_4x4(T_camera_board)

    assert T_camera_board @ T_board_camera == pytest.approx(np.eye(4), abs=1e-9)
    assert T_board_camera @ T_camera_board == pytest.approx(np.eye(4), abs=1e-9)

    # The camera's own origin, expressed in the board frame, must be
    # T_board_camera's translation column.
    camera_origin_h = np.array([0.0, 0.0, 0.0, 1.0])
    assert (T_board_camera @ camera_origin_h)[:3] == pytest.approx(T_board_camera[:3, 3])


def test_rotation_matrix_to_quat_wxyz_identity():
    q = pose_stats.rotation_matrix_to_quat_wxyz(np.eye(3))
    assert q == pytest.approx([1.0, 0.0, 0.0, 0.0], abs=1e-9)


def test_rotation_matrix_to_quat_wxyz_is_unit_norm_for_arbitrary_rotation():
    q = pose_stats.rotation_matrix_to_quat_wxyz(_rot_z(37.0))
    assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-9)


def test_quat_angle_deg_matches_known_rotation():
    q1 = pose_stats.rotation_matrix_to_quat_wxyz(np.eye(3))
    q2 = pose_stats.rotation_matrix_to_quat_wxyz(_rot_z(10.0))
    assert pose_stats.quat_angle_deg(q1, q2) == pytest.approx(10.0, abs=1e-6)


def test_quat_angle_deg_ignores_double_cover_sign_flip():
    q = pose_stats.rotation_matrix_to_quat_wxyz(_rot_z(5.0))
    assert pose_stats.quat_angle_deg(q, -q) == pytest.approx(0.0, abs=1e-9)


def test_mean_quaternion_of_identical_quats_is_itself():
    q = pose_stats.rotation_matrix_to_quat_wxyz(_rot_z(15.0))
    mean_q = pose_stats.mean_quaternion([q, q, q])
    assert mean_q == pytest.approx(q, abs=1e-9)


def test_mean_quaternion_handles_sign_flip_input():
    q = pose_stats.rotation_matrix_to_quat_wxyz(_rot_z(15.0))
    mean_q = pose_stats.mean_quaternion([q, -q, q])
    # -q represents the identical rotation; the mean should still recover it
    # (up to its own sign), i.e. the angle between them must be ~0.
    assert pose_stats.quat_angle_deg(mean_q, q) == pytest.approx(0.0, abs=1e-6)


def test_jitter_stats_from_values():
    stats = pose_stats.JitterStats.from_values([1.0, 2.0, 3.0])
    assert stats.mean == pytest.approx(2.0)
    assert stats.max == pytest.approx(3.0)
    assert stats.n == 3


def test_translation_jitter_zero_for_identical_translations():
    t = np.array([0.1, 0.2, 0.5])
    result = pose_stats.translation_jitter([t, t, t])
    assert result["centroid_deviation_mm"]["mean"] == pytest.approx(0.0, abs=1e-9)
    assert result["centroid_deviation_mm"]["max"] == pytest.approx(0.0, abs=1e-9)
    assert result["mean_translation_m"] == pytest.approx(t.tolist())


def test_translation_jitter_detects_known_offset():
    base = np.array([0.0, 0.0, 1.0])
    offset = np.array([0.001, 0.0, 1.0])  # 1mm offset on x
    result = pose_stats.translation_jitter([base, offset])
    # mean is the midpoint, so each sample deviates by 0.5mm from the centroid
    assert result["centroid_deviation_mm"]["mean"] == pytest.approx(0.5, abs=1e-6)


def test_rotation_jitter_zero_for_identical_rotations():
    q = pose_stats.rotation_matrix_to_quat_wxyz(_rot_z(0.0))
    result = pose_stats.rotation_jitter([q, q, q])
    assert result["angular_deviation_deg"]["mean"] == pytest.approx(0.0, abs=1e-9)
    assert result["angular_deviation_deg"]["max"] == pytest.approx(0.0, abs=1e-9)


def test_rotation_jitter_detects_known_spread():
    q_minus = pose_stats.rotation_matrix_to_quat_wxyz(_rot_z(-1.0))
    q_plus = pose_stats.rotation_matrix_to_quat_wxyz(_rot_z(1.0))
    result = pose_stats.rotation_jitter([q_minus, q_plus])
    assert result["angular_deviation_deg"]["max"] == pytest.approx(1.0, abs=1e-3)


def test_reprojection_stats():
    result = pose_stats.reprojection_stats([0.2, 0.4, 0.6])
    assert result["mean"] == pytest.approx(0.4)
    assert result["max"] == pytest.approx(0.6)


def test_mean_quaternion_empty_raises():
    with pytest.raises(ValueError):
        pose_stats.mean_quaternion([])


def test_jitter_stats_from_values_empty_raises():
    with pytest.raises(ValueError):
        pose_stats.JitterStats.from_values([])
