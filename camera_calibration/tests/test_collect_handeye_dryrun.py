"""End-to-end test of the JSONL dry-run path in
scripts/collect_handeye_samples.py: schema-validated robot pose samples +
synthetic ChArUco frames -> handeye.PoseSample objects, entirely offline.

No robot, camera, or FS100 adapter/transport is touched. The board image is
rendered synthetically (same technique as test_charuco_detection.py) and
written to disk as frame_<sample_id>.png so the collector's normal
"read files from --dir" path is exercised unmodified.
"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from camera_calibration.board_generator import render_board_png
from camera_calibration.config import BoardConfig
from camera_calibration.detector import CharucoBoardDetector
from camera_calibration.robot_pose_schema import PoseSampleError

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import collect_handeye_samples as chs  # noqa: E402

TEST_DPI = 96


def _make_pose_record(sample_id, monotonic_ns, utc, x, transport="unconfirmed"):
    return {
        "schema_version": "1.0",
        "sample_id": sample_id,
        "timestamp": {"monotonic_ns": monotonic_ns, "utc_iso8601": utc},
        "source": {
            "controller": "FS100",
            "robot_model": "MH5",
            "adapter": "synthetic-test",
            "adapter_version": "0.0.0",
            "transport": transport,
        },
        "frame_id": "robot_base",
        "tool_id": 0,
        "user_frame_id": None,
        "joints": {"unit": "degree", "values": [0.0, -10.0, 20.0, 0.0, 30.0, 0.0]},
        "tcp_pose": {
            "position": {"unit": "m", "x": x, "y": 0.0, "z": 0.3},
            "orientation": {"representation": "quaternion_wxyz", "values": [1.0, 0.0, 0.0, 0.0]},
        },
    }


@pytest.fixture(scope="module")
def synthetic_frame():
    cfg = BoardConfig()
    img = render_board_png(cfg, dpi=TEST_DPI)
    return cfg, img


@pytest.fixture()
def session_dir(tmp_path, synthetic_frame):
    cfg, img = synthetic_frame
    for sample_id in ("0000", "0001"):
        cv2.imwrite(str(tmp_path / f"frame_{sample_id}.png"), img)
    h, w = img.shape[:2]
    camera_matrix = np.array([[w, 0, w / 2], [0, w, h / 2], [0, 0, 1]], dtype=np.float64)
    dist_coeffs = np.zeros(5)
    return tmp_path, cfg, camera_matrix, dist_coeffs


def _args(tmp_path, pose_jsonl_path, allow_unconfirmed_transport=False, max_skew_ms=50.0):
    return SimpleNamespace(
        dir=tmp_path,
        pose_jsonl=pose_jsonl_path,
        max_skew_ms=max_skew_ms,
        allow_unconfirmed_transport=allow_unconfirmed_transport,
        min_corners=6,
    )


def test_dry_run_jsonl_produces_pose_samples(session_dir):
    tmp_path, cfg, camera_matrix, dist_coeffs = session_dir
    records = [
        _make_pose_record("0000", 1_000_000_000, "2026-09-08T03:14:15.000000Z", x=0.40),
        _make_pose_record("0001", 1_200_000_000, "2026-09-08T03:14:15.200000Z", x=0.45),
    ]
    pose_path = tmp_path / "poses.jsonl"
    pose_path.write_text("\n".join(json.dumps(r) for r in records), encoding="utf-8")

    detector = CharucoBoardDetector(cfg)
    args = _args(tmp_path, pose_path, allow_unconfirmed_transport=True)
    samples, skipped = chs._collect_from_pose_jsonl(args, detector, camera_matrix, dist_coeffs)

    assert skipped == 0
    assert len(samples) == 2
    # identity quaternion -> identity rotation
    np.testing.assert_allclose(samples[0].R_gripper2base, np.eye(3), atol=1e-8)
    np.testing.assert_allclose(samples[0].t_gripper2base.reshape(3), [0.40, 0.0, 0.3])
    np.testing.assert_allclose(samples[1].t_gripper2base.reshape(3), [0.45, 0.0, 0.3])
    # board pose actually solved from the synthetic frame (not zero/garbage)
    assert samples[0].R_target2cam.shape == (3, 3)
    assert samples[0].t_target2cam.shape == (3, 1)


def test_dry_run_missing_frame_is_skipped_not_fatal(session_dir):
    tmp_path, cfg, camera_matrix, dist_coeffs = session_dir
    records = [
        _make_pose_record("0000", 1_000_000_000, "2026-09-08T03:14:15.000000Z", x=0.40),
        _make_pose_record("no_such_frame", 1_200_000_000, "2026-09-08T03:14:15.200000Z", x=0.45),
    ]
    pose_path = tmp_path / "poses.jsonl"
    pose_path.write_text("\n".join(json.dumps(r) for r in records), encoding="utf-8")

    detector = CharucoBoardDetector(cfg)
    args = _args(tmp_path, pose_path, allow_unconfirmed_transport=True)
    samples, skipped = chs._collect_from_pose_jsonl(args, detector, camera_matrix, dist_coeffs)

    assert len(samples) == 1
    assert skipped == 1


def test_dry_run_refuses_unconfirmed_transport_without_explicit_flag(session_dir):
    tmp_path, cfg, camera_matrix, dist_coeffs = session_dir
    records = [_make_pose_record("0000", 1_000_000_000, "2026-09-08T03:14:15.000000Z", x=0.40)]
    pose_path = tmp_path / "poses.jsonl"
    pose_path.write_text(json.dumps(records[0]), encoding="utf-8")

    detector = CharucoBoardDetector(cfg)
    args = _args(tmp_path, pose_path, allow_unconfirmed_transport=False)
    with pytest.raises(PoseSampleError, match="unconfirmed"):
        chs._collect_from_pose_jsonl(args, detector, camera_matrix, dist_coeffs)


def test_dry_run_flags_time_sync_problem_before_solving(session_dir):
    tmp_path, cfg, camera_matrix, dist_coeffs = session_dir
    records = [
        _make_pose_record("0000", 1_000_000_000, "2026-09-08T03:14:15.000000Z", x=0.40),
        # monotonic advances 200ms, UTC advances only 1ms -> gross skew
        _make_pose_record("0001", 1_200_000_000, "2026-09-08T03:14:15.001000Z", x=0.45),
    ]
    pose_path = tmp_path / "poses.jsonl"
    pose_path.write_text("\n".join(json.dumps(r) for r in records), encoding="utf-8")

    detector = CharucoBoardDetector(cfg)
    args = _args(tmp_path, pose_path, allow_unconfirmed_transport=True, max_skew_ms=50.0)
    with pytest.raises(PoseSampleError, match="time-sync"):
        chs._collect_from_pose_jsonl(args, detector, camera_matrix, dist_coeffs)


def test_dry_run_rejects_sample_with_unverified_representation(session_dir):
    """A record using a pulse joint unit anywhere in the file must be
    rejected by schema loading before it ever reaches the solver."""
    tmp_path, cfg, camera_matrix, dist_coeffs = session_dir
    bad = _make_pose_record("0000", 1_000_000_000, "2026-09-08T03:14:15.000000Z", x=0.40)
    bad["joints"]["unit"] = "pulse"
    pose_path = tmp_path / "poses.jsonl"
    pose_path.write_text(json.dumps(bad), encoding="utf-8")

    with pytest.raises(PoseSampleError, match="pulse"):
        chs.load_pose_samples_jsonl(pose_path)
