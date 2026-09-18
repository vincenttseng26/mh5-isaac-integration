"""Tests for scripts/validate_pose_samples.py, the offline JSON/JSONL
validator. Pure file I/O against synthetic records — no hardware."""
import json
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import validate_pose_samples as vps  # noqa: E402


def _valid_record(sample_id, monotonic_ns, utc):
    return {
        "schema_version": "1.0",
        "sample_id": sample_id,
        "timestamp": {"monotonic_ns": monotonic_ns, "utc_iso8601": utc},
        "source": {
            "controller": "FS100",
            "robot_model": "MH5",
            "adapter": "synthetic-test",
            "adapter_version": "0.0.0",
            "transport": "unconfirmed",
        },
        "frame_id": "robot_base",
        "tool_id": 0,
        "user_frame_id": None,
        "joints": {"unit": "degree", "values": [0.0, -10.0, 20.0, 0.0, 30.0, 0.0]},
        "tcp_pose": {
            "position": {"unit": "m", "x": 0.4, "y": 0.0, "z": 0.3},
            "orientation": {"representation": "quaternion_wxyz", "values": [1.0, 0.0, 0.0, 0.0]},
        },
    }


def test_validate_file_clean_jsonl_has_zero_problems(tmp_path):
    records = [
        _valid_record("0000", 1_000_000_000, "2026-09-08T03:14:15.000000Z"),
        _valid_record("0001", 1_200_000_000, "2026-09-08T03:14:15.200000Z"),
    ]
    path = tmp_path / "samples.jsonl"
    path.write_text("\n".join(json.dumps(r) for r in records), encoding="utf-8")

    problems = vps.validate_file(path, frames_dir=None, max_skew_ms=50.0)
    assert problems == 0


def test_validate_file_json_array_also_supported(tmp_path):
    records = [_valid_record("0000", 1_000_000_000, "2026-09-08T03:14:15.000000Z")]
    path = tmp_path / "samples.json"
    path.write_text(json.dumps(records), encoding="utf-8")

    problems = vps.validate_file(path, frames_dir=None, max_skew_ms=50.0)
    assert problems == 0


def test_validate_file_counts_schema_rejection(tmp_path):
    bad = _valid_record("0000", 1_000_000_000, "2026-09-08T03:14:15.000000Z")
    bad["tcp_pose"]["orientation"]["representation"] = "euler_xyz"
    path = tmp_path / "samples.jsonl"
    path.write_text(json.dumps(bad), encoding="utf-8")

    problems = vps.validate_file(path, frames_dir=None, max_skew_ms=50.0)
    assert problems == 1


def test_validate_file_counts_time_sync_problem(tmp_path):
    records = [
        _valid_record("0000", 1_000_000_000, "2026-09-08T03:14:15.000000Z"),
        _valid_record("0001", 1_200_000_000, "2026-09-08T03:14:15.001000Z"),  # skewed
    ]
    path = tmp_path / "samples.jsonl"
    path.write_text("\n".join(json.dumps(r) for r in records), encoding="utf-8")

    problems = vps.validate_file(path, frames_dir=None, max_skew_ms=50.0)
    assert problems == 1


def test_validate_file_checks_frames_dir(tmp_path):
    record = _valid_record("0000", 1_000_000_000, "2026-09-08T03:14:15.000000Z")
    path = tmp_path / "samples.jsonl"
    path.write_text(json.dumps(record), encoding="utf-8")

    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    # no frame_0000.png written -> should be flagged
    problems = vps.validate_file(path, frames_dir=frames_dir, max_skew_ms=50.0)
    assert problems == 1

    (frames_dir / "frame_0000.png").write_bytes(b"\x89PNG\r\n")
    problems = vps.validate_file(path, frames_dir=frames_dir, max_skew_ms=50.0)
    assert problems == 0
