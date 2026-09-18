#!/usr/bin/env python3
"""Offline validator for robot pose sample files (JSON array or JSONL).

Never talks to a robot or camera — reads a file, validates every sample
against camera_calibration.robot_pose_schema, runs the cross-sample time
sync check, and (optionally) checks each sample against a frames directory
for sample_id <-> frame_<sample_id>.png pairing and capture-time skew.

Usage:
    python validate_pose_samples.py --input samples.jsonl
    python validate_pose_samples.py --input samples.json --frames-dir calib_data/session1 \
        --max-skew-ms 50

Exit code is non-zero if any sample fails schema validation or if any
time-sync problem is found.
"""
import argparse
import json
import sys
from pathlib import Path

import _bootstrap  # noqa: F401

from camera_calibration.robot_pose_schema import (
    PoseSampleError,
    RobotPoseSample,
    check_time_sync,
)


def load_raw_records(path: Path) -> list:
    text = path.read_text(encoding="utf-8")
    stripped = text.strip()
    if not stripped:
        return []
    if path.suffix == ".jsonl":
        return [json.loads(line) for line in stripped.splitlines() if line.strip()]
    # .json: accept either a JSON array, or JSON-Lines saved with a .json extension.
    try:
        parsed = json.loads(stripped)
    except json.JSONDecodeError:
        return [json.loads(line) for line in stripped.splitlines() if line.strip()]
    if isinstance(parsed, list):
        return parsed
    return [parsed]


def validate_file(path: Path, frames_dir: Path | None, max_skew_ms: float) -> int:
    """Returns the number of problems found (0 = clean)."""
    records = load_raw_records(path)
    print(f"Loaded {len(records)} raw record(s) from {path}")

    samples = []
    problems = 0
    for i, record in enumerate(records):
        sample_id = record.get("sample_id", f"<index {i}>") if isinstance(record, dict) else f"<index {i}>"
        try:
            sample = RobotPoseSample.from_dict(record)
        except PoseSampleError as exc:
            print(f"REJECT {sample_id}: {exc}")
            problems += 1
            continue
        samples.append(sample)
        ready = "production-ready" if sample.is_production_ready else "NOT production-ready (transport unconfirmed)"
        print(f"OK     {sample.sample_id}: frame_id={sample.frame_id} tool_id={sample.tool_id} "
              f"user_frame_id={sample.user_frame_id} joints_unit={sample.joints.unit} "
              f"orientation={sample.tcp_pose.orientation.representation} [{ready}]")

    if len(samples) >= 2:
        sync_problems = check_time_sync(samples, max_skew_ms=max_skew_ms)
        for p in sync_problems:
            print(f"TIME-SYNC PROBLEM: {p}")
        problems += len(sync_problems)

    if frames_dir is not None:
        for sample in samples:
            candidates = [
                frames_dir / f"frame_{sample.sample_id}.png",
                frames_dir / f"{sample.sample_id}.png",
            ]
            if not any(c.exists() for c in candidates):
                print(f"FRAME MISSING: no image found for sample_id={sample.sample_id!r} "
                      f"(looked for {', '.join(str(c) for c in candidates)})")
                problems += 1

    return problems


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True, help="samples.json or samples.jsonl")
    p.add_argument("--frames-dir", type=Path, default=None,
                   help="optional: check each sample_id has a matching frame_<id>.png")
    p.add_argument("--max-skew-ms", type=float, default=50.0,
                   help="max allowed skew (ms) between monotonic and UTC deltas across "
                        "consecutive samples (default: 50ms)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    problems = validate_file(args.input, args.frames_dir, args.max_skew_ms)
    print(f"\n{problems} problem(s) found.")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
