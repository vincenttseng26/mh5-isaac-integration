#!/usr/bin/env python3
"""Turn a folder of (image, robot pose) pairs into a hand-eye calibration
result. This script only reads files from disk — it never talks to a robot
or camera directly. Populate the input folder by pairing frames captured
with detect_offline.py/detect_live.py against the matching robot pose you
recorded at the same instant (see README.md 'Data collection procedure').

Two robot pose input formats are supported:

1. Default (legacy, per-frame YAML): one pose_XXXX.yaml per frame_XXXX.png
   in --dir. Each pose_*.yaml holds the end-effector (gripper) pose *in the
   robot base frame* at capture time, as either:
       translation: [x, y, z]       # meters
       quaternion: [w, x, y, z]
   or:
       matrix: [[...], [...], [...], [...]]   # 4x4 homogeneous, meters

2. --pose-jsonl (dry-run, FS100-adapter-ready): a single JSON/JSONL file of
   camera_calibration.robot_pose_schema.RobotPoseSample records, matched to
   frames by sample_id (frame_<sample_id>.png). This is a pure offline file
   read — no adapter, socket, or device is touched. Use this to validate the
   collection pipeline against synthetic or logged FS100-adapter output
   before any real transport exists (see README 'MH5 + FS100 資料收集').
   Every sample's tcp_pose must already be the end-effector (gripper) pose
   in the robot base frame — this script does not know how to convert
   between robot/user/tool frames, so pick frame_id accordingly upstream.
   Samples whose source.transport is "unconfirmed" are accepted for
   dry-run/schema testing but are NOT a substitute for a verified FS100
   transport; solve_hand_eye output from such data must not be trusted as
   a real extrinsic.

Example:
    python collect_handeye_samples.py --dir calib_data/session1 \
        --camera-matrix intrinsics.yaml --mode eye_in_hand \
        --out calib_data/handeye_result.yaml

    python collect_handeye_samples.py --dir calib_data/dryrun_session \
        --camera-matrix intrinsics.yaml --mode eye_in_hand \
        --pose-jsonl calib_data/dryrun_session/poses.jsonl \
        --out calib_data/handeye_result_dryrun.yaml
"""
import argparse
import json
from pathlib import Path

import _bootstrap  # noqa: F401

import cv2
import numpy as np
import yaml

from camera_calibration.config import DEFAULT_CONFIG_PATH, load_config
from camera_calibration.detector import CharucoBoardDetector
from camera_calibration.handeye import PoseSample, save_result_yaml, solve_hand_eye
from camera_calibration.robot_pose_schema import PoseSampleError, RobotPoseSample, check_time_sync


def quat_to_R(q) -> np.ndarray:
    w, x, y, z = q
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def load_robot_pose(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if "matrix" in data:
        M = np.array(data["matrix"], dtype=np.float64)
        return M[:3, :3], M[:3, 3:4]
    R = quat_to_R(data["quaternion"])
    t = np.array(data["translation"], dtype=np.float64).reshape(3, 1)
    return R, t


def load_pose_samples_jsonl(path: Path) -> list:
    """Load RobotPoseSample records from a JSON array or JSONL file. Pure
    file read, no device access. Raises PoseSampleError on the first
    invalid record so a bad dry-run file fails loudly instead of silently
    dropping samples (use scripts/validate_pose_samples.py beforehand to
    get a full report of every problem instead of stopping at the first)."""
    text = path.read_text(encoding="utf-8")
    stripped = text.strip()
    if not stripped:
        return []
    if path.suffix == ".jsonl":
        records = [json.loads(line) for line in stripped.splitlines() if line.strip()]
    else:
        parsed = json.loads(stripped)
        records = parsed if isinstance(parsed, list) else [parsed]
    return [RobotPoseSample.from_dict(r) for r in records]


def robot_pose_sample_to_gripper2base(sample: RobotPoseSample) -> tuple:
    """Convert a schema-validated RobotPoseSample's tcp_pose into
    (R_gripper2base, t_gripper2base) 3x3/3x1 numpy arrays in meters, for
    handeye.PoseSample. Only unambiguous orientation representations reach
    this point — robot_pose_schema.RobotPoseSample.from_dict already
    rejected anything else (e.g. unconverted FS100 Euler angles)."""
    orientation = sample.tcp_pose.orientation
    if orientation.representation == "quaternion_wxyz":
        R = quat_to_R(orientation.values)
    elif orientation.representation == "rotation_matrix_3x3":
        R = np.array(orientation.values, dtype=np.float64)
    else:  # pragma: no cover - schema validation already excludes this
        raise PoseSampleError(f"unhandled orientation representation: {orientation.representation!r}")

    position = sample.tcp_pose.position
    scale = {"m": 1.0, "mm": 1e-3}[position.unit]
    t = np.array([position.x, position.y, position.z], dtype=np.float64).reshape(3, 1) * scale
    return R, t


def load_intrinsics(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
    return camera_matrix, dist_coeffs


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dir", type=Path, required=True)
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    p.add_argument("--camera-matrix", type=Path, required=True)
    p.add_argument("--mode", choices=["eye_in_hand", "eye_to_hand"], required=True)
    p.add_argument("--min-corners", type=int, default=6)
    p.add_argument("--out", type=Path, default=Path("calib_data/handeye_result.yaml"))
    p.add_argument("--pose-jsonl", type=Path, default=None,
                   help="dry-run: read robot poses from a schema-validated JSON/JSONL "
                        "file (see robot_pose_schema.RobotPoseSample) instead of "
                        "per-frame pose_XXXX.yaml files. Pure file read, no device "
                        "access. Frames are matched by sample_id -> frame_<sample_id>.png.")
    p.add_argument("--max-skew-ms", type=float, default=50.0,
                   help="only used with --pose-jsonl: max allowed skew (ms) between "
                        "monotonic and UTC timestamp deltas across samples (default: 50ms)")
    p.add_argument("--allow-unconfirmed-transport", action="store_true",
                   help="only used with --pose-jsonl: without this flag, refuse to "
                        "solve if any sample's source.transport is 'unconfirmed' "
                        "(the only value accepted today, pending Agy's FS100 transport "
                        "confirmation — see README). Pass this explicitly for dry-run/"
                        "schema testing so the safeguard isn't bypassed by accident.")
    return p.parse_args()


def _collect_from_yaml_dir(args, detector, camera_matrix, dist_coeffs):
    frame_paths = sorted(args.dir.glob("frame_*.png"))
    samples = []
    skipped = 0
    for frame_path in frame_paths:
        pose_path = args.dir / frame_path.name.replace("frame_", "pose_").replace(".png", ".yaml")
        if not pose_path.exists():
            print(f"skip {frame_path.name}: no matching {pose_path.name}")
            skipped += 1
            continue

        img = cv2.imread(str(frame_path))
        detection = detector.detect(img)
        pose = detector.estimate_pose(detection, camera_matrix, dist_coeffs, min_corners=args.min_corners)
        if pose is None:
            print(f"skip {frame_path.name}: board pose not found "
                  f"({detection.num_charuco_corners} corners, need >= {args.min_corners})")
            skipped += 1
            continue

        R_gripper2base, t_gripper2base = load_robot_pose(pose_path)
        R_target2cam = pose.rotation_matrix
        t_target2cam = pose.tvec.reshape(3, 1)

        samples.append(PoseSample(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
        ))
    return samples, skipped


def _collect_from_pose_jsonl(args, detector, camera_matrix, dist_coeffs):
    pose_samples = load_pose_samples_jsonl(args.pose_jsonl)
    print(f"Loaded {len(pose_samples)} robot pose sample(s) from {args.pose_jsonl}")

    sync_problems = check_time_sync(pose_samples, max_skew_ms=args.max_skew_ms)
    for problem in sync_problems:
        print(f"TIME-SYNC PROBLEM: {problem}")
    if sync_problems:
        raise PoseSampleError(
            f"{len(sync_problems)} time-sync problem(s) in {args.pose_jsonl}; "
            "fix the source data or re-run scripts/validate_pose_samples.py for "
            "full details before solving."
        )

    unconfirmed = [s.sample_id for s in pose_samples if not s.is_production_ready]
    if unconfirmed and not args.allow_unconfirmed_transport:
        raise PoseSampleError(
            f"{len(unconfirmed)} sample(s) have source.transport='unconfirmed' "
            f"(e.g. {unconfirmed[0]!r}): FS100 transport has not been confirmed "
            "yet, so this data cannot produce a trustworthy hand-eye result. "
            "Pass --allow-unconfirmed-transport to proceed anyway for dry-run/"
            "schema testing only."
        )

    by_sample_id = {s.sample_id: s for s in pose_samples}
    samples = []
    skipped = 0
    for sample_id, pose_sample in sorted(by_sample_id.items()):
        frame_path = args.dir / f"frame_{sample_id}.png"
        if not frame_path.exists():
            print(f"skip sample_id={sample_id}: no matching {frame_path.name}")
            skipped += 1
            continue

        img = cv2.imread(str(frame_path))
        detection = detector.detect(img)
        pose = detector.estimate_pose(detection, camera_matrix, dist_coeffs, min_corners=args.min_corners)
        if pose is None:
            print(f"skip {frame_path.name}: board pose not found "
                  f"({detection.num_charuco_corners} corners, need >= {args.min_corners})")
            skipped += 1
            continue

        R_gripper2base, t_gripper2base = robot_pose_sample_to_gripper2base(pose_sample)
        samples.append(PoseSample(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=pose.rotation_matrix,
            t_target2cam=pose.tvec.reshape(3, 1),
        ))
    return samples, skipped


def main():
    args = parse_args()
    cfg = load_config(args.config)
    camera_matrix, dist_coeffs = load_intrinsics(args.camera_matrix)
    detector = CharucoBoardDetector(cfg)

    if args.pose_jsonl is not None:
        samples, skipped = _collect_from_pose_jsonl(args, detector, camera_matrix, dist_coeffs)
    else:
        samples, skipped = _collect_from_yaml_dir(args, detector, camera_matrix, dist_coeffs)

    print(f"Usable samples: {len(samples)} (skipped {skipped})")
    result = solve_hand_eye(samples, mode=args.mode)
    save_result_yaml(result, args.mode, args.out)

    print(f"Solved {result.label}:")
    print(result.as_4x4())
    print(f"Saved to {args.out}")


if __name__ == "__main__":
    main()
