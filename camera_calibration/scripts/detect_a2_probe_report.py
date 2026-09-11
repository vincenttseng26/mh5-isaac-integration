#!/usr/bin/env python3
"""Offline ChArUco detection report for the A2 board against already-captured
probe frames. No camera/ZED SDK needed, no new frames captured, no camera
settings touched.

For each input frame this produces:
  - <frame>_a2_overlay.png: detected markers/corners drawn on the frame,
    using the real A2 config (does not overwrite any existing overlay from
    a different board's probe run).
  - one entry in the JSON summary with markers, marker ids, charuco corner
    count/ids, plus:
      * a legacy_pattern cross-check (True vs False) so the empirical SVG
        pixel-diff determination in configs/a2_board_source/provenance.json
        is corroborated (or contradicted) by real detections, not assumed.
      * a wrong-dictionary control test (same board geometry, a different
        ArUco dictionary) to confirm markers are only found because the
        dictionary matches, not from an overly permissive detector.

Example:
    python detect_a2_probe_report.py \
        --config ../configs/board_config_a2.yaml \
        --dir ../calib_data/cyc_probe \
        --pattern "frame_0[0-9].png" \
        --wrong-dictionary DICT_6X6_250 \
        --out-summary ../calib_data/cyc_probe/a2_probe_summary.json
"""
import argparse
import json
import sys
from pathlib import Path

import _bootstrap  # noqa: F401

import cv2

from camera_calibration.config import BoardConfig, DEFAULT_CONFIG_PATH, load_config
from camera_calibration.isolation import run_isolated

A2_PROVENANCE_PATH = (
    Path(__file__).resolve().parents[1] / "configs" / "a2_board_source" / "provenance.json"
)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH.parent / "board_config_a2.yaml")
    p.add_argument("--dir", type=Path, required=True, help="directory containing already-captured frames")
    p.add_argument("--pattern", default="frame_0[0-9].png", help="glob pattern for input frames (must exclude existing *_overlay.png)")
    p.add_argument("--wrong-dictionary", default="DICT_6X6_250", help="dictionary used for the rejection control test")
    p.add_argument("--out-summary", type=Path, default=None, help="defaults to <dir>/a2_probe_summary.json")
    p.add_argument("--overlay-suffix", default="_a2_overlay.png")
    p.add_argument("--timeout", type=float, default=30.0)
    return p.parse_args()


def iter_frames(directory: Path, pattern: str):
    for path in sorted(directory.glob(pattern)):
        if "overlay" in path.stem:
            continue
        yield path


def detect_annotated(image_path: Path, cfg: BoardConfig, timeout: float):
    img = cv2.imread(str(image_path))
    if img is None:
        return None, f"cv2.imread returned None for {image_path}"
    result = run_isolated(
        "camera_calibration.detector",
        "detect_and_annotate_in_subprocess",
        args=(img, cfg.to_dict()),
        timeout=timeout,
    )
    return result, None


def detect_counts_only(image_path: Path, cfg: BoardConfig, timeout: float):
    img = cv2.imread(str(image_path))
    if img is None:
        return None, f"cv2.imread returned None for {image_path}"
    result = run_isolated(
        "camera_calibration.detector",
        "detect_in_subprocess",
        args=(img, cfg.to_dict()),
        timeout=timeout,
    )
    return result, None


def main():
    args = parse_args()
    cfg = load_config(args.config)
    cfg.validate()

    control_cfg = BoardConfig.from_dict({**cfg.to_dict(), "dictionary": args.wrong_dictionary})

    out_summary = args.out_summary or (args.dir / "a2_probe_summary.json")

    frames_report = []
    exit_code = 0

    for image_path in iter_frames(args.dir, args.pattern):
        frame_entry = {"image": str(image_path)}

        # 1) Real detection with the determined config (legacy_pattern as configured).
        result, err = detect_annotated(image_path, cfg, args.timeout)
        if err:
            frame_entry["error"] = err
            exit_code = 1
            frames_report.append(frame_entry)
            print(f"{image_path}: FAILED ({err})")
            continue
        if not result.ok:
            frame_entry["error"] = f"{result.status}: {result.error}"
            exit_code = 1
            frames_report.append(frame_entry)
            print(f"{image_path}: {result.status.upper()} - {result.error}")
            continue

        value = result.value
        overlay_path = image_path.with_name(image_path.stem + args.overlay_suffix)
        cv2.imwrite(str(overlay_path), value["overlay"])

        frame_entry.update({
            "num_markers": value["num_markers"],
            "marker_ids": value["marker_ids"],
            "num_charuco_corners": value["num_charuco_corners"],
            "charuco_ids": value["charuco_ids"],
            "theoretical_max_charuco_corners": cfg.num_internal_corners,
            "overlay_path": str(overlay_path),
        })

        # 2) legacy_pattern cross-check: run the opposite setting on the same
        #    frame and record its corner count too, so the SVG-pixel-diff
        #    determination in provenance.json is corroborated against real
        #    captured frames, not just the synthetic render.
        alt_cfg = BoardConfig.from_dict({**cfg.to_dict(), "legacy_pattern": not cfg.legacy_pattern})
        alt_result, alt_err = detect_counts_only(image_path, alt_cfg, args.timeout)
        if alt_err or not alt_result.ok:
            frame_entry["legacy_pattern_cross_check"] = {
                "configured_legacy_pattern": cfg.legacy_pattern,
                "alternate_legacy_pattern": alt_cfg.legacy_pattern,
                "alternate_error": alt_err or alt_result.error,
            }
        else:
            frame_entry["legacy_pattern_cross_check"] = {
                "configured_legacy_pattern": cfg.legacy_pattern,
                "configured_num_charuco_corners": value["num_charuco_corners"],
                "alternate_legacy_pattern": alt_cfg.legacy_pattern,
                "alternate_num_charuco_corners": alt_result.value["num_charuco_corners"],
            }

        # 3) wrong-dictionary control test: same board geometry, different
        #    dictionary -> markers should be rejected (near zero) to prove
        #    detections above are dictionary-specific, not incidental.
        control_result, control_err = detect_counts_only(image_path, control_cfg, args.timeout)
        if control_err or not control_result.ok:
            frame_entry["wrong_dictionary_control"] = {
                "dictionary": args.wrong_dictionary,
                "error": control_err or control_result.error,
            }
        else:
            frame_entry["wrong_dictionary_control"] = {
                "dictionary": args.wrong_dictionary,
                "num_markers": control_result.value["num_markers"],
                "num_charuco_corners": control_result.value["num_charuco_corners"],
                "rejected_as_expected": control_result.value["num_markers"] == 0,
            }

        frames_report.append(frame_entry)
        print(
            f"{image_path}: markers={value['num_markers']} "
            f"charuco_corners={value['num_charuco_corners']}/{cfg.num_internal_corners} "
            f"overlay={overlay_path}"
        )

    summary = {
        "board_config": {
            "config_path": str(args.config),
            **cfg.to_dict(),
            "theoretical_max_charuco_corners": cfg.num_internal_corners,
        },
        "provenance_file": str(A2_PROVENANCE_PATH),
        "capture_note": "frames were already captured beforehand; this run performs offline detection only, no new frames captured, no camera settings touched",
        "metric_status": "metric-unconfirmed",
        "metric_status_note": (
            "square_length_mm/marker_length_mm are design values from the source manifest.json. "
            "Not valid for metric pose/hand-eye until a user physically measures the printed "
            "100mm scale bar or a 45mm square and confirms 100% print scale."
        ),
        "frames": frames_report,
    }

    out_summary.parent.mkdir(parents=True, exist_ok=True)
    with open(out_summary, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"\nSummary written: {out_summary}")

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
