#!/usr/bin/env python3
"""A2 ChArUco board fixed-scene EXTRINSIC POSE STABILITY session.

This is a read-only-camera diagnostic: with the A2 board and camera both
held still, it captures ~N frames, estimates the board pose in the camera
frame for each frame using the ZED SDK's own factory intrinsics (never
guessed), and reports how much that estimated pose jitters frame-to-frame
plus the reprojection error. That jitter/error is a proxy for "is this
camera+board+lighting setup stable and accurate enough to be worth using
for calibration data collection" -- it is NOT a hand-eye calibration and
must never be reported or reused as one (there is no robot pose involved
here at all).

Hard rules this script must respect:
- Never call any ZED *set* method (exposure, gain, white balance, ...) --
  read-only. Only get_camera_information / get_camera_settings / grab /
  retrieve_image / close are used.
- Camera resolution must actually resolve to 1280x720 (HD720) -- verified
  from camera_information after open, not assumed from the InitParameters
  request. Abort (exit 1) rather than silently proceeding at some other
  resolution.
- Camera intrinsics (fx, fy, cx, cy, distortion) come only from
  cam.get_camera_information().camera_configuration.calibration_parameters
  .left_cam -- never a hard-coded/guessed value, never a different
  resolution's parameters.
- No robot_integration / FS100 import or command anywhere in this file.
- Board geometry/legacy_pattern come only from configs/board_config_a2.yaml
  (via camera_calibration.config.load_config) -- never hand-edited here.
- metric_status is read from configs/a2_board_source/provenance.json at
  run time, not hard-coded, so this script can never silently claim a
  confirmation that hasn't actually been recorded there.

Usage (on a machine with the ZED SDK + a connected ZED camera, run in the
foreground so its own exit code is visible):
    python scripts/capture_a2_extrinsic_stability.py
    python scripts/capture_a2_extrinsic_stability.py --num-frames 30 \
        --min-markers 10 --min-corners 15
"""
from __future__ import annotations

import argparse
import importlib
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent  # .../camera_calibration or .../mh5_zed_calibration
PARENT = ROOT.parent
if str(PARENT) not in sys.path:
    sys.path.insert(0, str(PARENT))

pkg = importlib.import_module(ROOT.name)
sys.modules.setdefault("camera_calibration", pkg)

from camera_calibration.config import load_config  # noqa: E402
from camera_calibration.isolation import run_isolated  # noqa: E402
from camera_calibration import pose_stats  # noqa: E402

import pyzed.sl as sl  # noqa: E402

A2_CONFIG_PATH = ROOT / "configs" / "board_config_a2.yaml"
PROVENANCE_PATH = ROOT / "configs" / "a2_board_source" / "provenance.json"
REQUIRED_RESOLUTION = (1280, 720)
DISCLAIMER = (
    "This session is a FIXED-SCENE EXTRINSIC POSE STABILITY check only "
    "(board pose in the camera frame, estimated per-frame from ChArUco "
    "detection + the camera's own factory intrinsics). It does NOT involve "
    "a robot pose, is NOT a hand-eye calibration, and must not be reported "
    "or reused as one."
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--num-frames", type=int, default=30)
    p.add_argument("--frame-interval-s", type=float, default=0.3,
                   help="delay between grabs so consecutive frames are independent sensor reads, "
                        "even though the physical scene is held fixed")
    p.add_argument("--min-markers", type=int, default=10, help="gate: minimum ArUco markers detected")
    p.add_argument("--min-corners", type=int, default=15, help="gate: minimum interpolated ChArUco corners")
    p.add_argument("--grab-attempt-multiplier", type=int, default=5,
                   help="max grab attempts = num_frames * this, to tolerate occasional grab failures")
    p.add_argument("--session-name", type=str, default=None,
                   help="defaults to session_<UTC timestamp>_a2_extrinsic_stability")
    p.add_argument("--min-valid-frame-rate", type=float, default=0.8)
    p.add_argument("--max-translation-jitter-mm", type=float, default=5.0,
                   help="gate: max allowed centroid deviation (mm) of any single valid frame's translation")
    p.add_argument("--max-rotation-jitter-deg", type=float, default=1.0,
                   help="gate: max allowed angular deviation (deg) of any single valid frame's rotation")
    p.add_argument("--max-reprojection-rms-px", type=float, default=1.0,
                   help="gate: max allowed MEAN reprojection RMS (px) across valid frames")
    p.add_argument("--detect-timeout-s", type=float, default=30.0)
    return p.parse_args()


def build_camera_matrix_and_dist(calib_left) -> tuple[np.ndarray, np.ndarray]:
    camera_matrix = np.array(
        [[calib_left.fx, 0.0, calib_left.cx], [0.0, calib_left.fy, calib_left.cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    disto = list(calib_left.disto)
    if len(disto) not in (4, 5, 8, 12, 14):
        disto = disto[:5]
    dist_coeffs = np.array(disto, dtype=np.float64)
    return camera_matrix, dist_coeffs


def diagnose_failure(reasons: list[str], mean_corners_frac: float, reproj_mean: float | None,
                      translation_max_mm: float | None, rotation_max_deg: float | None) -> str:
    hints = []
    if any("valid_frame_rate" in r for r in reasons) and mean_corners_frac < 0.3:
        hints.append(
            "有效影格率低且平均可偵測角點比例低(<30%理論值)：優先檢查『板子』"
            "(距離太遠/角度太斜/被遮擋) 或 『光線』(反光/對比不足/過暗)。"
        )
    if reproj_mean is not None and reproj_mean > 0.0 and any("reprojection" in r for r in reasons):
        hints.append(
            "重投影誤差偏高但角點數量尚可：檢查『板子』是否平整(彎曲/翹起)，"
            "或『相機』對焦是否清晰(避免模糊)。內參已直接取自ZED SDK,非猜測值,"
            "通常不是內參本身的問題。"
        )
    if (translation_max_mm is not None and any("translation" in r for r in reasons)) or (
        rotation_max_deg is not None and any("rotation" in r for r in reasons)
    ):
        hints.append(
            "重投影誤差與角點數量正常,但姿態(translation/rotation)抖動偏高："
            "檢查『相機』是否有震動、自動對焦擾動、或USB頻寬造成的擷取延遲,"
            "也請確認場景在整個擷取期間『真的』完全靜止(桌面/腳架/板子未被碰到)。"
        )
    if not hints:
        hints.append("多項指標未達標，建議依序檢查：板子擺放與光線 -> 相機穩定度與對焦 -> 重新執行本腳本確認。")
    return " ".join(hints)


def main() -> int:
    args = parse_args()

    lines: list[str] = []

    def log(msg: str) -> None:
        print(msg)
        lines.append(msg)

    log(f"時間: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    log("操作: A2 ChArUco 固定場景外參姿態穩定度檢查 (read-only camera, 不修改曝光/其他相機設定, "
        "不連FS100/不送機器人指令, 不是hand-eye calibration)")
    log(DISCLAIMER)
    log("")

    if not PROVENANCE_PATH.exists():
        log(f"ERROR: 找不到 provenance.json: {PROVENANCE_PATH}")
        return 1
    provenance = json.loads(PROVENANCE_PATH.read_text(encoding="utf-8"))
    metric_status = provenance.get("metric_status")
    metric_confirmation = provenance.get("metric_confirmation")
    log(f"metric_status (讀自 provenance.json,非硬編碼): {metric_status}")
    if metric_status != "metric-valid":
        log("ERROR: metric_status 不是 metric-valid，安全停止 -- 度量姿態在使用者實測確認前不可信賴。")
        return 1
    log(f"metric_confirmation: {json.dumps(metric_confirmation, ensure_ascii=False)}")

    board_cfg = load_config(A2_CONFIG_PATH)
    log(
        "board_config: name={} squares_x={} squares_y={} square_length_m={} "
        "marker_length_m={} dictionary={} legacy_pattern={}".format(
            board_cfg.name, board_cfg.squares_x, board_cfg.squares_y, board_cfg.square_length_m,
            board_cfg.marker_length_m, board_cfg.dictionary, board_cfg.legacy_pattern,
        )
    )

    session_name = args.session_name or time.strftime("session_%Y%m%dT%H%M%SZ", time.gmtime()) + "_a2_extrinsic_stability"
    session_dir = ROOT / "calib_data" / session_name
    session_dir.mkdir(parents=True, exist_ok=True)
    log(f"session_dir: {session_dir}")

    log_dir = ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"{session_name}.log"

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    cam = sl.Camera()
    status = cam.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        log(f"ERROR: 開啟相機失敗: {status}")
        log_path.write_text("\n".join(lines), encoding="utf-8")
        return 1

    rc = 0
    try:
        info = cam.get_camera_information()
        cam_model = str(info.camera_model)
        serial = info.serial_number
        cfg_res = info.camera_configuration.camera_resolution
        cfg_fps = info.camera_configuration.camera_fps
        actual_resolution = (cfg_res.width, cfg_res.height)
        log(f"camera_model: {cam_model} serial_number: {serial} resolution: {actual_resolution} fps: {cfg_fps}")

        if actual_resolution != REQUIRED_RESOLUTION:
            log(f"ERROR: 實際解析度 {actual_resolution} != 要求的 {REQUIRED_RESOLUTION}，安全停止(不得用其他解析度內參)。")
            return 1

        try:
            exposure_raw = cam.get_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE)
            gain_raw = cam.get_camera_settings(sl.VIDEO_SETTINGS.GAIN)
            log(f"current_exposure_setting (read-only,未修改): {exposure_raw}")
            log(f"current_gain_setting (read-only,未修改): {gain_raw}")
        except Exception as exc:  # noqa: BLE001
            log(f"讀取曝光/增益值失敗(非致命,未嘗試修改任何設定): {exc!r}")

        calib_left = info.camera_configuration.calibration_parameters.left_cam
        camera_matrix, dist_coeffs = build_camera_matrix_and_dist(calib_left)
        log(f"left intrinsics (ZED SDK, 1280x720, read-only): fx={calib_left.fx} fy={calib_left.fy} "
            f"cx={calib_left.cx} cy={calib_left.cy} dist={dist_coeffs.tolist()}")

        intrinsics_record = {
            "camera_model": cam_model,
            "serial_number": serial,
            "resolution": {"width": actual_resolution[0], "height": actual_resolution[1]},
            "fps": cfg_fps,
            "source": "ZED SDK factory calibration, read-only via get_camera_information() "
                      "at the actually-negotiated 1280x720 mode -- not guessed, not another resolution's values",
            "read_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "fx": calib_left.fx, "fy": calib_left.fy, "cx": calib_left.cx, "cy": calib_left.cy,
            "dist_coeffs": dist_coeffs.tolist(),
            "camera_matrix_3x3": camera_matrix.tolist(),
        }
        (session_dir / "camera_intrinsics.json").write_text(
            json.dumps(intrinsics_record, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        try:
            import yaml
            with open(session_dir / "camera_intrinsics.yaml", "w", encoding="utf-8") as f:
                yaml.safe_dump(intrinsics_record, f, sort_keys=False)
        except ModuleNotFoundError:
            log("提醒: 找不到 pyyaml，跳過 camera_intrinsics.yaml (json 版本仍會寫出)")

        image_mat = sl.Mat()
        captured_frames: list[dict] = []
        attempts = 0
        max_attempts = args.num_frames * args.grab_attempt_multiplier
        while len(captured_frames) < args.num_frames and attempts < max_attempts:
            attempts += 1
            grab_status = cam.grab()
            if grab_status != sl.ERROR_CODE.SUCCESS:
                log(f"grab失敗(第{attempts}次嘗試): {grab_status}")
                continue
            cam.retrieve_image(image_mat, sl.VIEW.LEFT)
            bgra = image_mat.get_data()
            bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            idx = len(captured_frames) + 1
            frame_path = session_dir / f"frame_{idx:02d}.png"
            cv2.imwrite(str(frame_path), bgr)
            ts = {"monotonic_ns": time.monotonic_ns(), "utc_iso8601": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
            captured_frames.append({"index": idx, "path": frame_path, "image": bgr, "timestamp": ts})
            log(f"已擷取 frame {idx}/{args.num_frames}: {frame_path}")
            time.sleep(args.frame_interval_s)

        log(f"共擷取 {len(captured_frames)} 張 frame (嘗試 {attempts} 次)")

        jsonl_path = session_dir / "per_frame.jsonl"
        per_frame_records = []
        with open(jsonl_path, "w", encoding="utf-8") as jsonl_f:
            for frame in captured_frames:
                idx = frame["index"]
                result = run_isolated(
                    "camera_calibration.detector",
                    "detect_pose_and_annotate_in_subprocess",
                    args=(frame["image"], board_cfg.to_dict(), camera_matrix.tolist(), dist_coeffs.tolist()),
                    kwargs={"min_markers": args.min_markers, "min_corners": args.min_corners},
                    timeout=args.detect_timeout_s,
                )
                record = {
                    "frame": idx,
                    "timestamp": frame["timestamp"],
                    "image_path": str(frame["path"]),
                    "gate_thresholds": {"min_markers": args.min_markers, "min_corners": args.min_corners},
                }
                if not result.ok:
                    record["error"] = f"{result.status}: {result.error}"
                    log(f"frame {idx}: 偵測失敗 ({result.status})")
                    per_frame_records.append(record)
                    jsonl_f.write(json.dumps(record, ensure_ascii=False) + "\n")
                    continue

                value = result.value
                overlay_path = session_dir / f"frame_{idx:02d}_overlay.png"
                cv2.imwrite(str(overlay_path), value["overlay"])
                record.update({
                    "num_markers": value["num_markers"],
                    "num_charuco_corners": value["num_charuco_corners"],
                    "theoretical_max_charuco_corners": board_cfg.num_internal_corners,
                    "gate_pass": value["gate_pass"],
                    "pose_ok": value["pose_ok"],
                    "overlay_path": str(overlay_path),
                })
                if value["pose_ok"]:
                    T_camera_board = pose_stats.rvec_tvec_to_4x4(value["rvec"], value["tvec"])
                    T_board_camera = pose_stats.invert_4x4(T_camera_board)
                    quat = pose_stats.rotation_matrix_to_quat_wxyz(T_camera_board[:3, :3])
                    record["T_camera_board"] = {
                        "description": "Transforms a point in the BOARD frame into the CAMERA frame: "
                                       "X_camera_h = T_camera_board @ X_board_h (homogeneous).",
                        "matrix_4x4": T_camera_board.tolist(),
                        "translation_m": T_camera_board[:3, 3].tolist(),
                        "quaternion_wxyz": quat.tolist(),
                        "axis_convention": "camera: OpenCV convention (+X right, +Y down, +Z forward out of lens). "
                                            "board: origin at top-left outer corner, +X right, +Y down, +Z toward printed face "
                                            "(see configs/a2_board_source/provenance.json manifest_snapshot.board_frame).",
                    }
                    record["T_board_camera"] = {
                        "description": "Inverse of T_camera_board: transforms a point in the CAMERA frame into the "
                                       "BOARD frame. X_board_h = T_board_camera @ X_camera_h (homogeneous).",
                        "matrix_4x4": T_board_camera.tolist(),
                        "translation_m": T_board_camera[:3, 3].tolist(),
                        "quaternion_wxyz": pose_stats.rotation_matrix_to_quat_wxyz(T_board_camera[:3, :3]).tolist(),
                    }
                    record["reprojection_error_px"] = value["reprojection_error_px"]
                    log(f"frame {idx}: markers={value['num_markers']} corners={value['num_charuco_corners']} "
                        f"pose_ok=True reproj_rms_px={value['reprojection_error_px']['rms']:.4f}")
                else:
                    log(f"frame {idx}: markers={value['num_markers']} corners={value['num_charuco_corners']} "
                        f"gate_pass={value['gate_pass']} pose_ok=False")
                per_frame_records.append(record)
                jsonl_f.write(json.dumps(record, ensure_ascii=False) + "\n")

        valid_records = [r for r in per_frame_records if r.get("pose_ok")]
        num_captured = len(captured_frames)
        valid_frame_rate = (len(valid_records) / num_captured) if num_captured else 0.0

        all_corners = [r.get("num_charuco_corners", 0) for r in per_frame_records]
        corners_stats_all = pose_stats.JitterStats.from_values(all_corners).to_dict() if all_corners else None
        mean_corners_frac = (corners_stats_all["mean"] / board_cfg.num_internal_corners) if corners_stats_all else 0.0

        results: dict = {
            "frames_captured": num_captured,
            "frames_attempted_grabs": attempts,
            "valid_frames": len(valid_records),
            "valid_frame_rate": valid_frame_rate,
            "corners_stats_all_frames": corners_stats_all,
            "theoretical_max_charuco_corners": board_cfg.num_internal_corners,
        }

        gate_reasons: list[str] = []
        reproj_mean = None
        translation_max_mm = None
        rotation_max_deg = None

        if valid_frame_rate < args.min_valid_frame_rate:
            gate_reasons.append(
                f"valid_frame_rate {valid_frame_rate:.2f} < min_valid_frame_rate {args.min_valid_frame_rate}"
            )

        if len(valid_records) >= 2:
            translations = [np.array(r["T_camera_board"]["translation_m"]) for r in valid_records]
            quats = [np.array(r["T_camera_board"]["quaternion_wxyz"]) for r in valid_records]
            reproj_values = [r["reprojection_error_px"]["rms"] for r in valid_records]

            t_jitter = pose_stats.translation_jitter(translations)
            r_jitter = pose_stats.rotation_jitter(quats)
            reproj_stats = pose_stats.reprojection_stats(reproj_values)

            results["translation_jitter"] = t_jitter
            results["rotation_jitter"] = r_jitter
            results["reprojection_stats_px"] = reproj_stats

            translation_max_mm = t_jitter["centroid_deviation_mm"]["max"]
            rotation_max_deg = r_jitter["angular_deviation_deg"]["max"]
            reproj_mean = reproj_stats["mean"]

            if translation_max_mm > args.max_translation_jitter_mm:
                gate_reasons.append(
                    f"translation centroid_deviation max {translation_max_mm:.3f}mm > "
                    f"max_translation_jitter_mm {args.max_translation_jitter_mm}"
                )
            if rotation_max_deg > args.max_rotation_jitter_deg:
                gate_reasons.append(
                    f"rotation angular_deviation max {rotation_max_deg:.3f}deg > "
                    f"max_rotation_jitter_deg {args.max_rotation_jitter_deg}"
                )
            if reproj_mean > args.max_reprojection_rms_px:
                gate_reasons.append(
                    f"reprojection rms mean {reproj_mean:.3f}px > max_reprojection_rms_px {args.max_reprojection_rms_px}"
                )
        else:
            gate_reasons.append(
                f"only {len(valid_records)} valid frame(s) -- need >= 2 to compute pose stability statistics"
            )

        gate_pass = len(gate_reasons) == 0
        results["quality_gate"] = {
            "pass": gate_pass,
            "reasons": gate_reasons,
            "thresholds": {
                "min_valid_frame_rate": args.min_valid_frame_rate,
                "max_translation_jitter_mm": args.max_translation_jitter_mm,
                "max_rotation_jitter_deg": args.max_rotation_jitter_deg,
                "max_reprojection_rms_px": args.max_reprojection_rms_px,
                "min_markers": args.min_markers,
                "min_corners": args.min_corners,
            },
        }
        if not gate_pass:
            results["quality_gate"]["diagnostic_hint"] = diagnose_failure(
                gate_reasons, mean_corners_frac, reproj_mean, translation_max_mm, rotation_max_deg
            )

        summary = {
            "session_name": session_name,
            "disclaimer": DISCLAIMER,
            "not_hand_eye_calibration": True,
            "metric_status": metric_status,
            "metric_confirmation": metric_confirmation,
            "board_config": {"config_path": str(A2_CONFIG_PATH), **board_cfg.to_dict()},
            "camera": intrinsics_record,
            "capture_params": {
                "num_frames_requested": args.num_frames,
                "frame_interval_s": args.frame_interval_s,
            },
            "results": results,
            "transform_convention": {
                "T_camera_board": "X_camera_h = T_camera_board @ X_board_h -- board frame expressed in the camera frame.",
                "T_board_camera": "X_board_h = T_board_camera @ X_camera_h -- inverse of T_camera_board.",
                "translation_units": "meters",
                "quaternion_order": "[w, x, y, z]",
                "rotation_axis_convention": "camera: OpenCV (+X right, +Y down, +Z forward). "
                                             "board: top-left outer corner origin, +X right, +Y down, +Z toward printed face.",
            },
            "next_steps": (
                [
                    "此次結果為固定場景外參姿態穩定度合格。下一步建議：在多個不同板子角度/距離/相機視角下重複本腳本"
                    "(每個姿態仍是靜態、read-only)，累積多組穩定度數據，確認在預期工作空間內都達標，再考慮進入"
                    "hand-eye 資料收集規劃。這一步本身仍不連 FS100、不送機器人指令 -- 是否/何時連接機器人由使用者另行決定。"
                ]
                if gate_pass
                else [
                    "品質未達標，已安全停止，尚不建議進行下一階段資料收集。請先依 quality_gate.diagnostic_hint 調整"
                    "後重新執行本腳本。"
                ]
            ),
        }
        summary_path = session_dir / "summary.json"
        summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
        log(f"summary 寫入: {summary_path}")
        log(f"per_frame JSONL 寫入: {jsonl_path}")
        log("")
        log(f"=== GATE: {'PASS' if gate_pass else 'FAIL'} ===")
        for r in gate_reasons:
            log(f"  - {r}")
        if not gate_pass:
            log(f"診斷建議: {results['quality_gate']['diagnostic_hint']}")

        rc = 0 if gate_pass else 2
    except Exception as exc:  # noqa: BLE001
        log(f"ERROR: 執行過程發生例外: {exc!r}")
        rc = 1
    finally:
        cam.close()
        log("相機已關閉 (cam.close())")

    log_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"LOG_WRITTEN:{log_path}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
