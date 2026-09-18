#!/usr/bin/env python3
"""ZED calibration probe — stage 2/3 (read-only camera info + up to 3 static
frames + ChArUco detection).

Rules this script must respect (see task log for full context):
- Never call any camera *set* method (exposure, gain, etc.) — read-only.
- Capture at most 3 static frames, saved under calib_data/probe/.
- Run ChArUco detection with the project's real board_config.yaml, save an
  overlay per frame, and report marker/corner counts.
- Never mark results as metric-valid — square size has not been physically
  measured/confirmed yet, so everything is tagged metric-unconfirmed.
- No FS100/robot_integration code is imported or touched here.
"""
from __future__ import annotations

import importlib
import json
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent  # .../mh5_zed_calibration
PARENT = ROOT.parent
if str(PARENT) not in sys.path:
    sys.path.insert(0, str(PARENT))

# The library modules use relative imports (`from . import compat`), which
# resolve fine under any package name. Only the *scripts* hardcode the
# literal name "camera_calibration"; this probe avoids that by importing
# the package under its real directory name and aliasing it, so `from
# camera_calibration.x import y` style imports would also work if needed.
pkg = importlib.import_module(ROOT.name)
sys.modules.setdefault("camera_calibration", pkg)

from camera_calibration.config import DEFAULT_CONFIG_PATH, load_config  # noqa: E402
from camera_calibration.detector import CharucoBoardDetector  # noqa: E402

import pyzed.sl as sl  # noqa: E402

PROBE_DIR = ROOT / "calib_data" / "probe"
LOG_PATH = ROOT / "logs" / "probe_stage2_camera.log"


def main() -> int:
    PROBE_DIR.mkdir(parents=True, exist_ok=True)
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = []

    def log(msg: str) -> None:
        print(msg)
        lines.append(msg)

    log(f"時間: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    log("操作: read-only camera probe, 最多3張靜態frame, 不修改曝光/其他相機設定, 不連FS100/不送機器人指令")
    log("")

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    cam = sl.Camera()
    status = cam.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        log(f"ERROR: 開啟相機失敗: {status}")
        LOG_PATH.write_text("\n".join(lines), encoding="utf-8")
        return 1

    try:
        info = cam.get_camera_information()
        cam_model = info.camera_model
        serial = info.serial_number
        cfg_res = info.camera_configuration.camera_resolution
        cfg_fps = info.camera_configuration.camera_fps
        log(f"camera_model: {cam_model}")
        log(f"serial_number: {serial}")
        log(f"resolution: {cfg_res.width}x{cfg_res.height}")
        log(f"fps: {cfg_fps}")

        try:
            exposure_raw = cam.get_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE)
            log(f"current_exposure_setting (read-only, 未修改): {exposure_raw}")
        except Exception as exc:  # noqa: BLE001
            log(f"讀取曝光值失敗(非致命,未嘗試修改任何設定): {exc!r}")

        board_cfg = load_config(DEFAULT_CONFIG_PATH)
        log(
            "board_config: name={} squares_x={} squares_y={} square_length_m={} "
            "marker_length_m={} dictionary={} legacy_pattern={}".format(
                board_cfg.name,
                board_cfg.squares_x,
                board_cfg.squares_y,
                board_cfg.square_length_m,
                board_cfg.marker_length_m,
                board_cfg.dictionary,
                board_cfg.legacy_pattern,
            )
        )
        detector = CharucoBoardDetector(board_cfg)

        image_mat = sl.Mat()
        max_frames = 3
        captured = 0
        attempts = 0
        results_summary = []
        while captured < max_frames and attempts < max_frames * 5:
            attempts += 1
            grab_status = cam.grab()
            if grab_status != sl.ERROR_CODE.SUCCESS:
                log(f"grab失敗(第{attempts}次嘗試): {grab_status}")
                continue
            cam.retrieve_image(image_mat, sl.VIEW.LEFT)
            bgra = image_mat.get_data()
            bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            captured += 1
            frame_path = PROBE_DIR / f"frame_{captured:02d}.png"
            cv2.imwrite(str(frame_path), bgr)
            log(f"已擷取 frame {captured}: {frame_path}")

            detection = detector.detect(bgr)
            overlay = detector.draw_overlay(bgr, detection)
            overlay_path = PROBE_DIR / f"frame_{captured:02d}_overlay.png"
            cv2.imwrite(str(overlay_path), overlay)
            results_summary.append(
                {
                    "frame": captured,
                    "num_markers": detection.num_markers,
                    "num_charuco_corners": detection.num_charuco_corners,
                    "overlay_path": str(overlay_path),
                }
            )
            log(
                f"frame {captured} ChArUco偵測: markers={detection.num_markers} "
                f"corners={detection.num_charuco_corners} overlay={overlay_path}"
            )

        log("")
        log("=== 結果標記 ===")
        log(
            "metric-unconfirmed: 尚未收到方格實際尺寸(30mm)的實測確認,"
            "本次所有偵測結果僅供結構性確認(是否可偵測到 marker/corner),"
            "不得視為已驗證的度量校正結果"
        )

        summary_path = PROBE_DIR / "probe_summary.json"
        summary_path.write_text(
            json.dumps(
                {
                    "camera_model": str(cam_model),
                    "serial_number": serial,
                    "resolution": f"{cfg_res.width}x{cfg_res.height}",
                    "fps": cfg_fps,
                    "frames": results_summary,
                    "metric_status": "metric-unconfirmed",
                },
                indent=2,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )
        log(f"summary JSON寫入: {summary_path}")
        rc = 0
    except Exception as exc:  # noqa: BLE001
        log(f"ERROR: probe 過程發生例外: {exc!r}")
        rc = 1
    finally:
        cam.close()
        log("相機已關閉 (cam.close())")

    LOG_PATH.write_text("\n".join(lines), encoding="utf-8")
    print(f"LOG_WRITTEN:{LOG_PATH}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
