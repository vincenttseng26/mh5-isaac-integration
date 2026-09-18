"""Environment diagnostics for the cv2.aruco segfault problem.

A Python try/except cannot catch a native segfault — by the time control
would return to Python, the process is already dead. This module can only
gather evidence *around* the crash (see isolation.py for the part that
actually survives one). Run it standalone:

    python -m camera_calibration.diagnose

The single most common real-world cause of "cv2.aruco segfaults on 4.6" is
having more than one opencv-* pip package installed at once (e.g.
opencv-python AND opencv-contrib-python, or a -headless variant alongside a
GUI one). They ship conflicting native .so files under the same cv2 module
name, and whichever one wins the import can have ABI-mismatched aruco
symbols. This is checked for explicitly below.
"""
from __future__ import annotations

import sys
from typing import Any, Dict


def collect_diagnostics() -> Dict[str, Any]:
    info: Dict[str, Any] = {"python_version": sys.version}

    try:
        import cv2

        info["cv2_version"] = cv2.__version__
        info["cv2_file"] = getattr(cv2, "__file__", "<unknown>")
    except Exception as exc:  # pragma: no cover
        info["cv2_import_error"] = repr(exc)
        return info

    try:
        import importlib.metadata as importlib_metadata

        installed = {}
        for dist in importlib_metadata.distributions():
            try:
                dist_name = dist.metadata["Name"]
            except Exception:
                continue
            if dist_name and dist_name.lower().startswith("opencv"):
                installed[dist_name] = dist.version
        info["opencv_pip_packages"] = installed
        if len(installed) > 1:
            info["warning_conflicting_opencv_packages"] = (
                "Multiple opencv-* pip packages are installed together: "
                f"{sorted(installed)}. Mixing opencv-python / "
                "opencv-contrib-python / *-headless variants is a well-known "
                "cause of native segfaults in cv2.aruco. Fix: "
                "'pip uninstall opencv-python opencv-python-headless "
                "opencv-contrib-python opencv-contrib-python-headless' then "
                "reinstall exactly ONE contrib package."
            )
    except Exception as exc:  # pragma: no cover
        info["pip_scan_error"] = repr(exc)

    try:
        from cv2 import aruco

        info["has_new_aruco_api (ArucoDetector/CharucoDetector)"] = hasattr(
            aruco, "CharucoDetector"
        ) and hasattr(aruco, "ArucoDetector")
        info["has_legacy_aruco_api (CharucoBoard_create)"] = hasattr(
            aruco, "CharucoBoard_create"
        )
        if info["cv2_version"].startswith("4.6"):
            info["warning_known_bad_version"] = (
                "OpenCV 4.6.x has multiple reported cv2.aruco native crashes "
                "(segfaults in detectMarkers/interpolateCornersCharuco under "
                "certain thread/allocator configs). Recommended fix: upgrade "
                "to a single, matching opencv-contrib-python>=4.8 build."
            )
    except Exception as exc:
        info["aruco_import_error"] = repr(exc)

    return info


def format_report(info: Dict[str, Any]) -> str:
    lines = ["=== camera_calibration environment diagnostics ==="]
    for key, value in info.items():
        lines.append(f"{key}: {value}")
    return "\n".join(lines)


def main() -> None:
    info = collect_diagnostics()
    print(format_report(info))


if __name__ == "__main__":
    main()
