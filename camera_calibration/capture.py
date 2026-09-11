"""Frame sources for detection: image files, a generic webcam, or a ZED
camera. Nothing in this module is imported or executed by the test suite —
tests must never touch real hardware.

The ZED path is a soft dependency: if pyzed isn't installed, importing this
module still succeeds, and only actually instantiating ZedCameraSource
raises a clear RuntimeError. This lets detect_offline.py / smoke tests run
on machines with no ZED SDK at all.
"""
from __future__ import annotations

import glob
from pathlib import Path
from typing import Iterator, List, Optional, Protocol

import cv2
import numpy as np


class FrameSource(Protocol):
    def read(self) -> Optional[np.ndarray]: ...
    def release(self) -> None: ...


class ImageFolderSource:
    """Offline source: iterate over image files in a directory. Used for
    testing the detection pipeline without any camera connected."""

    def __init__(self, folder: Path | str, pattern: str = "*.png"):
        self.paths: List[str] = sorted(glob.glob(str(Path(folder) / pattern)))
        if not self.paths:
            raise FileNotFoundError(f"No images matching {pattern!r} in {folder}")
        self._idx = 0

    def __iter__(self) -> Iterator[np.ndarray]:
        for p in self.paths:
            img = cv2.imread(p)
            if img is None:
                continue
            yield img

    def read(self) -> Optional[np.ndarray]:
        if self._idx >= len(self.paths):
            return None
        img = cv2.imread(self.paths[self._idx])
        self._idx += 1
        return img

    def release(self) -> None:
        pass


class GenericCameraSource:
    """Any UVC-compatible webcam via cv2.VideoCapture. Use this to exercise
    the live-detection loop without needing the ZED SDK installed."""

    def __init__(self, index: int = 0):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {index}")

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self) -> None:
        self.cap.release()


class ZedCameraSource:
    """ZED stereo camera via the pyzed SDK. Only instantiate this if the
    ZED SDK + pyzed are actually installed and a camera is physically
    connected; this class never gets constructed by tests or by default
    scripts."""

    def __init__(self, resolution: str = "HD720", fps: int = 30, depth: bool = False):
        try:
            import pyzed.sl as sl
        except ImportError as exc:
            raise RuntimeError(
                "pyzed (ZED SDK Python API) is not installed. Install the ZED "
                "SDK from stereolabs.com, then `pip install pyzed` inside the "
                "matching env, or use GenericCameraSource / ImageFolderSource "
                "instead for testing without a ZED camera."
            ) from exc

        self._sl = sl
        init_params = sl.InitParameters()
        init_params.camera_resolution = getattr(sl.RESOLUTION, resolution)
        init_params.camera_fps = fps
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE if depth else sl.DEPTH_MODE.NONE

        self.cam = sl.Camera()
        status = self.cam.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED camera open failed: {status}")
        self._image = sl.Mat()

    def read(self) -> Optional[np.ndarray]:
        sl = self._sl
        if self.cam.grab() != sl.ERROR_CODE.SUCCESS:
            return None
        self.cam.retrieve_image(self._image, sl.VIEW.LEFT)
        bgra = self._image.get_data()
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)

    def get_left_intrinsics(self):
        """Returns (camera_matrix, dist_coeffs) for the left lens from the
        ZED factory calibration, as read by the SDK at the configured
        resolution."""
        calib = self.cam.get_camera_information().camera_configuration.calibration_parameters.left_cam
        camera_matrix = np.array(
            [[calib.fx, 0, calib.cx], [0, calib.fy, calib.cy], [0, 0, 1]], dtype=np.float64
        )
        dist_coeffs = np.array(calib.disto[:5], dtype=np.float64)
        return camera_matrix, dist_coeffs

    def release(self) -> None:
        self.cam.close()
