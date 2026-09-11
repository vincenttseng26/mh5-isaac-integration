"""Compatibility shim over cv2.aruco.

OpenCV's ArUco/ChArUco API changed shape across versions:

- <= 4.6 ("legacy"): aruco.Dictionary_get, aruco.CharucoBoard_create,
  aruco.DetectorParameters_create, aruco.detectMarkers(image, dict, ...),
  aruco.interpolateCornersCharuco, aruco.estimatePoseCharucoBoard.
- >= 4.7 ("new"): aruco.getPredefinedDictionary, aruco.CharucoBoard(...) as
  a class, aruco.DetectorParameters(), aruco.ArucoDetector,
  aruco.CharucoDetector, board.matchImagePoints() + cv2.solvePnP instead of
  estimatePoseCharucoBoard (removed in some 4.x builds).

Every function below picks the right call at runtime via hasattr checks so
the rest of the codebase never has to know which OpenCV it's running on.
This module intentionally does not try to fix native segfaults (that isn't
possible from Python) — see isolation.py for how detection calls are
sandboxed instead.
"""
from __future__ import annotations

from typing import Any, Optional, Tuple

import cv2
import numpy as np

try:
    from cv2 import aruco
except ImportError as exc:  # pragma: no cover - environment problem, not a bug
    raise ImportError(
        "cv2.aruco is not available. You likely have plain 'opencv-python' "
        "installed instead of 'opencv-contrib-python' (aruco lives in the "
        "contrib modules). See camera_calibration/README.md 'Known OpenCV "
        "issues' section."
    ) from exc

HAS_NEW_ARUCO_API = hasattr(aruco, "CharucoDetector") and hasattr(aruco, "ArucoDetector")
HAS_LEGACY_ARUCO_API = hasattr(aruco, "CharucoBoard_create")


def get_dictionary(name: str):
    if not hasattr(aruco, name):
        raise ValueError(f"Unknown ArUco dictionary name: {name!r}")
    dict_id = getattr(aruco, name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dict_id)
    return aruco.Dictionary_get(dict_id)  # legacy


def make_charuco_board(cfg, dictionary) -> Any:
    """Build a CharucoBoard object for the given BoardConfig.

    Priority: if the legacy aruco.CharucoBoard_create is available at all
    (OpenCV <= ~4.8, sometimes alongside the new class API during the
    transition), use it directly — it reproduces the exact square/marker
    arrangement that cfg.legacy_pattern documents, with no extra flag
    needed. Only fall back to the new CharucoBoard class (setting
    setLegacyPattern from cfg) when CharucoBoard_create has been removed.
    """
    if HAS_LEGACY_ARUCO_API:
        return aruco.CharucoBoard_create(
            cfg.squares_x,
            cfg.squares_y,
            cfg.square_length_m,
            cfg.marker_length_m,
            dictionary,
        )
    board = aruco.CharucoBoard(
        (cfg.squares_x, cfg.squares_y), cfg.square_length_m, cfg.marker_length_m, dictionary
    )
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(bool(cfg.legacy_pattern))
    return board


def draw_board(board, size_px: Tuple[int, int], margin_size: int = 0, border_bits: int = 1) -> np.ndarray:
    if hasattr(board, "generateImage"):
        return board.generateImage(size_px, marginSize=margin_size, borderBits=border_bits)
    return board.draw(size_px, marginSize=margin_size, borderBits=border_bits)


def make_detector_params():
    if hasattr(aruco, "DetectorParameters") and HAS_NEW_ARUCO_API:
        return aruco.DetectorParameters()
    return aruco.DetectorParameters_create()


def detect_markers(image: np.ndarray, dictionary, parameters=None):
    """Returns (marker_corners, marker_ids, rejected)."""
    params = parameters if parameters is not None else make_detector_params()
    if HAS_NEW_ARUCO_API:
        detector = aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers(image)
    return aruco.detectMarkers(image, dictionary, parameters=params)


def interpolate_charuco(marker_corners, marker_ids, image: np.ndarray, board):
    """Returns (charuco_corners, charuco_ids), either possibly None."""
    if HAS_NEW_ARUCO_API and hasattr(aruco, "CharucoDetector"):
        detector = aruco.CharucoDetector(board)
        charuco_corners, charuco_ids, _m_corners, _m_ids = detector.detectBoard(image)
        return charuco_corners, charuco_ids
    if marker_ids is None or len(marker_ids) == 0:
        return None, None
    retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, image, board
    )
    if not retval:
        return None, None
    return charuco_corners, charuco_ids


def estimate_pose_charuco(
    charuco_corners: np.ndarray,
    charuco_ids: np.ndarray,
    board,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
    """Returns (success, rvec, tvec)."""
    if hasattr(aruco, "estimatePoseCharucoBoard"):
        ok, rvec, tvec = aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None
        )
        return bool(ok), rvec, tvec
    # New API removed estimatePoseCharucoBoard in some builds; fall back to
    # matchImagePoints + solvePnP, which is the documented replacement.
    if not hasattr(board, "matchImagePoints"):
        raise RuntimeError(
            "Neither aruco.estimatePoseCharucoBoard nor board.matchImagePoints "
            "is available in this OpenCV build; cannot estimate board pose."
        )
    obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
    if obj_points is None or len(obj_points) < 4:
        return False, None, None
    ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
    return bool(ok), rvec, tvec


def get_board_object_points(board) -> np.ndarray:
    """Nx3 array of every ChArUco corner's 3D position in the board frame,
    indexed by charuco corner id. Present under both API generations, just
    under a different accessor."""
    if hasattr(board, "getChessboardCorners"):
        return np.asarray(board.getChessboardCorners())
    return np.asarray(board.chessboardCorners)  # legacy


def build_info() -> str:
    return cv2.getBuildInformation()
