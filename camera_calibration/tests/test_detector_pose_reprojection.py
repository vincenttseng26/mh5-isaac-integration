"""End-to-end (still hardware-free) test for
detector.detect_pose_and_annotate_in_subprocess: renders a synthetic A2
board, builds a synthetic pinhole camera matrix that reproduces the
renderer's fronto-parallel orthographic projection exactly, and checks that
pose estimation + reprojection error come back sane. This is the same
function scripts/capture_a2_extrinsic_stability.py calls per real frame, so
this test is what catches a broken camera_matrix/dist_coeffs plumbing or a
reprojection-math bug before it ever touches the real ZED camera.

No camera or robot is touched anywhere in this file.
"""
import numpy as np
import pytest

from camera_calibration.config import BoardConfig, load_config
from camera_calibration.isolation import run_isolated

TEST_DPI = 96
METERS_PER_INCH = 0.0254


def _render(cfg: BoardConfig):
    return run_isolated(
        "camera_calibration.board_generator",
        "render_board_png",
        args=(cfg,),
        kwargs={"dpi": TEST_DPI},
        timeout=30.0,
    )


def _synthetic_pinhole_for_orthographic_render(dpi: int) -> np.ndarray:
    """render_board_png draws the board fronto-parallel, origin at the
    top-left corner, at exactly px_per_m = dpi/METERS_PER_INCH pixels per
    board-frame meter, with no other offset. A pinhole camera sitting on the
    board's Z axis (R=I) at distance Z0=1m with fx=fy=px_per_m, cx=cy=0
    reproduces that exact mapping (u = fx*X/Z0 = px_per_m*X), so pose
    estimation against this synthetic camera_matrix should recover R~=I,
    t~=[0,0,1] with near-zero reprojection error."""
    px_per_m = dpi / METERS_PER_INCH
    return np.array([[px_per_m, 0, 0], [0, px_per_m, 0], [0, 0, 1]], dtype=np.float64)


@pytest.fixture(scope="module")
def synthetic_a2_pose_scene():
    try:
        cfg = load_config()
    except ModuleNotFoundError:
        pytest.skip("pyyaml not installed")
    from pathlib import Path

    a2_path = Path(__file__).resolve().parents[1] / "configs" / "board_config_a2.yaml"
    cfg = load_config(a2_path)
    result = _render(cfg)
    if not result.ok:
        pytest.fail(f"board render failed: status={result.status} error={result.error}")
    return cfg, result.value


def test_pose_and_reprojection_on_synthetic_fronto_parallel_board(synthetic_a2_pose_scene):
    cfg, img = synthetic_a2_pose_scene
    camera_matrix = _synthetic_pinhole_for_orthographic_render(TEST_DPI)
    dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]

    result = run_isolated(
        "camera_calibration.detector",
        "detect_pose_and_annotate_in_subprocess",
        args=(img, cfg.to_dict(), camera_matrix.tolist(), dist_coeffs),
        kwargs={"min_markers": 1, "min_corners": 6},
        timeout=30.0,
    )
    assert result.ok, f"status={result.status} error={result.error}"
    value = result.value

    assert value["gate_pass"] is True
    assert value["pose_ok"] is True, "expected a solvable pose for a well-lit synthetic frame"

    rvec = np.array(value["rvec"]).reshape(3)
    tvec = np.array(value["tvec"]).reshape(3)

    # R should be close to identity (board fronto-parallel to camera): a
    # near-zero rotation vector directly implies R~=I.
    assert np.linalg.norm(rvec) < 0.05, f"expected ~0 rotation, got rvec={rvec}"
    # t should be close to [0, 0, 1] (1m along the synthetic camera's Z axis).
    assert tvec == pytest.approx([0.0, 0.0, 1.0], abs=0.02)

    reproj = value["reprojection_error_px"]
    assert reproj["rms"] < 1.0, f"expected sub-pixel reprojection error, got {reproj}"


def test_gate_rejects_when_thresholds_too_high(synthetic_a2_pose_scene):
    cfg, img = synthetic_a2_pose_scene
    camera_matrix = _synthetic_pinhole_for_orthographic_render(TEST_DPI)
    dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]

    result = run_isolated(
        "camera_calibration.detector",
        "detect_pose_and_annotate_in_subprocess",
        args=(img, cfg.to_dict(), camera_matrix.tolist(), dist_coeffs),
        kwargs={"min_markers": 10_000, "min_corners": 10_000},
        timeout=30.0,
    )
    assert result.ok, f"status={result.status} error={result.error}"
    value = result.value
    assert value["gate_pass"] is False
    assert value["pose_ok"] is False
    assert "rvec" not in value
