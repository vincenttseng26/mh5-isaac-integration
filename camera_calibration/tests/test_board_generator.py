"""Board-generation smoke tests. Rendering the board itself has also been
observed to be part of the crash-prone cv2.aruco call chain on some builds,
so this goes through isolation.run_isolated too rather than calling
compat.draw_board() directly in the pytest process.
"""
import pytest

from camera_calibration.config import BoardConfig
from camera_calibration.isolation import run_isolated

TEST_DPI = 96  # low res: keep the smoke test fast


def _render(cfg: BoardConfig):
    return run_isolated(
        "camera_calibration.board_generator",
        "render_board_png",
        args=(cfg,),
        kwargs={"dpi": TEST_DPI},
        timeout=30.0,
    )


@pytest.fixture(scope="module")
def rendered_board_image():
    """Render once, isolated in a subprocess, and share the ndarray result
    across tests in this file so the risky cv2.aruco call only happens once."""
    cfg = BoardConfig()
    result = _render(cfg)
    if not result.ok:
        pytest.fail(
            f"Board rendering did not complete cleanly (status={result.status}): "
            f"{result.error}\nRun `python -m camera_calibration.diagnose` for environment details."
        )
    return cfg, result.value


def test_render_board_png_isolated(rendered_board_image):
    cfg, img = rendered_board_image
    assert img is not None
    px_per_m = TEST_DPI / 0.0254
    expected_w = round(cfg.board_width_m * px_per_m)
    expected_h = round(cfg.board_height_m * px_per_m)
    assert img.shape[1] == expected_w
    assert img.shape[0] == expected_h
    # A real board must contain both black and white pixels, not a blank canvas.
    assert img.min() < 50
    assert img.max() > 200


def test_save_pdf_locked_requires_matplotlib_or_skips(rendered_board_image, tmp_path):
    pytest.importorskip("matplotlib")
    from camera_calibration.board_generator import save_pdf_locked

    cfg, img = rendered_board_image
    out_path = tmp_path / "board.pdf"
    save_pdf_locked(img, cfg, out_path)
    assert out_path.exists()
    assert out_path.stat().st_size > 0
