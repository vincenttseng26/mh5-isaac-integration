"""Generate a ChArUco board image locked to real-world physical size.

Two outputs:
  - PNG: raw raster of the board only, useful for on-screen detection tests.
  - PDF: the board plus a printed ruler and instructions, sized so that
    printing at 100% / "Actual Size" (never "Fit to Page") reproduces the
    exact square_length_m / marker_length_m from the config.

The board's own pixel size is derived directly from squares_x/squares_y *
square_length_m at the requested DPI, with marginSize=0 passed to OpenCV's
renderer — that guarantees the raster's physical size is exactly
board_width_m x board_height_m with no OpenCV-side margin distorting it.
Any extra whitespace/margin for printing is added by us afterwards, outside
of that exact-scale region.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from . import compat
from .config import BoardConfig

METERS_PER_INCH = 0.0254
MM_PER_METER = 1000.0


def render_board_png(cfg: BoardConfig, dpi: int = 600, border_bits: int = 1) -> np.ndarray:
    """Render the board at exactly board_width_m x board_height_m, at dpi."""
    cfg.validate()
    px_per_m = dpi / METERS_PER_INCH
    width_px = round(cfg.board_width_m * px_per_m)
    height_px = round(cfg.board_height_m * px_per_m)

    dictionary = compat.get_dictionary(cfg.dictionary)
    board = compat.make_charuco_board(cfg, dictionary)
    img = compat.draw_board(board, (width_px, height_px), margin_size=0, border_bits=border_bits)
    return img


def save_png(img: np.ndarray, path: Path | str) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), img):
        raise IOError(f"cv2.imwrite failed for {path}")
    return path


def save_pdf_locked(
    img: np.ndarray,
    cfg: BoardConfig,
    path: Path | str,
    margin_mm: float = 15.0,
    ruler_length_mm: float = 50.0,
) -> Path:
    """Save a PDF sized exactly board size + margin, at 100% print scale.

    Uses matplotlib because its PDF backend places raster images by
    absolute physical extent (inches/points), independent of screen DPI —
    unlike simply exporting a PNG into a page layout tool, which invites
    accidental rescaling.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise ImportError(
            "matplotlib is required for physically-locked PDF export. "
            "Install with: pip install matplotlib"
        ) from exc

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    board_w_mm = cfg.board_width_m * MM_PER_METER
    board_h_mm = cfg.board_height_m * MM_PER_METER
    footer_mm = 30.0  # extra space at the bottom for text + ruler

    page_w_mm = board_w_mm + 2 * margin_mm
    page_h_mm = board_h_mm + 2 * margin_mm + footer_mm

    page_w_in = page_w_mm / 25.4
    page_h_in = page_h_mm / 25.4

    fig = plt.figure(figsize=(page_w_in, page_h_in))

    # Position the board axes in figure-fraction coordinates so it lands at
    # an exact physical offset from the page edge.
    board_rect = [
        margin_mm / page_w_mm,
        (margin_mm + footer_mm) / page_h_mm,
        board_w_mm / page_w_mm,
        board_h_mm / page_h_mm,
    ]
    ax = fig.add_axes(board_rect)
    ax.imshow(img, cmap="gray", extent=[0, board_w_mm, 0, board_h_mm], interpolation="nearest")
    ax.set_xlim(0, board_w_mm)
    ax.set_ylim(0, board_h_mm)
    ax.set_aspect("equal")
    ax.axis("off")

    # Footer: ruler + spec text, entirely below the board so it never
    # overlaps the calibration pattern.
    footer_ax = fig.add_axes([0, 0, 1, footer_mm / page_h_mm])
    footer_ax.set_xlim(0, page_w_mm)
    footer_ax.set_ylim(0, footer_mm)
    footer_ax.axis("off")

    # Clamp so tiny custom boards (narrow page) don't push the ruler past
    # the page edge, where it would get clipped and be unmeasurable.
    ruler_length_mm = min(ruler_length_mm, max(page_w_mm - 2 * margin_mm, 10.0))
    ruler_x0 = margin_mm
    ruler_y = footer_mm * 0.35
    ruler_x1 = ruler_x0 + ruler_length_mm
    footer_ax.plot([ruler_x0, ruler_x1], [ruler_y, ruler_y], color="black", linewidth=1.5)
    tick_h = footer_mm * 0.08
    for x in (ruler_x0, ruler_x1):
        footer_ax.plot([x, x], [ruler_y - tick_h, ruler_y + tick_h], color="black", linewidth=1.5)
    footer_ax.text(
        (ruler_x0 + ruler_x1) / 2,
        ruler_y + tick_h * 1.8,
        f"Measure this line with a ruler after printing — must be exactly {ruler_length_mm:.2f} mm.\n"
        "If it isn't, your printer/viewer rescaled the page: reprint at 100% / Actual Size (not 'Fit to Page').",
        ha="center",
        va="bottom",
        fontsize=6,
    )

    spec_text = (
        f"{cfg.name}  |  {cfg.squares_x}x{cfg.squares_y} squares  |  "
        f"square={cfg.square_length_m * 1000:.2f} mm  marker={cfg.marker_length_m * 1000:.2f} mm  |  "
        f"{cfg.dictionary}  |  legacy_pattern={cfg.legacy_pattern}"
    )
    footer_ax.text(
        page_w_mm / 2,
        footer_mm * 0.92,
        spec_text,
        ha="center",
        va="top",
        fontsize=6,
    )

    fig.savefig(str(path), format="pdf")
    plt.close(fig)
    return path


def generate(
    cfg: BoardConfig,
    out_dir: Path | str,
    dpi: int = 600,
    make_pdf: bool = True,
) -> dict:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    img = render_board_png(cfg, dpi=dpi)

    png_path = save_png(img, out_dir / f"{cfg.name}.png")
    result = {"png": png_path}

    if make_pdf:
        pdf_path = save_pdf_locked(img, cfg, out_dir / f"{cfg.name}.pdf")
        result["pdf"] = pdf_path

    return result
