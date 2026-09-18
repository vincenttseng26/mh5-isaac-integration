#!/usr/bin/env python3
"""Generate a ChArUco board PNG + physically-locked PDF.

Examples:
    python generate_boards.py
    python generate_boards.py --config ../configs/board_config.yaml --dpi 600
    python generate_boards.py --squares-x 5 --squares-y 7 --square-length-mm 30 --marker-length-mm 22.5
"""
import argparse
from pathlib import Path

import _bootstrap  # noqa: F401

from camera_calibration.board_generator import generate
from camera_calibration.config import DEFAULT_CONFIG_PATH, load_config


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    p.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parents[1] / "calib_data" / "boards")
    p.add_argument("--dpi", type=int, default=600)
    p.add_argument("--no-pdf", action="store_true", help="skip the print-locked PDF, PNG only")

    # Optional per-field overrides on top of the config file.
    p.add_argument("--squares-x", type=int)
    p.add_argument("--squares-y", type=int)
    p.add_argument("--square-length-mm", type=float)
    p.add_argument("--marker-length-mm", type=float)
    p.add_argument("--dictionary", type=str)
    p.add_argument("--name", type=str)
    return p.parse_args()


def main():
    args = parse_args()
    cfg = load_config(args.config)

    if args.squares_x is not None:
        cfg.squares_x = args.squares_x
    if args.squares_y is not None:
        cfg.squares_y = args.squares_y
    if args.square_length_mm is not None:
        cfg.square_length_m = args.square_length_mm / 1000.0
    if args.marker_length_mm is not None:
        cfg.marker_length_m = args.marker_length_mm / 1000.0
    if args.dictionary is not None:
        cfg.dictionary = args.dictionary
    if args.name is not None:
        cfg.name = args.name
    cfg.validate()

    result = generate(cfg, args.out_dir, dpi=args.dpi, make_pdf=not args.no_pdf)

    print(f"Board: {cfg.name}  {cfg.squares_x}x{cfg.squares_y}  "
          f"square={cfg.square_length_m * 1000:.2f}mm  marker={cfg.marker_length_m * 1000:.2f}mm  "
          f"dict={cfg.dictionary}  legacy_pattern={cfg.legacy_pattern}")
    for kind, path in result.items():
        print(f"  {kind}: {path}")
    if "pdf" in result:
        print("\nIMPORTANT: print the PDF at 100% / Actual Size. Verify the printed "
              "ruler line at the bottom measures exactly 50.00 mm before using the "
              "board for calibration.")


if __name__ == "__main__":
    main()
