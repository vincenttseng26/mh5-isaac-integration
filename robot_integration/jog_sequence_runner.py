#!/usr/bin/env python3
"""Guided six-axis jog verification. Read-only.

Walks the operator through jogging S, L, U, R, B and T one at a time, records
/real/joint_states for each, and writes a combined report saying whether each
pendant axis moved the joint it is supposed to move and in which direction.

Creates no publisher, no service client and no command path. It cannot move the
robot; the operator moves it with the teach pendant.

Run this next to the Isaac view so the on-screen direction can be compared with
the physical direction at the same time.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_integration.jog_direction_recorder import (  # noqa: E402
    ARM_JOINTS,
    PENDANT_AXES,
    analyse,
    record,
)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/real/joint_states")
    ap.add_argument("--duration", type=float, default=20.0,
                    help="Recording window per axis, seconds")
    ap.add_argument("--direction", choices=["plus", "minus"], default="plus",
                    help="Which pendant direction key to press for every axis")
    ap.add_argument("--axes", default="S,L,U,R,B,T",
                    help="Comma separated subset of axes to run")
    ap.add_argument("--json-out", default="jog_verification.json")
    args = ap.parse_args()

    sign = "+" if args.direction == "plus" else "-"
    wanted = [a.strip().upper() for a in args.axes.split(",") if a.strip()]
    unknown = [a for a in wanted if a not in PENDANT_AXES]
    if unknown:
        print(f"[FAIL] unknown axis letters: {', '.join(unknown)}", file=sys.stderr)
        return 2

    print("=" * 68)
    print("READ-ONLY six-axis jog verification")
    print(f"subscribing {args.topic}; no publisher, no command path")
    print("This program cannot move the robot. You move it with the pendant.")
    print("=" * 68)
    print()
    print("Before starting, confirm on the pendant:")
    print("  1. Key switch is in TEACH mode, not PLAY and not REMOTE.")
    print("  2. Coordinate system is set to JOINT, so one key moves one axis.")
    print("  3. Manual speed is set to the slowest setting.")
    print("  4. The work area is clear and you can reach an E-stop.")
    print()
    try:
        input("Press Enter when the above is confirmed, or Ctrl-C to abort: ")
    except (EOFError, KeyboardInterrupt):
        print("\naborted")
        return 1

    results = []
    for axis in wanted:
        index = PENDANT_AXES.index(axis)
        expected = ARM_JOINTS[index]
        label = f"{axis}{sign}"
        print()
        print("-" * 68)
        print(f"Axis {axis}  ->  expected joint {expected}")
        print(f"Hold the enable switch, then press and hold the [{label}] key")
        print(f"for most of the next {args.duration:g} seconds. Move a visible")
        print("amount, roughly ten degrees. Watch the Isaac view at the same time.")
        try:
            input(f"Press Enter to start recording axis {axis}: ")
        except (EOFError, KeyboardInterrupt):
            print("\naborted")
            return 1

        print(f"[RECORDING] jog {label} now ...")
        samples = record(args.topic, args.duration)
        report = analyse(samples, label, args.topic, expected)
        results.append(report)

        if "error" in report:
            print(f"  [FAIL] {report['error']}")
        else:
            print(f"  {report['verdict']}")
            if report.get("axis_matches_expectation") is False:
                print("  [ATTENTION] this axis did not behave as expected")

        try:
            seen = input("  Did Isaac move the same way you saw the arm move? [y/n/skip]: ")
        except (EOFError, KeyboardInterrupt):
            seen = "skip"
        report["operator_says_isaac_matches_physical"] = seen.strip().lower()[:1] or "s"

    ok = [r for r in results
          if "error" not in r and r.get("axis_matches_expectation") is True
          and r.get("operator_says_isaac_matches_physical") == "y"]
    summary = {
        "topic": args.topic,
        "direction_key": sign,
        "axes_attempted": wanted,
        "axes_fully_confirmed": [r["pendant_axis_label"] for r in ok],
        "all_axes_confirmed": len(ok) == len(PENDANT_AXES) and len(wanted) == len(PENDANT_AXES),
        "per_axis": results,
    }
    summary["conclusion"] = (
        "joint sign, order and units confirmed for all six axes"
        if summary["all_axes_confirmed"] else
        "NOT confirmed; do not set joint_sign_order_units_verified to true"
    )

    with open(args.json_out, "w", encoding="utf-8") as handle:
        handle.write(json.dumps(summary, indent=2) + "\n")

    print()
    print("=" * 68)
    print(summary["conclusion"])
    print(f"written to {args.json_out}")
    print("=" * 68)
    return 0 if summary["all_axes_confirmed"] else 1


if __name__ == "__main__":
    sys.exit(main())
