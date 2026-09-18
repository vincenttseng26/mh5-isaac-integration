#!/usr/bin/env python3
"""Run a full pick-and-place cycle for one or more detected objects.

This sequences the stages that run_staged_grasp.py performs one at a time. It
changes the safety posture in exactly one way -- the operator confirms the whole
cycle once instead of each stage -- and changes nothing else. Every underlying
gate still applies to every stage: MoveIt collision checking, the staged
envelope's displacement/step/velocity bounds, the one-shot arm nonce, the
automatic disarm after each publish, and the gripper interlock.

Three lessons from the 2026-09-11 runs are built in:

  * Completion is confirmed from the audit trail, never from a status flag.
    `in_motion` still reads False in the window between accepting a trajectory
    and starting it, so polling it reports the previous pose as if the move had
    finished. The gripper's `busy` behaves the same way.
  * Holding is judged from finger width, not `grip_detected`. That flag goes
    back to False once the arm moves while the object is still firmly held.
  * The gripper travels FULLY OPEN and closes only at the grasp point. It is
    never pre-set to a width derived from the detection: that width comes from
    a sparse top-surface footprint and underestimates the object -- a cluster
    measured 34 mm across closed on the real object at 41.1 mm. Approaching
    with fingers sized to the estimate risks clipping the object and pushing it
    out of the way, which is indistinguishable from an empty grasp afterwards.

A pre-flight plans every stage of every object before anything moves, so an
unreachable pose is found with the arm still parked rather than mid-cycle.
"""

from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
STAGED = str(HERE / "run_staged_grasp.py")
ARM = ("192.168.50.10", 8769)
GRIPPER = ("192.168.50.10", 8768)
TOKEN = "MH5_LOCAL_TEST"
# pre_grasp hovers directly above the object, so the final approach is a
# straight vertical descent between the open fingers.
#
# Going straight to the grasp point was tried on 2026-09-11 and struck the
# object. MoveIt's path is collision-checked only against the FIXED workcell --
# the detected object is not in the planning scene, so the planner has no reason
# to avoid it and came in about 32 degrees off vertical, sweeping the gripper
# through the object's space on the way down.
#
# A geometric check run beforehand claimed 16 mm of clearance and was wrong: it
# modelled each finger as a single point at the TCP plus/minus half the opening,
# ignoring the fingers' own length and thickness and the knuckle linkage. Treat
# that style of check as necessary but not sufficient.
CYCLE = ("pre_grasp", "grasp", "lift", "transport", "place")


def arm_request(record, timeout=6.0):
    with socket.create_connection(ARM, timeout=timeout) as connection:
        connection.sendall((json.dumps(record) + "\n").encode())
        return json.loads(connection.makefile("r").readline())


def gripper_request(records, timeout=12.0, retries=3):
    for attempt in range(retries):
        try:
            with socket.create_connection(GRIPPER, timeout=timeout) as connection:
                stream = connection.makefile("r")
                replies = []
                for record in records:
                    connection.sendall((json.dumps(record) + "\n").encode())
                    replies.append(json.loads(stream.readline()))
                return replies
        except (OSError, ValueError):
            # Modbus reads slow down while the fingers move; retry rather than
            # treat a slow status read as a failure.
            if attempt == retries - 1:
                raise
            time.sleep(2.0)
    return []


def gripper_state():
    return gripper_request([{"cmd": "status"}])[0]["gripper"]


def gripper_move(width_tenth_mm, force_tenth_n=30):
    replies = gripper_request([
        {"cmd": "set_mode", "mode": "COMMAND", "token": TOKEN},
        {"cmd": "arm", "token": TOKEN},
    ])
    nonce = replies[-1].get("nonce")
    if not nonce:
        raise RuntimeError("gripper refused to arm: " + json.dumps(replies))
    # set_mode, arm and move must share one connection; the gateway disarms on
    # disconnect. gripper_request opens one connection per call, so redo them.
    with socket.create_connection(GRIPPER, timeout=12.0) as connection:
        stream = connection.makefile("r")

        def send(record):
            connection.sendall((json.dumps(record) + "\n").encode())
            return json.loads(stream.readline())

        send({"cmd": "set_mode", "mode": "COMMAND", "token": TOKEN})
        armed = send({"cmd": "arm", "token": TOKEN})
        if not armed.get("nonce"):
            raise RuntimeError("gripper refused to arm: " + json.dumps(armed))
        return send({"cmd": "move", "token": TOKEN, "nonce": armed["nonce"],
                     "width_tenth_mm": int(width_tenth_mm),
                     "force_tenth_n": int(force_tenth_n)})


def gripper_wait(target_tenth_mm=None, timeout=25.0):
    """Wait for the width to converge. `busy` is not a completion signal."""
    deadline = time.time() + timeout
    previous, stable, state = None, 0, gripper_state()
    while time.time() < deadline:
        time.sleep(0.8)
        try:
            state = gripper_state()
        except (OSError, ValueError):
            continue
        width = int(state.get("width_tenth_mm", -1))
        if target_tenth_mm is not None and abs(width - target_tenth_mm) <= 30:
            return state
        if previous is not None and abs(width - previous) <= 2:
            stable += 1
            if stable >= 3:
                return state
        else:
            stable = 0
        previous = width
    return state


def last_completion():
    reply = arm_request({"cmd": "audit", "token": TOKEN}, timeout=10.0)
    for event in reversed(reply.get("events", [])):
        if event.get("event") == "trajectory_completed":
            return event
    return None


def run_stage(stage, index, extra, confirm):
    """Publish one stage and wait for the audit to show it completed."""
    before = last_completion()
    before_id = (before or {}).get("command_id")
    command = [sys.executable, STAGED, "--stage", stage,
               "--object-index", str(index)] + extra
    if confirm:
        command.append("--confirm")
    result = subprocess.run(command, capture_output=True, text=True, timeout=180)
    if result.returncode != 0:
        raise RuntimeError(f"{stage} failed:\n{result.stdout}\n{result.stderr}")
    for line in result.stdout.splitlines():
        if line.startswith(("target", "worst axis", "drop point", "heights",
                            "holding", "object")):
            print("    " + line.rstrip())
    if not confirm:
        return None

    deadline = time.time() + 90.0
    while time.time() < deadline:
        time.sleep(1.5)
        event = last_completion()
        if event and event.get("command_id") != before_id:
            print(f"    completed {event.get('stage')} "
                  f"max_error_rad={event.get('max_error_rad'):.2e}")
            return event
    raise RuntimeError(f"{stage} published but no completion appeared in the audit")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--objects", nargs="+", type=int, default=[0, 1],
                        help="detection indices to pick, in order")
    parser.add_argument("--report",
                        default="/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/"
                                "zed_object_detection_latest.json")
    parser.add_argument("--max-report-age-s", type=float, default=900.0)
    parser.add_argument("--close-target-mm", type=float, default=20.0)
    parser.add_argument("--stage-gap-s", type=float, default=4.0,
                        help="pause after a stage completes before publishing "
                             "the next. Measured 2026-09-11: a trajectory "
                             "published about 2 s after the previous one "
                             "finished was accepted by the gateway "
                             "(published=true) but silently never executed by "
                             "the controller. The audit check catches that, but "
                             "the gap avoids it.")
    parser.add_argument("--travel-width-mm", type=float, default=100.0,
                        help="the gripper stays this wide for every move; it "
                             "only ever closes at the grasp point")
    parser.add_argument("--release-width-mm", type=float, default=100.0)
    parser.add_argument("--hold-margin-mm", type=float, default=5.0)
    parser.add_argument("--max-over-detected-mm", type=float, default=15.0,
                        help="how far above the detected width the fingers may "
                             "stop and still count as a grasp. Successful "
                             "grasps overshot by 7-8 mm; an empty one stopped "
                             "25.8 mm over.")
    parser.add_argument("--home-after", action="store_true", default=True)
    parser.add_argument("--confirm", action="store_true",
                        help="required; without it the whole cycle is planned "
                             "and printed but nothing moves")
    args = parser.parse_args()

    report = json.loads(Path(args.report).read_text())
    extra = ["--max-report-age-s", str(args.max_report_age_s),
             "--close-target-mm", str(args.close_target_mm)]

    print(f"objects      : {args.objects} of {report.get('object_count')} detected")
    print(f"report age   : {time.time() - float(report['timestamp_unix']):.0f} s\n")

    # Pre-flight: plan every stage before anything moves.
    print("=== pre-flight (planning only) ===")
    for index in args.objects:
        print(f"  object {index}:")
        for stage in CYCLE:
            run_stage(stage, index, extra, confirm=False)
    print("\npre-flight passed: every stage planned and within the envelope.")

    if not args.confirm:
        print("\nplanned only. Re-run with --confirm to run the cycle.")
        return 0

    for index in args.objects:
        opening = (report["detections"][index]["width_across_fingers_m"]
                   + 0.012)
        print(f"\n=== object {index}: pick and place ===")
        # Travel wide. The gripper is only ever narrow while it holds something.
        print(f"  -- open gripper fully to {args.travel_width_mm:.0f} mm "
              "before approaching --")
        gripper_move(round(args.travel_width_mm * 10))
        state = gripper_wait(round(args.travel_width_mm * 10))
        print(f"    width {state['width_mm']:.1f} mm "
              f"(object needs at least {opening * 1000:.1f} mm)")
        # The opening is never sized to the detection: that width comes from a
        # sparse top-surface footprint and underestimates badly -- 24 mm
        # detected against 49.8 mm measured at the fingers on 2026-09-11.
        if state["width_mm"] < opening * 1000.0:
            raise SystemExit(
                f"gripper only opened to {state['width_mm']:.1f} mm; object "
                f"{index} needs {opening * 1000:.1f} mm. Nothing has moved.")
        for stage in CYCLE:
            print(f"  -- {stage} --")
            run_stage(stage, index, extra, confirm=True)
            time.sleep(args.stage_gap_s)
            if stage == "grasp":
                print("  -- close gripper --")
                gripper_move(round(args.close_target_mm * 10))
                state = gripper_wait()
                width = state["width_mm"]
                detected = report["detections"][index]["width_across_fingers_m"] * 1000.0
                # Too narrow means the fingers met nothing. Too WIDE means they
                # stopped on something that is not this object: the detected
                # footprint underestimates the real width, but only by about
                # 7-8 mm in the grasps that succeeded, whereas an empty close
                # on 2026-09-11 stopped 25.8 mm above the detected width.
                too_narrow = width <= args.close_target_mm + args.hold_margin_mm
                too_wide = width > detected + args.max_over_detected_mm
                print(f"    width {width:.1f} mm (detected {detected:.1f} mm, "
                      f"close target {args.close_target_mm:.1f} mm)")
                if too_narrow or too_wide:
                    reason = ("the fingers closed to the target, so they met "
                              "nothing" if too_narrow else
                              f"the fingers stopped {width - detected:.1f} mm "
                              f"above the detected width, over the "
                              f"{args.max_over_detected_mm:.1f} mm allowance, so "
                              "they are resting on something that is not this "
                              "object")
                    raise SystemExit(
                        f"object {index} was not grasped: {reason}. Aborting; "
                        "the arm is parked where it is and nothing else moves.")
        print(f"  -- release to {args.release_width_mm:.0f} mm --")
        gripper_move(round(args.release_width_mm * 10))
        state = gripper_wait(round(args.release_width_mm * 10))
        print(f"    width {state['width_mm']:.1f} mm")
        if state["width_mm"] < args.release_width_mm - 10:
            raise SystemExit(
                f"the gripper did not open after placing object {index} "
                f"({state['width_mm']:.1f} mm). It is probably pressed into the "
                "platform; raise the arm before retrying the release.")

    if args.home_after:
        print("\n=== home ===")
        result = subprocess.run(
            [sys.executable, STAGED, "--stage", "home", "--confirm"],
            capture_output=True, text=True, timeout=180)
        if result.returncode != 0:
            raise RuntimeError("home failed:\n" + result.stdout + result.stderr)
        time.sleep(12)
        event = last_completion()
        print(f"  completed home max_error_rad={event.get('max_error_rad'):.2e}")

    print("\ncycle complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
