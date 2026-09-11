#!/usr/bin/env python3
"""Run ONE stage of a physical MH5 grasp, with the operator in the loop.

Each invocation performs exactly one stage. It re-reads live physical feedback,
re-plans that stage from where the robot actually is, prints the motion it is
about to make, and requires an explicit confirmation before anything is
published. It never chains stages on its own.

The virtual chain in the Isaac panel plans stage N+1 from the *planned* end of
stage N. That is fine for animation but wrong for hardware, where the robot ends
up wherever it actually ends up. Re-planning from fresh feedback each time is
the whole point of this script.

Safety rests on three independent layers, none of which this script can bypass:
  1. The MoveIt plan gateway collision-checks the start and goal against the
     fixed workcell scene and refuses to hand back an unsafe path.
  2. The CYC receiver's staged envelope bounds displacement, per-step size and
     velocity, and re-derives every timestamp itself.
  3. The receiver arms for exactly one stage, issues a nonce that this stage
     must quote, and disarms again the moment it publishes.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
from pathlib import Path

ARM_GATEWAY = ("192.168.50.10", 8769)
PLAN_GATEWAY = ("192.168.50.10", 8770)
GRIPPER_GATEWAY = ("192.168.50.10", 8768)
TOKEN = "MH5_LOCAL_TEST"
JOINTS = ("joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t")
SHORT = ("S", "L", "U", "R", "B", "T")

STAGE_KEYS = {"pre_grasp": "pre_grasp_base_link_m",
              "grasp": "grasp_base_link_m",
              "lift": "lift_base_link_m"}

# ChArUco A2 board centre in base_link, derived from the 30-frame static pose
# in the CYC validation session (mean camera-frame translation, per-axis jitter
# under 1.4 mm) mapped through the accepted optical-to-base transform including
# the operator's +25 mm x delta. The board frame's origin is its TOP-LEFT outer
# corner, so the centre is origin + (squares_x, squares_y) * 45 mm / 2.
# Cross-check: the transformed corners come out flat at z = -0.023..-0.020 and
# the centre lands at y = 0.0000, which matches the "charuco centered"
# correction that was built to centre the board.
BOARD_CENTRE_M = (0.4167, 0.0000, -0.0216)

# base_link is x forward, y left, z up, so the robot's right is -y.
DESTINATION_M = (BOARD_CENTRE_M[0],
                 BOARD_CENTRE_M[1] - 0.20,
                 BOARD_CENTRE_M[2] + 0.10)

# The pose the arm rests in between runs, recorded from the audit trail of the
# first staged move (command 5db89b50, 2026-09-10) as its physical start.
# Returning here is a joint goal, so unlike the grasp stages it needs no object
# detection and no object profile.
REST_POSE_RAD = (-0.029572362080216408, 0.06984885782003403, 0.11303521692752838,
                 -0.014476943761110306, -1.5957043170928955, 0.20770099759101868)


def arm_exchange(records, timeout=5.0):
    replies = []
    with socket.create_connection(ARM_GATEWAY, timeout=timeout) as connection:
        stream = connection.makefile("r")
        for record in records:
            connection.sendall((json.dumps(record, separators=(",", ":")) + "\n").encode())
            line = stream.readline()
            if not line:
                raise ConnectionError("arm gateway closed the connection")
            replies.append(json.loads(line))
    return replies


def arm_status():
    return arm_exchange([{"cmd": "status"}])[0]


def gripper_status(timeout=4.0):
    with socket.create_connection(GRIPPER_GATEWAY, timeout=timeout) as connection:
        connection.sendall(b'{"cmd":"status"}\n')
        line = connection.makefile("r").readline()
    if not line:
        raise ConnectionError("gripper gateway closed without a status reply")
    reply = json.loads(line)
    if not reply.get("ok"):
        raise RuntimeError("gripper status unavailable: " + str(reply.get("error")))
    return reply.get("gripper") or {}


def gripper_settled(target_tenth_mm=None, timeout=20.0, tolerance_tenth_mm=30):
    """Wait until the reported width stops changing, and return it.

    `busy` alone is NOT a completion signal: measured on 2026-09-11, the flag
    still reads False in the window between accepting a command and starting to
    move, so polling it immediately reports a stale width and makes an
    unfinished move look finished. Convergence of the width is the real signal.
    """
    import time
    deadline = time.time() + timeout
    previous = None
    stable = 0
    state = gripper_status()
    while time.time() < deadline:
        time.sleep(0.5)
        state = gripper_status()
        width = int(state.get("width_tenth_mm", -1))
        if target_tenth_mm is not None:
            if abs(width - int(target_tenth_mm)) <= tolerance_tenth_mm and not state.get("busy"):
                return state
        if previous is not None and abs(width - previous) <= 2 and not state.get("busy"):
            stable += 1
            if stable >= 2:
                return state
        else:
            stable = 0
        previous = width
    return state


def is_holding(state, close_target_mm, margin_mm=5.0):
    """Whether an object is actually between the fingers.

    `grip_detected` alone is NOT sufficient. Measured on 2026-09-11: it reads
    True right after a closing move terminates on force, then falls back to
    False once the arm moves, while the object is still firmly held. Relying on
    it would refuse a legitimate transport.

    Finger width is the dependable signal. A closing command latches its target;
    if the object were dropped the fingers would continue to that target, so a
    width that stays well above it means something is still between them. The
    lift that exposed this stopped at 41.1 mm against a 20.0 mm target.
    """
    if state.get("grip_detected"):
        return True, "grip_detected=true"
    width_mm = float(state.get("width_tenth_mm", -1)) / 10.0
    if width_mm > float(close_target_mm) + float(margin_mm):
        return True, (f"width {width_mm:.1f} mm exceeds the {close_target_mm:.1f} mm "
                      f"close target by more than {margin_mm:.1f} mm")
    return False, (f"width {width_mm:.1f} mm has closed to the "
                   f"{close_target_mm:.1f} mm target; the object is not held")


def validate_gripper_interlock(stage, state, required_opening_m=None,
                               close_target_mm=20.0):
    """Fail closed before descending, lifting or carrying an object.

    Pass a state obtained from gripper_settled(), not a bare status read: the
    `busy` check below cannot catch a command that has been accepted but has
    not started moving yet.
    """
    if state.get("busy"):
        raise ValueError("RG2-FT is still moving")
    if stage == "grasp":
        width = int(state.get("width_tenth_mm", -1))
        required = round(float(required_opening_m) * 10000.0)
        # Allow 2 mm of readback/control tolerance, but never descend with a
        # substantially narrower opening than the detected object requires.
        if width < required - 20:
            raise ValueError(
                f"RG2-FT opening is {width / 10:.1f} mm; need at least "
                f"{(required - 20) / 10:.1f} mm before descending")
    if stage in ("lift", "transport", "place"):
        holding, why = is_holding(state, close_target_mm)
        if not holding:
            raise ValueError(f"refusing {stage}: {why}")
        return why
    return None


def to_mirror():
    return arm_exchange([{"cmd": "set_mode", "mode": "MIRROR", "token": TOKEN}])[0]


def arm_staged_and_publish(stage, worst, points, timeout=10.0):
    """Select STAGED, arm, and publish one stage over a single connection.

    The gateway disarms and clears the nonce on client disconnect, so these
    three steps cannot be split across connections. The nonce is read from the
    arm reply and quoted back on the same socket.
    """
    replies = []
    with socket.create_connection(ARM_GATEWAY, timeout=timeout) as connection:
        stream = connection.makefile("r")

        def send(record):
            connection.sendall((json.dumps(record, separators=(",", ":")) + "\n").encode())
            line = stream.readline()
            if not line:
                raise ConnectionError("arm gateway closed the connection")
            reply = json.loads(line)
            replies.append(reply)
            return reply

        send({"cmd": "set_mode", "mode": "STAGED", "token": TOKEN})
        armed = send({"cmd": "arm", "token": TOKEN})
        nonce = armed.get("nonce")
        if not armed.get("ok") or not nonce:
            return replies
        send({"cmd": "staged_trajectory", "token": TOKEN, "schema": 3,
              "stage": stage, "nonce": nonce,
              "expected_total_rad": float(worst),
              "points": [{"positions": list(p)} for p in points]})
    return replies


def _plan(payload):
    with socket.create_connection(PLAN_GATEWAY, timeout=20) as connection:
        connection.sendall((json.dumps(payload) + "\n").encode())
        reply = json.loads(connection.makefile("r").readline())
    if not reply.get("ok"):
        raise RuntimeError("MoveIt refused the plan: " + str(reply.get("error")))
    return reply


def request_plan(position, yaw, start):
    return _plan({"target_pose": {"position_m": [float(v) for v in position],
                                  "yaw_rad": float(yaw)},
                  "start_rad": [float(v) for v in start]})


def request_joint_plan(target, start):
    """Plan to a joint-space goal, used for the return to the rest pose."""
    return _plan({"target_rad": [float(v) for v in target],
                  "start_rad": [float(v) for v in start]})


def describe(start, goal):
    delta = [b - a for a, b in zip(start, goal)]
    worst = max(abs(v) for v in delta)
    detail = "  ".join(f"{n}={math.degrees(v):+7.2f}" for n, v in zip(SHORT, delta))
    return delta, worst, detail


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True,
                        choices=sorted(set(STAGE_KEYS) | {"home", "transport", "place"}))
    parser.add_argument("--home-rad", nargs=6, type=float, default=list(REST_POSE_RAD),
                        help="joint goal for --stage home, in radians")
    parser.add_argument("--destination-m", nargs=3, type=float,
                        default=list(DESTINATION_M),
                        help="base_link drop point for --stage transport/place")
    parser.add_argument("--object-index", type=int, default=None,
                        help="which detection to act on, instead of the one "
                             "selected in the Isaac panel; stages are recomputed "
                             "from the profile for that object")
    parser.add_argument("--platform-height-m", type=float, default=0.10,
                        help="height of the placement platform above the "
                             "surface the object was picked from")
    parser.add_argument("--release-clearance-m", type=float, default=0.005,
                        help="extra height at place, so the object rests on the "
                             "platform without the gripper pressing into it")
    parser.add_argument("--close-target-mm", type=float, default=20.0,
                        help="width the gripper was told to close to; a held "
                             "object keeps the fingers well above it")
    parser.add_argument("--drop-spacing-m", type=float, default=0.10,
                        help="y spacing between drop points, so two objects do "
                             "not land on top of each other")
    parser.add_argument("--report",
                        default="/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/"
                                "zed_object_detection_latest.json")
    parser.add_argument("--profile",
                        default="/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/"
                                "object_profiles/blue_tape_measure.yaml")
    parser.add_argument("--max-report-age-s", type=float, default=300.0)
    parser.add_argument("--confirm", action="store_true",
                        help="required; without it the script plans and stops")
    args = parser.parse_args()

    import time

    going_home = args.stage == "home"
    target = yaw = None
    age = 0.0
    report = {}

    # Returning to the rest pose is a joint goal that does not depend on the
    # object at all, so it deliberately skips the detection report and the
    # profile's motion flag. Those guard reaching TOWARD something; they would
    # only strand the arm out over the table.
    if not going_home:
        import yaml

        report_path = Path(args.report)
        if not report_path.is_file():
            raise SystemExit("no detection report at " + str(report_path))
        report = json.loads(report_path.read_text())

        age = time.time() - float(report.get("timestamp_unix", 0.0))
        if age > args.max_report_age_s:
            raise SystemExit(f"detection report is {age:.0f} s old (limit "
                             f"{args.max_report_age_s:.0f} s); press DETECT OBJECT again")

        profile_raw = yaml.safe_load(Path(args.profile).read_text())
        if not profile_raw.get("robot_motion_allowed", False):
            raise SystemExit(
                f"profile {profile_raw.get('name')!r} still has robot_motion_allowed: "
                "false.\nVerify the detected centre against the physical object with a "
                "ruler and record the measurement in the profile before moving the arm.")

        stages = report.get("grasp_stages") or {}
        chosen_index = int(report.get("selected_index", 0))
        if args.object_index is not None:
            # Recompute staging for an explicitly chosen detection, so a
            # two-object run does not depend on someone clicking SELECT NEXT
            # OBJECT in the panel between stages.
            import numpy as np
            from grasp_pipeline import Detection, grasp_stages as compute_stages
            from grasp_pipeline import load_profile as load_grasp_profile

            found = report.get("detections") or []
            if not 0 <= args.object_index < len(found):
                raise SystemExit(f"--object-index {args.object_index} is outside "
                                 f"the {len(found)} detections in the report")
            raw = found[args.object_index]
            profile_obj, _ = load_grasp_profile(args.profile)
            detection = Detection(True, None,
                                  np.asarray(raw["centroid_base_link_m"], dtype=float),
                                  np.asarray(raw["extent_m"], dtype=float),
                                  float(raw["top_z_m"]), float(raw["yaw_rad"]),
                                  float(raw["width_across_fingers_m"]),
                                  int(raw["point_count"]),
                                  int(raw["candidate_clusters"]))
            computed = compute_stages(detection, profile_obj)
            if not computed.valid:
                raise SystemExit("grasp staging rejected for object "
                                 f"{args.object_index}: {computed.reason}")
            stages = computed.as_dict()
            chosen_index = args.object_index
            report = dict(report, selected_index=chosen_index)
        if not stages.get("valid"):
            raise SystemExit("the detection report holds no valid grasp staging")
        yaw = float(stages["yaw_rad"])
        if args.stage in STAGE_KEYS:
            target = stages[STAGE_KEYS[args.stage]]
        else:
            # transport crosses at the lift height; place descends to the height
            # the object was picked from, so an object put down on the same
            # tabletop ends up resting rather than dropped.
            destination = [float(v) for v in args.destination_m]
            # Spread the drop points along y around the destination, or the
            # second object lands on top of the first. Two objects come out at
            # -/+ half the spacing; the formula generalises to any count.
            count = max(1, int(report.get("object_count", 1)))
            chosen = int(report.get("selected_index", 0))
            offset = (chosen - (count - 1) / 2.0) * float(args.drop_spacing_m)
            drop_y = destination[1] + offset
            # The objects are picked off the tabletop but placed on a platform
            # standing above it, so both carrying heights are raised by the
            # platform height. Placing at the bare pick height drove the gripper
            # into the platform on 2026-09-11: the object bottomed out, the
            # fingers were pinned between it and the foam, and the release
            # command could not move them until the arm lifted clear.
            platform = float(args.platform_height_m)
            if args.stage == "transport":
                base_z = float(stages["lift_base_link_m"][2]) + platform
            else:
                base_z = (float(stages["grasp_base_link_m"][2]) + platform
                          + float(args.release_clearance_m))
            target = [destination[0], drop_y, base_z]
            print(f"heights      : platform +{platform * 1000:.0f} mm, "
                  f"release clearance +{float(args.release_clearance_m) * 1000:.0f} mm")
            print(f"drop point   : object {chosen + 1} of {count} goes to "
                  f"y={drop_y:+.4f} ({offset:+.3f} m from the destination centre)")

    status = arm_status()
    robot = status.get("robot") or {}
    if status.get("mode") != "MIRROR" or status.get("armed"):
        raise SystemExit(f"expected MIRROR / DISARMED, found mode="
                         f"{status.get('mode')} armed={status.get('armed')}")
    for key, want in (("e_stopped", False), ("in_error", False),
                      ("in_motion", False), ("motion_possible", True),
                      ("drives_powered", True)):
        if bool(robot.get(key)) is not want:
            raise SystemExit(f"controller reports {key}={robot.get(key)}; refusing")
    actual = tuple(status["positions"])

    if going_home:
        plan = request_joint_plan(args.home_rad, actual)
    else:
        plan = request_plan(target, yaw, actual)
    points = [tuple(float(v) for v in item["positions"]) for item in plan["points"]]
    delta, worst, detail = describe(actual, points[-1])

    print(f"stage        : {args.stage}")
    if going_home:
        print("target       : " +
              "  ".join(f"{n}={math.degrees(v):+7.2f}"
                        for n, v in zip(SHORT, args.home_rad)) + " deg")
    else:
        print(f"target       : {[round(float(v), 4) for v in target]} m, "
              f"yaw {math.degrees(yaw):+.2f} deg")
        # The report may hold several same-coloured objects. Say out loud which
        # one this stage will drive to, so a mis-selection is caught here rather
        # than by watching the arm go to the wrong place.
        count = int(report.get("object_count", 1))
        if count > 1:
            chosen = int(report.get("selected_index", 0))
            print(f"object       : {chosen + 1} of {count} detected "
                  f"(selected in the Isaac panel)")
        print(f"detection age: {age:.0f} s (report seq {report.get('zed_sequence')})")
    print(f"waypoints    : {len(points)}")
    print(f"joint move   : {detail}")
    print(f"worst axis   : {worst:.4f} rad ({math.degrees(worst):.2f} deg)")

    if not args.confirm:
        print("\nplanned only. Re-run with --confirm to move the physical arm.")
        return 0


    if args.stage in ("grasp", "lift", "transport", "place"):
        try:
            # Settle first. A bare status read can report a stale width while a
            # just-accepted gripper command has not begun moving.
            grip_state = gripper_settled()
            why = validate_gripper_interlock(
                args.stage, grip_state,
                (report.get("grasp_stages") or {}).get("gripper_open_m"),
                args.close_target_mm)
        except (ConnectionError, OSError, RuntimeError, TypeError, ValueError) as exc:
            raise SystemExit(f"gripper interlock refused {args.stage}: {exc}")
        print("gripper     : " + json.dumps(grip_state, sort_keys=True))
        if why:
            print(f"holding     : {why}")

    print("\nARMING the staged channel for exactly this one stage.")
    # Arming and publishing MUST share one connection: the gateway disarms and
    # drops the nonce whenever a client disconnects, so splitting these across
    # two connections can never publish anything.
    try:
        replies = arm_staged_and_publish(args.stage, worst, points)
    finally:
        to_mirror()

    if len(replies) < 3:
        # set_mode or arm refused, so nothing was ever published.
        raise SystemExit("could not arm the staged channel; nothing was sent: "
                         + json.dumps(replies))
    reply = replies[-1]

    print(json.dumps(reply, ensure_ascii=False, sort_keys=True))
    if not reply.get("ok"):
        return 1
    print(f"\npublished stage {reply.get('stage')} as {reply.get('command_id')}, "
          f"{reply.get('duration_s'):.1f} s. The gateway has disarmed and returned "
          f"to MIRROR.\nWatch the robot, then check completion with "
          f"send_mh5_trajectory.py --audit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
