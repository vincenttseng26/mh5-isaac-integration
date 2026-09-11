#!/usr/bin/env python3
"""Preview or execute one guarded RG2-FT action; never chains arm motion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import time

TOKEN = "MH5_LOCAL_TEST"
DEFAULT_REPORT = ("/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/"
                  "zed_object_detection_latest.json")
DEFAULT_PROFILE = ("/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/"
                   "object_profiles/blue_tape_measure.yaml")


def opening_from_report(report, max_age_s, now=None):
    now = time.time() if now is None else now
    age = now - float(report.get("timestamp_unix", 0.0))
    if age < -1.0 or age > max_age_s:
        raise ValueError(f"detection report age {age:.1f} s is outside 0..{max_age_s:.1f} s")
    stages = report.get("grasp_stages") or {}
    if not stages.get("valid"):
        raise ValueError("detection report has no valid grasp staging")
    opening_m = float(stages.get("gripper_open_m"))
    width = round(opening_m * 10000.0)  # m -> 0.1 mm
    if not 0 <= width <= 1000:
        raise ValueError(f"computed opening {opening_m * 1000:.1f} mm is outside RG2-FT stroke")
    return width, age


def exchange(host, port, records, timeout=4.0):
    replies = []
    with socket.create_connection((host, port), timeout=timeout) as connection:
        stream = connection.makefile("r")
        for record in records:
            connection.sendall((json.dumps(record, separators=(",", ":")) + "\n").encode())
            line = stream.readline()
            if not line:
                raise ConnectionError("gripper gateway closed the connection")
            replies.append(json.loads(line))
    return replies


def one_shot_move(host, port, width, force):
    with socket.create_connection((host, port), timeout=4.0) as connection:
        stream = connection.makefile("r")

        def send(record):
            connection.sendall((json.dumps(record, separators=(",", ":")) + "\n").encode())
            line = stream.readline()
            if not line:
                raise ConnectionError("gripper gateway closed the connection")
            return json.loads(line)

        mode = send({"cmd": "set_mode", "mode": "COMMAND", "token": TOKEN})
        if not mode.get("ok"):
            return [mode]
        armed = send({"cmd": "arm", "token": TOKEN})
        if not armed.get("ok"):
            return [mode, armed]
        moved = send({"cmd": "move", "token": TOKEN, "nonce": armed.get("nonce"),
                      "width_tenth_mm": width, "force_tenth_n": force})
        return [mode, armed, moved]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True,
                        choices=("status", "open", "prepare", "close", "verify", "stop"))
    parser.add_argument("--gateway", default="192.168.50.10")
    parser.add_argument("--port", type=int, default=8768)
    parser.add_argument("--report", default=DEFAULT_REPORT)
    parser.add_argument("--profile", default=DEFAULT_PROFILE)
    parser.add_argument("--max-report-age-s", type=float, default=300.0)
    parser.add_argument("--force-tenth-n", type=int, default=60,
                        help="closing/opening force in 0.1 N; allowed 30..120")
    parser.add_argument("--confirm", action="store_true")
    args = parser.parse_args()

    if args.stage in ("status", "verify"):
        reply = exchange(args.gateway, args.port, [{"cmd": "status"}])[0]
        print(json.dumps(reply, ensure_ascii=False, indent=2, sort_keys=True))
        if args.stage == "verify":
            state = reply.get("gripper") or {}
            if state.get("busy"):
                print("verification: FAIL (gripper is still moving)")
                return 2
            if not state.get("grip_detected"):
                print("verification: FAIL (RG2-FT reports no detected grip)")
                return 2
            print("verification: PASS (stationary and grip_detected=true)")
        return 0 if reply.get("ok") else 1

    if args.stage == "stop":
        reply = exchange(args.gateway, args.port, [{"cmd": "stop"}])[0]
        print(json.dumps(reply, ensure_ascii=False, sort_keys=True))
        return 0 if reply.get("ok") else 1

    if not 30 <= args.force_tenth_n <= 120:
        raise SystemExit("--force-tenth-n must be in 30..120 (3.0..12.0 N)")

    width, age = (1000 if args.stage == "open" else 0), None
    if args.stage == "prepare":
        import yaml
        profile = yaml.safe_load(Path(args.profile).read_text())
        if not profile.get("robot_motion_allowed", False):
            raise SystemExit("object profile has robot_motion_allowed: false")
        report = json.loads(Path(args.report).read_text())
        try:
            width, age = opening_from_report(report, args.max_report_age_s)
        except (TypeError, ValueError) as exc:
            raise SystemExit(str(exc))

    print(f"stage       : {args.stage}")
    print(f"target width: {width / 10:.1f} mm")
    print(f"force       : {args.force_tenth_n / 10:.1f} N")
    if age is not None:
        print(f"report age  : {age:.1f} s")
    if not args.confirm:
        print("\npreview only; no gateway command sent. Re-run with --confirm to move the gripper.")
        return 0

    replies = one_shot_move(args.gateway, args.port, width, args.force_tenth_n)
    print(json.dumps(replies, ensure_ascii=False, indent=2, sort_keys=True))
    if len(replies) != 3 or not replies[-1].get("ok"):
        return 1
    print("\nOne gripper motion was accepted; gateway returned to MIRROR/DISARMED.")
    print("Wait for motion to finish, then run --stage status or --stage verify.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ConnectionError, OSError, json.JSONDecodeError) as exc:
        print(f"gripper gateway unavailable: {exc}", file=sys.stderr)
        sys.exit(2)
