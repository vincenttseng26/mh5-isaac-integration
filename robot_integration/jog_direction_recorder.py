#!/usr/bin/env python3
"""Read-only joint sign/order/unit verification recorder.

Prerequisite for checklist section 9 item "compare shadow prediction against
actual teach-pendant motion; confirm joint sign, order and units". Subscribes to
/real/joint_states only. Creates no publisher, service client or command path.

Procedure: run this, then jog ONE axis at a time with the teach pendant while
watching the Isaac view. The report says which joint index moved and in which
direction, so it can be compared against the pendant axis label and the observed
physical and on-screen motion.
"""
import argparse
import json
import math
import sys
import time

ARM_JOINTS = ("joint_1_s", "joint_2_l", "joint_3_u",
              "joint_4_r", "joint_5_b", "joint_6_t")

# Teach-pendant axis letter for each joint, in canonical order.
PENDANT_AXES = ("S", "L", "U", "R", "B", "T")

MOVE_THRESHOLD_RAD = 0.005


def record(topic, duration, node=None):
    """Collect (timestamp_ns, six joint positions) samples. Read-only."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    owns_context = node is None
    if owns_context:
        rclpy.init()
        node = Node("jog_direction_recorder")

    samples = []

    def callback(msg):
        index = {name: i for i, name in enumerate(msg.name)}
        if any(n not in index or index[n] >= len(msg.position) for n in ARM_JOINTS):
            return
        values = [float(msg.position[index[n]]) for n in ARM_JOINTS]
        if all(math.isfinite(v) for v in values):
            samples.append((time.time_ns(), values))

    subscription = node.create_subscription(JointState, topic, callback, 10)
    start = time.time()
    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)

    if owns_context:
        node.destroy_node()
        rclpy.shutdown()
    return samples


def analyse(samples, label, topic, expected_joint=None):
    """Turn recorded samples into a per-joint direction report."""
    if len(samples) < 2:
        return {"error": "insufficient samples", "samples": len(samples),
                "pendant_axis_label": label}

    first = samples[0][1]
    last = samples[-1][1]
    report = {
        "pendant_axis_label": label,
        "topic": topic,
        "samples": len(samples),
        "duration_s": round((samples[-1][0] - samples[0][0]) / 1e9, 3),
        "expected_joint": expected_joint,
        "joints": [],
    }
    moved = []
    for i, name in enumerate(ARM_JOINTS):
        series = [s[1][i] for s in samples]
        delta = last[i] - first[i]
        entry = {
            "index": i,
            "name": name,
            "pendant_axis": PENDANT_AXES[i],
            "start_rad": round(first[i], 6),
            "end_rad": round(last[i], 6),
            "delta_rad": round(delta, 6),
            "delta_deg": round(math.degrees(delta), 4),
            "min_rad": round(min(series), 6),
            "max_rad": round(max(series), 6),
            "span_rad": round(max(series) - min(series), 6),
            "moved": abs(delta) > MOVE_THRESHOLD_RAD,
            "direction": "positive" if delta > MOVE_THRESHOLD_RAD
                         else "negative" if delta < -MOVE_THRESHOLD_RAD else "static",
        }
        report["joints"].append(entry)
        if entry["moved"]:
            moved.append(entry)

    report["moved_joint_names"] = [e["name"] for e in moved]
    report["single_axis_isolated"] = len(moved) == 1
    report["operator_must_confirm"] = [
        "Pendant axis label matches the joint name reported as moved.",
        "Isaac on-screen motion direction matches the observed physical motion.",
        "Magnitude in degrees matches the pendant position readout change.",
    ]

    if not moved:
        report["verdict"] = "no joint moved; nothing verified"
        report["axis_matches_expectation"] = False
    elif len(moved) > 1:
        report["verdict"] = ("more than one joint moved; rerun jogging a single axis "
                             "so sign and order can be attributed unambiguously")
        report["axis_matches_expectation"] = False
    else:
        entry = moved[0]
        report["verdict"] = (f"{entry['name']} moved {entry['delta_deg']:+.3f} deg "
                             f"({entry['direction']}) for pendant axis '{label}'")
        if expected_joint is not None:
            report["axis_matches_expectation"] = entry["name"] == expected_joint
            if not report["axis_matches_expectation"]:
                report["verdict"] += (f"; EXPECTED {expected_joint}, so the pendant axis "
                                      f"to joint mapping does not hold")
        else:
            report["axis_matches_expectation"] = None
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/real/joint_states")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--label", default="unlabelled",
                    help="Pendant axis being jogged, e.g. S+ or L-")
    ap.add_argument("--expect-joint", default=None,
                    help="Joint name expected to move, e.g. joint_1_s")
    ap.add_argument("--json-out")
    args = ap.parse_args()

    print(f"[READ-ONLY] subscribing {args.topic}; no publisher or command path")
    print(f"[ACTION] jog pendant axis '{args.label}' now; recording {args.duration:g}s")
    samples = record(args.topic, args.duration)
    report = analyse(samples, args.label, args.topic, args.expect_joint)

    text = json.dumps(report, indent=2)
    print(text)
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
    return 0 if "error" not in report else 1


if __name__ == "__main__":
    sys.exit(main())
