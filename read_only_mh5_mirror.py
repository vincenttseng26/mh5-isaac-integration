#!/usr/bin/env python3
"""Read-only ROS 2 joint-state mirror for the MH5 Isaac Sim asset.

The process creates exactly one ROS entity: a JointState subscription. It
never publishes a ROS topic, calls a service, or sends a command to the real
robot. Joint positions are written only to the in-memory Isaac articulation.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp


ARM_JOINTS = (
    "joint_1_s",
    "joint_2_l",
    "joint_3_u",
    "joint_4_r",
    "joint_5_b",
    "joint_6_t",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd",
        default="/home/vincent/Desktop/mh5_clean_import/combined/combined.usd",
        help="Clean imported MH5 USD file.",
    )
    parser.add_argument("--shadow-actions", type=Path,
                        help="JSONL local shadow actions; never published to ROS/FS100")
    parser.add_argument("--shadow-log", type=Path,
                        help="JSONL log for actions applied only to Isaac shadow articulation")
    parser.add_argument(
        "--topic",
        default="/real/joint_states",
        help="Read-only ROS 2 JointState input topic.",
    )
    parser.add_argument("--gripper-topic", default="/real/gripper_joint_states")
    parser.add_argument("--stale-timeout", type=float, default=0.25,
                        help="Warn when either feedback topic is silent this long (seconds).")
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac GUI.")
    parser.add_argument(
        "--exit-after-first-message",
        action="store_true",
        help="Exit after one valid message; intended for a safe connection test.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="Exit with an error if no valid message arrives in this many seconds (0 disables).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    usd_path = Path(args.usd).expanduser().resolve()
    if not usd_path.is_file():
        print(f"[FAIL] USD not found: {usd_path}", file=sys.stderr, flush=True)
        return 2
    if not args.topic.startswith("/"):
        print("[FAIL] --topic must be an absolute ROS topic", file=sys.stderr, flush=True)
        return 2

    simulation_app = SimulationApp(
        {
            "headless": args.headless,
            "hide_ui": args.headless,
            "enable_livestream": False,
        }
    )

    node = None
    rclpy_started = False
    result = 1
    try:
        import numpy as np
        import omni.kit.app
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import open_stage

        extension_manager = omni.kit.app.get_app().get_extension_manager()
        bridge_id = "isaacsim.ros2.bridge"
        if not extension_manager.is_extension_enabled(bridge_id):
            extension_manager.set_extension_enabled_immediate(bridge_id, True)
            for _ in range(3):
                simulation_app.update()

        # Enabling the bridge adds Isaac's internal Jazzy Python packages to
        # sys.path. Import them only after that extension has started.
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState

        if not open_stage(str(usd_path)):
            raise RuntimeError(f"Isaac could not open {usd_path}")
        simulation_app.update()

        # Runtime additions use an anonymous session layer. Nothing in the
        # clean imported asset is saved or modified on disk.
        stage = omni.usd.get_context().get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())

        World.clear_instance()
        world = World(stage_units_in_meters=1.0, backend="numpy", device="cpu")
        world.get_physics_context().set_gravity(0.0)
        robot = world.scene.add(
            SingleArticulation(
                prim_path="/mh5_rg2ft",
                name="mh5_read_only_mirror",
                reset_xform_properties=False,
            )
        )
        world.reset()

        runtime_names = tuple(robot.dof_names)
        if runtime_names[: len(ARM_JOINTS)] != ARM_JOINTS:
            raise RuntimeError(
                "Unexpected articulation DOF order: " + ", ".join(runtime_names)
            )
        arm_indices = np.arange(len(ARM_JOINTS), dtype=np.int32)
        limits = robot.dof_properties[: len(ARM_JOINTS)]

        rclpy.init(args=None)
        rclpy_started = True
        node = rclpy.create_node("isaac_mh5_read_only_mirror")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        state = {"pending": None, "valid_count": 0, "gripper": None, "gripper_count": 0,
                 "last_arm": None, "last_gripper": None}
        shadow_actions = []
        if args.shadow_actions:
            for line in args.shadow_actions.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    record = json.loads(line)
                    delta = np.asarray(record["delta_rad"], dtype=np.float32)
                    if delta.shape != (len(ARM_JOINTS),) or not np.isfinite(delta).all():
                        raise ValueError("shadow action must contain six finite delta_rad values")
                    if np.max(np.abs(delta)) > 0.01 + 1e-9:
                        raise ValueError("shadow action exceeds 0.01 rad step guard")
                    shadow_actions.append(delta)
        shadow_log = None
        if args.shadow_log:
            args.shadow_log.parent.mkdir(parents=True, exist_ok=True)
            shadow_log = args.shadow_log.open("w", encoding="utf-8")

        def joint_state_callback(message):
            name_to_index = {name: index for index, name in enumerate(message.name)}
            missing = [name for name in ARM_JOINTS if name not in name_to_index]
            if missing:
                node.get_logger().warning(
                    "Ignoring JointState missing arm joints: " + ", ".join(missing)
                )
                return

            values = []
            for dof_index, name in enumerate(ARM_JOINTS):
                message_index = name_to_index[name]
                if message_index >= len(message.position):
                    node.get_logger().warning(
                        f"Ignoring JointState: no position value for {name}"
                    )
                    return
                value = float(message.position[message_index])
                if not math.isfinite(value):
                    node.get_logger().warning(
                        f"Ignoring JointState: non-finite position for {name}"
                    )
                    return
                lower = float(limits[dof_index]["lower"])
                upper = float(limits[dof_index]["upper"])
                if value < lower - 1e-6 or value > upper + 1e-6:
                    node.get_logger().warning(
                        f"Ignoring JointState: {name}={value:.6f} is outside "
                        f"[{lower:.6f}, {upper:.6f}] rad"
                    )
                    return
                values.append(value)

            state["pending"] = np.asarray(values, dtype=np.float32)
            state["valid_count"] += 1
            state["last_arm"] = time.monotonic()

        subscription = node.create_subscription(
            JointState, args.topic, joint_state_callback, qos
        )
        def gripper_callback(message):
            if message.name != ["finger_joint"] or len(message.position) != 1:
                return
            value = float(message.position[0])
            lower = float(robot.dof_properties[6]["lower"])
            upper = float(robot.dof_properties[6]["upper"])
            if math.isfinite(value) and lower - 1e-6 <= value <= upper + 1e-6:
                state["gripper"] = np.asarray([value], dtype=np.float32)
                state["gripper_count"] += 1
                state["last_gripper"] = time.monotonic()
        gripper_subscription = node.create_subscription(
            JointState, args.gripper_topic, gripper_callback, qos
        )

        print("[PASS] Isaac articulation initialized", flush=True)
        print(f"[INFO] resolved articulation: {robot.prim_path}", flush=True)
        print(f"[INFO] runtime DOFs ({robot.num_dof}): {', '.join(runtime_names)}", flush=True)
        print(f"[SAFE] subscribing only: {args.topic}", flush=True)
        print("[SAFE] no ROS publisher, service client, command topic, or /clock", flush=True)
        if args.shadow_actions:
            print("[SHADOW] command_authority=shadow; actions target Isaac articulation only", flush=True)
            print("[SHADOW] FS100 transport=unreachable; no real-robot command path", flush=True)

        start_time = time.monotonic()
        applied_message_count = 0
        applied_gripper_count = 0
        shadow_index = 0
        shadow_positions = None
        zero_velocities = np.zeros(robot.num_dof, dtype=np.float32)
        stale_reported = {"arm": False, "gripper": False}
        while simulation_app.is_running():
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            for label, key, topic in (("arm", "last_arm", args.topic),
                                      ("gripper", "last_gripper", args.gripper_topic)):
                last = state[key]
                stale = last is None or now - last > args.stale_timeout
                if stale and not stale_reported[label]:
                    node.get_logger().warning(
                        f"STALE {label} feedback: no valid message on {topic} "
                        f"for {'startup' if last is None else f'{now-last:.2f}s'}"
                    )
                    stale_reported[label] = True
                elif not stale and stale_reported[label]:
                    node.get_logger().info(f"RECOVERED {label} feedback on {topic}")
                    stale_reported[label] = False
            if state["pending"] is not None and state["valid_count"] > applied_message_count:
                robot.set_joint_positions(state["pending"], joint_indices=arm_indices)
                applied_message_count = state["valid_count"]
                if shadow_positions is None:
                    shadow_positions = state["pending"].copy()
                observed = robot.get_joint_positions(joint_indices=arm_indices)
                print(
                    "[MIRROR] applied "
                    + " ".join(
                        f"{name}={float(value):.6f}"
                        for name, value in zip(ARM_JOINTS, observed)
                    ),
                    flush=True,
                )
                if args.exit_after_first_message:
                    result = 0
                    break
            if state["gripper"] is not None and state["gripper_count"] > applied_gripper_count:
                robot.set_joint_positions(state["gripper"], joint_indices=np.asarray([6], dtype=np.int32))
                applied_gripper_count = state["gripper_count"]
                observed_gripper = robot.get_joint_positions(
                    joint_indices=np.asarray([6], dtype=np.int32)
                )
                print(
                    f"[MIRROR-GRIPPER] applied {runtime_names[6]}="
                    f"{float(observed_gripper[0]):.6f}",
                    flush=True,
                )

            if shadow_positions is not None and shadow_index < len(shadow_actions):
                delta = shadow_actions[shadow_index]
                shadow_positions = shadow_positions + delta
                lower = limits["lower"].astype(np.float32)
                upper = limits["upper"].astype(np.float32)
                shadow_positions = np.clip(shadow_positions, lower, upper)
                robot.set_joint_positions(shadow_positions, joint_indices=arm_indices)
                if shadow_log:
                    shadow_log.write(json.dumps({
                        "mode": "shadow", "index": shadow_index,
                        "delta_rad": delta.tolist(),
                        "shadow_joint_position_rad": shadow_positions.tolist(),
                        "command_authority": "shadow",
                        "real_transport_touched": False,
                    }) + "\n")
                    shadow_log.flush()
                shadow_index += 1

            # Advance PhysX without rendering, then restore the measured pose
            # before rendering.  A mirror is a visualization of measured state,
            # not a free-running dynamics simulation; allowing the post-step
            # state to render makes a stationary physical robot appear to drift.
            world.step(render=False)
            if state["pending"] is not None:
                robot.set_joint_positions(state["pending"], joint_indices=arm_indices)
            if state["gripper"] is not None:
                robot.set_joint_positions(
                    state["gripper"], joint_indices=np.asarray([6], dtype=np.int32)
                )
            robot.set_joint_velocities(zero_velocities)
            if not args.headless:
                world.render()
            if args.timeout > 0 and time.monotonic() - start_time >= args.timeout:
                print(
                    f"[FAIL] no valid JointState received on {args.topic} "
                    f"within {args.timeout:g} seconds",
                    file=sys.stderr,
                    flush=True,
                )
                result = 3
                break
        else:
            result = 0

        # Keep the subscription referenced until after the event loop.
        del subscription
        del gripper_subscription
        if shadow_log:
            shadow_log.close()
        if not args.exit_after_first_message and result == 1:
            result = 0
    except KeyboardInterrupt:
        result = 0
    except Exception as exc:
        print(f"[FAIL] {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        result = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy_started:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        simulation_app.close()

    return result


if __name__ == "__main__":
    raise SystemExit(main())
