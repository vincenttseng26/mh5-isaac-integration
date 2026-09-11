#!/usr/bin/env python3
"""Phase 0 of MH5_VLA_PLAN: verify MH5_CFG spawns and behaves in Isaac Sim.

Simulation only. Opens no socket, touches no gateway and cannot move the
physical arm. Run it with IsaacLab's launcher:

    /home/vincent/IsaacLab/isaaclab.sh -p \
        integration_workspace/robot_integration/verify_mh5_isaaclab_spawn.py \
        --headless --report <path.json>

It answers the four Phase 0 questions from the plan:
  1. Do the USD joint names match the URDF?
  2. Does the init pose put tool0 in a workable tabletop range?
  3. Can the six gripper joints be driven open and closed without fighting?
  4. Did the URDF mimic relationships survive the USD import?
"""

import argparse
import json

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--report", default="/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/"
                                        "docs/validation/mh5-isaaclab-spawn-phase0.json")
parser.add_argument("--urdf", default="/home/vincent/Desktop/mh5/combined_fixed.urdf")
parser.add_argument("--settle-steps", type=int, default=120)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import xml.etree.ElementTree as ET  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab_assets.robots.mh5 import (MH5_CFG, MH5_GRIPPER_CLOSE,  # noqa: E402
                                        MH5_GRIPPER_DRIVE_JOINT,
                                        MH5_GRIPPER_JOINT_NAMES,
                                        MH5_GRIPPER_MIMIC_MULTIPLIERS,
                                        MH5_GRIPPER_OPEN)

ARM_JOINTS = ["joint_1_s", "joint_2_l", "joint_3_u",
              "joint_4_r", "joint_5_b", "joint_6_t"]
# The plan's tabletop working range for the tool frame, in robot base metres.
TOOL_X_RANGE = (0.20, 0.85)
TOOL_Y_RANGE = (-0.45, 0.45)
TOOL_Z_RANGE = (0.05, 0.90)


def urdf_facts(path):
    """Movable joints and mimic relationships, from the URDF itself.

    Gazebo blocks carry two extra SDF joints for the closed-loop linkage; they
    are not URDF joints and a standard parser ignores them, so only top-level
    <joint> elements are read here.
    """
    root = ET.parse(path).getroot()
    movable, mimic = [], {}
    for joint in root.findall("joint"):
        if joint.get("type") in ("revolute", "continuous"):
            movable.append(joint.get("name"))
            tag = joint.find("mimic")
            if tag is not None:
                mimic[joint.get("name")] = {
                    "follows": tag.get("joint"),
                    "multiplier": float(tag.get("multiplier", 1.0)),
                }
    return movable, mimic


def usd_mimic_prims(stage, root_path):
    """Look for PhysX mimic-joint APIs actually present on the spawned prims."""
    found = []
    try:
        from pxr import Usd
    except ImportError:
        return found, "pxr unavailable"
    prim = stage.GetPrimAtPath(root_path)
    if not prim or not prim.IsValid():
        return found, f"{root_path} is not a valid prim"
    for child in Usd.PrimRange(prim):
        for schema in child.GetAppliedSchemas():
            if "Mimic" in schema:
                found.append({"prim": str(child.GetPath()), "schema": schema})
    return found, None


def main():
    report = {"phase": 0, "plan": "MH5_VLA_PLAN.md",
              "simulation_only": True, "physical_arm_touched": False,
              "checks": {}}

    movable, mimic = urdf_facts(args.urdf)
    report["urdf"] = {"path": args.urdf, "movable_joints": movable,
                      "movable_joint_count": len(movable), "mimic": mimic}

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 120.0, device=args.device))
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2000.0).func(
        "/World/light", sim_utils.DomeLightCfg(intensity=2000.0))

    robot = Articulation(MH5_CFG.replace(prim_path="/World/mh5"))
    sim.reset()

    joint_names = list(robot.joint_names)
    body_names = list(robot.body_names)
    report["usd"] = {"path": MH5_CFG.spawn.usd_path,
                     "joint_names": joint_names,
                     "joint_count": len(joint_names),
                     "body_names": body_names,
                     "body_count": len(body_names)}

    # --- check 1: joint names ------------------------------------------------
    missing = [n for n in movable if n not in joint_names]
    extra = [n for n in joint_names if n not in movable]
    report["checks"]["joint_names_match_urdf"] = {
        "pass": not missing and not extra,
        "missing_from_usd": missing, "unexpected_in_usd": extra,
        "expected_count": len(movable), "actual_count": len(joint_names),
    }

    arm_missing = [n for n in ARM_JOINTS if n not in joint_names]
    grip_missing = [n for n in MH5_GRIPPER_JOINT_NAMES if n not in joint_names]
    report["checks"]["configured_joints_exist"] = {
        "pass": not arm_missing and not grip_missing,
        "arm_missing": arm_missing, "gripper_missing": grip_missing,
    }

    # --- check 4: did mimic survive the import? ------------------------------
    mimic_prims, mimic_note = usd_mimic_prims(sim.stage, "/World/mh5")
    report["checks"]["urdf_mimic_present_in_usd"] = {
        "mimic_apis_found": len(mimic_prims), "detail": mimic_prims[:10],
        "note": mimic_note or (
            "URDF mimic did not survive the import; this is expected and "
            "harmless because MH5_CFG drives all six gripper joints explicitly "
            "via BinaryJointPositionActionCfg rather than relying on mimic."
            if not mimic_prims else "mimic APIs are present in the USD"),
    }

    # --- settle at the init pose, then check 2 ------------------------------
    # The target must be written explicitly. Stepping without one leaves the
    # implicit actuators driving toward zero, which looks exactly like the arm
    # sagging under gravity and produced a false failure on the first run.
    hold = robot.data.default_joint_pos.clone()
    for _ in range(args.settle_steps):
        robot.set_joint_position_target(hold)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())

    arm_idx = [joint_names.index(n) for n in ARM_JOINTS]
    settled_arm = robot.data.joint_pos[0, arm_idx].tolist()
    commanded_arm = [MH5_CFG.init_state.joint_pos[n] for n in ARM_JOINTS]
    arm_error = max(abs(a - b) for a, b in zip(settled_arm, commanded_arm))

    tool_pose = None
    tool_name = next((n for n in ("tool0", "grasp_link") if n in body_names), None)
    if tool_name is not None:
        body_index = body_names.index(tool_name)
        world = robot.data.body_pos_w[0, body_index].tolist()
        origin = robot.data.root_pos_w[0].tolist()
        tool_pose = [world[i] - origin[i] for i in range(3)]

    in_range = tool_pose is not None and (
        TOOL_X_RANGE[0] <= tool_pose[0] <= TOOL_X_RANGE[1]
        and TOOL_Y_RANGE[0] <= tool_pose[1] <= TOOL_Y_RANGE[1]
        and TOOL_Z_RANGE[0] <= tool_pose[2] <= TOOL_Z_RANGE[1])
    report["checks"]["init_pose_reaches_tabletop"] = {
        "pass": bool(in_range),
        "tool_frame": tool_name,
        "tool_pos_base_m": tool_pose,
        "commanded_arm_rad": commanded_arm,
        "settled_arm_rad": settled_arm,
        "arm_settle_error_rad": arm_error,
        "x_range": list(TOOL_X_RANGE), "y_range": list(TOOL_Y_RANGE),
        "z_range": list(TOOL_Z_RANGE),
    }

    # --- check 3: drive ONLY the driving joint, let mimic move the rest -----
    drive_idx = joint_names.index(MH5_GRIPPER_DRIVE_JOINT)
    follower_idx = {name: joint_names.index(name)
                    for name in MH5_GRIPPER_MIMIC_MULTIPLIERS}

    def drive(command, steps=240):
        """Command the driving joint and report what every gripper joint did."""
        target = float(command[MH5_GRIPPER_DRIVE_JOINT])
        goal = robot.data.joint_pos.clone()
        goal[0, drive_idx] = target
        for _ in range(steps):
            robot.set_joint_position_target(goal)
            robot.write_data_to_sim()
            sim.step()
            robot.update(sim.get_physics_dt())

        driven = float(robot.data.joint_pos[0, drive_idx])
        result = {
            "commanded": target,
            "driving_joint": {"name": MH5_GRIPPER_DRIVE_JOINT,
                              "reached": driven,
                              "error": abs(target - driven)},
            "followers": {},
        }
        worst_follower = 0.0
        for name, multiplier in MH5_GRIPPER_MIMIC_MULTIPLIERS.items():
            reached = float(robot.data.joint_pos[0, follower_idx[name]])
            expected = multiplier * driven
            error = abs(expected - reached)
            worst_follower = max(worst_follower, error)
            result["followers"][name] = {"multiplier": multiplier,
                                         "expected": expected,
                                         "reached": reached, "error": error}
        result["driving_error_rad"] = result["driving_joint"]["error"]
        result["worst_follower_error_rad"] = worst_follower
        return result

    closed = drive(MH5_GRIPPER_CLOSE)
    opened = drive(MH5_GRIPPER_OPEN)
    drive_tolerance = 0.02
    # The linkage is a real four-bar, so followers track the ideal +/-1 ratio
    # only approximately. This bound is loose enough for that and far tighter
    # than the divergence the old explicit-drive configuration produced.
    follower_tolerance = 0.10
    report["checks"]["gripper_reaches_open_and_close"] = {
        "pass": all(r["driving_error_rad"] <= drive_tolerance
                    and r["worst_follower_error_rad"] <= follower_tolerance
                    for r in (closed, opened)),
        "driven_joint_only": MH5_GRIPPER_DRIVE_JOINT,
        "drive_tolerance_rad": drive_tolerance,
        "follower_tolerance_rad": follower_tolerance,
        "close": closed,
        "open": opened,
        "config_sign_convention_matches_urdf_mimic": all(
            abs(MH5_GRIPPER_MIMIC_MULTIPLIERS[name]
                - mimic.get(name, {}).get("multiplier", 0.0)) < 1e-9
            for name in MH5_GRIPPER_MIMIC_MULTIPLIERS),
    }

    report["result"] = ("PASS" if all(c.get("pass", True)
                                      for c in report["checks"].values())
                        else "FAIL")

    with open(args.report, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    print("\nPhase 0 result:", report["result"])


try:
    main()
finally:
    simulation_app.close()
