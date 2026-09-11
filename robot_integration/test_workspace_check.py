#!/usr/bin/env python3
"""Forward kinematics and workspace check tests.

Forward kinematics is validated against joint-to-target pairs recorded by MoveIt
in earlier accepted runs, so the check is anchored to independently produced
data rather than to its own output.
"""

import glob
import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_integration.kinematics import ARM_JOINTS, load_chain  # noqa: E402
from robot_integration.workspace_check import (  # noqa: E402
    WorkspaceBounds,
    WorkspaceChecker,
)

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "safety", "mh5_gateway_core_v0.yaml")
HOME = (0.0, 0.0, 0.0, 0.0, 0.0, 0.105)

# Maximum accepted deviation from the recorded MoveIt targets. The stored
# targets are rounded, so a sub-millimetre residual is expected.
FK_TOLERANCE_MM = 0.5


def load_ground_truth():
    """Collect (joint vector, grasp_link target) pairs from recorded MoveIt plans."""
    pairs = []
    for path in sorted(glob.glob(os.path.join(PROJECT_ROOT, "mh5_jg*.json"))
                       + glob.glob(os.path.join(PROJECT_ROOT, "mh5_grasp_jointgoal.json"))):
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError):
            continue
        if data.get("tcp_link") != "grasp_link":
            continue
        for segment in data.get("segments", []):
            target = segment.get("grasp_link_target_m")
            points = segment.get("points")
            if not target or not points:
                continue
            if tuple(segment.get("joint_names", ())) != ARM_JOINTS:
                continue
            pairs.append((tuple(points[-1]["positions_rad"]), tuple(target), path))
    return pairs


@pytest.fixture(scope="module")
def checker():
    return WorkspaceChecker.from_config(CONFIG_PATH)


# -- kinematics --------------------------------------------------------------

def test_ground_truth_pairs_exist():
    pairs = load_ground_truth()
    assert len(pairs) >= 10, f"expected at least 10 recorded pairs, found {len(pairs)}"


def test_forward_kinematics_matches_recorded_moveit_targets(checker):
    pairs = load_ground_truth()
    worst = 0.0
    for joints, target, path in pairs:
        got = checker.position_for(joints)
        error_mm = float(np.linalg.norm(got - np.array(target))) * 1000.0
        worst = max(worst, error_mm)
        assert error_mm < FK_TOLERANCE_MM, (
            f"{os.path.basename(path)}: FK error {error_mm:.4f} mm exceeds "
            f"{FK_TOLERANCE_MM} mm")
    assert worst < FK_TOLERANCE_MM


def test_chain_actuates_exactly_the_six_arm_joints(checker):
    assert checker.chain.actuated_names == ARM_JOINTS


def test_zero_pose_is_reproducible(checker):
    first = checker.position_for(HOME)
    second = checker.position_for(HOME)
    assert np.allclose(first, second)


def test_joint_1_rotation_moves_the_tool_in_y(checker):
    """A positive S rotation must change the tool position, not leave it fixed."""
    base = checker.position_for(HOME)
    rotated = checker.position_for((0.3, 0.0, 0.0, 0.0, 0.0, 0.105))
    assert not np.allclose(base, rotated, atol=1e-4)
    # Rotation about the base vertical axis preserves distance from that axis.
    r_base = math.hypot(float(base[0]), float(base[1]))
    r_rot = math.hypot(float(rotated[0]), float(rotated[1]))
    assert abs(r_base - r_rot) < 1e-6


def test_wrong_joint_count_raises(checker):
    with pytest.raises(ValueError):
        checker.chain.fk_position((0.0, 0.0, 0.0))


# -- workspace acceptance ----------------------------------------------------

def test_recorded_grasp_targets_are_inside_the_configured_volume(checker):
    """Every pose the planner actually used must survive the workspace check."""
    for joints, target, path in load_ground_truth():
        assert checker(joints) is True, (
            f"{os.path.basename(path)} target {target} rejected: {checker.last_reason}")


def test_point_above_ceiling_rejected(checker):
    bounds = checker.bounds
    high = WorkspaceChecker(checker.chain, WorkspaceBounds(
        x_m=bounds.x_m, y_m=bounds.y_m, z_m=(bounds.z_m[0], 0.02),
        max_radius_m=bounds.max_radius_m, min_radius_m=bounds.min_radius_m))
    joints, _, _ = load_ground_truth()[0]
    assert high(joints) is False
    assert "outside" in high.last_reason


def test_point_below_floor_rejected(checker):
    """A pose that drives the tool under the configured floor must be rejected."""
    bounds = checker.bounds
    raised_floor = WorkspaceChecker(checker.chain, WorkspaceBounds(
        x_m=bounds.x_m, y_m=bounds.y_m, z_m=(0.50, bounds.z_m[1]),
        max_radius_m=bounds.max_radius_m, min_radius_m=bounds.min_radius_m))
    joints, _, _ = load_ground_truth()[0]
    assert raised_floor(joints) is False
    assert "z=" in raised_floor.last_reason


def test_radius_ceiling_rejects_far_reach(checker):
    bounds = checker.bounds
    tight = WorkspaceChecker(checker.chain, WorkspaceBounds(
        x_m=bounds.x_m, y_m=bounds.y_m, z_m=bounds.z_m,
        max_radius_m=0.30, min_radius_m=0.01))
    joints, _, _ = load_ground_truth()[0]
    assert tight(joints) is False
    assert "exceeds" in tight.last_reason


def test_radius_floor_rejects_tucked_pose(checker):
    bounds = checker.bounds
    tight = WorkspaceChecker(checker.chain, WorkspaceBounds(
        x_m=bounds.x_m, y_m=bounds.y_m, z_m=bounds.z_m,
        max_radius_m=bounds.max_radius_m, min_radius_m=0.80))
    joints, _, _ = load_ground_truth()[0]
    assert tight(joints) is False
    assert "below" in tight.last_reason


# -- fail closed -------------------------------------------------------------

@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_input_rejected(checker, bad):
    assert checker((bad, 0.0, 0.0, 0.0, 0.0, 0.0)) is False
    assert checker.last_reason == "non-finite joint value"


def test_wrong_length_input_rejected(checker):
    assert checker((0.0, 0.0, 0.0)) is False
    assert checker.last_reason == "wrong joint count"


def test_invalid_bounds_rejected_at_construction(checker):
    with pytest.raises(ValueError):
        WorkspaceChecker(checker.chain, WorkspaceBounds(
            x_m=(0.8, 0.3), y_m=(-0.3, 0.3), z_m=(0.0, 0.5),
            max_radius_m=0.85, min_radius_m=0.25))


def test_inverted_radii_rejected_at_construction(checker):
    with pytest.raises(ValueError):
        WorkspaceChecker(checker.chain, WorkspaceBounds(
            x_m=(0.3, 0.8), y_m=(-0.3, 0.3), z_m=(0.0, 0.5),
            max_radius_m=0.20, min_radius_m=0.50))


def test_module_has_no_transport():
    import ast
    import robot_integration.workspace_check as module
    tree = ast.parse(open(module.__file__, "r", encoding="utf-8").read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    banned = {"socket", "rclpy", "rospy", "subprocess", "pymodbus", "requests", "urllib"}
    assert not (imported & banned)


# -- integration with the gateway -------------------------------------------

def test_gateway_accepts_checker_and_still_refuses_to_arm():
    """The real checker satisfies the callback contract but does not unlock arming."""
    from robot_integration.safety_gateway_core import (
        GatewayConfig, GatewayRejection, SafetyGatewayCore)
    config = GatewayConfig.load(CONFIG_PATH)
    gateway = SafetyGatewayCore(
        config, sink=None,
        workspace_check=WorkspaceChecker.from_config(CONFIG_PATH),
        collision_check=lambda q: True)
    with pytest.raises(GatewayRejection) as exc:
        gateway.arm(now_s=0.0, operator_ack=True)
    assert exc.value.code == "ARM_PRECONDITION_UNMET"
    assert "joint_sign_order_units_verified" in exc.value.detail
