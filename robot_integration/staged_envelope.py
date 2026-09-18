#!/usr/bin/env python3
"""Motion envelope for the staged MH5 execution channel.

Pure logic: no ROS, no transport, no publisher. Imported by the CYC receiver so
that the rules below can be unit-tested off the robot.

This channel exists because the original one-shot channel caps the TOTAL
displacement of a trajectory at 0.035 rad (about 2 degrees), which is a
deliberate first-contact envelope. A reach-and-grasp needs tens of degrees, so
rather than widening that cap -- which would silently weaken every existing
path -- this module defines a SEPARATE, larger, explicitly-limited envelope that
the operator must select on purpose, one stage at a time.

What this envelope does and does not guarantee:

  * It DOES bound displacement, per-waypoint step and joint velocity, and it
    re-derives every timestamp itself rather than trusting the client.
  * It does NOT check collisions. Collision-freedom comes from the MoveIt plan
    gateway, which validates the start and goal states and plans against the
    fixed workcell scene. This module is the kinematic envelope, not a
    collision checker, and it cannot rescue an unsafe path.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

# The reviewed first-contact velocity from config/safety/mh5_gateway_core_v0.yaml.
# Timestamps are recomputed from this, so a client cannot ask for a faster move.
STAGED_MAX_VELOCITY_RAD_S = 0.10

# One stage may move a joint by at most this much. The measured worst stage is
# the pre-grasp approach at about 0.88 rad, so this leaves headroom without
# approaching a free swing of the wrist.
STAGED_MAX_TOTAL_RAD = 1.20

# Largest permitted gap between consecutive waypoints. MoveIt's own output for
# these stages peaks near 0.11 rad.
STAGED_MAX_SEGMENT_RAD = 0.15

# The first waypoint must match live feedback at least as tightly as the
# original channel demands.
STAGED_MAX_START_ERROR_RAD = 0.02

STAGED_MAX_DURATION_S = 30.0
STAGED_MIN_SEGMENT_TIME_S = 0.05
STAGED_MAX_POINTS = 400
STAGED_FEEDBACK_MAX_AGE_S = 0.25

STAGES = ("pre_grasp", "grasp", "lift", "transport", "place", "retreat", "home")
"""Permitted stage labels.

``transport`` and ``place`` carry an object: ``transport`` crosses to the
destination at the lift height, ``place`` descends to set the object down. They
are labels for the audit trail only -- this module bounds every stage the same
way regardless of name, and the gripper interlock lives in the client.
"""


class StageRejected(ValueError):
    """Raised for every refusal, so the caller can fail closed uniformly."""


def _finite_vector(values: Sequence[float], label: str) -> Tuple[float, ...]:
    try:
        out = tuple(float(v) for v in values)
    except (TypeError, ValueError):
        raise StageRejected(f"{label} is not numeric")
    if len(out) != 6:
        raise StageRejected(f"{label} must contain six joint values")
    if not all(math.isfinite(v) for v in out):
        raise StageRejected(f"{label} contains a non-finite value")
    return out


def check_preconditions(mode: str, armed: bool, nonce: Optional[str],
                        record_nonce: Optional[str], actual: Optional[Sequence[float]],
                        joint_age: float, robot: Optional[Dict],
                        robot_age: float) -> Tuple[float, ...]:
    """Every gate that must hold before a staged move is even parsed."""
    if mode != "STAGED":
        raise StageRejected("gateway is not in STAGED mode")
    if not armed:
        raise StageRejected("gateway is not armed")
    if not nonce:
        raise StageRejected("no stage nonce has been issued")
    if not record_nonce or record_nonce != nonce:
        raise StageRejected("stage nonce does not match the issued one")
    if actual is None or not math.isfinite(joint_age) or joint_age > STAGED_FEEDBACK_MAX_AGE_S:
        raise StageRejected("joint feedback is stale")
    if robot is None or not math.isfinite(robot_age) or robot_age > STAGED_FEEDBACK_MAX_AGE_S:
        raise StageRejected("robot status is stale")
    if not robot.get("drives_powered"):
        raise StageRejected("drives are not powered")
    if not robot.get("motion_possible"):
        raise StageRejected("controller reports motion is not possible")
    if robot.get("e_stopped"):
        raise StageRejected("robot is E-stopped")
    if robot.get("in_error"):
        raise StageRejected("controller is in error")
    if robot.get("in_motion"):
        raise StageRejected("robot is already moving")
    return _finite_vector(actual, "physical feedback")


def retime(points: Sequence[Sequence[float]]) -> List[Tuple[Tuple[float, ...], float]]:
    """Assign timestamps so no joint exceeds the staged velocity ceiling.

    Client timing is discarded, not validated: the only timestamps that reach
    the controller are the ones derived here.
    """
    if len(points) < 2:
        raise StageRejected("a stage needs at least two waypoints")
    timed: List[Tuple[Tuple[float, ...], float]] = []
    elapsed = 0.0
    previous: Optional[Tuple[float, ...]] = None
    for raw in points:
        current = tuple(float(v) for v in raw)
        if previous is not None:
            step = max(abs(a - b) for a, b in zip(current, previous))
            elapsed += max(step / STAGED_MAX_VELOCITY_RAD_S, STAGED_MIN_SEGMENT_TIME_S)
        else:
            # ROS-Industrial requires a strictly positive first stamp.
            elapsed = STAGED_MIN_SEGMENT_TIME_S
        timed.append((current, elapsed))
        previous = current
    if timed[-1][1] > STAGED_MAX_DURATION_S:
        raise StageRejected(f"stage would run {timed[-1][1]:.1f} s, over the "
                            f"{STAGED_MAX_DURATION_S:.0f} s ceiling")
    return timed


def validate_stage(record: Dict, actual: Sequence[float]
                   ) -> Tuple[str, List[Tuple[Tuple[float, ...], float]]]:
    """Validate one staged trajectory and return its label and retimed points."""
    if record.get("schema") != 3:
        raise StageRejected("staged trajectory requires schema 3")
    stage = record.get("stage")
    if stage not in STAGES:
        raise StageRejected(f"unknown stage {stage!r}")

    raw_points = record.get("points")
    if not isinstance(raw_points, (list, tuple)):
        raise StageRejected("points must be a list")
    if not 2 <= len(raw_points) <= STAGED_MAX_POINTS:
        raise StageRejected(f"a stage needs 2..{STAGED_MAX_POINTS} waypoints")

    positions = [_finite_vector(item.get("positions", ()) if isinstance(item, dict)
                                else item, "waypoint")
                 for item in raw_points]

    start = _finite_vector(actual, "physical feedback")
    if max(abs(a - b) for a, b in zip(positions[0], start)) > STAGED_MAX_START_ERROR_RAD:
        raise StageRejected("first waypoint does not match physical feedback")

    previous = positions[0]
    for index, current in enumerate(positions[1:], 1):
        step = max(abs(a - b) for a, b in zip(current, previous))
        if step > STAGED_MAX_SEGMENT_RAD:
            raise StageRejected(f"waypoint {index} steps {step:.4f} rad, over the "
                                f"{STAGED_MAX_SEGMENT_RAD:.3f} rad limit")
        previous = current

    total = max(abs(a - b) for a, b in zip(positions[-1], start))
    if total > STAGED_MAX_TOTAL_RAD:
        raise StageRejected(f"stage moves a joint {total:.4f} rad, over the "
                            f"{STAGED_MAX_TOTAL_RAD:.3f} rad stage limit")

    declared = record.get("expected_total_rad")
    if declared is not None:
        try:
            declared = float(declared)
        except (TypeError, ValueError):
            raise StageRejected("expected_total_rad is not numeric")
        if not math.isfinite(declared) or abs(declared - total) > 1e-3:
            raise StageRejected(f"client expected {declared:.4f} rad but the "
                                f"trajectory moves {total:.4f} rad")

    return stage, retime(positions)
