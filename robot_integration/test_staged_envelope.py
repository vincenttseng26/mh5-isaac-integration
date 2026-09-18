#!/usr/bin/env python3
"""Tests for the staged execution envelope. Pure logic, no robot."""
import math

import pytest

from staged_envelope import (STAGED_MAX_DURATION_S, STAGED_MAX_POINTS,
                             STAGED_MAX_SEGMENT_RAD, STAGED_MAX_START_ERROR_RAD,
                             STAGED_MAX_TOTAL_RAD, STAGED_MAX_VELOCITY_RAD_S,
                             STAGED_MIN_SEGMENT_TIME_S, StageRejected,
                             check_preconditions, retime, validate_stage)

HOME = (0.0,) * 6
GOOD_ROBOT = {"drives_powered": True, "motion_possible": True, "e_stopped": False,
              "in_error": False, "in_motion": False}


def ramp(axis, total, count):
    """A straight line in joint space, evenly divided into count waypoints."""
    out = []
    for i in range(count):
        values = [0.0] * 6
        values[axis] = total * i / (count - 1)
        out.append({"positions": list(values)})
    return out


def stage_record(**kwargs):
    record = {"schema": 3, "stage": "pre_grasp", "points": ramp(0, 0.40, 20)}
    record.update(kwargs)
    return record


# --- preconditions ----------------------------------------------------------

def ok_preconditions(**kwargs):
    args = {"mode": "STAGED", "armed": True, "nonce": "abc", "record_nonce": "abc",
            "actual": HOME, "joint_age": 0.01, "robot": GOOD_ROBOT, "robot_age": 0.01}
    args.update(kwargs)
    return check_preconditions(**args)


def test_preconditions_pass_when_everything_is_healthy():
    assert ok_preconditions() == HOME


@pytest.mark.parametrize("override,message", [
    ({"mode": "MIRROR"}, "not in STAGED mode"),
    ({"mode": "COMMAND"}, "not in STAGED mode"),
    ({"armed": False}, "not armed"),
    ({"nonce": None}, "no stage nonce"),
    ({"record_nonce": None}, "nonce does not match"),
    ({"record_nonce": "wrong"}, "nonce does not match"),
    ({"joint_age": 5.0}, "joint feedback is stale"),
    ({"actual": None}, "joint feedback is stale"),
    ({"robot_age": 5.0}, "robot status is stale"),
    ({"robot": None}, "robot status is stale"),
    ({"robot": dict(GOOD_ROBOT, drives_powered=False)}, "drives are not powered"),
    ({"robot": dict(GOOD_ROBOT, motion_possible=False)}, "motion is not possible"),
    ({"robot": dict(GOOD_ROBOT, e_stopped=True)}, "E-stopped"),
    ({"robot": dict(GOOD_ROBOT, in_error=True)}, "in error"),
    ({"robot": dict(GOOD_ROBOT, in_motion=True)}, "already moving"),
])
def test_every_precondition_fails_closed(override, message):
    with pytest.raises(StageRejected, match=message):
        ok_preconditions(**override)


def test_a_replayed_nonce_is_refused_once_cleared():
    # The receiver clears the nonce after publishing; a resend must then fail.
    with pytest.raises(StageRejected, match="no stage nonce"):
        ok_preconditions(nonce=None, record_nonce="abc")


# --- retiming ---------------------------------------------------------------

def test_retime_never_exceeds_the_velocity_ceiling():
    points = [p["positions"] for p in ramp(0, 0.90, 30)]
    timed = retime(points)
    assert len(timed) == len(points)
    assert timed[0][1] == pytest.approx(STAGED_MIN_SEGMENT_TIME_S)
    for (a, ta), (b, tb) in zip(timed, timed[1:]):
        step = max(abs(x - y) for x, y in zip(a, b))
        dt = tb - ta
        assert dt > 0.0
        assert step / dt <= STAGED_MAX_VELOCITY_RAD_S + 1e-9


def test_retime_is_strictly_increasing_even_for_repeated_points():
    points = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    timed = retime(points)
    stamps = [t for _, t in timed]
    assert all(b > a for a, b in zip(stamps, stamps[1:]))


def test_retime_discards_client_timing():
    # Client stamps are not part of the input at all; only positions are.
    points = [p["positions"] for p in ramp(0, 0.20, 5)]
    assert [t for _, t in retime(points)] == [t for _, t in retime(points)]


def test_retime_rejects_a_stage_that_would_run_too_long():
    # Many large steps at the velocity ceiling exceed the duration ceiling.
    points = [[0.0] * 6]
    for i in range(1, 40):
        values = [0.0] * 6
        values[0] = 0.14 * i
        points.append(values)
    with pytest.raises(StageRejected, match="over the"):
        retime(points)


def test_retime_needs_two_points():
    with pytest.raises(StageRejected, match="at least two waypoints"):
        retime([[0.0] * 6])


# --- stage validation -------------------------------------------------------

def test_a_well_formed_stage_is_accepted_and_retimed():
    stage, timed = validate_stage(stage_record(), HOME)
    assert stage == "pre_grasp"
    assert len(timed) == 20
    assert timed[-1][1] <= STAGED_MAX_DURATION_S
    # 0.40 rad at 0.10 rad/s is about 4 s.
    assert timed[-1][1] == pytest.approx(0.40 / STAGED_MAX_VELOCITY_RAD_S, abs=0.1)


def test_schema_and_stage_name_are_enforced():
    with pytest.raises(StageRejected, match="schema 3"):
        validate_stage(stage_record(schema=2), HOME)
    with pytest.raises(StageRejected, match="unknown stage"):
        validate_stage(stage_record(stage="freestyle"), HOME)
    with pytest.raises(StageRejected, match="unknown stage"):
        validate_stage(stage_record(stage=None), HOME)


def test_first_waypoint_must_match_physical_feedback():
    drifted = [STAGED_MAX_START_ERROR_RAD * 2] + [0.0] * 5
    with pytest.raises(StageRejected, match="does not match physical feedback"):
        validate_stage(stage_record(), drifted)


def test_a_step_larger_than_the_segment_limit_is_refused():
    points = [{"positions": [0.0] * 6},
              {"positions": [STAGED_MAX_SEGMENT_RAD * 1.5] + [0.0] * 5}]
    with pytest.raises(StageRejected, match="over the"):
        validate_stage(stage_record(points=points), HOME)


def test_total_displacement_over_the_stage_limit_is_refused():
    over = STAGED_MAX_TOTAL_RAD + 0.10
    with pytest.raises(StageRejected, match="over the .* rad stage limit"):
        validate_stage(stage_record(points=ramp(0, over, 40)), HOME)


def test_a_stage_at_exactly_the_total_limit_is_allowed():
    stage, timed = validate_stage(
        stage_record(points=ramp(0, STAGED_MAX_TOTAL_RAD, 40)), HOME)
    assert stage == "pre_grasp" and timed


def test_declared_total_must_match_the_trajectory():
    with pytest.raises(StageRejected, match="client expected"):
        validate_stage(stage_record(expected_total_rad=0.99), HOME)
    stage, _ = validate_stage(stage_record(expected_total_rad=0.40), HOME)
    assert stage == "pre_grasp"


def test_declared_total_must_be_numeric():
    with pytest.raises(StageRejected, match="not numeric"):
        validate_stage(stage_record(expected_total_rad="lots"), HOME)
    with pytest.raises(StageRejected, match="client expected"):
        validate_stage(stage_record(expected_total_rad=float("nan")), HOME)


def test_malformed_points_are_refused():
    with pytest.raises(StageRejected, match="points must be a list"):
        validate_stage(stage_record(points="nope"), HOME)
    with pytest.raises(StageRejected, match="2\\.\\."):
        validate_stage(stage_record(points=[{"positions": [0.0] * 6}]), HOME)
    with pytest.raises(StageRejected, match="six joint values"):
        validate_stage(stage_record(points=[{"positions": [0.0] * 3},
                                            {"positions": [0.0] * 3}]), HOME)
    with pytest.raises(StageRejected, match="non-finite"):
        validate_stage(stage_record(points=[{"positions": [0.0] * 6},
                                            {"positions": [float("inf")] + [0.0] * 5}]),
                       HOME)
    with pytest.raises(StageRejected, match="not numeric"):
        validate_stage(stage_record(points=[{"positions": ["a"] * 6},
                                            {"positions": [0.0] * 6}]), HOME)


def test_too_many_points_are_refused():
    many = ramp(0, 0.40, STAGED_MAX_POINTS + 5)
    with pytest.raises(StageRejected, match="2\\.\\."):
        validate_stage(stage_record(points=many), HOME)


def test_bare_position_lists_are_accepted_as_waypoints():
    points = [[0.0] * 6, [0.05] + [0.0] * 5]
    stage, timed = validate_stage(stage_record(points=points), HOME)
    assert len(timed) == 2


def test_the_staged_limits_stay_far_below_a_free_swing():
    """A guard on the constants themselves, so a later edit is deliberate."""
    assert STAGED_MAX_VELOCITY_RAD_S <= 0.10
    assert STAGED_MAX_TOTAL_RAD <= 1.20      # about 69 degrees
    assert STAGED_MAX_SEGMENT_RAD <= 0.15
    assert STAGED_MAX_START_ERROR_RAD <= 0.02
    assert STAGED_MAX_DURATION_S <= 30.0


def test_transport_and_place_are_permitted_stage_labels():
    """Pick-and-place needs stages beyond reaching for the object."""
    for stage in ("transport", "place"):
        name, timed = validate_stage(stage_record(stage=stage), HOME)
        assert name == stage
        assert timed[-1][1] <= STAGED_MAX_DURATION_S


def test_new_stages_are_bounded_exactly_like_the_others():
    """A carrying stage gets no extra allowance just for being named one."""
    over = ramp(0, STAGED_MAX_TOTAL_RAD + 0.10, 40)
    for stage in ("transport", "place"):
        with pytest.raises(StageRejected, match="rad stage limit"):
            validate_stage(stage_record(stage=stage, points=over), HOME)
        big_step = [{"positions": [0.0] * 6},
                    {"positions": [STAGED_MAX_SEGMENT_RAD * 1.5] + [0.0] * 5}]
        with pytest.raises(StageRejected, match="over the"):
            validate_stage(stage_record(stage=stage, points=big_step), HOME)


def test_stage_list_is_the_reviewed_set():
    from staged_envelope import STAGES
    assert STAGES == ("pre_grasp", "grasp", "lift", "transport", "place",
                      "retreat", "home")
