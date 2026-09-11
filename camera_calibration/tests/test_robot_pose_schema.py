"""Synthetic tests for the transport-agnostic robot pose sample schema.

No robot, camera, or network transport is touched — every sample here is
hand-built JSON-like data representing what an FS100 adapter (once built)
or any other controller adapter would be expected to emit.
"""
import copy

import pytest

from camera_calibration.robot_pose_schema import (
    PoseSampleError,
    RobotPoseSample,
    TimeSyncError,
    UnverifiedRepresentationError,
    check_time_sync,
)


def make_valid_sample(sample_id="0000", monotonic_ns=1_000_000_000, utc="2026-09-08T03:14:15.000000Z"):
    return {
        "schema_version": "1.0",
        "sample_id": sample_id,
        "timestamp": {"monotonic_ns": monotonic_ns, "utc_iso8601": utc},
        "source": {
            "controller": "FS100",
            "robot_model": "MH5",
            "adapter": "synthetic-test",
            "adapter_version": "0.0.0",
            "transport": "unconfirmed",
        },
        "frame_id": "robot_base",
        "tool_id": 0,
        "user_frame_id": None,
        "joints": {"unit": "degree", "values": [0.0, -10.0, 20.0, 0.0, 30.0, 0.0]},
        "tcp_pose": {
            "position": {"unit": "m", "x": 0.4, "y": 0.0, "z": 0.3},
            "orientation": {"representation": "quaternion_wxyz", "values": [1.0, 0.0, 0.0, 0.0]},
        },
    }


def test_valid_sample_round_trips():
    sample = RobotPoseSample.from_dict(make_valid_sample())
    assert sample.sample_id == "0000"
    assert sample.frame_id == "robot_base"
    assert sample.is_production_ready is False  # transport is "unconfirmed"
    back = sample.to_dict()
    assert back["tool_id"] == 0
    assert back["joints"]["values"] == [0.0, -10.0, 20.0, 0.0, 30.0, 0.0]


def test_rotation_matrix_orientation_also_accepted():
    data = make_valid_sample()
    data["tcp_pose"]["orientation"] = {
        "representation": "rotation_matrix_3x3",
        "values": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    }
    sample = RobotPoseSample.from_dict(data)
    assert sample.tcp_pose.orientation.representation == "rotation_matrix_3x3"


@pytest.mark.parametrize("bad_unit", ["pulse", "counts", "raw_encoder"])
def test_pulse_joint_units_are_rejected_not_guessed(bad_unit):
    data = make_valid_sample()
    data["joints"]["unit"] = bad_unit
    with pytest.raises(UnverifiedRepresentationError, match="pulse"):
        RobotPoseSample.from_dict(data)


@pytest.mark.parametrize("bad_rep", ["euler_xyz", "euler_rz_ry_rx", "rpy_deg", "axis_angle"])
def test_euler_orientation_is_rejected_not_guessed(bad_rep):
    data = make_valid_sample()
    data["tcp_pose"]["orientation"] = {"representation": bad_rep, "values": [0.0, 0.0, 0.0]}
    with pytest.raises(UnverifiedRepresentationError, match="not verified"):
        RobotPoseSample.from_dict(data)


def test_unknown_transport_is_rejected():
    data = make_valid_sample()
    data["source"]["transport"] = "fs100_hses_udp"  # not yet confirmed/allowlisted
    with pytest.raises(UnverifiedRepresentationError, match="allowlist"):
        RobotPoseSample.from_dict(data)


def test_wrong_joint_count_rejected():
    data = make_valid_sample()
    data["joints"]["values"] = [0.0] * 7
    with pytest.raises(PoseSampleError, match="exactly 6"):
        RobotPoseSample.from_dict(data)


def test_unnormalized_quaternion_rejected():
    data = make_valid_sample()
    data["tcp_pose"]["orientation"]["values"] = [2.0, 0.0, 0.0, 0.0]
    with pytest.raises(PoseSampleError, match="not normalized"):
        RobotPoseSample.from_dict(data)


def test_missing_position_unit_rejected():
    data = make_valid_sample()
    del data["tcp_pose"]["position"]["unit"]
    with pytest.raises(PoseSampleError, match="unit"):
        RobotPoseSample.from_dict(data)


def test_naive_utc_timestamp_rejected():
    data = make_valid_sample(utc="2026-09-08T03:14:15.000000")  # no tz suffix
    with pytest.raises(TimeSyncError, match="timezone"):
        RobotPoseSample.from_dict(data)


def test_negative_tool_id_rejected():
    data = make_valid_sample()
    data["tool_id"] = -1
    with pytest.raises(PoseSampleError, match="tool_id"):
        RobotPoseSample.from_dict(data)


def test_wrong_schema_version_rejected():
    data = make_valid_sample()
    data["schema_version"] = "0.9"
    with pytest.raises(PoseSampleError, match="schema_version"):
        RobotPoseSample.from_dict(data)


def test_user_frame_id_none_means_robot_coordinate():
    data = make_valid_sample()
    data["user_frame_id"] = None
    sample = RobotPoseSample.from_dict(data)
    assert sample.user_frame_id is None


def test_user_frame_id_set_when_using_user_coordinate():
    data = make_valid_sample()
    data["user_frame_id"] = 3
    sample = RobotPoseSample.from_dict(data)
    assert sample.user_frame_id == 3


# --- time sync checks -------------------------------------------------

def _shift(data, monotonic_delta_ns, utc_delta_s):
    import datetime as dt
    new = copy.deepcopy(data)
    new["timestamp"]["monotonic_ns"] += monotonic_delta_ns
    base = dt.datetime.fromisoformat(data["timestamp"]["utc_iso8601"][:-1] + "+00:00")
    shifted = base + dt.timedelta(seconds=utc_delta_s)
    new["timestamp"]["utc_iso8601"] = shifted.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"
    return new


def test_time_sync_clean_when_clocks_agree():
    s0 = RobotPoseSample.from_dict(make_valid_sample(sample_id="0000"))
    d1 = _shift(make_valid_sample(sample_id="0000"), monotonic_delta_ns=200_000_000, utc_delta_s=0.2)
    d1["sample_id"] = "0001"
    s1 = RobotPoseSample.from_dict(d1)
    problems = check_time_sync([s0, s1], max_skew_ms=50.0)
    assert problems == []


def test_time_sync_flags_clock_skew():
    s0 = RobotPoseSample.from_dict(make_valid_sample(sample_id="0000"))
    # monotonic advances 200ms but UTC only advances 20ms -> 180ms skew, over 50ms budget
    d1 = _shift(make_valid_sample(sample_id="0000"), monotonic_delta_ns=200_000_000, utc_delta_s=0.02)
    d1["sample_id"] = "0001"
    s1 = RobotPoseSample.from_dict(d1)
    problems = check_time_sync([s0, s1], max_skew_ms=50.0)
    assert any("skew" in p for p in problems)


def test_time_sync_flags_utc_clock_going_backwards():
    s0 = RobotPoseSample.from_dict(make_valid_sample(sample_id="0000"))
    d1 = _shift(make_valid_sample(sample_id="0000"), monotonic_delta_ns=200_000_000, utc_delta_s=-1.0)
    d1["sample_id"] = "0001"
    s1 = RobotPoseSample.from_dict(d1)
    problems = check_time_sync([s0, s1], max_skew_ms=50.0)
    assert any("backwards" in p for p in problems)


def test_time_sync_flags_duplicate_sample_id():
    s0 = RobotPoseSample.from_dict(make_valid_sample(sample_id="dup"))
    s1 = RobotPoseSample.from_dict(make_valid_sample(sample_id="dup", monotonic_ns=2_000_000_000,
                                                       utc="2026-09-08T03:14:16.000000Z"))
    problems = check_time_sync([s0, s1], max_skew_ms=50.0)
    assert any("duplicate" in p for p in problems)
