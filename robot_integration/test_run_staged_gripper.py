import pytest

from run_staged_gripper import opening_from_report


def report(timestamp=100.0, opening=0.042, valid=True):
    return {"timestamp_unix": timestamp,
            "grasp_stages": {"valid": valid, "gripper_open_m": opening}}


def test_opening_converts_meters_to_tenth_mm():
    width, age = opening_from_report(report(), 20.0, now=105.0)
    assert width == 420 and age == 5.0


def test_opening_rejects_stale_future_invalid_and_out_of_stroke():
    with pytest.raises(ValueError, match="age"):
        opening_from_report(report(), 2.0, now=105.0)
    with pytest.raises(ValueError, match="age"):
        opening_from_report(report(timestamp=110.0), 20.0, now=105.0)
    with pytest.raises(ValueError, match="valid grasp"):
        opening_from_report(report(valid=False), 20.0, now=105.0)
    with pytest.raises(ValueError, match="stroke"):
        opening_from_report(report(opening=0.101), 20.0, now=105.0)
