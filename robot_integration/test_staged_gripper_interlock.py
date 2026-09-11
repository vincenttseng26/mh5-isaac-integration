import pytest

from run_staged_grasp import validate_gripper_interlock


def test_grasp_requires_stationary_sufficient_opening():
    validate_gripper_interlock("grasp", {
        "busy": False, "width_tenth_mm": 420, "grip_detected": False}, 0.044)
    with pytest.raises(ValueError, match="still moving"):
        validate_gripper_interlock("grasp", {
            "busy": True, "width_tenth_mm": 440}, 0.044)
    with pytest.raises(ValueError, match="need at least"):
        validate_gripper_interlock("grasp", {
            "busy": False, "width_tenth_mm": 419}, 0.044)


def test_lift_requires_stationary_detected_grip():
    validate_gripper_interlock("lift", {
        "busy": False, "width_tenth_mm": 300, "grip_detected": True})
    with pytest.raises(ValueError, match="grip_detected=false"):
        validate_gripper_interlock("lift", {
            "busy": False, "width_tenth_mm": 0, "grip_detected": False})
