import math
import json
import subprocess
from pathlib import Path
import pytest
from robot_integration.interfaces import SafetyGate, SafetyViolationError, TCPPose, JointPose, RobotState, RobotStatus
from robot_integration.mock_transport import MockRobotTransport

@pytest.fixture
def robot():
    return MockRobotTransport()

@pytest.fixture
def safety_gate():
    # Workspace limits: X: -500 to 500, Y: -500 to 500, Z: 0 to 1000
    return SafetyGate(max_speed=100.0, workspace_limits=((-500, 500), (-500, 500), (0, 1000)))

def test_disconnected(robot, safety_gate):
    assert robot.read_status().state == RobotState.DISCONNECTED
    with pytest.raises(SafetyViolationError, match="disconnected"):
        safety_gate.check_safety(robot.read_status())

def test_servo_off(robot, safety_gate):
    robot.connect()
    robot.set_servo(False)
    assert robot.read_status().state == RobotState.SERVO_OFF
    with pytest.raises(SafetyViolationError, match="explicit SERVO_ON"):
        safety_gate.check_safety(robot.read_status())

def test_connected_without_explicit_servo_on_is_blocked(robot, safety_gate):
    robot.connect()
    assert robot.read_status().state == RobotState.CONNECTED
    with pytest.raises(SafetyViolationError, match="explicit SERVO_ON"):
        safety_gate.check_safety(robot.read_status())

def test_alarm(robot, safety_gate):
    robot.connect()
    robot.set_alarm(True, "ERR_404")
    assert robot.read_status().state == RobotState.ALARM
    with pytest.raises(SafetyViolationError, match="ALARM"):
        safety_gate.check_safety(robot.read_status())

def test_e_stop(robot, safety_gate):
    robot.connect()
    robot.set_e_stop(True)
    assert robot.read_status().state == RobotState.E_STOP
    with pytest.raises(SafetyViolationError, match="E-STOP"):
        safety_gate.check_safety(robot.read_status())

def test_out_of_workspace_target(robot, safety_gate):
    robot.connect()
    robot.set_servo(True)
    target = TCPPose(x=600.0, y=0.0, z=500.0, rx=0.0, ry=0.0, rz=0.0)
    with pytest.raises(SafetyViolationError, match="out of workspace limits"):
        safety_gate.check_safety(robot.read_status(), target_tcp=target)

def test_out_of_workspace_current(robot, safety_gate):
    robot.connect()
    robot.set_servo(True)
    robot.set_in_workspace(False)
    with pytest.raises(SafetyViolationError, match="currently out of workspace limits"):
        safety_gate.check_safety(robot.read_status())

def test_speed_limit(robot, safety_gate):
    robot.connect()
    robot.set_servo(True)
    with pytest.raises(SafetyViolationError, match="exceeds maximum allowed speed"):
        safety_gate.check_safety(robot.read_status(), speed=150.0)

def test_negative_speed(robot, safety_gate):
    robot.connect()
    robot.set_servo(True)
    with pytest.raises(SafetyViolationError, match="cannot be negative"):
        safety_gate.check_safety(robot.read_status(), speed=-10.0)

def test_target_orientation(robot, safety_gate):
    robot.connect()
    robot.set_servo(True)
    target = TCPPose(x=100.0, y=100.0, z=500.0, rx=10.0, ry=0.0, rz=0.0) # > 2*pi
    with pytest.raises(SafetyViolationError, match="orientation .* exceeds reasonable bounds"):
        safety_gate.check_safety(robot.read_status(), target_tcp=target)

def test_safe_operation(robot, safety_gate):
    robot.connect()
    robot.set_servo(True)
    target = TCPPose(x=100.0, y=100.0, z=500.0, rx=0.0, ry=0.0, rz=0.0)
    assert safety_gate.check_safety(robot.read_status(), target_tcp=target, speed=50.0) == True

def test_motion_commands_disabled(robot):
    with pytest.raises(NotImplementedError, match="strictly forbidden"):
        robot.move_j(None, 0.0)
    with pytest.raises(NotImplementedError, match="strictly forbidden"):
        robot.move_l(None, 0.0)

def test_pose_nan_inf():
    with pytest.raises(ValueError, match="finite number"):
        JointPose(joints=[0.0, math.nan, 0.0])
    with pytest.raises(ValueError, match="finite number"):
        TCPPose(x=0.0, y=math.inf, z=0.0, rx=0.0, ry=0.0, rz=0.0)

def test_cli_json_roundtrip():
    import sys
    import os
    env = os.environ.copy()
    env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1])
    
    result = subprocess.run(
        [sys.executable, "-m", "robot_integration.probe_cli", "--dry-run"],
        env=env,
        capture_output=True,
        text=True
    )
    assert result.returncode == 0
    
    data = json.loads(result.stdout)
    assert "timestamp_ms" in data
    assert data["status"]["state"] == "servo_off"
    assert data["status"]["in_workspace"] == True
    
    assert data["joint_pose"]["unit"] == "radian"
    assert len(data["joint_pose"]["joints"]) == 6
    
    assert data["tcp_pose"]["translation_unit"] == "mm"
    assert data["tcp_pose"]["rotation_unit"] == "radian"
    assert data["tcp_pose"]["frame_id"] == "base"
    assert "x" in data["tcp_pose"]
