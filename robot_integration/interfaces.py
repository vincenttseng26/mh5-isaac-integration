import abc
import enum
import math
import json
from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional

class RobotState(enum.Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    SERVO_ON = "servo_on"
    SERVO_OFF = "servo_off"
    ALARM = "alarm"
    E_STOP = "e_stop"

def check_finite(val: float, name: str):
    if not isinstance(val, (int, float)):
        raise ValueError(f"{name} must be a number, got {type(val)}")
    if math.isnan(val) or math.isinf(val):
        raise ValueError(f"{name} must be a finite number, got {val}")

@dataclass
class JointPose:
    """Joint positions in radians"""
    joints: List[float]

    def __post_init__(self):
        for i, j in enumerate(self.joints):
            check_finite(j, f"joint[{i}]")

@dataclass
class TCPPose:
    """TCP translation in mm, rotation (Euler rx, ry, rz) in radians"""
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    frame_id: str = "base"

    def __post_init__(self):
        check_finite(self.x, "x")
        check_finite(self.y, "y")
        check_finite(self.z, "z")
        check_finite(self.rx, "rx")
        check_finite(self.ry, "ry")
        check_finite(self.rz, "rz")
        if not isinstance(self.frame_id, str):
            raise ValueError(f"frame_id must be a string, got {type(self.frame_id)}")

@dataclass
class RobotStatus:
    state: RobotState
    error_code: Optional[str] = None
    in_workspace: bool = True
    speed_fraction: float = 0.0

class SafetyViolationError(Exception):
    pass

class RobotInterface(abc.ABC):
    @abc.abstractmethod
    def read_status(self) -> RobotStatus:
        pass

    @abc.abstractmethod
    def read_joint_pose(self) -> JointPose:
        pass

    @abc.abstractmethod
    def read_tcp_pose(self) -> TCPPose:
        pass

    def move_j(self, pose: JointPose, speed: float) -> bool:
        raise NotImplementedError("Motion commands are disabled by default for safety.")

    def move_l(self, pose: TCPPose, speed: float) -> bool:
        raise NotImplementedError("Motion commands are disabled by default for safety.")

class SafetyGate:
    def __init__(self, max_speed: float, workspace_limits: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]):
        self.max_speed = max_speed
        self.workspace_limits = workspace_limits # ((x_min, x_max), (y_min, y_max), (z_min, z_max))

    def check_safety(self, status: RobotStatus, target_tcp: Optional[TCPPose] = None, speed: float = 0.0) -> bool:
        if status.state == RobotState.DISCONNECTED:
            raise SafetyViolationError("Robot is disconnected.")
        if status.state == RobotState.E_STOP:
            raise SafetyViolationError("E-STOP is active.")
        if status.state == RobotState.ALARM:
            raise SafetyViolationError(f"Robot is in ALARM state. Error: {status.error_code}")
        if status.state != RobotState.SERVO_ON:
            raise SafetyViolationError(
                f"Motion safety gate requires an explicit SERVO_ON state; got {status.state.value}."
            )
        
        if speed < 0:
            raise SafetyViolationError(f"Speed {speed} cannot be negative.")
        if speed > self.max_speed:
            raise SafetyViolationError(f"Speed {speed} exceeds maximum allowed speed {self.max_speed}.")

        if target_tcp:
            x_lim, y_lim, z_lim = self.workspace_limits
            if not (x_lim[0] <= target_tcp.x <= x_lim[1] and
                    y_lim[0] <= target_tcp.y <= y_lim[1] and
                    z_lim[0] <= target_tcp.z <= z_lim[1]):
                raise SafetyViolationError("Target position is out of workspace limits.")
            
            # Simple orientation check: reject any roll/pitch/yaw > 2*pi for sanity, though finite check handles Inf/NaN.
            if abs(target_tcp.rx) > 2*math.pi or abs(target_tcp.ry) > 2*math.pi or abs(target_tcp.rz) > 2*math.pi:
                raise SafetyViolationError("Target orientation (Euler angles) exceeds reasonable bounds ([-2pi, 2pi]).")
            
        if not status.in_workspace:
             raise SafetyViolationError("Robot is currently out of workspace limits.")

        return True
