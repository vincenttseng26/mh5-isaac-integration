from .interfaces import RobotInterface, RobotStatus, JointPose, TCPPose, RobotState

class MockRobotTransport(RobotInterface):
    def __init__(self):
        self._status = RobotStatus(state=RobotState.DISCONNECTED)
        self._joint_pose = JointPose(joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._tcp_pose = TCPPose(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0)

    def connect(self):
        self._status.state = RobotState.CONNECTED
        self._status.in_workspace = True

    def disconnect(self):
        self._status.state = RobotState.DISCONNECTED

    def set_e_stop(self, active: bool):
        if active:
            self._status.state = RobotState.E_STOP
        else:
            self._status.state = RobotState.CONNECTED

    def set_alarm(self, active: bool, code: str = ""):
        if active:
            self._status.state = RobotState.ALARM
            self._status.error_code = code
        else:
            self._status.state = RobotState.CONNECTED
            self._status.error_code = None

    def set_servo(self, on: bool):
        if self._status.state in [RobotState.E_STOP, RobotState.ALARM, RobotState.DISCONNECTED]:
            return # Cannot change servo state if in error or disconnected
        if on:
            self._status.state = RobotState.SERVO_ON
        else:
            self._status.state = RobotState.SERVO_OFF
            
    def set_in_workspace(self, in_workspace: bool):
        self._status.in_workspace = in_workspace

    def set_mock_poses(self, joint_pose: JointPose, tcp_pose: TCPPose):
        self._joint_pose = joint_pose
        self._tcp_pose = tcp_pose

    def read_status(self) -> RobotStatus:
        return self._status

    def read_joint_pose(self) -> JointPose:
        return self._joint_pose

    def read_tcp_pose(self) -> TCPPose:
        return self._tcp_pose

    def move_j(self, pose: JointPose, speed: float) -> bool:
        raise NotImplementedError("MOCK TRANSPORT: Execution of motion commands is strictly forbidden.")

    def move_l(self, pose: TCPPose, speed: float) -> bool:
        raise NotImplementedError("MOCK TRANSPORT: Execution of motion commands is strictly forbidden.")
