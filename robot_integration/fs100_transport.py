import json
import logging
from typing import Optional, Dict, Any
from .interfaces import RobotInterface, RobotStatus, JointPose, TCPPose, RobotState

logger = logging.getLogger(__name__)

class AbstractVendorClient:
    """
    Abstract mock for the vendor-specific communication protocol (e.g. MotoCom/HSE).
    We do NOT guess the packet bytes here. An actual implementation would use 
    verified byte structures from an official manual.
    """
    def connect(self, ip: str, port: int) -> bool:
        raise NotImplementedError()
        
    def disconnect(self) -> None:
        raise NotImplementedError()
        
    def read_status_data(self) -> Dict[str, Any]:
        raise NotImplementedError()
        
    def read_joint_data(self) -> Dict[str, Any]:
        raise NotImplementedError()
        
    def read_tcp_data(self) -> Dict[str, Any]:
        raise NotImplementedError()


class FS100Transport(RobotInterface):
    def __init__(self, config_path: str, vendor_client: AbstractVendorClient):
        self.config_path = config_path
        self.vendor_client = vendor_client
        self.config = self._load_config()
        self._connected = False

    def _load_config(self) -> dict:
        try:
            with open(self.config_path, 'r') as f:
                config = json.load(f)
            
            if not config.get("enabled", False):
                raise ValueError("FS100 Transport is disabled in config.")
                
            if not config.get("read_only", True):
                raise ValueError("FS100 Transport must be in read_only mode for safety.")
                
            ip = config.get("network", {}).get("ip_address")
            port = config.get("network", {}).get("port")
            
            if not ip or not port:
                raise ValueError("IP address and port must be explicitly filled in the configuration.")
                
            return config
        except Exception as e:
            logger.error(f"Failed to load FS100 configuration: {e}")
            # Fail closed by returning a safely disabled config
            return {"enabled": False, "read_only": True}

    def connect(self):
        if not self.config.get("enabled", False):
            raise RuntimeError("Cannot connect: FS100 Transport is disabled.")
            
        ip = self.config["network"]["ip_address"]
        port = self.config["network"]["port"]
        
        try:
            self._connected = self.vendor_client.connect(ip, port)
        except Exception as e:
            logger.error(f"Vendor client connection failed: {e}")
            self._connected = False
            raise

    def disconnect(self):
        if self._connected:
            self.vendor_client.disconnect()
            self._connected = False

    def read_status(self) -> RobotStatus:
        if not self._connected:
            return RobotStatus(state=RobotState.DISCONNECTED)
            
        try:
            raw_data = self.vendor_client.read_status_data()
            # In a real implementation, we would parse the raw status bits.
            # Here we map the abstract response.
            state = RobotState(raw_data.get("state", "disconnected"))
            error_code = raw_data.get("error_code")
            in_workspace = raw_data.get("in_workspace", False)
            
            return RobotStatus(state=state, error_code=error_code, in_workspace=in_workspace)
        except Exception as e:
            logger.error(f"Failed to read status: {e}")
            return RobotStatus(state=RobotState.DISCONNECTED)

    def read_joint_pose(self) -> JointPose:
        if not self._connected:
            raise RuntimeError("Cannot read joint pose: Not connected")
            
        try:
            data = self.vendor_client.read_joint_data()
            unit = data.get("unit")
            if not unit or unit in ("pulse", "unknown") or unit != "radian":
                raise ValueError(f"Missing, unknown, or unsupported joint unit: {unit}")
                
            return JointPose(joints=data["joints"])
        except Exception as e:
            logger.error(f"Failed to read joint pose: {e}")
            raise

    def read_tcp_pose(self) -> TCPPose:
        if not self._connected:
            raise RuntimeError("Cannot read TCP pose: Not connected")
            
        try:
            data = self.vendor_client.read_tcp_data()
            unit = data.get("unit")
            frame_id = data.get("frame_id")
            rot_rep = data.get("rotation_representation")
            
            if not unit or unit in ("pulse", "unknown") or unit != "mm":
                raise ValueError(f"Missing, unknown or unsupported TCP unit: {unit}")
            if not frame_id or frame_id == "unknown":
                raise ValueError(f"Missing or unknown TCP frame_id: {frame_id}")
            if not rot_rep or rot_rep == "unknown" or "unconfirmed" in rot_rep:
                raise ValueError(f"Missing, unknown or unconfirmed rotation representation: {rot_rep}")
            if "tool_id" not in data or "user_frame_id" not in data:
                raise ValueError("Missing tool_id or user_frame_id metadata")
                
            return TCPPose(
                x=data["x"], y=data["y"], z=data["z"],
                rx=data["rx"], ry=data["ry"], rz=data["rz"],
                frame_id=frame_id
            )
        except Exception as e:
            logger.error(f"Failed to read TCP pose: {e}")
            raise

    def move_j(self, pose: JointPose, speed: float) -> bool:
        raise NotImplementedError("WRITE/MOTION COMMANDS ARE STRICTLY FORBIDDEN IN FS100 TRANSPORT.")

    def move_l(self, pose: TCPPose, speed: float) -> bool:
        raise NotImplementedError("WRITE/MOTION COMMANDS ARE STRICTLY FORBIDDEN IN FS100 TRANSPORT.")
