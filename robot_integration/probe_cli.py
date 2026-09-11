import json
import time
import argparse
import sys
from .mock_transport import MockRobotTransport

def probe():
    parser = argparse.ArgumentParser(description="Read-only pure mock probe for Robot State")
    parser.add_argument("--dry-run", action="store_true", help="Ensure no motion is executed (read-only mode)")
    args = parser.parse_args()

    robot = MockRobotTransport()
    robot.connect()
    robot.set_servo(False)  # Ensure servo is off for safety by default in probe
    
    status = robot.read_status()
    joint_pose = robot.read_joint_pose()
    tcp_pose = robot.read_tcp_pose()
    
    output = {
        "timestamp_ms": int(time.time() * 1000),
        "status": {
            "state": status.state.value,
            "error_code": status.error_code,
            "in_workspace": status.in_workspace,
            "speed_fraction": status.speed_fraction
        },
        "joint_pose": {
            "joints": joint_pose.joints,
            "unit": "radian"
        },
        "tcp_pose": {
            "x": tcp_pose.x,
            "y": tcp_pose.y,
            "z": tcp_pose.z,
            "rx": tcp_pose.rx,
            "ry": tcp_pose.ry,
            "rz": tcp_pose.rz,
            "frame_id": tcp_pose.frame_id,
            "translation_unit": "mm",
            "rotation_unit": "radian",
            "rotation_representation": "euler_rx_ry_rz"
        }
    }
    
    print(json.dumps(output, indent=2))
    
if __name__ == "__main__":
    probe()
