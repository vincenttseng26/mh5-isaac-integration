import os
import json
import tempfile
import pytest
from unittest.mock import MagicMock

from robot_integration.fs100_transport import FS100Transport, AbstractVendorClient
from robot_integration.interfaces import RobotState, TCPPose, JointPose

@pytest.fixture
def valid_config_file():
    config = {
        "enabled": True,
        "read_only": True,
        "network": {
            "ip_address": "10.0.0.2",
            "port": 11000
        },
        "options": {
            "protocol": "unconfirmed"
        }
    }
    fd, path = tempfile.mkstemp(suffix=".json")
    with os.fdopen(fd, 'w') as f:
        json.dump(config, f)
    yield path
    os.remove(path)

@pytest.fixture
def disabled_config_file():
    config = {
        "enabled": False,
        "read_only": True,
        "network": {
            "ip_address": "10.0.0.2",
            "port": 11000
        }
    }
    fd, path = tempfile.mkstemp(suffix=".json")
    with os.fdopen(fd, 'w') as f:
        json.dump(config, f)
    yield path
    os.remove(path)
    
@pytest.fixture
def missing_ip_config_file():
    config = {
        "enabled": True,
        "read_only": True,
        "network": {
            "ip_address": "",
            "port": 11000
        }
    }
    fd, path = tempfile.mkstemp(suffix=".json")
    with os.fdopen(fd, 'w') as f:
        json.dump(config, f)
    yield path
    os.remove(path)

@pytest.fixture
def mock_vendor_client():
    client = MagicMock(spec=AbstractVendorClient)
    client.connect.return_value = True
    client.read_status_data.return_value = {
        "state": "servo_off",
        "error_code": None,
        "in_workspace": True
    }
    client.read_joint_data.return_value = {
        "unit": "radian",
        "joints": [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    }
    client.read_tcp_data.return_value = {
        "unit": "mm",
        "frame_id": "base",
        "rotation_representation": "euler_xyz",
        "tool_id": "tool0",
        "user_frame_id": "user0",
        "x": 100.0, "y": 200.0, "z": 300.0,
        "rx": 0.1, "ry": 0.2, "rz": 0.3
    }
    return client

def test_load_valid_config(valid_config_file, mock_vendor_client):
    transport = FS100Transport(valid_config_file, mock_vendor_client)
    assert transport.config["enabled"] is True
    assert transport.config["network"]["ip_address"] == "10.0.0.2"

def test_load_disabled_config(disabled_config_file, mock_vendor_client):
    transport = FS100Transport(disabled_config_file, mock_vendor_client)
    assert transport.config["enabled"] is False

def test_missing_ip_fails_closed(missing_ip_config_file, mock_vendor_client):
    transport = FS100Transport(missing_ip_config_file, mock_vendor_client)
    # The config loading should fail closed
    assert transport.config["enabled"] is False

def test_connect_disabled_raises(disabled_config_file, mock_vendor_client):
    transport = FS100Transport(disabled_config_file, mock_vendor_client)
    with pytest.raises(RuntimeError, match="disabled"):
        transport.connect()

def test_successful_connection_and_read(valid_config_file, mock_vendor_client):
    transport = FS100Transport(valid_config_file, mock_vendor_client)
    transport.connect()
    mock_vendor_client.connect.assert_called_once_with("10.0.0.2", 11000)
    
    status = transport.read_status()
    assert status.state == RobotState.SERVO_OFF
    
    joint_pose = transport.read_joint_pose()
    assert joint_pose.joints == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    
    tcp_pose = transport.read_tcp_pose()
    assert tcp_pose.x == 100.0
    assert tcp_pose.z == 300.0
    
    transport.disconnect()
    mock_vendor_client.disconnect.assert_called_once()

def test_motion_commands_forbidden(valid_config_file, mock_vendor_client):
    transport = FS100Transport(valid_config_file, mock_vendor_client)
    transport.connect()
    
    dummy_j = JointPose([0.0]*6)
    dummy_tcp = TCPPose(0, 0, 0, 0, 0, 0)
    
    with pytest.raises(NotImplementedError, match="STRICTLY FORBIDDEN"):
        transport.move_j(dummy_j, 10.0)
        
    with pytest.raises(NotImplementedError, match="STRICTLY FORBIDDEN"):
        transport.move_l(dummy_tcp, 10.0)

def test_missing_joint_metadata_fails(valid_config_file, mock_vendor_client):
    transport = FS100Transport(valid_config_file, mock_vendor_client)
    transport.connect()
    
    mock_vendor_client.read_joint_data.return_value = {
        "joints": [0.0]*6
    }
    with pytest.raises(ValueError, match="Missing, unknown, or unsupported"):
        transport.read_joint_pose()
        
    mock_vendor_client.read_joint_data.return_value = {
        "unit": "pulse",
        "joints": [0.0]*6
    }
    with pytest.raises(ValueError, match="Missing, unknown, or unsupported"):
        transport.read_joint_pose()

def test_missing_tcp_metadata_fails(valid_config_file, mock_vendor_client):
    transport = FS100Transport(valid_config_file, mock_vendor_client)
    transport.connect()
    
    # Missing unit
    mock_vendor_client.read_tcp_data.return_value = {
        "frame_id": "base",
        "rotation_representation": "euler_xyz",
        "tool_id": "0", "user_frame_id": "0",
        "x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0
    }
    with pytest.raises(ValueError, match="Missing, unknown or unsupported"):
        transport.read_tcp_pose()
        
    # Unconfirmed Euler
    mock_vendor_client.read_tcp_data.return_value = {
        "unit": "mm",
        "frame_id": "base",
        "rotation_representation": "unconfirmed_euler",
        "tool_id": "0", "user_frame_id": "0",
        "x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0
    }
    with pytest.raises(ValueError, match="unconfirmed rotation representation"):
        transport.read_tcp_pose()
        
    # Missing tool_id
    mock_vendor_client.read_tcp_data.return_value = {
        "unit": "mm",
        "frame_id": "base",
        "rotation_representation": "euler_xyz",
        "user_frame_id": "0",
        "x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0
    }
    with pytest.raises(ValueError, match="Missing tool_id or user_frame_id"):
        transport.read_tcp_pose()
