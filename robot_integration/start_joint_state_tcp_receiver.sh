#!/usr/bin/env bash
set -eo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
echo "[SAFE] TCP input 192.168.50.20:8766 -> local ROS 2 /real/joint_states"
echo "[SAFE] no socket or ROS command path to CYC/FS100"
exec python3 "${root}/robot_integration/joint_state_tcp_receiver.py"
