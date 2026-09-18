#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scp "$SCRIPT_DIR/publish_workcell_scene_ros2.py" cyc639@192.168.50.10:/tmp/
ssh cyc639@192.168.50.10 'bash -lc "source /opt/ros/humble/setup.bash; source /home/cyc639/ros2_ws/motoman_mh5_grasping_project/install/setup.bash; export ROS_DOMAIN_ID=17 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; python3 /tmp/publish_workcell_scene_ros2.py"'
