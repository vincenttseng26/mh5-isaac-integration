#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scp "$SCRIPT_DIR/moveit_plan_gateway_ros2.py" cyc639@192.168.50.10:/tmp/
ssh cyc639@192.168.50.10 'pkill -f "[m]oveit_plan_gateway_ros2.py" || true'
sleep 1
ssh cyc639@192.168.50.10 'nohup bash -lc "source /opt/ros/humble/setup.bash; source /home/cyc639/ros2_ws/motoman_mh5_grasping_project/install/setup.bash; export ROS_DOMAIN_ID=17 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; exec python3 /tmp/moveit_plan_gateway_ros2.py" >/tmp/mh5_moveit_plan_gateway.log 2>&1 </dev/null &'
echo '[SAFE] MoveIt plan-only gateway started; it has no execution publisher.'
