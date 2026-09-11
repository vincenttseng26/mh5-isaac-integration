#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scp "$SCRIPT_DIR/joint_trajectory_tcp_receiver_ros1.py" cyc639@192.168.50.10:/tmp/
# The receiver imports the staged envelope, so both files must land together or
# the gateway will not start at all.
scp "$SCRIPT_DIR/staged_envelope.py" cyc639@192.168.50.10:/tmp/
ssh cyc639@192.168.50.10 '
docker cp /tmp/joint_trajectory_tcp_receiver_ros1.py ros1_motoman_a1_runtime:/tmp/
docker cp /tmp/staged_envelope.py ros1_motoman_a1_runtime:/tmp/
docker exec ros1_motoman_a1_runtime bash -lc "pkill -f [j]oint_trajectory_tcp_receiver_ros1.py || true"
sleep 2
docker exec -d -e ROS_MASTER_URI=http://192.168.0.107:11311 -e ROS_IP=192.168.0.107 \
  ros1_motoman_a1_runtime bash -lc "source /opt/ros/noetic/setup.bash; source /home/ros/ws/devel/setup.bash; exec python3 /tmp/joint_trajectory_tcp_receiver_ros1.py >/tmp/mh5_trajectory_gateway.log 2>&1"
'
echo '[SAFE] Gateway started in MIRROR mode and DISARMED.'
