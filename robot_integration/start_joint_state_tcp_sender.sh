#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${project_root}/config/hardware.env"
"${project_root}/scripts/assert_read_only_ros1.sh"
container="${ROS1_CONTAINER}"
sender="${project_root}/scripts/joint_state_tcp_sender.py"

if [[ "$(docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null || true)" != true ]]; then
  echo "[FAIL] ${container} is not running; start read-only robot state first" >&2
  exit 1
fi
docker exec "${container}" pkill -f '[m]h5_joint_state_tcp_sender.py' 2>/dev/null || true
docker cp "${sender}" "${container}:/tmp/mh5_joint_state_tcp_sender.py"
docker exec --detach \
  --env "ROS_MASTER_URI=http://${ROS_HOST_IP}:11311" \
  --env "ROS_IP=${ROS_HOST_IP}" \
  "${container}" bash -lc \
  'source /opt/ros/noetic/setup.bash && exec python3 /tmp/mh5_joint_state_tcp_sender.py --host 192.168.50.20 --port 8766'
echo "[SAFE] one-way /joint_states sender started; destination 192.168.50.20:8766"
