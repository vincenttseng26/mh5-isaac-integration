#!/usr/bin/env bash
set -euo pipefail

# Simulation-only launcher. No ROS/TCP/MoveIt interfaces are started.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCENE="${SCRIPT_DIR}/scenes/mh5_training_workcell.usda"

if [[ -n "${ISAAC_SIM_PYTHON:-}" ]]; then
  ISAAC_PYTHON="${ISAAC_SIM_PYTHON}"
elif command -v isaacsim >/dev/null 2>&1; then
  exec isaacsim "${SCENE}" "$@"
elif [[ -x "${HOME}/isaacsim/python.sh" ]]; then
  ISAAC_PYTHON="${HOME}/isaacsim/python.sh"
else
  echo "Isaac Sim was not found. Set ISAAC_SIM_PYTHON to python.sh or install isaacsim." >&2
  exit 1
fi

exec "${ISAAC_PYTHON}" -c 'from isaacsim import SimulationApp; import sys
app=SimulationApp({"headless": False})
from omni.usd import get_context
get_context().open_stage(sys.argv[1])
while app.is_running():
    app.update()
app.close()' "${SCENE}" "$@"
