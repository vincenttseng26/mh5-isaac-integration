#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

scp "$SCRIPT_DIR/gripper_gateway_core.py" \
    "$SCRIPT_DIR/rg2ft_command_gateway.py" \
    cyc639@192.168.50.10:/tmp/
ssh cyc639@192.168.50.10 '
pkill -f "^python3 .*[/]rg2ft_command_gateway.py" || true
sleep 1
setsid -f python3 -u /tmp/rg2ft_command_gateway.py --bind 192.168.50.10 \
  >/tmp/rg2ft_command_gateway.log 2>&1 </dev/null
sleep 1
ss -ltn | grep -q "192.168.50.10:8768" || {
  tail -n 40 /tmp/rg2ft_command_gateway.log >&2
  exit 1
}
'
echo '[SAFE] RG2-FT gateway restarted in MIRROR / DISARMED mode.'
