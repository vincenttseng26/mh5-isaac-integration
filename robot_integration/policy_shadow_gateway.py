#!/usr/bin/env python3
"""Validate policy joint deltas against live MH5 state without executing them."""
import argparse
import json
import math
import socket
import time
from pathlib import Path

JOINTS = ('joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t')
STATE_GATEWAY = ('192.168.50.10', 8769)
PLAN_GATEWAY = ('192.168.50.10', 8770)
MAX_STEP_RAD = 0.01
MAX_FROM_PHYSICAL_RAD = 0.035


def request(endpoint, payload, timeout=4.0):
    with socket.create_connection(endpoint, timeout=timeout) as connection:
        connection.sendall((json.dumps(payload, separators=(',', ':')) + '\n').encode())
        line = connection.makefile('r').readline()
        if not line:
            raise ConnectionError(f'{endpoint} closed without a reply')
        return json.loads(line)


def validate_delta(value):
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError('delta_rad must contain exactly six values')
    delta = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in delta):
        raise ValueError('delta_rad contains a non-finite value')
    if max(abs(v) for v in delta) > MAX_STEP_RAD:
        raise ValueError(f'policy step exceeds {MAX_STEP_RAD} rad')
    return delta


def run(actions_path, log_path):
    state = request(STATE_GATEWAY, {'cmd': 'status'})
    if not state.get('ok') or len(state.get('positions') or ()) != 6:
        raise RuntimeError('fresh physical joint feedback unavailable')
    if state.get('mode') != 'MIRROR' or state.get('armed'):
        raise RuntimeError('physical gateway must be MIRROR / DISARMED')
    physical = tuple(float(v) for v in state['positions'])
    shadow = physical
    accepted = rejected = 0
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with actions_path.open(encoding='utf-8') as source, log_path.open('w', encoding='utf-8') as output:
        for index, line in enumerate(source):
            if not line.strip():
                continue
            timestamp = time.time()
            record = {
                'mode': 'shadow', 'index': index, 'timestamp': timestamp,
                'joint_names': JOINTS, 'physical_start_rad': physical,
                'command_authority': 'none', 'real_transport_touched': False,
            }
            try:
                action = json.loads(line)
                delta = validate_delta(action.get('delta_rad'))
                candidate = tuple(a + b for a, b in zip(shadow, delta))
                if max(abs(a - b) for a, b in zip(candidate, physical)) > MAX_FROM_PHYSICAL_RAD:
                    raise ValueError(f'shadow target exceeds {MAX_FROM_PHYSICAL_RAD} rad from physical pose')
                if max(abs(a - b) for a, b in zip(candidate, physical)) < 1e-7:
                    plan = {'ok': True, 'planner': 'validated-hold', 'points': [candidate],
                            'planning_time_s': 0.0}
                else:
                    plan = request(PLAN_GATEWAY, {'target_rad': candidate}, timeout=6.0)
                    if not plan.get('ok'):
                        raise RuntimeError(str(plan.get('error', 'MoveIt rejected target')))
                shadow = candidate
                record.update(delta_rad=delta, shadow_target_rad=shadow, accepted=True,
                              planner=plan.get('planner'), plan_points=len(plan.get('points', ())),
                              planning_time_s=plan.get('planning_time_s'))
                accepted += 1
            except Exception as exc:
                record.update(accepted=False, rejection=str(exc))
                rejected += 1
            output.write(json.dumps(record, ensure_ascii=False, separators=(',', ':')) + '\n')
            output.flush()
    final_state = request(STATE_GATEWAY, {'cmd': 'status'})
    final = tuple(float(v) for v in final_state.get('positions') or ())
    physical_change = max((abs(a - b) for a, b in zip(final, physical)), default=float('inf'))
    summary = {
        'ok': True, 'mode': 'shadow', 'accepted': accepted, 'rejected': rejected,
        'physical_max_change_rad': physical_change,
        'real_commands_sent': 0, 'log': str(log_path),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--actions', type=Path, required=True)
    parser.add_argument('--log', type=Path, required=True)
    args = parser.parse_args()
    run(args.actions, args.log)


if __name__ == '__main__':
    main()
