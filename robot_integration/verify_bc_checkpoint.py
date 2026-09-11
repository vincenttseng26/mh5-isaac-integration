#!/usr/bin/env python3
"""Offline-only contract verification for the MH5 recurrent BC checkpoint."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch
from torch import nn


class RecurrentPolicy(nn.Module):
    def __init__(self, observation_dim, action_dim, hidden_dim):
        super().__init__()
        self.encoder = nn.Sequential(nn.Linear(observation_dim, hidden_dim), nn.SiLU())
        self.gru = nn.GRU(hidden_dim, hidden_dim, batch_first=True)
        self.head = nn.Linear(hidden_dim, action_dim)

    def forward(self, observation, hidden=None):
        encoded = self.encoder(observation)
        recurrent, hidden = self.gru(encoded, hidden)
        return self.head(recurrent), hidden


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--steps', type=int, default=16)
    args = parser.parse_args()
    checkpoint = torch.load(args.checkpoint, map_location='cpu', weights_only=True)
    required = {'policy_type','observation_dim','action_dim','hidden_dim','observation_mean',
                'observation_std','action_mean','action_std','model_state_dict',
                'dataset_sha256','output_encoding','target_mode'}
    missing = sorted(required - set(checkpoint))
    if missing: raise SystemExit('checkpoint fields missing: '+','.join(missing))
    if checkpoint['policy_type'] != 'mh5_behavior_cloning_gru':
        raise SystemExit('unexpected policy type')
    if checkpoint['output_encoding'] != 'standardized_action' or checkpoint['target_mode'] != 'incremental':
        raise SystemExit('checkpoint is not the accepted incremental-action encoding')
    dataset_hash = sha256(args.dataset)
    if dataset_hash != checkpoint['dataset_sha256']:
        raise SystemExit('dataset hash does not match checkpoint')
    data = np.load(args.dataset, allow_pickle=False)
    observations = data['observations']; episode_ids = data['episode_ids']
    if observations.shape[1:] != (27,): raise SystemExit('dataset observation contract is not 27-D')
    first_episode = int(episode_ids[0]); indices = np.flatnonzero(episode_ids == first_episode)[:args.steps]
    sequence = torch.from_numpy(observations[indices].astype(np.float32)).unsqueeze(0)
    mean = checkpoint['observation_mean'].float(); std = checkpoint['observation_std'].float()
    if tuple(mean.shape)!=(27,) or tuple(std.shape)!=(27,) or not torch.all(std>0):
        raise SystemExit('invalid observation normalization tensors')
    normalized = (sequence - mean) / std
    policy = RecurrentPolicy(int(checkpoint['observation_dim']),int(checkpoint['action_dim']),
                             int(checkpoint['hidden_dim']))
    policy.load_state_dict(checkpoint['model_state_dict'],strict=True); policy.eval()
    with torch.inference_mode(): standardized,_ = policy(normalized)
    action = standardized * checkpoint['action_std'].float() + checkpoint['action_mean'].float()
    action = action.clamp(-1.0,1.0)
    if not torch.isfinite(action).all(): raise SystemExit('policy produced non-finite action')
    final_action = action[0,-1].tolist()
    arm_delta = [float(value)*0.025 for value in final_action[:6]]
    report = {
        'ok': True, 'execution_allowed': False, 'command_authority': 'none',
        'checkpoint': str(args.checkpoint.resolve()), 'checkpoint_sha256': sha256(args.checkpoint),
        'dataset': str(args.dataset.resolve()), 'dataset_sha256': dataset_hash,
        'policy_type': checkpoint['policy_type'], 'target_mode': checkpoint['target_mode'],
        'observation_dimension': int(checkpoint['observation_dim']),
        'action_dimension': int(checkpoint['action_dim']), 'sequence_steps': int(len(indices)),
        'maximum_abs_normalized_observation': float(normalized.abs().max()),
        'maximum_abs_action': float(action.abs().max()),
        'last_action_7': final_action, 'last_arm_delta_rad_at_scale_0_025': arm_delta,
        'finite': all(math.isfinite(v) for v in final_action), 'real_commands_sent': 0,
    }
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2,sort_keys=True))


if __name__ == '__main__': main()
