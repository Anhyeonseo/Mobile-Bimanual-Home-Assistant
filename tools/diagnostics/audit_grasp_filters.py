#!/usr/bin/env python3
"""Compare generic-gripper rejection with observed SO101 proposals on a fixed capture manifest.

No scene is dropped from the denominator. This audits candidate generation,
not path feasibility or physical success. The manifest is an audit_pick_scene
report whose capture/detection paths and source hashes freeze the dataset.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks')]
from home_perception.grasp import graspnet
from home_robot_tasks.snapshot_io import hashes
from tools.diagnostics.replay_s3_grasp import load_scene, grounding_observations
from tools.diagnostics.run_s3_stationary_lab import stock_jaw


def run(manifest, output):
    source = json.loads(manifest.read_text())
    profile = json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())
    jaw = stock_jaw()
    report = dict(mode='fixed_dataset_candidate_filter_audit', manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
                  hardware_commands=0, motion_authorized=False, physical_grasp_success_measured=False,
                  route_feasibility_measured=False, held_out_dataset=False, captures=[])
    report['code_sha256'] = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in (
        'home_perception/grasp/graspnet.py', 'home_perception/grasp/proposals.py',
        'home_perception/grasp/handoff.py', 'home_perception/grasp/refine.py')}
    for capture in source['captures']:
        record = dict(snapshot=capture['snapshot'], instances=[])
        report['captures'].append(record)
        try:
            snapshot = Path(capture['snapshot'])
            expected = capture['source_sha256']
            if hashes(snapshot) != expected:
                raise ValueError('capture digest changed since manifest')
            meta, depth, scene, plane = load_scene(snapshot)
            clouds, planes = {}, {}
            observations, masks = grounding_observations(snapshot, Path(capture['detections']), meta, depth, plane,
                expected, clouds, support_planes=planes, up_in_optical=np.asarray(profile['base_from_optical'])[2,:3])
            for index, observation in enumerate(observations):
                item = dict(index=index, label=observation['label'], perception_status=observation['status'])
                record['instances'].append(item)
                if observation['status'] != 'accepted':
                    item.update(status='PERCEPTION_REOBSERVE', reason=observation['reason'])
                    continue
                group = graspnet.candidates(depth, meta['aligned_depth_intrinsics'], masks, index, scene,
                    planes[index], observation['support_height_m'], (jaw.gaps[0], jaw.gaps[-1]), section_points=clouds[index])
                item.update(status='PROPOSALS_READY' if group['refined_grasps'] else 'NO_OBSERVED_PROPOSAL', group=group)
                print(snapshot.name, item['label'], group['counts'], group['proposal_selection']['rejected_by_evidence'], flush=True)
        except (OSError, KeyError, ValueError, RuntimeError) as exc:
            record.update(status='AUDIT_ERROR', reason=str(exc))
        # Leave usable partial evidence if inference is interrupted.
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    items = [i for c in report['captures'] for i in c['instances']]
    evaluated = [i for i in items if 'group' in i]
    report['counts'] = dict(captures=len(report['captures']), instances=len(items), evaluated_instances=len(evaluated),
        captures_with_error=sum(c.get('status')=='AUDIT_ERROR' for c in report['captures']),
        generic_filter_with_candidates=sum(i['group']['legacy_parallel_jaw_audit']['refined']>0 for i in evaluated),
        observed_proposals_available=sum(bool(i['group']['refined_grasps']) for i in evaluated))
    output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report['counts']))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('choose a new output; preserve previous comparison evidence')
    run(args.manifest, args.output)
