#!/usr/bin/env python3
"""Prepare one supported, stationary jaw test from a measured candidate. No device I/O."""
import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks')]
from home_robot_tasks.installed_jaw import InstalledLeftJaw
from home_robot_tasks.jaw_closure import closure_plan, target_gap
from home_robot_tasks.snapshot_io import FILES
from tools.diagnostics.run_s3_stationary_lab import stock_jaw


def prepare(replay, index, calibration, coordinates):
    group = replay['group']
    obs_index=group['observation_index']
    if (replay['mode'] != 'offline_grasp_replay' or replay['motion_authorized'] is not False
            or type(index) is not int or not 0 <= index < len(group['refined_grasps'])
            or type(obs_index) is not int or not 0 <= obs_index < len(replay['observations'])):
        raise ValueError('expected a selected candidate in a bound replay')
    obs=replay['observations'][obs_index]
    source=group['source_sha256']
    if (group['perception_status'] != 'accepted' or obs['status'] != 'accepted'
            or group['tcp_convention'] != 'graspnet_origin_plus_depth_to_pad_tip'
            or not replay['coordinate_frame'] == group['coordinate_frame'] == obs['frame_id'] == 'color_camera_optical_frame'
            or any(group[k] != replay[k] or group[k] != obs[k] for k in ('capture_id','label','source_sha256'))
            or group['mask_sha256'] != obs['mask_sha256'] or not isinstance(source,dict)
            or source.keys() != set(FILES)
            or any(not isinstance(h,str) or not re.fullmatch(r'[0-9a-f]{64}',h)
                   for h in (*source.values(),group['mask_sha256']))):
        raise ValueError('selected object or capture mismatch')
    candidate=group['refined_grasps'][index]
    jaw=InstalledLeftJaw(calibration,coordinates)
    closure=closure_plan(stock_jaw(),candidate)
    preview=jaw.preview(closure)
    diag=replay['candidate_diagnostics'][index]
    saved=dict(diag['jaw_closure'])
    if saved.get('status')=='BOUNDED_CLOSE':
        # Preserve already reviewed captures from before the explicit target field.
        saved['target_gap_m'] = target_gap(saved)
    matches=(closure.keys()==saved.keys() and all(
        math.isclose(value,saved[key],rel_tol=0.,abs_tol=1e-12) if type(value) is float else value==saved[key]
        for key,value in closure.items()))
    if (not matches or preview != diag['left_installed_pad_preview']
            or preview['status'] != 'MEASURED_GAP_PREVIEW'):
        raise ValueError('candidate closure or installed calibration mismatch')
    envelope=jaw.contact_envelope(closure)
    return dict(mode='supported_left_jaw_contact',arm='left',capture_id=replay['capture_id'],label=group['label'],
                candidate_index=index,source=candidate['source'],observation_index=obs_index,
                source_sha256=group['source_sha256'],mask_sha256=group['mask_sha256'],
                calibration_id=calibration['calibration_id'],calibration_section=calibration['section'],
                entry_raw=preview['entry_raw'],floor_raw=preview['lower_search_raw'],**envelope,
                visible_span_mm=1000*closure['visible_span_m'],entry_gap_mm=preview['entry_gap_mm'],
                target_gap_mm=1000*target_gap(closure),target_raw=preview['lower_search_raw'],
                floor_gap_mm=preview['lower_search_gap_mm'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('replay',type=Path);p.add_argument('--candidate',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    try:
        raw=a.replay.read_bytes()
        profile=(ROOT/'config/home_assistant/left_pad_gap.observed.json').read_bytes()
        result=prepare(json.loads(raw),a.candidate,json.loads(profile),
                       json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text()))
        result.update(replay_sha256=sha256(raw).hexdigest(),pad_profile_sha256=sha256(profile).hexdigest())
        with a.output.open('x') as out:json.dump(result,out,indent=2);out.write('\n')
    except (OSError,IndexError,ValueError,KeyError,TypeError) as exc:
        p.exit(2,str(exc)+'\n')
