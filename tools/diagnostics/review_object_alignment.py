#!/usr/bin/env python3
"""Offline pre-GraspNet base/lift proposals from a segmented map cloud.

No devices, navigation actions or model downloads. --observation accepts JSON
with points_map_m, support_z_m, stamp, scene_revision, current_xy_yaw and
current_lift_m. Historical stamps are retained; replay time is explicitly the
observation time, never a renewed live observation. Output cannot execute.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.assistant.contracts import Context, Stamp
from home_robot_tasks.assistant.object_alignment import ObjectAlignmentPlanner
from tools.diagnostics.run_s3_stationary_lab import model


def fixtures():
    def face(length,width):
        x,y=np.meshgrid(np.linspace(-length/2,length/2,35),np.linspace(-width/2,width/2,21))
        return np.column_stack([x.ravel(),y.ravel(),np.full(x.size,.02)])
    angles=np.linspace(0,2*np.pi,96,endpoint=False)
    z=np.linspace(.005,.10,20)
    cylinder=np.column_stack([np.tile(.023*np.cos(angles),len(z)),
                              np.tile(.023*np.sin(angles),len(z)),np.repeat(z,len(angles))])
    return {'elongated':face(.16,.04),'round_upright':cylinder,
            'square':face(.045,.045),
            'wide_body_narrow_part':np.vstack([face(.14,.12),face(.08,.03)+[.105,0,0]])}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    inputs=p.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--observation',type=Path)
    inputs.add_argument('--synthetic-suite',action='store_true')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--arm',choices=['left','right'],default='left')
    a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    planner=ObjectAlignmentPlanner({a.arm:model(0,a.arm)})
    context=Context('alignment-lab','fixture-1',planner.arms[a.arm].profile['calibration_id'],'offline-clock')
    observations=[]
    if a.observation:
        raw=a.observation.read_bytes();d=json.loads(raw)
        observations=[('observation',d,hashlib.sha256(raw).hexdigest())]
    else:
        for label,cloud in fixtures().items():
            for degrees in (0,45,90):
                angle=np.deg2rad(degrees);c,s=np.cos(angle),np.sin(angle)
                rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
                for height in (.65,.8,.9):
                    name=f'{label}-{degrees}-{round(height*100)}'
                    d=dict(points_map_m=(cloud@rotation.T+[1.,.5,height]).tolist(),
                           support_z_m=height,stamp=Stamp(1.,context,name).to_dict(),
                           scene_revision='fixture-1',current_xy_yaw=[.5,.5,0.],current_lift_m=.4)
                    observations.append((name,d,None))
    summary=[]
    for name,d,source_hash in observations:
        stamp=Stamp.parse(d['stamp']);started=time.monotonic()
        r=planner.plan(points_map_m=d['points_map_m'],support_z_m=d['support_z_m'],
                       stamp=stamp,context=stamp.context,now_s=stamp.observed_s,
                       scene_revision=d['scene_revision'],current_xy_yaw=d['current_xy_yaw'],
                       current_lift_m=d['current_lift_m'])
        r.update(review_mode='synthetic_geometry' if a.synthetic_suite else 'historical_cloud_replay',
                 source_file_sha256=source_hash,historical_only=True,hardware_commands=0,
                 elapsed_s=time.monotonic()-started)
        (a.output/(name+'.json')).write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
        summary.append(dict(case=name,status=r['status'],candidates=len(r['candidates']),
                            evaluated=r['evaluated'],elapsed_s=r['elapsed_s'],
                            best=r['candidates'][0] if r['candidates'] else None))
    report=dict(scope='model geometry only; no navigation, camera, STL path or grasp success validation',
                hardware_commands=0,motion_authorized=False,cases=len(summary),
                cases_with_proposals=sum(bool(v['candidates']) for v in summary),results=summary)
    (a.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='results'}))


if __name__=='__main__':main()
