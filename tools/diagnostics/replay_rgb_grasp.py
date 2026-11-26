#!/usr/bin/env python3
"""Replay saved per-arm RGB segmentation/FK evidence, without camera or motors.

Manifest: {calibration: RGBGraspVerifier constructor kwargs, frames: [observe
kwargs with target_masks and occluded_mask replaced by relative .npy paths]}.
Use masks from the SAME rectified RGB exposure, never an old planning mask.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.rgb_grasp_verification import RGBGraspVerifier


def replay(manifest):
    path=Path(manifest)
    data=json.loads(path.read_text())
    verifier=RGBGraspVerifier(**data['calibration'])
    results=[]
    for frame in data['frames']:
        measurements=dict(frame)
        measurements['target_masks']=[np.load(path.parent/p,allow_pickle=False) for p in frame['target_masks']]
        name=frame['occluded_mask']
        measurements['occluded_mask']=None if name is None else np.load(path.parent/name,allow_pickle=False)
        results.append(asdict(verifier.observe(**measurements)))
    return dict(results=results,hardware_commands=0,physical_validation=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=replay(args.manifest)
    with args.output.open('x') as stream:
        json.dump(report,stream,ensure_ascii=False,indent=2)


if __name__=='__main__':
    main()
