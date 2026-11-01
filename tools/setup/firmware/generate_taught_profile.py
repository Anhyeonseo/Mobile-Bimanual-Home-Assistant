#!/usr/bin/env python3
"""Produce an immutable, measured five-pose MCU profile; never authorize motion."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/run'))
from pose_locked_profile import compile_profile,gripper_tolerance_urad
from teach_left_demo import load

def header(p):
    rows=[]
    for direction,b in zip(p['directions'],p['bounds'],strict=True):
        rows.append('{2048U,%d,{%d,%d},{%d,%d}}'%(direction,b['minimum_raw'],b['maximum_raw'],b['minimum_urad'],b['maximum_urad']))
    points=',\n'.join('{'+','.join(str(x) for x in q)+'}' for q in p['targets_urad'])
    raw=',\n'.join('{'+','.join(str(x) for x in q)+'}' for q in p['targets_raw'])
    return '''/* Generated from measured capture; do not hand edit. */
#ifndef TAUGHT_POSE_PROFILE_H
#define TAUGHT_POSE_PROFILE_H
#include <stdint.h>
#define TAUGHT_PROFILE_SHA256 "%s"
#define TAUGHT_CALIBRATION_HASH UINT32_C(0x%08X)
#define TAUGHT_GRIPPER_DIRECTION %d
#define TAUGHT_GRIPPER_TOLERANCE_URAD %d
#define TAUGHT_LEFT_LIMITS {%s}
static const int32_t taught_targets[5][6]={%s};
static const uint16_t taught_raw[5][6]={%s};
static const uint16_t taught_offsets[6]={%s};
static const int32_t taught_minimum_urad[6]={%s};
static const int32_t taught_maximum_urad[6]={%s};
static const uint16_t taught_minimum_raw[6]={%s};
static const uint16_t taught_maximum_raw[6]={%s};
#endif
'''%(p['profile_sha256'],p['firmware_calibration_hash'],p['directions'][5],
      gripper_tolerance_urad(p),','.join(rows),points,raw,','.join(map(str,p['offsets_raw'])),
      *[','.join(str(b[k]) for b in p['bounds']) for k in ('minimum_urad','maximum_urad','minimum_raw','maximum_raw')])

def main():
    a=argparse.ArgumentParser();a.add_argument('--capture',type=Path,required=True);a.add_argument('--output',type=Path,required=True);args=a.parse_args()
    p=compile_profile(load(args.capture));args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'taught_pose_profile.h').write_text(header(p))
    (args.output/'profile.json').write_text(json.dumps(p,indent=2)+'\n')
    print(p['profile_sha256'])
if __name__=='__main__':main()
