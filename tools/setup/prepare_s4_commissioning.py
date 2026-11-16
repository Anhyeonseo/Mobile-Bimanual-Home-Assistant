#!/usr/bin/env python3
"""Export review geometry and navigation TF inputs; never connect or enable hardware."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.s4_base_preparation import prepare
from home_robot_tasks.s4_sensor_frames import load_sensor_frames


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    base=prepare(ROOT)
    frames=load_sensor_frames(ROOT,navigation_root=True)
    a.output.mkdir(parents=True,exist_ok=False)
    for name,value in [('base.nominal.json',base),('navigation_sensor_frames.json',frames)]:
        (a.output/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(a.output),hardware_enabled=False,
        pending=base['pending'],odometry_source='must be measured wheel feedback; no synthetic transform exported')))


if __name__=='__main__':main()
