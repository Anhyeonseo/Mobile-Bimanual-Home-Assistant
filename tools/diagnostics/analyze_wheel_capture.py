#!/usr/bin/env python3
"""Analyze an existing read-only wheel JSONL; does not open a serial port."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.s4_base_preparation import review_capture

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    result=review_capture([json.loads(line) for line in a.capture.read_text().splitlines() if line.strip()])
    with a.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result))

if __name__=='__main__':main()
