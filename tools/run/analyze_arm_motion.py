#!/usr/bin/env python3
"""Analyze a captured 12-axis target/measured JSON trace without opening devices."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.motion_quality import analyze_trace

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',type=Path,required=True)
    parser.add_argument('--settling-tolerance-rad',type=float,default=.02)
    parser.add_argument('--hold-window-s',type=float,default=.5)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    result=analyze_trace(json.loads(args.trace.read_text()),settling_tolerance_rad=args.settling_tolerance_rad,minimum_hold_s=args.hold_window_s)
    text=json.dumps(result,indent=2,allow_nan=False)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text+'\n')
    print(text)
if __name__=='__main__':main()
