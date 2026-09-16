#!/usr/bin/env python3
"""Validate/hash explicit board settings. Never writes to a device."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.commissioning import export_profile

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(json.dumps(export_profile(json.loads(a.profile.read_text()),a.output),indent=2))
if __name__=='__main__':main()
