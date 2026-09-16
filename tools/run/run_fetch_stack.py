#!/usr/bin/env python3
"""Run the complete production task assembly over explicitly synthetic devices."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.offline_stack import OfflineStack

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'config/fetch_stack.simulation.json')
    p.add_argument('--cancel-step',type=int,choices=range(15))
    p.add_argument('--fault',choices=('sensor_loss','reboot','load_loss'))
    p.add_argument('--output',type=Path)
    p.add_argument('--guarded-navigation',action='store_true',help='production travel guard with synthetic depth and wheel plant')
    args=p.parse_args()
    read=lambda n:json.loads((ROOT/'config'/n).read_text())
    stack=OfflineStack(json.loads(args.config.read_text()),read('home.example.json'),read('navigation_map.simulation.json'),guarded_navigation=args.guarded_navigation)
    request=dict(schema_version=1,operation='fetch_object',request_id='fetch-stack',request=read('fetch_remote.example.json'))
    result=stack.run(request,cancel_step=args.cancel_step,fault=args.fault)
    text=json.dumps(result,indent=2)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text+'\n')
    print(text)
    expected='CANCELLED' if args.cancel_step is not None else 'STOP_UNCONFIRMED' if args.fault in ('sensor_loss','reboot','load_loss') else 'SUCCEEDED'
    if result['status']!=expected:raise SystemExit(1)
if __name__=='__main__':main()
