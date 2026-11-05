#!/usr/bin/env python3
"""Build/evaluate/replay nominal S2 coordinates offline. No hardware commands."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks import s2_development as dev, s2_commissioning as s2


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    build=sub.add_parser('build')
    build.add_argument('--output',type=Path,required=True)
    for name in ('evaluate','export-urdf','replay'):
        cmd=sub.add_parser(name)
        cmd.add_argument('--profile',type=Path,required=True)
        cmd.add_argument('--output',type=Path,required=True)
        if name=='evaluate': cmd.add_argument('--state',type=Path,required=True)
        if name=='replay':
            cmd.add_argument('--evidence',type=Path,required=True)
            cmd.add_argument('--snapshots',type=Path,required=True,help='JSONL acquisition records; historical replay only')
    a=p.parse_args(argv)
    try:
        if a.output.exists(): raise ValueError('use a new output filename')
        g=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
        if a.command=='build': result=dev.build(ROOT)
        else:
            profile=json.loads(a.profile.read_text())
            if a.command=='export-urdf': result=dev.export_urdf(profile,g)
            elif a.command=='evaluate':
                state=json.loads(a.state.read_text())
                result=dev.evaluate(profile,g,state['raw'],state['lift_height_m'],camera_xyz_m=state.get('camera_xyz_m'))
            else:
                evidence=json.loads(a.evidence.read_text())
                session=dev.ObservationSession(profile,g,evidence)
                rows=[]
                with a.snapshots.open() as f:
                    for line in f:
                        if not line.strip(): continue
                        snap=json.loads(line)
                        rows.append(session.ingest(snap,now_s=snap['received_s'],clock_id=evidence['clock_id']))
                if not rows: raise ValueError('no snapshots')
                result={'mode':'historical_replay','live_hardware_verified':False,'motion_authorized':False,'observations':rows}
        a.output.parent.mkdir(parents=True,exist_ok=True)
        with a.output.open('x') as f:
            f.write(result if isinstance(result,str) else json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
        print(a.output)
        return 0
    except (OSError,ValueError,KeyError,TypeError) as e:
        print('S2 development: '+str(e),file=sys.stderr)
        return 2

if __name__=='__main__': raise SystemExit(main())
