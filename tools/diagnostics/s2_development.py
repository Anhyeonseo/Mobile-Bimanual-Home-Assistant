#!/usr/bin/env python3
"""Build/evaluate/replay nominal S2 coordinates offline. No hardware commands."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks import s2_development as dev, s2_geometry as s2


def compare_recording(profile, geometry, recording):
    """Replay a successful closed session. Report where a historical result diverges."""
    from home_robot_tasks.s2_observation import attach_coordinates
    import math
    from home_robot_tasks.s2_acquisition import EncoderTracker
    from home_robot_tasks.s2_rgbd_acquisition import association

    def equivalent(left, right):
        if isinstance(left, dict) and isinstance(right, dict):
            return left.keys()==right.keys() and all(equivalent(left[k],right[k]) for k in left)
        if isinstance(left, list) and isinstance(right, list):
            return len(left)==len(right) and all(equivalent(a,b) for a,b in zip(left,right))
        if type(left) is float and type(right) in (int,float):
            return math.isclose(left,right,rel_tol=0,abs_tol=1e-10)
        return type(left) is type(right) and left==right

    if (recording['status'] != 'motor_acquisition_verified' or recording['camera_synchronized'] is not True
            or recording['motion_authorized'] is not False or recording['motor_write_commands'] != 0
            or recording['origin_valid_after_close'] is not False):
        raise ValueError('completed read-only integrated recording required')
    rows = recording['records']
    if len(rows)<2 or len(rows)!=recording['sample_count']:
        raise ValueError('incomplete integrated recording')
    evidence = recording['reference_evidence']
    first = rows[0]
    tracker = EncoderTracker(profile, first, home_confirmed=evidence['arm_turns_anchored'],
                             bottom_confirmed=evidence['lift_bottom_confirmed'])
    if (tracker.origin != evidence['lift_origin_raw_unwrapped'] or
            tracker.state()['raw'] != evidence['arm_raw_unwrapped'] or
            evidence['controller_boot_id'] != f"observer-epoch-{first['epoch']}"):
        raise ValueError('first sample does not match the recorded anchor')
    session = dev.ObservationSession(profile, geometry, evidence)
    for i,row in enumerate(rows[1:], start=1):
        stage = 'L2.encoder_update'
        try:
            state = tracker.update(row)
            stage = 'L3A.match_rgbd'
            camera = row['camera']
            if not equivalent(association(row,camera)['acquisition_span_s'], camera['acquisition_span_s']):
                raise ValueError('camera timing differs from record')
            stage = 'L3A.coordinate_session'
            rebuilt = dict(sequence=row['sequence'],camera=camera)
            # Use the original host receipt time; replay must never freshen evidence.
            attach_coordinates(session,state,rebuilt,lambda:row['snapshot']['received_s'])
            for key in ('snapshot','coordinates'):
                if not equivalent(rebuilt[key],row[key]):
                    raise ValueError(f'{key} differs from record')
        except (ValueError,KeyError,TypeError,IndexError,OverflowError) as e:
            raise ValueError(f"{stage} record {i}, sequence {row.get('sequence')}: {e}") from e
    return {'mode':'historical_capture_comparison','calibration_id':profile['calibration_id'],
            'motor_records':len(rows),'coordinates_matched':len(rows)-1,'absolute_tolerance':1e-10,
            'live_hardware_verified':False,'motion_authorized':False}


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    build=sub.add_parser('build')
    build.add_argument('--output',type=Path,required=True)
    for name in ('evaluate','export-urdf','replay','compare-recording'):
        cmd=sub.add_parser(name)
        cmd.add_argument('--profile',type=Path,required=True)
        cmd.add_argument('--output',type=Path,required=True)
        if name=='evaluate': cmd.add_argument('--state',type=Path,required=True)
        if name=='compare-recording': cmd.add_argument('--recording',type=Path,required=True)
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
            elif a.command=='compare-recording':
                result=compare_recording(profile,g,json.loads(a.recording.read_text()))
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
