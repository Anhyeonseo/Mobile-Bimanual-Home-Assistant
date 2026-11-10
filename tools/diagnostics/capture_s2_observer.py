#!/usr/bin/env python3
"""Read both arms and ID10 lift. No motor writes; optional explicitly anchored FK."""
from contextlib import nullcontext
import argparse
import json
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks import s2_acquisition as acq, s2_development as dev, s2_geometry as s2
from home_robot_tasks.s2_observation import start_session, attach_coordinates
from home_robot_tasks.snapshot_io import save_frame
from home_robot_tasks.camera_evidence import near_depth_profile
from so101_arm_bridge.observer_protocol import Reader


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=5)
    p.add_argument('--rgbd',action='store_true',help='also verify D415 acquisition identity and timestamp association')
    p.add_argument('--near-field',action='store_true',help='use the verified 640x360 native depth profile')
    anchors=p.add_mutually_exclusive_group()
    anchors.add_argument('--home-and-bottom-confirmed',action='store_true')
    anchors.add_argument('--current-pose-and-bottom-confirmed',action='store_true',help='read-only turn resolution from recorded ranges and current EEPROM; no Home assumption')
    p.add_argument('--snapshot-root',type=Path,help='export one matched RGB/aligned-depth exposure after successful anchored capture')
    a=p.parse_args(argv)
    if a.near_field and not a.rgbd:p.error('--near-field requires --rgbd')
    if a.snapshot_root and not (a.rgbd and (a.home_and_bottom_confirmed or a.current_pose_and_bottom_confirmed)):p.error('snapshot export requires RGB-D and an explicit arm/bottom reference')
    if not 0<a.seconds<=10:p.error('seconds must be >0 and <=10')
    if a.snapshot_root:
        try:from PIL import Image  # validate the optional PNG encoder before opening devices
        except ImportError:p.error('Pillow required for snapshot export')
    if a.output.exists():p.error('output exists; use a new filename')
    report={'schema_version':1,'status':'failed','motor_write_commands':0,'motion_authorized':False,
            'camera_synchronized':False,'records':[],'origin_valid_after_close':False}
    stage='diagnostics.dependencies'
    try:
        import serial
        from so101_arm_bridge.backend_lease import acquire_backend_lease
        from so101_arm_bridge.serial_port import open_exclusive_serial
        stage='L3A.profile'
        g=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
        profile=json.loads(a.profile.read_text());dev.validate(profile,g)
        from home_robot_tasks.s2_rgbd_acquisition import Capture
        reference=json.loads((ROOT/'config/home_assistant/camera_device.observed.json').read_text()) if a.rgbd else None
        if a.near_field:
            near=json.loads((ROOT/'config/home_assistant/camera_device.near.observed.json').read_text())
            profile=near_depth_profile(profile,reference,near)
            dev.validate(profile,g)
            reference=near
        stage='L3A.rgbd_stream'
        with Capture(profile,reference,retain_frame=a.snapshot_root is not None) if a.rgbd else nullcontext() as camera:
            report=capture(a,report,g,profile,camera,serial,acquire_backend_lease,open_exclusive_serial)
    except (ImportError,OSError,ValueError,KeyError,TypeError,RuntimeError) as e:
        if report.get('error'):
            report['cleanup_error']=str(e)
        else:
            report.update(status='failed',error=str(e),failure_stage=stage)
    return save(a,report)


def capture(a,report,g,profile,camera,serial,acquire_backend_lease,open_exclusive_serial):
    stage="L2.open"
    reader=None
    retained=None
    try:
        with acquire_backend_lease('stm32',0),open_exclusive_serial(serial,a.port,921600,.05) as port:
            port.reset_input_buffer()
            stage='L2.observer'
            reader=Reader(port);report['binding']=reader.bind()
            current_pose=getattr(a,'current_pose_and_bottom_confirmed',False)
            calibration=reader.sample(arm_calibration_only=True) if current_pose else None
            if calibration is not None:report['arm_calibration']=calibration
            first=reader.sample();report['records'].append(first)
            bottom_samples=None
            if current_pose:
                bottom_samples=[first]
                for _ in range(4):
                    time.sleep(.01)
                    first=reader.sample();report['records'].append(first);bottom_samples.append(first)
            stage='L2.encoder_anchor'
            tracker=acq.EncoderTracker(profile,first,home_confirmed=a.home_and_bottom_confirmed,bottom_confirmed=True,
                range_reference=ROOT/'config/home_assistant/arm_ranges.development.json' if current_pose else None,
                calibration=calibration,bottom_samples=bottom_samples) if a.home_and_bottom_confirmed or current_pose else None
            if tracker:report['arm_anchor_basis']=tracker.anchor_evidence
            session=None
            if tracker and camera:
                stage='L3A.coordinate_reference'
                session,evidence=start_session(profile,g,tracker,first)
                report['reference_evidence']=evidence
            deadline=time.monotonic()+a.seconds
            while time.monotonic()<deadline:
                stage='L2.observer'
                row=reader.sample();report['records'].append(row)
                if camera:
                    stage='L3A.match_rgbd'
                    row['camera']=camera.match(row)
                    if len(report['records'])>2 and row['camera']['capture_id']==report['records'][-2].get('camera',{}).get('capture_id'):
                        raise ValueError('camera repeated; acquisition rejected')
                if tracker:
                    stage='L2.encoder_update'
                    state=tracker.update(row)
                    if session:
                        stage='L3A.coordinate_session'
                        attach_coordinates(session, state, row, time.monotonic)
                        if a.snapshot_root and retained is None:
                            retained=(camera.retain_matched(row['camera']),row)
                    else:
                        stage='L3A.coordinates'
                        row['coordinates']=dev.evaluate(profile,g,state['raw'],state['lift_height_m'])
                time.sleep(.01)
        if a.snapshot_root:
            stage='diagnostics.snapshot_export'
            if retained is None:raise ValueError('no anchored exposure to export')
            report['snapshot_export']=save_frame(a.snapshot_root,*retained)
        report['status']='motor_acquisition_verified'
        report['camera_synchronized']=camera is not None
        report['sample_count']=len(report['records'])
        report['maximum_acquisition_span_bound_s']=max(r['acquisition_span_bound_s'] for r in report['records'])
    except (OSError,ValueError,KeyError,TypeError,RuntimeError) as e:
        report['status']='failed';report['error']=str(e)
        report['failure_stage'] = f'L2.{reader.stage}' if stage=='L2.observer' and reader else stage
        if reader is not None:
            report['last_raw_exchange']=reader.last_exchange
            report['last_valid_sequence']=reader.last_sequence
    return report


def save(a,report):
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2,ensure_ascii=False))
    return 0 if report['status']=='motor_acquisition_verified' else 2

if __name__=='__main__':raise SystemExit(main())
