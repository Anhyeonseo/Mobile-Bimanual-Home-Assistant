#!/usr/bin/env python3
"""Read both arms and ID10 lift. No motor writes; optional explicitly anchored FK."""
from contextlib import nullcontext
import argparse
import json
from pathlib import Path
import sys
import time
import uuid
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks import s2_acquisition as acq, s2_development as dev, s2_commissioning as s2


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=5)
    p.add_argument('--rgbd',action='store_true',help='also verify D415 acquisition identity and timestamp association')
    p.add_argument('--home-and-bottom-confirmed',action='store_true')
    a=p.parse_args(argv)
    if not 0<a.seconds<=10:p.error('seconds must be >0 and <=10')
    if a.output.exists():p.error('output exists; use a new filename')
    report={'schema_version':1,'status':'failed','motor_write_commands':0,'motion_authorized':False,
            'camera_synchronized':False,'records':[],'origin_valid_after_close':False}
    try:
        import serial
        from so101_arm_bridge.backend_lease import acquire_backend_lease
        from so101_arm_bridge.serial_port import open_exclusive_serial
        g=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
        profile=json.loads(a.profile.read_text());dev.validate(profile,g)
        from home_robot_tasks.s2_rgbd_acquisition import Capture
        reference=json.loads((ROOT/'config/home_assistant/camera_device.observed.json').read_text()) if a.rgbd else None
        with Capture(profile,reference) if a.rgbd else nullcontext() as camera:
            report=capture(a,report,g,profile,camera,serial,acquire_backend_lease,open_exclusive_serial)
    except (OSError,ValueError,KeyError,TypeError,RuntimeError) as e:
        report['status']='failed';report['error']=str(e)
    return save(a,report)


def capture(a,report,g,profile,camera,serial,acquire_backend_lease,open_exclusive_serial):
    reader=None
    try:
        with acquire_backend_lease('stm32',0),open_exclusive_serial(serial,a.port,921600,.05) as port:
            port.reset_input_buffer()
            reader=acq.Reader(port);report['binding']=reader.bind()
            first=reader.sample();report['records'].append(first)
            tracker=acq.EncoderTracker(profile,first,home_confirmed=True,bottom_confirmed=True) if a.home_and_bottom_confirmed else None
            session=None
            if tracker and camera:
                identity={'session_id':str(uuid.uuid4()),'controller_boot_id':'observer-epoch-'+str(first['epoch']),
                          'origin_id':str(uuid.uuid4()),'arm_anchor_id':str(uuid.uuid4()),
                          'clock_id':'host-monotonic-'+str(uuid.uuid4())}
                evidence={**identity,'calibration_id':profile['calibration_id'],'lift_bottom_confirmed':True,
                          'arm_turns_anchored':True,'lift_origin_raw_unwrapped':tracker.origin,
                          'arm_raw_unwrapped':tracker.state()['raw'],
                          'established_s':max(r['acquired_s'] for r in first['samples'])}
                session=dev.ObservationSession(profile,g,evidence);report['reference_evidence']=evidence
            deadline=time.monotonic()+a.seconds
            while time.monotonic()<deadline:
                row=reader.sample();report['records'].append(row)
                if camera:
                    row['camera']=camera.match(row)
                    if len(report['records'])>2 and row['camera']['capture_id']==report['records'][-2].get('camera',{}).get('capture_id'):
                        raise ValueError('camera repeated; acquisition rejected')
                if tracker:
                    state=tracker.update(row)
                    if session:
                        snap={**identity,**{k:profile[k] for k in ('calibration_id','assembly_revision','camera_device','source_frame')},
                              **{k:state[k] for k in ('raw','joint_times_s','lift_raw_unwrapped','lift_time_s')},
                              'sequence':row['sequence'],'capture_id':row['camera']['capture_id'],
                              'camera_time_s':row['camera']['color_time_s'],'received_s':time.monotonic(),
                              'raw_encoding':'continuous_unwrapped_counts'}
                        row['snapshot']=snap
                        row['coordinates']=session.ingest(snap,now_s=time.monotonic(),clock_id=identity['clock_id'])
                    else:
                        row['coordinates']=dev.evaluate(profile,g,state['raw'],state['lift_height_m'])
                time.sleep(.01)
        report['status']='motor_acquisition_verified'
        report['camera_synchronized']=camera is not None
        report['sample_count']=len(report['records'])
        report['maximum_acquisition_span_bound_s']=max(r['acquisition_span_bound_s'] for r in report['records'])
    except (OSError,ValueError,KeyError,TypeError,RuntimeError) as e:
        report['status']='failed';report['error']=str(e)
        if reader is not None:report['last_raw_exchange']=reader.last_exchange
    return report


def save(a,report):
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2,ensure_ascii=False))
    return 0 if report['status']=='motor_acquisition_verified' else 2

if __name__=='__main__':raise SystemExit(main())
