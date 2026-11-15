#!/usr/bin/env python3
"""Archive a stopped height window with repeatable depth exposure.

Camera only: no serial/controller access, arm calibration or motion authority.
The caller must stop base/lift before invocation. This command cannot prove
physical stillness; geometry/ROI fitting and height accuracy need separate review.
Native depth is retained so plane fitting need not use rs.align's pixel filling.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.s2_rgbd_acquisition import Capture
from home_robot_tasks.stationary_rgbd import (
    HEIGHT_DEPTH_EXPOSURE_US, height_frame_issue, load_surface_height_model,
)


def capture_window(output,seconds=10.,*,capture_factory=Capture,clock=time.monotonic,camera_mode='wide'):
    if not 5<=seconds<=30:raise ValueError('height window must be 5 to 30 seconds')
    output=Path(output)
    model=load_surface_height_model(ROOT)
    from home_robot_tasks.camera_evidence import load_capture_profile
    profile,reference=load_capture_profile(ROOT,camera_mode)
    output.mkdir(parents=True,exist_ok=False)
    report=dict(status='CAPTURING',motor_commands=0,controller_access=False,
                motion_authorized=False,physical_stillness_verified=False,camera_mode=camera_mode,profiles=reference['profiles'],
                height_model_id=model.model_id,depth_exposure_us=HEIGHT_DEPTH_EXPOSURE_US,
                records=[],rejected_frames=0,settings_restored=False)
    def save():
        temp=output/'report.json.tmp'
        temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        temp.replace(output/'report.json')
    save()
    try:
        with capture_factory(profile,reference,retain_frame=True,retain_native_depth=True,
                             depth_exposure_us=HEIGHT_DEPTH_EXPOSURE_US) as camera:
            # Capture already warms up timestamps. Allow another 3s after stream
            # start before sampling the manual-exposure height window.
            start=clock();last=-float('inf');report['started_s']=start
            while clock()<start+3+seconds:
                record,images=camera.next_frame()
                if clock()<start+3:continue
                issue=height_frame_issue(record)
                if issue:
                    report['rejected_frames']+=1;report['last_frame_issue']=issue
                    continue
                if record['color_time_s']-last<.5:continue
                last=record['color_time_s'];rgb,aligned,metadata=images
                file=output/(str(len(report['records'])).zfill(3)+'.npz')
                np.savez_compressed(file,color_bgr=rgb,depth_aligned=aligned,
                                    native_depth=camera.native_depth_images[record['capture_id']])
                report['records'].append(dict(camera=record,file=file.name,
                    sha256=hashlib.sha256(file.read_bytes()).hexdigest()))
                (output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
                save()
        report['settings_restored']=True
        records=report['records']
        enough=(len(records)>=8 and
                records[-1]['camera']['color_time_s']-records[0]['camera']['color_time_s']>=3)
        report['status']='HEIGHT_WINDOW_RECORDED' if enough else 'RETRY_CAPTURE'
        return report
    except BaseException as error:
        report.update(status='CAPTURE_FAILED',error=str(error))
        raise
    finally:
        save()


def review_floor_window(output):
    """Derive exposure-bound height references from an existing native archive."""
    from home_robot_tasks.navigation_depth import native_endpoints
    from home_robot_tasks.stationary_rgbd import observed_floor_height_model
    output=Path(output)
    report=json.loads((output/'report.json').read_text())
    metadata=json.loads((output/'metadata.json').read_text())
    if report['status']!='HEIGHT_WINDOW_RECORDED':raise ValueError('complete height window required')
    ext=metadata['depth_to_color'];transform=np.eye(4)
    transform[:3,:3]=np.array(ext['rotation_column_major']).reshape(3,3,order='F')
    transform[:3,3]=ext['translation_m'];intr=metadata['native_depth_intrinsics']
    rows=[]
    for record in report['records']:
        file=output/record['file']
        if hashlib.sha256(file.read_bytes()).hexdigest()!=record['sha256']:
            raise ValueError('height frame hash mismatch')
        if height_frame_issue(record['camera']):raise ValueError('height exposure mismatch')
        with np.load(file) as archive:depth=archive['native_depth']*metadata['depth_scale_m']
        points=native_endpoints(depth,(depth>.15)&(depth<3.5),
            [intr[k] for k in ('fx','fy','ppx','ppy')],transform)
        model,evidence=observed_floor_height_model(ROOT,points)
        rows.append(dict(capture_id=record['camera']['capture_id'],file=record['file'],
            sha256=record['sha256'],model_id=model.model_id,
            base_z_from_optical=list(model.base_z_from_optical),ground_z_m=model.ground_z_m,
            evidence=evidence))
    result=dict(scope='height_only_per_exposure',motor_commands=0,controller_access=False,
        physical_stillness_verified=False,records=rows,
        observed_floor_frames=sum(r['evidence']['floor_observed'] for r in rows))
    (output/'floor-height-references.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--camera-mode',choices=('wide','legacy'),default='wide')
    parser.add_argument('--seconds',type=float,default=10.)
    parser.add_argument('--review-floor',action='store_true',help='review an existing archive only; does not open the camera or controller')
    args=parser.parse_args()
    if args.review_floor:
        report=review_floor_window(args.output)
        print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))
        return 0
    report=capture_window(args.output,args.seconds,camera_mode=args.camera_mode)
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))
    return 0 if report['status']=='HEIGHT_WINDOW_RECORDED' else 2


if __name__=='__main__':raise SystemExit(main())
