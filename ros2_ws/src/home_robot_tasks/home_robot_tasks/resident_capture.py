"""Validate a same-resident planning capture without inventing legacy raw records."""
import json
import math
import numpy as np
from . import s2_geometry as geo
from .snapshot_io import hashes


def review_fixed_capture(folder,report,profile):
    source=hashes(folder);meta=json.loads((folder/'metadata.json').read_text())
    if (report['schema_version']!=1 or report['status']!='stationary_capture_verified'
            or report['motor_write_commands']!=0 or report['motion_authorized'] is not False
            or report['calibration_id']!=profile['calibration_id'] or report['capture_id']!=folder.name
            or report['source_sha256']!=source
            or report['lift_reference_basis']!='operator_established_height_continuous_stationary_encoder_guard'):
        raise ValueError('resident capture provenance mismatch')
    c=report['camera'];j=report['joint_exposure'];window=report['stationary_window'];platform=report['platform']
    if (c['capture_id']!=folder.name or c['camera_device']!=profile['camera_device']
            or any(c[k]!=meta[k] for k in ('color_frame','depth_frame'))
            or len(window)<10 or window[-1]['lower_s']-window[0]['upper_s']<.3
            or j not in window or platform['present_mask']!=15 or max(platform['sample_age_ms'])>180):
        raise ValueError('resident exposure or stationarity evidence missing')
    samples=np.asarray([s['positions_rad'] for s in window])
    if samples.shape!=(len(window),12) or not np.isfinite(samples).all() or np.max(np.ptp(samples,axis=0))>.01:
        raise ValueError('resident arm not stationary')
    span=max(j['upper_s'],c['color_time_s'],c['depth_time_s'])-min(j['lower_s'],c['color_time_s'],c['depth_time_s'])
    if not 0<=span<=.05:raise ValueError('resident exposure not synchronized')
    lift=report['fixed_lift_m']
    if not math.isfinite(lift) or not 0<=lift<=profile['lift']['travel_m']:raise ValueError('invalid fixed height')
    model={'s2_lift_joint':lift}
    for side,offset in (('left',0),('right',6)):
        for i,n in enumerate(geo.JOINTS):model[side+'_model_'+n]=geo.model_from_control(j['positions_rad'][offset+i],profile['arms'][side]['alignment'][n])
    if model!=report['model_joint_positions_si']:raise ValueError('capture model conversion changed')
    answer=dict(mode='offline_capture_review',capture_id=folder.name,frame='base_link',motion_authorized=False,
        executable=False,physical_task_completed=False,grasp_pose=None,clock_id=report['clock_id'],
        session_id=report['session_id'],source_calibration_id=profile['calibration_id'],
        arm_coordinate_convention=profile['arm_coordinate_convention'],lift_height_m=lift,
        model_joint_positions_si=model,lift_reference_basis=report['lift_reference_basis'])
    for side in ('left','right'):answer[side+'_anchor_rad']={n:model[side+'_model_'+n] for n in geo.JOINTS[:5]}
    return answer,meta,source
