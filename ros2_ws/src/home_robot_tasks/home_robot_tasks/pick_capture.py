"""New unified-controller capture schema, retaining actual held-pose evidence.

Never manufacture legacy AP raw counts, 50ms synchronization, or read-only
motor-command claims. Capture occurs during an explicitly commanded constant
hold; its timing basis is the existing fixed-camera held admission rule.
"""
from dataclasses import asdict
import json
import math
from pathlib import Path
import numpy as np
from .pick_calibration import planning_lift_limit

from . import s2_geometry as geo
from .carry_live_inputs import JointExposure, match_held_exposure
from .snapshot_io import hashes
from .unified_carry_inputs import UnifiedPlatformGuard, controller_lift_height_m


def model_positions(joint, profile, lift):
    model = {'s2_lift_joint': lift}
    for side, offset in (('left',0), ('right',6)):
        for i, name in enumerate(geo.JOINTS):
            model[side+'_model_'+name] = geo.model_from_control(
                joint.positions_rad[offset+i], profile['arms'][side]['alignment'][name])
    return model


def make_evidence(record, history, platform, profile, *, now, requested_s, session_id, anchor):
    if min(record['color_time_s'], record['depth_time_s']) <= requested_s:
        raise ValueError('capture must be newer than this request')
    chosen = match_held_exposure(history, record['color_time_s'], record['depth_time_s'], now)
    if np.max(abs(np.asarray(chosen.positions_rad)-np.asarray(anchor))) > .02:
        raise ValueError('capture differs from held-pose anchor')
    from .pick_calibration import TOP_BASIS, TOP_BASES, top_model_height
    from so101_arm_bridge.lift_wire import LiftStatus
    top = platform.get('lift_reference_basis') in TOP_BASES
    status = LiftStatus(**platform['lift'])
    from so101_arm_bridge.mobile_wire import MobileStatus
    mobile = MobileStatus(**platform['mobile'])
    board = platform.get('controller_board')
    top_reference = platform.get('top_reference')
    lift = top_model_height(status, profile, board, top_reference, stationary_mobile=mobile) if top else controller_lift_height_m(status.height_um, profile, board=board)
    if not 0 <= lift <= planning_lift_limit(profile):
        raise ValueError('capture lift out of range')
    guard = UnifiedPlatformGuard(status.boot_id, lift, profile=profile, board=board, supported_top=top,
                                 top_reference=top_reference)
    guard.update(mobile, status,
                 platform['sent_s'], platform['received_s'])
    guard.require_fresh(now)
    if abs(guard.stamp-record['color_time_s']) > .2:
        raise ValueError('platform does not bracket capture hold')
    return dict(kind='unified_held_capture', schema_version=1, status='HELD_CAPTURE_VERIFIED',
                capture_id=record['capture_id'], calibration_id=profile['calibration_id'],
                profile_sha256=geo.digest(profile), camera=record,
                joint_exposure=asdict(chosen), stationary_window=[asdict(j) for j in history],
                platform=platform, captured_check_s=now, requested_s=requested_s,
                held_anchor_rad=list(anchor), fixed_lift_m=lift,
                model_joint_positions_si=model_positions(chosen,profile,lift),
                session_id=session_id, clock_id='host_monotonic:'+session_id,
                controller_boot_id=guard.boot_id,
                timing_basis='measured_constant_hold_fixed_camera',
                lift_reference_basis=platform['lift_reference_basis'] if top else 'same_session_supported_bottom_reference',
                motion_authorized=False, controller_hold_active=True)


def save_capture(folder, images, evidence):
    from PIL import Image
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    color, depth, metadata = images
    camera = evidence['camera']
    if (folder.name != camera['capture_id'] or
            any(metadata[k] != camera[k] for k in ('color_frame','depth_frame'))):
        raise ValueError('capture pixels and evidence differ')
    Image.fromarray(color[:,:,::-1]).save(folder/'color.png')
    Image.fromarray(depth).save(folder/'depth_aligned.png')
    (folder/'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False))
    evidence = dict(evidence, source_sha256=hashes(folder))
    (folder/'robot_capture.json').write_text(json.dumps(evidence,indent=2,allow_nan=False))
    return evidence


def review_capture(folder, report, profile):
    if (report.get('kind') != 'unified_held_capture' or report.get('schema_version') != 1
            or report.get('status') != 'HELD_CAPTURE_VERIFIED'
            or report.get('profile_sha256') != geo.digest(profile)
            or report.get('calibration_id') != profile['calibration_id']
            or report.get('motion_authorized') is not False
            or report.get('controller_hold_active') is not True
            or report.get('timing_basis') != 'measured_constant_hold_fixed_camera'
            or not isinstance(report.get('session_id'),str) or not report['session_id']):
        raise ValueError('unified capture schema/profile mismatch')
    meta = json.loads((folder/'metadata.json').read_text())
    source = hashes(folder)
    c = report['camera']
    if (source != report['source_sha256'] or c['capture_id'] != folder.name
            or report['capture_id'] != folder.name or c['camera_device'] != profile['camera_device']
            or c['source_frame'] != profile['source_frame']
            or any(meta[k] != c[k] for k in ('color_frame','depth_frame'))):
        raise ValueError('unified capture provenance mismatch')
    history = [JointExposure(**j) for j in report['stationary_window']]
    if any(len(j.positions_rad)!=12 or not np.isfinite(j.positions_rad).all()
           or not math.isfinite(j.lower_s) or not math.isfinite(j.upper_s) or j.upper_s<j.lower_s for j in history):
        raise ValueError('invalid captured joint window')
    rebuilt = make_evidence(c,history,report['platform'],profile,now=report['captured_check_s'],
                            requested_s=report['requested_s'],session_id=report['session_id'],
                            anchor=report['held_anchor_rad'])
    # JSON normalizes tuples to arrays; compare canonical digests.
    for key,value in rebuilt.items():
        if geo.digest(value) != geo.digest(report[key]):
            raise ValueError('unified capture evidence changed: '+key)
    answer = dict(mode='offline_capture_review', capture_id=folder.name, frame='base_link',
                  motion_authorized=False, executable=False, physical_task_completed=False, grasp_pose=None,
                  clock_id=report['clock_id'],session_id=report['session_id'],
                  source_calibration_id=profile['calibration_id'], arm_coordinate_convention=profile['arm_coordinate_convention'],
                  lift_height_m=report['fixed_lift_m'], model_joint_positions_si=report['model_joint_positions_si'],
                  lift_reference_basis=report['lift_reference_basis'])
    for side in ('left','right'):
        answer[side+'_anchor_rad']={n:answer['model_joint_positions_si'][side+'_model_'+n] for n in geo.JOINTS[:5]}
    return answer,meta,source
