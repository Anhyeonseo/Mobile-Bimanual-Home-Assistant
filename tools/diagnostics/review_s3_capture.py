#!/usr/bin/env python3
"""Offline S3 handoff: review a selected observation from one anchored RGB-D exposure."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks import s2_geometry as s2, s2_development as dev
from home_robot_tasks.s2_acquisition import stationary_lift_window

# Compatibility exports for earlier S3 tools.
from home_robot_tasks.snapshot_io import hashes, save_frame, FILES


LEGACY_PROFILE = 'config/home_assistant/historical/s2_coordinates.20261105.json'


def capture_profile(profile, geometry, calibration_id):
    if calibration_id == profile['calibration_id']:
        return profile
    old = json.loads((ROOT/LEGACY_PROFILE).read_text())
    dev.validate(old, geometry)
    if (old['calibration_id'] != calibration_id or profile['schema_version'] != 2
            or old['assembly_revision'] != profile['assembly_revision'] or old['camera_device'] != profile['camera_device']
            or old['base_from_optical'] != profile['base_from_optical'] or old['lift'] != profile['lift']
            or any(profile['source_files_sha256'].get(k) != v for k, v in old['source_files_sha256'].items())):
        raise ValueError('unsupported capture calibration or changed source geometry')
    return old


def review(folder, report, observations, profile, geometry, label, location):
    """Never provides executable geometry. UTC is provenance, not the alignment clock."""
    dev.validate(profile, geometry)
    if report.get('kind')=='resident_fixed_capture':
        from home_robot_tasks.resident_capture import review_fixed_capture
        answer,meta,source=review_fixed_capture(folder,report,profile)
        return select_observation(answer,observations,profile,meta,source,label,location)
    exported = report['snapshot_export']
    source = hashes(folder)
    if (report['status'] != 'motor_acquisition_verified' or report['camera_synchronized'] is not True
            or report['motor_write_commands'] != 0 or report['motion_authorized'] is not False
            or source != exported['source_sha256'] or folder.name != exported['capture_id']):
        raise ValueError('successful read-only capture and exact exported files required')
    rows = [r for r in report['records'] if r['sequence'] == exported['sequence']]
    if len(rows) != 1:
        raise ValueError('unique capture sequence required')
    row = rows[0]
    snap, camera, coords = row['snapshot'], row['camera'], row['coordinates']
    ref = report['reference_evidence']
    source_profile = capture_profile(profile, geometry, ref['calibration_id'])
    meta = json.loads((folder/FILES[2]).read_text())
    if (any(r['capture_id'] != folder.name for r in (camera, snap, coords))
            or any(snap[k] != coords[k] or snap[k] != ref[k] for k in ('session_id','origin_id','clock_id','controller_boot_id','calibration_id'))
            or snap['arm_anchor_id'] != ref['arm_anchor_id'] or snap['sequence'] != row['sequence']
            or ref['arm_turns_anchored'] is not True or ref['lift_bottom_confirmed'] is not True
            or snap['camera_time_s'] != camera['color_time_s'] or coords['observed_s'] != camera['color_time_s']
            or snap['calibration_id'] != source_profile['calibration_id']
            or snap.get('assembly_revision') != source_profile['assembly_revision']
            or snap.get('raw_encoding') != 'continuous_unwrapped_counts'
            or coords['coordinate_mode'] != 'development_capture_time' or coords['frame'] != 'base_link'
            or snap['camera_device'] != profile['camera_device'] or camera['camera_device'] != snap['camera_device']
            or camera['source_frame'] != profile['source_frame'] or snap['source_frame'] != camera['source_frame']
            or meta['coordinate_frame'] != camera['source_frame']
            or any(meta[k] != camera[k] for k in ('color_frame','depth_frame'))
            or not 0 <= camera['acquisition_span_s'] <= profile['policy']['maximum_skew_s']):
        raise ValueError('exposure/session/device/profile mismatch')
    answer = dict(mode='offline_capture_review', capture_id=folder.name, frame='base_link',
                  motion_authorized=False, executable=False, physical_task_completed=False,
                  grasp_pose=None, clock_id=snap['clock_id'], session_id=snap['session_id'],
                  source_calibration_id=source_profile['calibration_id'],
                  arm_coordinate_convention=profile.get('arm_coordinate_convention', 'legacy'))
    moving = [sample for sample in row['samples'] if sample['speed'] != 0]
    if len(row['samples']) != 13 or any((s.get('bus'),s.get('id')) != (0,10) for s in moving):
        return dict(answer, status='REOBSERVE', reason='arm/lift not confirmed stationary')
    if moving:
        # The lift register occasionally reports +/-50 with unchanged encoders.
        # Require a continuous, same-epoch window spanning the actual exposure;
        # never replace or erase the nonzero speed reading in the source record.
        index=report['records'].index(row)
        try:
            window=report['records'][max(0,index-2):index+3]
            evidence=stationary_lift_window(window)
        except (ValueError,KeyError,TypeError):
            return dict(answer,status='REOBSERVE',reason='lift position stability not confirmed around exposure')
        answer['stationarity_evidence']=dict(evidence,first_sequence=window[0]['sequence'],last_sequence=window[-1]['sequence'],
                                            reported_lift_speed=moving[0]['speed'])
    positions=coords['model_joint_positions_si']
    answer['lift_height_m'] = positions['s2_lift_joint']
    answer['model_joint_positions_si'] = {'s2_lift_joint': positions['s2_lift_joint']}
    for side in s2.ARMS:
        raw = snap['raw'][side]
        original = s2.model_angles(raw, source_profile['arms'][side]['alignment'])
        if any(not math.isclose(original[j], positions[f'{side}_model_{j}'], rel_tol=0., abs_tol=1e-10)
               for j in s2.JOINTS):
            raise ValueError('recorded joint angles disagree with capture raw counts')
        model = s2.model_angles(raw, profile['arms'][side]['alignment'])
        answer[side + '_anchor_rad'] = {j:model[j] for j in s2.JOINTS[:5]}
        answer['model_joint_positions_si'].update({f'{side}_model_{j}':model[j] for j in s2.JOINTS})
    return select_observation(answer,observations,profile,meta,source,label,location)


def select_observation(answer,observations,profile,meta,source,label,location):
    """Shared selection after verified capture binding (or an explicitly synthetic fixture)."""
    choice = choose_observation(observations, answer['capture_id'], profile, meta, source, label, location)
    if choice['status'] != 'SELECTED':
        return dict(answer, **choice)
    index = choice['observation_index']
    box, o = choice['box_xyxy'], observations[index]
    points = {}
    for field in ('surface_xyz_m','center_xyz_m'):
        if o.get(field) is None:
            continue
        points[field] = s2.optical_to_base(profile['base_from_optical'], o[field])
    if not points:
        return dict(answer, status='REOBSERVE', reason='no_3d_point')
    return dict(answer, status='TARGET_DISPLAY_ONLY', label=label, box_xyxy=box, observation_index=index,
                source_sha256=source, calibration_id=profile['calibration_id'],
                base_points_m=points, point_semantics='measured surface point / visible surface center; neither is a jaw contact')


def choose_observation(observations, capture_id, profile, meta, source, label, location):
    """Choose only in this image. A camera-only replay needs no base pose."""
    candidates = []
    for index,o in enumerate(observations):
        if (o['capture_id'] != capture_id or o['frame_id'] != profile['source_frame']
                or o['source_sha256'] != source or o['captured_at_utc'] != meta['captured_at_utc']
                or o.get('robot_transform') is not None
                or o.get('calibration_id') not in ('unmeasured', profile['calibration_id'])
                or o.get('clock_domain') != 'global_time'):
            raise ValueError('observation does not belong to the exported exposure')
        if o['status'] not in ('accepted','held','rejected'):
            raise ValueError('unknown observation status')
        # A detected object with unusable depth still occupies its image position.
        # Dropping it could turn the right-hand cup into a false 'left cup'.
        if o['label'] == label and (o['status'] != 'rejected' or o.get('failure')=='INVALID_DEPTH'):
            box = np.asarray(o['box_xyxy'], dtype=float)
            if (box.shape != (4,) or not np.isfinite(box).all() or not 0 <= box[0] < box[2] <= meta['color_intrinsics']['width']
                    or not 0 <= box[1] < box[3] <= meta['color_intrinsics']['height']):
                raise ValueError('invalid pixel box')
            candidates.append((box.tolist(), o, index))
    if not candidates:
        return dict(status='NOT_FOUND')
    if len(candidates) > 1:
        if location not in ('left','right'):
            return dict(status='AMBIGUOUS_TARGET')
        candidates.sort(key=lambda x: x[0][0]+x[0][2], reverse=location=='right')
        box=candidates[0][0]
        # A wide third box can overlap the first even when the second does not.
        if any((box[2]>=other[0] if location=='left' else box[0]<=other[2])
               for other,_,_ in candidates[1:]):
            return dict(status='AMBIGUOUS_TARGET')
    box, o, index = candidates[0]
    if o['status'] != 'accepted':
        return dict(status='REOBSERVE', reason=o.get('reason'))
    return dict(status='SELECTED', observation_index=index, box_xyxy=box)


def render_review(folder, result, destination):
    """Annotate the original color image. Blocked observations get no target box."""
    from PIL import Image, ImageDraw
    image = Image.open(folder/FILES[0]).convert('RGB')
    draw = ImageDraw.Draw(image)
    if result['status']=='TARGET_DISPLAY_ONLY':
        draw.rectangle(result['box_xyxy'],outline='lime',width=3)
        lines=['BASE (m) / '+result['label']]
        for kind, xyz in result['base_points_m'].items():
            lines.append(kind+': '+' '.join(f'{v:+.3f}' for v in xyz))
    else:
        lines=[result['status']]
    text='\n'.join(lines)
    # Numeric overlay uses ASCII independently of the system's installed fonts.
    text=text.encode('ascii','replace').decode('ascii')
    draw.rectangle(draw.multiline_textbbox((4,4),text),fill='black')
    draw.multiline_text((4,4),text,fill='white')
    with destination.open('xb') as f:image.save(f,format='PNG')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','report','observations','profile','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--label',required=True);p.add_argument('--screen-side',choices=('left','right'))
    p.add_argument('--preview',type=Path,help='optional annotated PNG, never a live motion target')
    a=p.parse_args()
    if a.output.exists() or (a.preview and (a.preview.exists() or a.preview==a.output)):
        p.error('use new, distinct output paths')
    try:
        read=lambda path:json.loads(path.read_text())
        result=review(a.snapshot, read(a.report), read(a.observations)['observations'], read(a.profile),
                      s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json'),a.label,a.screen_side)
        with a.output.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
        if a.preview:render_review(a.snapshot,result,a.preview)
        print(json.dumps(result,ensure_ascii=False));return 0 if result['status']=='TARGET_DISPLAY_ONLY' else 2
    except (ValueError,OSError,KeyError,TypeError) as e:
        p.exit(2,str(e)+'\n')

if __name__=='__main__':raise SystemExit(main())
