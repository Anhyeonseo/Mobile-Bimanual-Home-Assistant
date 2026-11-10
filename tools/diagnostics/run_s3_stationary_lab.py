#!/usr/bin/env python3
"""Run fixed-base left-arm scenarios or review a bound offline candidate group. No hardware."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge'),str(ROOT/'tools/diagnostics')]
from home_perception.geometry.plane import Plane, backproject
from home_perception.grasp.handoff import flat_candidates
from home_robot_tasks import s2_geometry as geo, s2_development as dev
from home_robot_tasks.stationary_pick_sim import FixturePayload, NAMES, StationaryArm, plan, plan_pick
from home_robot_tasks.installed_jaw import InstalledLeftJaw
from home_robot_tasks.so101_jaw import load_stock_jaw
from home_robot_tasks.stationary_trial import simulate_pick, simulate_trial
from review_s3_capture import select_observation

CONFIG=ROOT/'config/home_assistant'
CASES=[(.180,.100,-12),(.183,.105,8),(.186,.109,-5),(.190,.116,17),(.194,.112,-20)]
DEST=[.188,.092,.852]
SCENE=[([.126,.055,.808],[.245,.131,.840])]


def read(p):return json.loads(Path(p).read_text())


def stock_jaw():
    return load_stock_jaw(CONFIG / 'official_sources/so101_new_calib.urdf',
                          ROOT / 'ros2_ws/src/so101_description/meshes')


def model(lift, side="left", *, profile=None):
    if profile is None:
        profile=read(CONFIG/'s2_coordinates.development.json')
    return StationaryArm(profile,geo.load_geometry(CONFIG/'official_geometry.json'),
                         read(CONFIG/'arm_ranges.development.json'), lift, stock_jaw(), side)


def fixture(arm,case,repeat,fault=None, *, candidate_source='fixture', marker_length_m=.014, surface_z=.852):
    """Generated colored regions/depth, not detector output or measured performance."""
    x,y,yaw=case
    truth=np.array([x,y,surface_z])
    C=np.asarray(arm.profile['base_from_optical']);inv=np.linalg.inv(C)
    intr=read(CONFIG/'camera_device.observed.json')['sdk_calibration']['color_intrinsics']
    fx,fy,cx,cy=[intr[k] for k in ('fx','fy','ppx','ppy')]
    h,w=intr['height'],intr['width']
    rgb=np.zeros((h,w,3),np.uint8);depth=np.zeros((h,w),np.uint16)
    cap=f'virtual-{x}-{y}-{yaw}-{repeat}-{fault}-z{surface_z}-lift{arm.lift_m}-length{marker_length_m}';utc='fixture-clock'
    objects=[truth+[0,-.035,0],truth]  # distractor on right, same class
    if repeat==2:objects.reverse()
    meta=dict(captured_at_utc=utc,color_intrinsics=intr)
    observations=[];target_pixel=None
    object_pixels = []
    for index,point in enumerate(objects):
        # Analytic mask for an oriented oval top surface, rasterized with D415 intrinsics.
        # Ground-truth segmentation is intentional: this tests geometry, not detection.
        grid=np.linspace(-1.,1.,65);aa,bb=np.meshgrid(grid,grid)
        mask=aa*aa+bb*bb<=1
        local=np.vstack((aa[mask]*marker_length_m/2,bb[mask]*.004,np.zeros(mask.sum())))
        base=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@local+point[:,None]
        cam=inv[:3,:3]@base+inv[:3,3,None]
        u=np.rint(fx*cam[0]/cam[2]+cx).astype(int);v=np.rint(fy*cam[1]/cam[2]+cy).astype(int)
        inside=(u>=0)&(u<w)&(v>=0)&(v<h)&(cam[2]>0)
        if not inside.any():raise ValueError('synthetic target outside camera view')
        u,v=u[inside],v[inside]
        rgb[v,u]=[20,100,200];depth[v,u]=np.rint(cam[2,inside]*1000).astype(np.uint16)
        pixels=np.unique(np.stack((v,u),axis=1),axis=0);v,u=pixels.T
        object_pixels.append((v, u))
        z=depth[v,u].astype(float)/1000
        box=[int(u.min()),int(v.min()),int(u.max()+1),int(v.max()+1)]
        observations.append(dict(label='marker',capture_id=cap,captured_at_utc=utc,frame_id=arm.profile['source_frame'],
            clock_domain='global_time',calibration_id='unmeasured',status='accepted',
            box_xyxy=box,center_xyz_m=[float(np.mean((u-cx)*z/fx)),float(np.mean((v-cy)*z/fy)),float(np.mean(z))]))
        if np.array_equal(point,truth):target_pixel=index
    if fault=='overlap':observations[1]['box_xyxy']=observations[0]['box_xyxy']
    if fault=='held':observations[target_pixel].update(status='held',reason='synthetic_depth_dropout',center_xyz_m=None)
    source={k:hashlib.sha256(data).hexdigest() for k,data in (
        ('fixture_rgb',rgb.tobytes()),('fixture_depth',depth.tobytes()),('fixture_metadata',json.dumps(meta).encode()))}
    for o in observations:o['source_sha256']=source
    answer=dict(mode='synthetic_selection',capture_id=cap,frame='base_link',motion_authorized=False,
                executable=False,physical_task_completed=False,lift_height_m=arm.lift_m,session_id=cap,
                **{arm.side + '_anchor_rad': {n:float(q) for n,q in zip(NAMES,arm.anchor)}})
    review=select_observation(answer,observations,arm.profile,meta,source,'marker','left')
    if review['status']!='TARGET_DISPLAY_ONLY':return review,None,truth
    if review['observation_index']!=target_pixel:raise RuntimeError('selected the distractor')
    group={k:review[k] for k in ('capture_id','label','source_sha256','observation_index')}
    group.update(coordinate_frame=arm.profile['source_frame'],calibration_id=arm.profile['calibration_id'],
                 session_id=review['session_id'],perception_status='accepted')
    if fault=='wrong_exposure':group['capture_id']='old-frame'
    if candidate_source in ('yeoeun-rule', 'yeoeun-surface'):
        target_mask = np.zeros(depth.shape, dtype=bool)
        target_mask[object_pixels[review['observation_index']]] = True
        depth_m = depth.astype(float) / 1000
        table_point = inv @ [x, y, surface_z - .012, 1]
        plane = Plane(inv[:3, :3] @ [0, 0, 1], table_point[:3], 0., 1.)
        group.update(flat_candidates(backproject(depth_m, intr, target_mask & (depth > 0)),
                                     backproject(depth_m, intr, depth > 0), plane,
                                     surface_variants=candidate_source == 'yeoeun-surface'))
        group['mask_sha256'] = hashlib.sha256(target_mask.tobytes()).hexdigest()
        return review, group, truth
    if candidate_source != 'fixture':
        raise ValueError('unknown candidate source')
    grasps=[]
    for tilt,width in ((0,.065),(-15,.032),(-30,.032)):
        # Independent scenario orientation and contact hypothesis, not FK-generated goals.
        rot=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('y',tilt,degrees=True).as_matrix()@np.array([[0,0,1],[0,1,0],[-1,0,0]])
        base_tip=np.array(review['base_points_m']['center_xyz_m'])
        origin=(inv@[*list(base_tip-.02*rot[:,0]),1])[:3]
        grasps.append(dict(source='synthetic',width_m=width,depth_m=.02,
                           translation_m=origin.tolist(),rotation_matrix=(inv[:3,:3]@rot).tolist()))
    group['refined_grasps']=grasps
    return review,group,truth


def run_case(arm, case, repeat, fault=None, *, task='pick-lift', yaws=(0,-20,20), place_half_extent=0.,
             candidate_source='fixture', marker_length_m=.014, surface_z=.852):
    review, group, truth = fixture(arm, case, repeat, fault, candidate_source=candidate_source,
                                 marker_length_m=marker_length_m, surface_z=surface_z)
    out = dict(case=case, repeat=repeat, fault=fault, task=task, review=review, physical_task_completed=False)
    if group is None:
        return dict(out, status=review['status'])
    if not group['refined_grasps']:
        return dict(out, group=group, status='NO_PERCEPTION_CANDIDATE')
    neighbor = truth + np.array([0, -.035, 0])
    dz = np.array([0, 0, surface_z - .852])
    lo, hi = (corner + dz for corner in SCENE[0])
    # A shifted test object and its neighbour must still rest on the modeled tabletop.
    span_xy = np.abs(Rotation.from_euler('z', case[2], degrees=True).as_matrix()[:2,:2]) @ [marker_length_m/2, .004]
    lo[:2] = np.minimum(lo[:2], neighbor[:2] - span_xy)
    hi[:2] = np.maximum(hi[:2], truth[:2] + span_xy)
    support = [(lo.tolist(), hi.tolist())]
    scene = support + [((neighbor - [.007,.007,.009]).tolist(), (neighbor + [.007,.007,.009]).tolist())]
    if fault == 'obstacle':
        scene.append(([.12,.08,.84], [.22,.15,.92]))
    if fault == 'home_blocked':
        # Place the synthetic obstruction at the current Home TCP so a new
        # calibration cannot silently turn this negative case into free space.
        home_tip = arm.fk(arm.anchor)[0][:3, 3]
        scene.append(((home_tip-.025).tolist(), (home_tip+.025).tolist()))
    if task == 'pick-lift':
        payload = None
        if candidate_source != 'fixture':
            top = truth
            height = top[2] - support[0][1][2]
            half_extents = np.array([marker_length_m/2, .004, height/2])
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_euler('z', case[2], degrees=True).as_matrix()
            pose[:3, 3] = top - [0, 0, height/2]
            payload = FixturePayload(pose, half_extents)
            out['payload_box'] = dict(base_pose=pose.tolist(), half_extents_m=half_extents.tolist())
            # The other marker has the same dimensions and rests on the same table.
            span = np.abs(pose[:3, :3]) @ half_extents
            other_center = neighbor - [0, 0, height/2]
            scene[1] = ((other_center - span).tolist(), (other_center + span).tolist())
        path = plan_pick(arm, review, group, scene, payload=payload)
    elif task == 'pick-place':
        if fault == 'unreachable_place':
            scene[0] = (scene[0][0], [1.1,.131,.840])
        dest = [1.,.1,.852] if fault == 'unreachable_place' else DEST
        out['place_xyz_m'] = dest
        path = plan(arm, review, group, dest, scene, place_yaw_offsets_deg=yaws,
                    place_xy_half_extent_m=place_half_extent)
    else:
        raise ValueError('unknown stationary task')
    out.update(group=group, plan=path, status=path['status'], scene_boxes_m=scene)
    if path['status'] == 'VIRTUAL_PATH_FOUND':
        width = group['refined_grasps'][path['candidate_index']]['width_m']
        if task == 'pick-lift':
            center = truth if payload is None else payload.base_pose[:3, 3]
            trial = simulate_pick(arm, path, width, center, fault=fault, payload=payload)
        else:
            trial = simulate_trial(arm, path, width, truth, path['chosen_place_xyz_m'], fault=fault)
        out.update(trial=trial, status=trial['status'])
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--task', choices=('pick-lift', 'pick-place'), default='pick-lift',
                        help='default: pick, lift and hold; pick-place replays the earlier fixed-base comparison')
    parser.add_argument('--candidate-source', choices=('fixture', 'yeoeun-rule', 'yeoeun-surface'),
                        help='default pick-lift producer: original Yeoeun rule followed by checked surface variants')
    parser.add_argument('--marker-length-mm', type=float, help='synthetic marker long dimension; default 28 mm (old fixture 14 mm)')
    parser.add_argument('--input', type=Path, help='plan only: review, bound candidate group and scene_boxes_m')
    a = parser.parse_args(argv)
    if (a.task != 'pick-lift' or a.input) and (a.candidate_source is not None or a.marker_length_mm is not None):
        parser.error('synthetic candidate options apply only to pick-lift without --input')
    source = a.candidate_source or ('yeoeun-surface' if a.task == 'pick-lift' else 'fixture')
    length_mm = a.marker_length_mm if a.marker_length_mm is not None else (14. if source == 'fixture' else 28.)
    if not 8 <= length_mm <= 35:
        parser.error('synthetic marker length must be 8–35 mm')
    if a.output.exists():
        parser.error('choose a new output path')
    try:
        report = dict(mode='simulation_only', task=a.task, motion_authorized=False,
                      hardware_commands=0, physical_task_completed=False)
        if a.input:
            data = read(a.input)
            inputs = (model(data['review']['lift_height_m']), data['review'], data['group'])
            if a.task == 'pick-lift':
                pad_record = CONFIG / 'left_pad_gap.observed.json'
                measured = None
                if data['review']['mode'] == 'offline_capture_review' and pad_record.is_file():
                    record = read(pad_record)
                    if record.get('calibration_id') == inputs[0].profile['calibration_id']:
                        measured = InstalledLeftJaw(record, inputs[0].profile)
                report['jaw_basis'] = 'measured_installed_pads' if measured else 'nominal_official_stl'
                report['path'] = plan_pick(*inputs, data['scene_boxes_m'], installed_left_jaw=measured)
            else:
                report['path'] = plan(*inputs, data['place_xyz_m'], data['scene_boxes_m'],
                                      place_yaw_offsets_deg=data.get('place_yaw_offsets_deg', [0]),
                                      place_xy_half_extent_m=data.get('place_xy_half_extent_m', 0.))
        else:
            arm = model(.61)
            settings = dict(task=a.task, candidate_source=source, marker_length_m=length_mm/1000)
            normal = [run_case(arm, case, repeat, **settings) for case in CASES for repeat in (1, 2)]
            report.update(candidate_source=source, marker_length_mm=length_mm)
            faults = ['overlap', 'held', 'wrong_exposure', 'obstacle', 'home_blocked', 'empty_close',
                      'lost_load', 'cancel']
            if a.task == 'pick-lift':
                faults.extend(['dropped_during_hold', 'table_contact', 'occluded_probe'])
                if source != 'fixture':
                    faults.extend(['one_tip_contact', 'early_resistance'])
            else:
                region = [run_case(arm, case, repeat, task=a.task, place_half_extent=.015)
                          for case in CASES for repeat in (1, 2)]
                report.update(region_cases=region, allowed_region_half_extent_m=.015, place_xyz_m=DEST,
                              region_counts=dict(Counter(x['status'] for x in region)))
                faults.extend(['unreachable_place', 'stuck_release'])
            failures = [run_case(arm, CASES[0], 1, fault, **settings) for fault in faults]
            report.update(cases=normal, faults=failures, nominal_counts=dict(Counter(x['status'] for x in normal)),
                          simulated_lift_m=arm.lift_m, scene_boxes_m=SCENE)
        a.output.parent.mkdir(parents=True, exist_ok=True)
        with a.output.open('x') as f:
            json.dump(report, f, indent=2, allow_nan=False)
        print(report.get('nominal_counts') or report['path']['status'])
        if a.input and report['path']['status'] != 'VIRTUAL_PATH_FOUND':
            return 2
        return 0
    except (OSError, ValueError, KeyError, TypeError) as e:
        parser.exit(2, str(e) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
