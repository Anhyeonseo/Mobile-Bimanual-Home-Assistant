#!/usr/bin/env python3
"""Offline support-height and two-arm feasibility trials for stock SO-ARM101s; no device imports."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks'), str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_perception.geometry.plane import Plane
from home_perception.grasp.handoff import flat_candidates
from home_robot_tasks.stationary_lift import SupportedContact, choose_height
from home_robot_tasks.stationary_pair import paired_path
from home_robot_tasks.stationary_pick_sim import (FixturePayload, GRASP_FROM_TCP, Rejected,
                                                  _pick_waypoints, check_stages, pad_tip_optical)
from home_robot_tasks.stationary_trial import simulate_pick
from tools.diagnostics import run_s3_stationary_lab as lab


# Declared evaluation layouts, separate from the previous five tuning cases.
# (x, y, yaw, surface Z, marker length, starting lift travel), all lengths in metres.
LAYOUTS = [
    (.186,.091,-9,.867,.028,.61), (.192,.093,12,.842,.029,.58),
    (.206,.072,5,.821,.030,.61), (.218,.055,-18,.802,.032,.61),
    (.237,.040,10,.772,.027,.61), (.33,.070,-4,.815,.028,.60),
]


def height_trial(layout):
    x,y,yaw,z,length,current = layout
    cache = {}
    rejections = Counter()
    def probe(height):
        if height not in cache:
            try:
                case = lab.run_case(lab.model(height), (x,y,yaw), 1, candidate_source='yeoeun-surface',
                                    marker_length_m=length, surface_z=z)
            except ValueError as exc:
                if str(exc) != 'synthetic target outside camera view':
                    raise
                case = dict(status='OUTSIDE_CAMERA_VIEW')
            rejections.update(a.get('checking', 'input') + '/' + a['reason']
                              for a in case.get('plan', {}).get('attempts', []) if 'reason' in a)
            cache[height] = case
        case = cache[height]
        return case.get('plan', {}).get('status', case['status'])

    arm = lab.model(current)
    baseline = probe(current)
    first = cache[current]
    if not first.get('group', {}).get('refined_grasps'):
        return dict(layout=layout, baseline=first['status'], selected=None, status=first['status'])
    # Several contact poses can belong to one object. Seed the search from their median tip height.
    optical_to_base = np.asarray(arm.profile['base_from_optical'])
    contact_zs = [(optical_to_base @ pad_tip_optical(c))[2, 3] for c in first['group']['refined_grasps']
                  if arm.jaw.accepts(c['width_m'])]
    if not contact_zs:
        return dict(layout=layout, baseline=first['status'], selected=None, status='NO_ELIGIBLE_CONTACT')
    observed_z = float(np.median(contact_zs))
    selection = choose_height([arm], [SupportedContact(z-.012, observed_z)], probe)
    naive = float(np.clip(z-.012 - arm.profile['ground_z_m'] - arm.profile['lift']['floor_height_low_m']['left'],
                          0., arm.profile['lift']['travel_m']))
    surface_match = probe(naive)
    winner = cache.get(selection['lift_m'], {})
    return dict(layout=layout, baseline=first['status'], baseline_path=baseline, support_match_lift_m=naive, support_match_path=surface_match,
                rejections=dict(rejections), selection=selection,
                status=winner.get('status', selection['status']), final_rise_m=winner.get('trial',{}).get('final_rise_m'))


def patch_points(center, yaw):
    xy = np.stack(np.meshgrid(np.linspace(-1,1,33), np.linspace(-1,1,25)), axis=-1).reshape(-1,2)
    xy = xy[np.sum(xy**2,axis=1) <= 1]
    points = np.column_stack((xy[:,0]*.014,xy[:,1]*.004,np.zeros(len(xy))))
    return points @ Rotation.from_euler('z', yaw, degrees=True).as_matrix().T + center


def visible(arm, points):
    optical = np.linalg.inv(arm.profile['base_from_optical'])
    cam = points @ optical[:3,:3].T + optical[:3,3]
    k = lab.read(lab.CONFIG/'camera_device.observed.json')['sdk_calibration']['color_intrinsics']
    if (cam[:,2] <= 0).any():
        return False
    u = k['fx']*cam[:,0]/cam[:,2]+k['ppx']
    v = k['fy']*cam[:,1]/cam[:,2]+k['ppy']
    return bool(np.all((u >= 0)&(u < k['width'])&(v >= 0)&(v < k['height'])))


def patch_goal(arm, candidate):
    rotation = np.asarray(candidate['rotation_matrix'])
    goal = np.eye(4)
    goal[:3,:3] = rotation @ GRASP_FROM_TCP
    goal[:3,3] = (candidate['translation_m'] + candidate['depth_m']*rotation[:,0]
                  + goal[:3,0]*arm.jaw.center_from_fixed_m(candidate['width_m']))
    return goal


def patch_paths(arm, candidates, scene, body, patch, *, scene_from_base=None, first_only=False):
    """A stipulated graspable tab is known in base coordinates, independent of camera visibility."""
    paths = []
    rejected = Counter()
    for i,c in enumerate(candidates):
        w,d = c['width_m'], c['depth_m']
        if not arm.jaw.accepts(w):
            rejected['JAW_SECTION_LIMIT'] += 1
            continue
        goal = patch_goal(arm,c)
        attempt = dict(anchor_rad=arm.anchor.tolist(), stages=[], width=w, depth=d, candidate=i)
        try:
            check_stages(arm, arm.anchor, _pick_waypoints(goal,.02), scene, w, d, .009, body, attempt, scene_from_base=scene_from_base)
        except Rejected as exc:
            rejected[attempt['checking'] + '/' + str(exc)] += 1
            continue
        plan = dict(mode='nominal_stationary_simulation', task='pick_lift',
                    status='VIRTUAL_PATH_FOUND', attempts=[attempt])
        # Verify grip on this local contact patch only; rigidity of a shared body is screened separately.
        hold = simulate_pick(arm, plan, w, patch.base_pose[:3,3], payload=patch)
        if hold['status'] != 'SIMULATED_PICK_HELD':
            rejected[hold['status']] += 1
            continue
        paths.append(attempt)
        if first_only:
            break
    return paths, dict(rejected)


def pair_trial(name, centers, yaws, current, shared):
    centers = np.asarray(centers)
    patches, clouds = [], []
    for center,yaw in zip(centers,yaws):
        pose = np.eye(4)
        pose[:3,:3] = Rotation.from_euler('z',yaw,degrees=True).as_matrix()
        pose[:3,3] = center-[0,0,.006]
        patches.append(FixturePayload(pose,np.array([.014,.004,.006])))
        clouds.append(patch_points(center,yaw))
    if shared:
        # Known rigid box connecting two graspable end tabs; bulk contact and forces are not modeled.
        pose = np.eye(4)
        pose[:3,3] = centers.mean(axis=0)-[0,0,.006]
        diff = abs(centers[0]-centers[1])
        body = FixturePayload(pose,diff/2+[.014,.014,.006])
        loads = [body, body]
    else:
        loads = patches
    low,high = centers.min(axis=0), centers.max(axis=0)
    scene = [(low-[.020,.025,.040], np.array([high[0]+.035, high[1]+.025, low[2]-.012]))]
    grasps = []
    for center,cloud in zip(centers,clouds):
        plane = Plane(np.array([0.,0.,1.]), center-[0,0,.012], 0., 1.)
        grasps.append(flat_candidates(cloud, np.vstack(clouds), plane, surface_variants=True)['refined_grasps'])

    arms = [lab.model(current,side) for side in ('left','right')]
    visibility = [visible(arms[0],cloud) for cloud in clouds]
    audits, winners = {}, {}
    def probe(height):
        pair = [lab.model(height,side) for side in ('left','right')]
        left, right = [patch_paths(a,g,scene,obj,patch) for a,g,obj,patch in zip(pair,grasps,loads,patches)]
        reasons = Counter()
        audit = dict(available_paths=[len(left[0]),len(right[0])], arm_rejections=[left[1],right[1]])
        audits[height] = audit
        for l in left[0]:
            for r in right[0]:
                result = paired_path(pair,[l,r],loads,shared=shared)
                reasons[result['status']] += 1
                if result['status'] == 'PAIR_SCREEN_PASSED':
                    winners[height] = (pair,[l,r],result)
                    audit['pair_checks'] = dict(reasons)
                    return 'VIRTUAL_PATH_FOUND'
        audit['pair_checks'] = dict(reasons)
        return 'NO_COMPATIBLE_PAIR' if left[0] and right[0] else 'MISSING_ARM_PATH'

    selection = choose_height(arms,[SupportedContact(c[2]-.012,c[2]) for c in centers],probe)
    report = dict(name=name, mode='one_rigid_object' if shared else 'two_separate_objects',
                  centers_base_m=centers.tolist(), fully_in_camera_frame=visibility, height=selection,
                  audits=[dict(lift_m=h,**a) for h,a in audits.items()], hardware_commands=0,
                  both_patches_fully_in_frame=all(visibility), end_to_end_pick_verified=False,
                  observation_source='stipulated_fixture_contacts', physically_verified=False)
    if winners:
        arms, paths, sync = winners[selection['lift_m']]
        report['synchronous'] = sync
        report['late_right'] = paired_path(arms,paths,loads,shared=shared,lag_fraction=.2)
        report['missing_right_contact'] = paired_path(arms,paths,loads,shared=shared,contacts_verified=(True,False))
        if not shared:
            report['left_then_right'] = paired_path(arms,paths,loads,shared=False,schedule='left_then_right')
    return report


def floor_trial():
    arm = lab.model(0.)
    floor_z = arm.profile['ground_z_m']
    table = [(np.array([-1.,-1.,floor_z-.025]), np.array([1.,1.,floor_z]))]
    trials = []
    for x in (.20, .30, .40, .54):
        top = np.array([x,.10,floor_z+.012])
        points = patch_points(top,-5)
        plane = Plane(np.array([0.,0.,1.]), top-[0,0,.012], 0.,1.)
        candidates = flat_candidates(points,points,plane,surface_variants=True)['refined_grasps']
        pose = np.eye(4)
        pose[:3,:3] = Rotation.from_euler('z',-5,degrees=True).as_matrix()
        pose[:3,3] = top-[0,0,.006]
        payload = FixturePayload(pose,np.array([.014,.004,.006]))
        paths,reasons = patch_paths(arm,candidates,table,payload,payload)
        trials.append(dict(x_m=x, lift_m=arm.lift_m, fully_in_frame=visible(arm,points),
                           paths=len(paths), candidates=len(candidates), rejections=reasons))
    return dict(support_z_base_m=floor_z, face_height_above_floor_m=.012, trials=trials)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--study',choices=['height','arms','all'],default='all')
    a = p.parse_args()
    if a.output.exists():
        p.error('choose a new output file')
    result = dict(mode='offline_only',hardware_commands=0,motion_authorized=False,physical_task_completed=False)
    if a.study in ('all','height'):
        result['height_trials'] = [height_trial(layout) for layout in LAYOUTS]
        # Positive controls: move the established workspace vertically without tuning new XY poses.
        result['transferred_controls'] = [
            height_trial((*lab.CASES[i],z,.028,start)) for i,z,start in
            [(0,.865,.60),(2,.845,.58),(3,.852,.61),(4,.842,.58)]]
        result['floor'] = floor_trial()
    if a.study in ('all','arms'):
        result['arms'] = [
            pair_trial('separated_local_workspaces', [[.186,.109,.852],[.182,-.204,.852]],[-5,-5],.61,False),
            pair_trial('shared_end_tabs', [[.186,.109,.852],[.186,-.204,.852]],[-5,-5],.61,True),
            pair_trial('two_contacts_near_center', [[.22,.039,.836],[.22,-.039,.836]],[10,-10],.60,False),
        ]
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
    print(a.output)

if __name__ == '__main__':
    main()
