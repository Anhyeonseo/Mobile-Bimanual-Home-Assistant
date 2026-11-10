#!/usr/bin/env python3
"""Screen base stops against a fixed tabletop at one lift height. Entirely synthetic."""
import argparse
from collections import Counter
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.stats import qmc

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks'), str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_perception.geometry.plane import Plane
from home_perception.grasp.handoff import flat_candidates
from home_robot_tasks.assistant.contracts import Context, Stamp
from home_robot_tasks.assistant.placement import Candidate, Check, PICK_LIFT_PHASES, PlacementPlanner, PlacementProfile
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.navigation import NavigateRequest, NavigationMap
from home_robot_tasks.stationary_lift import LiftHeightRule
from home_robot_tasks.stationary_pair import _hand, paired_path
from home_robot_tasks.stationary_pick_sim import NAMES, POS_TOL, Rejected, FixturePayload
from tools.diagnostics import run_s3_lift_and_arms as trials, run_s3_stationary_lab as lab


START = (-.75, 0.)
TABLE_XY = ((0., -.50), (.50, .50))
DISTANCES = (.21, .275, .295, .32, .36)
AZIMUTHS = tuple(map(math.radians,(150, 180, -150)))
HEADINGS = tuple(map(math.radians,(-55,-35,-20,0,20,35,55)))


@dataclass(frozen=True)
class Layout:
    name: str
    support_m: float
    # Map coordinates (X, Y, yaw deg) of 28 × 8 × 12 mm objects on the table.
    objects: tuple


LAYOUTS = (
    Layout('high_edge', .91, ((.018, .10, -10.),)),
    Layout('high_inset', .91, ((.12, .10, -10.),)),
    Layout('lower_edge', .75, ((.018, .10, -10.),)),
    Layout('pair_close', .91, ((.018, .04, -10.), (.018, -.04, 12.))),
    Layout('pair_wide', .91, ((.018, .155, -10.), (.018, -.155, 12.))),
)


def world_from_base(pose, ground_z):
    transform = np.eye(4)
    transform[:3,:3] = Rotation.from_euler('z',pose[2]).as_matrix()
    transform[:3,3] = [pose[0],pose[1],-ground_z]
    return transform


def points_in(transform, points):
    return points @ transform[:3,:3].T + transform[:3,3]


def navigation_map(boxes):
    # Use the existing simulation footprint/clearance. Occupy the tabletop XY projection even above lidar.
    data = lab.read(ROOT/'config/navigation_map.simulation.json')
    res = .02
    xs, ys = np.meshgrid(-1.2+(np.arange(120)+.5)*res, -1.+(np.arange(100)+.5)*res)
    occupied = np.zeros(xs.shape,dtype=bool)
    for lo,hi in boxes:
        occupied |= (xs+res/2 > lo[0]) & (xs-res/2 < hi[0]) & (ys+res/2 > lo[1]) & (ys-res/2 < hi[1])
    data.update(map_id='stop-lab', revision='fixture-1', origin=dict(x=-1.2,y=-1.), resolution_m=res,
                rows=[''.join('#' if p else '.' for p in row) for row in occupied])
    return NavigationMap(data)


def blocked_rays(origin, points, boxes):
    """Whether any finite segment to a patch intersects a scene box before reaching it."""
    directions = points-origin
    for lo,hi in boxes:
        enter = np.zeros(len(points))
        leave = np.full(len(points), 1.-1e-8)
        valid = np.ones(len(points), dtype=bool)
        for axis in range(3):
            d = directions[:,axis]
            moving = abs(d) > 1e-12
            if not lo[axis] <= origin[axis] <= hi[axis]:
                valid &= moving
            a = (lo[axis]-origin[axis])/d[moving]
            b = (hi[axis]-origin[axis])/d[moving]
            enter[moving] = np.maximum(enter[moving], np.minimum(a,b))
            leave[moving] = np.minimum(leave[moving], np.maximum(a,b))
        if np.any(valid & (enter <= leave)):
            return True
    return False


def capture_patches(camera, intr, patches, boxes):
    """Project full top faces, reject occlusion and backproject unique RGB-D pixels in metres."""
    optical = np.linalg.inv(camera)
    seen = set()
    observed = []
    min_margin = 1.
    for points in patches:
        xyz = points_in(optical,points)
        if np.min(xyz[:,2]) <= 0:
            return 'BEHIND_CAMERA', [], 0.
        u = np.rint(intr['ppx']+intr['fx']*xyz[:,0]/xyz[:,2]).astype(int)
        v = np.rint(intr['ppy']+intr['fy']*xyz[:,1]/xyz[:,2]).astype(int)
        margin = min(u.min(), v.min(), intr['width']-1-u.max(), intr['height']-1-v.max())
        if margin < 0:
            return 'PATCH_OUTSIDE_FRAME', [], 0.
        if blocked_rays(camera[:3,3], points, boxes):
            return 'PATCH_OCCLUDED', [], 0.
        codes = v*intr['width']+u
        order = np.argsort(xyz[:,2], kind='stable')
        unique, which = np.unique(codes[order], return_index=True)
        if seen.intersection(unique):
            return 'PATCHES_OVERLAP_IN_IMAGE', [], 0.
        seen.update(unique)
        idx = order[which]
        z = np.rint(xyz[idx,2]*1000)/1000
        samples = np.column_stack(((u[idx]-intr['ppx'])*z/intr['fx'],
                                   (v[idx]-intr['ppy'])*z/intr['fy'],z))
        observed.append(points_in(camera,samples))
        min_margin = min(min_margin, float(margin)/(min(intr['width'],intr['height'])/2))
    return 'VISIBLE', observed, min_margin


class StopChecks:
    def __init__(self, layout, arms):
        self.arms = arms
        self.surface = layout.support_m
        self.centers = np.array([[x,y,self.surface+.012] for x,y,_ in layout.objects])
        self.patches = [trials.patch_points(center,obj[2]) for center,obj in zip(self.centers,layout.objects)]
        self.bodies = []
        for center, (_,_,yaw) in zip(self.centers, layout.objects):
            pose = np.eye(4)
            pose[:3,:3] = Rotation.from_euler('z',yaw,degrees=True).as_matrix()
            pose[:3,3] = center-[0,0,.006]
            self.bodies.append(FixturePayload(pose,np.array([.014,.004,.006])))
        self.boxes = [(np.array([*TABLE_XY[0],self.surface-.03]),np.array([*TABLE_XY[1],self.surface]))]
        self.map = navigation_map(self.boxes)
        self.intr = lab.read(lab.CONFIG/'camera_device.observed.json')['sdk_calibration']['color_intrinsics']
        self.routes, self.views, self.witnesses = {}, {}, {}
        self.arm_rejections = Counter()
        self.motion_calls = 0
        self.motion_status = {}

    def _candidates(self, pose):
        base = np.linalg.inv(self._transform(pose))
        patches = [points_in(base,p) for p in self.views[pose][1]]
        scene_points = np.vstack(patches)
        plane = Plane(base[:3,:3] @ [0.,0.,1.], points_in(base,np.array([0.,0.,self.surface])), 0.,1.)
        return [flat_candidates(p,scene_points,plane,surface_variants=True)['refined_grasps'] for p in patches]

    def _transform(self, pose):
        return world_from_base(pose,self.arms['left'].profile['ground_z_m'])

    def navigation(self, candidate):
        x,y,yaw = candidate.base_xy_yaw
        if (x,y) not in self.routes:
            request = NavigateRequest(candidate.candidate_id,self.map.map_id,self.map.revision,self.map.floor_id,
                                      self.map.frame_id,x,y,self.map.floor_z,yaw)
            try:
                route = self.map.plan(request,START)
                points = [START,*[(p['x'],p['y']) for p in route['path_xy']],(x,y)]
                cost = sum(math.dist(a,b) for a,b in zip(points,points[1:]))
                check = Check(True,'GRID_ROUTE',cost=cost,mode='simulation')
            except InvalidTask as exc:
                check = Check(False,str(exc),mode='simulation')
            self.routes[x,y] = check
        return self.routes[x,y]

    def view(self, candidate):
        stop = candidate.base_xy_yaw
        if stop not in self.views:
            world = self._transform(stop)
            camera = world @ self.arms['left'].profile['base_from_optical']
            status, points, margin = capture_patches(camera,self.intr,self.patches,self.boxes)
            self.views[stop] = (Check(status=='VISIBLE',status,margin=margin,mode='simulation'), points)
        return self.views[stop][0]

    def whole_motion(self, candidate):
        self.motion_calls += 1
        outcome = self._check_motion(candidate)
        self.motion_status[(candidate.base_xy_yaw, candidate.arm)] = outcome.reason
        return outcome

    def _check_motion(self, candidate):
        pose = candidate.base_xy_yaw
        world = self._transform(pose)
        base = np.linalg.inv(world)
        sides = [candidate.arm]
        other = 'right' if sides[0]=='left' else 'left'
        if len(self.bodies) == 2:
            sides.append(other)
        boxes = list(self.boxes)
        if len(sides) == 1:
            # Include the idle arm's modeled links and open jaw when the active arm travels.
            parked = self.arms[other]
            _, parked_boxes = _hand(parked,parked.anchor,float(parked.jaw.gaps[-1]),.02)
            for lo,hi in parked_boxes:
                center = points_in(world,(lo+hi)/2)
                span = np.abs(world[:3,:3]) @ ((hi-lo)/2)
                boxes.append((center-span,center+span))
        paths, bodies = {}, {}
        candidates = self._candidates(pose)
        for i,side in enumerate(sides):
            hypotheses = candidates[i]
            if not hypotheses:
                return Check(False, 'NO_GRASP_CANDIDATE',mode='simulation')
            body = FixturePayload(base @ self.bodies[i].base_pose, self.bodies[i].half_extents)
            paths[side], reasons = trials.patch_paths(self.arms[side],hypotheses,boxes,body,body,
                                                       scene_from_base=world,first_only=len(sides)==1)
            self.arm_rejections.update(reasons)
            if not paths[side]:
                return Check(False, side.upper()+'_NO_PATH_OR_HOLD',mode='simulation')
            bodies[side] = body
        if len(sides) == 2:
            for left in paths['left']:
                for right in paths['right']:
                    pair = paired_path([self.arms[s] for s in ('left','right')],[left,right],
                                       [bodies[s] for s in ('left','right')],shared=False,schedule='left_then_right')
                    if pair['status'] == 'PAIR_SCREEN_PASSED':
                        self.witnesses[candidate.candidate_id] = dict(assigned_arms=sides,
                            grasp_indices=[left['candidate'],right['candidate']], schedule=pair['schedule'])
                        return Check(True,'PAIR_HELD',phases=tuple(sorted(PICK_LIFT_PHASES)),mode='simulation')
                    self.arm_rejections[pair['status']] += 1
            return Check(False,'NO_COMPATIBLE_STAGED_PATH',mode='simulation')
        self.witnesses[candidate.candidate_id] = dict(assigned_arms=sides, grasp_indices=[paths[sides[0]][0]['candidate']])
        return Check(True,'SINGLE_HELD',phases=tuple(sorted(PICK_LIFT_PHASES)),mode='simulation')


def sampled_tcp_positions(arm, joints):
    """Batch forward kinematics for an independent reach diagnostic, without IK or collision claims."""
    poses = np.broadcast_to(arm.mount,(len(joints),4,4)).copy()
    for i,(origin,axis) in enumerate(arm.joints):
        poses = poses @ origin
        poses[:,:3,:3] = poses[:,:3,:3] @ Rotation.from_rotvec(joints[:,i,None]*axis).as_matrix()
    return (poses @ arm.tcp)[:,:3,3]


def forward_bound(arm, limit_x, *, max_cells=100000):
    """Bound TCP X over every allowed joint angle; fail inconclusive if the budget is spent.

    From the midpoint of each joint-angle cell, changing one joint at a time
    displaces the TCP by at most the chord on its downstream bounding sphere.
    Summing these chords bounds X displacement for the whole cell.
    """
    basis = dict(scope='nominal_fk_over_supplied_joint_intervals', physical_reach_verified=False,
                 frame='base_link', arm_side=arm.side, lift_travel_m=arm.lift_m, limit_x_m=limit_x,
                 joint_intervals_deg={j: np.degrees([lo, hi]).tolist()
                                      for j, lo, hi in zip(NAMES, arm.lo, arm.hi, strict=True)})
    lengths = [np.linalg.norm(origin[:3,3]) for origin,_ in arm.joints]
    axes = np.array([axis for _,axis in arm.joints])
    rates = np.linalg.norm(axes,axis=1)
    end = arm.tcp[:3,3]
    radii = np.array([np.linalg.norm(end)+sum(lengths[i+1:]) for i in range(len(lengths))])
    radii[-1] = np.linalg.norm(np.cross(axes[-1]/rates[-1], end))
    centers = np.array([(arm.lo+arm.hi)/2])
    half = np.array([(arm.hi-arm.lo)/2])
    tested = 0
    while len(centers):
        if tested+len(centers) > max_cells:
            return dict(basis, status='INCONCLUSIVE', cells=tested)
        tested += len(centers)
        x = sampled_tcp_positions(arm, centers)[:,0]
        if np.any(x >= limit_x):
            return dict(basis, status='FK_WITNESS', cells=tested)
        chords = 2*radii*np.sin(np.minimum(rates*half, math.pi)/2)
        possible = x+chords.sum(axis=1)+1e-9 >= limit_x
        centers, half = centers[possible], half[possible]
        if len(centers):
            index = np.arange(len(centers))
            axis = chords[possible].argmax(axis=1)
            half[index,axis] /= 2
            offset = np.zeros_like(half)
            offset[index,axis] = half[index,axis]
            centers = np.vstack((centers-offset,centers+offset))
            half = np.vstack((half,half))
    return dict(basis, status='ALL_TCP_X_BELOW_LIMIT', cells=tested)


def position_only_screen(checks):
    """Contrast IK on the same pregrasp with/without orientation. No trajectory is accepted here."""
    totals = Counter()
    targets = {side: [] for side in checks.arms}
    for pose,(view,_) in checks.views.items():
        if not view.valid:
            continue
        candidates = checks._candidates(pose)[0]
        for arm in checks.arms.values():
            xyz_ok = pose_ok = False
            for c in candidates:
                if not arm.jaw.accepts(c['width_m']):
                    continue
                goal = trials.patch_goal(arm,c)
                goal[:3,3] -= .02*goal[:3,2]
                targets[arm.side].append(goal[:3,3].copy())
                if not xyz_ok:
                    def residual(q):
                        return arm.fk(q)[0][:3,3] - goal[:3,3]
                    for seed in (arm.anchor, (arm.lo+arm.hi)/2):
                        q = least_squares(residual,seed,bounds=(arm.lo,arm.hi),max_nfev=100).x
                        if np.linalg.norm(residual(q)) <= POS_TOL:
                            xyz_ok = True
                            break
                if not pose_ok:
                    try:
                        arm.solve(goal,arm.anchor)
                        pose_ok = xyz_ok = True
                    except Rejected:
                        pass
                if pose_ok:
                    break
            totals['arm_stop_pairs'] += 1
            totals['pregrasp_xyz_only'] += int(xyz_ok)
            totals['pregrasp_xyz_and_rotation'] += int(pose_ok)
    sampled = {}
    for side, goals in targets.items():
        if not goals:
            continue
        xyz = np.asarray(goals)
        arm = checks.arms[side]
        unit = qmc.Sobol(5,scramble=True,seed=418).random_base2(15)
        positions = sampled_tcp_positions(arm, arm.lo + unit*(arm.hi-arm.lo))
        lo, hi = float(xyz[:,2].min()), float(xyz[:,2].max())
        within_z = positions[(positions[:,2] >= lo) & (positions[:,2] <= hi)]
        sampled[side] = dict(forward_bound=forward_bound(arm,.25), samples=len(positions), between_goal_z=len(within_z), goal_z_range_m=[lo,hi],
                             minimum_required_tcp_x_m=float(xyz[:,0].min()),
                             maximum_sampled_tcp_x_m=float(within_z[:,0].max()) if len(within_z) else None)
    return dict(totals, sampled_reach=sampled)


def front_edge_sweep(checks, lift):
    # Refine near the accessible front edge, without changing the map, lift, arm or grasp policy.
    counts = Counter(stops=0, reachable=0, visible=0, path_and_hold=0)
    selected = None
    for x in (-.262,-.282,-.312):
        for y in (.05,.075,.10,.125,.15,.175,.20,.225,.25):
            for deg in range(-45,46,5):
                pose = (x,y,math.radians(deg))
                c = Candidate('supplemental',pose,lift,'left','tabletop-surface-rule','pick_lift')
                counts['stops'] += 1
                if not checks.navigation(c).valid:
                    continue
                counts['reachable'] += 1
                if not checks.view(c).valid:
                    continue
                counts['visible'] += 1
                if checks.whole_motion(c).valid:
                    counts['path_and_hold'] += 1
                    if selected is None:
                        selected = dict(stop=pose, **checks.witnesses[c.candidate_id])
    return dict(counts=dict(counts), first=selected, tested_arm='left')


def run_layout(layout):
    rule = LiftHeightRule.from_profile(lab.read(lab.CONFIG/'s2_coordinates.development.json'))
    setting = rule.setting(layout.support_m)
    lift = setting['target_travel_from_bottom_m']
    checks = StopChecks(layout,{side:lab.model(lift,side) for side in ('left','right')})
    profile = PlacementProfile(0.,rule.travel_m,(lift,),DISTANCES,AZIMUTHS,HEADINGS,('left','right'))
    context = Context(checks.map.map_id, checks.map.revision, checks.arms['left'].profile['calibration_id'],
                      'fixture-clock',checks.map.frame_id)
    planner = PlacementPlanner(profile,checks)
    plan = planner.plan(target_m=checks.centers.mean(axis=0).tolist(),stamp=Stamp(1.,context,layout.name),
                        context=context,now_s=1.,scene_revision=layout.name,grasp_ids=['tabletop-surface-rule'],
                        current_xy_yaw=(*START,0.),current_height_m=lift,operation='pick_lift')
    selected = plan['candidates'][:1]
    best = None
    if selected:
        c = selected[0]
        best = dict(c, **checks.witnesses[c['candidate_id']])
    stops = []
    for (x,y), nav in checks.routes.items():
        facing = math.atan2(checks.centers[:,1].mean()-y,checks.centers[:,0].mean()-x)
        for offset in HEADINGS:
            yaw = math.atan2(math.sin(facing+offset),math.cos(facing+offset))
            pose = (x,y,yaw)
            view = checks.views.get(pose)
            stops.append(dict(base_xy_yaw=pose, navigation=nav.reason, view=view[0].reason if view else None,
                              arms={s:checks.motion_status[pose,s] for s in checks.arms if (pose,s) in checks.motion_status}))
    record = dict(layout=layout.name, objects=layout.objects, support_height_m=layout.support_m,
                lift_setting=setting, evaluated=plan['evaluated'], distinct_stops=len(checks.routes)*len(HEADINGS),
                reachable_stops=sum(c.valid for c in checks.routes.values())*len(HEADINGS),
                fully_visible_reachable_stops=sum(c.valid for c,_ in checks.views.values()),
                motion_evaluations=checks.motion_calls, accepted_arm_assignments=len(plan['candidates']), best=best,
                stage_rejections=plan['rejected'], arm_rejections=dict(checks.arm_rejections), stops=stops,
                requires_reobservation=True, motion_authorized=False)
    if layout.name == 'high_edge':
        record['diagnostic_pregrasp'] = position_only_screen(checks)
        record['finer_front_row'] = front_edge_sweep(checks, lift)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('choose a new output file')
    report = dict(mode='synthetic_stationary_stop_screen', hardware_commands=0, physical_task_completed=False,
                  motion_authorized=False, start_xy_m=START, table_xy_m=TABLE_XY,
                  map_radius_m=lab.read(ROOT/'config/navigation_map.simulation.json')['robot_radius_m'],
                  map_clearance_m=lab.read(ROOT/'config/navigation_map.simulation.json')['clearance_m'],
                  resolution_m=.02, physical_footprint_verified=False, detection_measured=False,
                  distances_m=DISTANCES, azimuths_deg=np.degrees(AZIMUTHS).tolist(),
                  heading_offsets_deg=np.degrees(HEADINGS).tolist(), layouts=[])
    for layout in LAYOUTS:
        row = run_layout(layout)
        report['layouts'].append(row)
        print(layout.name,row['reachable_stops'],row['fully_visible_reachable_stops'],row['accepted_arm_assignments'],flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as out:
        json.dump(report,out,indent=2,allow_nan=False)


if __name__ == '__main__':
    main()
