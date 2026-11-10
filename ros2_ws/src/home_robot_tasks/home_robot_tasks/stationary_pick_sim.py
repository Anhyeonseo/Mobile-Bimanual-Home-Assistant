"""L3C/D offline-only candidate evaluation with the nominal S2 arm models.

The virtual jaw convention follows yeoeun/perception-offline-lab 8da08b0,
config/grasp_ik.virtual.json: TCP +Z approaches, -X closes. No device imports.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull

from . import s2_geometry as geo, s2_development as dev
from . import so101_joint_coordinates as arm_coords
from .jaw_closure import closure_plan
from so101_arm_bridge.servo_position import single_turn_window
from .probe_evidence import STROKE_M, MAX_RISE_M, MAX_LATERAL_M

NAMES = geo.JOINTS[:5]
# Simulation assumptions, NOT measured installation limits.
GRASP_FROM_TCP = np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
LINK_RADIUS = .008
POS_TOL = .005
ANGLE_TOL = math.radians(10)
MAX_STEP = math.radians(2)
# Nominal final-insertion tolerances, in addition to the existing pose and collision checks.
APPROACH_SAMPLE_M = .002
APPROACH_LATERAL_M = .002
APPROACH_DRIFT_RAD = math.radians(2)


class Rejected(ValueError):
    pass


@dataclass
class FixturePayload:
    """Rectangular payload envelope, anchored in base_link at the moment of grasp."""
    base_pose: np.ndarray
    half_extents: np.ndarray


class StationaryArm:
    def __init__(self, profile, geometry, ranges, lift_m, jaw, side="left"):
        if side not in ("left", "right"):
            raise ValueError("unknown arm")
        self.jaw, self.side = jaw, side
        dev.validate(profile,geometry)
        if profile.get('arm_coordinate_convention') != arm_coords.CONVENTION:
            raise ValueError('stationary planning requires official SO-ARM101 model coordinates')
        if not 0 <= lift_m <= profile['lift']['travel_m']:
            raise ValueError('fixed lift outside model')
        arm=profile['arms'][side]
        self.profile=profile;self.lift_m=lift_m
        self.mount=geo.rigid(arm['base_from_arm_at_lift_zero']).copy()
        self.mount[:3,3]+=np.asarray(profile['lift']['axis_base'])*lift_m
        self.alignment = arm['alignment']
        if (ranges['zero_raw'] != 2048 or ranges['raw_units_per_turn'] != 4096
                or any(ranges['arms'][side][j]['positive_raw_direction'] != self.alignment[j]['raw_to_control_sign']
                       for j in geo.JOINTS)
                or ranges['arms'][side]['gripper']['opening_raw_direction'] != self.alignment['gripper']['sign']):
            raise ValueError('control ranges do not match the model coordinate convention')
        self.anchor=np.array([arm['reference_model_rad'][n] for n in NAMES])
        spec={j['name']:j for j in geometry['arm_joints']}
        self.joints=[];bounds=[];self.raw_windows={}
        for name,upstream in zip(NAMES,geo.UPSTREAM):
            j=spec[upstream];a=arm['alignment'][name];r=ranges['arms'][side][name]
            # Bound IK/MoveIt before searching, rather than discover an encoder
            # seam only after a complete route has been chosen for execution.
            self.raw_windows[name]=single_turn_window(r)
            limits=sorted(geo.model_angle_from_raw(raw, a, name)
                          for raw in self.raw_windows[name])
            bounds.append([max(float(j['limit']['lower']),limits[0]),min(float(j['limit']['upper']),limits[1])])
            self.joints.append((geo.origin(j),np.array([float(x) for x in j['axis']['xyz'].split()])))
        self.tcp=geo.origin(spec['gripper_frame_joint'])@geo.rigid(arm['reference_from_tcp'])
        self.tcp_from_stock = np.linalg.inv(geo.rigid(arm['reference_from_tcp']))
        self.lo,self.hi=np.array(bounds).T
        if np.any(self.lo>=self.hi) or np.any(self.anchor<self.lo) or np.any(self.anchor>self.hi):
            raise ValueError('no compatible Home and joint ranges')

    def fk(self,q):
        pose=self.mount.copy()
        points=[pose[:3,3].copy()]
        for (origin,axis),angle in zip(self.joints,q,strict=True):
            pose=pose@origin
            points.append(pose[:3,3].copy())
            pose[:3,:3]=pose[:3,:3]@Rotation.from_rotvec(axis*angle).as_matrix()
        pose=pose@self.tcp
        points.append(pose[:3,3].copy())
        return pose,np.array(points)

    def solve(self,goal,seed):
        # A seed is a numerical guess, not a measured state or requested goal.
        # Bring it into the solver domain and try the existing alternative seed
        # instead of failing a feasible grasp on SciPy's x0 bounds exception.
        seed=np.asarray(seed,dtype=float)
        if seed.shape!=(5,) or not np.isfinite(seed).all():
            raise ValueError('finite five-joint IK seed required')
        seed=np.clip(seed,self.lo,self.hi)
        def residual(q):
            actual=self.fk(q)[0]
            return np.r_[(actual[:3,3]-goal[:3,3])/POS_TOL,
                          Rotation.from_matrix(goal[:3,:3].T@actual[:3,:3]).as_rotvec()/ANGLE_TOL]
        for initial in (seed,(self.lo+self.hi)/2):
            q=least_squares(residual,initial,bounds=(self.lo,self.hi),max_nfev=100).x
            err=residual(q)
            if np.linalg.norm(err[:3])<=1 and np.linalg.norm(err[3:])<=1:
                return q,float(np.linalg.norm(err[:3])*POS_TOL),float(np.linalg.norm(err[3:])*ANGLE_TOL)
        raise Rejected('NO_POSE_SOLUTION')

    def segment(self,start,end,scene,carrying,width,depth, *, payload=None, scene_from_base=None, observed_scene=None, stage=None,
                approach_goals=None, approach_report=None, depth_gaps=None, margin_checks=None):
        """Sample continuous (not modulo) joint interpolation; only listed fixture volumes."""
        if stage == 'pick_approach' and approach_goals is None:
            raise Rejected('MISSING_PREGRASP')
        travel=max(abs(end-start))
        if travel>=math.pi:raise Rejected('LARGE_IK_BRANCH_CHANGE')
        steps=max(1,math.ceil(travel/MAX_STEP))
        if approach_goals is not None:
            pre, grip = approach_goals
            distance = np.linalg.norm(grip[:3,3] - pre[:3,3])
            steps = max(steps, math.ceil(distance / APPROACH_SAMPLE_M))
        joint_samples = np.linspace(start, end, steps + 1)
        frames = (self.fk(q) for q in joint_samples)
        if approach_goals is not None:
            # Retain the short insertion path so both checks inspect exactly the same samples.
            frames = list(frames)
            poses = np.array([pose for pose, _ in frames])
            report = straight_approach_report(poses, pre, grip)
            if approach_report is not None:
                approach_report.update(report)
            if report['reason']:
                raise Rejected(report['reason'])
        attached = None
        if carrying and payload is not None:
            attached = np.linalg.inv(self.fk(start)[0]) @ payload.base_pose
        if scene_from_base is not None:
            scene_from_base = geo.rigid(scene_from_base)
        box_array = np.asarray(scene, dtype=float).reshape(-1, 2, 3)
        box_low, box_high = box_array[:, 0], box_array[:, 1]
        for step, (pose, points) in enumerate(frames):
            if scene_from_base is not None:
                pose = scene_from_base @ pose
                points = points @ scene_from_base[:3,:3].T + scene_from_base[:3,3]
            for i, (a,b) in enumerate(zip(points,points[1:])):
                # The target stays in place until contact. The final tool segment enters its grasp volume.
                if observed_scene is not None and not carrying and (stage == 'pick_pregrasp' or i < len(points)-2):
                    lo, hi = observed_scene.target_bounds
                    if intersects(a, b, lo-LINK_RADIUS, hi+LINK_RADIUS):
                        raise Rejected('ARM_TARGET_INTERSECTION')
                nearby = np.all((box_high >= np.minimum(a,b)-LINK_RADIUS) & (box_low <= np.maximum(a,b)+LINK_RADIUS), axis=1)
                for lo,hi in box_array[nearby]:
                    if intersects(a,b,lo-LINK_RADIUS,hi+LINK_RADIUS):
                        raise Rejected('ARM_SCENE_INTERSECTION')
            # Enclose both pads throughout opening/closing (virtual dimensions).
            approach, closing_axis = pose[:3,2], -pose[:3,0]
            corners=np.array([pose[:3,3]+a*approach+b*closing_axis for a in (-depth,0)
                              for b in (self.jaw.fixed_face_m, self.jaw.fixed_face_m + width)])
            lo_j,hi_j=corners.min(axis=0)-.003,corners.max(axis=0)+.003
            if observed_scene is not None:
                closing = bool(carrying) or (stage == 'pick_approach' and step == steps)
                stock_pose = pose @ self.tcp_from_stock
                if self.jaw.may_touch(width, stock_pose, box_array, closing=closing):
                    if self.jaw.may_touch(width, stock_pose, box_array, closing=closing, margin=0.):
                        raise Rejected('JAW_SCENE_INTERSECTION')
                    if margin_checks is None:
                        raise Rejected('JAW_CLEARANCE_MARGIN')
                    margin_checks.append(step)
            elif np.any(np.all((hi_j >= box_low) & (lo_j <= box_high), axis=1)):
                raise Rejected('JAW_SCENE_INTERSECTION')
            if observed_scene is not None:
                if stage == 'pick_pregrasp':
                    lo, hi = observed_scene.target_bounds
                    if np.all(hi_j >= lo) and np.all(lo_j <= hi):
                        raise Rejected('JAW_TARGET_INTERSECTION')
                # The hand may start outside the camera. Verify the pregrasp endpoint and the whole final approach/retreat.
                if stage != 'pick_pregrasp' or step == steps:
                    observed_scene.check_visible(lo_j, hi_j, depth_gaps=depth_gaps)
            if attached is not None:
                box = pose @ attached
                center = box[:3, 3]
                span = np.abs(box[:3, :3]) @ payload.half_extents
                # Resting contact is permitted; positive overlap during lifting is not.
                if np.any(np.all((center+span > box_low+1e-9) & (center-span < box_high-1e-9), axis=1)):
                    raise Rejected('PAYLOAD_SCENE_INTERSECTION')
                if observed_scene is not None:
                    observed_scene.check_visible(center-span, center+span, depth_gaps=depth_gaps)
            elif carrying:
                # Payload box represented by a bounding sphere during loaded travel.
                for lo,hi in scene:
                    center = pose[:3,3] + closing_axis * self.jaw.center_from_fixed_m(width)
                    if np.linalg.norm(center-np.clip(center,lo,hi)) < carrying:
                        raise Rejected('PAYLOAD_SCENE_INTERSECTION')
        return steps


def straight_approach_report(poses, pre, grip):
    """Check sampled base-frame TCPs against the grasp axis, including roll and forward progress."""
    axis = grip[:3, 2]
    requested = grip[:3, 3] - pre[:3, 3]
    distance = float(requested @ axis)
    if (distance <= 0 or not np.allclose(requested, distance * axis, atol=1e-9, rtol=0)
            or not np.allclose(pre[:3, :3], grip[:3, :3], atol=1e-9, rtol=0)):
        raise Rejected('INVALID_STRAIGHT_APPROACH_GOALS')
    offset = poses[:, :3, 3] - pre[:3, 3]
    progress = offset @ axis
    lateral = float(np.max(np.linalg.norm(offset - progress[:, None] * axis, axis=1)))
    orientations = Rotation.from_matrix(poses[:, :3, :3])
    drift = float(np.max((orientations[0].inv() * orientations).magnitude()))
    heading = float(np.max((Rotation.from_matrix(grip[:3, :3]).inv() * orientations).magnitude()))
    reason = None
    if heading > ANGLE_TOL:
        reason = 'APPROACH_NOT_ALIGNED'
    elif drift > APPROACH_DRIFT_RAD:
        reason = 'APPROACH_ANGLE_CHANGED'
    elif lateral > APPROACH_LATERAL_M:
        reason = 'APPROACH_OFF_LINE'
    elif (abs(progress[0]) > POS_TOL or abs(progress[-1] - distance) > POS_TOL
          or progress[-1] <= progress[0] or np.min(np.diff(progress)) < -1e-9):
        reason = 'APPROACH_PROGRESS_INVALID'
    return dict(reason=reason, samples=len(poses), distance_m=distance, max_lateral_m=lateral,
                max_orientation_error_deg=math.degrees(heading), max_rotation_from_entry_deg=math.degrees(drift))


def intersects(a,b,lo,hi):
    """Segment vs closed axis-aligned box, using slab entry/exit parameters."""
    enter,leave=0.,1.
    for x,d,l,h in zip(a,b-a,lo,hi):
        if abs(d)<1e-12:
            if not l<=x<=h:return False
        else:
            u,v=sorted(((l-x)/d,(h-x)/d))
            enter=max(enter,u);leave=min(leave,v)
            if enter>leave:return False
    return True


def pad_tip_optical(candidate):
    """Resolve a raw GraspNet origin to our virtual pad-tip frame exactly once."""
    grasp = np.eye(4)
    grasp[:3, 3] = geo.vector(candidate['translation_m'], 3, 'grasp position')
    if grasp[2, 3] <= 0:
        raise Rejected('NONPOSITIVE_DEPTH')
    rotation = np.asarray(candidate['rotation_matrix'], dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('3x3 candidate rotation required')
    grasp[:3, :3] = rotation
    geo.rigid(grasp)
    grasp[:3, 3] += geo.number(candidate['depth_m'], 'depth', .5) * rotation[:, 0]
    return grasp


def check_contact_target(point, center, target_vertices_base=None):
    """Validate against the selected observed body, not a sphere around its mean.

    Legacy point-only captures retain the old 3 cm bound. A supplied body must
    be capture-bound by the caller; this never establishes path clearance.
    """
    if target_vertices_base is None:
        inside=np.linalg.norm(point-geo.vector(center,3,'observed point'))<=.03
    else:
        vertices=np.asarray(target_vertices_base,dtype=float)
        if vertices.ndim!=2 or vertices.shape[1]!=3 or not 4<=len(vertices)<=512 or not np.isfinite(vertices).all():
            raise ValueError('bounded observed target vertices required')
        hull=ConvexHull(vertices)
        inside=np.all(hull.equations[:,:3]@point+hull.equations[:,3]<=.003)
    if not inside:
        raise Rejected('CANDIDATE_OFF_TARGET')


def _bound_candidate(review, group, candidate, arm, *, target_vertices_base=None):
    if (review.get('mode') not in ('offline_capture_review','synthetic_selection')
            or group.get('perception_status')!='accepted' or review.get('status')!='TARGET_DISPLAY_ONLY' or review.get('executable') is not False
            or review.get('motion_authorized') is not False or review.get('frame')!='base_link'
            or review.get('calibration_id')!=arm.profile['calibration_id']):
        raise Rejected('OBSERVATION_NOT_ACCEPTED')
    # Producer must bind the group to its original observation and source files.
    # A matching class name or a legacy report's executable flag is insufficient.
    for key in ('capture_id','label','observation_index','source_sha256','session_id','calibration_id'):
        if key not in review or group.get(key)!=review[key]:
            raise Rejected('CANDIDATE_IDENTITY_MISMATCH')
    hashes=review['source_sha256']
    required_files=({'color.png','depth_aligned.png','metadata.json'} if review['mode']=='offline_capture_review'
                    else {'fixture_rgb','fixture_depth','fixture_metadata'})
    if (not isinstance(hashes,dict) or set(hashes)!=required_files or
            any(not isinstance(h,str) or len(h)!=64 or any(c not in '0123456789abcdef' for c in h) for h in hashes.values()) or
            type(review['observation_index']) is not int or review['observation_index']<0 or
            group.get('coordinate_frame')!=arm.profile['source_frame']):
        raise Rejected('CANDIDATE_FRAME_MISMATCH')
    if (candidate.get('source') in ('rule', 'graspnet', 'surface_variant')
            and group.get('tcp_convention') != 'graspnet_origin_plus_depth_to_pad_tip'):
        raise Rejected('UNKNOWN_GRASP_TCP_CONVENTION')
    width=geo.number(candidate['width_m'],'opening',.5)
    depth = geo.number(candidate['depth_m'], 'depth', .5)
    if not arm.jaw.accepts(width):
        raise Rejected('SO101_JAW_SECTION_LIMIT')
    if not 0 < depth <= .05:
        raise Rejected('INVALID_CANDIDATE_DEPTH')
    base = geo.rigid(arm.profile['base_from_optical']) @ pad_tip_optical(candidate)
    points=review['base_points_m']
    center=points.get('center_xyz_m',points.get('surface_xyz_m'))
    check_contact_target(base[:3,3],center,target_vertices_base)
    tcp = base.copy()
    tcp[:3, :3] = base[:3, :3] @ GRASP_FROM_TCP
    # Official TCP is on the fixed jaw. The requested centre is towards the moving jaw (-TCP X).
    tcp[:3, 3] += tcp[:3, 0] * arm.jaw.center_from_fixed_m(width)
    return tcp


def _pick_waypoints(grip, clearance):
    pre = grip.copy()
    pre[:3, 3] -= clearance * grip[:3, 2]
    up = grip.copy()
    up[2, 3] += clearance
    return [('pick_pregrasp', pre, False), ('pick_approach', grip, False),
            ('pick_retreat', up, True)]


def _place_waypoints(grip, destination, yaw, clearance, radius, support, jaw_offset):
    lo, hi = support
    if (np.any(destination[:2] - radius < lo[:2]) or np.any(destination[:2] + radius > hi[:2])
            or not 0 <= destination[2] - radius - hi[2] <= .012):
        raise Rejected('NO_PLACE_SUPPORT')
    drop = grip.copy()
    drop[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix() @ grip[:3, :3]
    drop[:3, 3] = destination + drop[:3, 0]*jaw_offset
    above = drop.copy()
    above[2, 3] += clearance
    return [('place_preplace', above, True), ('place_approach', drop, True),
            ('place_retreat', above, False)]


def _probe_prefix(arm, start, end):
    """Select a bounded prefix of the already solved loaded retreat, on the same IK branch."""
    origin = arm.fk(start)[0][:3, 3]
    if arm.fk(end)[0][2, 3] - origin[2] < STROKE_M:
        raise Rejected('TEST_LIFT_TOO_SHORT')
    low, high = 0., 1.
    for _ in range(16):
        fraction = (low + high) / 2
        if arm.fk(start + fraction * (end - start))[0][2, 3] - origin[2] < STROKE_M:
            low = fraction
        else:
            high = fraction
    goal = start + high * (end - start)
    xyz = np.array([arm.fk(q)[0][:3, 3] - origin for q in np.linspace(start, goal, 11)])
    lateral = float(max(np.linalg.norm(xyz[:, :2], axis=1)))
    if np.min(np.diff(xyz[:, 2])) < -1e-9 or max(xyz[:, 2]) > MAX_RISE_M or lateral > MAX_LATERAL_M:
        raise Rejected('TEST_LIFT_OUTSIDE_STROKE')
    return goal, dict(rise_m=float(xyz[-1, 2]), max_lateral_m=lateral, model_rad=goal.tolist())


def check_stages(arm, seed, waypoints, boxes, width, depth, payload_radius, payload, attempt, *, scene_from_base=None, observed_scene=None, include_probe=True):
    """Solve and sample a complete nominal path, retaining the first failed stage."""
    pregrasp = None
    for stage, goal, loaded in waypoints:
        attempt['checking'] = stage
        approach_goals = None
        if stage == 'pick_pregrasp':
            pregrasp = goal
        elif stage == 'pick_approach':
            if pregrasp is None:
                raise Rejected('MISSING_PREGRASP')
            approach_goals = (pregrasp, goal)
            attempt['straight_approach'] = {}
        q, metres, radians = arm.solve(goal, seed)
        gaps = []
        margin_checks = []
        try:
            if stage == 'pick_retreat' and include_probe:
                probe_q, probe_record = _probe_prefix(arm, seed, q)
                probe_record['path_steps'] = arm.segment(
                    seed, probe_q, boxes, payload_radius, width, depth, payload=payload,
                    scene_from_base=scene_from_base, observed_scene=observed_scene, stage=stage,
                    depth_gaps=gaps, margin_checks=margin_checks)
                attempt['test_lift'] = probe_record
            count = arm.segment(seed, q, boxes, payload_radius if loaded else 0.,
                                width, depth, payload=payload, scene_from_base=scene_from_base,
                                observed_scene=observed_scene, stage=stage, depth_gaps=gaps, margin_checks=margin_checks,
                                approach_goals=approach_goals, approach_report=attempt.get('straight_approach'))
        finally:
            if gaps:
                worst = max(gaps, key=lambda sample: sample['missing_pixels'] / sample['checked_pixels'])
                attempt.setdefault('unobserved_depth', {})[stage] = dict(checks_with_gaps=len(gaps), worst_sample=worst)
            if margin_checks:
                attempt.setdefault('jaw_margin_samples', {})[stage] = margin_checks
        control_rad = {j:geo.control_from_model(float(angle), arm.alignment[j])
                       for j, angle in zip(NAMES, q, strict=True)}
        attempt['stages'].append(dict(stage=stage, model_rad=q.tolist(), control_rad=control_rad, goal_base=goal.tolist(),
                                      position_error_m=metres, orientation_error_deg=math.degrees(radians),
                                      path_steps=count))
        seed = q
    if attempt.get('unobserved_depth') or attempt.get('jaw_margin_samples'):
        # All modeled stages were checked, but this candidate cannot be selected for use.
        attempt['geometric_path_requires_confirmation'] = True
        if attempt.get('unobserved_depth'):
            attempt['checking'] = next(iter(attempt['unobserved_depth']))
            raise Rejected('GRIPPER_OR_PAYLOAD_DEPTH_UNKNOWN')
        attempt['checking'] = next(iter(attempt['jaw_margin_samples']))
        raise Rejected('JAW_CLEARANCE_MARGIN')


def _evaluate_paths(arm, review, group, scene, *, lift_clearance, payload_radius,
                    destination=None, yaws=(0,), payload=None, scene_from_base=None, observed_scene=None, installed_left_jaw=None):
    if installed_left_jaw is not None and (arm.side != 'left' or destination is not None):
        raise ValueError('installed left jaw calibration applies to left-arm picks')
    if not .005 <= lift_clearance <= .1 or not 0 < payload_radius <= .1:
        raise ValueError('invalid simulated clearances')
    boxes = []
    for lo, hi in scene:
        lo, hi = geo.vector(lo, 3, 'box min'), geo.vector(hi, 3, 'box max')
        if np.any(lo >= hi):
            raise ValueError('empty scene box')
        boxes.append((lo, hi))
    if not boxes:
        raise ValueError('observed workspace geometry required')
    candidates = group['refined_grasps']
    if not isinstance(candidates, list) or len(candidates) > 64:
        raise ValueError('bounded grasp candidates required')
    attempts = []
    result = dict(mode='nominal_stationary_simulation', arm_coordinate_convention=arm_coords.CONVENTION,
                  task='pick_lift' if destination is None else 'pick_place',
                  motion_authorized=False, executable=False, physical_task_completed=False,
                  stock_jaw_section=arm.jaw.describe(),
                  full_robot_collision_verified=False, self_collision_checked=False,
                  continuous_collision_proof=False, capture_id=review['capture_id'], attempts=attempts)
    if (destination is None and arm.side == 'left' and review.get('mode') == 'offline_capture_review'
            and installed_left_jaw is None):
        return dict(result,status='LEFT_PAD_CALIBRATION_REQUIRED')
    for index, candidate in enumerate(candidates):
        for yaw in yaws:
            attempt = dict(index=index, stages=[])
            attempts.append(attempt)
            try:
                grip = _bound_candidate(review, group, candidate, arm)
                width = candidate['width_m']
                if destination is None:
                    closure = closure_plan(arm.jaw, candidate)
                    attempt['jaw_closure'] = closure
                    if installed_left_jaw is not None:
                        pad_bounds = installed_left_jaw.preview(closure)
                        attempt['left_installed_pad_preview'] = pad_bounds
                        if pad_bounds['status'] != 'MEASURED_GAP_PREVIEW':
                            raise Rejected(pad_bounds['status'])
                    if 'closing_section' in candidate or review['mode'] == 'offline_capture_review':
                        if closure['status'] != 'BOUNDED_CLOSE':
                            raise Rejected(closure['status'])
                        width = closure['entry_gap_m']
                        if width != candidate['width_m']:
                            # The previously bound optical grasp centre stays fixed. Widen
                            # the opening towards the moving jaw before checking the new path.
                            shift = arm.jaw.center_from_fixed_m(width) - arm.jaw.center_from_fixed_m(candidate['width_m'])
                            grip[:3, 3] += grip[:3, 0] * shift
                waypoints = _pick_waypoints(grip, lift_clearance)
                if destination is not None:
                    attempt['place_yaw_offset_deg'] = yaw
                    waypoints.extend(_place_waypoints(grip, destination, yaw, lift_clearance,
                                                      payload_radius, boxes[0],
                                                      arm.jaw.center_from_fixed_m(candidate['width_m'])))
                if review.get('lift_height_m') != arm.lift_m:
                    raise Rejected('LIFT_ANCHOR_MISMATCH')
                seed = np.array([geo.number(review[arm.side + '_anchor_rad'][n], n, 7) for n in NAMES])
                if np.any(seed < arm.lo) or np.any(seed > arm.hi):
                    raise Rejected('ANCHOR_OUTSIDE_BOUNDS')
                attempt['anchor_rad'] = seed.tolist()
                check_stages(arm, seed, waypoints, boxes, width, candidate['depth_m'],
                             payload_radius, payload, attempt, scene_from_base=scene_from_base,
                             observed_scene=observed_scene, include_probe=destination is None)
                attempt['status'] = 'VIRTUAL_PATH_FOUND'
                if destination is not None:
                    lo, hi = boxes[0]
                    result.update(support_top_m=float(hi[2]), support_box_m=[lo.tolist(), hi.tolist()])
                return dict(result, status=attempt['status'], candidate_index=index)
            except (Rejected, ValueError, KeyError, TypeError) as exc:
                attempt.update(status='REJECTED', reason=str(exc))
                if not attempt.get('checking', '').startswith('place_'):
                    break
    if any(a.get('geometric_path_requires_confirmation') for a in attempts):
        return dict(result, status='PATH_REQUIRES_GRIPPER_CLEARANCE_CONFIRMATION')
    return dict(result, status='NO_VIRTUAL_PATH')


def plan_pick(arm, review, group, scene, *, lift_clearance=.02, payload_radius=.009, payload=None,
              observed_scene=None, installed_left_jaw=None):
    """Evaluate the path through loaded vertical retreat at the current base pose."""
    transform = None
    if observed_scene is not None:
        transform = observed_scene.support_from_optical @ np.linalg.inv(geo.rigid(arm.profile['base_from_optical']))
        low, high = observed_scene.target_bounds
        pose = np.eye(4)
        pose[:3, 3] = (low + high)/2
        payload = FixturePayload(np.linalg.inv(transform) @ pose, (high-low)/2)
        scene = observed_scene.support_boxes + observed_scene.obstacle_boxes
    return _evaluate_paths(arm, review, group, scene, lift_clearance=lift_clearance, scene_from_base=transform,
                           observed_scene=observed_scene, payload_radius=payload_radius, payload=payload,
                           installed_left_jaw=installed_left_jaw)


def plan(arm, review, group, place_xyz, scene, *, lift_clearance=.02, payload_radius=.009,
         place_yaw_offsets_deg=(0,), place_xy_half_extent_m=0.):
    """Default is an exact destination. Only vary XY inside an explicitly allowed square."""
    center=geo.vector(place_xyz,3,'place TCP')
    h=geo.number(place_xy_half_extent_m,'place region half extent',.05)
    if h<0:raise ValueError('negative place region')
    offsets=[(0.,0.)]
    inner=max(0.,h-POS_TOL)  # leave pose tolerance inside the caller's region
    if inner:
        offsets.extend([(0,inner),(0,-inner),(inner,0),(-inner,0),(inner,inner),
                        (inner,-inner),(-inner,inner),(-inner,-inner)])
    if not 1 <= len(place_yaw_offsets_deg) <= 9:
        raise ValueError('bounded, explicitly allowed placement yaw offsets required')
    yaws = [geo.number(v, 'place yaw', 180) for v in place_yaw_offsets_deg]
    searched=[]
    for x,y in offsets:
        dest=(center+[x,y,0]).tolist()
        out = _evaluate_paths(arm, review, group, scene, lift_clearance=lift_clearance,
                              payload_radius=payload_radius, destination=np.asarray(dest), yaws=yaws)
        searched.append({'destination_m':dest,'status':out['status'],
                         'rejections':[a.get('reason') for a in out['attempts'] if a['status']=='REJECTED']})
        if out['status']=='VIRTUAL_PATH_FOUND':break
        if not any(a.get('checking','').startswith('place_') or a.get('reason')=='NO_PLACE_SUPPORT'
                   for a in out['attempts']):break
    return dict(out, destinations_tested=searched, requested_place_xyz_m=center.tolist(),
                place_xy_half_extent_m=h, chosen_place_xyz_m=dest if out['status']=='VIRTUAL_PATH_FOUND' else None)
