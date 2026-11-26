"""L3D stage coordinator: jointly select a grasp and a reachable carry goal.

The injected planner owns collision checking and trajectories. This module has
no transport or execution interface. One fixed lift and current scene per call.
"""
from dataclasses import dataclass, field, replace
from functools import cached_property

import numpy as np

from .carry_posture import carry_goals, checked_joints, CarryPolicy
from .pick_table_clearance import TableClearanceRejected
from .s2_geometry import rigid
from .carry_load import CarriedGeometry, payload_load, grasp_balance
from .grasp_preferences import grasp_preferences, contact_posture, tabletop_pinch_alignment


class PlanRejected(ValueError):
    pass


@dataclass(frozen=True)
class GraspRouteCandidate:
    candidate_id: str
    calibration_id: str
    scene_id: str
    pregrasp_model_rad: np.ndarray
    base_from_grasp_tcp: np.ndarray
    tcp_from_object: np.ndarray
    object_size_m: tuple
    open_jaw_model_rad: float
    held_jaw_model_rad: float
    object_constraint: object = None
    payload_mesh: object = None
    # Original observation stays fixed until closing; attachment predicts the
    # small displacement into the fixed finger. Neither is a measured grasp.
    observed_base_from_object: object = None
    fixed_side_clearance_m: float = 0.

    def __post_init__(self):
        if not all((self.candidate_id, self.calibration_id, self.scene_id)):
            raise ValueError('candidate identity and scene binding required')
        rigid(self.base_from_grasp_tcp); rigid(self.tcp_from_object)
        if not np.isfinite(self.fixed_side_clearance_m) or not 0<=self.fixed_side_clearance_m<=.010:
            raise ValueError('fixed-side entry clearance must be within 0..10 mm')
        if self.observed_base_from_object is not None:
            observed=rigid(self.observed_base_from_object)
            expected=rigid(self.base_from_grasp_tcp)@rigid(self.tcp_from_object)
            if not np.allclose(observed[:3,:3],expected[:3,:3],atol=1e-8) or np.linalg.norm(observed[:3,3]-expected[:3,3])>.015:
                raise ValueError('closing permits only a small planned object translation')
        elif self.fixed_side_clearance_m:
            raise ValueError('entry clearance requires the original observed object pose')
        size = np.asarray(self.object_size_m)
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError('finite positive payload dimensions required')
        if not np.isfinite([self.open_jaw_model_rad, self.held_jaw_model_rad]).all():
            raise ValueError('finite jaw model angles required')
        if self.object_constraint and not np.allclose(self.tcp_from_object, self.object_constraint.tcp_from_object):
            raise ValueError('payload and orientation constraint attachment differ')
        if self.payload_mesh is not None:
            vertices=np.asarray(self.payload_mesh['vertices']);faces=np.asarray(self.payload_mesh['faces'])
            if (vertices.ndim!=2 or vertices.shape[1]!=3 or not 4<=len(vertices)<=512 or not np.isfinite(vertices).all()
                    or faces.ndim!=2 or faces.shape[1]!=3 or not 4<=len(faces)<=1024
                    or faces.dtype.kind not in 'iu' or np.any(faces<0) or np.any(faces>=len(vertices))):
                raise ValueError('invalid bounded payload mesh')


@dataclass
class PickCarryPlan:
    status: str
    candidate_id: str | None = None
    stages: list = field(default_factory=list)
    carry_goal: object = None
    failures: list = field(default_factory=list)
    motion_authorized: bool = False
    physical_task_completed: bool = False
    resolved_grasp: object = None
    carry_quality: object = None
    selection: object = None


def at_actual_contact(candidate, base_from_tcp):
    """Attach without teleporting/tilting the observed object by the IK residual."""
    actual=rigid(base_from_tcp)
    world=rigid(candidate.base_from_grasp_tcp)@rigid(candidate.tcp_from_object)
    relative=np.linalg.inv(actual)@world
    constraint=replace(candidate.object_constraint,tcp_from_object=relative) if candidate.object_constraint else None
    return replace(candidate,base_from_grasp_tcp=actual,tcp_from_object=relative,object_constraint=constraint)


def fixed_contact_review(arm, candidate, *, tolerance_m=.002, support_normal=None):
    """Validate both sides at the final orientation, independently of target ACM."""
    import itertools
    if not np.isfinite(tolerance_m) or not 0 < tolerance_m <= .003:
        raise ValueError('bounded contact geometry tolerance required')
    tcp=rigid(candidate.base_from_grasp_tcp)
    obj=tcp@rigid(candidate.tcp_from_object)
    vertices=(np.asarray(candidate.payload_mesh['vertices']) if candidate.payload_mesh else
              np.asarray(list(itertools.product(*[(-s/2,s/2) for s in candidate.object_size_m]))))
    vertices=vertices@obj[:3,:3].T+obj[:3,3]
    kwargs = {} if support_normal is None else {'support_normal': support_normal}
    section=arm.jaw.contact_section(tcp@arm.tcp_from_stock,vertices,**kwargs)
    held=float(np.interp(candidate.held_jaw_model_rad,arm.jaw.angles,arm.jaw.gaps))
    opened=float(np.interp(candidate.open_jaw_model_rad,arm.jaw.angles,arm.jaw.gaps))
    moving_error=arm.jaw.fixed_face_m+held-section['high_m']
    opening_clearance=arm.jaw.fixed_face_m+opened-section['high_m']
    section.update(held_gap_m=held,open_gap_m=opened,moving_contact_error_m=moving_error,
                   open_moving_clearance_m=opening_clearance,tolerance_m=tolerance_m,
                   accepted=bool(abs(section['fixed_intrusion_m'])<=tolerance_m and
                                 abs(moving_error)<=tolerance_m and opening_clearance>=.002))
    if candidate.observed_base_from_object is not None:
        observed=rigid(candidate.observed_base_from_object)
        local=(vertices-obj[:3,3])@obj[:3,:3]
        entry_vertices=local@observed[:3,:3].T+observed[:3,3]
        entry=arm.jaw.contact_section(tcp@arm.tcp_from_stock,entry_vertices,**kwargs)
        if support_normal is not None:
            section['entry_moving_on_sidewall'] = entry['moving_on_sidewall']
        fixed_gap=-entry['fixed_intrusion_m']
        moving_gap=arm.jaw.fixed_face_m+opened-entry['high_m']
        section.update(entry_fixed_clearance_m=fixed_gap,open_moving_clearance_m=moving_gap,
            requested_fixed_clearance_m=candidate.fixed_side_clearance_m,
            planned_object_translation_base_m=(obj[:3,3]-observed[:3,3]).tolist(),
            object_translation_physically_verified=False,
            accepted=bool(section['accepted'] and fixed_gap>=0 and
                abs(fixed_gap-candidate.fixed_side_clearance_m)<=tolerance_m and moving_gap>=.002))
    if support_normal is not None:
        section['bilateral_side_contact'] = (section['fixed_on_sidewall'] and section['moving_on_sidewall']
                                            and section.get('entry_moving_on_sidewall', True))
        section['accepted'] = section['accepted'] and section['bilateral_side_contact']
    return section


def entry_pad_gaps(candidate, stock, fixed_vertices, opened_vertices):
    """Signed lateral gaps across each open pad near the CAD contact section."""
    if candidate.observed_base_from_object is None or not candidate.payload_mesh:
        return None
    from scipy.spatial import ConvexHull
    observed = rigid(candidate.observed_base_from_object)
    vertices = np.asarray(candidate.payload_mesh['vertices']) @ observed[:3, :3].T + observed[:3, 3]
    planes = ConvexHull((vertices-stock[:3, 3]) @ stock[:3, :3]).equations
    slope = -planes[:, 0]  # distance along fixed-to-moving (-stock X)
    entering, leaving = slope < -1e-9, slope > 1e-9
    parallel = ~(entering | leaving)
    result = {}
    for side, pad in (('fixed', fixed_vertices), ('moving', opened_vertices)):
        section = pad[np.abs(pad[:, 2]) <= .001]  # sample CAD vertices around TCP height
        offsets = section[:, 1:] @ planes[:, 1:3].T + planes[:, 3]
        low = np.max(-offsets[:, entering] / slope[entering], axis=1)
        high = np.min(-offsets[:, leaving] / slope[leaving], axis=1)
        overlaps = (high >= low) & np.all(offsets[:, parallel] <= 1e-8, axis=1)
        if not np.any(overlaps):
            return None
        gap = low + section[:, 0] if side == 'fixed' else -section[:, 0] - high
        result[side] = float(np.min(gap[overlaps]))
    return result


def tabletop_contact_margin(arm, candidate, support_plane, close_angle=None):
    """Estimate tolerance to height and wrist tilt at the checked direct-close endpoint.

    Perturb the hand relative to the stationary observed target. The resulting
    poses only order candidates; none is sent to IK or the controller.
    """
    from scipy.spatial.transform import Rotation
    plane = np.asarray(support_plane, dtype=float)
    up = plane[:3] / np.linalg.norm(plane[:3])
    plane_offset = plane[3] / np.linalg.norm(plane[:3])
    tcp = arm.fk(candidate.pregrasp_model_rad)[0]
    stock = tcp @ arm.tcp_from_stock
    closing = -stock[:3, 0]
    pivot = stock[:3, 3] + closing * arm.jaw.fixed_face_m
    tilt_axis = np.cross(closing, up)
    if np.linalg.norm(tilt_axis) < 1e-8:
        tilt_axis = stock[:3, 1]
    tilt_axis = tilt_axis / np.linalg.norm(tilt_axis)
    fixed = arm.jaw.fixed_vertices @ stock[:3, :3].T + stock[:3, 3]
    # Sample the moving finger through the commanded closing preload as well as
    # the held width. Full wire-sample collision checking still follows planning.
    jaw = arm.jaw
    end = candidate.held_jaw_model_rad if close_angle is None else close_angle
    angles = np.append(np.linspace(candidate.open_jaw_model_rad, end, 9), candidate.held_jaw_model_rad)
    moving_local = jaw.moving_vertices - jaw.hinge_point
    moving = np.concatenate([Rotation.from_rotvec(jaw.hinge_axis * a).apply(moving_local)
                             + jaw.hinge_point for a in angles]) @ stock[:3, :3].T + stock[:3, 3]
    worst_moving = float('inf')
    passed = 0
    worst_clearance = float('inf')
    for degrees in (-5., 0., 5.):
        rotation = Rotation.from_rotvec(tilt_axis * np.deg2rad(degrees)).as_matrix()
        tilted = tcp.copy()
        tilted[:3, :3] = rotation @ tcp[:3, :3]
        tilted[:3, 3] = pivot + rotation @ (tcp[:3, 3] - pivot)
        fixed_gap = float(np.min((fixed - pivot) @ (rotation.T @ up)) + pivot @ up + plane_offset)
        moving_gap = float(np.min((moving - pivot) @ (rotation.T @ up)) + pivot @ up + plane_offset)
        for height in (-.004, 0., .004):
            clearance = fixed_gap + height
            worst_clearance = min(worst_clearance, clearance)
            worst_moving = min(worst_moving, moving_gap + height)
            if min(clearance, moving_gap + height) <= 0:
                continue
            pose = tilted.copy()
            pose[:3, 3] += height * up
            try:
                contact = fixed_contact_review(arm, at_actual_contact(candidate, pose), support_normal=up)
            except ValueError:
                # A closing line outside the object in this trial affects only its rank.
                continue
            passed += int(contact['accepted'])
    opened = Rotation.from_rotvec(jaw.hinge_axis * candidate.open_jaw_model_rad).apply(moving_local) + jaw.hinge_point
    pad_gaps = entry_pad_gaps(candidate, stock, jaw.fixed_vertices, opened)
    return dict(entry_pad_gaps_m=pad_gaps, passed=passed, trials=9, minimum_fixed_clearance_m=worst_clearance,
                minimum_moving_clearance_m=worst_moving, height_offsets_mm=[-4, 0, 4], tilt_offsets_deg=[-5, 0, 5])


def tabletop_route_rank(margin, alignment):
    def clearance_bin(metres):
        return min(4, int(np.floor(metres*1000)))
    side = margin.get('entry_pad_gaps_m')
    return (margin['passed'], bool(alignment and alignment['preferred']),
            clearance_bin(min(side.values())) if side else -1,
            clearance_bin(min(margin['minimum_fixed_clearance_m'], margin['minimum_moving_clearance_m'])))


def compare_lateral_pinch(margin, alignment):
    if not margin or not alignment:
        return False
    pads = margin.get('entry_pad_gaps_m')
    # Confirmed eraser pick rgbd-670:18 had 19.1 degrees and ~2.3 mm
    # on its tighter pad. These bands trigger comparison only.
    return alignment['short_axis_angle_deg'] > 20. or bool(pads and min(pads.values()) < .002)


def better_lateral_pinch(before, after):
    # Move only for a straight-across route with better alignment or more pad
    # space, and no loss of perturbation tolerance or table clearance band.
    return (after is not None and after[1] and after[0] >= before[0]
            and after[2] >= before[2] and after[3] >= before[3]
            and (not before[1] or after[2] > before[2]))


def closing_sweep_candidate(candidate):
    """Enclose original-to-held object translation for closing collision checks."""
    if candidate.observed_base_from_object is None:return candidate
    from scipy.spatial import ConvexHull
    import itertools
    expected=rigid(candidate.base_from_grasp_tcp)@rigid(candidate.tcp_from_object)
    observed_in_expected=np.linalg.inv(expected)@rigid(candidate.observed_base_from_object)
    v=(np.asarray(candidate.payload_mesh['vertices']) if candidate.payload_mesh else
       np.asarray(list(itertools.product(*[(-s/2,s/2) for s in candidate.object_size_m]))))
    points=np.vstack([v,v@observed_in_expected[:3,:3].T+observed_in_expected[:3,3]])
    hull=ConvexHull(points)
    mesh=dict(vertices=points.tolist(),faces=hull.simplices.tolist(),kind='planned_closing_translation_sweep')
    return replace(candidate,payload_mesh=mesh)


@dataclass
class _CandidateEvaluation:
    """Geometry and preferences for one pose within one planning call.

    Keep the candidate itself: actual contact has the same candidate_id but a
    different attachment transform and must receive a fresh evaluation.
    """
    candidate: GraspRouteCandidate
    arm: object
    start: np.ndarray
    policy: object

    @cached_property
    def payload(self):
        c = self.candidate
        return CarriedGeometry(c.tcp_from_object, c.object_size_m, c.payload_mesh)

    @cached_property
    def balance(self):
        return grasp_balance(self.candidate.base_from_grasp_tcp, self.payload)

    @cached_property
    def pinch_alignment(self):
        if self.policy is None or self.candidate.observed_base_from_object is None:
            return None
        return tabletop_pinch_alignment(self.candidate, self.arm.tcp_from_stock)

    @cached_property
    def preference(self):
        return grasp_preferences(self.arm, self.start, self.candidate, self.policy)


def plan_pick_carry(arm, planner, start_model_rad, candidates, *, scene_id,
                    lift_distance_m=.02, reference_model_rad=None, policy=CarryPolicy(),
                    maximum_grasps=8, grasp_policy=None, stop_after_first=True, top_calibrated=False, exhaust_candidates=False, lift_directions_base=None, check_route_clearance=None, close_at_pregrasp=False, pregrasp_check=False, support_normal=None, support_plane=None, close_angles=None):
    """Planner protocol: open, connect, cartesian, close; each returns a StagePath.

    No failed approach or partial Cartesian result may reach the carry stage.
    Closing checks geometry only: contact/holding requires the runtime gripper
    feedback controller before a physical lifting command can be authorized.
    Default: return the first complete route. Failed candidates/goals fall
    through to the next alternative; do not preplan successful backup routes.
    stop_after_first=False is an explicit offline quality-comparison mode.
    """
    start = checked_joints(arm, start_model_rad)
    if not 0 < lift_distance_m <= .1 or not 1 <= maximum_grasps <= 32:
        raise ValueError('bounded lift and grasp budget required')
    directions=np.asarray([(0.,0.,1.)] if lift_directions_base is None else lift_directions_base,dtype=float)
    if (directions.ndim!=2 or directions.shape[1:]!=(3,) or not len(directions)
            or not np.isfinite(directions).all() or np.any(directions[:,2]<=0)):
        raise ValueError('finite upward lift directions required')
    directions=directions/np.linalg.norm(directions,axis=1)[:,None]
    answers, failures = [], []
    # Rank only capture-bound candidates; stale metadata cannot enter geometry.
    current=[]
    for candidate in candidates:
        if candidate.scene_id != scene_id or candidate.calibration_id != arm.profile['calibration_id']:
            failures.append({'candidate': candidate.candidate_id, 'stage': 'binding', 'reason': 'STALE_SCENE_OR_CALIBRATION'})
            continue
        posture = contact_posture(candidate.base_from_grasp_tcp, grasp_policy) if grasp_policy is not None else None
        if posture is not None and not posture['accepted']:
            failures.append({'candidate':candidate.candidate_id,'stage':'grasp_posture',
                'reason':'EXCESSIVE_TABLETOP_CONTACT_TILT','diagnostics':posture})
            continue
        current.append(candidate)
    evaluations = {}

    def evaluate(candidate):
        key = id(candidate)
        if key not in evaluations:
            evaluations[key] = _CandidateEvaluation(candidate, arm, start, grasp_policy)
        return evaluations[key]

    def balance(candidate):
        return evaluate(candidate).balance
    if grasp_policy is not None and not pregrasp_check:
        retained=[]
        for candidate in current:
            evidence=balance(candidate)
            if evidence['normalized_offset']>grasp_policy.maximum_balance_offset:
                failures.append({'candidate':candidate.candidate_id,'stage':'grasp_balance',
                    'reason':'CONTACT_TOO_FAR_FROM_PAYLOAD_CENTER','diagnostics':evidence})
            else:retained.append(candidate)
        current=retained
    def preference(c):
        value=evaluate(c).preference
        if not top_calibrated:return value
        from .pick_calibration import top_posture_preference
        posture=top_posture_preference(c.pregrasp_model_rad)
        return dict(value,cost=value['cost']+posture['cost'],calibration_posture=posture)
    def balance_cost(c):
        offset=balance(c)['normalized_offset']
        return (offset/grasp_policy.preferred_balance_offset)**2 if grasp_policy else 2*offset**2
    # Spend the bounded route budget on a mixture of balanced and easy-to-grip
    # alternatives. A 180-degree finger swap is not equivalent on a SO101.
    current.sort(key=lambda c:balance_cost(c)+preference(c)['cost'])
    preferred_count=maximum_grasps-(max(1,maximum_grasps//4) if maximum_grasps>1 else 0)
    scheduled=current[:preferred_count]
    scheduled_ids={c.candidate_id for c in scheduled}
    # Reserve alternatives without the wrist/angle preference. A crowded pool
    # of similar preferred poses must not erase a feasible opposite-side grasp.
    while len(scheduled)<min(maximum_grasps,len(current)):
        def exploration(c):
            wrist_novelty=min(abs(c.pregrasp_model_rad[4]-s.pregrasp_model_rad[4]) for s in scheduled)/np.pi
            return balance(c)['normalized_offset']-.5*min(wrist_novelty,1.)
        remaining=[c for c in current if c.candidate_id not in scheduled_ids]
        if not remaining:break
        c=min(remaining,key=exploration)
        scheduled.append(c);scheduled_ids.add(c.candidate_id)
    if exhaust_candidates:
        scheduled.extend(c for c in current if c.candidate_id not in {s.candidate_id for s in scheduled})
    # Search the entire aligned pool before spending the remaining budget on
    # the usual alternatives. Stable sorting preserves the existing tie-breaks.
    aligned = [c for c in current if (alignment := evaluate(c).pinch_alignment)
               and alignment['preferred']]
    if aligned:
        aligned.sort(key=lambda c: evaluate(c).pinch_alignment['longitudinal_center_offset_m'])
        aligned_ids = {c.candidate_id for c in aligned}
        scheduled = (aligned + [c for c in scheduled if c.candidate_id not in aligned_ids])[:len(scheduled)]
    if grasp_policy and not pregrasp_check:
        scheduled.sort(key=lambda c: balance(c)['normalized_offset'] > grasp_policy.primary_balance_offset)
    margins = {}
    if close_at_pregrasp and not pregrasp_check and support_plane is not None:
        margins = {c.candidate_id: tabletop_contact_margin(arm, c, support_plane,
                   close_angles[c.candidate_id] if close_angles is not None else None) for c in scheduled}
        scheduled.sort(key=lambda c: tabletop_route_rank(margins[c.candidate_id], evaluate(c).pinch_alignment), reverse=True)
    for candidate in scheduled:
        stage = 'open_geometry'
        try:
            pre = checked_joints(arm, candidate.pregrasp_model_rad)
            if candidate.object_constraint and not candidate.object_constraint.satisfied(candidate.base_from_grasp_tcp):
                raise PlanRejected('grasp already violates object orientation')
            paths = [planner.open(start, candidate)]
            stage = 'pregrasp'
            paths.append(planner.connect(start, pre, candidate, attached=False, label='pregrasp'))
            if pregrasp_check:
                stage='table_clearance'
                if check_route_clearance is None:
                    raise ValueError('inspection requires checked open-finger clearance')
                check_route_clearance(candidate,paths)
                return PickCarryPlan('OFFLINE_ROUTE_FOUND',candidate.candidate_id,paths,
                    failures=failures,resolved_grasp=candidate,
                    carry_quality={'tabletop_pinch_alignment':evaluate(candidate).pinch_alignment},
                    selection={'policy':'open_pregrasp_inspection'})
            stage = 'approach'
            if close_at_pregrasp:
                # The checked connection endpoint is the closing pose. Retain
                # the stage boundary for the controller without issuing travel.
                endpoint=paths[-1].end
                paths.append(replace(paths[-1], label='approach', joints=np.array([endpoint]),
                                     planner='hold checked pregrasp endpoint', duration_s=0.))
            else:
                paths.append(planner.cartesian(pre, candidate.base_from_grasp_tcp, candidate,
                                               attached=False, label='approach'))
            candidate=at_actual_contact(candidate,arm.fk(paths[-1].end)[0])
            if support_normal is not None:
                stage = 'close_geometry'
                sides = fixed_contact_review(arm, candidate, support_normal=support_normal)
                if not sides['bilateral_side_contact']:
                    raise PlanRejected('opposing jaw sidewall contacts unavailable')
                stage = 'approach'
            if grasp_policy is not None and not contact_posture(candidate.base_from_grasp_tcp,grasp_policy)['accepted']:
                raise PlanRejected('actual contact exceeds tabletop posture envelope')
            if grasp_policy is not None and balance(candidate)['normalized_offset']>grasp_policy.maximum_balance_offset:
                raise PlanRejected('actual contact too far from payload center')
            stage = 'close_geometry'
            paths.append(planner.close(paths[-1].end, candidate))
            stage = 'lift'
            lift_attempts=[]
            for direction in directions:
                lift_pose = rigid(candidate.base_from_grasp_tcp).copy()
                lift_pose[:3, 3] += direction*lift_distance_m
                try:
                    lift_path=planner.cartesian(paths[-1].end,lift_pose,candidate,attached=True,label='lift')
                except PlanRejected as exc:
                    lift_attempts.append(dict(direction_base=direction.tolist(),reason=str(exc),
                                              diagnostics=getattr(exc,'diagnostics',None)))
                else:
                    paths.append(lift_path)
                    break
            else:
                if len(lift_attempts)==1:
                    failure=PlanRejected(lift_attempts[0]['reason'])
                    failure.diagnostics=lift_attempts[0]['diagnostics']
                else:
                    failure=PlanRejected('all checked lift directions blocked')
                    failure.diagnostics=dict(lift_attempts=lift_attempts)
                raise failure
            calibration_review=None
            if top_calibrated:
                from .pick_calibration import review_top_path
                calibration_review=review_top_path([(p.label,p.joints) for p in paths[2:]])
            stage = 'carry_goals'
            goals = carry_goals(arm, paths[-1].end, candidate.base_from_grasp_tcp,
                                reference_model_rad=reference_model_rad,
                                object_constraint=candidate.object_constraint, policy=policy,
                                payload=evaluate(candidate).payload)
            if not goals:
                raise PlanRejected('no carry goal satisfies the requested orientation')
            for goal in goals:
                try:
                    carry = planner.connect(paths[-1].end, goal.model_rad, candidate, attached=True, label='carry')
                    # No detector score can override a failed whole-route check.
                    if check_route_clearance is not None:
                        try:
                            check_route_clearance(candidate, paths+[carry])
                        except TableClearanceRejected as exc:
                            failures.append(dict(candidate=candidate.candidate_id, stage='table_clearance',
                                                 reason=str(exc), diagnostics=exc.diagnostics))
                            # All carry goals share the same open-through-lift path.
                            break
                    travel = sum(float(np.sum(np.linalg.norm(np.diff(p.joints, axis=0), axis=1))) for p in paths+[carry])
                    quality=payload_load(arm,goal.model_rad,evaluate(candidate).payload)
                    clearance=planner.payload_clearance(goal.model_rad,candidate) if hasattr(planner,'payload_clearance') else None
                    quality['payload_clearance_probe']=clearance
                    if margins:
                        quality['tabletop_contact_margin'] = margins[candidate.candidate_id]
                    quality['grasp_balance']=balance(candidate)
                    quality['tabletop_pinch_alignment']=evaluate(candidate).pinch_alignment
                    quality['grasp_preference']=preference(candidate)
                    if calibration_review is not None:quality['top_calibration']=calibration_review
                    quality['carry_grasp_balance']=grasp_balance(arm.fk(goal.model_rad)[0],
                        evaluate(candidate).payload)
                    # Clearance is a soft preference after the original full-path
                    # collision test, not a larger hard obstacle/payload shell.
                    penalty=.4*(1-clearance['passed_margin_m']/.02) if clearance else 0.
                    answers.append((goal.cost + .05*travel + penalty + quality['grasp_preference']['cost']
                                    + (balance_cost(candidate) if grasp_policy else 0.),
                                    candidate, paths+[carry], goal,quality))
                    if stop_after_first:
                        return PickCarryPlan('OFFLINE_ROUTE_FOUND', candidate.candidate_id, paths+[carry], goal, failures,
                            resolved_grasp=candidate, carry_quality=quality,
                            selection=dict(policy='first_complete_route', physical_balance_validated=False))
                except PlanRejected as exc:
                    failures.append({'candidate': candidate.candidate_id, 'stage': 'carry', 'reason': str(exc),
                                     'diagnostics':getattr(exc,'diagnostics',None)})
        except (PlanRejected, ValueError) as exc:
            failures.append({'candidate': candidate.candidate_id, 'stage': stage, 'reason': str(exc),
                             'diagnostics':getattr(exc,'diagnostics',None)})
    if not answers:
        return PickCarryPlan('NO_COMPLETE_ROUTE', failures=failures)
    # The offline comparison mode uses the same priorities as first-route search.
    eligible = answers
    if margins:
        best = max(tabletop_route_rank(a[4]['tabletop_contact_margin'], a[4]['tabletop_pinch_alignment']) for a in answers)
        eligible = [a for a in answers if tabletop_route_rank(a[4]['tabletop_contact_margin'], a[4]['tabletop_pinch_alignment']) == best]
    ranked = [a for a in eligible if (alignment := a[4]['tabletop_pinch_alignment'])
              and alignment['preferred']]
    if ranked:
        closest = min(a[4]['tabletop_pinch_alignment']['longitudinal_center_offset_m'] for a in ranked)
        ranked = [a for a in ranked if a[4]['tabletop_pinch_alignment']['longitudinal_center_offset_m'] <= closest+1e-9]
    else:
        ranked = eligible
    best_balance=min(a[4]['grasp_balance']['normalized_offset'] for a in ranked)
    shortlist=[a for a in ranked if a[4]['grasp_balance']['normalized_offset']<=best_balance+policy.grasp_balance_band]
    _, candidate, paths, goal,quality = min(shortlist, key=lambda answer: answer[0])
    selection=dict(policy='near_best_feasible_balance_then_grasp_and_carry_cost',
        best_normalized_offset=best_balance,normalized_preference_band=policy.grasp_balance_band,
        observed_geometry_proxy=True,physical_balance_validated=False,
        alternatives=[dict(candidate_id=a[1].candidate_id,route_cost=a[0],
            center_to_closing_line_m=a[4]['grasp_balance']['center_to_closing_line_m'],
            normalized_offset=a[4]['grasp_balance']['normalized_offset'],
            grasp_preference=a[4]['grasp_preference'],
            within_balance_band=a[4]['grasp_balance']['normalized_offset']<=best_balance+policy.grasp_balance_band)
            for a in answers])
    if grasp_policy is not None:
        selection['policy']='near_best_feasible_balance_then_tabletop_ease_and_carry_cost'
    return PickCarryPlan('OFFLINE_ROUTE_FOUND', candidate.candidate_id, paths, goal, failures,
                        resolved_grasp=candidate,carry_quality=quality,selection=selection)


@dataclass
class PlaceCarryPlan:
    status: str
    stages: list = field(default_factory=list)
    released_grasp: object = None
    failures: list = field(default_factory=list)
    motion_authorized: bool = False
    physical_task_completed: bool = False
    support_observation_id: str = ''
    runtime_release_verification_required: bool = True


def plan_place_carry(arm, planner, start_model_rad, held, *, scene_id,
                     base_from_object, support_bounds_m, support_observation_id,
                     support_observed_s, now_s, empty_transport_model_rad,
                     clearance_m=.03, maximum_age_s=2.):
    """Carry -> above support -> vertical lower -> release -> rise -> transport.

    Reuses the pick planner's full STL/payload collision checks. The rectangular
    horizontal support is a fresh observation, not a configured furniture name.
    A 0..2 mm bottom gap is a geometric proposal; only the runtime's independent
    release/support observation may claim that the object was actually placed.
    """
    import itertools
    from scipy.spatial.transform import Rotation
    from .carry_cartesian import validate_vertical_descent
    from .navigation import identifier, number
    # Entry observation belongs to the completed pick, not the new placement.
    held=replace(held,observed_base_from_object=None,fixed_side_clearance_m=0.)
    result = PlaceCarryPlan('NO_COMPLETE_ROUTE')
    stage = 'binding'
    try:
        identifier(support_observation_id, 'support observation')
        now, observed = number(now_s, 'clock'), number(support_observed_s, 'support time')
        if (not 0 < number(maximum_age_s, 'support age limit') <= 5
                or not 0 <= now-observed <= maximum_age_s
                or held.scene_id != scene_id or held.calibration_id != arm.profile['calibration_id']):
            raise PlanRejected('stale support, scene or calibration')
        if not .01 <= number(clearance_m, 'place clearance') <= .05:
            raise PlanRejected('bounded vertical placement clearance required')
        support = np.asarray(support_bounds_m, dtype=float)
        if (support.shape != (2,3) or not np.isfinite(support).all()
                or np.any(support[1,:2] <= support[0,:2])
                or abs(support[1,2]-support[0,2]) > 1e-8):
            raise PlanRejected('horizontal observed support bounds required')
        desired = rigid(base_from_object)
        vertices = (np.asarray(held.payload_mesh['vertices']) if held.payload_mesh is not None else
                    np.asarray(list(itertools.product(*[(-v/2,v/2) for v in held.object_size_m]))))
        world = vertices@desired[:3,:3].T+desired[:3,3]
        if (np.any(world[:,:2].min(axis=0) < support[0,:2]+.003)
                or np.any(world[:,:2].max(axis=0) > support[1,:2]-.003)
                or not 0 <= float(world[:,2].min()-support[0,2]) <= .002):
            raise PlanRejected('payload outside support or bottom gap exceeds 2 mm')
        start = checked_joints(arm,start_model_rad)
        empty_transport = checked_joints(arm,empty_transport_model_rad)
        contact = desired@np.linalg.inv(rigid(held.tcp_from_object))
        preplace = contact.copy(); preplace[2,3] += clearance_m
        stage = 'preplace_ik'
        q, _, _ = arm.solve(preplace,start)
        actual = arm.fk(q)[0]
        if (np.linalg.norm(actual[:3,3]-preplace[:3,3]) > .001
                or Rotation.from_matrix(actual[:3,:3].T@preplace[:3,:3]).magnitude() > np.deg2rad(2)):
            raise PlanRejected('preplace IK exceeds Cartesian handoff tolerance')
        stage = 'preplace'
        paths = [planner.connect(start,q,held,attached=True,label='place_preplace')]
        stage = 'lower'
        lower = planner.cartesian(paths[-1].end,contact,held,attached=True,label='place_lower')
        validate_vertical_descent([arm.fk(q)[0] for q in lower.joints])
        paths.append(lower)
        # Keep the object at the actual release pose; never teleport to the
        # ideal target while checking the open jaw and retreat collision sweep.
        released = replace(held,base_from_grasp_tcp=arm.fk(lower.end)[0])
        actual_object = released.base_from_grasp_tcp@released.tcp_from_object
        actual_vertices=vertices@actual_object[:3,:3].T+actual_object[:3,3]
        if (np.any(actual_vertices[:,:2].min(axis=0) < support[0,:2])
                or np.any(actual_vertices[:,:2].max(axis=0) > support[1,:2])
                or not -.0001 <= float(actual_vertices[:,2].min()-support[0,2]) <= .003):
            raise PlanRejected('actual release misses observed support')
        stage = 'release_geometry'
        paths.append(planner.release(paths[-1].end,released))
        stage = 'retreat'
        retreat = released.base_from_grasp_tcp.copy();retreat[2,3] += clearance_m
        paths.append(planner.cartesian(paths[-1].end,retreat,released,attached=False,label='place_retreat'))
        stage = 'empty_transport'
        paths.append(planner.connect(paths[-1].end,empty_transport,released,attached=False,label='empty_transport'))
        return PlaceCarryPlan('OFFLINE_ROUTE_FOUND',paths,released,
                              support_observation_id=support_observation_id)
    except (ValueError, PlanRejected) as error:
        result.failures.append(dict(stage=stage,reason=str(error),diagnostics=getattr(error,'diagnostics',None)))
        return result
