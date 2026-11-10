"""L3D stage coordinator: jointly select a grasp and a reachable carry goal.

The injected planner owns collision checking and trajectories. This module has
no transport or execution interface. One fixed lift and current scene per call.
"""
from dataclasses import dataclass, field, replace

import numpy as np

from .carry_posture import carry_goals, checked_joints, CarryPolicy
from .s2_geometry import rigid
from .carry_load import CarriedGeometry, payload_load, grasp_balance
from .grasp_preferences import grasp_preferences, contact_posture


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

    def __post_init__(self):
        if not all((self.candidate_id, self.calibration_id, self.scene_id)):
            raise ValueError('candidate identity and scene binding required')
        rigid(self.base_from_grasp_tcp); rigid(self.tcp_from_object)
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


def plan_pick_carry(arm, planner, start_model_rad, candidates, *, scene_id,
                    lift_distance_m=.02, reference_model_rad=None, policy=CarryPolicy(),
                    maximum_grasps=8, grasp_policy=None):
    """Planner protocol: open, connect, cartesian, close; each returns a StagePath.

    No failed approach or partial Cartesian result may reach the carry stage.
    Closing checks geometry only: contact/holding requires the runtime gripper
    feedback controller before a physical lifting command can be authorized.
    """
    start = checked_joints(arm, start_model_rad)
    if not 0 < lift_distance_m <= .1 or not 1 <= maximum_grasps <= 32:
        raise ValueError('bounded lift and grasp budget required')
    answers, failures = [], []
    # Rank only capture-bound candidates; stale metadata cannot enter geometry.
    current=[]
    for candidate in candidates:
        if candidate.scene_id != scene_id or candidate.calibration_id != arm.profile['calibration_id']:
            failures.append({'candidate': candidate.candidate_id, 'stage': 'binding', 'reason': 'STALE_SCENE_OR_CALIBRATION'})
            continue
        if grasp_policy is not None and not contact_posture(candidate.base_from_grasp_tcp,grasp_policy)['accepted']:
            failures.append({'candidate':candidate.candidate_id,'stage':'grasp_posture',
                'reason':'EXCESSIVE_TABLETOP_CONTACT_TILT','diagnostics':contact_posture(candidate.base_from_grasp_tcp,grasp_policy)})
            continue
        current.append(candidate)
    def balance(candidate):
        return grasp_balance(candidate.base_from_grasp_tcp,CarriedGeometry(candidate.tcp_from_object,
            candidate.object_size_m,candidate.payload_mesh))
    if grasp_policy is not None:
        retained=[]
        for candidate in current:
            evidence=balance(candidate)
            if evidence['normalized_offset']>grasp_policy.maximum_balance_offset:
                failures.append({'candidate':candidate.candidate_id,'stage':'grasp_balance',
                    'reason':'CONTACT_TOO_FAR_FROM_PAYLOAD_CENTER','diagnostics':evidence})
            else:retained.append(candidate)
        current=retained
    def preference(c):return grasp_preferences(arm,start,c,grasp_policy)
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
    for candidate in scheduled:
        stage = 'approach'
        try:
            pre = checked_joints(arm, candidate.pregrasp_model_rad)
            if candidate.object_constraint and not candidate.object_constraint.satisfied(candidate.base_from_grasp_tcp):
                raise PlanRejected('grasp already violates object orientation')
            paths = [planner.open(start, candidate)]
            paths.append(planner.connect(start, pre, candidate, attached=False, label='pregrasp'))
            paths.append(planner.cartesian(pre, candidate.base_from_grasp_tcp, candidate,
                                           attached=False, label='approach'))
            candidate=at_actual_contact(candidate,arm.fk(paths[-1].end)[0])
            if grasp_policy is not None and not contact_posture(candidate.base_from_grasp_tcp,grasp_policy)['accepted']:
                raise PlanRejected('actual contact exceeds tabletop posture envelope')
            if grasp_policy is not None and balance(candidate)['normalized_offset']>grasp_policy.maximum_balance_offset:
                raise PlanRejected('actual contact too far from payload center')
            stage = 'close_geometry'
            paths.append(planner.close(paths[-1].end, candidate))
            stage = 'lift'
            lift_pose = rigid(candidate.base_from_grasp_tcp).copy()
            lift_pose[2, 3] += lift_distance_m
            paths.append(planner.cartesian(paths[-1].end, lift_pose, candidate, attached=True, label='lift'))
            stage = 'carry_goals'
            goals = carry_goals(arm, paths[-1].end, candidate.base_from_grasp_tcp,
                                reference_model_rad=reference_model_rad,
                                object_constraint=candidate.object_constraint, policy=policy,
                                payload=CarriedGeometry(candidate.tcp_from_object,candidate.object_size_m,candidate.payload_mesh))
            if not goals:
                raise PlanRejected('no carry goal satisfies the requested orientation')
            for goal in goals:
                try:
                    carry = planner.connect(paths[-1].end, goal.model_rad, candidate, attached=True, label='carry')
                    # No detector score can override a failed whole-route check.
                    travel = sum(float(np.sum(np.linalg.norm(np.diff(p.joints, axis=0), axis=1))) for p in paths+[carry])
                    quality=payload_load(arm,goal.model_rad,CarriedGeometry(candidate.tcp_from_object,
                        candidate.object_size_m,candidate.payload_mesh))
                    clearance=planner.payload_clearance(goal.model_rad,candidate) if hasattr(planner,'payload_clearance') else None
                    quality['payload_clearance_probe']=clearance
                    quality['grasp_balance']=balance(candidate)
                    quality['grasp_preference']=preference(candidate)
                    quality['carry_grasp_balance']=grasp_balance(arm.fk(goal.model_rad)[0],
                        CarriedGeometry(candidate.tcp_from_object,candidate.object_size_m,candidate.payload_mesh))
                    # Clearance is a soft preference after the original full-path
                    # collision test, not a larger hard obstacle/payload shell.
                    penalty=.4*(1-clearance['passed_margin_m']/.02) if clearance else 0.
                    answers.append((goal.cost + .05*travel + penalty + quality['grasp_preference']['cost']
                                    + (balance_cost(candidate) if grasp_policy else 0.),
                                    candidate, paths+[carry], goal,quality))
                except PlanRejected as exc:
                    failures.append({'candidate': candidate.candidate_id, 'stage': 'carry', 'reason': str(exc),
                                     'diagnostics':getattr(exc,'diagnostics',None)})
        except (PlanRejected, ValueError) as exc:
            failures.append({'candidate': candidate.candidate_id, 'stage': stage, 'reason': str(exc),
                             'diagnostics':getattr(exc,'diagnostics',None)})
    if not answers:
        return PickCarryPlan('NO_COMPLETE_ROUTE', failures=failures)
    best_balance=min(a[4]['grasp_balance']['normalized_offset'] for a in answers)
    shortlist=[a for a in answers if a[4]['grasp_balance']['normalized_offset']<=best_balance+policy.grasp_balance_band]
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
