"""L3C joint base/lift/grasp candidate selection. This module cannot execute.

Hard feasibility comes from explicit navigation, view and whole-motion backends.
A reachable endpoint alone is insufficient. Synthetic checks stay synthetic.
"""
from collections import Counter
from dataclasses import asdict, dataclass
import math
from typing import Protocol

from ..fetch import InvalidTask
from ..navigation import identifier
from .contracts import Context, Stamp, finite, vector

REQUIRED_PHASES = {'approach', 'contact', 'lift', 'retreat', 'transport'}
PICK_LIFT_PHASES = {'approach', 'contact', 'lift', 'hold'}
PLACE_PHASES = {'approach', 'support', 'release', 'retreat', 'transport'}


@dataclass(frozen=True)
class Candidate:
    candidate_id: str
    base_xy_yaw: tuple
    lift_height_m: float
    arm: str
    grasp_id: str
    operation: str


@dataclass(frozen=True)
class Check:
    valid: bool
    reason: str
    cost: float = 0
    margin: float = 0
    phases: tuple = ()
    mode: str = 'unverified'

    def validate(self):
        if type(self.valid) is not bool or not isinstance(self.reason, str):
            raise InvalidTask('typed feasibility result required')
        finite(self.cost, 'check cost', 0)
        finite(self.margin, 'check margin', 0, 1)
        if self.mode not in {'simulation', 'measured', 'unverified'}:
            raise InvalidTask('invalid feasibility provenance')
        return self


class Feasibility(Protocol):
    def navigation(self, candidate: Candidate) -> Check: ...
    def view(self, candidate: Candidate) -> Check: ...
    def whole_motion(self, candidate: Candidate) -> Check: ...


@dataclass(frozen=True)
class PlacementProfile:
    lift_min_m: float
    lift_max_m: float
    heights_m: tuple
    distances_m: tuple
    azimuths_rad: tuple
    yaw_offsets_rad: tuple = (0.0,)
    arms: tuple = ('left',)
    maximum_candidates: int = 512
    distance_scale_m: float = 5.0
    weights: tuple = (1.0, 0.5, 0.5, 1.0, 1.0)
    source: str = 'simulation'

    def __post_init__(self):
        lo = finite(self.lift_min_m, 'lift minimum', 0, 3)
        hi = finite(self.lift_max_m, 'lift maximum', 0, 3)
        if lo >= hi or self.source not in {'simulation', 'measured'}:
            raise InvalidTask('invalid placement profile')
        for name, values, low, high in (
                ('height', self.heights_m, lo, hi), ('distance', self.distances_m, .01, 10),
                ('azimuth', self.azimuths_rad, -math.pi, math.pi), ('yaw', self.yaw_offsets_rad, -math.pi, math.pi)):
            if not isinstance(values, (list, tuple)) or not 1 <= len(values) <= 64:
                raise InvalidTask('bounded nonempty candidate dimensions required')
            for v in values: finite(v, name, low, high)
        if not self.arms or set(self.arms) - {'left', 'right'} or len(set(self.arms)) != len(self.arms):
            raise InvalidTask('explicit installed arms required')
        if type(self.maximum_candidates) is not int or not 1 <= self.maximum_candidates <= 4096:
            raise InvalidTask('bounded candidate budget required')
        finite(self.distance_scale_m, 'distance normalization', .01, 100)
        if len(self.weights) != 5 or any(finite(w, 'weight', 0, 100) < 0 for w in self.weights) or not any(self.weights):
            raise InvalidTask('five nonnegative nonzero weights required')


class PlacementPlanner:
    def __init__(self, profile, feasibility):
        if not isinstance(profile, PlacementProfile):
            raise InvalidTask('placement profile required')
        self.profile, self.feasibility = profile, feasibility

    def plan(self, *, target_m, stamp, context, now_s, scene_revision, grasp_ids,
             current_xy_yaw, current_height_m, operation='pick', maximum_age_s=2):
        if not isinstance(stamp, Stamp) or not isinstance(context, Context):
            raise InvalidTask('stamped target required')
        stamp.check(context, now_s, maximum_age_s)
        identifier(scene_revision, 'scene_revision')
        if operation not in {'pick', 'pick_lift', 'place'}:
            raise InvalidTask('unsupported manipulation operation')
        target = vector(target_m, 3, 'target')
        current = vector(current_xy_yaw, 3, 'current base pose')
        profile = self.profile
        if operation == 'pick_lift' and len(profile.heights_m) != 1:
            raise InvalidTask('single_pick_lift_setting_required')
        height = finite(current_height_m, 'current height', profile.lift_min_m, profile.lift_max_m)
        if not isinstance(grasp_ids, (list, tuple)) or not 1 <= len(grasp_ids) <= 16 or len(set(grasp_ids)) != len(grasp_ids):
            raise InvalidTask('bounded unique grasp hypotheses required')
        for grasp in grasp_ids: identifier(grasp, 'grasp_id')
        total = math.prod(map(len, (profile.distances_m, profile.azimuths_rad,
                                   profile.heights_m, profile.yaw_offsets_rad, profile.arms, grasp_ids)))
        if total > profile.maximum_candidates:
            raise InvalidTask('candidate_budget_exceeded')
        accepted, rejected, evaluations = [], Counter(), 0
        phases = {'pick': REQUIRED_PHASES, 'pick_lift': PICK_LIFT_PHASES, 'place': PLACE_PHASES}[operation]
        for distance in profile.distances_m:
            for azimuth in profile.azimuths_rad:
                x, y = target[0]+distance*math.cos(azimuth), target[1]+distance*math.sin(azimuth)
                facing = math.atan2(target[1]-y, target[0]-x)
                for offset in profile.yaw_offsets_rad:
                    yaw = math.atan2(math.sin(facing+offset), math.cos(facing+offset))
                    for h in profile.heights_m:
                        for arm in profile.arms:
                            for grasp in grasp_ids:
                                evaluations += 1
                                candidate = Candidate(f'candidate-{evaluations}', (x, y, yaw), h, arm, grasp, operation)
                                checks = []
                                try:
                                    for name in ('navigation', 'view', 'whole_motion'):
                                        check = getattr(self.feasibility, name)(candidate)
                                        if not isinstance(check, Check):
                                            raise InvalidTask('feasibility result required')
                                        check.validate()
                                        if not check.valid or check.mode == 'unverified':
                                            rejected[f'{name}:{check.reason or "unverified"}'] += 1
                                            break
                                        checks.append(check)
                                    if len(checks) != 3:
                                        continue
                                    nav, view, motion = checks
                                    if not phases.issubset(set(motion.phases)):
                                        rejected['whole_motion:incomplete_phase_check'] += 1
                                        continue
                                    if profile.source == 'measured' and any(c.mode != 'measured' for c in checks):
                                        rejected['provenance:synthetic_check_for_measured_profile'] += 1
                                        continue
                                    costs = (nav.cost/profile.distance_scale_m,
                                             abs(h-height)/(profile.lift_max_m-profile.lift_min_m),
                                             abs(math.atan2(math.sin(yaw-current[2]), math.cos(yaw-current[2])))/math.pi,
                                             1-view.margin, 1-motion.margin)
                                    accepted.append({**asdict(candidate), 'cost': sum(w*c for w, c in zip(profile.weights, costs)),
                                        'cost_terms': dict(zip(('travel', 'lift', 'yaw', 'view', 'motion'), costs)),
                                        'checks': [asdict(c) for c in checks],
                                        'mode': 'measured' if profile.source == 'measured' else 'simulation'})
                                except InvalidTask as error:
                                    rejected['check_invalid:'+str(error)] += 1
        accepted.sort(key=lambda c: (c['cost'], c['candidate_id']))
        return dict(status='CANDIDATES_READY' if accepted else 'UNREACHABLE',
                    context=context.to_dict(), capture_id=stamp.capture_id,
                    observed_s=stamp.observed_s, scene_revision=scene_revision,
                    target_m=list(target), candidates=accepted, rejected=dict(rejected), evaluated=evaluations,
                    executable=False, requires_reobservation=True, motion_authorized=False,
                    policy='best_of_enumerated_feasible_candidates')

    @staticmethod
    def validate_handoff(plan, *, context, scene_revision, now_s, maximum_age_s=2):
        Stamp(plan['observed_s'], Context.parse(plan['context']), plan['capture_id']).check(context, now_s, maximum_age_s)
        if scene_revision != plan['scene_revision'] or plan['status'] != 'CANDIDATES_READY':
            raise InvalidTask('placement_plan_invalidated')
        # Coarse candidates must still pass the existing ManipulationPort after
        # arrival/reobservation. This function does not authorize trajectories.
        return True
