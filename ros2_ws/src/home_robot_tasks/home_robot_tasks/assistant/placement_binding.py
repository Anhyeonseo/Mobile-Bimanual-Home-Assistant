"""L3C -> existing MissionCatalog binding, without a second task executor.

The target-to-map callback must use the capture-time calibrated transform. A
coarse candidate is used only for alignment/lift; the existing fresh capture,
ManipulationPort and independent outcome monitor still gate the arm.
"""
from copy import deepcopy
from dataclasses import replace
import math

from ..fetch import InvalidTask
from ..navigation import NavigateRequest
from .contracts import Stamp, finite, vector
from .placement import PlacementPlanner


class TaskPlacementPolicy:
    def __init__(self, *, profile, feasibility_factory, target_to_map, context,
                 current_configuration, scene_revision, clock, grasp_ids,
                 fresh_grasp_task, maximum_target_age_s=2, maximum_alignment_s=30,
                 arrival_tolerance_m=.03, arrival_tolerance_rad=.1, height_tolerance_m=.003,
                 pick_height_for=None):
        self.profile, self.feasibility_factory = profile, feasibility_factory
        self.pick_height_for = pick_height_for
        self.target_to_map, self.context = target_to_map, context
        self.current, self.scene, self.clock, self.grasps = current_configuration, scene_revision, clock, grasp_ids
        self.target_age = finite(maximum_target_age_s, 'target age', .01, 5)
        self.alignment_budget = finite(maximum_alignment_s, 'alignment budget', .1, 180)
        if not callable(fresh_grasp_task):
            raise InvalidTask('fresh grasp geometry adapter required')
        self.fresh_grasp_task = fresh_grasp_task
        self.xy_tolerance = finite(arrival_tolerance_m, 'arrival tolerance', .001, .2)
        self.yaw_tolerance = finite(arrival_tolerance_rad, 'yaw tolerance', .001, .3)
        self.height_tolerance = finite(height_tolerance_m, 'height tolerance', .001, .05)
        self.selected = {}

    def reset(self):
        self.selected.clear()

    def resolve(self, skill, parameters, catalog, now_s):
        operation = 'place' if skill == 'place' or skill.endswith('_place') else 'pick'
        key = (operation, parameters['object_id'])
        if skill.startswith('align_'):
            self.selected.pop(key, None)
            target = catalog.checked_target(operation == 'place')
            xyz, stamp = self.target_to_map(target)
            if not isinstance(stamp, Stamp):
                raise InvalidTask('capture-time map observation required')
            context = self.context()
            if context.frame_id != catalog.map.frame_id or (context.map_id, context.map_revision) != (catalog.map.map_id, catalog.map.revision):
                raise InvalidTask('placement_navigation_context_mismatch')
            current_xy_yaw, height = self.current()
            profile = self.profile
            if operation == 'pick':
                if self.pick_height_for is not None:
                    fixed_height = finite(self.pick_height_for(target), 'selected lift height',
                                          profile.lift_min_m, profile.lift_max_m)
                    profile = replace(profile, heights_m=(fixed_height,))
                elif len(profile.heights_m) != 1:
                    raise InvalidTask('single_pick_lift_setting_required')
            backend = self.feasibility_factory(xyz, operation, target)
            plan = PlacementPlanner(profile, backend).plan(target_m=xyz, stamp=stamp, context=context,
                now_s=now_s, scene_revision=self.scene(), grasp_ids=self.grasps(target, operation),
                current_xy_yaw=current_xy_yaw, current_height_m=height, operation=operation,
                maximum_age_s=self.target_age)
            if plan['status'] != 'CANDIDATES_READY':
                raise InvalidTask('no_feasible_base_lift_combination')
            selected = plan['candidates'][0]
            x, y, yaw = selected['base_xy_yaw']
            nav = catalog.map.plan(NavigateRequest(selected['candidate_id'], catalog.map.map_id,
                catalog.map.revision, catalog.map.floor_id, catalog.map.frame_id,
                x, y, catalog.map.floor_z, yaw), catalog.xy())
            self.selected[key] = dict(candidate=deepcopy(selected), context=context,
                scene_revision=self.scene(), expires_s=now_s+self.alignment_budget,
                capture_id=stamp.capture_id)
            catalog.navigation_goal = deepcopy(nav['goal'])
            return nav
        if skill.startswith('adjust_lift_') or skill in {'pick', 'place'}:
            entry = self.selected.get(key)
            if not entry:
                raise InvalidTask('joint_placement_selection_required')
            entry['context'].check(self.context())
            if entry['scene_revision'] != self.scene() or not now_s <= entry['expires_s']:
                raise InvalidTask('placement_selection_expired_or_scene_changed')
            current, height = self.current()
            x, y, yaw = vector(current, 3, 'current pose')
            candidate = entry['candidate']
            tx, ty, tyaw = candidate['base_xy_yaw']
            if math.hypot(x-tx, y-ty) > self.xy_tolerance or abs(math.atan2(math.sin(yaw-tyaw), math.cos(yaw-tyaw))) > self.yaw_tolerance:
                raise InvalidTask('selected_base_pose_not_reached')
            if skill.startswith('adjust_lift_'):
                return {'height_um': round(candidate['lift_height_m']*1_000_000)}
            if abs(finite(height, 'current lift')-candidate['lift_height_m']) > self.height_tolerance:
                raise InvalidTask('selected_lift_height_not_reached')
            # The coarse map target is never forwarded as a post-movement arm
            # target. Reobservation and fresh grasp geometry are both required.
            target = catalog.checked_target(operation == 'place')
            if target['capture_id'] == entry['capture_id']:
                raise InvalidTask('post_alignment_reobservation_required')
            task = self.fresh_grasp_task(deepcopy(target), deepcopy(candidate), operation, parameters['object_id'])
            if (not isinstance(task, dict) or set(task) != {'operation', 'object_id', 'arm', 'target', 'position_m', 'quaternion_xyzw'} or
                    task['operation'] != operation or task['object_id'] != parameters['object_id'] or
                    task['arm'] != candidate['arm'] or task['target'] != target):
                raise InvalidTask('fresh_grasp_identity_mismatch')
            vector(task['position_m'], 3, 'fresh grasp position')
            q = vector(task['quaternion_xyzw'], 4, 'fresh grasp orientation')
            if abs(sum(v*v for v in q)-1) > 1e-6:
                raise InvalidTask('normalized grasp orientation required')
            catalog.manipulation_target = deepcopy(target)
            return deepcopy(task)
        raise InvalidTask('unsupported_placement_policy_skill')
