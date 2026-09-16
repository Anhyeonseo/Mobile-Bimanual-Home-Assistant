"""Registered semantic places and fresh observations -> explicit port parameters.

Geometry is configuration, never inferred from a room name or language-model
output. Moving the platform invalidates robot-frame targets until recaptured.
"""
from copy import deepcopy
from .fetch import InvalidTask
from .navigation import NavigateRequest, identifier, number
from .search import SearchSession, Viewpoint


class MissionCatalog:
    def __init__(self, document, world, navigation_map, current_xy, motion, scene_revision, clock):
        expected = {'schema_version', 'mode', 'map_id', 'map_revision', 'transport', 'places', 'grasp'}
        if not isinstance(document, dict) or set(document) != expected or type(document['schema_version']) is not int or document['schema_version'] != 1:
            raise InvalidTask('invalid mission catalog')
        if document['mode'] != 'simulation':
            raise InvalidTask('physical mission catalog is not commissioned')
        if (document['map_id'], document['map_revision']) != (navigation_map.map_id, navigation_map.revision):
            raise InvalidTask('catalog map revision mismatch')
        self.doc, self.world, self.map = deepcopy(document), deepcopy(world), navigation_map
        self.xy, self.motion, self.scene, self.clock = current_xy, motion, scene_revision, clock
        t = self.doc['transport']
        if set(t) != {'arm', 'height_um'} or not isinstance(t['arm'], dict) or type(t['height_um']) is not int:
            raise InvalidTask('explicit transport configuration required')
        self.views = {}
        if not isinstance(self.doc['places'], dict) or not 1 <= len(self.doc['places']) <= 32:
            raise InvalidTask('bounded registered places required')
        for name, place in self.doc['places'].items():
            identifier(name, 'place')
            if name not in world['places'] or set(place) != {'approach', 'height_um', 'views'}:
                raise InvalidTask('unknown or incomplete registered place')
            self._navigation(place['approach'])
            if type(place['height_um']) is not int or not 0 <= place['height_um'] <= 1000000:
                raise InvalidTask('bounded lift height required')
            if not isinstance(place['views'], list) or not 1 <= len(place['views']) <= 32:
                raise InvalidTask('bounded viewpoint catalog required')
            for view in place['views']:
                if set(view) != {'id', 'regions', 'approach', 'height_um'}:
                    raise InvalidTask('invalid viewpoint')
                vid = identifier(view['id'], 'view')
                if vid in self.views or len(self.views) >= 32:
                    raise InvalidTask('duplicate or excessive viewpoints')
                if not isinstance(view['regions'], list) or not 1 <= len(view['regions']) <= 32:
                    raise InvalidTask('explicit visible-region candidates required')
                for r in view['regions']: identifier(r, 'region')
                if len(set(view['regions'])) != len(view['regions']): raise InvalidTask('duplicate region')
                if type(view['height_um']) is not int or not 0 <= view['height_um'] <= 1000000:
                    raise InvalidTask('bounded viewpoint height required')
                self._navigation(view['approach'])
                self.views[vid] = {'navigate_request': deepcopy(view['approach']),
                    'height_um': view['height_um'], 'transport_arm_route': deepcopy(t['arm'])}
        grasp = self.doc['grasp']
        if set(grasp) != {'arm', 'offset_m', 'quaternion_xyzw', 'maximum_target_age_s'} or grasp['arm'] not in ('left', 'right'):
            raise InvalidTask('explicit grasp configuration required')
        if len(grasp['offset_m']) != 3 or len(grasp['quaternion_xyzw']) != 4:
            raise InvalidTask('invalid grasp dimensions')
        offset = [number(v, 'grasp offset') for v in grasp['offset_m']]
        q = [number(v, 'grasp quaternion') for v in grasp['quaternion_xyzw']]
        if any(abs(v) > .3 for v in offset) or abs(sum(v*v for v in q)-1) > 1e-6:
            raise InvalidTask('invalid grasp geometry')
        if not 0 < number(grasp['maximum_target_age_s'], 'target age') <= 2:
            raise InvalidTask('bounded target age required')
        self.target = self.surface = None
        self.object_id = self.source_place = self.destination = None
        self.navigation_goal = None
        self.source_view = None
        self.invalid_places = {}

    def invalidate_place(self, name, reason):
        """An external scene/map observer reports changed registered geometry.

        Never reuse old approaches or move a named place from a costmap guess.
        Rebuild the catalog/runtime with a checked registration after stopping.
        """
        if name not in self.doc['places'] or not isinstance(reason, str) or not reason:
            raise InvalidTask('registered place and change reason required')
        self.invalid_places[name] = reason
        self.target = self.surface = None

    def check_place(self, name):
        if name in self.invalid_places:
            raise InvalidTask('registered place changed: '+name+': '+self.invalid_places[name])
        if name not in self.doc['places']:
            raise InvalidTask('unregistered place')

    def _navigation(self, request):
        return self.map.plan(NavigateRequest.from_dict(request), self.xy())

    def make_search(self, goal_id, parameters, now, camera):
        locations = parameters['locations']
        if not locations or len(locations) != len(set(locations)):
            raise InvalidTask('finite distinct search locations required')
        rooms = {self.world['places'][p]['room'] for p in locations}
        if len(rooms) != 1: raise InvalidTask('search locations cross rooms')
        room = next(iter(rooms))
        views = []
        for place in locations:
            self.check_place(place)
            for view in self.doc['places'][place]['views']:
                self._navigation(view['approach'])
                views.append(Viewpoint(view['id'], room, tuple(f'{place}:{r}' for r in view['regions'])))
        return SearchSession(goal_id, parameters['object_id'], room, self.scene(), views, camera, now_s=now)

    def accept_target(self, target):
        self.source_place = self.source_view = None
        self.target = deepcopy(target)
        self.target["scene_revision"] = self.scene()
        # SearchSession retains the selected view ID in its target result.
        for name, place in self.doc['places'].items():
            if any(v['id'] == target.get('view_id') for v in place['views']):
                self.check_place(name)
                self.source_place = name
                self.source_view = deepcopy(next(v for v in place["views"] if v["id"] == target.get("view_id")))
                break
        if self.source_place is None: raise InvalidTask('target has no registered viewpoint')

    def capture_sink(self, goal, skill, parameters, observation, camera, *, surface_observer):
        frame, info = observation
        if skill == 'reobserve_target':
            self.check_place(self.source_place)
            matches = [d for d in info['detections'] if d.object_id == parameters['object_id'] and d.confidence >= .7]
            if len(matches) != 1: raise InvalidTask('unambiguous target observation required')
            target = camera.locate(frame, self.motion(), matches[0].samples_uv_depth,
                                   now_s=self.clock(), requested_s=frame.observed_s)
            self.target = {**target, "scene_revision": self.scene()}
        else:
            self.check_place(parameters['destination_place'])
            # Independent surface/object observer consumes the validated frame.
            # No configured destination coordinate is fabricated as an observation.
            target = surface_observer(skill, parameters, observation)
            if not isinstance(target, dict) or target.get('capture_id') != frame.capture_id:
                raise InvalidTask('destination observation must reference this capture')
            if (target.get('pose_revision') != frame.pose_revision or target.get('observed_s') != frame.observed_s
                    or target.get('frame_id') != frame.root_frame or target.get('calibration_id') != frame.calibration_id):
                raise InvalidTask('destination observation identity mismatch')
            self.surface = {**deepcopy(target), "scene_revision": self.scene()}

    def checked_target(self, placement=False):
        self.check_place(self.destination if placement else self.source_place)
        target = self.surface if placement else self.target
        if (not isinstance(target, dict) or target.get('pose_revision') != self.motion().pose_revision
                or target.get("scene_revision") != self.scene()
                or not 0 <= self.clock()-number(target.get('observed_s'), 'target stamp') <= self.doc['grasp']['maximum_target_age_s']):
            raise InvalidTask('fresh target in current pose/scene required')
        identifier(target.get('frame_id'), 'target frame')
        identifier(target.get('capture_id'), 'capture identity')
        xyz = target.get('position_m')
        if not isinstance(xyz, (tuple,list)) or len(xyz) != 3: raise InvalidTask('3D target required')
        for v in xyz: number(v, 'target coordinate')
        return deepcopy(target)

    def resolve(self, skill, parameters, now):
        if skill.startswith('prepare_'): return deepcopy(self.doc['transport'])
        if skill == 'navigate_to':
            # Application already validates the request. Revalidate revision/path
            # through the original request at the application boundary.
            if (parameters['map_id'], parameters['map_revision']) != (self.map.map_id, self.map.revision):
                raise InvalidTask('stale navigation plan')
            self.navigation_goal = deepcopy(parameters['goal'])
            return deepcopy(parameters)
        if skill in ('navigate', 'navigate_with_load') or skill.startswith('align_'):
            if skill == 'navigate': place = self.world['rooms'][parameters['room']]['search_locations'][0]
            elif skill == 'align_for_pick': place = self.source_place
            else: place = parameters['destination_place']
            if place not in self.doc['places']: raise InvalidTask('unregistered approach')
            self.check_place(place)
            if skill != 'navigate' and skill != 'align_for_pick': self.destination = place
            approach = self.source_view['approach'] if skill == 'align_for_pick' else self.doc['places'][place]['approach']
            plan = self._navigation(approach)
            self.navigation_goal = deepcopy(plan['goal'])
            return plan
        if skill == 'search':
            self.object_id = identifier(parameters['object_id'], 'object')
            self.target = self.surface = self.source_place = self.source_view = None
            return deepcopy(parameters)
        if skill.startswith('adjust_lift_'):
            place = self.source_place if skill.endswith('_pick') else parameters['destination_place']
            self.check_place(place)
            selected = self.source_view if skill.endswith('_pick') else self.doc['places'][place]
            return {'height_um': selected['height_um']}
        if skill in ('pick', 'place'):
            target = self.checked_target(skill == 'place')
            self.manipulation_target = target
            g = self.doc['grasp']
            return {'operation': skill, 'object_id': parameters['object_id'], 'arm': g['arm'],
                    'target': target, 'position_m': [v+d for v,d in zip(target['position_m'], g['offset_m'])],
                    'quaternion_xyzw': list(g['quaternion_xyzw'])}
        return {'requested_s': now, **deepcopy(parameters)}
