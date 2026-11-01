"""Explicit synthetic household fixture and geometric planning oracle.

These are integration fixtures, not measured SO101 dimensions, IK, contact
physics or a device driver. All outputs carry simulation provenance.
"""
import math
from copy import deepcopy

from ..fetch import InvalidTask
from ..navigation import NavigateRequest
from .contracts import Stamp, make_entity
from .placement import Check, REQUIRED_PHASES, PLACE_PHASES


def seed_demo(store, context, now_s):
    stamp = Stamp(now_s, context, 'synthetic-capture')
    definitions = [
        make_entity('living_room', 'room', '거실', 'living_room', stamp,
                    aliases=['living room'], source='simulation'),
        make_entity('bedroom', 'room', '침실', 'bedroom', stamp,
                    aliases=['bedroom'], source='simulation'),
        make_entity('coffee_table', 'surface', '탁자', 'table', stamp,
                    aliases=['거실 탁자', '테이블', 'table'], room_id='living_room',
                    position_m=[3, 3, .7], extent_m=[1, .6, .04], geometry_kind='support_surface', source='simulation'),
        make_entity('nightstand', 'surface', '협탁', 'nightstand', stamp,
                    aliases=['침실 협탁', 'nightstand'], room_id='bedroom',
                    position_m=[7, 3, .8], extent_m=[.6, .6, .04], geometry_kind='support_surface', source='simulation'),
        make_entity('marker-1', 'object', '검은색 마커펜', 'marker', stamp,
                    aliases=['마커펜', 'black marker'], attributes={'color': 'black'}, room_id='living_room',
                    support_id='coffee_table', position_m=[3, 3, .73], extent_m=[.14, .02, .02],
                    geometry_kind='bbox_center', source='simulation'),
        make_entity('cup-1', 'object', '검은 컵 A', 'cup', stamp,
                    attributes={'color': 'black'}, room_id='living_room', support_id='coffee_table',
                    position_m=[2.7, 3.1, .76], extent_m=[.08, .08, .12], geometry_kind='bbox_center', source='simulation'),
        make_entity('cup-2', 'object', '검은 컵 B', 'cup', stamp,
                    attributes={'color': 'black'}, room_id='living_room', support_id='coffee_table',
                    position_m=[3.3, 3.1, .76], extent_m=[.08, .08, .12], geometry_kind='bbox_center', source='simulation'),
    ]
    for value in definitions:
        old = store.get(value['entity_id'])
        if old and old['source'] != 'simulation':
            raise InvalidTask('cannot_replace_measured_entity_with_demo')
        value['revision'] = old['revision']+1 if old else 1
        store.put(value, expected_revision=old['revision'] if old else None)
    return len(definitions)


class SyntheticFeasibility:
    """Checks every synthetic stage against a reach shell and optional boxes.

The reach shell is deliberately labelled simulation. A physical backend must
supply calibrated IK, full swept collision, payload and stability checks.
"""
    def __init__(self, navigation_map, current_xy, target, *, obstacles=()):
        self.map, self.current, self.target = navigation_map, current_xy, tuple(target)
        self.obstacles = obstacles

    def navigation(self, candidate):
        x, y, yaw = candidate.base_xy_yaw
        try:
            request = NavigateRequest(candidate.candidate_id, self.map.map_id, self.map.revision,
                self.map.floor_id, self.map.frame_id, x, y, self.map.floor_z, yaw)
            plan = self.map.plan(request, self.current)
            points = [self.current] + [(p['x'], p['y']) for p in plan['path_xy']] + [(x, y)]
            distance = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
            return Check(True, '', cost=distance, margin=.5, mode='simulation')
        except InvalidTask as error:
            return Check(False, str(error), mode='simulation')

    def view(self, candidate):
        x, y, yaw = candidate.base_xy_yaw
        camera = (x+.1*math.cos(yaw), y+.1*math.sin(yaw), .5+candidate.lift_height_m)
        dx, dy = self.target[0]-camera[0], self.target[1]-camera[1]
        bearing = abs(math.atan2(math.sin(math.atan2(dy, dx)-yaw), math.cos(math.atan2(dy, dx)-yaw)))
        distance = math.dist(camera, self.target)
        valid = .2 <= distance <= 1.5 and bearing <= math.pi/4
        return Check(valid, '' if valid else 'synthetic_camera_range_or_fov',
                     margin=max(0, 1-abs(distance-.6)/.9) if valid else 0, mode='simulation')

    def whole_motion(self, candidate):
        shoulder = (*candidate.base_xy_yaw[:2], .35+candidate.lift_height_m)
        x, y, z = self.target
        direction = (math.cos(candidate.base_xy_yaw[2]), math.sin(candidate.base_xy_yaw[2]))
        approach = (x-.06*direction[0], y-.06*direction[1], z+.06)
        lift = (x, y, z+.1)
        retreat = (x-.1*direction[0], y-.1*direction[1], z+.1)
        transport = (shoulder[0]+.18*direction[0], shoulder[1]+.18*direction[1], shoulder[2]+.1)
        route = [transport, approach, (x, y, z), lift, retreat, transport]
        margin = 1.0
        for start, end in zip(route, route[1:]):
            for i in range(21):
                point = tuple(a+(b-a)*i/20 for a, b in zip(start, end))
                reach = math.dist(point, shoulder)
                if not .05 <= reach <= .65:
                    return Check(False, 'synthetic_reach_shell', mode='simulation')
                margin = min(margin, (reach-.05)/.3, (.65-reach)/.3)
                if any(all(lo <= p <= hi for lo, p, hi in zip(box[0], point, box[1])) for box in self.obstacles):
                    return Check(False, 'synthetic_path_collision', mode='simulation')
        return Check(True, '', margin=max(0, min(1, margin)),
                     phases=tuple(sorted(REQUIRED_PHASES if candidate.operation == 'pick' else PLACE_PHASES)),
                     mode='simulation')
