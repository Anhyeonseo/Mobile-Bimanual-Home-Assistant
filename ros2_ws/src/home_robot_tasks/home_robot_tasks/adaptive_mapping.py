"""Versioned local map changes from independent, fresh geometric observations.

This consumes registered sensor grids, never semantic labels or a saved map as
fresh free-space evidence. A current SLAM occupancy grid is not such a sensor
grid: the caller must rasterize the current rays, leaving unseen cells unknown.
"""
from copy import deepcopy
import hashlib
import math

from .assistant.contracts import document, finite
from .fetch import InvalidTask
from .navigation import NavigationMap, identifier


class AdaptiveMap:
    def __init__(self, map_document, *, calibration_id, clock_id, protected_cells=()):
        self.nav = NavigationMap(map_document)
        self.calibration_id = identifier(calibration_id, 'calibration')
        self.clock_id = identifier(clock_id, 'clock')
        self.protected = frozenset(tuple(c) for c in protected_cells)
        if any(len(c) != 2 or any(type(v) is not int for v in c) or
               not (0 <= c[0] < self.nav.width and 0 <= c[1] < self.nav.height)
               for c in self.protected):
            raise InvalidTask('bounded structural cells required')
        self.last_stamp = None
        self.clear_votes = {}
        self.events = []

    def update(self, *, lidar_rows, depth_rows, observed_s, now_s, capture_id,
               frame_id, calibration_id, clock_id, localization_qualified,
               full_height_qualified):
        """Unknown preserves history; occupied wins; clearing needs two views.

        Both grids share the map lattice and exposure (within the capture
        adapter's synchronization tolerance). Depth free means the entire travel
        envelope's height was observed free, not just the floor or a single ray.
        Two independent captures at least 100 ms apart prevent a one-frame hole
        from erasing an old obstacle. This is a proposal, not motion authority.
        """
        identifier(capture_id, 'capture id')
        stamp = finite(observed_s, 'observation time', 0)
        if not 0 <= finite(now_s, 'now', 0) - stamp <= .5:
            raise InvalidTask('fresh_local_geometry_required')
        if (frame_id, calibration_id, clock_id) != (
                self.nav.frame_id, self.calibration_id, self.clock_id):
            raise InvalidTask('local_geometry_context_mismatch')
        if localization_qualified is not True or full_height_qualified is not True:
            raise InvalidTask('localized_full_height_geometry_required')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise InvalidTask('duplicate_or_out_of_order_local_geometry')
        for rows in (lidar_rows, depth_rows):
            if (not isinstance(rows, list) or len(rows) != self.nav.height or
                    any(not isinstance(r, str) or len(r) != self.nav.width or
                        set(r) - set('.#?') for r in rows)):
                raise InvalidTask('registered_sensor_grid_required')
        if any(e['capture_id'] == capture_id for e in self.events):
            raise InvalidTask('capture_id_reused')
        rows = [list(r) for r in self.nav.rows]
        changed, cleared, added, votes = [], [], [], {}
        for y in range(self.nav.height):
            for x in range(self.nav.width):
                p = (x, y)
                laser, depth, old = lidar_rows[y][x], depth_rows[y][x], rows[y][x]
                new = old
                if '#' in (laser, depth):
                    new = '#'
                elif laser == depth == '.':
                    if old == '#' and p not in self.protected:
                        first = self.clear_votes.get(p, stamp)
                        votes[p] = first
                        if stamp - first >= .1:
                            new = '.'
                            votes.pop(p, None)
                    elif old != '#':
                        new = '.'
                if new != old:
                    rows[y][x] = new
                    changed.append([x, y])
                    if old == '#':
                        cleared.append([x, y])
                    if new == '#':
                        added.append([x, y])
        previous = self.nav.revision
        new_document = deepcopy(self.nav._document)
        if changed:
            new_document['rows'] = [''.join(r) for r in rows]
            new_document['revision'] = 'local-' + hashlib.sha256(document(dict(
                previous=previous, rows=new_document['rows'], capture=capture_id)).encode()).hexdigest()[:24]
        result = dict(capture_id=capture_id, observed_s=stamp, previous_revision=previous,
                      calibration_id=self.calibration_id, clock_id=self.clock_id, frame_id=self.nav.frame_id,
                      revision=new_document['revision'], changed_cells=changed,
                      cleared_cells=cleared, added_cells=added, map_document=new_document,
                      local_reobservation_required=bool(votes), motion_authorized=False)
        self.nav = NavigationMap(new_document)
        self.last_stamp, self.clear_votes = stamp, votes
        self.events.append({k: v for k, v in result.items() if k != 'map_document'})
        self.events = self.events[-256:]
        return result

    def route_affected(self, update, path_xy):
        """Includes body clearance; a changed cell beside a centreline matters."""
        padding = math.ceil((self.nav._document['robot_radius_m'] +
                             self.nav._document['clearance_m']) / self.nav.resolution)
        changed = [tuple(c) for c in update['changed_cells']]
        for point in path_xy:
            x = math.floor((finite(point['x'], 'path x') - self.nav.ox) / self.nav.resolution)
            y = math.floor((finite(point['y'], 'path y') - self.nav.oy) / self.nav.resolution)
            if any(max(abs(x-a), abs(y-b)) <= padding for a, b in changed):
                return True
        return False

    def apply_to_application(self, app, update, *, now_s):
        """Existing application cancels and confirms stop before swapping maps.

        The runtime must then provide a fresh localization and replan. No saved
        pose, pending fetch or old route is silently resumed here.
        """
        if update['previous_revision'] != app.map.revision:
            raise InvalidTask('application_map_changed')
        app.replace_map(NavigationMap(update['map_document']), now_s)

    def observe_sensors(self, *, ranges, angle_min, angle_increment, range_min,
                        range_max, lidar_xy_yaw, volume, envelope, expected_epoch,
                        observed_s, now_s, capture_id, clock_id, localization_qualified):
        """Registered current laser rays plus the existing native-depth volume.

        The trusted worker provides capture-time LiDAR pose. Missing/invalid rays
        remain unknown. This deliberately does not consume SLAM's accumulated
        occupancy as evidence that a formerly occupied cell is now clear.
        """
        from .navigation_fusion import TravelEnvelope
        from .ros_navigation_depth import projected_grid
        nav, p = self.nav, volume.p
        observed_s, now_s = finite(observed_s, 'observation time', 0), finite(now_s, 'now', 0)
        if (not isinstance(envelope, TravelEnvelope) or volume.epoch != expected_epoch or
                not expected_epoch or not volume.fresh(now_s) or
                abs(volume.last_stamp-observed_s) > .05 or
                p.calibration != self.calibration_id or p.world_frame != nav.frame_id or
                abs(p.resolution_m-nav.resolution) > 1e-9 or
                p.minimum_height_m > nav.floor_z or p.maximum_height_m < nav.floor_z+envelope.height_m or
                envelope.radius_m > nav._document['robot_radius_m']):
            raise InvalidTask('synchronized_full_body_depth_required')
        near, far = finite(range_min, 'laser near', 0), finite(range_max, 'laser far', .01, 30)
        first, increment = finite(angle_min, 'first angle'), finite(angle_increment, 'angular step')
        if (near >= far or increment == 0 or not isinstance(ranges, (list, tuple)) or
                not 2 <= len(ranges) <= 2048 or abs(increment)*len(ranges) > 2*math.pi+.1 or
                len(ranges)*math.ceil(2*far/nav.resolution) > p.maximum_ray_steps or
                not isinstance(lidar_xy_yaw, (list, tuple)) or len(lidar_xy_yaw) != 3):
            raise InvalidTask('bounded laser scan and capture pose required')
        px, py, yaw = [finite(v, 'laser pose') for v in lidar_xy_yaw]
        laser = [['?']*nav.width for _ in range(nav.height)]
        # Reuse the depth ray traversal: same lattice, endpoint excluded.
        for i, value in enumerate(ranges):
            if type(value) not in (float, int) or not math.isfinite(value) or not near <= value <= far:
                continue
            angle = first+i*increment+yaw
            end = (px+value*math.cos(angle), py+value*math.sin(angle), 0.)
            for a, b, _ in volume._ray((px,py,0.), end):
                x, y = a-round(nav.ox/nav.resolution), b-round(nav.oy/nav.resolution)
                if 0 <= x < nav.width and 0 <= y < nav.height and laser[y][x] != '#':
                    laser[y][x] = '.'
            x = math.floor((end[0]-nav.ox)/nav.resolution)
            y = math.floor((end[1]-nav.oy)/nav.resolution)
            if 0 <= x < nav.width and 0 <= y < nav.height:
                laser[y][x] = '#'
        projection = projected_grid(volume, origin_xy=(nav.ox,nav.oy), width=nav.width,
                                    height=nav.height, now=now_s)
        chars = {100:'#', 0:'.', -1:'?'}
        depth = [''.join(chars[v] for v in projection[y*nav.width:(y+1)*nav.width])
                 for y in range(nav.height)]
        return self.update(lidar_rows=[''.join(r) for r in laser], depth_rows=depth,
            observed_s=observed_s, now_s=now_s, capture_id=capture_id,
            frame_id=nav.frame_id, calibration_id=p.calibration, clock_id=clock_id,
            localization_qualified=localization_qualified, full_height_qualified=True)
