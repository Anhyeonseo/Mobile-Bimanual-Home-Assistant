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
        if localization_qualified is not True or type(full_height_qualified) is not bool:
            raise InvalidTask('localized_geometry_required')
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
                elif full_height_qualified and laser == depth == '.':
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
        return self.update(**self.sensor_observation(
            ranges=ranges, angle_min=angle_min, angle_increment=angle_increment,
            range_min=range_min, range_max=range_max, lidar_xy_yaw=lidar_xy_yaw,
            volume=volume, envelope=envelope, expected_epoch=expected_epoch,
            observed_s=observed_s, now_s=now_s, capture_id=capture_id,
            clock_id=clock_id, localization_qualified=localization_qualified))

    def sensor_observation(self, *, ranges, angle_min, angle_increment, range_min,
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
            for x, y, _ in volume._ray((px-nav.ox,py-nav.oy,0.),
                                      (end[0]-nav.ox,end[1]-nav.oy,0.),
                                      resolution_m=nav.resolution):
                if 0 <= x < nav.width and 0 <= y < nav.height and laser[y][x] != '#':
                    laser[y][x] = '.'
            x = math.floor((end[0]-nav.ox)/nav.resolution)
            y = math.floor((end[1]-nav.oy)/nav.resolution)
            if 0 <= x < nav.width and 0 <= y < nav.height:
                laser[y][x] = '#'
        projection = projected_grid(volume, origin_xy=(nav.ox,nav.oy), width=nav.width,
                                    height=nav.height, now=now_s,
                                    resolution_m=nav.resolution,
                                    height_interval_m=(nav.floor_z,nav.floor_z+envelope.height_m))
        chars = {100:'#', 0:'.', -1:'?'}
        depth = [''.join(chars[v] for v in projection[y*nav.width:(y+1)*nav.width])
                 for y in range(nav.height)]
        return dict(lidar_rows=[''.join(r) for r in laser], depth_rows=depth,
            observed_s=observed_s, now_s=now_s, capture_id=capture_id,
            frame_id=nav.frame_id, calibration_id=p.calibration, clock_id=clock_id,
            localization_qualified=localization_qualified, full_height_qualified=True)


class LivingMap:
    """Automatic local updates, with a separate persistent furniture layer.

    Inputs are registered *current sensor* grids, not a SLAM map or semantic
    bounding boxes. furniture_cells require a registered identity match before
    persistence; dynamic_cells veto promotion. New furniture requires an
    explicit user-requested local scan, regardless of observation duration.
    Neither field grants clearance. Unclassified hits remain live obstacles.
    The timing defaults are development policy, not measured robot limits.
    """

    def __init__(self, map_document, *, calibration_id, clock_id, profile_id, height_m,
                 store, protected_cells=(), promotion_s=2., clearing_s=.25,
                 maximum_gap_s=1.5):
        self.base_hash = hashlib.sha256(document(map_document).encode()).hexdigest()
        self.profile = identifier(profile_id, 'travel profile')
        self.height = finite(height_m, 'travel height', .1, 3.)
        self.calibration = identifier(calibration_id, 'calibration')
        self.promotion_s = finite(promotion_s, 'promotion seconds', .1, 60)
        self.clearing_s = finite(clearing_s, 'clearing seconds', .1, 60)
        self.maximum_gap_s = finite(maximum_gap_s, 'observation gap', .1, 10)
        self.store = store
        baseline = AdaptiveMap(map_document, calibration_id=calibration_id,
                               clock_id=clock_id, protected_cells=protected_cells)
        self.protected = baseline.protected
        self.binding = dict(base_sha256=self.base_hash, calibration_id=self.calibration,
                            profile_id=self.profile, height_m=self.height,
                            persistence_policy='registered_furniture_only_v1',
                            protected_cells=sorted(map(list, self.protected)))
        saved = store.record('living_map', baseline.nav.map_id)
        if saved is not None and saved['binding'] != self.binding:
            raise InvalidTask('living_map_context_changed')
        self.stable = NavigationMap(saved['map_document'] if saved else map_document)
        if saved:
            # A snapshot may change occupancy/revision, never the lattice/body.
            original, restored = deepcopy(map_document), deepcopy(saved['map_document'])
            for key in ('revision', 'rows'):
                original.pop(key); restored.pop(key)
            if original != restored:
                raise InvalidTask('living_map_snapshot_geometry_changed')
        self.live = AdaptiveMap(self.stable._document, calibration_id=calibration_id,
                                clock_id=clock_id, protected_cells=self.protected)
        self.previous_maps = saved['previous_maps'] if saved else []
        self.registry = deepcopy(saved['registered_furniture']) if saved else {}
        self._protect_unregistered_baseline()
        self.votes = {}
        # Transient hits, votes, timestamps and motion permission do not survive
        # restart. The restored map is memory until new sensing is available.
        self.last_observation = None

    def _protect_unregistered_baseline(self):
        owned = {tuple(c) for value in self.registry.values() for c in value['cells']}
        self.live.protected = self.protected | frozenset(
            (x,y) for y,row in enumerate(self.stable.rows) for x,cell in enumerate(row)
            if cell == '#' and (x,y) not in owned)

    def _cells(self, values, field):
        cells = set()
        for cell in values:
            if (not isinstance(cell, (tuple, list)) or len(cell) != 2 or
                    any(type(v) is not int for v in cell) or
                    not (0 <= cell[0] < self.stable.width and 0 <= cell[1] < self.stable.height)):
                raise InvalidTask('invalid_' + field)
            cells.add(tuple(cell))
        return cells

    def register_furniture(self, furniture_id, cells, *, registration_id, source, now_s):
        """Trusted initial-map acceptance or user-requested local scan only.

        Perception must never call this merely because a new chair was seen.
        Registration permits later identity-matched updates; it does not turn
        a detection into an identity match or authorize motion.
        """
        identifier(furniture_id, 'registered furniture')
        identifier(registration_id, 'registration request')
        cells = self._cells(cells, 'registration_cells')
        if not cells or source not in {'accepted_initial_map', 'user_requested_local_scan'}:
            raise InvalidTask('explicit_furniture_registration_required')
        previous = self.registry.get(furniture_id)
        registration = dict(registration_id=registration_id, source=source,
                            initial_cells=sorted(map(list, cells)))
        if previous:
            if previous['registration'] != registration:
                raise InvalidTask('furniture_registration_conflict')
            return deepcopy(previous)
        if any(v['registration']['registration_id'] == registration_id for v in self.registry.values()):
            raise InvalidTask('registration_request_reused')
        occupied = self.stable if source == 'accepted_initial_map' else self.live.nav
        if source == 'user_requested_local_scan' and not self.fresh(now_s):
            raise InvalidTask('fresh_registration_scan_required')
        other = {tuple(c) for v in self.registry.values() for c in v['cells']}
        if cells & (self.protected | other) or any(occupied.rows[y][x] != '#' for x,y in cells):
            raise InvalidTask('observed_unambiguous_furniture_geometry_required')
        registry = deepcopy(self.registry)
        registry[furniture_id] = dict(registration=registration, cells=sorted(map(list, cells)))
        self.store.save_record('living_map', self.stable.map_id, dict(
            schema_version=2, binding=self.binding, map_document=self.stable._document,
            previous_maps=self.previous_maps, registered_furniture=registry,
            evidence=dict(registration_id=registration_id, source=source)))
        self.registry = registry
        self._protect_unregistered_baseline()
        return deepcopy(registry[furniture_id])

    def observe(self, *, furniture_cells=(), dynamic_cells=(), identity_matches=(), **observation):
        furniture = self._cells(furniture_cells, 'furniture_cells')
        dynamic = self._cells(dynamic_cells, 'dynamic_cells')
        # Validate without mutating committed state on rejected input.
        candidate = deepcopy(self.live)
        live_update = candidate.update(**observation)
        stamp = observation['observed_s']
        laser, depth = observation['lidar_rows'], observation['depth_rows']
        if any('#' not in (laser[y][x], depth[y][x]) for x, y in furniture | dynamic):
            raise InvalidTask('current_occupied_evidence_required')
        # A category or time spent stationary is not evidence of an existing
        # household identity. Unknown/new furniture remains transient forever
        # until explicit registration. Ambiguous identity claims stay transient.
        claims, rejected_matches = {}, []
        for match in identity_matches:
            try:
                fid = match['furniture_id']
                if (fid not in self.registry or match['capture_id'] != observation['capture_id'] or
                        match['observed_s'] != stamp or match.get('identity_verified') is not True or
                        match.get('method') not in {'continuous_track', 'appearance_geometry_reidentification'}):
                    raise InvalidTask('verified_registered_identity_required')
                for p in self._cells(match['cells'], 'matched_cells') & furniture - dynamic:
                    claims.setdefault(p, set()).add(fid)
            except (InvalidTask, KeyError, TypeError) as error:
                rejected_matches.append(str(error))
        eligible = {p: next(iter(ids)) for p, ids in claims.items() if len(ids) == 1}
        owners = {tuple(c): fid for fid, value in self.registry.items() for c in value['cells']}
        eligible = {p: fid for p,fid in eligible.items() if p not in owners or owners[p] == fid}
        rows = [list(row) for row in self.stable.rows]
        votes, added, cleared = {}, [], []
        for y in range(self.stable.height):
            for x in range(self.stable.width):
                p, old = (x, y), rows[y][x]
                hit = '#' in (laser[y][x], depth[y][x])
                free = (observation['full_height_qualified'] and laser[y][x] == depth[y][x] == '.')
                action = ('add' if hit and old != '#' and p in eligible else
                          'clear' if free and old == '#' and p in owners and p not in self.protected else None)
                if action is None:
                    continue
                previous = self.votes.get(p)
                owner = eligible[p] if action == 'add' else owners[p]
                if previous and previous[0] == action and previous[4] == owner and stamp - previous[2] <= self.maximum_gap_s:
                    _, first, _, count, _ = previous
                    count += 1
                else:
                    first, count = stamp, 1
                votes[p] = (action, first, stamp, count, owner)
                duration, minimum = (self.promotion_s, 3) if action == 'add' else (self.clearing_s, 2)
                if count >= minimum and stamp - first >= duration:
                    rows[y][x] = '#' if action == 'add' else '.'
                    (added if action == 'add' else cleared).append([x, y])
                    votes.pop(p)
        stable_doc = deepcopy(self.stable._document)
        stable_update = dict(previous_revision=self.stable.revision,
                             added_cells=added, cleared_cells=cleared,
                             changed_cells=added + cleared,
                             capture_id=observation['capture_id'], observed_s=stamp,
                             frame_id=self.stable.frame_id, calibration_id=self.calibration,
                             clock_id=self.live.clock_id, motion_authorized=False)
        # Keep newly observed hazards even if durable storage subsequently fails.
        self.live, self.last_observation = candidate, deepcopy(observation)
        if added or cleared:
            registry = deepcopy(self.registry)
            for x,y in cleared:
                registry[owners[(x,y)]]['cells'].remove([x,y])
            for x,y in added:
                current = registry[eligible[(x,y)]]['cells']
                if [x,y] not in current:
                    current.append([x,y]); current.sort()
            stable_doc['rows'] = [''.join(row) for row in rows]
            stable_doc['revision'] = 'living-' + hashlib.sha256(document(dict(
                previous=self.stable.revision, rows=stable_doc['rows'],
                capture=observation['capture_id'])).encode()).hexdigest()[:24]
            generations = [deepcopy(self.stable._document), *self.previous_maps][:2]
            # One atomic SQLite record contains both current map and rollback
            # generations: no pointer to a partially saved geometry.
            self.store.save_record('living_map', self.stable.map_id, dict(
                schema_version=2, binding=self.binding, map_document=stable_doc,
                previous_maps=generations, registered_furniture=registry, evidence=stable_update))
            self.stable = NavigationMap(stable_doc)
            self.previous_maps = generations
            self.registry = registry
        self.votes = votes
        stable_update.update(revision=self.stable.revision, map_document=deepcopy(self.stable._document))
        return dict(live=live_update, persistent=stable_update,
                    saved=bool(added or cleared), awaiting_confirmation=len(votes),
                    identity_matches_rejected=rejected_matches,
                    hardware_commands=0, motion_authorized=False)

    def observe_sensors(self, *, furniture_cells=(), dynamic_cells=(), identity_matches=(), **sensors):
        return self.observe(furniture_cells=furniture_cells, dynamic_cells=dynamic_cells, identity_matches=identity_matches,
                            **self.live.sensor_observation(**sensors))

    def observe_capture(self, *, semantic_capture=None, dynamic_cells=(), identity_matches=(), **observation):
        """Connect the existing RGBDFurnitureMapper without using old instances.

        Late/unmatched inference is optional and cannot delay hazard updates.
        Current masks plus independently verified registered identity permit
        persistence. Category recognition never registers a new household item.
        Inference stays outside the collision worker.
        """
        furniture, rejected = set(), None
        if semantic_capture is not None:
            try:
                from .assistant.contracts import Stamp
                from .assistant.furniture import _validated
                stamp = Stamp.parse(semantic_capture['provenance']['stamp'])
                context = stamp.context
                if ((context.map_id, context.frame_id, context.calibration_id, context.clock_id) !=
                        (self.stable.map_id, self.stable.frame_id, self.calibration, self.live.clock_id) or
                        context.map_revision not in {self.stable.revision, self.live.nav.revision} or
                        stamp.capture_id != observation['capture_id'] or
                        stamp.observed_s != observation['observed_s']):
                    raise InvalidTask('semantic_capture_mismatch')
                shapes = semantic_capture['observations']
                if not isinstance(shapes, list) or len(shapes) > 128:
                    raise InvalidTask('bounded_semantic_observations_required')
                for shape in shapes:
                    shape = _validated(shape, context, observation['now_s'])
                    if shape['stamp'] != stamp.to_dict():
                        raise InvalidTask('semantic_shape_capture_mismatch')
                    size, nav = shape['voxel_size_m'], self.stable
                    for x, y, z in shape['observed_voxels']:
                        if (z+1)*size <= nav.floor_z or z*size >= nav.floor_z+self.height:
                            continue
                        xs = range(max(0, math.floor((x*size-nav.ox)/nav.resolution)),
                                   min(nav.width, math.ceil(((x+1)*size-nav.ox)/nav.resolution)))
                        ys = range(max(0, math.floor((y*size-nav.oy)/nav.resolution)),
                                   min(nav.height, math.ceil(((y+1)*size-nav.oy)/nav.resolution)))
                        furniture.update((a,b) for b in ys for a in xs
                                         if '#' in (observation['lidar_rows'][b][a],
                                                    observation['depth_rows'][b][a]))
            except (InvalidTask, KeyError, TypeError, IndexError) as error:
                furniture, rejected = set(), str(error)
        result = self.observe(furniture_cells=sorted(furniture), dynamic_cells=dynamic_cells,
                              identity_matches=identity_matches, **observation)
        result['semantic_evidence_rejected'] = rejected
        return result

    def fresh(self, now_s):
        return (self.last_observation is not None and
                0 <= now_s - self.last_observation['observed_s'] <= .5)


class LivingMapNavigation:
    """Simulation integration of automatic stop/replan with a retained goal.

    Uses the existing executor/lease; never resets a controller, disables hold,
    or replays manipulation. Physical adapters are deliberately not admitted.
    New pose evidence is supplied by the caller after confirmed stop, never by
    assuming the robot stayed at its pre-cancellation position.
    """

    def __init__(self, app, mapping):
        if app.adapter.mode != 'simulation' or app.map.descriptor() != mapping.live.nav.descriptor():
            raise InvalidTask('matching_simulation_application_required')
        self.app, self.mapping = app, mapping
        self.goal = None
        self.active = None
        self.sequence = 0
        self.state = 'IDLE'
        self.reason = None
        self.stop_after = None
        self.deadline = None
        self.attempted = None

    def start(self, request, now_s):
        from .navigation import NavigateRequest
        NavigateRequest.from_dict(request)
        if self.goal is not None:
            raise InvalidTask('living_navigation_already_started')
        if not self.mapping.fresh(now_s):
            raise InvalidTask('fresh_local_geometry_required')
        self.app.submit(request, now_s)
        self.goal, self.active = deepcopy(request), request['request_id']
        self.deadline = now_s + self.app.task_timeout_s
        self.state = 'MOVING'
        return self.status()

    def status(self):
        return dict(status=self.state, reason=self.reason,
                    request_id=self.goal['request_id'] if self.goal else None,
                    active_attempt=self.active, replans=self.sequence,
                    hardware_commands=0, physical_task_completed=False)

    def cancel(self, now_s):
        if self.active and self.app.lease.owner == self.active:
            self.app.cancel(self.active, now_s)
        self.state, self.reason = 'CANCELLED', 'user_cancelled'
        return self.status()

    def tick(self, now_s, *, pose_xy=None, pose_observed_s=None):
        if self.state in {'IDLE', 'CANCELLED', 'SUCCEEDED', 'FAILED'}:
            self.app.tick(now_s)  # Cancellation still needs stop feedback.
            return self.status()
        if now_s >= self.deadline:
            if self.app.lease.owner == self.active:
                self.app.cancel(self.active, now_s)
            self.state, self.reason = 'FAILED', 'mission_timeout'
            return self.status()
        if self.app.lease.owner not in (None, self.active):
            self.state, self.reason = 'FAILED', 'control_owner_changed'
            return self.status()
        result = self.app.status(self.active)
        if self.state == 'MOVING' and result['status'] in {'SUCCEEDED', 'FAILED', 'CANCELLED'}:
            self.state, self.reason = result['status'], result['reason']
            return self.status()
        latest = self.mapping.live.nav
        changed = latest.revision != self.app.map.revision
        fresh = self.mapping.fresh(now_s)
        if self.state == 'MOVING' and (changed or not fresh):
            # Off-route observations do not interrupt the executor. Do not swap
            # its revision mid-flight; accumulate all changes against that map.
            cells = [[x, y] for y in range(latest.height) for x in range(latest.width)
                     if latest.rows[y][x] != self.app.map.rows[y][x]]
            affected = self.mapping.live.route_affected(
                {'changed_cells': cells}, self.app.plans[self.active]['path_xy'])
            if not fresh or affected:
                self.app.cancel(self.active, now_s)
                self.state = 'STOPPING'
                self.reason = 'obstacle_changed' if fresh else 'sensor_wait'
                self.stop_after = now_s
        # Process hazards before the executor can start another command.
        self.app.tick(now_s)
        if self.state == 'STOPPING':
            if self.app.lease.owner is not None:
                return self.status()
            stopped = self.app.status(self.active)
            if not stopped['simulated_stop_confirmed'] or stopped['status'] != 'CANCELLED':
                self.state, self.reason = 'FAILED', 'stop_not_confirmed'
                return self.status()
            self.stop_after = now_s
            self.state = 'WAITING_FOR_OBSERVATION'
        if self.state in {'WAITING_FOR_OBSERVATION', 'WAITING_FOR_PATH'}:
            if (not fresh or pose_xy is None or pose_observed_s is None or
                    not self.stop_after <= pose_observed_s <= now_s or now_s-pose_observed_s > .5 or
                    self.mapping.last_observation['observed_s'] < self.stop_after):
                return self.status()
            key = (latest.revision, tuple(pose_xy))
            if self.state == 'WAITING_FOR_PATH' and key == self.attempted:
                return self.status()
            self.app.replace_map(latest, now_s)
            request = deepcopy(self.goal)
            request.update(map_revision=latest.revision,
                           request_id=f'living-{hashlib.sha256(self.goal["request_id"].encode()).hexdigest()[:16]}-{self.sequence+1}')
            self.attempted = key
            try:
                self.app.localize_simulated(pose_xy, now_s)
                self.app.submit(request, now_s)
            except InvalidTask as error:
                if str(error) in {'no_path', 'point_blocked_or_insufficient_clearance'}:
                    self.state, self.reason = 'WAITING_FOR_PATH', '주변이 비면 이동을 이어갑니다'
                else:
                    self.state, self.reason = 'FAILED', str(error)
                return self.status()
            self.sequence += 1
            self.active = request['request_id']
            self.state, self.reason = 'MOVING', None
        return self.status()
