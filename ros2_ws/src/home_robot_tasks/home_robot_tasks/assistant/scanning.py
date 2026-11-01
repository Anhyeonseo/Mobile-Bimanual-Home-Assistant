"""L3A/B scan sessions, conservative region proposals and semantic review.

The service consumes posed map/observation records. It is not a replacement for
SLAM. No scanning UI action issues a velocity, lift or motor command.
"""
from collections import deque
from copy import deepcopy
from dataclasses import asdict
import json
import hashlib
import math

from ..fetch import InvalidTask
from ..navigation import NavigationMap, identifier
from ..navigation_fusion import fuse_navigation_map
from .contracts import Context, Stamp, document, finite, text, make_entity

ROOM_EVIDENCE = {
    'bedroom': {'bed': 3, 'wardrobe': 1, 'nightstand': 1},
    'living_room': {'sofa': 3, 'television': 1, 'coffee_table': 1},
    'kitchen': {'stove': 3, 'refrigerator': 2, 'sink': 1},
    'bathroom': {'toilet': 3, 'bathtub': 2, 'sink': 1},
    'study': {'desk': 2, 'bookshelf': 1},
}


def infer_room(categories):
    """Inspectable heuristic proposal, not a calibrated class probability."""
    observed = set(categories)
    scores = {name: sum(weight for item, weight in rule.items() if item in observed)
              for name, rule in ROOM_EVIDENCE.items()}
    best = max(scores.values(), default=0)
    candidates = sorted(k for k, v in scores.items() if v == best and v >= 2)
    return {'label': candidates[0] if len(candidates) == 1 else None,
            'status': 'proposed' if len(candidates) == 1 else 'uncertain',
            'alternatives': candidates, 'evidence': sorted(observed),
            'rule_scores': scores, 'probability': None, 'user_confirmed': False}


def propose_regions(map_document, *, erosion_cells=1, minimum_core_cells=3):
    """Split free-space cores at narrow passages, then assign free fringe cells.

This makes proposals only. Open-plan rooms can remain one region, and wide doors
can remain connected. Occupied/unknown cells are never bridged as free space.
"""
    nav = NavigationMap(map_document)
    if type(erosion_cells) is not int or not 0 <= erosion_cells <= 3:
        raise InvalidTask('bounded region erosion required')
    if type(minimum_core_cells) is not int or minimum_core_cells < 1:
        raise InvalidTask('positive region core size required')
    free = {(x, y) for y, row in enumerate(nav.rows) for x, c in enumerate(row) if c == '.'}
    core = {p for p in free if all((p[0]+dx, p[1]+dy) in free
            for dx in range(-erosion_cells, erosion_cells+1)
            for dy in range(-erosion_cells, erosion_cells+1))}
    remaining, components = set(core), []
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1))
    while remaining:
        first = min(remaining, key=lambda p: (p[1], p[0]))
        remaining.remove(first)
        q, cells = deque([first]), {first}
        while q:
            x, y = q.popleft()
            for dx, dy in directions:
                p = (x+dx, y+dy)
                if p in remaining:
                    remaining.remove(p); cells.add(p); q.append(p)
        if len(cells) >= minimum_core_cells:
            components.append(cells)
    owners, queue = {}, deque()
    for index, cells in enumerate(components):
        for p in sorted(cells):
            owners[p] = index; queue.append(p)
    while queue:
        x, y = queue.popleft()
        for dx, dy in directions:
            p = (x+dx, y+dy)
            if p in free and p not in owners:
                owners[p] = owners[(x, y)]; queue.append(p)
    regions = []
    for i in range(len(components)):
        cells = sorted(p for p, owner in owners.items() if owner == i)
        xs, ys = zip(*cells)
        regions.append({'region_id': f'region-{i+1}', 'cells': [list(p) for p in cells],
            'area_m2': len(cells)*nav.resolution**2,
            'center_xy': [nav.ox+(sum(xs)/len(xs)+.5)*nav.resolution,
                          nav.oy+(sum(ys)/len(ys)+.5)*nav.resolution],
            'classification': infer_room([])})
    return {'regions': regions, 'unassigned_free_cells': len(free-set(owners)),
            'method': 'eroded_free_space_proposals_v1', 'requires_review': True}


class ScanManager:
    def __init__(self, store, context, *, busy=lambda: False):
        self.store, self.context, self.busy = store, context, busy
        for session in store.records('scan'):
            if session['status'] == 'SCANNING':
                session.update(status='PAUSED', reason='server_restarted', motion_authorized=False)
                store.save_record('scan', session['session_id'], session)

    def sessions(self):
        return self.store.records('scan')

    def active(self):
        return [s for s in self.sessions() if s['status'] in {'SCANNING', 'PAUSED', 'REVIEW'}]

    def get(self, session_id):
        result = self.store.record('scan', identifier(session_id, 'session_id'))
        if result is None:
            raise InvalidTask('unknown_scan')
        return result

    def start(self, session_id, name, *, mode='replay'):
        identifier(session_id, 'session_id'); name = text(name, 'scan name', 80)
        if not isinstance(mode, str) or mode not in {'simulation', 'replay'}:
            raise InvalidTask('live_scan_adapter_not_commissioned')
        previous = self.store.record('scan', session_id)
        if previous:
            if (previous['name'], previous['mode']) != (name, mode):
                raise InvalidTask('scan_id_conflict')
            return previous
        if self.busy() or self.active():
            raise InvalidTask('robot_or_scan_busy')
        if len(self.sessions()) >= 64:
            raise InvalidTask('scan_history_full')
        result = dict(session_id=session_id, name=name, mode=mode, status='SCANNING', reason='',
            context=self.context.to_dict(), map_document=None, captures=[], regions=[],
            navigation_map_document=None, navigation_build=None, navigation_ready=False,
            surface_views=[], hints=['아직 지도가 없습니다. 지도와 촬영 기록을 입력하세요.'],
            coverage=None, motion_authorized=False, hardware_enabled=False)
        self.store.save_record('scan', session_id, result)
        return result

    def pause(self, session_id):
        value = self.get(session_id)
        if value['status'] not in {'SCANNING', 'PAUSED'}:
            raise InvalidTask('scan_not_scanning')
        value.update(status='PAUSED', reason='user_paused')
        self.store.save_record('scan', session_id, value)
        return value

    def resume(self, session_id):
        value = self.get(session_id)
        if value['status'] not in {'SCANNING', 'PAUSED'} or self.busy():
            raise InvalidTask('scan_cannot_resume')
        # Reopening the service doesn't authorize old clock-stamped captures.
        if value['context'] != self.context.to_dict():
            raise InvalidTask('scan_context_changed_start_new_session')
        value.update(status='SCANNING', reason='')
        self.store.save_record('scan', session_id, value)
        return value

    def ingest_map(self, session_id, map_document):
        value = self.get(session_id)
        if value['status'] != 'SCANNING':
            raise InvalidTask('scan_not_scanning')
        nav = NavigationMap(map_document)
        if (nav.map_id, nav.frame_id) != (self.context.map_id, self.context.frame_id):
            raise InvalidTask('scan_map_frame_mismatch')
        if value['map_document'] == map_document:
            return value
        if value['map_document'] and value['map_document']['revision'] == nav.revision:
            raise InvalidTask('map_revision_reused')
        # A changed SLAM revision invalidates old global-coordinate scan evidence.
        if value['map_document']:
            value['captures'], value['surface_views'] = [], []
        value['map_document'] = deepcopy(map_document)
        value.update(navigation_map_document=None, navigation_build=None, navigation_ready=False)
        value['regions'] = propose_regions(map_document)['regions']
        known = sum(c != '?' for row in nav.rows for c in row)
        cells = nav.width*nav.height
        value['coverage'] = {'known_grid_cells': known, 'grid_cells': cells,
            'known_grid_fraction': known/cells, 'whole_home_fraction': None,
            'surface_views': 0, 'note': '지도 범위 내 관측률이며 집 전체 스캔 완료율이 아닙니다.'}
        value['hints'] = ['가구 상판과 가려진 곳은 RGB-D로 따로 관측하세요.',
                          '처음 출발한 곳으로 돌아와 지도 연결 상태를 확인하세요.']
        if known < cells:
            value['hints'].insert(0, '미관측 구역이 남아 있습니다. 접근 가능한 곳부터 확인하세요.')
        self.store.save_record('scan', session_id, value)
        return value

    def depth_epoch(self, session_id):
        """Depth accumulation belongs to this scan, source map and calibration.

        The sensor/replay adapter resets its volume to this epoch before integrating
        frames. A SLAM correction or changed calibration cannot reuse old world voxels.
        """
        value = self.get(session_id)
        if not value['map_document']:
            raise InvalidTask('scanning_map_required')
        digest = hashlib.sha256(document(dict(session_id=session_id,
            context=value['context'], source_map=value['map_document'])).encode()).hexdigest()[:24]
        return 'scan-depth-'+digest

    def build_navigation(self, session_id, volume, envelope, *, now_s):
        """Generate and checkpoint the RGB-D-aware map DURING scanning.

        This is a trusted mapping adapter boundary, not a browser-supplied assertion of
        clearance. Pass a completed volume snapshot owned by the mapping worker,
        not a volume being integrated concurrently. The saved grid survives
        restart; live TravelGuard is still required.
        """
        value = self.get(session_id)
        if value['status'] != 'SCANNING' or not value['map_document'] or self.busy():
            raise InvalidTask('idle_scanning_map_required')
        if value['context'] != self.context.to_dict():
            raise InvalidTask('scan_context_changed_start_new_session')
        checkpoint = deepcopy(value)
        result = fuse_navigation_map(value['map_document'], volume, envelope,
            calibration_id=self.context.calibration_id, expected_epoch=self.depth_epoch(session_id),
            revision='scan-projection-candidate', now_s=now_s)
        # Content + body geometry + source version identify the generated grid;
        # refreshing observation time alone doesn't invent another map revision.
        digest = hashlib.sha256(document(dict(source_map=value['map_document'],
            rows=result['map_document']['rows'], envelope=asdict(envelope),
            calibration_id=self.context.calibration_id)).encode()).hexdigest()
        result['map_document']['revision'] = 'body-map-'+digest[:24]
        value['navigation_map_document'] = result.pop('map_document')
        value['navigation_build'] = dict(result, envelope=asdict(envelope),
            source_map_sha256=hashlib.sha256(document(value['map_document']).encode()).hexdigest(),
            navigation_map_sha256=hashlib.sha256(document(value['navigation_map_document']).encode()).hexdigest(),
            generated_during_scan=True, physical_map_verified=False,
            accumulated_3d_volume_persisted=False,
            projection_height_m=[volume.p.minimum_height_m, volume.p.maximum_height_m])
        value['navigation_ready'] = True  # Available for software planning; not commissioned hardware.
        value['coverage']['body_grid_known_fraction'] = sum(c != '?' for row in value['navigation_map_document']['rows'] for c in row)/(len(value['navigation_map_document']['rows'])*len(value['navigation_map_document']['rows'][0]))
        with self.store.lock:
            if self.get(session_id) != checkpoint:
                raise InvalidTask('scan_changed_during_navigation_build')
            self.store.save_record('scan', session_id, value)
        return value

    def navigation_document(self, session_id, *, required=True):
        value = self.get(session_id)
        nav, build = value.get('navigation_map_document'), value.get('navigation_build')
        if not nav or not build or not value.get('navigation_ready'):
            if required:
                raise InvalidTask('scan_height_geometry_required')
            return None
        if (build['source_map_sha256'] != hashlib.sha256(document(value['map_document']).encode()).hexdigest() or
                build['navigation_map_sha256'] != hashlib.sha256(document(nav).encode()).hexdigest() or
                build['calibration_id'] != self.context.calibration_id or
                build['depth_epoch'] != self.depth_epoch(session_id)):
            raise InvalidTask('scan_navigation_provenance_mismatch')
        NavigationMap(nav)
        return deepcopy(nav)

    def observe(self, session_id, observation, *, now_s):
        value = self.get(session_id)
        if value['status'] != 'SCANNING' or value['map_document'] is None:
            raise InvalidTask('scanning_map_required')
        if not isinstance(observation, dict) or set(observation) != {'stamp', 'xy_m', 'categories', 'surface_ids'}:
            raise InvalidTask('invalid scan observation')
        stamp = Stamp.parse(observation['stamp'])
        expected = Context(self.context.map_id, value['map_document']['revision'],
                           self.context.calibration_id, self.context.clock_id, self.context.frame_id)
        stamp.check(expected, now_s, 5)
        if not isinstance(observation['xy_m'], (list, tuple)) or len(observation['xy_m']) != 2:
            raise InvalidTask('scan XY required')
        x, y = [finite(v, 'scan coordinate', -10000, 10000) for v in observation['xy_m']]
        categories = observation['categories']
        surfaces = observation['surface_ids']
        if not isinstance(categories, list) or len(categories) > 32 or not isinstance(surfaces, list) or len(surfaces) > 32:
            raise InvalidTask('bounded scan evidence required')
        for c in categories: text(c, 'category', 80)
        for s in surfaces: identifier(s, 'surface_id')
        for old in value['captures']:
            if old['stamp']['capture_id'] == stamp.capture_id:
                if old != observation:
                    raise InvalidTask('capture_id_conflict')
                return value
        if len(value['captures']) >= 256:
            raise InvalidTask('scan_capture_budget_exceeded')
        nav = NavigationMap(value['map_document'])
        cell = (math.floor((x-nav.ox)/nav.resolution), math.floor((y-nav.oy)/nav.resolution))
        if not (0 <= cell[0] < nav.width and 0 <= cell[1] < nav.height) or nav.rows[cell[1]][cell[0]] != '.':
            raise InvalidTask('scan_pose_not_observed_free_space')
        value['captures'].append(deepcopy(observation))
        for region in value['regions']:
            if list(cell) in region['cells']:
                evidence = region['classification']['evidence'] + categories
                # New observations never overwrite a user-reviewed label.
                if not region['classification']['user_confirmed']:
                    region['classification'] = infer_room(evidence)
        value['surface_views'] = sorted(set(value['surface_views']) | set(surfaces))
        value['coverage']['surface_views'] = len(value['surface_views'])
        self.store.save_record('scan', session_id, value)
        return value

    def review(self, session_id):
        value = self.get(session_id)
        if value['status'] not in {'SCANNING', 'PAUSED', 'REVIEW'} or not value['map_document']:
            raise InvalidTask('map_required_for_review')
        value['status'] = 'REVIEW'
        self.store.save_record('scan', session_id, value)
        return value

    def label(self, session_id, region_id, label):
        value = self.get(session_id)
        if value['status'] != 'REVIEW':
            raise InvalidTask('scan_review_required')
        region = next((r for r in value['regions'] if r['region_id'] == region_id), None)
        if region is None:
            raise InvalidTask('unknown_region')
        region['classification'].update(label=text(label, 'region label', 80),
                                         user_confirmed=True, status='confirmed')
        self.store.save_record('scan', session_id, value)
        return value

    def save(self, session_id):
        value = self.get(session_id)
        if value['status'] == 'SAVED':
            return value
        if value['status'] != 'REVIEW' or self.busy():
            raise InvalidTask('idle_review_required')
        navigation_document = self.navigation_document(session_id, required=False)
        nav = NavigationMap(navigation_document or value['map_document'])
        value.update(status='SAVED', reason='', physical_map_verified=False)
        value['navigation_ready'] = navigation_document is not None
        value['saved_artifact'] = 'full_body_map_bundle' if navigation_document else 'scan_record_only'
        # Publish the immutable geometry and session checkpoint in one commit.
        maps = [value['map_document']] + ([navigation_document] if navigation_document else [])
        room_records = []
        for index, region in enumerate(value['regions'], 1):
            classification = region['classification']
            digest = hashlib.sha256((session_id+':'+region['region_id']).encode()).hexdigest()[:20]
            c = Context(self.context.map_id, nav.revision, self.context.calibration_id,
                        self.context.clock_id, self.context.frame_id)
            stamp = Stamp(max((o['stamp']['observed_s'] for o in value['captures']), default=0),
                          c, 'scan-'+digest)
            room_records.append(make_entity('room-'+digest, 'room', classification['label'] or f'구역 {index}',
                classification['label'] or 'unknown', stamp,
                position_m=[*region['center_xy'], nav.floor_z], geometry_kind='bbox_center',
                state='observed' if classification['user_confirmed'] else 'unconfirmed',
                source='simulation' if value['mode'] == 'simulation' else ('user' if classification['user_confirmed'] else 'inferred')))
        value['room_entity_ids'] = [r['entity_id'] for r in room_records]
        with self.store.lock, self.store.db:
            for map_doc in maps:
                encoded_map = document(map_doc)
                old = self.store.db.execute('SELECT body FROM assistant_maps WHERE map_id=? AND revision=?',
                                           (map_doc['map_id'], map_doc['revision'])).fetchone()
                if old and old[0] != encoded_map:
                    raise InvalidTask('map_revision_reused')
                self.store.db.execute('INSERT OR IGNORE INTO assistant_maps VALUES (?,?,?)',
                                      (map_doc['map_id'], map_doc['revision'], encoded_map))
            for room in room_records:
                self.store.db.execute('INSERT INTO assistant_entities VALUES (?,?)', (room['entity_id'], document(room)))
            self.store.db.execute('UPDATE assistant_records SET body=? WHERE kind=? AND id=?',
                                 (document(value), 'scan', session_id))
        return value

    def discard(self, session_id):
        value = self.get(session_id)
        if value['status'] == 'SAVED':
            raise InvalidTask('saved_map_is_immutable')
        value.update(status='DISCARDED', captures=[], surface_views=[], regions=[], map_document=None,
                     navigation_map_document=None, navigation_build=None, navigation_ready=False)
        self.store.save_record('scan', session_id, value)
        return value
