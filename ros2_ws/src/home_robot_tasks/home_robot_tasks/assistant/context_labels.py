"""Evidence-linked household names for offline, conversational image review.

Names refer to historical furniture observations, not executable goals. The
reviewer supplies interpretations and image references; positions come only
from the existing RGB-D observations. Raw classifications remain immutable.
"""
from copy import deepcopy
import hashlib
import json
import unicodedata

import numpy as np

from ..navigation import identifier


def digest(value):
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True,
                                    allow_nan=False).encode()).hexdigest()


def _name(value):
    if not isinstance(value, str) or not value.strip() or len(value) > 100:
        raise ValueError('bounded nonempty name required')
    return value.strip()


def _views(indices, frames):
    if not indices or len(set(indices)) != len(indices):
        raise ValueError('distinct evidence views required')
    if any(type(i) is not int or not 0 <= i < len(frames) for i in indices):
        raise ValueError('unknown evidence view')
    return [dict(view=i, image=frames[i]['image'],
                 capture=deepcopy(frames[i]['capture'])) for i in indices]


def build_context_labels(report, review, *, report_sha256):
    """Compile a complete review: every raw instance is grouped or suppressed."""
    if review['source_report_sha256'] != report_sha256:
        raise ValueError('review source hash mismatch')
    instances = {i['instance_id']: i for i in report['instances']}
    frames = report['frames']
    reviewed = review['reviewed_views']
    _views(reviewed, frames)
    if set(reviewed) != set(range(len(frames))):
        raise ValueError('complete declared image review required')
    identifier(review['review_id'], 'review id')
    zones = deepcopy(review['zones'])
    zone_ids = set()
    for zone in zones:
        identifier(zone['id'], 'zone id')
        if zone['id'] in zone_ids:
            raise ValueError('duplicate zone id')
        zone_ids.add(zone['id'])
        zone['label'] = _name(zone['label'])
        zone['aliases'] = [_name(n) for n in zone.get('aliases', [])]
        zone['evidence'] = _views(zone.pop('evidence_views'), frames)
        # These are functional areas inferred from images, not segmented rooms.
        zone.update(room_boundary_verified=False, polygon=None,
                    name_state='proposed', user_confirmed=False)
    seen, keys, entities = set(), set(), []
    for item in review['entities']:
        item = deepcopy(item)
        identifier(item['id'], 'household id')
        if item['id'] in keys or item['id'] in zone_ids:
            raise ValueError('duplicate household id')
        keys.add(item['id'])
        members = item['member_instance_ids']
        if (not members or len(set(members)) != len(members) or
                not set(members) <= instances.keys() or seen.intersection(members)):
            raise ValueError('unknown or reused furniture instance')
        if item['zone_id'] not in zone_ids:
            raise ValueError('unknown zone')
        evidence = _views(item.pop('evidence_views'), frames)
        if not set(item['display_views']) <= {e['view'] for e in evidence}:
            raise ValueError('display image without evidence')
        associated = {a.get('instance_id') for e in evidence
                      for a in frames[e['view']]['mapping']['associations']}
        if not set(members) <= associated:
            raise ValueError('member without referenced image evidence')
        observed = [instances[k] for k in members]
        if any(o['context'] != report['context'] for o in observed):
            raise ValueError('furniture context mismatch')
        sizes = {o['voxel_size_m'] for o in observed}
        if len(sizes) != 1:
            raise ValueError('incompatible voxel sizes')
        cells = np.unique(np.concatenate([o['observed_voxels'] for o in observed]), axis=0)
        xyz = (cells + .5) * sizes.pop()
        item.update(label=_name(item['label']),
                    aliases=[_name(n) for n in item.get('aliases', [])],
                    evidence=evidence, name_state='proposed', user_confirmed=False,
                    association='conversational_multiview_visual_review_estimate',
                    raw_categories=sorted({o['category'] for o in observed}),
                    geometry_source_digest=digest(observed),
                    observed_anchor_xyz_m=np.median(xyz, axis=0).tolist(),
                    observed_bounds_m={'min': xyz.min(0).tolist(), 'max': xyz.max(0).tolist()},
                    last_observed_s=max(o['last_observed_s'] for o in observed),
                    extent_complete=False, identity_verified=False,
                    navigation_goal_available=False, reobserve_required=True)
        seen.update(members)
        entities.append(item)
    suppressed = deepcopy(review.get('suppressed', []))
    for item in suppressed:
        key = item['instance_id']
        if key not in instances or key in seen:
            raise ValueError('unknown or reused suppressed instance')
        _name(item['reason'])
        item['geometry_retained'] = True
        seen.add(key)
    if seen != instances.keys():
        raise ValueError('unreviewed instances remain')
    return dict(schema_version=1, review_id=review['review_id'], revision=1,
                method='assistant_visual_review_in_conversation',
                context=deepcopy(report['context']), source_report_sha256=report_sha256,
                source_map_sha256=report['map_sha256'],
                zones=zones, entities=entities, suppressed=suppressed,
                questions=deepcopy(review.get('questions', [])),
                evidence_views_reviewed=sorted(reviewed),
                source_geometry_modified=False, motion_authorized=False,
                external_model_api_calls=0, user_labels_confirmed=False)


def rename_context_label(bundle, key, label, aliases, *, expected_revision):
    """A personal name changes no geometry, timestamp, or physical identity."""
    if bundle['revision'] != expected_revision:
        raise ValueError('label revision conflict')
    result = deepcopy(bundle)
    match = [i for i in result['zones'] + result['entities'] if i['id'] == key]
    if len(match) != 1:
        raise ValueError('unknown household id')
    match[0].update(label=_name(label), aliases=list(dict.fromkeys(_name(n) for n in aliases)),
                    name_state='user_named', user_confirmed=True)
    result['revision'] += 1
    result['user_labels_confirmed'] = all(i['user_confirmed'] for i in result['zones'] + result['entities'])
    return result


def resolve_context_label(bundle, phrase):
    """Exact normalized names/aliases; duplicate names return every candidate."""
    def normalize(text):
        return ''.join(unicodedata.normalize('NFKC', text).casefold().split())
    query = normalize(_name(phrase))
    zones = {z['id']: z for z in bundle['zones']}
    found = []
    for item in bundle['zones'] + bundle['entities']:
        names = [item['label'], *item['aliases']]
        if 'zone_id' in item:
            zone = zones[item['zone_id']]
            names += [z + ' ' + n for z in [zone['label'], *zone['aliases']]
                      for n in [item['label'], *item['aliases']]]
        if query in {normalize(n) for n in names}:
            found.append(deepcopy(item))
    return dict(status='NOT_FOUND' if not found else 'MATCH' if len(found) == 1 else 'AMBIGUOUS',
                candidates=found, motion_authorized=False, reobserve_required=True)


def load_household_profile(directory, *, source_map=None):
    """Load the accepted recorded map and names without promoting geometry."""
    from pathlib import Path
    from .contracts import Context
    from ..fetch import InvalidTask
    root = Path(directory)
    labels = json.loads((root/'household-labels.json').read_text())
    copied = (root/'source-map.json').read_bytes()
    raw = Path(source_map).read_bytes() if source_map else copied
    if hashlib.sha256(raw).hexdigest() != labels['source_map_sha256'] or json.loads(raw)!=json.loads(copied):
        raise InvalidTask('household_map_hash_mismatch')
    Context.parse(labels['context'])
    occupancy = json.loads(raw)
    w, h = occupancy['width'], occupancy['height']
    if type(w) is not int or type(h) is not int or not 3 <= w <= 256 or not 3 <= h <= 256:
        raise InvalidTask('household_map_dimensions')
    grid = np.asarray(occupancy['data'])
    if grid.shape != (w*h,) or not np.isfinite(grid).all() or np.any((grid < -1) | (grid > 100)):
        raise InvalidTask('household_map_cells')
    grid = grid.reshape(h, w)
    context = labels['context']
    # NavigationMap is reused for persistence/coordinates only. This profile
    # disables application admission; the square-inflated planner is not used.
    display_map = dict(schema_version=1, map_id=context['map_id'], revision=context['map_revision'],
        floor_id='recorded-floor', frame_id=context['frame_id'], floor_z_m=0.,
        resolution_m=occupancy['resolution'], origin=dict(zip(('x','y'),occupancy['origin'])),
        rows=[''.join('?' if v < 0 else '#' if v >= 65 else '.' for v in row) for row in grid],
        robot_radius_m=.01, clearance_m=0., surface_tolerance_m=.03)
    from ..navigation import NavigationMap
    NavigationMap(display_map)
    return {'labels': labels, 'display_map': display_map,
            'source_map_sha256': labels['source_map_sha256'],
            'profile_sha256': digest(labels), 'read_only_geometry': True,
            'evidence_root':str((root/'evidence').resolve()),
            'evidence_sha256':json.loads((root/'evidence-hashes.json').read_text()) if (root/'evidence-hashes.json').exists() else {}}


class HouseholdMemory:
    """Persistent names and recorded anchors, not physical support surfaces."""
    def __init__(self, store, profile, *, clock_id):
        from .contracts import Context
        from ..fetch import InvalidTask
        self.store, self.map = store, deepcopy(profile['display_map'])
        self.context = Context(**{**profile['labels']['context'], 'clock_id': clock_id})
        self.store.add_map(self.map)
        self.key = identifier(profile['labels']['review_id'], 'review id')
        previous = store.record('household_profile', self.key)
        if previous and previous['profile_sha256'] != profile['profile_sha256']:
            raise InvalidTask('household_profile_changed_use_new_state_directory')
        if not previous:
            if store.entities(self.context):
                raise InvalidTask('household_import_requires_empty_map_memory')
            with store.lock, store.db:
                # Validate all entities before inserting; WorldStore owns the
                # same SQLite transaction and relation checks.
                entities = self._entities(profile['labels'])
                for value in entities:
                    store._relations(value)
                    store.db.execute('INSERT INTO assistant_entities VALUES (?,?)',
                                     (value['entity_id'], json.dumps(value, ensure_ascii=False)))
                store.db.execute('INSERT INTO assistant_records VALUES (?,?,?)',
                    ('household_profile', self.key, json.dumps(profile, ensure_ascii=False)))
        self.profile = previous or deepcopy(profile)

    def _entities(self, bundle):
        from .contracts import Stamp, Context, make_entity
        from ..fetch import InvalidTask
        entries = bundle['zones']+bundle['entities']
        if len(entries) > 256 or len({x['id'] for x in entries}) != len(entries):
            raise InvalidTask('bounded_unique_household_names_required')
        result = []
        for value in entries:
            furniture = 'zone_id' in value
            stamp = Stamp(value.get('last_observed_s', 0.), Context.parse(bundle['context']), 'name-'+value['id'])
            result.append(make_entity(value['id'], 'surface' if furniture else 'room',
                value['label'], value.get('category','functional_area'), stamp,
                aliases=value['aliases'], room_id=value.get('zone_id'),
                position_m=value.get('observed_anchor_xyz_m'),
                geometry_kind='bbox_center' if furniture else 'none',
                state='observed' if value['user_confirmed'] else 'unconfirmed', source='inferred',
                attributes={'name_status':'confirmed' if value['user_confirmed'] else 'proposed',
                            'geometry_authority':'historical_anchor_only'}))
        return result

    def status(self):
        values = self.store.entities(self.context)
        return dict(review_id=self.key, source_map_sha256=self.profile['source_map_sha256'],
            names=[v for v in values if v['kind'] != 'object'],
            name_evidence={x['id']:x.get('display_views',[]) for x in self.profile['labels']['entities']},
            observations=self.store.records('object_capture'),
            mode='recorded_home_review', physical_execution_enabled=False,
            map_document={**self.map, 'mode':'recorded_home_review', 'display_only':True},
            room_boundaries_verified=False, current_robot_pose_available=False)

    def evidence(self, view):
        import base64
        from pathlib import Path
        from ..fetch import InvalidTask
        if type(view) is not int or view not in self.profile['labels']['evidence_views_reviewed']:
            raise InvalidTask('unknown_evidence_view')
        name = f'view-{view:02}.jpg'
        expected = self.profile['evidence_sha256'].get(name)
        if not expected:
            raise InvalidTask('evidence_unavailable')
        data = (Path(self.profile['evidence_root'])/name).read_bytes()
        if len(data)>2_000_000 or hashlib.sha256(data).hexdigest()!=expected:
            raise InvalidTask('evidence_hash_mismatch')
        return dict(mime_type='image/jpeg',base64=base64.b64encode(data).decode(),historical=True)

    def name(self, entity_id, label, aliases, expected_revision):
        from ..fetch import InvalidTask
        from .contracts import entity
        current = self.store.get(entity_id)
        if not current or current not in self.store.entities(self.context) or current['kind'] not in {'room','surface'}:
            raise InvalidTask('known_household_name_required')
        if type(expected_revision) is not int or expected_revision != current['revision']:
            raise InvalidTask('entity_revision_conflict')
        updated = entity({**current,'label':label,'aliases':aliases, 'state':'observed',
            'attributes':{**current['attributes'],'name_status':'confirmed'},'revision':expected_revision+1})
        # Source, capture stamp, anchors and measured identity do not change.
        return self.store.put(updated, expected_revision=expected_revision)
