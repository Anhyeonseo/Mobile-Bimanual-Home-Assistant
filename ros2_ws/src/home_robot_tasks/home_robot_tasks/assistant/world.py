"""L3B persistent semantic memory. Writes are atomic and revisions immutable."""
from copy import deepcopy
import json
import sqlite3
import threading

from ..fetch import InvalidTask
from ..navigation import NavigationMap, identifier
from .contracts import Context, Stamp, document, entity, finite, text


class WorldStore:
    def __init__(self, path=':memory:'):
        self.lock = threading.RLock()
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute('PRAGMA foreign_keys=ON')
        self.db.executescript('''
            CREATE TABLE IF NOT EXISTS assistant_maps (
                map_id TEXT NOT NULL, revision TEXT NOT NULL, body TEXT NOT NULL,
                PRIMARY KEY(map_id,revision));
            CREATE TABLE IF NOT EXISTS assistant_entities (id TEXT PRIMARY KEY, body TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS assistant_events (id TEXT PRIMARY KEY, body TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS assistant_records (
                kind TEXT NOT NULL, id TEXT NOT NULL, body TEXT NOT NULL, PRIMARY KEY(kind,id));
        ''')
        self.db.commit()

    def close(self):
        self.db.close()

    def record(self, kind, key):
        with self.lock:
            row = self.db.execute('SELECT body FROM assistant_records WHERE kind=? AND id=?', (kind, key)).fetchone()
            return json.loads(row[0]) if row else None

    def records(self, kind):
        with self.lock:
            return [json.loads(r[0]) for r in self.db.execute('SELECT body FROM assistant_records WHERE kind=? ORDER BY id', (kind,))]

    def save_record(self, kind, key, body):
        identifier(key, 'record id')
        encoded = document(body)
        with self.lock, self.db:
            self.db.execute('INSERT INTO assistant_records VALUES (?,?,?) ON CONFLICT(kind,id) DO UPDATE SET body=excluded.body', (kind, key, encoded))

    def add_map(self, value):
        validated = NavigationMap(value)
        encoded = document(value)
        with self.lock, self.db:
            old = self.db.execute('SELECT body FROM assistant_maps WHERE map_id=? AND revision=?',
                                 (validated.map_id, validated.revision)).fetchone()
            if old and old[0] != encoded:
                raise InvalidTask('map_revision_reused')
            self.db.execute('INSERT OR IGNORE INTO assistant_maps VALUES (?,?,?)', (validated.map_id, validated.revision, encoded))
        return validated

    def get_map(self, map_id, revision):
        with self.lock:
            row = self.db.execute('SELECT body FROM assistant_maps WHERE map_id=? AND revision=?', (map_id, revision)).fetchone()
        if not row:
            raise InvalidTask('unknown_map_revision')
        return json.loads(row[0])

    def get(self, key):
        with self.lock:
            row = self.db.execute('SELECT body FROM assistant_entities WHERE id=?', (key,)).fetchone()
            return json.loads(row[0]) if row else None

    def entities(self, context=None):
        with self.lock:
            values = [json.loads(r[0]) for r in self.db.execute('SELECT body FROM assistant_entities ORDER BY id')]
        if context:
            values = [v for v in values if all(v['stamp']['context'][k] == getattr(context, k)
                      for k in ('map_id', 'map_revision', 'frame_id', 'calibration_id'))]
        return values

    def _relations(self, value):
        for key, kind in (('room_id', 'room'), ('support_id', 'surface')):
            target = value[key]
            if target:
                parent = self.get(target)
                if not parent or parent['kind'] != kind:
                    raise InvalidTask(f'invalid {key}')
                for component in ('map_id', 'map_revision', 'frame_id', 'calibration_id'):
                    if parent['stamp']['context'][component] != value['stamp']['context'][component]:
                        raise InvalidTask('relation_context_mismatch')
                if key == 'support_id' and parent['room_id'] != value['room_id']:
                    raise InvalidTask('support_room_mismatch')

    def put(self, value, *, expected_revision=None):
        value = entity(value)
        c = value['stamp']['context']
        map_document = self.get_map(c['map_id'], c['map_revision'])
        if map_document['frame_id'] != c['frame_id']:
            raise InvalidTask('entity_map_frame_mismatch')
        with self.lock, self.db:
            self._relations(value)
            previous = self.get(value['entity_id'])
            if previous == value:
                return previous
            if previous and previous['stamp']['context'] == value['stamp']['context'] and value['stamp']['observed_s'] < previous['stamp']['observed_s']:
                raise InvalidTask('out_of_order_observation')
            if previous is None:
                if expected_revision is not None or value['revision'] != 1:
                    raise InvalidTask('entity_revision_conflict')
            elif expected_revision != previous['revision'] or value['revision'] != previous['revision'] + 1:
                raise InvalidTask('entity_revision_conflict')
            self.db.execute('INSERT INTO assistant_entities VALUES (?,?) ON CONFLICT(id) DO UPDATE SET body=excluded.body',
                            (value['entity_id'], document(value)))
        return deepcopy(value)

    def aliases(self, key, aliases, expected_revision):
        identifier(key, 'entity_id')
        if type(expected_revision) is not int or expected_revision < 1:
            raise InvalidTask('positive expected entity revision required')
        with self.lock:
            previous = self.get(key)
            if not previous:
                raise InvalidTask('unknown_entity')
            revised = deepcopy(previous)
            revised.update(aliases=aliases, revision=previous['revision'] + 1)
            # User labels never refresh sensor time or assert observed geometry.
            return self.put(revised, expected_revision=expected_revision)

    def event(self, event_id, body):
        """Same event ID is immutable across retries and process restarts."""
        identifier(event_id, 'event_id')
        encoded = document(body)
        with self.lock, self.db:
            row = self.db.execute('SELECT body FROM assistant_events WHERE id=?', (event_id,)).fetchone()
            if row and row[0] != encoded:
                raise InvalidTask('event_id_conflict')
            self.db.execute('INSERT OR IGNORE INTO assistant_events VALUES (?,?)', (event_id, encoded))

    def mark_state(self, key, state, stamp, *, expected_revision, support_id=None,
                   position_m=None, evidence=None):
        if state not in {'occluded', 'stale', 'carried', 'released_unconfirmed', 'placed'}:
            raise InvalidTask('unsupported memory transition')
        with self.lock:
            value = self.get(key)
            if not value or value['kind'] != 'object':
                raise InvalidTask('known object required')
            old = Stamp.parse(value['stamp'])
            old.context.check(stamp.context)
            if stamp.observed_s < old.observed_s:
                raise InvalidTask('out_of_order_observation')
            if state in {'carried', 'released_unconfirmed', 'placed'} and not (isinstance(evidence, dict) and
                    evidence.get('verified') is True and evidence.get('entity_id') == key and
                    evidence.get('capture_id') == stamp.capture_id):
                raise InvalidTask('independent_object_evidence_required')
            if state == 'placed':
                if value['state'] not in {'carried', 'released_unconfirmed'} or not support_id or position_m is None:
                    raise InvalidTask('placement_requires_carried_object_and_surface')
                support = self.get(support_id)
                if not support:
                    raise InvalidTask('unknown_surface')
                value.update(room_id=support['room_id'], support_id=support_id,
                             position_m=position_m, geometry_kind='surface_point')
            elif state == 'released_unconfirmed':
                if value['state'] != 'carried':
                    raise InvalidTask('release_requires_carried_object')
            elif state == 'carried':
                if value['state'] not in {'observed', 'placed'}:
                    raise InvalidTask('fresh_object_required_before_carry')
                value.update(support_id=None, position_m=None, geometry_kind='none')
            value.update(state=state, stamp=stamp.to_dict(), revision=value['revision'] + 1)
            return self.put(value, expected_revision=expected_revision)

    def candidates(self, *, context, category=None, attributes=None, room_id=None,
                   support_id=None, now_s, maximum_age_s=60):
        """Return all matches and freshness; caller must not silently pick rank 1."""
        now = finite(now_s, 'now', 0)
        maximum_age_s = finite(maximum_age_s, 'age', 0)
        matches = []
        for value in self.entities(context):
            if value['kind'] != 'object' or value['state'] in {'carried', 'released_unconfirmed'}:
                continue
            if category and category not in {value['category'], value['label'], *value['aliases']}:
                continue
            if room_id and value['room_id'] != room_id:
                continue
            if support_id and value['support_id'] != support_id:
                continue
            if any(value['attributes'].get(k) != v for k, v in (attributes or {}).items()):
                continue
            stamp = Stamp.parse(value['stamp'])
            fresh = (stamp.context == context and value['state'] in {'observed', 'placed'} and
                     0 <= now-stamp.observed_s <= maximum_age_s)
            matches.append({'entity': value, 'fresh': fresh,
                            'requires_reobservation': True, 'physical_grasp_pose_available': False})
        return matches
