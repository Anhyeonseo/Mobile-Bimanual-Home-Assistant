"""SQLite names and measured stop registrations, independent of task execution.

NULL means unknown, never (0, 0, 0). This registry does not commission a robot,
check reachability, or connect the physical FSM to Nav2.
"""
import math
from pathlib import Path
import re
import sqlite3
import unicodedata


SCHEMA = '''
CREATE TABLE places (
    place_id TEXT PRIMARY KEY,
    label TEXT NOT NULL,
    room_id TEXT,
    map_id TEXT, map_revision TEXT, frame_id TEXT,
    furniture_x_m REAL, furniture_y_m REAL, furniture_yaw_rad REAL,
    stop_x_m REAL, stop_y_m REAL, stop_yaw_rad REAL,
    surface_height_m REAL,
    CHECK ((map_id IS NULL AND map_revision IS NULL AND frame_id IS NULL)
        OR (map_id IS NOT NULL AND map_revision IS NOT NULL AND frame_id IS NOT NULL)),
    CHECK ((furniture_x_m IS NULL AND furniture_y_m IS NULL AND furniture_yaw_rad IS NULL)
        OR (furniture_x_m IS NOT NULL AND furniture_y_m IS NOT NULL AND furniture_yaw_rad IS NOT NULL AND map_id IS NOT NULL)),
    CHECK ((stop_x_m IS NULL AND stop_y_m IS NULL AND stop_yaw_rad IS NULL)
        OR (stop_x_m IS NOT NULL AND stop_y_m IS NOT NULL AND stop_yaw_rad IS NOT NULL AND map_id IS NOT NULL)),
    CHECK (surface_height_m IS NULL OR surface_height_m BETWEEN 0 AND 3)
);
CREATE TABLE aliases (
    place_id TEXT NOT NULL REFERENCES places(place_id),
    name TEXT NOT NULL,
    normalized_name TEXT NOT NULL,
    PRIMARY KEY (place_id, normalized_name)
);
CREATE INDEX aliases_lookup ON aliases(normalized_name);
PRAGMA user_version = 1;
'''


def _text(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise ValueError(name + ' must be a nonempty string of up to 128 characters')
    return value.strip()


def normalize_name(value):
    return ''.join(unicodedata.normalize('NFKC', _text(value, 'place name')).casefold().split())


class PlaceRegistry:
    def __init__(self, path, *, create=False):
        if str(path) != ':memory:':
            path = Path(path)
            if not path.is_file() and not create:
                raise FileNotFoundError('장소 DB가 없습니다. 먼저 init을 실행하세요: ' + str(path))
            if create:
                path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(path)
        self.db.row_factory = sqlite3.Row
        self.db.execute('PRAGMA foreign_keys=ON')
        version = self.db.execute('PRAGMA user_version').fetchone()[0]
        if create and version == 0:
            tables = self.db.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall()
            if tables:
                self.db.close()
                raise ValueError('기존 다른 용도의 DB에는 장소 테이블을 만들지 않습니다')
            self.db.executescript(SCHEMA)
        elif version != 1:
            self.db.close()
            raise ValueError('unsupported place registry schema version')

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.db.close()

    def seed(self, document):
        if not isinstance(document, dict) or type(document.get('schema_version')) is not int or document['schema_version'] != 1:
            raise ValueError('place seed schema_version must be 1')
        places = document['places']
        if not isinstance(places, list) or not places:
            raise ValueError('place seed must contain named places')
        prepared, seen = [], set()
        for place in places:
            if not isinstance(place, dict):
                raise ValueError('each place must be an object')
            if set(place) - {'place_id', 'label', 'aliases', 'room_id', 'map', 'furniture_pose', 'stop_pose', 'surface_height_m'}:
                raise ValueError('unknown place seed fields')
            pid = _text(place['place_id'], 'place_id')
            if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', pid) or pid in seen:
                raise ValueError('invalid or duplicate place_id')
            seen.add(pid)
            label = _text(place['label'], 'label')
            if not isinstance(place.get('aliases', []), list):
                raise ValueError('aliases must be a list')
            aliases = [_text(name, 'alias') for name in [label, *place.get('aliases', [])]]
            if any(place.get(key) is not None for key in
                   ('room_id', 'map', 'furniture_pose', 'stop_pose', 'surface_height_m')):
                raise ValueError('init only seeds names with NULL coordinates; register measured stops separately')
            prepared.append((pid, label, aliases))
        # Re-running init adds names/aliases but never erases measured data.
        with self.db:
            for pid, label, aliases in prepared:
                self.db.execute('INSERT INTO places(place_id,label) VALUES (?,?) ON CONFLICT(place_id) DO NOTHING', (pid, label))
                for name in aliases:
                    self.db.execute('INSERT INTO aliases VALUES (?,?,?) ON CONFLICT(place_id,normalized_name) DO NOTHING',
                                    (pid, name, normalize_name(name)))

    def resolve(self, name):
        name = _text(name, 'place name')
        if self.db.execute('SELECT 1 FROM places WHERE place_id=?', (name,)).fetchone():
            return name
        rows = self.db.execute('SELECT place_id FROM aliases WHERE normalized_name=? ORDER BY place_id',
                               (normalize_name(name),)).fetchall()
        if not rows:
            raise ValueError('등록되지 않은 장소: ' + name)
        if len(rows) > 1:
            raise ValueError('장소 이름이 모호합니다: ' + name + ' -> ' + ', '.join(row[0] for row in rows))
        return rows[0][0]

    def get(self, name):
        pid = self.resolve(name)
        row = self.db.execute('SELECT * FROM places WHERE place_id=?', (pid,)).fetchone()
        pose = lambda prefix: None if row[prefix + '_x_m'] is None else {
            'x_m': row[prefix + '_x_m'], 'y_m': row[prefix + '_y_m'], 'yaw_rad': row[prefix + '_yaw_rad']}
        return {'place_id': pid, 'label': row['label'], 'room_id': row['room_id'],
                'aliases': [r[0] for r in self.db.execute('SELECT name FROM aliases WHERE place_id=? ORDER BY normalized_name', (pid,))],
                'map': None if row['map_id'] is None else {key: row[key] for key in ('map_id', 'map_revision', 'frame_id')},
                'furniture_pose': pose('furniture'), 'stop_pose': pose('stop'),
                'surface_height_m': row['surface_height_m'],
                'stop_status': 'UNREGISTERED' if row['stop_x_m'] is None else 'REGISTERED'}

    def list_places(self):
        return [self.get(row[0]) for row in self.db.execute('SELECT place_id FROM places ORDER BY rowid').fetchall()]

    def set_stop(self, name, *, map_id, map_revision, frame_id, x_m, y_m, yaw_rad):
        pid = self.resolve(name)
        binding = tuple(_text(value, key) for value, key in
                        ((map_id, 'map_id'), (map_revision, 'map_revision'), (frame_id, 'frame_id')))
        for value in (x_m, y_m, yaw_rad):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('stop coordinates must be finite numbers, not NULL or bool')
        if abs(x_m) > 1e6 or abs(y_m) > 1e6 or not -math.pi <= yaw_rad <= math.pi:
            raise ValueError('invalid stop coordinates; use meters and yaw radians in [-pi,pi]')
        with self.db:
            old = self.db.execute('SELECT map_id,map_revision,frame_id FROM places WHERE place_id=?', (pid,)).fetchone()
            if tuple(old) != binding:
                self.db.execute('UPDATE places SET furniture_x_m=NULL,furniture_y_m=NULL,furniture_yaw_rad=NULL WHERE place_id=?', (pid,))
            self.db.execute('UPDATE places SET map_id=?,map_revision=?,frame_id=?,stop_x_m=?,stop_y_m=?,stop_yaw_rad=? WHERE place_id=?',
                            (*binding, x_m, y_m, yaw_rad, pid))
        return self.get(pid)

    def require_stop(self, name, *, map_id, map_revision):
        place = self.get(name)
        if place['stop_pose'] is None:
            raise ValueError('정차 좌표 미등록: ' + place['label'])
        if (place['map']['map_id'], place['map']['map_revision']) != (map_id, map_revision):
            raise ValueError('장소 좌표의 지도/리비전이 현재 지도와 다릅니다')
        return {**place['map'], **place['stop_pose']}
