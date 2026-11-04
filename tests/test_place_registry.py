from copy import deepcopy
import json
from pathlib import Path

import pytest

from home_robot_tasks.place_registry import PlaceRegistry
from tools.run.places_db import main

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def seed():
    return json.loads((ROOT / 'config/home_places.json').read_text())


@pytest.fixture
def registry(tmp_path, seed):
    with PlaceRegistry(tmp_path / 'home.sqlite3', create=True) as store:
        store.seed(seed)
        yield store


def test_six_requested_places_have_unknown_coordinates_not_zero_defaults(registry):
    places = registry.list_places()
    assert [p['label'] for p in places] == ['큰 책상', '탁상', '미니 테이블', '소파', '싱크대', '쓰레기통']
    for place in places:
        assert place['map'] is None and place['furniture_pose'] is None and place['stop_pose'] is None
        assert place['surface_height_m'] is None and place['stop_status'] == 'UNREGISTERED'
    columns = ('furniture_x_m', 'furniture_y_m', 'furniture_yaw_rad', 'stop_x_m', 'stop_y_m', 'stop_yaw_rad')
    for row in registry.db.execute('SELECT * FROM places'):
        assert all(row[column] is None for column in columns)


@pytest.mark.parametrize('name,pid', [('큰 책상', 'desk'), ('큰책상', 'desk'), ('책상', 'desk'),
                                    ('미니테이블', 'mini_table'), ('미니 테이블', 'mini_table'),
                                    ('쇼파', 'sofa'), ('휴지통', 'trash_bin')])
def test_korean_names_and_aliases_resolve_to_registered_ids(registry, name, pid):
    assert registry.resolve(name) == pid


def test_unknown_stop_cannot_be_used_as_a_navigation_goal(registry):
    with pytest.raises(ValueError, match='정차 좌표 미등록'):
        registry.require_stop('책상', map_id='home', map_revision='1')


def register_stop(registry, **changes):
    return registry.set_stop('책상', **dict(map_id='home', map_revision='1', frame_id='map',
                                           x_m=0., y_m=0., yaw_rad=0., **changes))


def test_explicit_measured_zero_pose_is_distinct_from_unknown_furniture_position(registry):
    register_stop(registry)
    place = registry.get('desk')
    assert place['stop_status'] == 'REGISTERED' and place['furniture_pose'] is None
    assert registry.require_stop('책상', map_id='home', map_revision='1') == {
        'map_id': 'home', 'map_revision': '1', 'frame_id': 'map', 'x_m': 0., 'y_m': 0., 'yaw_rad': 0.}
    with pytest.raises(ValueError, match='지도/리비전'):
        registry.require_stop('desk', map_id='home', map_revision='2')


def test_reinitialization_never_erases_registered_coordinates_or_duplicates_rows(registry, seed):
    register_stop(registry)
    before = registry.get('책상')
    aliases = registry.db.execute('SELECT COUNT(*) FROM aliases').fetchone()[0]
    registry.seed(seed)
    registry.seed(seed)
    assert registry.get('책상') == before
    assert len(registry.list_places()) == 6
    assert registry.db.execute('SELECT COUNT(*) FROM aliases').fetchone()[0] == aliases


def test_generic_ambiguous_name_is_not_guessed(registry, seed):
    data = deepcopy(seed)
    data['places'][1]['aliases'].append('테이블')
    data['places'][2]['aliases'].append('테이블')
    registry.seed(data)
    with pytest.raises(ValueError, match='모호'):
        registry.resolve('테이블')
    with pytest.raises(ValueError, match='등록되지 않은'):
        registry.resolve('침대')


@pytest.mark.parametrize('field,value', [('x_m', None), ('y_m', float('nan')),
                                       ('x_m', float('inf')), ('yaw_rad', True), ('yaw_rad', 4.)])
def test_invalid_stop_is_rejected_without_partial_registration(registry, field, value):
    arguments = dict(map_id='home', map_revision='1', frame_id='map', x_m=1., y_m=2., yaw_rad=0.)
    arguments[field] = value
    with pytest.raises(ValueError):
        registry.set_stop('desk', **arguments)
    assert registry.get('desk')['map'] is None and registry.get('desk')['stop_pose'] is None


def test_changing_map_invalidates_old_furniture_coordinates(registry):
    register_stop(registry)
    with registry.db:
        registry.db.execute("UPDATE places SET furniture_x_m=1,furniture_y_m=2,furniture_yaw_rad=0 WHERE place_id='desk'")
    registry.set_stop('desk', map_id='home', map_revision='2', frame_id='map', x_m=3., y_m=4., yaw_rad=0.)
    assert registry.get('desk')['furniture_pose'] is None


def test_invalid_seed_is_transactional_and_does_not_replace_existing_db(registry, seed):
    data = deepcopy(seed)
    data['places'].append({'place_id': 'new_place', 'label': '추가 장소', 'aliases': []})
    data['places'].append({'place_id': 'invalid', 'label': '잘못된 입력', 'stop_pose': {'x_m': 0}})
    with pytest.raises(ValueError):
        registry.seed(data)
    assert len(registry.list_places()) == 6


def test_cli_creates_persistent_db_and_lookup_returns_null_coordinates(tmp_path, capsys):
    database = tmp_path / 'home.sqlite3'
    assert main(['--database', str(database), 'init']) == 0
    assert len(json.loads(capsys.readouterr().out)['places']) == 6
    assert main(['--database', str(database), 'show', '책상']) == 0
    assert json.loads(capsys.readouterr().out)['stop_pose'] is None
