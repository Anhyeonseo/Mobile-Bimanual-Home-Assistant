import json

import numpy as np
import pytest

from home_perception.adapters import observation_from_grounding
from home_perception.contracts import (ACCEPTED, HELD, INVALID_DEPTH, ObjectObservation, REJECTED,
                                       ScanSession, TaskRequest, WorldEntity)

OBSERVATION = dict(capture_id='05', captured_at_utc='2026-11-01T11:12:44+00:00',
                   frame_id='color_camera_optical_frame', label='remote',
                   box_xyxy=[1., 2., 3., 4.], detector='qwen')


@pytest.mark.parametrize('item', [
    TaskRequest('r1', 'u1', '리모컨 가져다줘', action='fetch', target='리모컨', unresolved=['destination']),
    ObjectObservation(**OBSERVATION, status=ACCEPTED, center_xyz_m=[0., 0., .4], valid_depth_fraction=1.),
    WorldEntity('e1', 'remote', 'stale', 'observed', 'map', evidence=['capture-1']),
    ScanSession('s1', 'd415', 'unmeasured', 'running', coverage_fraction=.5),
])
def test_contract_json_round_trip(item):
    assert type(item).from_json(item.to_json()) == item


@pytest.mark.parametrize('build', [
    lambda: TaskRequest('r', 'u', 'text'),
    lambda: ObjectObservation(**OBSERVATION, status='maybe'),
    lambda: ObjectObservation(**OBSERVATION, status=HELD),
    lambda: ObjectObservation(**OBSERVATION, status=ACCEPTED, failure=INVALID_DEPTH),
    lambda: ObjectObservation(**OBSERVATION, status=REJECTED, reason='x', center_xyz_m=[0., 0., 1.]),
    lambda: ObjectObservation(**OBSERVATION, status=ACCEPTED, center_xyz_m=[float('nan'), 0., 1.]),
    lambda: WorldEntity('e', 'cup', 'observed', 'observed', 'map'),
    lambda: ScanSession('s', 'd', 'c', 'done'),
])
def test_contract_rejects_invalid_values(build):
    with pytest.raises(ValueError):
        build()


def test_contract_rejects_unknown_fields():
    with pytest.raises(ValueError, match='unknown'):
        TaskRequest.from_dict({'request_id': 'r', 'user_id': 'u', 'text': 't', 'action': 'a', 'joints': [0]})


def report(point):
    return {'source_snapshot': '/x/05_remote_only', 'captured_at_utc': '2026-11-01T11:12:44+00:00',
            'coordinate_frame': 'color_camera_optical_frame', 'model_id': 'dino', 'robot_transform': None,
            'point_semantics': 'surface pixel; not object centroid'}, {
            'label': 'remote', 'box_xyxy': [1, 2, 3, 4], 'score': .4, 'surface_point': point}


def test_grounding_adapter_keeps_surface_point_semantics():
    item = observation_from_grounding(*report({'valid': True, 'xyz_m': [0, 0, .4], 'valid_fraction': 1.}))
    assert item.status == ACCEPTED and item.surface_xyz_m == [0., 0., .4]
    assert item.center_xyz_m is None and item.robot_transform is None
    assert 'not object centroid' in item.point_semantics


def test_grounding_adapter_maps_missing_depth_to_failure():
    item = observation_from_grounding(*report({'valid': False, 'xyz_m': None, 'reason': 'insufficient_depth'}))
    assert item.status == REJECTED and item.failure == INVALID_DEPTH and item.surface_xyz_m is None


@pytest.fixture
def snapshot(tmp_path):
    cv2 = pytest.importorskip('cv2')
    cv2.imwrite(str(tmp_path / 'color.png'), np.zeros((4, 6, 3), np.uint8))
    cv2.imwrite(str(tmp_path / 'depth_aligned.png'), np.full((4, 6), 400, np.uint16))
    metadata = {'coordinate_frame': 'color_camera_optical_frame', 'alignment': 'depth to color via rs.align',
                'captured_at_utc': '2026-11-01T00:00:00+00:00', 'depth_scale_m': .001,
                'aligned_depth_intrinsics': {'width': 6, 'height': 4, 'fx': 10., 'fy': 10., 'ppx': 3., 'ppy': 2.,
                                             'coeffs': [0.] * 5}}
    (tmp_path / 'metadata.json').write_text(json.dumps(metadata))
    return tmp_path, metadata


def test_loader_converts_depth_and_finds_folders(snapshot):
    from home_perception.io.snapshot import load_snapshot, snapshot_folders
    folder, _ = snapshot
    loaded = load_snapshot(folder)
    assert loaded.depth_raw.dtype == np.uint16
    assert loaded.depth_m()[0, 0] == pytest.approx(.4)
    assert len(loaded.source_sha256) == 3
    assert folder in snapshot_folders(folder.parent)


@pytest.mark.parametrize('change,match', [
    (dict(coordinate_frame='base_link'), 'aligned'),
    (dict(alignment='color to depth'), 'aligned'),
    (dict(depth_scale_m=1.), 'depth scale'),
])
def test_loader_rejects_wrong_metadata(snapshot, change, match):
    from home_perception.io.snapshot import load_snapshot
    folder, metadata = snapshot
    (folder / 'metadata.json').write_text(json.dumps(metadata | change))
    with pytest.raises(ValueError, match=match):
        load_snapshot(folder)


def test_loader_rejects_eight_bit_depth(snapshot):
    import cv2
    from home_perception.io.snapshot import load_snapshot
    folder, _ = snapshot
    cv2.imwrite(str(folder / 'depth_aligned.png'), np.full((4, 6), 40, np.uint8))
    with pytest.raises(ValueError, match='uint16'):
        load_snapshot(folder)
