from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from tools.lib.rgbd_recording import validate_recording, write_recording
from tools.lib.run_manifest import sha256


@pytest.fixture
def recording(tmp_path):
    return write_recording(tmp_path / 'recording', np.zeros((6, 8, 3), np.uint8),
                           np.full((6, 8), 450, np.uint16), [[10, 0, 4], [0, 10, 3], [0, 0, 1]], .001,
                           evidence_source='synthetic', recording_id='fixture',
                           workspace=np.ones((6, 8), np.uint8))


def change(path, mutate):
    data = json.loads(path.read_text())
    mutate(data)
    path.write_text(json.dumps(data))


def test_raw_depth_scale_and_missing_extrinsics_remain_explicit(recording):
    report = validate_recording(recording)
    assert report['valid'] and report['evidence_source'] == 'synthetic'
    assert report['frames'][0]['median_valid_depth_m'] == pytest.approx(.45)
    assert not report['frames'][0]['camera_to_base_available']
    assert not report['physical_task_completed']


def test_missing_depth_is_valid_data_but_never_a_fabricated_surface(recording):
    depth = recording.parent / 'depth.npy'
    np.save(depth, np.zeros((6, 8), np.uint16))
    change(recording, lambda d: d['frames'][0]['depth'].update(sha256=sha256(depth)))
    result = validate_recording(recording)['frames'][0]
    assert result['median_valid_depth_m'] is None and result['valid_depth_fraction'] == 0


def test_changed_asset_rejected_before_model_input(recording):
    (recording.parent / 'depth.npy').write_bytes(b'changed')
    with pytest.raises(ValueError, match='checksum'):
        validate_recording(recording)


@pytest.mark.parametrize('relative', ['../outside.npy', '/tmp/outside.npy'])
def test_recording_cannot_reference_external_assets(recording, relative):
    change(recording, lambda d: d['frames'][0]['rgb'].update(path=relative))
    with pytest.raises(ValueError, match='relative|outside'):
        validate_recording(recording)


def test_symlink_cannot_escape_recording(recording, tmp_path):
    target = tmp_path / 'outside.npy'
    np.save(target, np.zeros((6, 8, 3), np.uint8))
    (recording.parent / 'link.npy').symlink_to(target)
    change(recording, lambda d: d['frames'][0]['rgb'].update(path='link.npy', sha256=sha256(target)))
    with pytest.raises(ValueError, match='outside'):
        validate_recording(recording)


@pytest.mark.parametrize('key,value', [('depth_scale_m', 0), ('depth_scale_m', True),
                                      ('depth_scale_m', float('nan')), ('depth_alignment', 'unaligned'),
                                      ('width', 7), ('height', True),
                                      ('intrinsic_matrix', [[0, 0, 4], [0, 10, 3], [0, 0, 1]])])
def test_invalid_geometry_is_rejected(recording, key, value):
    change(recording, lambda d: d['camera'].update({key: value}))
    with pytest.raises(ValueError):
        validate_recording(recording)


def test_rotation_reflection_rejected(recording):
    transform = np.eye(4)
    transform[0, 0] = -1
    change(recording, lambda d: d['frames'][0].update(camera_to_base=transform.tolist()))
    with pytest.raises(ValueError, match='rigid transform'):
        validate_recording(recording)


def test_valid_extrinsics_are_available_without_claiming_robot_execution(recording):
    transform = np.eye(4)
    transform[:3, 3] = [.1, .2, .5]
    change(recording, lambda d: d['frames'][0].update(camera_to_base=transform.tolist()))
    report = validate_recording(recording)
    assert report['frames'][0]['camera_to_base_available'] and not report['physical_task_completed']


@pytest.mark.parametrize('duplicate_id,time', [(True, 1.), (False, 0.), (False, -.1)])
def test_duplicated_or_out_of_order_frames_rejected(recording, duplicate_id, time):
    def mutate(data):
        frame = deepcopy(data['frames'][0])
        frame.update(capture_id='frame-000000' if duplicate_id else 'second', observed_s=time)
        data['frames'].append(frame)
    change(recording, mutate)
    with pytest.raises(ValueError, match='unique|timestamps'):
        validate_recording(recording)


def test_recordings_cannot_be_overwritten(recording):
    with pytest.raises(FileExistsError):
        write_recording(recording.parent, np.zeros((6, 8, 3), np.uint8),
                        np.zeros((6, 8), np.uint16), np.eye(3), .001,
                        evidence_source='synthetic', recording_id='other')
