"""Coordinate direction and provenance checks; never physical acceptance."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from home_robot_tasks.s4_sensor_frames import load_sensor_frames, sensor_frames
from home_robot_tasks.ros_rgbd_source import quaternion_transform
from home_robot_tasks.s2_rgbd_acquisition import Capture

ROOT = Path(__file__).resolve().parents[1]


def test_floor_lidar_and_s3_camera_agree_without_second_height_adjustment():
    result = load_sensor_frames(ROOT)
    frames = {t['child']: np.array(t['matrix']) for t in result['transforms']}
    floor_from_lidar = np.linalg.inv(frames['base_footprint']) @ frames['lidar_a']
    assert floor_from_lidar[2, 3] == pytest.approx(.16140244318550648)
    # The sensor's zero-angle ray points toward the rear of the base at yaw pi.
    point = frames['lidar_a'] @ [1, 0, 0, 1]
    assert point[:3] == pytest.approx([-.84900059351632, -.0017101033546, .0912024431855])
    profile = json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())
    np.testing.assert_allclose(frames['color_camera_optical_frame'], profile['base_from_optical'], atol=1e-12)
    assert not result['motion_authorized']
    assert not result['precision_extrinsics_qualified']
    assert all(t['parent'] not in ('map', 'odom') for t in result['transforms'])
    for t in result['transforms']:
        actual = quaternion_transform(SimpleNamespace(**dict(zip('xyz', t['translation_m']))),
                                      SimpleNamespace(**dict(zip('xyzw', t['quaternion_xyzw']))))
        np.testing.assert_allclose(actual, t['matrix'], atol=1e-12)


def test_sdk_column_major_depth_to_color_is_not_inverted():
    extr = dict(rotation_column_major=[0, 1, 0, -1, 0, 0, 0, 0, 1], translation_m=[.02, -.01, 0])
    result = load_sensor_frames(ROOT, depth_to_color=extr)
    frame = result['transforms'][-1]
    assert frame['parent'] == 'color_camera_optical_frame'
    assert np.array(frame['matrix']) @ [1, 0, 2, 1] == pytest.approx([.02, .99, 2, 1])
    with pytest.raises(ValueError):
        load_sensor_frames(ROOT, depth_to_color={**extr, 'rotation_column_major': [1]*9})


def test_sdk_float32_rotation_survives_quaternion_conversion():
    reference = json.loads((ROOT/'config/home_assistant/camera_device.observed.json').read_text())
    matrix = np.array(reference['sdk_calibration']['color_optical_from_depth_optical'])
    frame = load_sensor_frames(ROOT, depth_to_color=dict(
        rotation_column_major=matrix[:3, :3].flatten(order='F').tolist(),
        translation_m=matrix[:3, 3].tolist()))['transforms'][-1]
    actual = quaternion_transform(SimpleNamespace(**dict(zip('xyz', frame['translation_m']))),
                                  SimpleNamespace(**dict(zip('xyzw', frame['quaternion_xyzw']))))
    np.testing.assert_allclose(actual, matrix, atol=1e-7, rtol=0)


@pytest.mark.parametrize('change', [
    {'mount_confirmed': False}, {'accuracy_qualified': True},
    {'setup_method': {'physical_comparison_performed': False}},
    {'scan_height_above_floor_m': .0912024431855},
    {'base_from_lidar_translation_m': [float('nan'), 0, .09]},
])
def test_invalid_or_unreviewed_mount_rejected(change):
    mount = json.loads((ROOT/'config/home_assistant/lidar_mount.review.json').read_text())
    mount.update(deepcopy(change))
    with pytest.raises(ValueError):
        sensor_frames(ROOT, mount)


def test_native_retention_requires_image_capture():
    with pytest.raises(ValueError, match='retention'):
        Capture({'camera_device': {}}, retain_native_depth=True)


def test_native_capture_keeps_full_frame_and_bounded_copies(monkeypatch):
    from unittest.mock import MagicMock
    from home_robot_tasks import s2_rgbd_acquisition as acquisition
    camera = Capture({'camera_device': {}}, retain_frame=True, retain_native_depth=True)
    camera.rs = MagicMock()
    camera.device = {}
    camera.active = MagicMock()
    camera.clock = SimpleNamespace(convert=lambda t, domain: t/1000)
    camera.align = MagicMock()
    camera._intr = lambda intr: {}
    monkeypatch.setattr(acquisition, 'from_metadata', lambda m: {'camera_device': {}})
    color, native, aligned = MagicMock(), MagicMock(), MagicMock()
    native_array = np.ones((8, 12), dtype=np.uint16)
    native.get_data.return_value = native_array
    aligned.get_data.return_value = np.ones((4, 6), dtype=np.uint16)*2
    color.get_data.return_value = np.zeros((4, 6, 3), dtype=np.uint8)
    camera.align.process.return_value.get_depth_frame.return_value = aligned
    frames = SimpleNamespace(get_depth_frame=lambda: native, get_color_frame=lambda: color)
    for n in range(5):
        for frame in (native, color):
            frame.get_frame_number.return_value = n
            frame.get_timestamp.return_value = n*33
        record = camera._record(frames)
    assert set(camera.native_depth_images) == {'rgbd-2', 'rgbd-3', 'rgbd-4'}
    assert camera.native_depth_images['rgbd-4'].shape == (8, 12)
    native_array[:] = 99
    assert camera.native_depth_images['rgbd-4'][0, 0] == 1
    old_api_tuple = camera.retain_matched(record)
    assert len(old_api_tuple) == 3 and old_api_tuple[1].shape == (4, 6)
    assert not camera.native_depth_images and not camera.images
