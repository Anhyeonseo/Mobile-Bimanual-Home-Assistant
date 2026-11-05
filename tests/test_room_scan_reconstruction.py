"""Guard against successful-looking maps from invalid depth, poses, or recorder errors."""
import json
import numpy as np
import pytest
from tools.diagnostics import record_room_rgbd as recording
from tools.diagnostics.room_scan_quality import Readiness, image_quality, summarize_quality
from tools.diagnostics.reconstruct_room_rgbd import scaled_intrinsics, valid_transform


def test_covered_lens_is_not_ready():
    pytest.importorskip("cv2")
    q = image_quality(np.full((360, 640, 3), 100, np.uint8), np.zeros((360, 640), np.uint16))
    assert not q['ready']
    assert q['useful_depth_fraction'] == 0
    assert len(q['warnings']) >= 2


def test_too_close_depth_is_not_good_room_geometry():
    pytest.importorskip("cv2")
    q = image_quality(np.zeros((180, 320, 3), np.uint8), np.full((180, 320), 300, np.uint16))
    assert q['valid_depth_fraction'] == 1
    assert q['useful_depth_fraction'] == 0
    assert not q['ready']


def test_depth_units_are_respected():
    pytest.importorskip("cv2")
    color = np.zeros((180, 320, 3), np.uint8)
    a = image_quality(color, np.full((180, 320), 1000, np.uint16), .001)
    b = image_quality(color, np.full((180, 320), 10000, np.uint16), .0001)
    assert a['useful_depth_fraction'] == b['useful_depth_fraction'] == 1


def test_readiness_needs_continuous_recent_good_frames():
    gate = Readiness()
    for t in (0, .5, 1, 1.5):
        assert not gate.update(t, True)
    assert gate.update(2, True)
    assert not gate.ready(4)
    assert not gate.update(4, True)
    assert not gate.update(4.5, False)
    assert not gate.update(5, True)


def test_image_hints_never_certify_global_map():
    result = summarize_quality([{'ready': True}] * 10)
    assert result['status'] == 'image_hints_ok'
    assert not result['tracking_verified'] and not result['coverage_verified']
    assert summarize_quality([])['status'] == 'needs_review'


def test_resize_uses_pixel_center_convention():
    intr = scaled_intrinsics(dict(width=1280, height=720, fx=1000, fy=1000, cx=639.5, cy=359.5), 640, 360)
    assert intr['fx'] == 500
    assert intr['cx'] == 319.5
    assert intr['cy'] == 179.5


def test_pose_rejects_reflection_nan_and_large_jump():
    assert valid_transform(np.eye(4), .1)
    for T in (np.diag([-1., 1, 1, 1]), np.eye(4) * np.nan):
        assert not valid_transform(T, .1)
    jump = np.eye(4)
    jump[0, 3] = 2
    assert not valid_transform(jump, .1)


def test_verification_exception_is_persisted(monkeypatch, tmp_path):
    def fail(*_):
        raise RuntimeError('bad bag')
    monkeypatch.setattr(recording, 'verify', fail)
    with pytest.raises(RuntimeError, match='bad bag'):
        recording.verify_and_save(tmp_path / 'recording.bag', None)
    result = json.loads((tmp_path / 'verification.json').read_text())
    assert result['status'] == 'error'
    assert result['error'] == 'bad bag'


def test_synthetic_translation_and_missing_depth_produce_separate_fragments(tmp_path):
    """Optional CPU integration test; no SDK, device, network or private bag needed."""
    cv2 = pytest.importorskip('cv2')
    pytest.importorskip('open3d')
    from tools.diagnostics.reconstruct_room_rgbd import reconstruct
    dataset = tmp_path / 'dataset'
    (dataset / 'color').mkdir(parents=True)
    (dataset / 'depth').mkdir()
    rng = np.random.default_rng(42)
    texture = cv2.GaussianBlur(rng.integers(0, 256, (360, 640, 3), dtype=np.uint8), (3, 3), 0)
    rows = []
    for i in range(8):
        color = cv2.warpAffine(texture, np.float32([[1, 0, -i * 5], [0, 1, 0]]), (640, 360))
        depth = np.full((360, 640), 1000 if i != 4 else 0, np.uint16)
        c, d = f'color/{i:06}.png', f'depth/{i:06}.png'
        cv2.imwrite(str(dataset / c), color)
        cv2.imwrite(str(dataset / d), depth)
        rows.append(dict(seconds=i * .1, color=c, depth=d, quality=image_quality(color, depth)))
    (dataset / 'dataset.json').write_text(json.dumps(dict(
        intrinsics=dict(width=640, height=360, fx=500, fy=500, cx=319.5, cy=179.5),
        duration_seconds=.8, frames=rows)))
    report = reconstruct(dataset, tmp_path / 'reconstruction')
    assert len(report['segments']) == 2
    assert not report['global_map_valid'] and not report['robot_navigation_valid']
    positions = [t['camera_to_fragment'][0][3] for t in report['trajectory'] if t['fragment'] == 0]
    assert abs(positions[-1] - .03) < .006
    second_origin = next(t['camera_to_fragment'] for t in report['trajectory'] if t['fragment'] == 1)
    assert np.allclose(second_origin, np.eye(4))
