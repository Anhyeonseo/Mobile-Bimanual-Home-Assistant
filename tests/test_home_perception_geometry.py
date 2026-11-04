from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('cv2')
from home_perception.contracts import ACCEPTED, HELD, INVALID_DEPTH, REJECTED
from home_perception.detect.qwen_vl import parse_detections
from home_perception.eval.robustness import PERTURBATIONS, box_iou, compare, perturb
from home_perception.geometry.mask_cloud import classify
from home_perception.geometry.plane import backproject, fit_plane
from home_perception.io.snapshot import load_snapshot, snapshot_folders

DATA = Path(__file__).resolve().parents[1] / 'data/tabletop_rgbd_20261101'
K = {'fx': 500., 'fy': 500., 'ppx': 100., 'ppy': 100.}


def scene():
    """Flat table at 0.4 m with a 40x40 px box 20 mm above it."""
    depth = np.full((200, 200), .4)
    mask = np.zeros((200, 200), bool)
    mask[80:120, 80:120] = True
    depth[mask] = .38
    return depth, mask


def test_backproject_uses_pinhole_optical_axes():
    depth = np.zeros((200, 200))
    depth[100, 150] = .5
    assert backproject(depth, K, depth > 0)[0] == pytest.approx([.05, 0, .5])


def test_plane_recovers_table_and_faces_camera():
    depth, mask = scene()
    points = backproject(depth, K, depth > 0)
    plane = fit_plane(points)
    assert plane.normal == pytest.approx([0, 0, -1], abs=1e-6)
    assert plane.rms_m == pytest.approx(0, abs=1e-6)
    assert plane.height(np.array([[0, 0, .38]]))[0] == pytest.approx(.02)


@pytest.mark.parametrize('quality,fraction,count,expected', [
    (.9, 1., 500, (ACCEPTED, None, None)),
    (.9, .5, 500, (HELD, INVALID_DEPTH, 'partial_mask_depth')),
    (.9, .1, 500, (REJECTED, INVALID_DEPTH, 'insufficient_mask_depth')),
    (.9, 1., 10, (REJECTED, INVALID_DEPTH, 'insufficient_mask_depth')),
    (.2, 1., 500, (REJECTED, None, 'low_mask_quality')),
])
def test_classification(quality, fraction, count, expected):
    assert classify(quality, fraction, count) == expected


def test_object_centre_and_box_from_mask():
    pytest.importorskip('open3d')
    from home_perception.geometry.mask_cloud import measure
    depth, mask = scene()
    plane = fit_plane(backproject(depth, K, depth > 0))
    result = measure(mask, depth, K, plane, .9)
    assert result.status == ACCEPTED and result.valid_depth_fraction == 1
    assert result.center_xyz_m == pytest.approx([0, 0, .38], abs=.002)
    assert result.support_height_m == pytest.approx(.02)
    assert sorted(result.obb['extent_m'])[1:] == pytest.approx([.03, .03], abs=.004)


def test_mask_without_depth_is_invalid_and_has_no_coordinates():
    pytest.importorskip('open3d')
    from home_perception.geometry.mask_cloud import measure
    depth, mask = scene()
    depth[mask] = 0
    result = measure(mask, depth, K, None, .9)
    assert (result.status, result.failure) == (REJECTED, INVALID_DEPTH)
    assert result.center_xyz_m is None and result.obb is None


def test_depth_gap_that_splits_an_object_is_held_not_accepted():
    pytest.importorskip('open3d')
    from home_perception.geometry.mask_cloud import measure
    depth, mask = scene()
    mask[:] = False
    mask[95:105, 20:180] = True
    depth[mask] = .38
    plane = fit_plane(backproject(depth, K, depth > 0))
    depth[:, 90:130] = 0
    result = measure(mask, depth, K, plane, .9)
    assert (result.status, result.reason) == (HELD, 'fragmented_mask_cloud')
    assert result.failure == INVALID_DEPTH and result.cluster_fraction < .8


def test_workspace_covers_table_and_keeps_tall_object_past_its_edge():
    from home_perception.geometry.workspace import workspace_mask
    depth = np.full((200, 200), 2.)
    depth[60:190, 30:170] = .4
    plane = fit_plane(backproject(depth, K, depth == .4))
    tall =np.zeros((200, 200), bool)
    tall[20:70, 90:110] = True
    result = workspace_mask(depth, K, plane, [tall])
    assert result[120, 100] and result[30, 100]
    assert not result[10, 10]


def test_mask_shape_must_match_depth():
    pytest.importorskip('open3d')
    from home_perception.geometry.mask_cloud import measure
    with pytest.raises(ValueError):
        measure(np.zeros((2, 2), bool), np.zeros((3, 3)), K)


def test_qwen_output_has_no_invented_score(tmp_path):
    path = tmp_path / 'detections.json'
    path.write_text('{"model": "qwen", "revision": "r1", "detections": [{"label": "cup", "bbox_xyxy": [1, 2, 3, 4]}]}')
    detection = parse_detections(path)[0]
    assert detection.box_xyxy == [1., 2., 3., 4.] and detection.score is None and detection.revision == 'r1'


@pytest.mark.parametrize('kind', PERTURBATIONS)
def test_perturbation_keeps_shape_and_changes_image(kind):
    image = np.random.default_rng(0).integers(40, 200, (32, 32, 3), np.uint8)
    result = perturb(image, kind)
    assert result.shape == image.shape and result.dtype == np.uint8 and (result != image).any()


def test_box_comparison():
    assert box_iou([0, 0, 2, 2], [0, 0, 2, 2]) == 1
    assert box_iou([0, 0, 1, 1], [2, 2, 3, 3]) == 0
    assert compare([[0, 0, 2, 2]], []) == {'baseline': 1, 'perturbed': 0, 'matched': 0}


@pytest.mark.skipif(not DATA.exists(), reason='tabletop captures not present')
def test_tabletop_captures_load_and_table_is_flat():
    folders = snapshot_folders(DATA)
    assert len(folders) == 10
    for folder in folders:
        snapshot = load_snapshot(folder)
        depth = snapshot.depth_m()
        plane = fit_plane(backproject(depth, snapshot.intrinsics, depth > 0))
        assert plane.rms_m < .002 and snapshot.objects
