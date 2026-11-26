import numpy as np
import pytest

from home_perception.geometry.plane import Plane, backproject
from home_perception.grasp.so101 import candidates


def scene(width=.03):
    x, y = np.meshgrid(np.linspace(-width/2, width/2, 35), np.linspace(-.06, .06, 90))
    points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .58)))
    return points, Plane(np.array([0., 0., -1.]), np.array([0., 0., .6]), .001, 1.)


def test_fixed_contacts_both_sides_and_tilts():
    points, plane = scene()
    group = candidates(points, plane, (.0024, .0633))
    assert len(group['refined_grasps']) == 24
    assert group['counts']['generated'] <= 84
    assert {-10, 0, 10}.issubset(c['yaw_deg'] for c in group['refined_grasps'])
    assert {c['jaw_sign'] for c in group['refined_grasps']} == {-1, 1}
    assert {c['tilt_deg'] for c in group['refined_grasps']} == {-25, 0, 25}
    for c in group['refined_grasps']:
        r = np.array(c['rotation_matrix'])
        assert np.allclose(r.T @ r, np.eye(3)) and np.isclose(np.linalg.det(r), 1)
        fixed, moving = np.array(c['fixed_contact_optical_m']), np.array(c['moving_contact_optical_m'])
        assert np.allclose(moving-fixed, c['width_m']*r[:, 1])
        assert c['entry_gap_m'] <= .0633
        assert not c['robot_feasibility_checked'] and not c['motion_authorized']


def test_too_wide_and_empty_have_no_candidates():
    points, plane = scene(.1)
    assert not candidates(points, plane, (.0024, .0633))['refined_grasps']
    assert not candidates(np.empty((0, 3)), plane, (.0024, .0633))['refined_grasps']


def test_partial_depth_image_and_missing_depth():
    _, plane = scene()
    depth = np.full((120, 120), .6)
    mask = np.zeros_like(depth, dtype=bool)
    mask[20:100, 50:70] = True
    depth[mask] = .58
    depth[50:55, 50:55] = 0
    intr = dict(fx=400., fy=400., ppx=60., ppy=60.)
    points = backproject(depth, intr, mask & (depth > 0))
    assert candidates(points, plane, (.0024, .0633))['refined_grasps']


@pytest.mark.parametrize('invalid', [np.array([[np.nan, 0., .5]]), np.zeros((3, 2))])
def test_invalid_points_rejected(invalid):
    _, plane = scene()
    with pytest.raises(ValueError):
        candidates(invalid, plane, (.0024, .0633))


def test_entry_clearance_rejects_apparent_width_fit():
    points, plane = scene(.03)
    assert not candidates(points, plane, (.0024, .031))['refined_grasps']
