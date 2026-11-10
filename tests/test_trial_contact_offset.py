import copy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from home_perception.geometry.plane import Plane
from home_perception.grasp.handoff import observed_contact_line
from home_perception.grasp.trial_contact_offset import apply_trial_offset


def scene():
    points = np.array([[x, y, .59] for x in np.linspace(-.08, .08, 161)
                       for y in np.linspace(-(.02+.05*x), .02+.05*x, 31)])
    plane = Plane(np.array([0., 0., -1.]), np.array([0., 0., .6]), .001, 1.)
    rotation = np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]])
    candidate = dict(source='graspnet', rotation_matrix=rotation.tolist(),
                     translation_m=[0., 0., .57], depth_m=.02, width_m=.04)
    return dict(refined_grasps=[candidate]), points, plane


def test_trial_remeasures_width_preserves_orientation_and_does_not_mutate_source():
    group, points, plane = scene(); before = copy.deepcopy(group)
    moved = apply_trial_offset(group, points, plane, np.eye(4), .03, (.0024, .0633))
    candidate = moved['refined_grasps'][0]; offset = candidate['trial_contact_offset']
    assert group == before
    assert candidate['rotation_matrix'] == group['refined_grasps'][0]['rotation_matrix']
    assert candidate['translation_m'][2] == pytest.approx(.57)
    assert offset['base_axis'][0] == pytest.approx(1.)
    assert offset['requested_m'] == .03 and not offset['calibration_changed']
    assert candidate['closing_section'] == observed_contact_line(candidate, points)
    assert candidate['width_m'] > .042
    assert candidate['balance_proxy']['transverse_offset_m'] == pytest.approx(.03)


def test_direction_is_defined_in_base_even_when_camera_frame_changes():
    group, points, plane = scene()
    expected = apply_trial_offset(group, points, plane, np.eye(4), .03, (.0024, .0633))
    r = Rotation.from_euler('xyz', [35, -50, 80], degrees=True).as_matrix()
    t = np.array([.1, -.2, .6]); candidate = group['refined_grasps'][0]
    candidate['rotation_matrix'] = (r @ candidate['rotation_matrix']).tolist()
    candidate['translation_m'] = (r @ candidate['translation_m'] + t).tolist()
    base = np.eye(4); base[:3, :3] = r.T; base[:3, 3] = -r.T @ t
    moved = apply_trial_offset(group, points @ r.T + t,
                              Plane(r @ plane.normal, r @ plane.point + t, .001, 1.),
                              base, .03, (.0024, .0633))
    actual = moved['refined_grasps'][0]
    np.testing.assert_allclose(r.T @ (np.array(actual['translation_m']) - t),
                               expected['refined_grasps'][0]['translation_m'], atol=1e-10)


def test_missing_section_is_not_invented_and_zero_offset_is_unchanged():
    group, points, plane = scene()
    assert apply_trial_offset(group, points, plane, np.eye(4), 0., (.0024, .0633)) == group
    points = points[abs(points[:, 0]) > .05]
    moved = apply_trial_offset(group, points, plane, np.eye(4), .03, (.0024, .0633))
    assert not moved['refined_grasps']
    assert moved['trial_contact_offset']['rejected_by_evidence']['shifted_target_section_unresolved'] == 1


@pytest.mark.parametrize('distance', [-.01, .031, float('nan')])
def test_trial_budget_is_explicit_and_bounded(distance):
    group, points, plane = scene()
    with pytest.raises(ValueError):
        apply_trial_offset(group, points, plane, np.eye(4), distance, (.0024, .0633))
