import numpy as np
from home_robot_tasks.carry_binding import BoundPayload
from home_robot_tasks.carry_scene import SceneBox
from home_robot_tasks.tabletop_alignment import preferred_offset, scene_sweep, translated_observation
from home_robot_tasks.carry_program import compile_program
from tools.diagnostics import review_tabletop_pick
import pytest


def test_offsets_preserve_world_geometry_and_camera_axes():
    optical = np.eye(4)
    optical[:2, :2] = [[0, -1], [1, 0]]
    optical[:3, 3] = [.2, .1, .4]
    original = np.eye(4)
    original[:3, 3] = [.5, .3, .8]
    payload = BoundPayload('cap', {}, 'cal', original, (.04, .02, .01))
    box = SceneBox('support', original, (1., 1., .02), 'support')
    camera_point = (np.linalg.inv(optical) @ original)[:3, 3]
    review, group, boxes, target = translated_observation(
        [0, .1], {'base_from_optical': optical},
        {'base_points_m': {'center_xyz_m': original[:3, 3]}},
        {'refined_grasps': [{'translation_m': camera_point}]}, [box], payload)
    expected = [.5, .2, .8]
    np.testing.assert_allclose(optical[:3, :3] @ group['refined_grasps'][0]['translation_m'] +
                               optical[:3, 3], expected)
    np.testing.assert_allclose(review['base_points_m']['center_xyz_m'], expected)
    np.testing.assert_allclose(target.base_from_object[:3, 3], expected)
    sweep = list(scene_sweep(boxes, [0, .1]))
    np.testing.assert_allclose(sweep[0][1][0].base_pose, original)
    np.testing.assert_allclose(sweep[-1][1][0].base_pose[:3, 3], expected)
    assert len(sweep) == 11
    np.testing.assert_allclose(original[:3, 3], [.5, .3, .8])


def test_reachable_intervals_do_not_cross_failed_positions():
    assert preferred_offset({0: True, 50: True}) == 0
    assert preferred_offset({-50: True, 0: False, 50: True, 100: True, 150: True}) == 100
    assert preferred_offset({0: False, 50: False}) is None


def test_unreachable_origin_then_reachable_left_uses_one_resident(tmp_path, monkeypatch):
    import json
    prepared = tmp_path/'prepared.json'
    prepared.write_text(json.dumps({'review': {'base_points_m': {'center_xyz_m': [.4, .1, .7]}}}))
    calls = []
    def simulated(options, *, session_factory, preview_base_xy):
        calls.append((session_factory, preview_base_xy))
        status = ('HYPOTHETICAL_ROUTE_FOUND' if preview_base_xy and preview_base_xy[1] > 0
                  else 'NO_COMPLETE_ROUTE')
        return dict(status=status, capture_id='test', recovery={'base_reposition_candidate': True})
    monkeypatch.setattr(review_tabletop_pick, 'plan', simulated)
    summary = review_tabletop_pick.review(prepared, tmp_path, tmp_path/'review', [-50, 50], 90)
    assert summary['preferred_tested_offset_mm'] == 50
    assert not summary['motion_authorized'] and summary['hardware_commands'] == 0
    assert calls[0][1] is None and len(set(id(c[0]) for c in calls)) == 1


def test_hypothetical_result_cannot_be_compiled():
    with pytest.raises(ValueError, match='successful bound'):
        compile_program({'status': 'HYPOTHETICAL_ROUTE_FOUND'}, {}, None, None, bounds=None)


def test_only_observed_hand_obstacle_collisions_on_approach_qualify():
    from home_robot_tasks.pick_recovery import observed_approach_collision
    external = ('capture_obstacle', 'left_s2_gripper_link')
    self_contact = ('left_s2_gripper_link', 'left_s2_shoulder_link')
    def eligible(stage, contacts):
        return observed_approach_collision(dict(stage=stage, reason=f'invalid full robot state: {contacts!r}'))
    assert eligible('approach', [external])
    assert not eligible('approach', [external, self_contact])
    assert not eligible('lift', [external])


@pytest.mark.parametrize('yaw', [0., np.pi/2])
def test_tilted_jaw_exiting_top_cannot_pinch_both_sidewalls(yaw):
    from itertools import product
    from types import SimpleNamespace
    from scipy.spatial.transform import Rotation
    from home_robot_tasks.so101_jaw import StockJaw
    yaw_rot = Rotation.from_euler('z', yaw).as_matrix()
    box = np.array(list(product([-.014, .014], [-.025, .025], [0., .012]))) @ yaw_rot.T
    def section(pitch):
        pose = np.eye(4)
        pose[:3, :3] = yaw_rot @ Rotation.from_euler('y', pitch, degrees=True).as_matrix()
        pose[:3, 3] = yaw_rot @ [.014, 0., .002]
        return StockJaw.contact_section(SimpleNamespace(fixed_face_m=0.), pose, box,
                                        support_normal=[0., 0., 1.])
    straight, tilted = section(0), section(30)
    assert straight['fixed_on_sidewall'] and straight['moving_on_sidewall']
    assert tilted['fixed_on_sidewall'] and not tilted['moving_on_sidewall']


def test_measured_pinch_comparison_is_a_preference():
    from home_robot_tasks.carry_planning import compare_lateral_pinch, better_lateral_pinch
    margin = dict(entry_pad_gaps_m=dict(fixed=.0044, moving=.0023))
    assert not compare_lateral_pinch(margin, dict(short_axis_angle_deg=19.1))
    assert compare_lateral_pinch(margin, dict(short_axis_angle_deg=30.6))
    assert compare_lateral_pinch(dict(entry_pad_gaps_m=dict(fixed=.001, moving=.003)), dict(short_axis_angle_deg=5))
    assert not compare_lateral_pinch(margin, None)
    assert better_lateral_pinch((9, False, 2, 1), (9, True, 2, 1))
    assert not better_lateral_pinch((9, False, 2, 1), (9, True, 1, 1))
    assert not better_lateral_pinch((9, False, 2, 1), (8, True, 3, 1))
    assert not better_lateral_pinch((9, True, 1, 0), (9, True, 1, 0))


@pytest.mark.parametrize('improved,expected', [(True, 50), (False, 0)])
def test_checked_stationary_diagonal_compared_to_checked_strafe(tmp_path, monkeypatch, improved, expected):
    import json
    prepared = tmp_path/'prepared.json'
    prepared.write_text(json.dumps({'review': {'base_points_m': {'center_xyz_m': [.4,.1,.7]}}}))
    def simulated(options, *, session_factory, preview_base_xy):
        shifted = preview_base_xy is not None
        straight = shifted and improved
        quality = dict(tabletop_pinch_alignment=dict(preferred=straight, short_axis_angle_deg=4 if straight else 31),
                       tabletop_contact_margin=dict(passed=9, minimum_fixed_clearance_m=.004,
                           minimum_moving_clearance_m=.002, entry_pad_gaps_m=dict(fixed=.004, moving=.003)))
        return dict(status='HYPOTHETICAL_ROUTE_FOUND' if shifted else 'OFFLINE_ROUTE_FOUND',
                    capture_id='example', observed_sweep_clear=shifted, within_camera_view=True,
                    plan=dict(carry_quality=quality), recovery=dict(base_reposition_candidate=False))
    monkeypatch.setattr(review_tabletop_pick, 'plan', simulated)
    result = review_tabletop_pick.review(prepared, tmp_path, tmp_path/'review', [50], 90)
    assert len(result['samples']) == 2
    assert result['preferred_tested_offset_mm'] == expected
