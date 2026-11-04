from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tools.lib.grasp_ik import (BoundedPoseIK, IKPolicy, evaluate_candidates,
                                load_settings, target_from_grasp)
from tools.lib.grasp_preview import VisualRobot, joint_positions

ROOT = Path(__file__).resolve().parents[1]
PLANAR = '''<robot name="planar"><link name="root"/><link name="tool"/>
<joint name="q" type="revolute"><parent link="root"/><child link="tool"/>
<axis xyz="0 0 1"/><limit lower="-0.5" upper="0.5"/></joint>
</robot>'''


def arm_model(tmp_path):
    names = ['base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll']
    axes = ['0 0 1', '0 1 0', '0 1 0', '0 1 0', '1 0 0']
    origins = ['0 0 0.1', '0 0 0', '0.15 0 0', '0.12 0 0', '0.08 0 0']
    elements = ['<robot name="five_dof"><link name="root"/>']
    parent = 'root'
    for name, axis, xyz in zip(names, axes, origins):
        link = 'left_' + name + '_link'
        elements.append(f'<link name="{link}"/><joint name="left_{name}_joint" type="revolute">'
                        f'<parent link="{parent}"/><child link="{link}"/><origin xyz="{xyz}"/>'
                        f'<axis xyz="{axis}"/><limit lower="-1.5" upper="1.5"/></joint>')
        parent = link
    elements.append(f'<link name="left_gripper_frame_link"/><joint name="tcp" type="fixed">'
                    f'<parent link="{parent}"/><child link="left_gripper_frame_link"/><origin xyz="0.05 0 0"/></joint>')
    elements.append(f'<link name="jaw"/><joint name="left_gripper_joint" type="revolute">'
                    f'<parent link="{parent}"/><child link="jaw"/><axis xyz="0 0 1"/>'
                    '<limit lower="0" upper="1"/></joint></robot>')
    return VisualRobot(''.join(elements), tmp_path)


@pytest.fixture
def arm(tmp_path):
    model = arm_model(tmp_path)
    layout = {'arm': 'left', 'camera_translation_m': [0, 0, 0], 'camera_rpy_degrees': [0, 0, 0],
              'joint_positions_degrees': {name: 0. for name in
                                         ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll', 'gripper')}}
    settings = load_settings(ROOT / 'config/grasp_ik.virtual.json')
    settings['policy'].update(maximum_starts=4, maximum_function_evaluations=80)
    return model, layout, settings


def known_target(model, layout):
    positions = joint_positions(layout)
    values = [.2, -.4, .6, -.5, .3]
    names = [f'left_{n}_joint' for n in ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll')]
    positions.update(zip(names, values))
    return model.link_transforms(positions)['left_gripper_frame_link'], names


def grasp_for_target(target, settings, score=.5):
    convention = np.eye(4)
    convention[:3, :3] = settings['grasp_from_tcp_rotation']
    convention[:3, 3] = settings['grasp_to_tcp_translation_m']
    grasp = target @ np.linalg.inv(convention)
    return {'translation_m': grasp[:3, 3].tolist(), 'rotation_matrix': grasp[:3, :3].tolist(),
            'width_m': .04, 'depth_m': .02, 'score': score}


def test_known_five_joint_pose_is_recovered_and_verified_by_fk(arm):
    model, layout, _ = arm
    target, names = known_target(model, layout)
    policy = IKPolicy(position_tolerance_m=.0001, orientation_tolerance_degrees=.1,
                      maximum_starts=4, maximum_function_evaluations=100)
    anchor = joint_positions(layout)
    result = BoundedPoseIK(model, names, 'left_gripper_frame_link', policy).solve(target, anchor)
    assert result['status'] == 'IK_FEASIBLE'
    actual = model.link_transforms({**anchor, **result['joint_positions_rad']})['left_gripper_frame_link']
    desired = np.asarray(result['target_tcp_transform'])
    assert np.linalg.norm(actual[:3, 3] - desired[:3, 3]) <= policy.position_tolerance_m
    assert np.degrees(Rotation.from_matrix(desired[:3, :3].T @ actual[:3, :3]).magnitude()) <= policy.orientation_tolerance_degrees
    assert 'left_gripper_joint' not in result['joint_positions_rad']
    assert not result['collision_checked'] and not result['path_checked']


def test_position_only_success_cannot_approve_an_incompatible_orientation(tmp_path):
    model = VisualRobot(PLANAR, tmp_path)
    target = np.eye(4)
    target[:3, :3] = Rotation.from_euler('x', 90, degrees=True).as_matrix()
    result = BoundedPoseIK(model, ['q'], 'tool', IKPolicy(maximum_starts=2)).solve(target, {'q': 0.})
    assert result['status'] == 'ORIENTATION_NOT_MATCHED'
    assert result['position_only_diagnostic']['position_error_m'] == pytest.approx(0)
    assert result['orientation_error_degrees'] >= 89
    assert not result['kinematically_feasible']


@pytest.mark.parametrize('symmetry,expected', [(True, 'IK_FEASIBLE'), (False, 'ORIENTATION_NOT_MATCHED')])
def test_parallel_jaw_half_turn_is_explicit_and_handles_pi_rotation(tmp_path, symmetry, expected):
    model = VisualRobot(PLANAR, tmp_path)
    target = np.eye(4)
    target[:3, :3] = Rotation.from_euler('z', 180, degrees=True).as_matrix()
    result = BoundedPoseIK(model, ['q'], 'tool', IKPolicy(maximum_starts=2, allow_parallel_jaw_half_turn=symmetry)).solve(target, {'q': 0.})
    assert result['status'] == expected
    if symmetry:
        assert result['half_turn_applied'] and result['orientation_error_degrees'] < 1e-4


@pytest.mark.parametrize('height,accepted', [(.004, True), (.006, False)])
def test_acceptance_uses_separate_position_threshold_not_optimizer_success(tmp_path, height, accepted):
    model = VisualRobot(PLANAR, tmp_path)
    target = np.eye(4)
    target[2, 3] = height
    result = BoundedPoseIK(model, ['q'], 'tool', IKPolicy(maximum_starts=2)).solve(target, {'q': 0.})
    assert result['kinematically_feasible'] == accepted
    assert result['position_error_m'] == pytest.approx(height)


def test_far_target_is_rejected_without_claiming_a_global_impossibility_proof(arm):
    model, layout, _ = arm
    target, names = known_target(model, layout)
    target[:3, 3] = [5, 5, 5]
    result = BoundedPoseIK(model, names, 'left_gripper_frame_link', IKPolicy(maximum_starts=2, maximum_function_evaluations=40)).solve(target, joint_positions(layout))
    assert result['status'] == 'NO_POSITION_SOLUTION_FOUND'
    assert not result['search_complete']
    assert all(model.bounds[n][0] <= q <= model.bounds[n][1] for n, q in result['joint_positions_rad'].items())


def test_grasp_and_tcp_transforms_compose_offset_in_grasp_frame(arm):
    _, _, settings = arm
    settings['grasp_to_tcp_translation_m'] = [.01, .02, .03]
    camera = np.eye(4)
    camera[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    camera[:3, 3] = [.5, .2, .1]
    grasp = {'rotation_matrix': np.eye(3).tolist(), 'translation_m': [.1, 0, .4]}
    target = target_from_grasp(grasp, camera, settings)
    assert target[:3, 3] == pytest.approx([.48, .31, .53])
    assert target[:3, 2] == pytest.approx([0, 1, 0])


def test_candidate_selection_excludes_high_scoring_rejected_pose_and_preserves_inputs(arm):
    model, layout, settings = arm
    target, _ = known_target(model, layout)
    accepted = grasp_for_target(target, settings, .1)
    rejected = deepcopy(accepted)
    rejected.update(translation_m=[5, 5, 5], score=100.)
    before = deepcopy((layout, accepted, rejected))
    result = evaluate_candidates(model, layout, [accepted, rejected], settings)
    assert result['selected_index'] == 0 and result['accepted_count'] == 1
    assert result['candidates'][1]['status'] == 'NO_POSITION_SOLUTION_FOUND'
    assert (layout, accepted, rejected) == before
    assert not result['physical_task_completed'] and result['hardware_commands'] == 0
    assert not result['collision_checked'] and not result['gripper_width_checked']
    assert result['robot_description_sha256'] and len(result['active_joint_limits_rad']) == 5


@pytest.mark.parametrize('field,value', [('rotation_matrix', np.diag([1, 1, -1]).tolist()),
                                       ('translation_m', [float('nan'), 0, 0]),
                                       ('score', float('nan')), ('width_m', -1)])
def test_invalid_candidate_never_has_a_selected_solution(arm, field, value):
    model, layout, settings = arm
    target, _ = known_target(model, layout)
    grasp = grasp_for_target(target, settings)
    grasp[field] = value
    result = evaluate_candidates(model, layout, [grasp], settings)
    assert result['selected_index'] is None
    assert result['candidates'][0]['status'] == 'INVALID_CANDIDATE'


@pytest.mark.parametrize('changes', [{'position_tolerance_m': True}, {'position_tolerance_m': 0},
                                    {'orientation_tolerance_degrees': float('nan')},
                                    {'orientation_tolerance_degrees': 180}, {'maximum_starts': 1},
                                    {'maximum_function_evaluations': 0}, {'random_seed': True},
                                    {'allow_parallel_jaw_half_turn': 1}])
def test_invalid_policy_cannot_relax_or_unbound_the_check(changes):
    with pytest.raises(ValueError):
        IKPolicy(**changes)


def test_camera_change_invalidates_solution_and_rejected_solution_cannot_be_applied(arm):
    from tools.run.preview_grasp_scene import PreviewScene
    model, layout, settings = arm
    target, _ = known_target(model, layout)
    report = evaluate_candidates(model, layout, [grasp_for_target(target, settings)], settings)
    report['layout_at_check'] = deepcopy(layout)
    scene = PreviewScene.__new__(PreviewScene)
    scene.robot, scene.layout, scene.ik_result = model, deepcopy(layout), report
    assert scene.ik_current()
    # A widget round-trip through float32 must not invalidate a target.
    report['layout_at_check']['camera_translation_m'][0] = .12
    scene.layout['camera_translation_m'][0] = float(np.float32(.12))
    assert scene.ik_current()
    report['layout_at_check'] = deepcopy(layout)
    scene.layout = deepcopy(layout)
    scene.layout['camera_translation_m'][0] += .01
    assert not scene.ik_current()
    with pytest.raises(ValueError, match='current passing'):
        scene.apply_selected_ik()
    scene.layout = deepcopy(layout)
    scene.apply_selected_ik()
    assert scene.ik_current()  # Applying the solved joints does not change the target.
    assert scene.layout['joint_positions_degrees']['gripper'] == layout['joint_positions_degrees']['gripper']
    scene.ik_result['selected_index'] = None
    with pytest.raises(ValueError, match='current passing'):
        scene.apply_selected_ik()


def test_empty_candidates_have_no_selected_solution(arm):
    model, layout, settings = arm
    report = evaluate_candidates(model, layout, [], settings)
    assert report['selected_index'] is None and report['accepted_count'] == 0
