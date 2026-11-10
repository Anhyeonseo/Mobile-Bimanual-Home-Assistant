import hashlib
import json
from copy import deepcopy

import numpy as np
import pytest
from scipy.io import savemat

from home_perception.grasp.handoff import CONVENTION
from home_robot_tasks.stationary_pick_sim import pad_tip_optical, plan_pick
from tools.diagnostics import run_s3_stationary_lab as lab
from tools.diagnostics import yeoeun_grasp_export as export


def test_same_short_side_contact_uses_one_depth_offset_and_lifts():
    arm = lab.model(.61)
    trial = lab.run_case(arm, lab.CASES[2], 1, candidate_source='yeoeun-surface', marker_length_m=.028)
    assert trial['status'] == 'SIMULATED_PICK_HELD' and trial['trial']['retained_at_end']
    from home_robot_tasks import s2_geometry as geo
    for stage in trial['plan']['attempts'][-1]['stages']:
        angles = [geo.model_from_control(stage['control_rad'][j],arm.alignment[j]) for j in geo.JOINTS[:5]]
        np.testing.assert_allclose(angles,stage['model_rad'],atol=1e-12)
    candidates = trial['group']['refined_grasps']
    original = candidates[0]
    chosen = candidates[trial['plan']['candidate_index']]
    assert original['source'] == 'rule' and chosen['source'] == 'surface_variant'
    assert original['width_m'] == chosen['width_m']
    tip = pad_tip_optical(chosen)
    np.testing.assert_allclose(tip[:3, 3] - chosen['translation_m'],
                               chosen['depth_m'] * np.array(chosen['rotation_matrix'])[:, 0], atol=1e-12)
    goal = trial['plan']['attempts'][-1]['stages'][1]['goal_base']
    fixed_goal = np.array(goal)
    jaw_midpoint = fixed_goal[:3, 3] - fixed_goal[:3, 0] * arm.jaw.center_from_fixed_m(chosen['width_m'])
    np.testing.assert_allclose(jaw_midpoint, (arm.profile['base_from_optical'] @ tip)[:3, 3])
    group = dict(trial['group'])
    del group['tcp_convention']
    path = plan_pick(arm, trial['review'], group, trial['scene_boxes_m'])
    assert {a['reason'] for a in path['attempts']} == {'UNKNOWN_GRASP_TCP_CONVENTION'}


@pytest.mark.parametrize('fault,expected', [
    ('obstacle', 'NO_VIRTUAL_PATH'), ('empty_close', 'STOPPED_PICK'),
    ('lost_load', 'STOPPED_PICK'), ('dropped_during_hold', 'LOAD_LOST_DURING_HOLD')])
def test_surface_candidates_keep_existing_motion_and_load_checks(fault, expected):
    arm = lab.model(.61)
    out = lab.run_case(arm, lab.CASES[2], 1, fault, candidate_source='yeoeun-surface', marker_length_m=.028)
    assert out['status'] == expected


def test_irregular_shape_is_not_given_a_surface_variant():
    arm = lab.model(.61)
    out = lab.run_case(arm, lab.CASES[0], 1, candidate_source='yeoeun-surface')
    assert out['status'] == 'NO_PERCEPTION_CANDIDATE'
    assert out['group']['candidate_reason'] == 'not_elongated' and 'trial' not in out


def flat_body(length=.15, width=.03):
    from home_perception.geometry.plane import Plane
    along,across=np.meshgrid(np.linspace(-length/2,length/2,101),np.linspace(-width/2,width/2,25))
    target=np.c_[along.ravel(),across.ravel(),np.full(along.size,.64)]
    return target,Plane(np.array([0.,0.,-1.]),np.array([0.,0.,.65]),.001,.9)


def test_surface_sections_sample_observed_body_without_moving_scene():
    from home_perception.grasp.handoff import flat_candidates
    target,plane=flat_body();original=target.copy()
    candidates=flat_candidates(target,target,plane,surface_variants=True)['refined_grasps']
    assert len(candidates)<=51
    variants=[c for c in candidates if c['source']=='surface_variant']
    assert {c['longitudinal_offset_m'] for c in variants}=={0.,-.01,.01,-.02,.02}
    centre=next(c for c in variants if c['longitudinal_offset_m']==0 and c['tilt_deg']==0 and c['jaw_sign']==1)
    tip=lambda c:np.array(c['translation_m'])+c['depth_m']*np.array(c['rotation_matrix'])[:,0]
    axis=np.array(centre['rotation_matrix'])[:,2]
    for c in variants:
        assert c['closing_section']['status']=='VISIBLE_SPAN'
        assert (tip(c)-tip(centre))@axis==pytest.approx(c['longitudinal_offset_m'])
    np.testing.assert_array_equal(target,original)


def test_surface_sections_do_not_extend_past_observed_ends():
    from home_perception.grasp.handoff import flat_candidates
    target,plane=flat_body(length=.04,width=.008)
    variants=flat_candidates(target,target,plane,surface_variants=True)['refined_grasps']
    assert all(abs(c.get('longitudinal_offset_m',0))<.02 for c in variants)


def test_offcentre_surface_section_still_checks_neighbouring_material():
    from home_perception.grasp.handoff import flat_candidates
    target,plane=flat_body()
    candidates=flat_candidates(target,target,plane,surface_variants=True)['refined_grasps']
    chosen=next(c for c in candidates if c.get('longitudinal_offset_m')==.02 and c['tilt_deg']==0 and c['jaw_sign']==1)
    rotation=np.array(chosen['rotation_matrix'])
    obstacle=np.array(chosen['translation_m'])+rotation@np.array([.01,chosen['width_m']/2,0.])
    scene=np.vstack((target,obstacle+np.arange(8)[:,None]*.00001))
    checked=flat_candidates(target,scene,plane,surface_variants=True)['refined_grasps']
    assert any(c.get('longitudinal_offset_m')==0 for c in checked)
    assert not any(c.get('longitudinal_offset_m')==.02 and c['tilt_deg']==0 and c['jaw_sign']==1 for c in checked)


def write(path, data):
    path.write_text(json.dumps(data))


@pytest.fixture
def exported(tmp_path):
    capture = 'test-exposure'
    root, perception = tmp_path/'grasps', tmp_path/'perception'
    frame = root/capture
    for directory in (perception/capture, frame/'objects', frame/'sample', frame/'grasps_000'):
        directory.mkdir(parents=True)
    meta = dict(depth_scale_m=.001, aligned_depth_intrinsics=dict(fx=200., fy=200., ppx=20., ppy=15.))
    K = [[200.,0,20.],[0,200.,15.],[0,0,1.]]
    sample = frame/'sample'
    for name in ('color.png','depth.png','workspace_mask.png'):
        (sample/name).write_bytes(name.encode())
    savemat(sample/'meta.mat', {'factor_depth':1000.,'intrinsic_matrix':K})
    inputs = {f.name:export.digest(f) for f in sample.iterdir()}
    source = {'color.png':inputs['color.png'],'depth_aligned.png':inputs['depth.png'],
              'metadata.json':hashlib.sha256(json.dumps(meta).encode()).hexdigest()}
    mask = 'mask_000.png'
    for directory in (perception/capture, frame/'objects'):
        (directory/mask).write_bytes(b'test-segmentation')
    optical = 'color_camera_optical_frame'
    record = dict(index=0, label='remote', status='OK', mask=mask, perception_status='accepted')
    write(frame/'objects/objects.json', dict(input_sha256=inputs, objects=[record],
                                              coordinate_frame=optical, intrinsic_matrix=K))
    checkpoint = export.CHECKPOINT_SHA256
    write(frame/'grasps_000/grasps.json', dict(target=record, coordinate_frame=optical,
                                               checkpoint_sha256=checkpoint, baseline_revision=export.BASELINE_REVISION,
                                               model='official GraspNet baseline (RealSense)'))
    candidate = dict(source='graspnet', score=.6, width_m=.03, depth_m=.02,
                     rotation_matrix=np.eye(3).tolist(), translation_m=[.01,.01,.5])
    row = [[.6,.03,.02,.02,*np.eye(3).ravel(),.01,.01,.5,-1.]]
    np.save(frame/'grasps_000/grasps_refined.npy', row)
    np.save(frame/'grasps_000/grasps_filtered.npy', row)
    write(root/'report.json', dict(mode='offline_grasp_candidates',status='COMPLETED', hardware_commands=0,
                                   robot_transform=None,checkpoint_sha256=checkpoint,objects=[
                                       dict(record,capture=capture,exit_code=0,refined=1,flat_rule='not_flat',
                                            refined_grasps=[candidate])]))
    obs = dict(capture_id=capture, label=record['label'], frame_id=optical, source_sha256=source)
    write(perception/'report.json', dict(mode='offline_perception_smoke',status='COMPLETED', robot_transform=None,
                                         hardware_commands=0,captures=[dict(capture=capture,observations=[obs])]))
    write(perception/capture/'observations.json',dict(observations=[obs]))
    return root, perception, capture, 0, obs, source, meta


def test_upstream_export_preserves_graspnet_rows_and_raw_origin(exported):
    root, perception, capture, _, obs, _, _ = exported
    assert export.original_observations(perception, capture) == [obs]
    group = export.refined_group(*exported)
    assert group['tcp_convention'] == CONVENTION and group['refined_grasps'][0]['source'] == 'graspnet'
    assert group['refined_grasps'][0]['translation_m'] == [.01,.01,.5]


@pytest.mark.parametrize('change', ['mask','image','wrong_index','different_observation','width','scale'])
def test_upstream_export_refuses_crossed_inputs(exported, change):
    root, perception, capture, index, obs, source, meta = deepcopy(exported)
    frame = root/capture
    if change == 'mask':
        (frame/'objects/mask_000.png').write_bytes(b'other mask')
    elif change == 'image':
        (frame/'sample/depth.png').write_bytes(b'other depth')
    elif change == 'wrong_index':
        index = 1
    elif change == 'different_observation':
        obs['label'] = 'cup'
    elif change == 'scale':
        meta['depth_scale_m'] *= 2
    else:
        report = export.read(root/'report.json')
        report['objects'][0]['refined_grasps'][0]['width_m'] = .01
        write(root/'report.json', report)
    with pytest.raises(ValueError):
        export.refined_group(root, perception, capture, index, obs, source, meta)


def test_detection_replay_refuses_changed_mask_or_capture(tmp_path):
    pytest.importorskip('PIL')
    from PIL import Image
    from tools.diagnostics.replay_s3_grasp import grounding_observations
    shape = (40, 80)
    mask_path = tmp_path/'mask_000.png'
    Image.fromarray(np.full(shape, 255, dtype=np.uint8)).save(mask_path)
    meta = dict(captured_at_utc='test', coordinate_frame='color_camera_optical_frame')
    report = dict(schema_version=2, source_snapshot=str(tmp_path), captured_at_utc='test', segmentation={'model':'sam'},
                  coordinate_frame=meta['coordinate_frame'], robot_transform=None, source_sha256={},
                  detections=[dict(id=0,segmentation=dict(mask_file=mask_path.name,sha256='0'*64,predicted_iou=.98))])
    path = tmp_path/'detections.json'
    write(path, report)
    with pytest.raises(ValueError, match='mask digest'):
        grounding_observations(tmp_path, path, meta, np.ones(shape), None, {})
    with pytest.raises(ValueError, match='exposure'):
        grounding_observations(tmp_path, path, meta, np.ones(shape), None, {'different':'frame'})


def test_stock_jaw_section_uses_opposing_official_meshes_and_rotating_finger():
    jaw = lab.stock_jaw()
    # Zero-angle CAD places the opposed faces ~7.9 mm to either side in gripper_link.
    assert np.interp(0., jaw.angles, jaw.gaps) == pytest.approx(.0158, abs=.00005)
    assert jaw.tip_to_section_m == pytest.approx(.00630, abs=.00002)
    assert .063 < jaw.gaps[-1] < .064 and 33 < np.degrees(jaw.angles[-1]) < 34
    assert jaw.accepts(.060) and not jaw.accepts(.065)
    # The official TCP is at the fixed jaw face, not halfway between the jaws.
    assert jaw.center_from_fixed_m(.058) == pytest.approx(.029, abs=.00001)
    with pytest.raises(ValueError, match='SO101_JAW_SECTION_LIMIT'):
        jaw.center_from_fixed_m(.065)


def test_graspnet_export_never_promotes_a_rule_row(exported):
    root, _, capture, *_ = exported
    frame = root/capture/'grasps_000'
    network = np.load(frame/'grasps_refined.npy')
    rule = network.copy()
    rule[0, 0] = 0.
    np.save(frame/'grasps_refined.npy', np.vstack([rule, network]))
    report = export.read(root/'report.json')
    obj = report['objects'][0]
    original = obj['refined_grasps'][0]
    obj.update(refined=2, refined_grasps=[dict(original, source='rule', score=0.), original])
    write(root/'report.json', report)
    assert export.refined_group(*exported)['refined_grasps'] == [original]
    assert export.refined_group(*exported, candidate_source='flat-rule')['refined_grasps'][0]['source'] == 'rule'
    obj['refined_grasps'][0]['source'] = 'graspnet'
    write(root/'report.json', report)
    with pytest.raises(ValueError, match='missing from collision-filtered'):
        export.refined_group(*exported)


def test_graspnet_requires_both_mask_projection_and_measured_depth():
    from home_perception.grasp.graspnet import associate

    intr = dict(fx=100., fy=100., ppx=5., ppy=5.)
    mask = np.zeros((12, 12), dtype=bool)
    mask[5, 5] = True
    rows = np.zeros((4, 17))
    rows[:, 1:4] = .02
    rows[:, 13:16] = [[0, 0, .5], [0, 0, .65], [.01, 0, .5], [0, 0, -.5]]
    selected = associate(rows, intr, mask, [[0., 0., .5]])
    np.testing.assert_array_equal(selected, rows[:1])


def test_stock_jaw_pose_survives_duplicate_wider_network_prediction():
    from home_perception.geometry.plane import Plane
    from home_perception.grasp.graspnet import rank_for_jaw
    from home_perception.grasp.refine import refine

    plane = Plane(np.array([0., 0., -1.]), np.array([0., 0., .6]), 0., 1.)
    points = np.column_stack((np.linspace(-.006, .006, 60), np.zeros(60), np.full(60, .582)))
    rows = np.zeros((2, 17))
    rows[:, :4] = [[.9, .09, .02, .02], [.8, .04, .02, .02]]
    rows[:, 4:13] = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]]).ravel()
    rows[:, 15] = .569
    default, _ = refine(rows, points, plane, .018)
    assert list(default[:, 1]) == [.09]
    ranked, _ = rank_for_jaw(rows, points, plane, .018, (.0024, .0633))
    assert list(ranked[:, 1]) == [.04, .09]
