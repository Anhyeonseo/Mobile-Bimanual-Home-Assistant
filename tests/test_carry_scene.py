"""Observed-scene handoff: no invented robot state or axis-aligned table."""
from copy import deepcopy
import numpy as np
import pytest
from home_robot_tasks.carry_scene import from_prepared_capture, MAX_OBSERVED_BOXES_PER_ROLE
from home_robot_tasks.s2_geometry import JOINTS


@pytest.fixture
def scene():
    transform=np.array([[0,-1,0,.3],[1,0,0,-.2],[0,0,1,.8],[0,0,0,1.]])
    p=dict(calibration_id='profile',base_from_optical=transform.tolist(),lift=dict(travel_m=.64))
    shared=dict(capture_id='capture',source_sha256={'a':'b'},calibration_id='profile',session_id='session',observation_index=0)
    q={'s2_lift_joint':.21,**{f'{side}_model_{n}':i*.1 for side in ('left','right') for i,n in enumerate(JOINTS)}}
    review=dict(shared,mode='offline_capture_review',status='TARGET_DISPLAY_ONLY',executable=False,motion_authorized=False,
        frame='base_link',lift_height_m=.21,model_joint_positions_si=q,
        **{side+'_anchor_rad':{n:q[f'{side}_model_{n}'] for n in JOINTS[:5]} for side in ('left','right')})
    observed=dict(shared,frame='capture_support',mask_sha256='mask',motion_authorized=False,visible_payload_extent_only=True,
        support_from_optical=np.eye(4).tolist(),support_boxes_m=[[[0,0,-.01],[.2,.1,0]]],obstacle_boxes_m=[],
        target_bounds_m=[[.01,.02,0],[.05,.08,.03]])
    prepared=dict(mode='offline_grasp_replay',status='CAPTURE_CANDIDATES_READY',motion_authorized=False,hardware_commands=0,
        capture_id='capture',source_sha256=shared['source_sha256'],review=review,observed_scene=observed,group=dict(shared,mask_sha256='mask'))
    return prepared,p


def test_preserves_observed_rotation_and_complete_captured_state(scene):
    data,p=scene;boxes,payload,q=from_prepared_capture(data,p)
    np.testing.assert_allclose(boxes[0].base_pose[:3,3],[.25,-.1,.795])
    np.testing.assert_allclose(boxes[0].base_pose[:3,:3],np.array(p['base_from_optical'])[:3,:3])
    assert len(q)==13 and q['right_model_gripper']==.5 and q['s2_lift_joint']==.21
    np.testing.assert_allclose(payload.base_from_object[:3,3],[.25,-.17,.815])


def test_zero_additional_padding_retains_actual_point_bounds(scene):
    data,p=scene
    data['observed_scene'].update(nominal_obstacle_padding_m=.001,
        nominal_obstacle_boxes_m=[[[.009,.019,0],[.021,.031,.041]]])
    boxes,_,_=from_prepared_capture(data,p,nominal_obstacles=True,obstacle_padding_m=0.)
    obstacle=boxes[-1]
    np.testing.assert_allclose(obstacle.size_m,[.01,.01,.04])
    np.testing.assert_allclose(obstacle.base_pose[:3,3],[.275,-.185,.82])


def test_new_scene_uses_measured_envelope_without_an_operator_override(scene):
    data,p=scene
    data['observed_scene'].update(preferred_obstacle_envelope='point_bounds',nominal_obstacle_padding_m=.001,
        nominal_obstacle_boxes_m=[[[.009,.019,0],[.021,.031,.041]]],
        obstacle_boxes_m=[[[.005,.015,0],[.025,.035,.045]]])
    boxes,_,_=from_prepared_capture(data,p)
    np.testing.assert_allclose(boxes[-1].size_m,[.012,.012,.041])
    rounded,_,_=from_prepared_capture(data,p,nominal_obstacles=False)
    np.testing.assert_allclose(rounded[-1].size_m,[.02,.02,.045])


def test_dense_floor_capture_keeps_every_observed_box(scene):
    data,p=scene
    bounds=[[[i*.005,0,-.8],[i*.005+.002,.002,-.798]] for i in range(20001)]
    data['observed_scene'].update(preferred_obstacle_envelope='point_bounds',
        nominal_obstacle_padding_m=.001,nominal_obstacle_boxes_m=bounds)
    boxes,_,_=from_prepared_capture(data,p)
    assert len(boxes)==len(bounds)+1
    assert boxes[-1].name=='observed_obstacle_20000'
    transform=np.array(p['base_from_optical'])
    expected=(transform@np.r_[np.mean(bounds[-1],axis=0),1])[:3]
    np.testing.assert_allclose(boxes[-1].base_pose[:3,3],expected)
    np.testing.assert_allclose(boxes[-1].size_m,[.002,.002,.002])


def test_refined_mesh_keeps_support_frame_and_is_used_in_default_scene(scene):
    from itertools import product
    from home_perception.geometry.obstacle_envelopes import separate_target_padding
    data,p=scene
    target=np.array(list(product([0.,.02],[0.,.02],[0.,.02])))
    returns=np.array([[.0201,.01,.008]])
    bounds=[[.0191,.009,0.],[.0211,.011,.009]]
    meshes=separate_target_padding(bounds,returns,target)
    data['observed_scene'].update(preferred_obstacle_envelope='point_bounds',nominal_obstacle_padding_m=.001,
        nominal_obstacle_boxes_m=[bounds],nominal_obstacle_padding_refinements=[dict(
            box_index=0,meshes=meshes,measured_returns=1,all_returns_preserved=True)])
    boxes,_,_=from_prepared_capture(data,p)
    assert len(boxes)==1+len(meshes) and all(b.mesh is not None for b in boxes[1:])
    np.testing.assert_allclose(boxes[-1].base_pose,p['base_from_optical'])
    # Zero-padding comparison uses its own point bounds; never clips twice.
    boxes,_,_=from_prepared_capture(data,p,obstacle_padding_m=0.)
    assert len(boxes)==2 and boxes[-1].mesh is None


@pytest.mark.parametrize('index,preserved',[(1,True),(0,False),(-1,True)])
def test_rejects_invalid_refinement_evidence(scene,index,preserved):
    data,p=scene
    data['observed_scene'].update(preferred_obstacle_envelope='point_bounds',nominal_obstacle_padding_m=.001,
        nominal_obstacle_boxes_m=[[[0.,0.,0.],[.01,.01,.01]]],
        nominal_obstacle_padding_refinements=[dict(box_index=index,measured_returns=1,
            all_returns_preserved=preserved,meshes=[{}])])
    with pytest.raises(ValueError,match='invalid observed obstacle padding refinement'):
        from_prepared_capture(data,p)


def test_scene_larger_than_supported_exposure_is_rejected_without_truncation(scene):
    data,p=scene
    data['observed_scene']['obstacle_boxes_m']=[[[0,0,0],[.01,.01,.01]]]*(MAX_OBSERVED_BOXES_PER_ROLE+1)
    with pytest.raises(ValueError,match='bounded observed obstacle'):
        from_prepared_capture(data,p)


@pytest.mark.parametrize('change',['other_session','other_mask','missing_jaw','changed_anchor','imagined_lift','nan_box','wrong_frame'])
def test_rejects_broken_binding(scene,change):
    data,p=deepcopy(scene)
    if change=='other_session':data['group']['session_id']='older'
    if change=='other_mask':data['observed_scene']['mask_sha256']='other'
    if change=='missing_jaw':del data['review']['model_joint_positions_si']['right_model_gripper']
    if change=='changed_anchor':data['review']['left_anchor_rad']['base']=.9
    if change=='imagined_lift':data['review']['model_joint_positions_si']['s2_lift_joint']=.51
    if change=='nan_box':data['observed_scene']['support_boxes_m'][0][1][0]=float('nan')
    if change=='wrong_frame':data['observed_scene']['frame']='base_link'
    with pytest.raises(ValueError):from_prepared_capture(data,p)
