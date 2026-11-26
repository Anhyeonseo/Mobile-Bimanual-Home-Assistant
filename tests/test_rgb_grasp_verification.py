from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from home_robot_tasks.rgb_grasp_verification import RGBGraspVerifier, GraspRetryGate


def pose(x=0., z=0.):
    value=np.eye(4);value[0,3]=x;value[2,3]=z
    return value


def verifier(arm='left'):
    return RGBGraspVerifier(arm=arm, attempt_id='try-1', capture_id='capture-1',
        target_id='remote', camera_frame=arm+'_optical', parent_frame=arm+'_wrist',
        parent_from_optical=np.eye(4), camera_matrix=[[500,0,160],[0,500,120],[0,0,1]],
        image_size=[320,240], target_points_base=[[-.02,-.02,1],[.02,-.02,1],[.02,.02,1],[-.02,.02,1]],
        reference_base_from_tcp=pose(z=1))


def rectangle(x):
    mask=np.zeros((240,320),bool);mask[110:131,x-10:x+11]=True
    return mask


def sample(n=0, *, moving_camera=False, target='held', arm='left', **changes):
    stamp=10+n*.1
    x=(160 if target=='held' else 110) if moving_camera else (210 if target=='held' else 160)
    data=dict(arm=arm,camera_frame=arm+'_optical',sequence=n+1,observed_s=stamp,now=stamp+.01,
        pose_observed_s=stamp,jaw_observed_s=stamp,base_from_tcp=pose(.1,1),
        base_from_parent=pose(.1 if moving_camera else 0),target_masks=[rectangle(x)],
        occluded_mask=np.zeros((240,320),bool),jaw_state='contact' if target=='held' else 'empty')
    data.update(changes)
    return data


@pytest.mark.parametrize('arm',['left','right'])
@pytest.mark.parametrize('moving_camera',[False,True])
@pytest.mark.parametrize('target,expected',[('held','SUCCESS'),('source','FAILURE')])
def test_competing_hypotheses_with_fixed_or_arm_mounted_camera(arm,moving_camera,target,expected):
    v=verifier(arm)
    results=[v.observe(**sample(n,arm=arm,moving_camera=moving_camera,target=target)) for n in range(4)]
    assert [r.state for r in results[:3]]==['UNKNOWN']*3
    assert results[-1].state==expected


@pytest.mark.parametrize('change,reason',[
    ({'target_masks':[]},'TARGET_MISSING_OR_AMBIGUOUS'),
    ({'target_masks':[rectangle(210),rectangle(160)]},'TARGET_MISSING_OR_AMBIGUOUS'),
    ({'occluded_mask':None},'VISIBILITY_EVIDENCE_REQUIRED'),
    ({'occluded_mask':np.ones((240,320),bool)},'OCCLUDED'),
    ({'jaw_state':'unknown'},'CONTRADICTORY_OR_INSUFFICIENT_EVIDENCE'),
    ({'base_from_tcp':pose(z=1)},'INSUFFICIENT_MEASURED_MOTION'),
    ({'arm':'right'},'WRONG_ARM_OR_CAMERA'),
    ({'pose_observed_s':9.8},'STALE_OR_UNSYNCHRONIZED'),
    ({'now':9.9},'STALE_OR_UNSYNCHRONIZED'),
    ({'verification_phase':False},'WAIT_VERIFICATION_PHASE'),
    ({'base_from_parent':pose(1)},'HYPOTHESIS_OUT_OF_VIEW'),
])
def test_no_disappearance_or_occlusion_success(change,reason):
    result=verifier().observe(**sample(**change))
    assert (result.state,result.reason)==('UNKNOWN',reason)


def test_motion_toward_camera_is_not_distinguishable_in_rgb():
    result=verifier().observe(**sample(base_from_tcp=pose(z=1.02)))
    assert result.reason=='HYPOTHESES_NOT_SEPARABLE'


def test_replay_does_not_satisfy_dwell():
    v=verifier()
    v.observe(**sample())
    assert v.observe(**sample()).reason=='REPEATED_OR_REGRESSED_SAMPLE'
    assert v.observe(**sample(3)).state=='UNKNOWN'


def test_loss_after_success_cannot_turn_into_retryable_miss():
    v=verifier()
    for n in range(4):r=v.observe(**sample(n))
    assert r.state=='SUCCESS'
    assert v.observe(**sample(4,target_masks=[])).state=='UNKNOWN'
    for n in range(5,9):r=v.observe(**sample(n,target='source'))
    assert r.reason=='PAYLOAD_LOST_REQUIRES_REVIEW'


def test_image_adapter_calls_existing_detector_and_requires_visibility():
    v=verifier();calls=[]
    def detector(rgb):
        calls.append(rgb.shape)
        return [SimpleNamespace(object_id='remote',confidence=.9,mask=rectangle(210))]
    data=sample();data.pop('target_masks');data.pop('occluded_mask')
    result=v.observe_rgb(np.zeros((240,320,3),np.uint8),detector=detector,
        visibility=lambda rgb,meta:np.zeros(rgb.shape[:2],bool),**data)
    assert result.reason=='WAIT_STABLE_EVIDENCE'
    assert calls==[(240,320,3)]


def failure():
    v=verifier()
    for n in range(4):result=v.observe(**sample(n,target='source'))
    return result


def recovery(**changes):
    data=dict(now=11.,stopped=True,controller_ok=True,empty_hand=True,recovery_complete=True,
        new_capture_id='capture-2',capture_s=10.8,measured_state_s=10.9)
    data.update(changes)
    return data


def test_retry_is_single_use_and_requires_new_scene():
    gate=GraspRetryGate('left',1);miss=failure()
    assert not gate.admit(miss,**recovery(new_capture_id='capture-1'))[0]
    assert gate.admit(miss,**recovery())[0]
    assert not gate.admit(miss,**recovery())[0]
    assert not gate.admit(replace(miss,attempt_id='try-2'),**recovery())[0]


@pytest.mark.parametrize('changes',[
    {'stopped':False},{'controller_ok':False},{'empty_hand':False},
    {'recovery_complete':False},{'measured_state_s':10.5},{'capture_s':10.2},
    {'capture_s':float('nan')},
])
def test_faults_or_old_recovery_cannot_admit_retry(changes):
    assert not GraspRetryGate('left').admit(failure(),**recovery(**changes))[0]


@pytest.mark.parametrize('state',['SUCCESS','UNKNOWN'])
def test_only_confirmed_failure_can_retry(state):
    assert not GraspRetryGate('left').admit(replace(failure(),state=state),**recovery())[0]


def test_right_evidence_cannot_admit_left_retry():
    assert not GraspRetryGate('left').admit(replace(failure(),arm='right'),**recovery())[0]


def test_rotating_mount_changes_projection_not_the_world_reference():
    v=verifier()
    # Camera translates with wrist and rotates 90 degrees about optical Z.
    camera=pose(.1);camera[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
    for n in range(4):
        r=v.observe(**sample(n,base_from_parent=camera,target_masks=[rectangle(160)]))
    assert r.state=='SUCCESS'


def test_snapshot_session_accepts_bound_evidence_and_expires_success():
    from test_carry_snapshot_execution import admitted_inputs,refresh_mechanics
    from home_robot_tasks.carry_execution import CarrySession
    inputs,now,_,_=admitted_inputs()
    inputs.bind_grasp_verifier(arm='left',attempt_id='try-1',camera_frame='left_optical')
    inputs.admit_snapshot();inputs.set_phase('hold')
    evidence=replace(failure(),capture_id=inputs.program.capture_id,target_id=inputs.program.candidate_id,
        observed_s=now[0]-.01,state='SUCCESS',reason='TARGET_FOLLOWED_GRIPPER')
    inputs.add_grasp_verification(evidence)
    f=inputs.feedback()
    assert f.payload_relation=='held' and f.vision_s==evidence.observed_s
    session=CarrySession(inputs.program,None,None,None,lambda:now[0],observation_policy='snapshot_then_verify')
    q=list(f.positions_rad);q[5]=inputs.program.close_control_rad
    session.last_feedback=replace(f,positions_rad=tuple(q))
    assert session.holding_reason=='HOLDING_VISUALLY_VERIFIED'
    with pytest.raises(ValueError,match='replayed'):
        inputs.add_grasp_verification(evidence)
    with pytest.raises(ValueError,match='binding'):
        inputs.add_grasp_verification(replace(evidence,arm='right'))
    now[0]+=.3;refresh_mechanics(inputs,now[0],11)
    with pytest.raises(ValueError,match='observation stale'):
        inputs.feedback()


def test_confirmed_negative_is_latched_in_runtime():
    from test_carry_snapshot_execution import admitted_inputs,refresh_mechanics
    inputs,now,_,_=admitted_inputs()
    inputs.bind_grasp_verifier(arm='left',attempt_id='try-1',camera_frame='left_optical')
    inputs.admit_snapshot();inputs.set_phase('hold')
    evidence=replace(failure(),capture_id=inputs.program.capture_id,target_id=inputs.program.candidate_id,
        observed_s=now[0]-.01)
    inputs.add_grasp_verification(evidence)
    assert inputs.feedback().payload_relation=='empty'
    now[0]+=.3;refresh_mechanics(inputs,now[0],11)
    inputs.add_grasp_verification(replace(evidence,observed_s=now[0],sequence=10,state='UNKNOWN'))
    assert inputs.feedback().payload_relation=='empty'


def test_replay_diagnostic_reads_saved_masks_without_hardware(tmp_path):
    import json
    from tools.diagnostics.replay_rgb_grasp import replay
    v=verifier()
    config=dict(arm='left',attempt_id='try-1',capture_id='capture-1',target_id='remote',
        camera_frame='left_optical',parent_frame='left_wrist',parent_from_optical=np.eye(4).tolist(),
        camera_matrix=v.k.tolist(),image_size=[320,240],target_points_base=v.points[:3].T.tolist(),
        reference_base_from_tcp=v.reference.tolist())
    np.save(tmp_path/'target.npy',rectangle(210))
    np.save(tmp_path/'occluded.npy',np.zeros((240,320),bool))
    frames=[]
    for n in range(4):
        data=sample(n);data['target_masks']=['target.npy'];data['occluded_mask']='occluded.npy'
        for key in ('base_from_parent','base_from_tcp'):data[key]=data[key].tolist()
        frames.append(data)
    path=tmp_path/'manifest.json';path.write_text(json.dumps(dict(calibration=config,frames=frames)))
    report=replay(path)
    assert report['results'][-1]['state']=='SUCCESS'
    assert report['hardware_commands']==0 and not report['physical_validation']
