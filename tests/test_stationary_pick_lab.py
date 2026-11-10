"""Hardware-free left-arm planning. No recognition weights or robot backend."""
import json
import subprocess
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.stationary_pick_sim import NAMES, plan_pick, straight_approach_report
from home_robot_tasks.stationary_trial import simulate_pick
from tools.diagnostics import run_s3_stationary_lab as lab


@pytest.fixture(scope='module')
def arm():return lab.model(.61)


def test_fk_matches_the_existing_coordinate_model_away_from_home(arm):
    geometry=lab.read(lab.CONFIG/'official_geometry.json')
    q=arm.anchor+[.12,.13,-.06,-.025,.25]
    model_angles={**dict(zip(NAMES,q)),'gripper':.1}
    expected=arm.mount@geo.arm_transforms(geometry,model_angles)['gripper_frame_link']
    np.testing.assert_allclose(arm.fk(q)[0],expected,atol=1e-12)


def test_model_refuses_an_unrelated_control_direction_table(arm):
    ranges = lab.read(lab.CONFIG/'arm_ranges.development.json')
    ranges['arms']['left']['elbow']['positive_raw_direction'] *= -1
    with pytest.raises(ValueError, match='control ranges do not match'):
        lab.StationaryArm(arm.profile, lab.read(lab.CONFIG/'official_geometry.json'), ranges, arm.lift_m, arm.jaw)


@pytest.mark.parametrize('problem,reason',[
    ('foreign_observation','CANDIDATE_IDENTITY_MISMATCH'),('other_session','CANDIDATE_IDENTITY_MISMATCH'),
    ('different_image','CANDIDATE_IDENTITY_MISMATCH'),('held','OBSERVATION_NOT_ACCEPTED'),
    ('width','SO101_JAW_SECTION_LIMIT'),('bad_rotation','proper rigid transform required'),
    ('behind','NONPOSITIVE_DEPTH'),('off_target','CANDIDATE_OFF_TARGET'),
    ('lift','LIFT_ANCHOR_MISMATCH'),('anchor','ANCHOR_OUTSIDE_BOUNDS')])
def test_candidate_is_not_an_execution_permission(arm,problem,reason):
    review,group,_=lab.fixture(arm,lab.CASES[0],1)
    c=group['refined_grasps'][1];group['refined_grasps']=[c]
    group['executable']=True  # upstream flag must never bypass a check
    if problem=='foreign_observation':group['observation_index']+=1
    if problem=='other_session':group['session_id']='old-session'
    if problem=='different_image':group['source_sha256']={}
    if problem=='held':group['perception_status']='held'
    if problem=='width':c['width_m']=.08
    if problem=='bad_rotation':c['rotation_matrix']=np.diag([1,-1,1]).tolist()
    if problem=='behind':c['translation_m'][2]=-1
    if problem=='off_target':c['translation_m'][0]+=.1
    if problem=='lift':review['lift_height_m']=.4
    if problem=='anchor':review['left_anchor_rad']['wrist_flex']=float(arm.hi[3])+.01
    out=plan_pick(arm,review,group,lab.SCENE)
    assert out['status']=='NO_VIRTUAL_PATH' and not out['executable'] and not out['motion_authorized']
    assert out['attempts'][0]['reason']==reason


def test_fixed_destination_vs_explicit_region_does_not_silently_move_goal(arm):
    case=lab.CASES[0]
    fixed=lab.run_case(arm,case,1,task='pick-place')
    assert fixed['status']=='NO_VIRTUAL_PATH' and len(fixed['plan']['destinations_tested'])==1
    allowed=lab.run_case(arm,case,1,task='pick-place',place_half_extent=.015)
    assert allowed['status']=='SIMULATED_SEQUENCE_COMPLETED'
    final=np.array(allowed['trial']['trace'][-1]['object_xyz_m'])
    assert max(abs(final[:2]-lab.DEST[:2]))<=.015
    assert allowed['trial']['simulated_rise_m']>=.012
    assert [r['stage'] for r in allowed['trial']['trace']]==[
        'pick_open','pick_pregrasp','pick_approach','pick_close','pick_attach','pick_retreat',
        'place_preplace','place_approach','place_open','place_detach','place_retreat']
    assert allowed['trial']['physical_task_completed'] is False


@pytest.mark.parametrize('fault,status,forbidden',[
    ('empty_close','STOPPED_PICK','pick_probe'), ('table_contact','STOPPED_PICK','pick_attach'),
    ('occluded_probe','STOPPED_PICK','pick_attach'),('lost_load','STOPPED_PICK','place_preplace'),
    ('dropped_during_hold','LOAD_LOST_DURING_HOLD','place_preplace'),('cancel','STOPPED_PICK','pick_close'),
    ('overlap','AMBIGUOUS_TARGET','pick_open'),('held','REOBSERVE','pick_open'),
    ('obstacle','NO_VIRTUAL_PATH','pick_open'),('home_blocked','NO_VIRTUAL_PATH','pick_open'),
    ('wrong_exposure','NO_VIRTUAL_PATH','pick_open')])
def test_failure_does_not_advance_to_next_operation(arm,fault,status,forbidden):
    out=lab.run_case(arm,lab.CASES[0],1,fault)
    assert out['status']==status
    assert forbidden not in [r['stage'] for r in out.get('trial',{}).get('trace',[])]
    if fault in ('table_contact', 'occluded_probe'):
        trial = out['trial']
        assert trial['trace'][-1]['stage'] == 'pick_probe'
        assert trial['test_lift']['outcome'] == ('TARGET_DID_NOT_FOLLOW' if fault == 'table_contact' else 'MISSING_SAMPLE')
        assert not trial['test_lift']['verified'] and not trial['retained_at_end']
    if fault == 'dropped_during_hold':
        samples = out['trial']['hold_samples']
        assert samples[0]['held'] and samples[0]['rise_m'] >= .012
        assert samples[-1]['attached'] and not samples[-1]['held']
        assert not out['trial']['retained_at_end']


@pytest.mark.parametrize('fault,status,forbidden', [
    ('stuck_release', 'STOPPED_PLACE', 'place_detach'),
    ('unreachable_place', 'NO_VIRTUAL_PATH', 'pick_open')])
def test_historical_place_comparison_rejections(arm, fault, status, forbidden):
    out = lab.run_case(arm, lab.CASES[0], 1, fault, task='pick-place',
                       place_half_extent=.015 if fault == 'stuck_release' else 0.)
    assert out['status'] == status
    assert forbidden not in [r['stage'] for r in out.get('trial', {}).get('trace', [])]


def test_pick_ends_holding_even_when_fixed_place_is_unreachable(arm):
    case = lab.CASES[2]  # the historical fixed destination fails for this pose
    out = lab.run_case(arm, case, 1)
    assert out['status'] == 'SIMULATED_PICK_HELD'
    assert 'place_xyz_m' not in out and 'chosen_place_xyz_m' not in out['plan']
    stages = out['plan']['attempts'][-1]['stages']
    assert [s['stage'] for s in stages] == ['pick_pregrasp', 'pick_approach', 'pick_retreat']
    line = out['plan']['attempts'][-1]['straight_approach']
    assert line['reason'] is None and line['samples'] >= 11
    assert line['distance_m'] == pytest.approx(.02)
    assert line['max_lateral_m'] <= .002 and line['max_rotation_from_entry_deg'] <= 2.
    trial = out['trial']
    assert [s['stage'] for s in trial['trace']] == [
        'pick_open', 'pick_pregrasp', 'pick_approach', 'pick_close', 'pick_probe', 'pick_attach', 'pick_retreat']
    assert trial['retained_at_end']
    probe = trial['test_lift']
    assert probe['verified'] and probe['outcome'] == 'TARGET_FOLLOWED'
    assert .0039 <= probe['plan']['rise_m'] <= .0041
    assert not next(r for r in trial['trace'] if r['stage'] == 'pick_probe')['attached']
    assert [s['elapsed_s'] for s in trial['hold_samples']] == [0, .1, .2, .3, .4, .5]
    assert all(s['held'] and s['attached'] and s['rise_m'] >= .012 for s in trial['hold_samples'])
    assert trial['hardware_commands'] == 0 and not trial['physical_task_completed']


@pytest.mark.parametrize('deviation,reason', [
    ('none', None), ('side', 'APPROACH_OFF_LINE'), ('roll', 'APPROACH_ANGLE_CHANGED'),
    ('not_aligned', 'APPROACH_NOT_ALIGNED'), ('reverse', 'APPROACH_PROGRESS_INVALID'),
])
def test_approach_checks_the_middle_even_when_the_endpoints_match(deviation, reason):
    pre = np.eye(4)
    pre[:3,:3] = Rotation.from_euler('y', 65, degrees=True).as_matrix()
    grip = pre.copy()
    grip[:3,3] += .02 * pre[:3,2]
    poses = np.repeat(pre[None], 5, axis=0)
    poses[:,:3,3] += np.linspace(0, .02, 5)[:,None] * pre[:3,2]
    if deviation == 'side':
        poses[2,:3,3] += .003 * pre[:3,0]
    elif deviation == 'roll':
        poses[2,:3,:3] = pre[:3,:3] @ Rotation.from_euler('z', 3, degrees=True).as_matrix()
    elif deviation == 'not_aligned':
        poses[:,:3,:3] = pre[:3,:3] @ Rotation.from_euler('x', 11, degrees=True).as_matrix()
    elif deviation == 'reverse':
        poses[2,:3,3] = .004 * pre[:3,2]
    assert straight_approach_report(poses, pre, grip)['reason'] == reason


def test_contact_without_sufficient_rise_is_incomplete(arm):
    review, group, truth = lab.fixture(arm, lab.CASES[0], 1)
    path = plan_pick(arm, review, group, lab.SCENE)
    width = group['refined_grasps'][path['candidate_index']]['width_m']
    trial = simulate_pick(arm, path, width, truth, minimum_rise=.03)
    assert trial['status'] == 'INSUFFICIENT_SYNTHETIC_RISE'
    assert trial['retained_at_end'] and trial['final_rise_m'] < .03


def test_bound_input_cli_and_candidate_order(arm,tmp_path):
    a=lab.run_case(arm,lab.CASES[0],1);b=lab.run_case(arm,lab.CASES[0],2)
    assert a['review']['observation_index']!=b['review']['observation_index']
    inp=tmp_path/'in.json';out=tmp_path/'out.json'
    inp.write_text(json.dumps({k:a[k] for k in ('review','group','scene_boxes_m')}))
    subprocess.run([sys.executable,str(lab.ROOT/'tools/diagnostics/run_s3_stationary_lab.py'),
                    '--input',str(inp),'--output',str(out)],check=True)
    result=json.loads(out.read_text());assert result['path']['status']=='VIRTUAL_PATH_FOUND'
    assert result['hardware_commands']==0 and result['motion_authorized'] is False
    assert result['task'] == 'pick-lift' and 'destinations_tested' not in result['path']
    # Old files can contain a destination. It must not gate the current pick.
    data = json.loads(inp.read_text())
    data['place_xyz_m'] = [10., 10., 10.]
    inp.write_text(json.dumps(data))
    replay_out = tmp_path / 'replay.json'
    assert lab.main(['--input', str(inp), '--output', str(replay_out)]) == 0
    assert json.loads(replay_out.read_text())['path'] == result['path']
