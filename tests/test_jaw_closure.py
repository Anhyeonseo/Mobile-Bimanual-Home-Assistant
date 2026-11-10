import numpy as np
import pytest

from home_perception.grasp.handoff import observed_closing_section
from home_robot_tasks.jaw_closure import closure_plan, next_gap
from tools.diagnostics.run_s3_stationary_lab import stock_jaw


def candidate():
    return dict(width_m=.028, depth_m=.015, translation_m=[0.,0.,.4], rotation_matrix=np.eye(3).tolist())


def section_candidate(width, offset=0.):
    c=candidate()
    y=np.linspace(-width/2+offset,width/2+offset,101)
    points=np.column_stack((np.full_like(y,.014),y,np.full_like(y,.4)))
    # A wide part of the same object away from the chosen grip must not set the closing distance.
    off_section=points.copy();off_section[:,2]+=.07;off_section[:,1]*=4
    c['closing_section']=observed_closing_section(c,np.vstack((points,off_section)))
    return c


def test_same_candidate_adapts_to_a_different_local_width_without_an_object_label():
    a,b=(closure_plan(stock_jaw(), section_candidate(w)) for w in (.01,.04))
    assert a['visible_span_m']==pytest.approx(.0096)
    assert b['visible_span_m']==pytest.approx(.0384)
    assert a['entry_gap_m']==.028 and b['entry_gap_m']>.04
    assert a['min_search_gap_m']<b['min_search_gap_m']
    assert b['entry_angle_rad']>a['entry_angle_rad'] and not b['hardware_authorized']


def test_extra_close_preload_changes_fixed_goal_without_widening_contact_or_entry():
    candidate=section_candidate(.04)
    old=closure_plan(stock_jaw(),candidate,close_allowance_m=.004)
    new=closure_plan(stock_jaw(),candidate)
    assert new['target_gap_m']==pytest.approx(old['target_gap_m']-.001)
    assert new['min_search_gap_m']==new['target_gap_m']
    for key in ('visible_span_m','entry_gap_m','upper_contact_gap_m','entry_angle_rad'):
        assert new[key]==old[key]
    assert next_gap(new,new['target_gap_m'],'free')==('STOP_NO_CONTACT',None)
    assert not new['hardware_authorized']


@pytest.mark.parametrize('allowance',[0,.0009,.0051,float('nan'),float('inf'),True])
def test_close_preload_budget_is_bounded(allowance):
    with pytest.raises(ValueError,match='close allowance'):
        closure_plan(stock_jaw(),section_candidate(.04),close_allowance_m=allowance)


def test_asymmetric_contour_sets_entry_clearance_or_refuses_an_oversize_grasp():
    c=section_candidate(.04,offset=.005)
    p=closure_plan(stock_jaw(),c)
    assert p['entry_gap_m']>=2*(c['closing_section']['high_m']+.002)
    assert closure_plan(stock_jaw(),section_candidate(.10))['status']=='SO101_JAW_SECTION_LIMIT'


def test_partial_or_sparse_sections_preserve_uncertainty():
    c=section_candidate(.02,offset=.025)
    assert closure_plan(stock_jaw(),c)['status']=='WIDTH_UNRESOLVED'
    c['closing_section']=observed_closing_section(c,np.array([[.014,0.,.4]]))
    assert c['closing_section']['status']=='INSUFFICIENT_SECTION'
    assert next_gap(closure_plan(stock_jaw(),c),.02,'free')==('STOP_UNCONFIRMED',None)


def test_feedback_stops_before_pushing_and_cannot_close_past_the_width_floor():
    p=closure_plan(stock_jaw(),section_candidate(.02)); gap=p['visible_span_m']
    assert next_gap(p,gap,'rising')==('WAIT_AT_CURRENT_GAP',None)
    assert next_gap(p,gap,'persistent')==('CONTACT_CANDIDATE',None)
    assert next_gap(p,p['entry_gap_m'],'persistent')==('UNEXPECTED_RESISTANCE',None)
    assert next_gap(p,gap,'free',healthy=False)==('STOP_UNCONFIRMED',None)
    floor=p['min_search_gap_m']
    assert next_gap(p,floor+.0003,'free')==('CLOSE_STEP',floor)
    assert next_gap(p,floor,'free')==('STOP_NO_CONTACT',None)
    assert next_gap(p,floor-.0001,'persistent')==('STOP_OUTSIDE_PLAN',None)


def test_wider_entry_reaches_the_existing_path_checker_before_any_motion(monkeypatch):
    from home_robot_tasks import stationary_pick_sim as planning
    from tools.diagnostics import run_s3_stationary_lab as lab
    arm=lab.model(.61)
    review,group,_=lab.fixture(arm,lab.CASES[0],1)
    candidate=group['refined_grasps'][1]
    candidate['closing_section']=dict(status='VISIBLE_SPAN',low_m=-.016,high_m=.018)
    group['refined_grasps']=[candidate]
    checked=[]
    def inspect(arm,seed,waypoints,boxes,width,*args,**kwargs):
        checked.append(width)
        raise planning.Rejected('BLOCKED_WIDER_ENTRY')
    monkeypatch.setattr(planning,'check_stages',inspect)
    path=planning.plan_pick(arm,review,group,lab.SCENE)
    assert checked==pytest.approx([.040])
    assert path['status']=='NO_VIRTUAL_PATH' and not path['motion_authorized']


def test_virtual_empty_closure_stops_at_local_floor_before_the_probe():
    from tools.diagnostics import run_s3_stationary_lab as lab
    report=lab.run_case(lab.model(.61),lab.CASES[2],1,'empty_close',
                        candidate_source='yeoeun-surface',marker_length_m=.028)
    assert report['status']=='STOPPED_PICK'
    trace=report['trial']['jaw_closure']
    assert trace[-1]['action']=='STOP_NO_CONTACT'
    floor=report['plan']['attempts'][-1]['jaw_closure']['min_search_gap_m']
    assert trace[-1]['gap_m']==pytest.approx(floor)
    assert all(row['next_gap_m'] is None or row['next_gap_m']>=floor for row in trace)


def test_preselected_rigid_target_is_independent_of_contact_band(monkeypatch):
    from home_robot_tasks import jaw_closure
    candidate=section_candidate(.035)
    original=closure_plan(stock_jaw(),candidate)
    monkeypatch.setattr(jaw_closure,'CONTACT_UNCERTAINTY_M',.004)
    changed=closure_plan(stock_jaw(),candidate)
    assert changed['target_gap_m']==original['target_gap_m']==pytest.approx(original['visible_span_m']-.005)
    assert changed['upper_contact_gap_m']>original['upper_contact_gap_m']
    legacy=dict(original)
    del legacy['target_gap_m']
    assert next_gap(legacy,original['entry_gap_m'],'free')==next_gap(original,original['entry_gap_m'],'free')
    with pytest.raises(ValueError,match='inconsistent'):
        next_gap(dict(original,min_search_gap_m=.001),original['entry_gap_m'],'free')


def test_uncalibrated_candidate_is_skipped_before_ik_then_next_candidate_is_used():
    from tools.diagnostics import run_s3_stationary_lab as lab
    from home_robot_tasks.installed_jaw import InstalledLeftJaw
    from home_robot_tasks.stationary_pick_sim import plan_pick

    arm=lab.model(.61)
    measured=InstalledLeftJaw(lab.read(lab.CONFIG/'left_pad_gap.observed.json'),arm.profile)
    review,group,_=lab.fixture(arm,lab.CASES[0],1)
    missing=plan_pick(arm,dict(review,mode='offline_capture_review'),group,lab.SCENE)
    assert missing['status']=='LEFT_PAD_CALIBRATION_REQUIRED' and missing['attempts']==[]
    good=dict(group['refined_grasps'][1],closing_section=dict(status='VISIBLE_SPAN',low_m=-.010,high_m=.010))
    thin=dict(good,closing_section=dict(status='VISIBLE_SPAN',low_m=-.003,high_m=.003))
    group['refined_grasps']=[thin,good]
    result=plan_pick(arm,review,group,lab.SCENE,installed_left_jaw=measured)
    first=result['attempts'][0]
    assert first['reason']=='OUTSIDE_MEASURED_PAD_RANGE' and not first['stages']
    assert result['status']=='VIRTUAL_PATH_FOUND' and result['candidate_index']==1
    assert result['attempts'][1]['left_installed_pad_preview']['status']=='MEASURED_GAP_PREVIEW'


@pytest.mark.parametrize('fault,closing_decision', [('one_tip_contact','CONTACT_CANDIDATE'),
                                                  ('early_resistance','UNEXPECTED_RESISTANCE')])
def test_single_tip_and_early_resistance_cannot_authorize_a_test_lift(fault,closing_decision):
    from tools.diagnostics import run_s3_stationary_lab as lab
    report=lab.run_case(lab.model(.61),lab.CASES[0],1,fault,candidate_source='yeoeun-surface',marker_length_m=.028)
    assert report['status']=='STOPPED_PICK'
    trial=report['trial']
    assert trial['trace'][-1]['stage']=='pick_close'
    assert trial['jaw_closure'][-1]['action']==closing_decision
    target=report['plan']['attempts'][-1]['jaw_closure']['target_gap_m']
    assert all(row['target_gap_m']==target for row in trial['jaw_closure'])
    assert not trial['retained_at_end'] and not trial['test_lift']['verified']
