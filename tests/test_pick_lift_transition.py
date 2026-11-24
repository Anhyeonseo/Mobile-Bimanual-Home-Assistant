"""New station ordering and failures, with no hardware access."""
from copy import deepcopy
from dataclasses import asdict, replace
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from home_robot_tasks.pick_calibration import TOP_BOARD, RISE_BOARD, RISE_TOP_BASIS, top_model_height, rise_target_um
from home_robot_tasks.pick_lift_transition import planning_review, rise_to_top
from home_robot_tasks.pick_capture import make_evidence
from home_robot_tasks.pick_web_backend import WebPickBackend
from home_robot_tasks.pick_backend import PipelineBackend
from home_robot_tasks.carry_planning import PlanRejected, plan_pick_carry
from so101_arm_bridge.lift_wire import LiftStatus
from test_pick_pipeline import capture_fixture
from test_carry_planning import arm, candidate, FakePlanner


def arrival_fixture():
    profile,platform,history,camera,_=capture_fixture()
    target=rise_target_um(RISE_BOARD)
    platform.update(controller_board=list(RISE_BOARD),lift_reference_basis=RISE_TOP_BASIS)
    platform['lift'].update(target_um=target,height_um=target,position_raw=1906)
    platform['top_reference']=dict(basis=RISE_TOP_BASIS,operator_confirmed=False,arrival_verified=True,
        bottom_referenced=True,boot_id=4,lift_session=1,calibration_id=profile['calibration_id'],
        controller_board=list(RISE_BOARD),height_um=target,target_um=target,position_raw=1906)
    evidence=make_evidence(camera,history,platform,profile,now=20.02,requested_s=19.9,
                           session_id='one-session',anchor=(0.,)*12)
    joints=dict(evidence['model_joint_positions_si'],s2_lift_joint=0.)
    review=dict(session_id='one-session',lift_height_m=0.,model_joint_positions_si=joints,
                lift_reference_basis='same_session_supported_bottom_reference')
    source={'original.png':'unchanged'}
    prepared=dict(capture_id='bottom-1',source_sha256=source,review=review,group={'immutable':[1,2,3]},
        start_revalidation=dict(kind='observed_bottom_measured_top',capture_id='bottom-1',source_sha256=source,
            session_id='one-session',controller_boot_id=4,observation_time_s=10.,target_unchanged=True,
            lift_reference_basis=RISE_TOP_BASIS,arrival_evidence=evidence,
            arrival_model_joint_positions_si=evidence['model_joint_positions_si']))
    return profile,prepared


def test_bottom_pixels_and_candidates_are_not_relabelled_as_top_capture():
    profile,prepared=arrival_fixture();before=deepcopy(prepared)
    review=planning_review(prepared,profile)
    assert prepared==before
    assert prepared['review']['lift_height_m']==0.
    assert review['lift_height_m']==.630
    assert review['model_joint_positions_si']['s2_lift_joint']==.630
    assert prepared['start_revalidation']['arrival_evidence']['platform']['lift']['height_um']>640000


@pytest.mark.parametrize('fault',['old_frame','boot','session','anchor','unverified','origin'])
def test_arrival_cannot_hide_stale_or_changed_evidence(fault):
    profile,data=arrival_fixture();r=data['start_revalidation']
    if fault=='old_frame':r['observation_time_s']=30.
    if fault=='boot':r['controller_boot_id']=99
    if fault=='session':r['session_id']='other'
    if fault=='anchor':r['arrival_model_joint_positions_si']=dict(r['arrival_model_joint_positions_si'],s2_lift_joint=.5)
    if fault=='unverified':r['target_unchanged']=False
    if fault=='origin':r['arrival_evidence']['platform']['top_reference']['bottom_referenced']=False
    with pytest.raises(ValueError):planning_review(data,profile)


@pytest.mark.parametrize('failure',[None,'fault','wrong_arrival','cancel','boot'])
def test_rise_arrival_and_failures_keep_feedback_and_stop(failure,monkeypatch):
    profile,data=arrival_fixture();target=rise_target_um(RISE_BOARD)
    status=LiftStatus(**data['start_revalidation']['arrival_evidence']['platform']['lift'])
    clock=[0.];events=[]
    class Lift:
        mobile=NS(boot_id=4)
        active=False
        def status(self):
            if not self.active:return replace(status,height_um=0,target_um=0)
            if failure=='fault':return replace(status,fault=1)
            if failure=='boot':return replace(status,boot_id=99)
            if clock[0]<.08:return replace(status,state=3,flags=31,command_raw=30,height_um=300000)
            return replace(status,height_um=target+(5000 if failure=='wrong_arrival' else 0))
        def move(self,session,height):
            assert height==target;self.active=True;events.append('move')
        def keepalive(self):events.append('keepalive')
    lift=Lift()
    stop=NS(request=lambda:events.append('stop'),status=lambda:(NS(confirmed=True,flags=255),clock[0]))
    def check():
        if failure=='cancel' and lift.active:raise RuntimeError('cancelled')
    proof=dict(status='LIFT_PATH_CHECKED',calibration_id=profile['calibration_id'],target_um=target)
    kw=dict(check=check,clock=lambda:clock[0],sleep=lambda n:clock.__setitem__(0,clock[0]+n))
    if failure:
        with pytest.raises((ValueError,RuntimeError)):rise_to_top(lift,lift.status,stop,proof,profile,RISE_BOARD,**kw)
        assert 'stop' in events
    else:
        result=rise_to_top(lift,lift.status,stop,proof,profile,RISE_BOARD,**kw)
        assert result['final'].height_um==target and 'keepalive' in events and 'stop' not in events
    assert events.count('move')==1


def test_station_orders_lift_validation_rise_arrival_binding_then_arm_plan(tmp_path,monkeypatch):
    calls=[]
    backend=WebPickBackend(dict(mode='station'))
    hardware=NS(guard=NS(board=RISE_BOARD),rise_to_pick_height=lambda ctx,p:calls.append('rise'),
        capture=lambda ctx,**kw:(tmp_path/'arrival',None,{'actual':'top'}))
    backend.hardware=hardware
    def compute(ctx,method,**values):
        calls.append(method)
        if method=='lift_preflight':return {'report':str(tmp_path/'lift.json')}
        if method=='bind_arrival':
            assert values['snapshot']=='original-bottom'
            return 'top-prepared'
    backend._compute=compute
    def plan(self,ctx,value):
        calls.append('arm-plan');assert value['prepared']=='top-prepared'
        assert value['snapshot']=='original-bottom'
        return value
    monkeypatch.setattr(PipelineBackend,'plan',plan)
    ctx=NS(folder=tmp_path,progress=lambda *a:None,note=lambda *a:None,artifact=lambda *a:None,check=lambda:None)
    raised=backend.rise(ctx,dict(operation='observe-rise-pick',prepared='bottom-prepared',snapshot='original-bottom',detections='bottom-masks'))
    assert calls==['lift_preflight','rise','save','bind_arrival']
    backend.plan(ctx,raised)
    assert calls==['lift_preflight','rise','save','bind_arrival','arm-plan']
    assert backend.after_motion


def test_lift_preflight_failure_never_moves(monkeypatch):
    backend=WebPickBackend(dict(mode='station'))
    backend.hardware=NS(guard=NS(board=RISE_BOARD),rise_to_pick_height=lambda *a:pytest.fail('moved on failed proof'))
    def fail(*a,**kw):raise ValueError('collision')
    backend._compute=fail
    with pytest.raises(ValueError,match='collision'):
        backend.rise(NS(folder=Path('/tmp'),progress=lambda *a:None),
                     dict(operation='observe-rise-pick',prepared='bottom'))
    assert not backend.after_motion


def test_remaining_candidates_are_tried_only_after_first_batch_fails(arm):
    class Planner(FakePlanner):
        def connect(self,start,goal,c,*,attached,label):
            if c.candidate_id!='8':
                self.calls.append((c.candidate_id,label,attached));raise PlanRejected('blocked')
            return super().connect(start,goal,c,attached=attached,label=label)
    candidates=[candidate(arm,str(i)) for i in range(10)]
    planner=Planner()
    result=plan_pick_carry(arm,planner,arm.anchor,candidates,scene_id='scene-1',exhaust_candidates=True)
    assert result.status=='OFFLINE_ROUTE_FOUND' and result.candidate_id=='8'
    assert '9' not in {c[0] for c in planner.calls}


def test_same_tcp_section_width_maps_to_same_collision_gap():
    from tools.diagnostics.run_s3_stationary_lab import stock_jaw,ROOT
    from home_robot_tasks.installed_jaw import StockLeftJaw,InstalledLeftJaw
    from home_robot_tasks.s2_geometry import model_angle_from_raw
    config=ROOT/'config/home_assistant';read=lambda n:json.loads((config/n).read_text())
    profile=read('s2_coordinates.development.json');stock=stock_jaw()
    jaw=StockLeftJaw(stock,profile,read('arm_ranges.development.json')['arms']['left']['gripper'])
    measured=InstalledLeftJaw(read('left_pad_gap.observed.json'),profile)
    assert measured.gap_mm(2243)-jaw.gap_mm(2243)>4.
    for width in (43.265,47.265):
        raw=jaw.raw_at_least(width)
        angle=model_angle_from_raw(raw,profile['arms']['left']['alignment']['gripper'],'gripper')
        collision_gap=float(np.interp(angle,stock.angles,stock.gaps))*1000
        assert width<=collision_gap<width+.3


def test_installed_firmware_rejects_top_move_before_any_device_access():
    from home_robot_tasks.pick_lift_transition import require_top_move
    require_top_move(TOP_BOARD,610000)
    with pytest.raises(ValueError,match='펌웨어'):
        require_top_move(TOP_BOARD,643847)
    profile,_=arrival_fixture()
    proof=dict(status='LIFT_PATH_CHECKED',calibration_id=profile['calibration_id'],target_um=rise_target_um(TOP_BOARD))
    with pytest.raises(ValueError,match='펌웨어'):
        rise_to_top(None,lambda:pytest.fail('device opened'),None,proof,profile,TOP_BOARD)


def test_cartesian_branch_jump_tries_continuous_ik_without_disabling_collision(monkeypatch):
    import sys
    from home_robot_tasks.carry_moveit import MoveItCarryPlanner
    fake_srv=NS(GetCartesianPath=NS(Request=lambda:NS(header=NS())))
    monkeypatch.setitem(sys.modules,'moveit_msgs.srv',fake_srv)
    planner=MoveItCarryPlanner.__new__(MoveItCarryPlanner)
    planner.group='left_arm';planner.tcp='left_tcp'
    planner.prepare=lambda *a:None;planner.state=lambda *a:NS();planner.pose=lambda x:x
    planner.call=lambda *a:NS(error_code=NS(val=1),fraction=1.,solution=NS())
    def jump(*a):raise PlanRejected('large joint branch change; no encoder wrapping allowed')
    planner.checked_path=jump
    checked=[];planner.valid_many=lambda samples,*a:checked.append(samples)
    def fk(q):
        pose=np.eye(4);pose[0,3]=q[0];return pose,None
    planner.arm=NS(fk=fk)
    samples=np.zeros((2,5));samples[1,0]=.01
    monkeypatch.setattr('home_robot_tasks.carry_cartesian.bounded_cartesian',lambda *a:samples)
    goal=fk(samples[-1])[0]
    result=planner.cartesian(samples[0],goal,NS(),attached=False,label='approach')
    assert len(checked)==1 and np.array_equal(result.joints,samples)
    planner.valid_many=lambda *a:(_ for _ in ()).throw(PlanRejected('collision'))
    with pytest.raises(PlanRejected,match='collision'):
        planner.cartesian(samples[0],goal,NS(),attached=False,label='approach')


def test_encoder_target_is_independent_of_old_height_fit():
    from home_robot_tasks.pick_calibration import TOP_TRAVEL_COUNTS
    from home_robot_tasks.pick_lift_transition import require_top_move
    from so101_arm_bridge.unified_commissioning import pick_board_scale
    assert TOP_TRAVEL_COUNTS==30717
    assert rise_target_um(RISE_BOARD)==30717*87287//4096==654588
    assert pick_board_scale(*RISE_BOARD)==87287
    require_top_move(RISE_BOARD,rise_target_um(RISE_BOARD))
    profile,data=arrival_fixture()
    # A changed geometric travel estimate does not change the raw movement goal.
    profile['lift']['travel_m']=.64
    status=LiftStatus(**data['start_revalidation']['arrival_evidence']['platform']['lift'])
    ref=data['start_revalidation']['arrival_evidence']['platform']['top_reference']
    assert top_model_height(status,profile,RISE_BOARD,ref)==.630


def test_long_continuous_wrist_motion_is_not_a_branch_jump():
    from home_robot_tasks.carry_moveit import MoveItCarryPlanner
    planner=MoveItCarryPlanner.__new__(MoveItCarryPlanner)
    planner.names=['b','s','e','f','r'];checked=[]
    planner.valid_many=lambda samples,*a:checked.extend(samples)
    start=np.array([0.,0.,0.,0.,-1.57]);goal=start.copy();goal[-1]=1.70
    def trajectory(rows):
        return NS(joint_trajectory=NS(joint_names=planner.names,
            points=[NS(positions=q,time_from_start=NS(sec=1,nanosec=0)) for q in rows]))
    path=planner.checked_path(trajectory(np.linspace(start,goal,24)),start,NS(),False,'pregrasp','test')
    assert checked and path.end[-1]==pytest.approx(1.70)
    with pytest.raises(PlanRejected,match='branch'):
        planner.checked_path(trajectory([start,goal]),start,NS(),False,'pregrasp','test')


@pytest.mark.parametrize('board',[(0x2492E,3399362976),(0x2492F,3399362976),(0x24930,3399362976)])
def test_transport_patch_keeps_measured_lift_scale_and_target(board):
    from home_robot_tasks.pick_calibration import top_reference_um
    from home_robot_tasks.pick_lift_transition import require_top_move
    from so101_arm_bridge.unified_commissioning import check_pick_calibration
    profile,_=arrival_fixture()
    assert check_pick_calibration(*board,profile)==87287
    assert top_reference_um(profile,board)==654588
    require_top_move(board,654588)
    with pytest.raises(ValueError):require_top_move(board,654589)
    with pytest.raises(ValueError):check_pick_calibration(board[0],2696088964,profile) if board[0] in (0x2492F,0x24930) else check_pick_calibration(board[0],1,profile)


def test_fixed_bottom_binding_keeps_geometry_and_rejects_changed_controller():
    profile,data=arrival_fixture();original=deepcopy(data['review'])
    record=data['start_revalidation'];record['target_unchanged']=None
    record['target_assumption']='fixed_bottom_observation';record['target_check']=None
    result=planning_review(data,profile)
    assert result['lift_height_m']==.630 and data['review']==original
    record['controller_boot_id']=999
    with pytest.raises(ValueError):planning_review(data,profile)
