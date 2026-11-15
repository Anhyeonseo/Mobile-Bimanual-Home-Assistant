import math
import time
from dataclasses import replace
from threading import Event
from types import SimpleNamespace

import numpy as np
import pytest

from home_robot_tasks.assistant.contracts import Context, Stamp
from home_robot_tasks.assistant.alignment_contracts import TaskScene, AlignmentSelection, HeldObject
from home_robot_tasks.assistant.prealignment_binding import PrealignmentPolicy
from home_robot_tasks.assistant.object_alignment import AlignmentPolicy, FixedHeightAlignmentPlanner
from home_robot_tasks.assistant.placement import Check
from home_robot_tasks.assistant.view_geometry import view_check
from home_robot_tasks.alignment_navigation import alignment_path
from home_robot_tasks.place_geometry import SupportGrid, place_candidates
from home_robot_tasks.async_resolution import AsyncResolvedPort
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.skill_ports import PortResult
from home_robot_tasks.stationary_lift import LiftHeightRule
from tools.diagnostics.run_s3_stationary_lab import model


def scene(now=1.,capture='before',context=None,**kw):
    x,y=np.meshgrid(np.linspace(.92,1.08,20),np.linspace(.48,.52,12))
    points=np.c_[x.ravel(),y.ravel(),np.full(x.size,.82)]
    context=context or Context('map','v1','cal','boot')
    args=dict(object_id='object',surface_id='table',stamp=Stamp(now,context,capture),scene_revision='s1',
              model_revision='model1',pose_revision='pose1',base_xy_yaw=(0.,0.,0.),lift_m=.5,
              points_map_m=points,support_z_m=.8,source_sha256={'rgb':'a'*64,'depth':'b'*64})
    return TaskScene(**{**args,**kw})


def held(now=1.,**kw):
    args=dict(object_id='object',grasp_id='verified',context=Context('map','v1','cal','boot'),
              model_revision='model1',observed_s=now,tcp_from_object=np.eye(4),size_m=(.16,.04,.02),translation_uncertainty_m=.002)
    return HeldObject(**{**args,**kw})


def grid(empty=None,now=1.):
    pose=np.eye(4);pose[2,3]=.8
    return SupportGrid(Stamp(now,Context('map','v1','cal','boot'),'dest'),'desk','s1',pose,(0.,0.),.01,
                       np.ones((60,60),bool) if empty is None else empty)


@pytest.mark.parametrize('bad',['old_id','old_time','map','boot','model','pose','height'])
def test_recapture_requires_actual_post_alignment_exposure(bad):
    before=scene(); selected=AlignmentSelection(before,dict(base_xy_yaw=[.1,0,0],lift_travel_m=.5),1,60,aligned_s=2)
    fresh=scene(2.1,'after',base_xy_yaw=(.1,0.,0.))
    if bad=='old_id':fresh=replace(fresh,stamp=Stamp(2.1,fresh.stamp.context,'before'))
    if bad=='old_time':fresh=replace(fresh,stamp=Stamp(1.9,fresh.stamp.context,'after'))
    if bad in ('map','boot'):
        ctx=replace(fresh.stamp.context,**({'map_revision':'v2'} if bad=='map' else {'clock_id':'reboot'}))
        fresh=replace(fresh,stamp=Stamp(2.1,ctx,'after'))
    if bad=='model':fresh=replace(fresh,model_revision='m2')
    if bad=='pose':fresh=replace(fresh,base_xy_yaw=(.3,0.,0.))
    if bad=='height':fresh=replace(fresh,lift_m=.52)
    with pytest.raises(InvalidTask):selected.check_capture(fresh,now=2.2,context=before.stamp.context)


def test_recapture_accepts_new_exposure_without_renaming_old_capture():
    a=scene(); selected=AlignmentSelection(a,dict(base_xy_yaw=[.1,0,0],lift_travel_m=.5),1,60,2)
    selected.check_capture(scene(2.1,'after',base_xy_yaw=(.1,0.,0.)),now=2.2,context=a.stamp.context)


def test_fixed_lookup_and_stl_section_reference_real_so101():
    arm=model(0);ctx=Context('map','v1',arm.profile['calibration_id'],'boot')
    p=FixedHeightAlignmentPlanner({'left':arm},AlignmentPolicy(joint_samples=512,arm_samples=8,maximum_regions=4))
    observed=scene(context=ctx)
    result=p.plan_fixed(observed,current_xy_yaw=[.5,.5,0.],current_lift_m=.5,
        height_rule=LiftHeightRule.from_profile(arm.profile),now_s=1,
        screen=lambda c:{k:Check(True,'fixture',mode='simulation') for k in ('view','configuration','navigation')},maximum_screened=8)
    assert result['candidates']
    assert all(c['lift_travel_m']==.5 for c in result['candidates'])
    for c in result['candidates']:
        actual=model(c['lift_travel_m']);pose=actual.fk(c['fk_witness_model_rad'])[0]@actual.tcp_from_stock
        center=pose[:3,3]-pose[:3,0]*actual.jaw.center_from_fixed_m(c['observed_region']['observed_width_m'])
        x,y,yaw=c['base_xy_yaw'];r=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        np.testing.assert_allclose(r@center[:2]+[x,y],c['observed_region']['anchor_map_m'][:2],atol=1e-7)
        assert abs(center[2]-arm.profile['ground_z_m']-c['observed_region']['anchor_map_m'][2])<.002


def test_swept_alignment_avoids_obstacle_even_with_clear_endpoints():
    def free(p):return not (.10<p[0]<.20 and abs(p[1])<.08)
    path=alignment_path([0,0,0],[.3,0,0],free,radius_m=.6)
    assert path and any(abs(p[1])>=.08 for p in path['poses_xy_yaw'])
    for p in path['poses_xy_yaw']:assert free(p)
    assert alignment_path([0,0,0],[.3,0,0],lambda p:p[0]<.1,radius_m=.4,maximum_expansions=50) is None


def test_rotation_checks_body_sweep_not_just_position():
    path=alignment_path([0,0,0],[0,0,math.pi/2],lambda p:abs(p[2])<.5,maximum_expansions=30)
    assert path is None


def test_view_uses_projection_and_obstacle_depth():
    points=np.c_[np.linspace(-.02,.02,30),np.zeros(30),np.ones(30)]
    intr=dict(width=640,height=480,fx=400,fy=400,ppx=320,ppy=240)
    assert view_check(points,np.eye(4),intr).valid
    assert not view_check(points,np.eye(4),intr,obstacles_map_m=points*.5).valid
    points[:,0]+=2
    assert not view_check(points,np.eye(4),intr).valid


def test_place_uses_measured_attachment_and_occupied_cells():
    empty=np.ones((60,60),bool);empty[20:40,20:40]=False
    attachment=np.eye(4);attachment[:3,3]=[.06,-.02,.01]
    load=held(tcp_from_object=attachment)
    choices=place_candidates(grid(empty),load,now_s=1.1)
    assert choices
    for p in choices:
        np.testing.assert_allclose(p['map_from_tcp']@attachment,p['map_from_object'])
        x,y=p['map_from_object'][:2,3]
        assert not (.2<x<.4 and .2<y<.4)
    assert not place_candidates(grid(np.zeros((60,60),bool)),load,now_s=1.1)
    with pytest.raises(InvalidTask):place_candidates(grid(),held(now=0),now_s=1.1)


class Port:
    def __init__(self):self.starts=[];self.state='RUNNING'
    def ready(self):return True
    def start(self,g,p):self.starts.append((g,p))
    def poll(self,g):return PortResult(self.state)
    def cancel(self,g):self.state='CANCELLED'
    def phase_safe(self,g):return True


def async_fixture():
    now=[1.];release=Event();started=Event();ports=Port();epoch=['first'];commits=[]
    def compute(r):started.set();release.wait(5);return {'ok':1}
    policy=SimpleNamespace(snapshot=lambda *a:epoch[0],compute=compute,
        check_pending=lambda r,*a:None if r==epoch[0] else (_ for _ in ()).throw(InvalidTask('epoch changed')),
        commit=lambda *a:commits.append(a) or {'value':1})
    p=AsyncResolvedPort('pick',ports,policy,SimpleNamespace(resolve=None),lambda:now[0],timeout_s=2)
    p.start('g',{});assert started.wait(1)
    return p,ports,now,release,epoch,commits


@pytest.mark.parametrize('reason',['cancel','timeout','epoch'])
def test_late_background_result_never_dispatches_after_cancel_or_context_change(reason):
    p,port,now,event,epoch,commits=async_fixture()
    try:
        start=time.monotonic();assert p.poll('g').state=='RUNNING';assert time.monotonic()-start<.05
        if reason=='cancel':p.cancel('g')
        elif reason=='timeout':now[0]=4;p.poll('g')
        else:epoch[0]='reboot';p.poll('g')
        assert not p.ready()  # worker has not returned; no queued replacement
        event.set();p.pending.result(2)
        assert p.poll('g').state in ('CANCELLED','FAILED')
        assert not port.starts and not commits
    finally:event.set();p.close()


def test_background_success_commits_and_dispatches_on_poll_only():
    p,port,now,event,epoch,commits=async_fixture()
    try:
        event.set();p.pending.result(2)
        assert not port.starts and not commits
        assert p.poll('g').state=='RUNNING'
        assert len(port.starts)==len(commits)==1
        port.state='SUCCEEDED';assert p.poll('g').state=='SUCCEEDED'
    finally:p.close()


def test_policy_alignment_then_new_capture_only_then_fine_route():
    from home_robot_tasks.task_route_program import TaskRouteProgram
    now=[1.];pose=[0.,0.,0.];height=[.5];current=[scene()];events=[]
    catalog=SimpleNamespace(map=SimpleNamespace(map_id='map',revision='v1'),invalid_places={},target={},surface={})
    catalog.checked_target=lambda placement:dict(capture_id=current[0].stamp.capture_id,observed_s=current[0].stamp.observed_s)
    candidate=dict(base_xy_yaw=[.1,0.,0.],lift_travel_m=.5,arm='left')
    def align(req):events.append('alignment_without_grasp');return dict(status='SCREENED_PROPOSALS',candidates=[candidate])
    def fine(req):
        events.append('fine_'+req.scene.stamp.capture_id)
        return TaskRouteProgram('pick','object',req.scene.stamp.capture_id,'cal','model1',.5,(),None)
    policy=PrealignmentPolicy(scene_for=lambda *a:current[0],configuration=lambda:(pose[:],height[0],now[0],True),
        epoch=lambda:('boot','v1','s1','model1'),prepare=lambda *a:{'transport':{'arm':{},'height_um':500000}},
        alignment_planner=align,route_planner=fine,navigation_plan=lambda *a:{'goal':{'x':.1,'y':0,'yaw_rad':0}},
        loaded_transport=lambda *a:{'arm':{},'height_um':500000})
    parameters={'object_id':'object'}
    request=policy.snapshot('align_for_pick',parameters,catalog,now[0])
    nav=policy.commit(request,policy.compute(request),catalog,now[0])
    assert events==['alignment_without_grasp'] and nav['goal']['x']==.1
    with pytest.raises(InvalidTask,match='not reached'):policy.resolve('adjust_lift_for_pick',parameters,catalog,now[0])
    pose[0]=.1;now[0]=2
    assert policy.resolve('adjust_lift_for_pick',parameters,catalog,now[0])=={'height_um':500000}
    policy.before_capture('reobserve_target',parameters,catalog,now[0])
    with pytest.raises(InvalidTask,match='new exposure'):policy.snapshot('pick',parameters,catalog,now[0])
    now[0]=2.1;current[0]=scene(now[0],'fresh',base_xy_yaw=tuple(pose))
    catalog.target=catalog.checked_target(False)
    request=policy.snapshot('pick',parameters,catalog,now[0])
    answer=policy.commit(request,policy.compute(request),catalog,now[0])
    assert answer['program'].capture_id=='fresh' and events==['alignment_without_grasp','fine_fresh']


def route_program(operation):
    from home_robot_tasks.task_route_program import TaskRouteProgram
    from home_robot_tasks.carry_program import CarryStage
    from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
    from home_robot_tasks.arm_routes import ContactEnvelope
    names=(('open','pregrasp','approach','close','probe','lift','carry') if operation=='pick'
        else ('place_preplace','place_lower','release','place_retreat','empty_transport'))
    stages=tuple(CarryStage(n,(TimedJointPoint(50,(0.,)*12),TimedJointPoint(100,(0.,)*12)),{},
                    n in ('probe','lift','carry','place_preplace','place_lower')) for n in names)
    return TaskRouteProgram(operation,'object','fresh','cal','model1',.5,stages,None,ContactEnvelope(5,-.1,.1,0.,'object','fresh'))


@pytest.mark.parametrize('operation', ['pick','place'])
def test_full_route_stages_require_independent_grasp_and_release(operation):
    from home_robot_tasks.task_route_program import TaskRoutePort
    from home_robot_tasks.execution import Feedback
    from home_robot_tasks.manipulation_port import PlanningContext
    now=[1.];arm=Port();arm.state='SUCCEEDED';scene_port=Port();scene_port.state='SUCCEEDED'
    evidence={k:True for k in ('hardware_ready','base_stopped','lift_stopped','gripper_empty','gripper_open',
        'contact_plausible','probe_contact_live','probe_clearance_verified','probe_follow_verified','grasp_verified',
        'load_retained','object_attached','placement_support_verified','release_verified','object_detached','arm_clear','arm_in_transport_pose')}
    missing='probe_follow_verified' if operation=='pick' else 'release_verified';evidence[missing]=False
    port=TaskRoutePort(arm,scene_port,context_for=lambda *a:PlanningContext(now[0],'fresh','pose','s1','cal','base_link',(0.,)*12,True,True),
        scene_command=lambda action,p,t:dict(operation=action,object_id=p.object_id),
        monitor=lambda g,p,t:Feedback(g,t,'RUNNING',dict(evidence)),clock=lambda:now[0])
    port.start('job',{'program':route_program(operation)})
    for _ in range(22):now[0]+=.05;assert port.poll('job').state=='RUNNING'
    assert not scene_port.starts  # trajectory success alone cannot attach/detach
    evidence[missing]=True
    for _ in range(30):
        now[0]+=.05
        if port.poll('job').state=='SUCCEEDED':break
    assert port.poll('job').state=='SUCCEEDED'
    assert [v[1]['operation'] for v in scene_port.starts]==['attach' if operation=='pick' else 'detach']
    history=port.sequence.entries['job']['history']
    assert history[-1]['phase']==('pick_carry' if operation=='pick' else 'place_empty_transport')


def test_measured_contact_keeps_pressure_without_faking_joint_anchor():
    from home_robot_tasks.arm_routes import ResidentRouteBuilder,ContactEnvelope
    from home_robot_tasks.manipulation_port import PlanningContext
    from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
    measured=(0.,)*5+(-.24,)+(0.,)*6
    commanded=(0.,)*5+(-.2,)+(0.,)*6
    params=dict(trajectory=(TimedJointPoint(50,commanded),TimedJointPoint(100,commanded)),
                context=PlanningContext(1,'capture','pose','scene','cal','base',measured,True,True),
                contact_envelope=ContactEnvelope(5,-.28,-.2,-.2,'object','capture'))
    args=dict(minimum_rad=(-2.,)*12,maximum_rad=(2.,)*12,max_speed_rad_s=(1.,)*12,
        max_acceleration_rad_s2=(1.,)*12,max_jerk_rad_s3=(1.,)*12,validate_route=lambda *a:True)
    snapshot=SimpleNamespace(positions_urad=tuple(round(v*1e6) for v in measured))
    with pytest.raises(InvalidTask,match='live gripper'):ResidentRouteBuilder(**args)(params,snapshot)
    route=ResidentRouteBuilder(**args,contact_guard=lambda e,s:True)(params,snapshot)
    assert all(p.positions_rad[5]==-.2 for p in route)
    bad=dict(params,trajectory=(TimedJointPoint(50,commanded),TimedJointPoint(100,measured)))
    with pytest.raises(InvalidTask,match='constant'):ResidentRouteBuilder(**args,contact_guard=lambda *a:True)(bad,snapshot)


def test_pick_replans_loaded_tail_only_after_independent_probe_and_attachment():
    from home_robot_tasks.task_route_program import TaskRoutePort
    from home_robot_tasks.execution import Feedback
    from home_robot_tasks.manipulation_port import PlanningContext
    now=[1.];arm=Port();arm.state='SUCCEEDED';scene_port=Port();scene_port.state='SUCCEEDED';tail=Port();tail.state='SUCCEEDED'
    evidence={k:True for k in ('hardware_ready','base_stopped','lift_stopped','gripper_empty','gripper_open',
        'contact_plausible','probe_contact_live','probe_clearance_verified','probe_follow_verified','grasp_verified',
        'load_retained','object_attached','arm_clear','arm_in_transport_pose')}
    evidence['object_attached']=False
    p=TaskRoutePort(arm,scene_port,loaded_tail=tail,clock=lambda:now[0],
        context_for=lambda *a:PlanningContext(now[0],'fresh','pose','s1','cal','base',(0.,)*12,True,True),
        scene_command=lambda *a:{'operation':'attach'},monitor=lambda g,n,t:Feedback(g,t,'RUNNING',dict(evidence)))
    p.start('job',{'program':route_program('pick')})
    for _ in range(20):now[0]+=.05;p.poll('job')
    assert scene_port.starts and not tail.starts
    evidence['object_attached']=True
    for _ in range(10):now[0]+=.05;p.poll('job')
    assert p.poll('job').state=='SUCCEEDED' and len(tail.starts)==1
    assert len(arm.starts)==5  # open/pregrasp/approach/close/probe, never stale lift/carry
    assert tail.starts[0][1]['grasp_program'].capture_id=='fresh'


def test_planning_budget_does_not_require_its_own_future_collision_result():
    import json
    from pathlib import Path
    from home_robot_tasks.fetch import plan_fetch, FetchRequest
    root=Path(__file__).resolve().parents[1]
    p=plan_fetch(FetchRequest('remote_control','living_room','bedroom_drop_zone'),json.loads((root/'config/home.example.json').read_text()),planning_in_skill=True)
    pick=next(s for s in p['steps'] if s['skill']=='pick')
    assert 'collision_checked' not in pick['start_requires'] and pick['timeout_s']==300
    assert 'fresh_target_pose' in pick['start_requires']


def test_cancelled_worker_blocks_other_skill_until_result_is_quarantined():
    from home_robot_tasks.async_resolution import PlanningWorker
    pool=PlanningWorker();event=Event()
    policy=SimpleNamespace(snapshot=lambda *a:None,compute=lambda r:event.wait(2) or {},
                           check_pending=lambda *a:None,commit=lambda *a:{})
    catalog=SimpleNamespace(resolve=None)
    a=AsyncResolvedPort('pick',Port(),policy,catalog,lambda:1.,worker=pool)
    b=AsyncResolvedPort('place',Port(),policy,catalog,lambda:1.,worker=pool)
    try:
        a.start('a',{});a.cancel('a')
        assert not b.ready()
        event.set();a.pending.result(2)
        assert b.ready() and not a.port.starts
    finally:event.set();a.close();b.close();pool.close()


def test_supported_release_allows_contact_offset_but_not_arm_motion_or_resqueeze():
    from home_robot_tasks.arm_routes import ResidentRouteBuilder,ContactEnvelope
    from home_robot_tasks.manipulation_port import PlanningContext
    from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
    measured=(0.,)*5+(-.24,)+(0.,)*6
    hold=(0.,)*5+(-.2,)+(0.,)*6
    opened=(0.,)*5+(-.5,)+(0.,)*6
    args=dict(minimum_rad=(-2.,)*12,maximum_rad=(2.,)*12,max_speed_rad_s=(1.,)*12,
        max_acceleration_rad_s2=(1.,)*12,max_jerk_rad_s3=(1.,)*12,validate_route=lambda *a:True,
        contact_guard=lambda e,s:e.release_goal_rad==-.5)
    snap=SimpleNamespace(positions_urad=tuple(round(v*1e6) for v in measured))
    params=dict(trajectory=(TimedJointPoint(50,hold),TimedJointPoint(550,opened)),
        context=PlanningContext(1,'c','p','s','cal','base',measured,True,True),
        contact_envelope=ContactEnvelope(5,-.28,-.2,-.2,'object','c',-.5))
    assert ResidentRouteBuilder(**args)(params,snap)[-1].positions_rad[5]==-.5
    moved=list(opened);moved[0]=.01
    params['trajectory']=(TimedJointPoint(50,hold),TimedJointPoint(550,tuple(moved)))
    with pytest.raises(InvalidTask,match='only open'):ResidentRouteBuilder(**args)(params,snap)


def test_dynamic_loaded_transport_uses_measured_joint_and_gap_not_fixed_home():
    from home_robot_tasks.offline_stack import OfflineStack
    import json
    from pathlib import Path
    from home_robot_tasks.carry_program import CarryStage
    from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
    from home_robot_tasks.arm_routes import ContactEnvelope
    root=Path(__file__).resolve().parents[1]
    read=lambda n:json.loads((root/'config'/n).read_text())
    plant=OfflineStack(read('fetch_stack.simulation.json'),read('home.example.json'),read('navigation_map.simulation.json'))
    q=(.1,)*5+(-.2,)+(0.,)*6
    stage=CarryStage('carry',(TimedJointPoint(50,q),TimedJointPoint(100,q)),{},True)
    program=replace(route_program('pick'),operation='loaded',stages=(stage,),contact_envelope=ContactEnvelope(5,-.28,-.2,-.2,'object','fresh'))
    plant.state.bind_transport_program(program)
    assert not plant.state.conditions()['arm_in_transport_pose']
    actual=(.1,)*5+(-.24,)+(0.,)*6
    plant.state.latest=replace(plant.state.latest,joints_rad=actual,lift_um=500000)
    c=plant.state.conditions()
    assert c['arm_in_transport_pose'] and c['lift_in_transport_position']
    plant.state.reset_transport_target()
    assert not plant.state.conditions()['arm_in_transport_pose']


def test_geometry_evidence_cannot_override_payload_or_allow_unobserved_release():
    from home_robot_tasks.mobile_manipulation import GeometryEvidence,contact_guard
    from home_robot_tasks.arm_routes import ContactEnvelope
    good=GeometryEvidence('object',1.,'s1',True,False,True,True,False)
    payload=SimpleNamespace(object_id='object',proof=lambda t:SimpleNamespace(observed_s=1.,conditions={'load_retained':True}))
    check=contact_guard(payload,lambda *a:good,lambda:'s1',lambda:1.1)
    envelope=ContactEnvelope(5,-.28,-.2,-.2,'object','capture')
    assert check(envelope,None)
    assert not check(replace(envelope,release_goal_rad=-.5),None)
    good=replace(good,placement_support_verified=True)
    assert check(replace(envelope,release_goal_rad=-.5),None)
    with pytest.raises(InvalidTask):replace(good,observed_s=0.).check('object','s1',1.1)
    with pytest.raises(InvalidTask):replace(good,detached=True).check('object','s1',1.1)


@pytest.mark.parametrize('kind',['long','round','box','thin'])
@pytest.mark.parametrize('yaw',[0.,.7,1.5])
def test_fixed_height_alignment_is_not_specific_to_remote_or_world_heading(kind,yaw):
    arm=model(0);ctx=Context('map','v1',arm.profile['calibration_id'],'boot')
    if kind=='round':
        a,r=np.meshgrid(np.linspace(0,2*np.pi,48),np.linspace(.004,.025,10))
        xy=np.c_[(r*np.cos(a)).ravel(),(r*np.sin(a)).ravel()]
    else:
        length,width={'long':(.16,.04),'box':(.04,.04),'thin':(.2,.02)}[kind]
        x,y=np.meshgrid(np.linspace(-length/2,length/2,25),np.linspace(-width/2,width/2,15));xy=np.c_[x.ravel(),y.ravel()]
    r=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    points=np.c_[xy@r.T+[1.,.5],np.full(len(xy),.82)]
    p=FixedHeightAlignmentPlanner({'left':arm},AlignmentPolicy(joint_samples=512,arm_samples=8,maximum_regions=4))
    result=p.plan_fixed(scene(context=ctx,points_map_m=points),current_xy_yaw=[.5,.5,0.],current_lift_m=.5,
        height_rule=LiftHeightRule.from_profile(arm.profile),now_s=1.,maximum_screened=4,
        screen=lambda c:{k:Check(True,'synthetic geometry only',mode='simulation') for k in ('navigation','configuration','view')})
    assert result['status']=='SCREENED_PROPOSALS'
    # New entry-path screening may require an existing bounded height alternative.
    assert all(c['height_lookup_m']==.5 and abs(c['lift_travel_m']-.5)<=.0400001 for c in result['candidates'])
    assert all(c['entry_review']['accepted'] and c['contact_direction']['accepted'] for c in result['candidates'])
    if kind in ('long','thin'):
        assert all(c['contact_direction']['approach_parallel_error_deg']<=10 for c in result['candidates'])
    assert not result['fine_grasp_generated']


def test_navigation_and_alignment_share_one_action_owner():
    from home_robot_tasks.nav2_port import Nav2MotionPort
    pose,path=Port(),Port();mux=Nav2MotionPort(pose,path)
    mux.start('transit',{'goal':{}})
    assert pose.starts and not path.starts and not mux.ready()
    with pytest.raises(RuntimeError):mux.start('overlap',{'poses_xy_yaw':[]})
    pose.state='SUCCEEDED';assert mux.poll('transit').state=='SUCCEEDED'
    mux.start('align',{'poses_xy_yaw':[[0,0,0],[.1,0,0]]})
    mux.cancel('align');assert mux.poll('align').state=='CANCELLED'
    assert len(pose.starts)==len(path.starts)==1


def test_stored_target_alignment_does_not_require_goal_camera_visibility():
    from home_robot_tasks.alignment_navigation import AlignmentScreen
    def make(strategy, free=True):
        return AlignmentScreen(start=[0.,0.,0.],pose_free=lambda p,c:free,
            configuration_check=lambda c:Check(free,'body',mode='simulation'),
            view_check=lambda c:Check(False,'outside_camera',cost=1.,mode='simulation'),
            observation_strategy=strategy,mode='simulation')
    c={'base_xy_yaw':[.1,0.,0.]}
    assert not make('recapture_at_goal')(c)['navigation'].valid
    c={'base_xy_yaw':[.1,0.,0.]}
    checks=make('stored_target')(c)
    assert checks['navigation'].valid and checks['view'].valid
    assert not c['projected_view']['valid'] and c['requires_target_tracking']
    assert c['observation_strategy']=='stored_target'
    assert not make('stored_target',False)({'base_xy_yaw':[.1,0.,0.]})['navigation'].valid
    with pytest.raises(InvalidTask):make('disable_checks')


def test_remembered_target_transforms_actual_motion_without_new_exposure():
    a=scene();motion=np.eye(4);motion[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];motion[:3,3]=[.1,.2,0]
    got=a.points_after_relative_motion(motion,ground_z_m=-.07)
    observed=a.points_map_m-[0,0,.07]
    expected=(observed-motion[:3,3])@motion[:3,:3]
    np.testing.assert_allclose(got,expected)
    assert a.stamp.capture_id=='before' and a.stamp.observed_s==1.
    # Changing arbitrary map coordinates must not change a relative target.
    theta=.7;c,s=np.cos(theta),np.sin(theta);R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    shifted=replace(a,base_xy_yaw=(2.,3.,theta),points_map_m=a.points_map_m@R.T+[2,3,0])
    np.testing.assert_allclose(shifted.points_after_relative_motion(motion,ground_z_m=-.07),got,atol=1e-12)
