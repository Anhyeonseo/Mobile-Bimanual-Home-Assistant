"""Self-occlusion is expected after a fixed-scene capture, not an actuator fault."""
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from home_robot_tasks.carry_execution import CarrySession
from home_robot_tasks.carry_live_inputs import FixedPlatformGuard, LiveCarryInputs
from test_carry_execution import program, feedback
from test_carry_live_inputs import packet, platform, scene
from tools.diagnostics.carry_stream_sim import SimClock, simulate

POLICY='snapshot_then_verify'


def admitted_inputs():
    color,depth,mask,intr=scene();now=[10.04]
    guard=FixedPlatformGuard(.3,.00002);guard.update(platform(),10.03,10.031)
    inputs=LiveCarryInputs(program(),lambda q:np.eye(4),(color,depth),mask,intr,np.eye(4).tolist(),
        guard,clock=lambda:now[0],observation_policy=POLICY)
    inputs.add_joints(packet(),10.03,10.034)
    record={'color_time_s':10.025,'depth_time_s':10.026}
    images=(color,(depth*1000).astype(np.uint16),{'depth_scale_m':.001})
    inputs.add_frame(record,images)
    return inputs,now,record,images


def refresh_mechanics(inputs,now,seq):
    inputs.add_joints(packet(completed_pairs=seq),now-.01,now-.007)
    tick=1000+round((now-10.04)*1000)
    inputs.add_platform(platform(tick),now-.01,now-.009)


@pytest.mark.parametrize('camera',['absent','fully_occluded','other_object'])
def test_camera_is_not_a_motor_interlock_after_snapshot_admission(camera):
    inputs,now,record,images=admitted_inputs()
    assert inputs.tracker is None  # No textured-patch dependency for planning.
    original_scene_time=inputs.scene_time
    inputs.admit_snapshot()
    for seq,phase in enumerate(('open','pregrasp','approach','close','probe','lift','carry','hold'),start=11):
        now[0]+=.5
        refresh_mechanics(inputs,now[0],seq)
        inputs.set_phase(phase)
        if camera!='absent':
            color,depth,metadata=images
            color=np.zeros_like(color) if camera=='fully_occluded' else 255-color
            inputs.add_frame({'color_time_s':now[0]-.02,'depth_time_s':now[0]-.019},
                             (color,np.zeros_like(depth),metadata))
        f=inputs.feedback()
        assert inputs.validate_stage(None,None)
        assert f.observation_policy==POLICY and f.payload_relation=='unknown'
        assert f.vision_sequence==-1 and f.vision_s==-1
        assert f.scene_validated_s==original_scene_time  # Not restamped as live vision.
        assert inputs.failure is None


@pytest.mark.parametrize('defect',['missing_scene','stale_scene','changed_target'])
def test_snapshot_admission_cannot_bypass_a_missing_stale_or_changed_scene(defect):
    inputs,now,record,images=admitted_inputs()
    if defect=='missing_scene':inputs.validation=None
    elif defect=='stale_scene':
        now[0]+=.4;refresh_mechanics(inputs,now[0],11)
    else:
        color,depth,metadata=images;depth=depth.copy()
        area=inputs.mask if defect=='changed_target' else ~inputs.mask
        depth[area]+=20
        inputs.add_frame(record,(color,depth,metadata))
    with pytest.raises(ValueError):inputs.admit_snapshot()
    assert not inputs.admitted


@pytest.mark.parametrize('change',['depth_noise','removed_background','changed_background_rgb'])
def test_stationary_snapshot_keeps_planned_collision_scene_instead_of_background_equality(change):
    inputs,now,record,images=admitted_inputs()
    color,depth,metadata=images;color=color.copy();depth=depth.copy()
    if change=='depth_noise':depth[~inputs.mask]-=20
    elif change=='removed_background':depth[~inputs.mask]+=100
    else:color[~inputs.mask]=255-color[~inputs.mask]
    inputs.add_frame(record,(color,depth,metadata));inputs.admit_snapshot()
    assert inputs.admitted and inputs.failure is None
    assert inputs.validation['live_background_revalidated'] is False
    assert inputs.validation['environment_basis']=='native_checked_rgbd_planning_snapshot'
    assert inputs.validation['live_target_revalidated'] is True
    assert set(inputs.validation['regions'])=={'target'}


@pytest.mark.parametrize('defect',['stale_joints','stale_platform','base_moved','lift_moved'])
def test_snapshot_does_not_weaken_actual_robot_feedback_guards(defect):
    inputs,now,_,_=admitted_inputs();inputs.admit_snapshot();inputs.set_phase('pregrasp')
    now[0]+=.5
    if defect=='stale_joints':inputs.add_platform(platform(1500),10.53,10.531)
    elif defect=='stale_platform':inputs.add_joints(packet(completed_pairs=11),10.53,10.534)
    else:
        raw=(20,0,0,4095) if defect=='base_moved' else (0,0,0,50)
        with pytest.raises(ValueError,match='moved'):inputs.add_platform(platform(1500,raw),10.53,10.531)
        return
    with pytest.raises(ValueError,match='stale'):inputs.feedback()


def test_late_frame_from_before_admission_cannot_overwrite_running_scene(monkeypatch):
    inputs,_,record,images=admitted_inputs()
    def in_flight(*args,**kwargs):
        inputs.admit_snapshot()
        return {'unchanged':False}  # Arrived after admission; cannot latch error.
    monkeypatch.setattr(inputs.scene,'compare',in_flight)
    inputs.add_frame(record,images)
    assert inputs.admitted and inputs.failure is None
    assert inputs.feedback().scene_current


@pytest.mark.parametrize('continuous',[False,True])
def test_whole_route_without_visual_follow_retains_goals_but_never_claims_grasp_success(continuous):
    p=program();result=simulate(p,observation_policy=POLICY,continuous_observation=continuous)
    assert result['state']=='HOLDING' and result['holding_evidence']=='HOLDING_UNVERIFIED'
    assert result['simulated_stop_calls']==0 and result['simulated_enable_calls']==1
    assert [s['stage'] for s in result['stages']]==[s.name for s in p.stages]
    assert result['final_command_rad'][5]==p.close_control_rad
    assert result['physical_task_completed'] is False


@pytest.mark.parametrize('fault,state',[
    ('jaw_outside_envelope','FAILED'),('slip','FAILED'),('dispatch','FAILED'),('timeout','FAILED'),
    ('stale','FAILED'),('base_moved','FAILED'),('cancel','CANCELLED'),('stop_unconfirmed','STOPPING')])
def test_snapshot_still_rejects_mechanical_failures_and_operator_cancellation(fault,state):
    result=simulate(program(),fault=fault,observation_policy=POLICY,continuous_observation=True)
    assert result['state']==state and result['simulated_stop_calls']>0
    assert not result['physical_task_completed']


def test_empty_close_cannot_be_reported_as_a_success_from_goal_position_alone():
    result=simulate(program(),fault='empty_close',observation_policy=POLICY)
    assert result['state']=='HOLDING' and result['holding_evidence']=='HOLDING_UNVERIFIED'
    assert not result['physical_task_completed']


def test_policy_binding_prevents_accidental_switch_during_execution():
    p=program();clock=SimClock();f=feedback(p,clock,observation_policy=POLICY)
    stop=NS(request=lambda *a:None,poll=lambda *a:NS(status='STOPPED'))
    motion=NS(ready=lambda:True,maintain=lambda:None)
    session=CarrySession(p,motion,stop,lambda:f,clock,observation_policy=POLICY)
    session.start('bound')
    f=replace(f,observation_policy='continuous_patch')
    session.poll()
    assert session.state=='FAILED' and 'policy changed' in session.reason


def test_recorded_173ms_platform_sample_is_not_used_to_start_a_new_reader():
    inputs,now,_,_=admitted_inputs();inputs.admit_snapshot();inputs.set_phase('open')
    now[0]=10.1
    inputs.add_joints(packet(completed_pairs=11),10.09,10.094)
    inputs.add_platform(platform(1065,sample_age_ms=(103,68,173,138)),10.095,10.098)
    original_stamp=inputs.platform.stamp
    assert inputs.feedback().base_stopped  # Valid for observation at receipt.
    assert not inputs.route_start_ready()  # No headroom for a reader handoff.
    now[0]=10.13
    with pytest.raises(ValueError,match='platform sample stale'):
        inputs.feedback()  # Old 30 ms throttle would reach this boundary.
    assert inputs.platform.stamp==original_stamp  # Never renew cached data.
    now[0]=10.134
    inputs.add_joints(packet(completed_pairs=12),10.125,10.129)
    inputs.add_platform(platform(1100,sample_age_ms=(100,65,30,0)),10.130,10.133)
    assert inputs.route_start_ready()


def test_waiting_for_fresh_sweep_services_keepalive_without_start_or_torque_release():
    from home_robot_tasks.skill_ports import PortResult
    p=program();clock=SimClock();ready=[False];calls=[]
    f=lambda:feedback(p,clock,sequence=round(clock()*1000),observation_policy=POLICY)
    motion=NS(ready=lambda:True,maintain=lambda:calls.append('heartbeat'),
        start=lambda *a:calls.append('start'),poll=lambda *a:PortResult('RUNNING',''))
    stop=NS(request=lambda *a:calls.append('stop'))
    session=CarrySession(p,motion,stop,f,clock,observation_policy=POLICY,ready_for_route=lambda:ready[0])
    session.start('fresh-handoff')
    for _ in range(5):
        assert session.poll().reason=='WAIT_FRESH_PLATFORM';clock.advance(.02)
    assert calls==['heartbeat']*5 and session.state=='RUNNING'
    ready[0]=True;session.poll()
    assert calls.count('start')==1 and 'stop' not in calls


def test_periodic_hold_can_wait_for_read_handoff_without_releasing_held_goals():
    p=program();clock=SimClock();calls=[]
    f=lambda:feedback(p,clock,observation_policy=POLICY,positions_rad=p.stages[-1].points[0].positions_rad)
    motion=NS(ready=lambda:True,maintain=lambda:calls.append('heartbeat'),start=lambda *a:calls.append('start'))
    session=CarrySession(p,motion,NS(request=lambda *a:calls.append('stop')),f,clock,
                         observation_policy=POLICY,ready_for_route=lambda:False)
    session.start('held');session.state='HOLDING';session.active=False
    assert session.poll().reason=='WAIT_FRESH_PLATFORM'
    assert calls==['heartbeat'] and session.state=='HOLDING'


@pytest.mark.parametrize('phase_ms',[0,15,30,45,60,75,90,105,120,135])
def test_full_resident_route_and_periodic_hold_with_every_platform_scan_phase(phase_ms):
    result=simulate(program(),observation_policy=POLICY,continuous_observation=True,
                    ready_for_route=lambda now:(round(now*1000)+phase_ms)%140<20)
    assert result['state']=='HOLDING' and result['holding_evidence']=='HOLDING_UNVERIFIED'
    assert result['simulated_stop_calls']==0
    assert [x['stage'] for x in result['stages']]==[s.name for s in program().stages]
def test_idle_reader_cleanup_preserves_first_failure():
    from tools.run.stationary_pick import stop_idle_reads

    class FaultingTransport:
        def stationary_platform(self, operation):
            assert operation == 2
            raise RuntimeError('cleanup rejected')

        def stationary_runtime(self):
            return {'first_stop_source_line': 4853, 'stop_latched': True}

    transport = FaultingTransport()
    try:
        try:
            raise ValueError('original stale feedback')
        finally:
            stop_idle_reads(transport)
    except ValueError as exc:
        assert str(exc) == 'original stale feedback'
        assert exc.diagnostics['idle_reader_cleanup_error'] == 'cleanup rejected'
        assert exc.diagnostics['firmware_runtime']['stop_latched']
    else:
        raise AssertionError('original failure was swallowed')
    try:
        stop_idle_reads(transport)
    except RuntimeError as exc:
        assert str(exc) == 'cleanup rejected'
    else:
        raise AssertionError('standalone cleanup failure was swallowed')
