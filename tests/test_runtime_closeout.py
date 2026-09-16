"""Regression of real integration failures; sensors/physics stay synthetic."""
from dataclasses import replace
import json
from pathlib import Path
import pytest
from home_robot_tasks.offline_stack import OfflineStack
from home_robot_tasks.payload_monitor import PayloadObservation

ROOT=Path(__file__).resolve().parents[1]
def read(name):return json.loads((ROOT/'config'/name).read_text())
def stack():return OfflineStack(read('fetch_stack.simulation.json'),read('home.example.json'),read('navigation_map.simulation.json'),guarded_navigation=True)
def request():return dict(schema_version=1,operation='fetch_object',request_id='integration',request=read('fetch_remote.example.json'))
def reach(s,skill):
    s.runtime.submit(request());task=s.runtime.app.tasks['integration']
    for _ in range(1600):
        s.tick()
        if task.state=='RUNNING' and task.step['skill']==skill:return task
        assert task.state not in task.TERMINAL,task.result()
    pytest.fail('phase not reached')
def finish(s,task):
    for _ in range(1600):
        s.tick()
        if task.state in task.TERMINAL or task.state=='STOP_UNCONFIRMED':return task.result()
    pytest.fail(str(task.result()))


def test_full_fetch_with_measured_wheel_odometry_posture_reset_and_guard():
    s=stack();r=s.run(request());n=s.guarded_navigation
    assert r['status']=='SUCCEEDED' and r['completed_steps']==15 and r['stop_requests']==0
    assert len(n.resets)==5 and len(set(epoch for _,epoch,_ in n.resets))==5
    assert n.commands and all(height==10000 and rates[3]==0 for _,rates,height,_ in n.commands)
    assert all(height==10000 for _,_,height in n.resets)
    assert n.driver.owner is None and not n.driver.fault
    assert s.xy_yaw[:2]==pytest.approx((6,4),abs=.03)


@pytest.mark.parametrize('missing',['depth','clearance'])
def test_missing_post_transition_observation_times_out_without_motion(missing):
    s=stack();n=s.guarded_navigation
    setattr(n,missing+'_enabled',False)
    r=s.run(request())
    assert r['status']=='FAILED' and 'travel observation timeout' in r['reason']
    assert not n.starts and not n.commands and s.stop.requests==1


def test_temporary_depth_gap_before_dispatch_can_reobserve_without_rearming():
    s=stack();n=s.guarded_navigation;n.depth_enabled=False
    task=reach(s,'navigate')
    for _ in range(8):s.tick()
    assert not n.starts and not n.commands and n.driver.owner is None
    n.depth_enabled=True
    assert finish(s,task)['status']=='SUCCEEDED'
    assert n.session==7 and not n.driver.fault


@pytest.mark.parametrize('skill',['navigate','align_for_pick','navigate_with_load','align_for_place'])
def test_cancel_travel_wait_or_active_action_keeps_whole_stop_contract(skill):
    s=stack();task=reach(s,skill)
    s.runtime.app.cancel(task.run_id,s.now)
    result=finish(s,task)
    assert result['status']=='CANCELLED' and s.stop.requests==1 and not result['control_owned']


def test_depth_loss_while_carrying_stops_and_does_not_resume_on_return():
    s=stack();task=reach(s,'navigate_with_load');n=s.guarded_navigation
    for _ in range(40):
        s.tick()
        if n.driver.owner is not None and any(abs(v)>.01 for v in s.velocity):break
    assert n.driver.owner is not None and s.held
    n.depth_enabled=False
    for _ in range(8):s.tick()
    assert n.driver.fault and s.stop.requests==1
    count=len(n.starts);n.depth_enabled=True
    for _ in range(8):s.tick()
    assert len(n.starts)==count and n.driver.fault and not n.port.ready()


def test_changed_registered_furniture_stops_active_task_and_invalidates_targets():
    s=stack();task=reach(s,'navigate_with_load')
    s.assembly.catalog.invalidate_place('bedroom_drop_zone','moved table')
    r=finish(s,task)
    assert s.stop.requests==1 and r['status']!='SUCCEEDED' and s.runtime.fault
    assert s.assembly.catalog.target is None and s.assembly.catalog.surface is None
    with pytest.raises(ValueError):s.runtime.submit({**request(),'request_id':'retry'})


def test_found_view_controls_alignment_and_lift_not_primary_place_geometry():
    s=stack();c=s.assembly.catalog;view=c.doc['places']['sofa']['views'][1]
    view['approach']['point']['x']=2.5
    c.accept_target({'view_id':view['id']})
    assert c.resolve('align_for_pick',{},s.now)['goal']['x']==2.5
    assert c.resolve('adjust_lift_for_pick',{},s.now)['height_um']==250000
    with pytest.raises(ValueError):c.accept_target({'view_id':'unknown'})
    assert c.source_place is None


def test_wrong_payload_camera_invalidates_proof_and_blocks_admission():
    s=stack();s.now+=.01
    with pytest.raises(ValueError,match='unqualified payload camera'):
        s.payload.observe_vision(PayloadObservation(s.seq+1,s.now,'remote_control','empty',camera_frame='rear_unqualified'),s.now)
    assert not s.payload.proof(s.now).conditions['payload_safe']
    with pytest.raises(ValueError):s.runtime.submit(request())


def test_base_motion_keeps_measured_lift_hold_but_vertical_drift_invalidates_it():
    s=stack();old=s.state.latest
    s.now+=.05
    s.state.observe(replace(old,sequence=old.sequence+1,observed_s=s.now,xy_yaw=(1.6,1.5,0),base_velocity=(.2,0,0)))
    assert s.state.conditions()['lift_hold_verified']
    old=s.state.latest;s.now+=.05
    s.state.observe(replace(old,sequence=old.sequence+1,observed_s=s.now,lift_um=old.lift_um+2000))
    assert not s.state.conditions()['lift_hold_verified']


def test_cancel_active_guarded_navigation_waits_for_measured_whole_stop():
    s=stack();task=reach(s,'navigate');n=s.guarded_navigation
    for _ in range(30):
        s.tick()
        if n.driver.owner and any(abs(v)>.01 for v in s.velocity):break
    assert n.driver.owner and any(abs(v)>.01 for v in s.velocity)
    s.runtime.app.cancel(task.run_id,s.now)
    assert task.state=='STOPPING' and s.runtime.app.lease.owner==task.run_id
    r=finish(s,task)
    assert r['status']=='CANCELLED' and not r['control_owned'] and s.stop.requests==1
    assert n.driver.fault and not n.port.ready()


@pytest.mark.parametrize('fault',['load_loss','sensor_loss','reboot'])
def test_guarded_whole_fetch_fault_preserves_unconfirmed_control(fault):
    s=stack();r=s.run(request(),fault=fault)
    assert r['status']=='STOP_UNCONFIRMED' and r['control_owned'] and s.stop.requests==1


def test_base_owner_assignment_is_atomic_for_invalid_authority():
    s=stack();d=s.guarded_navigation.driver
    with pytest.raises(ValueError):d.acquire('task/phase:1','bad/source','epoch')
    assert d.owner is None and d.source is None and d.epoch is None
