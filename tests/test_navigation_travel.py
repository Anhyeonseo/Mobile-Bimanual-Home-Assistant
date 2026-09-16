from dataclasses import replace
from types import SimpleNamespace as NS
import math
import numpy as np
import pytest
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.navigation_depth import DepthProfile,DepthVolume,TravelGuard,TravelEvidence
from home_robot_tasks.ros_navigation_depth import projected_grid
from home_robot_tasks.base_driver import BaseProfile,MobileBaseDriver
from home_robot_tasks.base_motion import OmniKinematics,Wheel
from home_robot_tasks.navigation_binding import GuardedNavigationPort
from home_robot_tasks.skill_ports import PortResult


def volume():
    v=DepthVolume(DepthProfile('depth','cal-1',.1,.1,4,0,1,.5,.5));v.reset('travel-1');return v


def observe(v,depth,stamp=1.,**kw):
    # Camera optical z points forwards (+x), y points down; camera height .5m.
    tf=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,.5],[0,0,0,1]],float)
    args=dict(observed_s=stamp,now_s=stamp,frame='depth',calibration='cal-1',epoch='travel-1',motion_qualified=True)
    args.update(kw)
    v.integrate(np.array([[depth]],float),(1,1,0,0),tf,**args)


def test_tabletop_is_retained_until_valid_depth_ray_clears_original_location():
    v=volume();observe(v,1.)
    a=next(iter(v.projected_obstacles()))
    assert a==(10,0)
    observe(v,2.,1.1)
    assert a not in v.projected_obstacles()
    assert (20,0) in v.projected_obstacles()
    # A lower 2D scan is not an input to this volume. Incomplete columns remain
    # unknown even if this one upper ray has passed through them.
    assert projected_grid(v,origin_xy=(1.,0.),width=1,height=1,now=1.1)==[-1]
    assert projected_grid(v,origin_xy=(2.,0.),width=1,height=1,now=1.1)==[100]


@pytest.mark.parametrize('kwargs',[{'frame':'rgb'},{'calibration':'other'},{'epoch':'old'},
    {'motion_qualified':False},{'now_s':2},{'now_s':.5}])
def test_bad_depth_never_clears_obstacle(kwargs):
    v=volume();observe(v,1.)
    before=dict(v.cells)
    with pytest.raises(InvalidTask):observe(v,2.,1.1,**kwargs)
    assert v.cells==before and not v.fresh(1.1)


@pytest.mark.parametrize('depth',[0,float('nan'),float('inf'),.05,5])
def test_missing_out_of_range_depth_is_unknown_not_free(depth):
    v=volume();observe(v,1.)
    with pytest.raises(InvalidTask):observe(v,depth,1.1)
    assert (10,0) in v.projected_obstacles()


def test_replay_mode_reset_and_budget_are_fail_closed():
    v=volume();observe(v,1.)
    with pytest.raises(InvalidTask):observe(v,2.)
    v.reset('travel-2');assert not v.cells and not v.fresh(1.)
    v=DepthVolume(replace(volume().p,resolution_m=.01,maximum_ray_steps=100));v.reset('travel-1')
    with pytest.raises(InvalidTask,match='work budget'):observe(v,3.)
    assert not v.cells


def guard(v):
    return TravelGuard(v,radius_m=.2,height_m=1,margin_m=.02,reaction_s=.1,braking_m_s2=1,
                       maximum_speed_m_s=.4,maximum_yaw_rad_s=.8)


def filled():
    v=volume();v.last_stamp=1.;v.error=None
    v.cells={(x,y,z):(False,1.) for x in range(-20,21) for y in range(-20,21) for z in range(10)}
    return v


@pytest.mark.parametrize('cmd',[(.2,0,0),(0,.2,0),(-.2,0,0),(.1,.1,.2),(0,0,.3)])
def test_full_height_observed_corridor_allows_omni_motion(cmd):
    v=filled();assert guard(v).check(cmd,(0,0,0),(0,0,0),TravelEvidence(1.,v.epoch,True,True,True),1.1)==cmd


@pytest.mark.parametrize('kind',['upper_hit','upper_unknown','stale','mode','localization','posture','initial_envelope'])
def test_missing_upper_space_or_state_rejects_travel(kind):
    v=filled();e=TravelEvidence(1.,v.epoch,True,True,True)
    if kind=='upper_hit':v.cells[(4,0,8)]=(True,1.)
    if kind=='upper_unknown':del v.cells[(4,0,8)]
    if kind=='mode':e=replace(e,epoch='old')
    if kind=='localization':e=replace(e,localized=False)
    if kind=='posture':e=replace(e,ready=False)
    if kind=='initial_envelope':e=replace(e,envelope_clear=False)
    with pytest.raises(InvalidTask):guard(v).check((.3,0,0),(0,0,0),(0,0,0),e,2 if kind=='stale' else 1.1)


def test_command_zero_still_checks_measured_stopping_sweep():
    v=filled();v.cells[(4,0,8)]=(True,1.)
    with pytest.raises(InvalidTask):guard(v).check((0,0,0),(.3,0,0),(0,0,0),TravelEvidence(1.,v.epoch,True,True,True),1.1)


class Client:
    session=7
    def __init__(self,clock):self.clock=clock;self.calls=[];self.tick=None;self.boot=42;self.rates=(0,0,0,0)
    def status(self):
        tick=int(self.clock()*1000) if self.tick is None else self.tick
        return NS(feedback_tick_ms=tick,mcu_tick_ms=int(self.clock()*1000),velocity_raw=self.rates,
                  boot_id=self.boot,ready_for_motion=lambda *args:True)
    def velocity(self,rates,**kw):self.calls.append((rates,kw));self.rates=rates;return self.status()


def driver():
    now=[10.];clock=lambda:now[0];client=Client(clock)
    geometry=OmniKinematics(tuple(Wheel(.2*math.cos(a),.2*math.sin(a),a+math.pi/2,.05) for a in (0,2*math.pi/3,4*math.pi/3)))
    stops=[];checks=[]
    d=MobileBaseDriver(client,geometry,BaseProfile((100,100,100),(10,10,10),(100,100,100),.2,.1,.1),clock,
        NS(check=lambda *a:checks.append(a)),lambda:NS(epoch='e'),stops.append)
    d.acquire('goal','nav2','e');now[0]+=.01
    d.submit((.2,0,0),observed_s=now[0],owner='goal',source='nav2',epoch='e')
    return d,now,stops,checks


def test_si_cmd_raw_zero_lift_and_measured_odom_not_commanded_pose():
    d,n,s,c=driver();d.tick();assert not s
    assert d.observation.pose==(0,0,0)
    raw=d.client.calls[-1][0];assert raw[3]==0 and any(raw[:3])
    assert c[-1][0]==pytest.approx(d.geometry.body_twist(tuple(v/100 for v in raw[:3])))
    n[0]+=.05;d.tick();n[0]+=.05;d.tick();assert d.observation.pose[0]>0
    assert d.client.calls[-1][1]['lifetime_ms']<100


def test_repeated_feedback_is_not_retimestamped_and_eventually_stops():
    d,n,s,c=driver();d.tick();first=d.observation
    d.client.tick=first.feedback_tick_ms;n[0]+=.05;d.tick()
    assert d.observation==first
    n[0]+=.06;d.tick();assert d.fault and s


@pytest.mark.parametrize('kind',['expired','future','wrong_owner','replay','cycle','boot','mode'])
def test_invalid_command_feedback_or_owner_latches_whole_stop(kind):
    d,n,s,c=driver();d.tick()
    if kind=='cycle':n[0]+=.11
    elif kind=='boot':d.client.boot=43;n[0]+=.01
    elif kind=='mode':d.travel_evidence=lambda:NS(epoch='other');n[0]+=.01
    else:
        stamp=n[0];owner='goal'
        if kind=='expired':stamp-=1
        if kind=='future':stamp+=1
        if kind=='wrong_owner':owner='other';n[0]+=.01;stamp=n[0]
        with pytest.raises(InvalidTask):d.submit((1,0,0),observed_s=stamp,owner=owner,source='nav2',epoch='e')
    d.tick();assert d.fault and s and d.owner=='goal'
    with pytest.raises(InvalidTask):d.acquire('other','nav2','e')


def test_navigation_success_waits_for_measured_stop():
    d,n,s,c=driver();d.release(stop_confirmed=True)
    stopped=[False]
    nav=NS(ready=lambda:True,start=lambda *a:None,poll=lambda g:PortResult('SUCCEEDED'),cancel=lambda g:None)
    p=GuardedNavigationPort(nav,d,lambda:n[0],lambda:'e',lambda after:stopped[0]);p.start('second',{})
    n[0]+=.01
    assert p.poll('second').state=='RUNNING' and d.command[1]==(0,0,0)
    assert d.owner=='second'
    stopped[0]=True;n[0]+=.01
    assert p.poll('second').state=='SUCCEEDED' and d.owner is None


def test_camera_mount_frustum_detects_rear_near_upper_and_floor_blind_regions():
    from home_robot_tasks.camera_coverage import frustum_coverage
    visible=frustum_coverage([(2,0,.5),(-2,0,.5),(.1,0,.5),(1,0,2),(1,0,0)],
        height_m=.5,pitch_down_rad=0,camera_x_m=0,width=640,height=480,fx=500,fy=660,cx=320,cy=240,
        minimum_depth_m=.45,maximum_depth_m=4)
    assert visible==[True,False,False,False,False]


def test_guard_processing_delay_cannot_refresh_command_lifetime():
    d,n,s,c=driver()
    d.guard=NS(check=lambda *a:n.__setitem__(0,n[0]+.11))
    d.tick()
    assert d.fault and s and not d.client.calls


def test_state_travel_evidence_requires_measured_lift_and_clearance_epoch():
    from home_robot_tasks.navigation_binding import StateTravelEvidence
    keys=('hardware_ready','arm_in_transport_pose','lift_in_transport_position','lift_stopped','lift_homed','lift_hold_verified','localized')
    conditions=dict.fromkeys(keys,True)
    state=NS(conditions=lambda:conditions,latest=NS(observed_s=1.),clock=lambda:1.1)
    provider=StateTravelEvidence(state,volume(),lambda:(.9,'travel-1',True))
    assert provider().ready and provider().observed_s==.9
    conditions['lift_stopped']=False
    assert not provider().ready


def test_nav2_depth_candidate_keeps_laser_and_upper_depth_separate():
    from pathlib import Path
    import yaml
    cfg=yaml.safe_load((Path(__file__).parents[1]/'config/nav2.depth.simulation.yaml').read_text())
    assert cfg['/offline_nav2/controller_server']['ros__parameters']['enable_stamped_cmd_vel'] is True
    local=cfg['/offline_nav2/local_costmap/local_costmap']['ros__parameters']
    assert local['plugins'].index('depth_layer')>local['plugins'].index('obstacle_layer')
    assert local['use_maximum'] and not local['depth_layer']['footprint_clearing_enabled']


def test_future_clearance_cannot_hide_behind_older_robot_stamp():
    from home_robot_tasks.navigation_binding import StateTravelEvidence
    state=NS(conditions=lambda:{},latest=NS(observed_s=1.),clock=lambda:1.1)
    with pytest.raises(InvalidTask,match='future'):
        StateTravelEvidence(state,volume(),lambda:(2.,'travel-1',True))()


def test_ros_factory_adapts_common_no_argument_whole_stop_request(monkeypatch):
    import home_robot_tasks.base_driver as base
    import home_robot_tasks.nav2_port as nav
    from home_robot_tasks.navigation_binding import build_guarded_navigation
    original,n,_,_=driver()
    monkeypatch.setattr(base,'RosBaseIO',lambda node,d,**kw:NS(driver=d))
    monkeypatch.setattr(nav,'Nav2Port',lambda *args:NS())
    stops=[]
    port,io,maintain=build_guarded_navigation(node=None,client=original.client,geometry=original.geometry,
        profile=original.p,guard=NS(volume=volume()),state=None,clearance=None,clock=lambda:n[0],
        request_stop=lambda:stops.append('whole-stop'),io_parameters={},reset_observations=lambda epoch:None)
    io.driver._fault('sensor lost')
    assert stops==['whole-stop'] and io.driver.stop_error is None
