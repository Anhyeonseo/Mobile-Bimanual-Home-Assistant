import math
import numpy as np
import pytest
from home_robot_tasks import platform_preparation
from home_robot_tasks.navigation_trial import NearGoal,swept_obstacle


def trial():
    geometry={'wheels':[dict(x_m=.18*math.cos(a),y_m=.18*math.sin(a),rolling_angle_rad=a+math.pi/2,radius_m=.05) for a in (0,2*math.pi/3,4*math.pi/3)]}
    area=dict(feasible_by_yaw=np.ones((72,100,100),bool),resolution_m=.05,origin_xy_m=np.array([-2.5,-2.5]))
    return NearGoal(geometry,area)


def test_preview_never_moves_and_start_is_explicit():
    t=trial();t.plan((0,0,0));assert t.command(1,(0,0,0),1,sensors_fresh=True,corridor_clear=True)==(0,0,0,0)
    with pytest.raises(ValueError):t.start(1,(0,0,0),operator_confirmed=False)
    t.start(1,(0,0,0),operator_confirmed=True)
    r=t.command(1.1,(0,0,0),1.1,sensors_fresh=True,corridor_clear=True)
    assert any(r) and max(map(abs,r))<=50 and r[3]==0
    with pytest.raises(ValueError):t.start(2,(0,0,0),operator_confirmed=True)


def test_completion_needs_three_distinct_poses_then_measured_stop():
    t=trial();t.plan((0,0,0));t.start(1,(0,0,0),operator_confirmed=True)
    for _ in range(10):t.command(1.1,(.19,0,0),1.1,sensors_fresh=True,corridor_clear=True)
    assert t.state=='RUNNING'
    for stamp in (1.2,1.3):t.command(stamp,(.19,0,0),stamp,sensors_fresh=True,corridor_clear=True)
    assert t.state=='SETTLING'
    with pytest.raises(ValueError):t.finish(False)
    t.finish(True);assert t.state=='SUCCEEDED'


def test_sensor_loss_preserves_goal_and_requires_resume_after_two_seconds():
    t=trial();t.plan((0,0,0));t.start(1,(0,0,0),operator_confirmed=True)
    assert not any(t.command(1.1,(0,0,0),1,sensors_fresh=False,corridor_clear=True))
    assert not any(t.command(3.2,(0,0,0),1,sensors_fresh=False,corridor_clear=True))
    assert t.state=='WAITING_SENSOR' and t.resume_required and t.goal==(.2,0,0)


def test_cancel_and_changed_preview_cannot_move():
    t=trial();t.plan((0,0,0))
    with pytest.raises(ValueError):t.start(1,(.1,0,0),operator_confirmed=True)
    t.cancel();assert not any(t.command(1,(0,0,0),1,sensors_fresh=True,corridor_clear=True))


def test_sweep_detects_obstacle_between_endpoints_and_abutting_body():
    assert swept_obstacle([[.35,0]],(.2,0))
    assert swept_obstacle([[.22,0]],(.2,0))
    assert not swept_obstacle([[1,0]],(.2,0))
    assert not swept_obstacle([[0,0]],(.2,0))


def test_map_obstacle_or_unknown_rejects_route():
    t=trial();t.area['feasible_by_yaw'][:,:,53]=False
    with pytest.raises(ValueError):t.plan((0,0,0))


def web_state():
    import sys,time
    from pathlib import Path
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/run'))
    from navigation_trial_web import State
    t=trial();s=State(t,None);s.pose=[0,0,0];s.pose_stamp=s.scan_stamp=s.depth_stamp=time.monotonic()
    s.scan_error=s.depth_error=None;s.scan_points=[[2,0]];s.depth_points=[[2,0,1]]
    s.path=t.plan(s.pose);s.controller_ready=True
    return s


def test_browser_start_checks_posture_sensors_preview_and_is_one_shot():
    s=web_state();assert s.snapshot()['ready']
    owner='a'*20
    with pytest.raises(ValueError):s.action(dict(owner=owner,action='start',confirmed=False))
    assert s.request is None and s.nonzero==0
    s.action(dict(owner=owner,action='start',confirmed=True))
    assert s.request=='start' and s.trial.state=='READY' and s.nonzero==0
    with pytest.raises(ValueError):s.action(dict(owner=owner,action='start',confirmed=True))


@pytest.mark.parametrize('blocked',('controller','depth','scan','obstacle'))
def test_start_rejects_missing_live_evidence(blocked):
    s=web_state()
    if blocked=='controller':s.controller_ready=False
    elif blocked=='depth':s.depth_stamp=-1
    elif blocked=='scan':s.pose_stamp=-1
    else:s.scan_points=[[.3,0]]
    with pytest.raises(ValueError):s.action(dict(owner='a'*20,action='start',confirmed=True))
    assert s.request is None and s.nonzero==0


def test_http_status_is_read_only_and_start_requires_bearer():
    import http.client,json,threading
    from http.server import ThreadingHTTPServer
    from navigation_trial_web import handler_for
    s=web_state();server=ThreadingHTTPServer(('127.0.0.1',0),handler_for(s,'secret',{}))
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        c=http.client.HTTPConnection('127.0.0.1',server.server_port)
        c.request('GET','/status',headers={'Authorization':'Bearer secret'});r=c.getresponse();assert r.status==200;r.read()
        assert s.request is None and s.nonzero==0
        c.request('POST','/action',body=json.dumps(dict(owner='a'*20,action='start',confirmed=True)),headers={'Content-Type':'application/json'})
        r=c.getresponse();assert r.status==403;r.read();assert s.request is None
        c.close()
    finally:server.shutdown();server.server_close();thread.join()


def test_voxel_dedup_preserves_every_endpoint_cell():
    from home_robot_tasks.navigation_trial import voxel_centers
    rng=np.random.default_rng(52)
    points=rng.uniform(-.64,.64,(30000,3))
    points=np.concatenate([points,points[:1000],[[0,0,0],[.02,-.02,.001]]])
    expected=np.unique(np.floor(points/.02).astype(np.int64),axis=0)*.02+.01
    np.testing.assert_allclose(voxel_centers(points),expected,atol=1e-15)
    assert voxel_centers([]).shape==(0,3)
    with pytest.raises(ValueError,match='extent'):voxel_centers([[0,0,0],[100,100,100]])


@pytest.mark.parametrize('reason', NearGoal.WAITING)
def test_long_pause_requires_explicit_resume_and_excludes_idle_time(reason):
    t=trial();t.plan((0,0,0));t.start(1,(0,0,0),operator_confirmed=True);t.pause(1.1,reason)
    for stamp in (102.,102.2,102.4):
        assert not any(t.command(stamp,(0,0,0),stamp,sensors_fresh=True,corridor_clear=True))
    assert t.state in t.WAITING and t.recovery_ready and t.resume_required
    with pytest.raises(ValueError):t.resume(102.4,(0,0,0),102.4,operator_confirmed=False,sensors_fresh=True,corridor_clear=True)
    t.resume(102.4,(0,0,0),102.4,operator_confirmed=True,sensors_fresh=True,corridor_clear=True)
    assert any(t.command(102.5,(0,0,0),102.5,sensors_fresh=True,corridor_clear=True))
    assert t.goal==(.2,0,0)


def test_short_pause_waits_for_distinct_stable_observations():
    t=trial();t.plan((0,0,0));t.start(1,(0,0,0),operator_confirmed=True)
    t.command(1.1,(0,0,0),1.1,sensors_fresh=False,corridor_clear=True)
    for stamp in (1.2,1.3,1.4):
        assert not any(t.command(stamp,(0,0,0),1.2,sensors_fresh=True,corridor_clear=True))
    assert t.state=='WAITING_SENSOR'
    for stamp in (1.5,1.6):
        assert not any(t.command(stamp,(0,0,0),stamp,sensors_fresh=True,corridor_clear=True))
    assert t.state=='RUNNING'
    assert any(t.command(1.7,(0,0,0),1.7,sensors_fresh=True,corridor_clear=True))


def test_browser_reconnect_never_auto_resumes_even_with_fresh_sensors():
    t=trial();t.plan((0,0,0));t.start(1,(0,0,0),operator_confirmed=True);t.pause(1.1,'WAITING_BROWSER')
    for stamp in (1.2,1.4,1.6):
        assert not any(t.command(stamp,(0,0,0),stamp,sensors_fresh=True,corridor_clear=True))
    assert t.recovery_ready and t.state=='WAITING_BROWSER'
    # Same evidence cannot authorize a route that changed after the button was enabled.
    t.area['feasible_by_yaw'][:,:,53]=False
    with pytest.raises(ValueError):t.resume(1.6,(0,0,0),1.6,operator_confirmed=True,sensors_fresh=True,corridor_clear=True)
    assert t.state=='WAITING_BROWSER'


def test_ready_false_for_queued_running_paused_finished():
    s=web_state();s.request='start';assert not s.snapshot()['ready'];s.request=None
    for state in ('RUNNING','WAITING_BROWSER','SETTLING','FAILED','SUCCEEDED'):
        s.trial.state=state;assert not s.snapshot()['ready']


def test_new_browser_can_explicitly_resume_expired_owner_only():
    import time
    s=web_state();now=time.monotonic();s.trial.start(now,s.pose,operator_confirmed=True)
    s.owner='a'*20;s.last_browser=now;s.trial.pause(now,'WAITING_BROWSER');s.trial.recovery_ready=True
    with pytest.raises(ValueError):s.action(dict(owner='b'*20,action='resume',confirmed=True))
    s.last_browser=now-4
    s.action(dict(owner='b'*20,action='heartbeat'))
    assert s.trial.state=='WAITING_BROWSER' and s.request is None
    s.action(dict(owner='b'*20,action='resume',confirmed=True))
    assert s.request=='resume' and s.owner=='b'*20 and not s.snapshot()['resume_ready']



def test_stop_observation_outlives_deadline_and_reads_despite_arm_failure(monkeypatch):
    from dataclasses import dataclass
    import navigation_trial_web as web
    @dataclass
    class Proof:
        base_confirmed: bool
    class Record:
        def __init__(self):self.events=[]
        def write(self,event,**data):self.events.append((event,data))
    class Closing:
        steps=0
        def is_set(self):return self.steps>=12
        def wait(self,_):self.steps+=1
    class Stop:
        def __init__(self,state):self.state=state;self.reads=0
        def status(self):
            self.reads+=1
            return Proof(6<=self.state.closed.steps<11),None
    class Transport:
        def stationary_runtime(self):return {'cached':True}
    s=web_state();s.record=Record();s.closed=Closing();s.controller_ready=False
    monkeypatch.setattr(web.time,'monotonic',lambda:s.closed.steps*.5)
    def broken_arm(**_):raise RuntimeError('arm observation unavailable')
    stop=Stop(s);web.observe_stopped(s,broken_arm,stop,Transport())
    assert stop.reads==12  # includes observations after the former two-second exit
    assert any(event=='final_stop' for event,_ in s.record.events)
    assert not s.stop_confirmed  # later loss revokes confirmation
    assert s.request is None and s.nonzero==0 and not s.controller_ready


def straight_trial():
    import json
    from pathlib import Path
    from pulse_mobile_base import PATTERNS
    geometry=json.loads((Path(__file__).resolve().parents[1]/'config/home_assistant/s4_base.nominal.json').read_text())
    return NearGoal(geometry,trial().area,forward_pattern=PATTERNS['forward'][1])


def test_forward_trial_matches_successful_teleop_and_reanchors_preview():
    t=straight_trial();t.plan((.61995,.70744,-.402015))
    start=(.61530,.73170,-.357046)
    t.start(1,start,operator_confirmed=True)
    assert t.start_pose==start
    assert math.dist(t.goal[:2],start[:2])==pytest.approx(.2)
    assert t.goal[2]==start[2]
    for dx,dy,da in ((0,0,0),(.01,.01,.02),(.02,-.01,-.02)):
        pose=(start[0]+dx,start[1]+dy,start[2]+da)
        assert t.command(1.1,pose,1.1,sensors_fresh=True,corridor_clear=True)==(-50,50,0,0)


def test_forward_trial_rejects_blocked_reanchored_route():
    t=straight_trial();t.plan((0,0,0));t.map_free=lambda _:False
    with pytest.raises(ValueError):t.start(1,(.01,0,0),operator_confirmed=True)
    assert t.state=='READY'


def test_forward_trial_does_not_chase_sideways_or_overshoot():
    t=straight_trial();t.plan((0,0,0));t.start(1,(0,0,0),operator_confirmed=True)
    for stamp in (1.1,1.2):
        assert t.command(stamp,(.09,-.09,0),stamp,sensors_fresh=True,corridor_clear=True)==(0,0,0,0)
    with pytest.raises(ValueError,match='경로 이탈'):
        t.command(1.5,(.09,-.09,0),1.5,sensors_fresh=True,corridor_clear=True)
    with pytest.raises(ValueError,match='거리 도달'):
        t.command(1.6,(.26,0,0),1.6,sensors_fresh=True,corridor_clear=True)


def test_failed_observer_consumes_operator_stop_even_without_arm_reads(monkeypatch):
    import navigation_trial_web as web
    s=web_state();s.trial.state='FAILED';s.request='cancel'
    class Record:
        def write(self,*a,**kw):pass
    class Stop:
        requests=0
        def request(self):self.requests+=1
        def status(self):s.closed.set();raise RuntimeError('no proof')
    class Transport:
        def stationary_runtime(self):return {}
    s.record=Record();stop=Stop()
    def broken(**kw):raise RuntimeError('arm snapshot unavailable')
    web.observe_stopped(s,broken,stop,Transport())
    assert stop.requests==1 and s.request is None and not s.stop_confirmed


def test_fixed_lift_sweep_blocks_side_contact_without_rectangular_empty_corners():
    import json
    from pathlib import Path
    body=json.loads((Path(__file__).resolve().parents[1]/'config/home_assistant/navigation_body.fixed.json').read_text())
    poly=body['footprint_xy_m']
    # A post 20 cm to the side of the translated lift, beyond the bare base.
    assert swept_obstacle([[.27,.208]],(.2,0),footprint=poly,margin=0)
    assert not swept_obstacle([[.27,.208]],(.2,0),body=(-.154,.20,-.19,.195),margin=0)
    # Preserve the actual curved/narrow front instead of filling a large rectangle.
    assert not swept_obstacle([[.38,.20]],(.2,0),footprint=poly,margin=0)


def test_navigation_geometry_rejects_stale_map_and_footprint(tmp_path):
    import json,hashlib
    from pathlib import Path
    from home_robot_tasks.navigation_trial import validate_navigation_body
    body=json.loads((Path(__file__).resolve().parents[1]/'config/home_assistant/navigation_body.fixed.json').read_text())
    m=tmp_path/'map.json';m.write_text('{}')
    area=dict(navigation_footprint_xy_m=body['footprint_xy_m'],clearance_m=.03,source_map_sha256=hashlib.sha256(m.read_bytes()).hexdigest())
    assert validate_navigation_body(body,area,m)==body
    for change in ({'navigation_footprint_xy_m':[]},{'clearance_m':.01},{'source_map_sha256':'old'}):
        with pytest.raises(ValueError):validate_navigation_body(body,{**area,**change},m)


@pytest.mark.parametrize('blocked',[False,True])
def test_bad_registration_keeps_fresh_obstacles_and_last_good_pose(blocked):
    s=web_state();start=s.pose_stamp;goal=s.trial.goal
    s.trial.start(start,s.pose,operator_confirmed=True)
    points=np.tile([2.,1.],(80,1))
    if blocked:points[0]=[.3,0]
    s.observe_scan(points,start+.1)
    with pytest.raises(ValueError) as failure:
        s.observe_localization([.3,.3,1],start+.1,.6,60)
    s.scan_rejected(failure.value)
    assert s.pose==[0,0,0] and s.pose_stamp==start
    assert s.sensors_ready(start+.1)
    command=s.trial.command(start+.1,s.pose,s.pose_stamp,
        sensors_fresh=s.sensors_ready(start+.1),corridor_clear=s.corridor_clear())
    assert bool(any(command)) is not blocked
    assert s.trial.state==('WAITING_OBSTACLE' if blocked else 'RUNNING')
    assert s.trial.goal==goal


def test_rejected_scans_cannot_refresh_collision_or_localization_age():
    s=web_state();start=s.pose_stamp
    s.scan_rejected('one malformed frame')
    assert s.sensors_ready(start+.1)
    assert not s.sensors_ready(start+.351)  # no new collision scan
    for elapsed in (.2,.3,.4,.5,.51):
        s.observe_scan(np.tile([2.,1.],(80,1)),start+elapsed)
        s.scan_rejected('map outlier')
    assert s.pose_stamp==start
    assert not s.sensors_ready(start+.51)  # fresh scan cannot refresh stale pose


def test_scan_coverage_loss_and_initial_localization_never_authorize_motion():
    s=web_state();start=s.pose_stamp
    with pytest.raises(ValueError):s.observe_scan(np.array([[.3,0]]),start+.1)
    assert not s.sensors_ready(start+.1) and not s.corridor_clear()
    s.observe_scan(np.tile([2.,1.],(80,1)),start+.2)
    s.pose=None;s.pose_stamp=-1
    s.scan_rejected('no initial localization')
    assert not s.sensors_ready(start+.2)


def test_successful_localization_clears_warning_without_hiding_raw_scan_age():
    s=web_state();start=s.pose_stamp
    s.scan_rejected('temporary outlier')
    s.observe_scan(np.tile([2.,1.],(80,1)),start+.1)
    s.observe_localization([.01,0,0],start+.1,.9,80)
    assert s.scan_warning is None and s.pose==[.01,0,0]
    assert s.pose_stamp==s.scan_stamp==start+.1
    with pytest.raises(ValueError):s.observe_scan(np.tile([2.,1.],(80,1)),start)
    assert s.scan_stamp==start+.1


def test_new_obstacle_can_occlude_map_without_losing_supported_tracking():
    s=web_state();start=s.pose_stamp
    support=dict(matched_endpoints=175,rmse_m=.032,minor_spread_m=.4)
    s.observe_localization([.01,0,0],start+.1,.73,179,support)
    assert s.pose==[.01,0,0] and s.pose_stamp==start+.1


def test_occluded_acquisition_requires_reviewed_seed_and_multiple_matching_scans():
    s=web_state();s.pose=None;s.pose_stamp=-1;s.acquisition_reference=[0,0,0]
    support=dict(matched_endpoints=175,rmse_m=.032,minor_spread_m=.4)
    for at in (1.,1.2):
        with pytest.raises(ValueError):s.observe_localization([.01,0,0],at,.73,179,support)
        assert s.pose is None
    s.observe_localization([.015,0,0],1.4,.73,179,support)
    assert s.pose==[.015,0,0]
    s.pose=None;s.acquisition_samples=[]
    for at in (2.,2.2,2.4,2.6):
        with pytest.raises(ValueError):s.observe_localization([.3,0,0],at,.73,179,support)
    assert s.pose is None


@pytest.mark.parametrize('moving',[False,True])
def test_occluded_reacquisition_after_gap_requires_stationary_feedback(moving):
    s=web_state();start=s.pose_stamp
    s.nav2_control={'execute':False}
    support=dict(matched_endpoints=175,rmse_m=.032,minor_spread_m=.4)
    for i,elapsed in enumerate((1.,1.2,1.4)):
        at=start+elapsed
        s.wheel_observation=dict(observed_s=at-.02,twist=[.02 if moving else 0.,0.,0.])
        if moving or i<2:
            with pytest.raises(ValueError):s.observe_localization([.01,0,0],at,.73,179,support)
            assert s.pose_stamp==start
        else:
            s.observe_localization([.01,0,0],at,.73,179,support)
            assert s.pose_stamp==at


@pytest.mark.parametrize('bad',['unknown_pose','gap','few_points','single_wall','residual','jump'])
def test_occlusion_tracking_does_not_admit_ambiguous_or_stale_pose(bad):
    s=web_state();start=s.pose_stamp;at=start+.1;candidate=[.01,0,0]
    support=dict(matched_endpoints=175,rmse_m=.032,minor_spread_m=.4)
    if bad=='unknown_pose':s.pose=None
    if bad=='gap':at=start+.6
    if bad=='few_points':support['matched_endpoints']=60
    if bad=='single_wall':support['minor_spread_m']=.01
    if bad=='residual':support['rmse_m']=.08
    if bad=='jump':candidate=[.5,0,0]
    with pytest.raises(ValueError):s.observe_localization(candidate,at,.73,179,support)
    assert s.pose_stamp==start



def turn_trial():
    from home_robot_tasks.navigation_trial import TurnDriveTrial
    from pulse_mobile_base import PATTERNS
    t=trial();g=dict(wheels=[dict(x_m=.18*math.cos(a),y_m=.18*math.sin(a),rolling_angle_rad=a+math.pi/2,radius_m=.05) for a in (0,2*math.pi/3,4*math.pi/3)],wheel_order=[7,8,9])
    body=dict(footprint_xy_m=[[-.1541,-.2134],[.2003,-.2134],[.2003,.2115],[-.1541,.2115]],clearance_m=.03)
    return TurnDriveTrial(g,t.area,patterns=PATTERNS,navigation_body=body)


def test_turn_drive_has_rotation_sweep_and_never_mixes_translation_with_yaw():
    t=turn_trial();path=t.plan((0,0,0));assert len(path)>40
    assert any(p[0]==p[1]==0 and p[2]<0 for p in path)
    assert t.command(0,(0,0,0),0,sensors_fresh=True,corridor_clear=True)==(0,0,0,0)
    t.start(1,(0,0,0),operator_confirmed=True)
    assert t.command(1.1,(0,0,0),1.1,sensors_fresh=True,corridor_clear=True)==(-50,-50,-50,0)
    pose=(0,0,t.goal[2])
    for now in (1.2,1.4,1.6):assert not any(t.command(now,pose,now,sensors_fresh=True,corridor_clear=True))
    assert t.phase=='DRIVING'
    assert t.command(2,pose,2,sensors_fresh=True,corridor_clear=True)==(-75,75,0,0)
    for now in (2.2,2.4,2.6):assert not any(t.command(now,t.goal,now,sensors_fresh=True,corridor_clear=True))
    assert t.state=='SETTLING';t.finish(True);assert t.state=='SUCCEEDED'


def test_turn_drive_waits_for_obstacle_and_resumes_same_goal_without_reset():
    t=turn_trial();t.plan((0,0,0));goal=t.goal;t.start(1,(0,0,0),operator_confirmed=True)
    assert not any(t.command(1.1,(0,0,0),1.1,sensors_fresh=True,corridor_clear=False))
    assert t.state=='WAITING_OBSTACLE'
    assert not any(t.command(4,(0,0,0),4,sensors_fresh=True,corridor_clear=False))
    for now in (4.1,4.3,4.5):assert not any(t.command(now,(0,0,0),now,sensors_fresh=True,corridor_clear=True))
    assert t.resume_required and t.recovery_ready and t.goal==goal
    t.resume(4.6,(0,0,0),4.6,operator_confirmed=True,sensors_fresh=True,corridor_clear=True)
    assert t.goal==goal and t.command(4.7,(0,0,0),4.7,sensors_fresh=True,corridor_clear=True)==(-50,-50,-50,0)


def test_turn_drive_reorients_before_forward_when_pose_heading_changes():
    t=turn_trial();t.plan((0,0,0));t.start(1,(0,0,0),operator_confirmed=True);t.phase='DRIVING'
    assert not any(t.command(1.1,(0,0,0),1.1,sensors_fresh=True,corridor_clear=True))
    assert t.phase=='TURNING'
    assert t.command(1.5,(0,0,0),1.5,sensors_fresh=True,corridor_clear=True)==(-50,-50,-50,0)


def test_rotating_wide_lift_detects_corner_obstacle_outside_translation_box():
    from home_robot_tasks.navigation_trial import swept_path_obstacle
    poly=[[-.1,-.25],[.1,-.25],[.1,.25],[-.1,.25]]
    point=[[.22,0,.5]]
    assert not swept_obstacle(point,(0,0),footprint=poly,margin=0)
    path=[(0,0,a) for a in np.linspace(0,math.pi/2,46)]
    assert swept_path_obstacle(point,path,footprint=poly,margin=0)
    assert not swept_path_obstacle([[1,1]],path,footprint=poly,margin=0)


def test_turn_drive_checks_map_along_in_place_rotation():
    t=turn_trial();t.area['feasible_by_yaw'][70,50,50]=False
    with pytest.raises(ValueError):t.plan((0,0,0))


def test_turn_drive_blocked_start_means_wait_not_permission_to_move():
    s=web_state();s.trial=turn_trial();s.path=s.trial.plan(s.pose);s.scan_points=[[.3,-.15]]
    assert not s.corridor_clear() and s.snapshot()['ready']
    s.action(dict(owner='a'*20,action='start',confirmed=True));assert s.request=='start'
    s.trial.start(1,s.pose,operator_confirmed=True)
    assert not any(s.trial.command(1.1,s.pose,1.1,sensors_fresh=True,corridor_clear=s.corridor_clear()))
    assert s.trial.state=='WAITING_OBSTACLE'


def test_turn_drive_uses_full_depth_and_lidar_path():
    s=web_state();s.trial=turn_trial();s.path=s.trial.plan(s.pose)
    x,y,_=s.trial.goal;s.depth_points=[[x+.12,y,.5]]
    assert not s.corridor_clear() and s.corridor_hits()['depth']
    s.depth_points=[[1,1,.5]];assert s.corridor_clear()


def test_turn_drive_cancel_never_replays_old_commands():
    t=turn_trial();t.plan((0,0,0));t.start(1,(0,0,0),operator_confirmed=True);t.cancel()
    assert t.command(2,(0,0,0),2,sensors_fresh=True,corridor_clear=True)==(0,0,0,0)



def test_turn_drive_does_not_claim_arrival_with_wrong_final_heading():
    t=turn_trial();t.plan((0,0,0));t.start(1,(0,0,0),operator_confirmed=True)
    pose=(*t.goal[:2],0)
    assert t.command(2,pose,2,sensors_fresh=True,corridor_clear=True)==(-50,-50,-50,0)
    assert t.state=='RUNNING' and t.phase=='FINAL_TURNING'


def test_turn_drive_closed_loop_simulation_stops_and_resumes_with_goal_preserved():
    t=turn_trial();t.plan((0,0,0));t.start(0,(0,0,0),operator_confirmed=True);goal=t.goal
    pose=np.array([0.,0.,0.]);paused=False
    for i in range(1,2000):
        now=i*.1;blocked=30<=i<38
        command=t.command(now,pose,now,sensors_fresh=True,corridor_clear=not blocked)
        if blocked:assert not any(command);paused=True
        if command[:3]==(-75,75,0):pose[:2]+=.06*.1*np.array([math.cos(pose[2]),math.sin(pose[2])])
        elif command[:3]==(-50,-50,-50):pose[2]-=.3*.1
        elif command[:3]==(50,50,50):pose[2]+=.3*.1
        else:assert not any(command)
        if t.state=='SETTLING':break
    assert paused and t.goal==goal and t.state=='SETTLING'
    assert math.dist(pose[:2],goal[:2])<=.04
    t.finish(True);assert t.state=='SUCCEEDED'



def test_rotation_broadphase_includes_margin_at_polygon_corners():
    from home_robot_tasks.navigation_trial import swept_path_obstacle
    poly=[[-.1,-.25],[.1,-.25],[.1,.25],[-.1,.25]]
    assert swept_path_obstacle([[.129,.279]],[(0,0,0)],footprint=poly,margin=.03)



def asynchronous_web_state():
    s=web_state();s.trial=turn_trial();s.path=s.trial.plan(s.pose)
    hits=s.corridor_hits();s.async_corridor=True
    s.corridor_observation=dict(goal=s.trial.goal,hits=hits,pose_stamp=s.pose_stamp,
        scan_stamp=s.scan_stamp,depth_stamp=s.depth_stamp,calculation_ms=90)
    return s


def test_live_status_and_command_do_not_run_geometry_on_controller_thread(monkeypatch):
    import time
    s=asynchronous_web_state()
    def forbidden(*args):raise AssertionError('geometry blocks serial heartbeat')
    monkeypatch.setattr(s,'compute_corridor_hits',forbidden)
    assert s.snapshot()['ready'] and s.corridor_clear()
    s.action(dict(owner='a'*20,action='start',confirmed=True))
    now=time.monotonic();s.trial.start(now,s.pose,operator_confirmed=True)
    assert any(s.trial.command(now,s.pose,s.pose_stamp,
        sensors_fresh=s.motion_sensors_ready(now),corridor_clear=s.corridor_clear()))


def test_processed_observation_never_renews_old_sensor_timestamps(monkeypatch):
    import navigation_trial_web as web
    s=asynchronous_web_state();stamp=s.scan_stamp
    monkeypatch.setattr(web.time,'monotonic',lambda:stamp+.351)
    assert s.corridor_hits() is None and not s.snapshot()['ready']
    # A later calculation completion is not a new physical observation.
    s.scan_stamp=s.depth_stamp=s.pose_stamp=stamp+.35
    assert s.corridor_hits() is None


def test_route_change_invalidates_processed_collision_proof():
    s=asynchronous_web_state();assert s.corridor_clear()
    s.trial.goal=(.1,.2,1)
    assert s.corridor_hits() is None and not s.snapshot()['ready']


def test_unprocessed_route_waits_for_sensor_without_controller_fault():
    import time
    s=asynchronous_web_state();now=time.monotonic()
    s.trial.start(now,s.pose,operator_confirmed=True);s.corridor_observation=None
    command=s.trial.command(now,s.pose,s.pose_stamp,
        sensors_fresh=s.motion_sensors_ready(now),corridor_clear=s.corridor_clear())
    assert not any(command) and s.trial.state=='WAITING_SENSOR'
    assert not s.snapshot()['ready']


def test_processed_obstacle_still_waits_without_nonzero_command():
    import time
    s=asynchronous_web_state();s.corridor_observation['hits']['depth']=[[.3,0,.5]]
    assert s.snapshot()['ready']  # Start accepts a waiting session only.
    now=time.monotonic();s.trial.start(now,s.pose,operator_confirmed=True)
    assert not any(s.trial.command(now,s.pose,s.pose_stamp,
        sensors_fresh=s.motion_sensors_ready(now),corridor_clear=s.corridor_clear()))
    assert s.trial.state=='WAITING_OBSTACLE'


def test_evidence_timeout_has_distinct_observed_reason():
    from so101_arm_bridge.stationary_telemetry import describe_runtime_fault
    assert '자세 상태 갱신 지연' in describe_runtime_fault({'mobile_board':{'source':1}})
def test_bottom_reference_uses_lift_samples_before_mobile_feedback_exists():
    from types import SimpleNamespace as NS
    from navigation_trial_web import reference_supported_bottom
    clock=[0.];referenced=[];renewals=[]
    def observe():
        holding=bool(referenced) and clock[0]>=referenced[0]+.6
        return NS(flags=29 if holding else 6 if referenced else 12,
            state=5 if holding else 4 if referenced else 0,fault=0,command_raw=0,
            height_um=0,target_um=0,position_raw=3833+int(clock[0]*10)%2,
            mcu_tick_ms=int(clock[0]*1000),observed_ms=int(clock[0]*1000))
    def status():raise AssertionError('unhomed AS feedback must not gate bottom reference')
    def reference(*a,**kw):
        assert clock[0]>=.45 and kw['operator_confirmed'];referenced.append(clock[0])
    reference_supported_bottom(observe,NS(status=status),NS(reference_bottom=reference,keepalive=lambda:renewals.append(clock[0])),
        clock=lambda:clock[0],sleep=lambda t:clock.__setitem__(0,clock[0]+t))
    assert len(referenced)==1 and len(renewals)>20


@pytest.mark.parametrize('problem',['stale','repeated','motion','fault','interlock','command'])
def test_bottom_reference_requires_fresh_stationary_lift_and_mcu_interlocks(problem):
    from types import SimpleNamespace as NS
    from navigation_trial_web import reference_supported_bottom
    clock=[1.]
    def observe():
        tick=int(clock[0]*1000)
        return NS(flags=8 if problem=='interlock' else 12,state=0,
            fault=1 if problem=='fault' else 0,command_raw=1 if problem=='command' else 0,
            position_raw=int(clock[0]*100)%4096 if problem=='motion' else 3833,
            mcu_tick_ms=tick,observed_ms=tick-201 if problem=='stale' else 1000 if problem=='repeated' else tick)
    with pytest.raises(TimeoutError):
        reference_supported_bottom(observe,object(),object(),clock=lambda:clock[0],
            sleep=lambda t:clock.__setitem__(0,clock[0]+t))


@pytest.mark.parametrize('condition',['transient','persistent','stale','rejected'])
def test_mobile_admission_waits_parked_and_never_retries_rejected_arm(monkeypatch,condition):
    from types import SimpleNamespace as NS
    import navigation_trial_web as web
    clock=[0.];calls=[];observations=[]
    monkeypatch.setattr(platform_preparation,'supported_lift_is_holding',lambda _:True)
    def observe():observations.append(clock[0])
    def status():
        speed=50 if condition=='persistent' or clock[0]<.1 else 0
        return NS(velocity_raw=(0,0,0,speed),feedback_tick_ms=int(clock[0]*1000),
                  feedback_is_fresh=lambda age:condition!='stale')
    def arm(session):
        calls.append(session)
        assert clock[0]>=.1
        if condition=='rejected':raise RuntimeError('rejected')
    run=lambda:web.arm_observed_stationary(observe,NS(status=status,arm=arm),
        clock=lambda:clock[0],sleep=lambda t:clock.__setitem__(0,clock[0]+t))
    if condition in ('persistent','stale'):
        with pytest.raises(TimeoutError):run()
        assert calls==[]
    elif condition=='rejected':
        with pytest.raises(RuntimeError,match='rejected'):run()
        assert calls==[1]
    else:run();assert calls==[1]
    assert len(observations)>=6


@pytest.mark.parametrize('fault',['transient','persistent','accepted','boot','stale',
                                'unhealthy','mismatch','expired','zero','unknown'])
def test_initial_arm_only_defers_proven_unarmed_speed_race(monkeypatch,fault):
    from types import SimpleNamespace as NS
    from dataclasses import replace
    from so101_arm_bridge.mobile_wire import MobileStatus,MobileCommand
    from so101_arm_bridge.mobile_client import MobileReplyError
    import navigation_trial_web as web
    now=[0.];attempts=[]
    monkeypatch.setattr(platform_preparation,'supported_lift_is_holding',lambda _:True)
    def status():
        tick=int(now[0]*1000)
        return MobileStatus(5,1,tick,58,7,0,0,0,0,23,tick,(0,0,0,0),0,0)
    def arm(session):
        attempts.append(now[0])
        if fault=='transient' and len(attempts)>1:return
        r=replace(status(),kind=129,request_sequence=0,velocity_raw=(-50,0,0,0))
        updates={'accepted':dict(state=1,session=1),'boot':dict(boot_id=59),
                 'stale':dict(feedback_tick_ms=r.mcu_tick_ms-101),'unhealthy':dict(flags=19),
                 'mismatch':dict(request_sequence=9),'zero':dict(velocity_raw=(0,0,0,0)),
                 'unknown':dict(state=3,reason=1)}
        r=replace(r,**updates.get(fault,{}))
        expiry=r.mcu_tick_ms+(0 if fault=='expired' else 190)
        raise MobileReplyError(MobileCommand(1,1,0,expiry).encode(),0,1,r)
    run=lambda:web.arm_observed_stationary(lambda:None,NS(status=status,arm=arm,boot_id=58),
        clock=lambda:now[0],sleep=lambda t:now.__setitem__(0,now[0]+t))
    if fault=='transient':
        run();assert len(attempts)==2 and attempts[1]-attempts[0]>=.1
    elif fault=='persistent':
        with pytest.raises(TimeoutError):run()
        assert 1<len(attempts)<30 and now[0]<3.03
    else:
        with pytest.raises(MobileReplyError):run()
        assert len(attempts)==1


def test_initial_arm_wait_requires_distinct_samples(monkeypatch):
    from types import SimpleNamespace as NS
    import navigation_trial_web as web
    now=[0.];attempts=[]
    monkeypatch.setattr(platform_preparation,'supported_lift_is_holding',lambda _:True)
    feedback=NS(velocity_raw=(0,0,0,0),feedback_tick_ms=1,feedback_is_fresh=lambda age:True)
    with pytest.raises(TimeoutError):
        web.arm_observed_stationary(lambda:None,NS(status=lambda:feedback,arm=attempts.append),
            clock=lambda:now[0],sleep=lambda t:now.__setitem__(0,now[0]+t))
    assert not attempts



def test_transport_admission_uses_fresh_initial_hold_and_retries_only_busy(monkeypatch):
    import navigation_trial_web as web
    from types import SimpleNamespace as NS
    now=[0.];calls=[]
    monkeypatch.setattr(platform_preparation,'supported_lift_is_holding',lambda s:s=='held')
    def observe():calls.append('measure');return 'held'
    def enter():calls.append('enter');return calls.count('enter')==2
    web.enter_base_transport(observe,NS(enter_transport=enter),clock=lambda:now[0],
        sleep=lambda dt:now.__setitem__(0,now[0]+dt))
    assert calls==['measure','enter','measure','enter']
    def ambiguous():raise RuntimeError('ambiguous reply')
    with pytest.raises(RuntimeError,match='ambiguous'):
        web.enter_base_transport(observe,NS(enter_transport=ambiguous))
    with pytest.raises(RuntimeError,match='리프트'):
        web.enter_base_transport(lambda:None,NS(enter_transport=enter))


def test_transport_arming_needs_only_heartbeat_and_three_wheels(monkeypatch):
    import navigation_trial_web as web
    from types import SimpleNamespace as NS
    now=[0.];armed=[]
    def forbidden(_):raise AssertionError('no lift telemetry during base transport')
    monkeypatch.setattr(platform_preparation,'supported_lift_is_holding',forbidden)
    def feedback():return NS(velocity_raw=(0,0,0,0),feedback_tick_ms=int(now[0]*1000),feedback_is_fresh=lambda _:True)
    web.arm_observed_stationary(lambda:None,NS(status=feedback,arm=armed.append),transport_only=True,
        clock=lambda:now[0],sleep=lambda dt:now.__setitem__(0,now[0]+dt))
    assert armed==[1]


def test_inspection_queries_only_wheels_without_motion_endpoint():
    from navigation_trial_web import inspection_wheel_observation
    from types import SimpleNamespace
    calls=[]
    class Inventory:
        def read_device_inventory(self,bus,motor):
            calls.append((bus,motor));return {'id':motor}
    window=SimpleNamespace(observe=lambda rows,start,end,geometry:(rows,start,end))
    result,rows=inspection_wheel_observation(Inventory(),None,window,clock=lambda:1)
    assert calls==[(0,7),(0,8),(0,9)] and result==(rows,1,1)


def test_fine_candidate_is_collision_bound_and_never_dispatches_nav2_followpath():
    from test_nav2_trial import fine_trial
    s=web_state();s.trial=fine_trial();s.trial.state='RUNNING';s.trial.fine_twist=(0.,-.007,0.)
    control=s.sensor_control()
    assert control['execute'] is False and control['motion_active'] is True
    assert control['fine_twist']==[0.,-.007,0.]
    s.async_corridor=True
    s.corridor_observation=dict(goal=s.trial.goal,pose_stamp=s.pose_stamp,scan_stamp=s.scan_stamp,
        depth_stamp=s.depth_stamp,nav_twist=[0.,0.,0.],hits=dict(lidar=[],depth=[]))
    assert s.corridor_hits() is None
    s.corridor_observation['nav_twist']=control['fine_twist']
    assert s.corridor_hits()==dict(lidar=[],depth=[])
    s.trial._fine_observe(1)
    assert s.sensor_control()['motion_active'] is False
    assert s.corridor_hits() is None  # old nonzero candidate does not match observation phase


def test_departure_logs_spike_late_duplicate_and_new_stationary_window(monkeypatch):
    from so101_arm_bridge.mobile_wire import MobileStatus
    from types import SimpleNamespace as NS
    now = [0.]
    rows = []
    arms = []
    readings = iter([(10,10,0), (30,30,50), (50,50,0), (70,70,0),
                     (180,70,0), (190,190,0)])
    def status():
        mcu,tick,speed = next(readings)
        return MobileStatus(5,1,mcu,2,7,0,0,0,0,31,tick,(0,speed,0,0),654588,0)
    monkeypatch.setattr(platform_preparation, 'supported_lift_is_holding', lambda _:True)
    platform_preparation.arm_observed_stationary(lambda:None,NS(status=status,arm=arms.append),
        clock=lambda:now[0], sleep=lambda dt:now.__setitem__(0,now[0]+dt),
        on_sample=lambda **row:rows.append(row))
    assert arms == [1]
    assert [r['reason'] for r in rows] == ['quiet','nonzero speed','quiet','quiet','stale','quiet']
    assert rows[1]['wheel_velocity_raw'] == [0,50,0]
    assert rows[-2]['sample_age_ms'] == 110
    assert rows[-1]['distinct_samples'] == 3 and rows[-1]['span_ms'] == 140
