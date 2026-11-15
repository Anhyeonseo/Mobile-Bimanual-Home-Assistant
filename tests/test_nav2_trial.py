import json,math,time
from pathlib import Path
import numpy as np
import pytest

from home_robot_tasks.nav2_trial import Nav2Trial,obstacle_grid,nav2_trial_config

ROOT=Path(__file__).resolve().parents[1]


def trial():
    from pulse_mobile_base import PATTERNS
    read=lambda name:json.loads((ROOT/'config/home_assistant'/name).read_text())
    area=dict(feasible_by_yaw=np.ones((72,100,100),bool),resolution_m=.05,origin_xy_m=np.array([-2.5,-2.5]))
    t=Nav2Trial(read('s4_base.nominal.json'),area,patterns=PATTERNS,
        navigation_body=read('navigation_body.fixed.json'),command_profile=read('s4_base.command-candidate.json'),
        controller_maximum_raw=read('s4_unified.policy.json')['maximum_wheel_speed_raw'])
    t.plan((0,0,0));return t


def feedback(t,at,**changes):
    result=dict(ready=True,observed_s=at,goal=list(t.goal),odom_s=at,depth_s=at,
        executing=True,command_s=at,twist=[.016,0,0],result=None,error=None)
    result.update(changes);return result


@pytest.mark.parametrize('remaining',[0.,.05,.14])
def test_explicit_map_goal_can_resume_near_destination_without_extending_target(remaining):
    t=trial();t.target_pose=(remaining,0,.4);t.state='PREVIEW'
    t.plan((0,0,0))
    assert t.goal==t.target_pose
    assert not any(t.command(1,(0,0,0),1,sensors_fresh=True,corridor_clear=True))
    t.target_pose=(t.distance+.01,0,.4);t.state='PREVIEW'
    with pytest.raises(ValueError,match='distance'):t.plan((0,0,0))


def test_nav2_cannot_move_before_operator_start_even_with_valid_command():
    t=trial();t.nav=feedback(t,1)
    assert not any(t.command(1,(0,0,0),1,sensors_fresh=True,corridor_clear=True))
    with pytest.raises(ValueError):t.start(1,(0,0,0),operator_confirmed=False)
    t.start(1,(0,0,0),operator_confirmed=True);t.nav=feedback(t,1.1)
    raw=t.command(1.1,(0,0,0),1.1,sensors_fresh=True,corridor_clear=True)
    assert raw[0]<0 and raw[1]==-raw[0] and raw[2:]==(0,0)
    assert max(map(abs,raw))<=100


@pytest.mark.parametrize('twist',[(.016,0,0),(-.016,.013,.09),(0,-.013,-.09),(.016,.013,.09)])
def test_installed_limit_preserves_combined_three_wheel_direction(twist):
    from home_robot_tasks.base_motion import TeleopCommandModel
    t=trial();original=TeleopCommandModel(json.loads((ROOT/'config/home_assistant/s4_base.command-candidate.json').read_text()))
    original_raw=original.raw_command(twist);limited=t.model.raw_command(twist)
    assert max(map(abs,limited))<=100
    factor=max(map(abs,limited))/max(map(abs,original_raw))
    assert limited==pytest.approx(tuple(v*factor for v in original_raw))


@pytest.mark.parametrize('kind',['expired','wrong_goal','old_epoch','no_action','no_sensor','obstacle','backend_not_ready'])
def test_nav2_gate_never_replays_invalid_movement(kind):
    t=trial();t.start(1,(0,0,0),operator_confirmed=True);t.nav=feedback(t,2)
    if kind=='expired':t.nav['command_s']=1.5
    if kind=='wrong_goal':t.nav['goal']=[99,99,0]
    if kind=='old_epoch':t.nav['command_s']=.99
    if kind=='no_action':t.nav['executing']=False
    if kind=='backend_not_ready':t.nav['ready']=False
    assert not any(t.command(2,(0,0,0),2,sensors_fresh=kind!='no_sensor',corridor_clear=kind!='obstacle'))


def test_nav2_done_requires_pose_agreement_and_then_measured_stop():
    t=trial();t.start(1,(0,0,0),operator_confirmed=True);t.nav=feedback(t,2,result='SUCCEEDED')
    with pytest.raises(ValueError,match='disagrees'):t.command(2,(0,0,0),2,sensors_fresh=True,corridor_clear=True)
    assert not any(t.command(2,t.goal,2,sensors_fresh=True,corridor_clear=False))
    assert t.state=='SETTLING'
    with pytest.raises(ValueError):t.finish(False)
    t.finish(True);assert t.state=='SUCCEEDED'


def test_readiness_requires_fresh_ros_observations():
    t=trial();now=time.monotonic();t.nav=feedback(t,now);assert t.backend_ready()
    t.nav['odom_s']=now-1;assert not t.backend_ready()


def test_depth_grid_excludes_robot_but_keeps_upper_obstacles_and_unknown():
    body=[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]
    grid=obstacle_grid([[0,0,.5],[.5,0,1]],body,pose=(1.,2.,math.pi/2),origin=(-.25,.75))
    assert np.count_nonzero(grid==100)==1 and grid[70,50]==100
    assert np.count_nonzero(grid==-1)==9999
    assert np.all(obstacle_grid([],body)==-1)
    rectangular=obstacle_grid([[.5,0,1],[8,0,1]],body,shape=(40,80),resolution=.05,origin=(-1.,-1.))
    assert rectangular.shape==(40,80) and rectangular[20,30]==100
    assert np.count_nonzero(rectangular==100)==1


def test_config_reuses_lift_footprint_and_depth_without_amcl_or_differential_model():
    import yaml
    template=yaml.safe_load((ROOT/'config/nav2.alohamini.candidate.yaml').read_text())
    cfg=nav2_trial_config(template,ROOT)
    assert '/s4_nav2_trial/amcl' not in cfg
    assert cfg['/s4_nav2_trial/controller_server']['ros__parameters']['enable_stamped_cmd_vel']
    dwb=cfg['/s4_nav2_trial/controller_server']['ros__parameters']['FollowPath']
    assert dwb['max_vel_x']==.008 and dwb['max_vel_theta']==.045
    assert dwb['sim_time']==8.
    local=cfg['/s4_nav2_trial/local_costmap/local_costmap']['ros__parameters']
    assert local['use_maximum'] and 'depth_layer' in local['plugins']
    assert not local['depth_layer']['footprint_clearing_enabled']
    global_map=cfg['/s4_nav2_trial/global_costmap/global_costmap']['ros__parameters']
    assert 'depth_layer' in global_map['plugins'] and global_map['use_maximum']
    assert global_map['depth_layer']['map_topic']=='/s4_nav2_trial/global_depth_grid'
    assert local['footprint_padding']==0.
    body=json.loads((ROOT/'config/home_assistant/navigation_body.fixed.json').read_text())
    poly=np.array(body['footprint_xy_m']);edges=np.roll(poly,-1,axis=0)-poly
    normals=np.column_stack((edges[:,1],-edges[:,0]))/np.linalg.norm(edges,axis=1)[:,None]
    bounds=np.sum(normals*poly,axis=1)
    for costmap in (local,global_map):
        padded=np.array(json.loads(costmap['footprint']))
        assert np.max(padded@normals.T-bounds,axis=0)==pytest.approx(
            np.full(len(poly),body['clearance_m']),abs=1e-8)
    assert local['footprint']!=template['/offline_nav2/local_costmap/local_costmap']['ros__parameters']['footprint']
    for name in ('global_costmap','local_costmap'):
        layer=cfg[f'/s4_nav2_trial/{name}/{name}']['ros__parameters']['obstacle_layer']['scan']
        assert layer['min_obstacle_height']==0. and layer['max_obstacle_height']==1.12
    target=nav2_trial_config(template,ROOT,map_target=True)
    omni=target['/s4_nav2_trial/controller_server']['ros__parameters']['FollowPath']
    assert omni['min_vel_x']==-.008 and omni['max_vel_x']==.008
    assert omni['PathAlign.scale']==omni['GoalAlign.scale']==0
    assert omni['ObstacleFootprint.scale']==dwb['ObstacleFootprint.scale']
    assert omni['RotateToGoal.scale']==dwb['RotateToGoal.scale']
    assert 0<omni['prune_distance']<omni['forward_prune_distance']


def test_nav2_route_wait_is_not_a_controller_fault_and_keeps_goal():
    t=trial();now=time.monotonic();t.start(now,(0,0,0),operator_confirmed=True)
    t.nav=feedback(t,now+.1,wait_reason='blocked route')
    assert not any(t.command(now+.1,(0,0,0),now+.1,sensors_fresh=True,corridor_clear=True))
    assert t.state=='WAITING_OBSTACLE' and t.goal is not None
    from home_robot_tasks.nav2_trial import recoverable_nav2_result
    assert all(recoverable_nav2_result('follow',n) for n in (102,103,104,105,106,107))
    assert not recoverable_nav2_result('follow',101)
    assert recoverable_nav2_result('plan',208) and not recoverable_nav2_result('plan',204)


def test_terminal_nav2_failure_cannot_launch_another_action():
    from home_robot_tasks.nav2_trial import Nav2SensorBridge
    from types import SimpleNamespace
    import threading
    b=Nav2SensorBridge.__new__(Nav2SensorBridge)
    b.error='Nav2 follow path 101: invalid controller';b.goal=[1,0,0]
    b.state=SimpleNamespace(lock=threading.RLock(),nav2_feedback={})
    # The rest of the live bridge is intentionally absent: neither planning,
    # publishing nor action startup is permitted after terminal failure.
    b.tick();b.tick()
    assert b.state.nav2_feedback['error']==b.error
    assert not b.state.nav2_feedback['ready'] and not b.state.nav2_feedback['executing']


def test_invalid_path_discards_old_commands_and_requests_fresh_plan():
    from home_robot_tasks.nav2_trial import Nav2SensorBridge
    from types import SimpleNamespace as NS
    import threading
    b=Nav2SensorBridge.__new__(Nav2SensorBridge)
    b.error=None;b.processes=[];b.publish_pose_depth=lambda:None;b.activate=lambda:None
    b.goal=[1,0,0];b.result=None;b.active='old';b.cancelling=False
    b.command_s=time.monotonic();b.twist=(.008,0,0);b.activated=True
    b.plan_future=None;b.planned_goal=b.goal;b.path=[[0,0,0],[1,0,0]]
    b.retry_after=0;b.wait_reason=None;b.last_pose=b.last_depth=time.monotonic();b.sequence=1
    b.follow=NS(poll=lambda _:NS(state='FAILED',reason='nav2_failed'),
                entries={'old':{'error_code':103,'error_message':'Resulting plan has 0 poses in it.'}},ready=lambda:True)
    b.planner=NS(server_is_ready=lambda:False)
    b.state=NS(lock=threading.RLock(),nav2_control={'goal':b.goal,'execute':True},
               nav2_feedback={},sensors_ready=lambda _:True)
    b.tick()
    assert b.error is None and b.active is None and b.path is None and b.planned_goal is None
    assert b.twist==(0,0,0) and b.command_s<0
    assert b.goal==[1,0,0] and b.state.nav2_feedback['wait_reason']
    assert not b.state.nav2_feedback['ready'] and not b.state.nav2_feedback['executing']


def test_immediate_obstacle_stops_and_cannot_resume_when_it_disappears(monkeypatch):
    t=trial();now=time.monotonic();t.start(now,(0,0,0),operator_confirmed=True)
    clock=[now];monkeypatch.setattr(time,'monotonic',lambda:clock[0])
    t.nav=feedback(t,now+.05)
    assert any(t.command(now+.05,(0,0,0),now+.05,sensors_fresh=True,corridor_clear=True))
    t.nav=feedback(t,now+.1)
    assert not any(t.command(now+.1,(0,0,0),now+.1,sensors_fresh=True,corridor_clear=False))
    assert t.collision_hold and t.resume_required and t.state=='WAITING_OBSTACLE'
    for delay in (.2,.4,.6):
        clock[0]=now+delay
        t.nav=feedback(t,now+delay)
        assert not any(t.command(now+delay,(0,0,0),now+delay,sensors_fresh=True,corridor_clear=True))
    assert t.recovery_ready and t.state=='WAITING_OBSTACLE'
    t.resume(now+.7,(0,0,0),now+.7,operator_confirmed=True,sensors_fresh=True,corridor_clear=True)
    assert not t.collision_hold and t.state=='RUNNING'
    # Old pre-stop output may not be replayed after the explicit resume.
    t.nav=feedback(t,now+.71,command_s=now+.05)
    assert not any(t.command(now+.71,(0,0,0),now+.71,sensors_fresh=True,corridor_clear=True))


def test_planner_wait_is_distinct_from_immediate_collision_hold():
    t=trial();now=time.monotonic();t.start(now,(0,0,0),operator_confirmed=True)
    t.nav=feedback(t,now+.1,wait_reason='replanning',ready=False)
    assert not any(t.command(now+.1,(0,0,0),now+.1,sensors_fresh=True,corridor_clear=True))
    assert not t.collision_hold and not t.resume_required


def test_early_stop_zone_extends_slow_motion_but_not_side_or_rear():
    from navigation_trial_web import State
    t=trial();s=State(t,None);s.pose=[0,0,0]
    s.nav2_feedback={'collision_twist':[.008,0,0]}
    front=max(p[0] for p in t.navigation_body['footprint_xy_m'])
    assert s.compute_corridor_hits(s.pose,t.goal,[[front+.17,0]],[])['lidar']
    assert not s.compute_corridor_hits(s.pose,t.goal,[[front+.30,0],[0,.6],[-.5,0]],[])['lidar']
    assert math.hypot(*t.obstacle_sweep((.008,0,0))[-1][:2])==pytest.approx(.15)
    assert math.hypot(*t.sweep((.008,0,0))[-1][:2])<.02


def test_nav2_zero_retains_only_collision_direction_for_bounded_interval():
    from home_robot_tasks.nav2_trial import Nav2SensorBridge
    b=Nav2SensorBridge.__new__(Nav2SensorBridge)
    b.twist=(0,0,0);b.last_motion_s=10.;b.last_motion_twist=(.008,0,0)
    assert b.collision_twist(10.3)==(.008,0,0)
    assert b.twist==(0,0,0)
    assert b.collision_twist(10.51)==(0,0,0)


def test_longer_obstacle_trial_keeps_speed_and_requires_explicit_start():
    t=trial();old_limit=t.model.limit
    from navigation_trial_web import make_trial
    from types import SimpleNamespace
    read=lambda n:json.loads((ROOT/'config/home_assistant'/n).read_text())
    t=make_trial(SimpleNamespace(nav2=True,nav2_distance=1.2,nav2_goal=(1.19,0,0)),
        read('s4_base.nominal.json'),t.area,read('navigation_body.fixed.json'))
    t.plan((0,0,0));assert t.model.limit==old_limit and t.active_timeout_s==540
    t.nav=feedback(t,1)
    assert not any(t.command(1,(0,0,0),1,sensors_fresh=True,corridor_clear=True))
    assert not t.route_valid((1.5,0,0))


def test_stale_ros_tf_or_depth_also_stops_otherwise_fresh_sensor_input():
    t=trial();t.start(1,(0,0,0),operator_confirmed=True);t.nav=feedback(t,2,odom_s=1.)
    assert not any(t.command(2,(0,0,0),2,sensors_fresh=True,corridor_clear=True))
    assert t.state=='WAITING_SENSOR'


def test_shorter_preview_uses_same_nav2_gates_and_displays_actual_distance():
    from types import SimpleNamespace
    from navigation_trial_web import make_trial,State
    source=trial()
    read=lambda n:json.loads((ROOT/'config/home_assistant'/n).read_text())
    t=make_trial(SimpleNamespace(nav2=True,nav2_distance=.3),read('s4_base.nominal.json'),
                 source.area,read('navigation_body.fixed.json'))
    t.plan((0,0,0));assert math.dist(t.goal[:2],(0,0))==pytest.approx(.3)
    assert '30cm' in State(t,None).snapshot()['scope']
    t.nav=feedback(t,1)
    assert not any(t.command(1,(0,0,0),1,sensors_fresh=True,corridor_clear=True))


def fixed_target(goal):
    from types import SimpleNamespace
    from navigation_trial_web import make_trial
    read=lambda n:json.loads((ROOT/'config/home_assistant'/n).read_text())
    return make_trial(SimpleNamespace(nav2=True,nav2_distance=.4,nav2_goal=goal),
        read('s4_base.nominal.json'),trial().area,read('navigation_body.fixed.json'))


def test_fixed_map_goal_returns_without_turning_body_to_reverse_heading():
    from navigation_trial_web import State
    t=fixed_target((-.3,0,-math.pi/6));path=t.plan((0,0,0))
    assert path==[]  # no invented straight preview before the Nav2 response
    t.nav=feedback(t,1,path=[[0,0,0],[-.3,0,-math.pi/6]])
    assert t.path((0,0,0),t.goal)==t.nav['path']
    assert '오른쪽 30°' in State(t,None).snapshot()['scope']
    t.nav=feedback(t,1,twist=[-.008,0,-.02])
    assert not any(t.command(1,(0,0,0),1,sensors_fresh=True,corridor_clear=True))
    t.start(1,(0,0,0),operator_confirmed=True)
    t.nav=feedback(t,1.1,twist=[-.008,0,-.02])
    assert any(t.command(1.1,(0,0,0),1.1,sensors_fresh=True,corridor_clear=True))
    assert not any(t.command(1.2,(0,0,0),1.2,sensors_fresh=True,corridor_clear=False))


@pytest.mark.parametrize('goal',[(.41,0,0),(-.41,0,0),(-.3,0,math.pi/2)])
def test_fixed_map_goal_retains_distance_and_turn_bounds(goal):
    with pytest.raises(ValueError,match='bounded trial'):fixed_target(goal).plan((0,0,0))


def test_fixed_map_goal_allows_planner_detour_but_rejects_occupied_endpoints():
    t=fixed_target((-.3,0,-math.pi/6));t.area['feasible_by_yaw'][:,:,47]=False
    assert t.plan((0,0,0))==[] and t.goal==(-.3,0,-math.pi/6)
    # The wall blocks the direct segment; actual Nav2 still has to find a path.
    assert not t.backend_ready()
    t.area['feasible_by_yaw'][:,:,44]=False
    with pytest.raises(ValueError,match='지도 장애물'):t.plan((0,0,0))
    t.area['feasible_by_yaw'][:]=True;t.plan((0,0,0));t.state='SUCCEEDED'
    with pytest.raises(ValueError,match='consumed'):t.plan((0,0,0))


def test_nav2_live_pose_cell_boundary_is_not_a_second_collision_authority():
    t=fixed_target((-.3,0,-math.pi/6))
    # A parked pose snaps to an offline blocked centre, while the destination
    # is valid. No output exists until actual Nav2 planning and explicit start.
    t.area['feasible_by_yaw'][:,:,50]=False
    t.plan((.01,0,0));assert not t.backend_ready()
    t.start(1,(.01,0,0),operator_confirmed=True)
    t.nav=feedback(t,1.1,twist=[-.008,0,0])
    assert any(t.command(1.1,(.009,0,0),1.1,sensors_fresh=True,corridor_clear=True))
    # Real obstacle evidence remains independent and latches user resumption.
    assert not any(t.command(1.2,(.009,0,0),1.2,sensors_fresh=True,corridor_clear=False))
    assert t.collision_hold and t.resume_required


def test_nav2_retains_trial_envelope_without_offline_cell_veto():
    t=trial();t.start(1,(0,0,0),operator_confirmed=True);t.nav=feedback(t,1.1)
    with pytest.raises(ValueError,match='envelope'):
        t.command(1.1,(1,0,0),1.1,sensors_fresh=True,corridor_clear=True)


def test_changed_map_does_not_admit_old_revision_velocity():
    t=trial();now=time.monotonic();t.start(now,(0,0,0),operator_confirmed=True)
    t.required_map_revision='new';t.nav=feedback(t,now+.1,map_revision='old')
    assert not t.backend_ready()
    assert not any(t.command(now+.1,(0,0,0),now+.1,sensors_fresh=True,corridor_clear=True))
    t.nav=feedback(t,time.monotonic(),map_revision='new')
    assert t.backend_ready()


def test_normal_offset_keeps_physical_clearance_without_axis_corner_overgrowth():
    from home_robot_tasks.nav2_trial import offset_convex_footprint
    import json
    body=json.loads((ROOT/'config/home_assistant/navigation_body.fixed.json').read_text())
    poly=np.array(body['footprint_xy_m']);gap=.03+math.sqrt(2)*.05/2
    padded=np.array(offset_convex_footprint(poly,gap))
    edges=np.roll(poly,-1,axis=0)-poly
    normals=np.column_stack((edges[:,1],-edges[:,0]))/np.linalg.norm(edges,axis=1)[:,None]
    bounds=np.sum(normals*poly,axis=1)
    assert np.max(padded@normals.T-bounds,axis=0)==pytest.approx(np.full(len(poly),gap),abs=1e-8)
    # Previous sign padding inflated diagonal sides more than the specified gap.
    previous=poly+np.sign(poly)*gap
    assert np.max(previous@normals.T-bounds)>gap*1.35
    # The independent 3 cm body clearance remains fully enclosed.
    physical=np.array(offset_convex_footprint(poly,.03))
    assert np.all(physical@normals.T<=bounds+gap+1e-9)


def test_explicit_alignment_turn_and_arrival_do_not_change_transit():
    from pulse_mobile_base import PATTERNS
    t=trial();t.target_pose=(.2,0,math.radians(67));t.state='PREVIEW'
    with pytest.raises(ValueError,match='turn'):t.plan((0,0,0))
    read=lambda n:json.loads((ROOT/'config/home_assistant'/n).read_text())
    a=Nav2Trial(read('s4_base.nominal.json'),t.area,patterns=PATTERNS,navigation_body=t.navigation_body,
        command_profile=read('s4_base.command-candidate.json'),controller_maximum_raw=100,
        target_pose=t.target_pose,manipulation_alignment=True)
    a.plan((0,0,0));a.start(1,(0,0,0),operator_confirmed=True);a.nav=feedback(a,2,result='SUCCEEDED')
    with pytest.raises(ValueError,match='disagrees'):a.command(2,(.15,0,a.goal[2]),2,sensors_fresh=True,corridor_clear=True)
    assert not any(a.command(2,(.18,0,a.goal[2]-.05),2,sensors_fresh=True,corridor_clear=True))
    assert a.state=='SETTLING'
    a.state='PREVIEW';a.target_pose=(.2,0,math.radians(91))
    with pytest.raises(ValueError,match='turn'):a.plan((0,0,0))


def test_alignment_changes_goal_tolerance_and_planned_speed_floor_only():
    import yaml
    template=yaml.safe_load((ROOT/'config/nav2.alohamini.candidate.yaml').read_text())
    normal=nav2_trial_config(template,ROOT,map_target=True)
    aligned=nav2_trial_config(template,ROOT,map_target=True,manipulation_alignment=True)
    c=aligned['/s4_nav2_trial/controller_server']['ros__parameters']
    assert c['goal_checker']['xy_goal_tolerance']==.025
    assert c['goal_checker']['yaw_goal_tolerance']==.07
    c['goal_checker'].update(xy_goal_tolerance=.1,yaw_goal_tolerance=.17)
    assert c['FollowPath']['min_speed_xy']==.0045
    assert c['FollowPath']['min_speed_theta']==.025
    c['FollowPath'].update(xy_goal_tolerance=.1,min_speed_xy=0.,min_speed_theta=0.)
    assert aligned['/s4_nav2_trial/planner_server']['ros__parameters']['GridBased']['tolerance']==0.
    aligned['/s4_nav2_trial/planner_server']['ros__parameters']['GridBased']['tolerance']=.1
    assert aligned==normal


def test_alignment_cannot_follow_a_nearby_substitute_goal():
    from home_robot_tasks.nav2_trial import alignment_goal_matches
    assert alignment_goal_matches([[0,0,0],[.1,.2,.3]],[.1,.2,.3])
    assert not alignment_goal_matches([[0,0,0],[.05,.15,.3]],[.1,.2,.3])
    assert not alignment_goal_matches([], [.1,.2,.3])
    assert not alignment_goal_matches([[float('nan'),.2,.3]], [.1,.2,.3])


def test_alignment_speed_floor_rejects_observed_stall_without_post_scaling():
    import yaml
    from home_robot_tasks.base_motion import TeleopCommandModel
    template=yaml.safe_load((ROOT/'config/nav2.alohamini.candidate.yaml').read_text())
    cfg=nav2_trial_config(template,ROOT,map_target=True,manipulation_alignment=True)
    dwb=cfg['/s4_nav2_trial/controller_server']['ros__parameters']['FollowPath']
    model=TeleopCommandModel(json.loads((ROOT/'config/home_assistant/s4_base.command-candidate.json').read_text()))
    model.limit=100
    # Match Jazzy KinematicParameters::isValidSpeed (including float epsilon).
    def accepted(twist):
        x,y,w=twist;v2=x*x+y*y;eps=1.1920929e-7
        return v2<=dwb['max_speed_xy']**2+eps and not (
            v2+eps<dwb['min_speed_xy']**2 and abs(w)+eps<dwb['min_speed_theta']) and any(twist)
    for raw in ((12,12,-23),(-16,16,0),(4,-28,23)):
        assert not accepted(model.expected_twist(raw))
    for twist in ((.0045,0,0),(-.0045,0,0),(0,.0045,0),(0,-.0045,0),(0,0,.025),(0,0,-.025)):
        assert accepted(twist)
        assert max(abs(v) for v in model.raw_command(twist))<=100
    assert not accepted((0,0,0))
    assert model.raw_command((0,0,0))==(0,0,0)  # Explicit STOP stays zero.


def fine_trial():
    t=trial();t.manipulation_alignment=True;t.fine_alignment=True
    t.target_pose=(.05,0,0);t.state='PREVIEW';t.arrival_xy=.01;t.arrival_yaw=math.radians(1)
    t.plan((0,0,0));return t


def test_fine_alignment_requires_stationary_start_and_bounded_goal():
    t=fine_trial()
    with pytest.raises(ValueError,match='정지'):t.start(1,(0,0,0),operator_confirmed=True)
    t.stationary_report=dict(ready=True,observed_s=1)
    t.start(1,(0,0,0),operator_confirmed=True)
    t=fine_trial();t.state='PREVIEW';t.target_pose=(.16,0,0)
    with pytest.raises(ValueError,match='15cm'):t.plan((0,0,0))


def test_fine_alignment_requires_post_motion_window_not_single_nav2_success():
    t=fine_trial();t.state='RUNNING';t.nav=feedback(t,2,result='SUCCEEDED')
    assert not any(t.command(2,(.05,0,0),2,sensors_fresh=True,corridor_clear=True))
    assert t.state=='VERIFYING_ALIGNMENT'
    t.stationary_report=dict(ready=True,observed_s=2.1,span_s=1.5,pose=[.05,0,0])
    t.command(2.1,(.05,0,0),2.1,sensors_fresh=True,corridor_clear=True)
    assert t.state=='VERIFYING_ALIGNMENT'  # old/pre-motion window not evidence
    t.stationary_report=dict(ready=True,observed_s=4,span_s=1.5,pose=[.055,0,.01])
    t.command(4,(.055,0,.01),4,sensors_fresh=True,corridor_clear=True)
    assert t.state=='SETTLING' and t.fine_verified
    t.finish(True);assert t.state=='SUCCEEDED'


def test_fine_alignment_unstable_or_outside_stops_for_review_not_false_success():
    for unstable in (True,False):
        t=fine_trial();t.state='VERIFYING_ALIGNMENT';t.verification_started=1
        if not unstable:t.stationary_report=dict(ready=True,observed_s=4,span_s=1.5,pose=[.075,0,0])
        now=10 if unstable else 4
        assert not any(t.command(now,(.075,0,0),now,sensors_fresh=True,corridor_clear=True))
        assert t.state=='SETTLING' and not t.fine_verified
        t.finish(True);assert t.state=='ALIGNMENT_REVIEW'


def test_fine_nav2_config_changes_only_final_checker_not_collision_or_velocity():
    import yaml
    template=yaml.safe_load((ROOT/'config/nav2.alohamini.candidate.yaml').read_text())
    regular=nav2_trial_config(template,ROOT,map_target=True,manipulation_alignment=True)
    fine=nav2_trial_config(template,ROOT,map_target=True,manipulation_alignment=True,fine_alignment=True)
    c=fine['/s4_nav2_trial/controller_server']['ros__parameters']
    assert c['goal_checker']['xy_goal_tolerance']==.008 and c['goal_checker']['stateful'] is False
    c['goal_checker']=regular['/s4_nav2_trial/controller_server']['ros__parameters']['goal_checker']
    c['FollowPath']['xy_goal_tolerance']=.025
    assert fine==regular


def test_fine_alignment_rejects_stationary_window_from_replaced_map():
    t=fine_trial();t.required_map_revision='new'
    t.stationary_report=dict(ready=True,observed_s=1,map_revision='old')
    with pytest.raises(ValueError,match='정지'):t.start(1,(0,0,0),operator_confirmed=True)
    t.state='VERIFYING_ALIGNMENT';t.verification_started=1
    t.stationary_report.update(observed_s=4,span_s=1.5,pose=list(t.goal))
    assert not any(t.command(4,t.goal,4,sensors_fresh=True,corridor_clear=True))
    assert t.state=='VERIFYING_ALIGNMENT' and not t.fine_verified
