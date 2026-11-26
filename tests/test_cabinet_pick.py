from contextlib import nullcontext
import json
from threading import Event
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from home_robot_tasks.pick_service import PickStation, Rejected
from home_robot_tasks.pick_web_backend import WebPickBackend
from tools.run import cabinet_navigation as navigation


@pytest.mark.parametrize('park_fails,late_arm', [(False, False), (True, False), (False, True)])
def test_only_confirmed_park_hands_back_live_owner(monkeypatch, tmp_path, park_fails, late_arm):
    trial = MagicMock(state='READY', resume_required=False)
    trial.goal = [1., 0., 0.]
    trial.WAITING = ()
    trial.geometry.body_twist.return_value = [0., 0., 0.]
    def command(*args, **kwargs):
        trial.state = 'SETTLING'
        return (0, 0, 0, 0)
    trial.command.side_effect = command
    state = MagicMock(closed=Event(), pose=[1., 0., 0.], lock=nullcontext())
    state.snapshot.return_value = {'ready': True}
    monkeypatch.setattr(navigation, 'State', lambda *_: state)
    monkeypatch.setattr(navigation, 'sensor_bridge_worker', lambda state, _: state.closed.wait(5))
    monkeypatch.setattr(navigation, 'arm_observed_stationary', lambda observe, mobile, **kwargs: observe())
    monkeypatch.setattr(navigation, 'settle_wheels', lambda mobile, observe, **_: observe())
    monkeypatch.setattr(navigation, 'supported_lift_is_holding', lambda _: True)
    hw = MagicMock(stopped=False)
    hw.guard.supported_top = False
    hw.guard.current_height = 0.
    hw.observe_preparation.return_value.height_um = 0
    hw.mobile.status.return_value = SimpleNamespace(capabilities=8, feedback_is_fresh=lambda _:True,
        feedback_tick_ms=123, mcu_tick_ms=123, boot_id=4, velocity_raw=(0,0,0,0))
    if late_arm:
        sample = hw.observe_preparation.return_value
        hw.observe_preparation.side_effect = [sample, navigation.ObservationNotFresh('slow'), sample, sample, sample]
        trial.start.side_effect = lambda *args, **kwargs: setattr(trial, 'state', 'RUNNING')
        reply = SimpleNamespace(**vars(hw.mobile.status.return_value),
                                ready_for_motion=lambda *_: True)
        hw.mobile.velocity.return_value = reply
    hw.mobile.park.return_value = SimpleNamespace(boot_id=4)
    hw.mobile.session = 8
    hw.mobile.sequence = 7
    ctx = MagicMock()
    ctx.wait.side_effect = lambda future: future.result(timeout=2)
    args = SimpleNamespace(output=tmp_path)
    if park_fails:
        hw.mobile.park.side_effect = RuntimeError('parking failed')
        with pytest.raises(RuntimeError, match='parking failed'):
            navigation.drive_to_view(ctx, hw, args, trial)
        hw.abort.assert_called_once()
        trial.finish.assert_not_called()
    else:
        navigation.drive_to_view(ctx, hw, args, trial)
        trial.finish.assert_called_once_with(True)
        hw.guard.require_fresh.assert_called_once()
        hw.abort.assert_not_called()
        assert json.loads(next(tmp_path.glob('*.jsonl')).read_text().splitlines()[-1])['event']=='parked'
    if late_arm:
        commands = [c.args[0] for c in hw.mobile.velocity.call_args_list]
        assert commands == [(0,0,0,0), (0,0,0,0)]
        trial.pause.assert_called_once()
        events = [json.loads(l)['event'] for l in next(tmp_path.glob('*.jsonl')).read_text().splitlines()]
        assert events.index('arm_observation_wait') < events.index('arm_observation_recovered') < events.index('parked')
    assert state.closed.is_set() and not hw.base_moving


@pytest.mark.parametrize('nav_succeeds', [False, True])
@pytest.mark.parametrize('upper', [False, True])
def test_station_binds_pick_to_navigation_completion(tmp_path, nav_succeeds, upper):
    backend = MagicMock(mode='station', options={})
    backend.ready.return_value = (True, '')
    backend.check_operation.side_effect = lambda op, *_: op
    def passthrough(ctx, request):
        return request
    steps = ('navigate_cabinet','prepare_cabinet_height','capture','detect','candidates',
             'plan_with_reposition','execute','verify')
    if upper:
        steps = tuple(s for s in steps if s != 'prepare_cabinet_height')
    for name in steps:
        getattr(backend, name).side_effect = passthrough
    backend.verify.side_effect = lambda *_: dict(status='UNVERIFIED', physical_success=False,
                                                 reason='operator observation')
    if not nav_succeeds:
        backend.navigate_cabinet.side_effect = RuntimeError('route unavailable')
    station = PickStation(backend, tmp_path)
    station.data['accepting'] = True
    args = dict(operation='cabinet-pick', supported_empty_bottom=not upper, supported_empty_top=upper)
    with pytest.raises(Rejected):
        station.submit('흰 지우개', initial_pose=[float('nan'),0,0], **args)
    request_id = station.submit('흰 지우개', initial_pose=[0,0,0], **args)
    station._run_request(station.pending)
    report = json.loads((tmp_path/request_id/'result.json').read_text())
    assert [stage['id'] for stage in report['stages']] == [
        'NAVIGATING', *([] if upper else ['LIFTING']), 'CAPTURING','DETECTING','GENERATING_GRASPS','PLANNING','EXECUTING','VERIFYING']
    calls = [name for name, _, _ in backend.method_calls if name in steps]
    if nav_succeeds:
        assert calls == list(steps)
        assert report['result']['status'] == 'UNVERIFIED'
    else:
        assert calls == ['navigate_cabinet']
        assert report['result']['status'] == 'FAILED'


def test_upper_capture_comes_from_current_hardware(tmp_path):
    backend = WebPickBackend({'mode':'station','device':'cpu'})
    backend.worker = object()
    backend.hardware = MagicMock()
    ctx = MagicMock(folder=tmp_path)
    backend._capture_held = MagicMock(return_value={'snapshot':'new-upper-snapshot'})
    request = dict(operation='cabinet-pick', supported_empty_bottom=True)
    backend.hardware.guard.supported_top = False
    with pytest.raises(Rejected):
        backend.capture(ctx, request)
    backend.hardware.guard.supported_top = True
    assert backend.capture(ctx, request)['snapshot']=='new-upper-snapshot'
    backend._capture_held.assert_called_once_with(ctx, dict(request, camera_mode='near'))


def test_top_route_rechecks_held_posture():
    observed = {side: list(range(100, 106)) for side in ('left', 'right')}
    inventory = dict(all_snapshots_complete=True, devices=[
        dict(bus=side, requested_id=i, feedback=dict(position_raw=value))
        for side, joints in observed.items() for i, value in enumerate(joints, 1)])
    body = dict(raw=observed, lift_position_m=.65, calibration_id='field')
    hardware = SimpleNamespace(travel_inventory=inventory, profile=dict(calibration_id='field'),
                               guard=SimpleNamespace(supported_top=True, current_height=.65))
    navigation.bind_upper_start(hardware, body)
    inventory['devices'][1]['feedback']['position_raw'] += 32
    with pytest.raises(ValueError, match='팔 자세가 바뀌어'):
        navigation.bind_upper_start(hardware, body)
    hardware.guard.supported_top = False
    with pytest.raises(ValueError, match='상단 실제 높이'):
        navigation.bind_upper_start(hardware, body)


def test_separate_arms_leave_free_space_between_them():
    import cv2
    import numpy as np
    from tools.run.cabinet_upper_body import padded_outline
    from tools.diagnostics.review_s4_sensor_capture import footprint_kernel

    def rectangle(x0, x1, y0, y1):
        return np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])
    parts = [rectangle(-.20,.10,-.18,.18), rectangle(.05,.26,.09,.15),
             rectangle(.05,.26,-.15,-.09)]
    contour = np.array(padded_outline(parts,.03),dtype=np.float32)
    kernel = footprint_kernel(parts,.01,0.,.03)
    cy,cx = np.array(kernel.shape)//2
    for x,y,blocked in [(.22,0.,False),(.22,.12,True),(.22,-.12,True),(.22,.17,True),(0.,0.,True)]:
        assert bool(kernel[cy-round(y/.01),cx-round(x/.01)]) == blocked
        assert (cv2.pointPolygonTest(contour,(x,y),False)>=0) == blocked



def test_retry_after_stop_recreates_planner_before_acquiring_hardware():
    from home_robot_tasks.pick_backend import PipelineBackend
    events = []
    backend = WebPickBackend(dict(mode='station'), hardware_factory=lambda options: events.append('hardware'))
    def fail_worker(ctx):
        events.append('worker')
        raise RuntimeError('model startup')
    backend._prepare_worker = fail_worker
    with pytest.raises(RuntimeError, match='model startup'):
        PipelineBackend._prepare_hardware(backend, MagicMock(), {})
    assert events == ['worker']


def test_planar_positions_rotate_with_base_and_check_entire_sweep():
    import math
    from tools.run.cabinet_navigation import placement_candidates
    # Forward in the base is +map Y. A thin obstacle cuts off the far side.
    trial = SimpleNamespace(map_free=lambda p: not .06 <= p[1] <= .09)
    rows = placement_candidates(trial, (0., 0., math.pi/2), [.53, .04, .75], [.34, .12], math.pi/2)
    assert rows and all(row['goal'][1] < .06 for row in rows)
    assert rows[0]['delta'] == pytest.approx([.05, -.08])
    assert not rows[0]['lateral_only']
    assert rows[0]['goal'][0] == pytest.approx(-rows[0]['delta'][1])
    assert any(row['delta'][0] < 0 for row in rows)


@pytest.mark.parametrize('drift,repeat,expected', [(False,False,True),(True,False,False),(False,True,False)])
def test_settle_position_window_requires_distinct_stationary_encoders(drift,repeat,expected):
    from home_robot_tasks.base_alignment import WheelRestWindow
    rest = WheelRestWindow()
    for i in range(15):
        tick = 1000+i*100
        reply = SimpleNamespace(mcu_tick_ms=tick, velocity_raw=(0,0,50,0), feedback_is_fresh=lambda _:True)
        snap = dict(firmware_tick_ms=1100 if repeat else tick, sample_age_ms=[20,10,0],
                    positions_raw=[300,500,600+(i if drift else i%2)],present_mask=7)
        rest.observe(reply,snap)
    assert rest.proven(tick,900) is expected


def test_view_inside_arrival_window_waits_for_scan_proof_without_arming(monkeypatch, tmp_path):
    import time
    start = [1.00003, -.53781, .96785]
    goal = [1.06785, -.60444, .78531]
    trial = MagicMock(arrival_xy=.12, arrival_yaw=.2, required_map_revision=None)
    state = MagicMock(pose=start, closed=Event(), lock=nullcontext())
    state.stationary_localization = {'ready':False}
    state.snapshot.return_value = {'ready':True}
    monkeypatch.setattr(navigation, 'State', lambda *_: state)
    monkeypatch.setattr(navigation, 'sensor_bridge_worker', lambda state,_: state.closed.wait(3))
    hardware = MagicMock(stopped=False)
    hardware.guard.supported_top = True
    hardware.mobile.status.return_value = SimpleNamespace(capabilities=8, state=0, reason=0, session=0,
        flags=31, feedback_is_fresh=lambda _:True, velocity_raw=(0,0,0,0), boot_id=223,
        feedback_tick_ms=123, mcu_tick_ms=123)
    ctx = MagicMock()
    ctx.pause.side_effect = lambda _: setattr(state,'stationary_localization', dict(ready=True,observed_s=time.monotonic()))
    ctx.wait.side_effect = lambda future: future.result(timeout=2)
    args = SimpleNamespace(navigation_body={}, manipulation_alignment=False, output=tmp_path, nav2_goal=goal)
    assert navigation.drive_to_view(ctx, hardware, args, trial) == start
    ctx.pause.assert_called_once()
    trial.start.assert_not_called()
    hardware.mobile.arm.assert_not_called()
    hardware.mobile.velocity.assert_not_called()


def test_lidar_cabinet_front_uses_finite_surface_and_mapped_lateral_anchor():
    import numpy as np
    points = np.column_stack([np.full(61, .52), np.linspace(-.30, .30, 61)])
    points[8, 0] += .035  # Single mixed return.
    face = navigation.cabinet_front(points, [0, 0, 0], [.55, .08], 0.)
    assert face['point'] == pytest.approx([.52, .08], abs=.003)
    assert face['yaw'] == pytest.approx(0., abs=.01)
    # Occluded anchor, adjacent perpendicular surface, or two plausible faces.
    assert navigation.cabinet_front(points[:20], [0,0,0], [.55,.18], 0.) is None
    assert navigation.cabinet_front(points[:, ::-1], [0,0,0], [0,.52], 0.) is None
    assert navigation.cabinet_front(np.vstack((points, points+[.09,0])), [0,0,0], [.55,.08], 0.) is None


def test_front_view_is_map_clear_and_not_a_grasp_pose():
    trial = SimpleNamespace(navigation_body={'footprint_xy_m': [[.26, 0]], 'padded_outline_xy_m': [[.29, 0]]},
        area={'resolution_m': .05, 'origin_xy_m':[0.,0.]}, arrival_xy=.03, arrival_yaw=.1, maximum_turn=1., distance=.65,
        map_free=lambda p: p[0] < .12)
    pose = [0., 0., 0.]
    face = dict(point=[.42,.08], yaw=0., lateral_m=[-.15,.20])
    goal = next(navigation.front_view_goals(trial, pose, face, dict(view_xy_yaw=[.09,0.,0.],table_height_m=.8)))
    assert .08 <= goal[0] < .12 and goal[1:] == pytest.approx([0., 0.])
    trial.map_free = lambda _: False
    assert not list(navigation.front_view_goals(trial, pose, face, dict(view_xy_yaw=[.09,0.,0.],table_height_m=.8)))


@pytest.mark.parametrize('rejections', [0, 1, 2, 3])
def test_cabinet_requires_executable_plan_from_latest_park(tmp_path, monkeypatch, rejections):
    from home_robot_tasks.pick_service import PlanningRejected
    backend = WebPickBackend({'mode':'station'})
    args, trial = SimpleNamespace(observed_cabinet_face={'yaw':1.5707963267948966, 'point':[0.,.30]}), SimpleNamespace(arrival_xy=.03,navigation_body={'footprint_xy_m':[[0.,.25]]})
    backend.cabinet_navigation = (args, trial, [0.,0.,0.])
    ctx = MagicMock(folder=tmp_path)
    ctx.station.data = {'current': {}}
    def observation(i):
        return dict(operation='cabinet-pick', supported_empty_top=True, prompt='eraser',
                    snapshot=f'capture-{i}', camera_mode='near', prepared=f'prepared-{i}')
    records = []
    def plan(ctx, value):
        i = len(records)
        assert value == observation(i)
        records.append(value['snapshot'])
        if i < rejections:
            raise PlanningRejected('unreachable', dict(base_reposition_candidate=True, capture_id=records[-1]))
        return dict(value, bundle='checked-'+records[-1], compare_lateral=False)
    monkeypatch.setattr(backend, 'plan', plan)
    positions = []
    def candidates(_, pose, target, reference, face_yaw):
        positions.append(pose[0])
        return [dict(delta=[.1,0.], goal=[pose[0]+.1,0.,0.])]
    monkeypatch.setattr(navigation, 'placement_candidates', candidates)
    def compute(ctx, method, **kwargs):
        if method == 'reposition_target':
            return [.45,.12,.8] if rejections else [.341,.124,.8]
        assert method == 'cabinet_placement'
        return dict(capture_id=records[-1], selected=kwargs['candidates'][0])
    monkeypatch.setattr(backend, '_compute', compute)
    monkeypatch.setattr(navigation, 'drive_to_pick_pose', lambda ctx,hw,nav,selected:
                        dict(parked=True, pose=[nav[2][0]+.09,0.,0.]))
    monkeypatch.setattr(backend, '_observe_after_park', lambda ctx,value: (ctx,observation(len(records))))
    if rejections < 3:
        result = backend.plan_with_reposition(ctx, observation(0))
        assert result['bundle'] == f'checked-capture-{rejections}'
    else:
        with pytest.raises(PlanningRejected):
            backend.plan_with_reposition(ctx, observation(0))
    assert positions == pytest.approx([0.,.09][:min(rejections,2)])


@pytest.mark.parametrize('status,moves', [('NO_DETECTIONS', 1), ('DETECTED_NO_DEPTH', 0)])
def test_cabinet_search_moves_only_for_missing_object(tmp_path, monkeypatch, status, moves):
    from home_robot_tasks.pick_backend import PipelineBackend
    from home_robot_tasks.pick_service import Context
    backend = WebPickBackend({'mode':'station'})
    args = SimpleNamespace(observed_cabinet_face=dict(point=[.45,0.], yaw=0., lateral_m=[-.32,.36]))
    backend.cabinet_navigation = (args, SimpleNamespace(map_free=lambda _: True), [0.,0.,0.])
    station = PickStation(backend, tmp_path)
    station.data['current'] = dict(id='test', artifacts=[])
    ctx = Context(station, tmp_path)
    request = dict(operation='cabinet-pick', supported_empty_top=True, snapshot='original')
    def detect(_,ctx,value):
        if value['snapshot'] == 'original':
            raise Rejected('target detection failed: '+status)
        return dict(value, detections='fresh')
    monkeypatch.setattr(PipelineBackend, 'detect', detect)
    monkeypatch.setattr(backend, 'capture', lambda ctx,req: dict(req,snapshot='other-side'))
    calls = []
    def move(ctx,hw,nav,shift):
        calls.append(shift)
        assert shift['delta'][1] > 0
        return dict(pose=shift['goal'])
    monkeypatch.setattr(navigation, 'drive_to_pick_pose', move)
    if moves:
        assert backend.detect(ctx, request)['snapshot'] == 'other-side'
        assert backend.cabinet_navigation[2] == calls[-1]['goal']
        with pytest.raises(Rejected):
            backend.detect(ctx, dict(request, after_pick_alignment=True))
    else:
        with pytest.raises(Rejected):
            backend.detect(ctx, request)
    assert len(calls) == moves


def test_front_slot_can_approach_beside_blocked_cabinet_center():
    trial = SimpleNamespace(navigation_body={'footprint_xy_m': [[.25,.12],[.25,-.12]], 'padded_outline_xy_m': [[.28,.12],[.28,-.12]]},
        area={'resolution_m': .05, 'origin_xy_m':[0.,0.]}, arrival_xy=.03, arrival_yaw=.1, maximum_turn=1., distance=.65,
        map_free=lambda p: p[0] < .12 or p[1] <= -.09)
    face = dict(point=[.49,0.], yaw=0., lateral_m=[-.19,.18])
    goal = next(navigation.front_view_goals(trial, [0.,0.,0.], face, dict(view_xy_yaw=[.16,0.,0.],table_height_m=.8)))
    assert goal[0] > .15 and -.19 <= goal[1] <= -.09
    assert navigation.cabinet_gap(goal, face, trial.navigation_body) == pytest.approx(.06)
    # The remaining frontage has no map-clear camera view at the archived distance.
    face['lateral_m'] = [-.04,.18]
    assert not list(navigation.front_view_goals(trial, [0.,0.,0.], face, dict(view_xy_yaw=[.16,0.,0.],table_height_m=.8)))


def test_saved_depth_uses_voxel_height_and_original_square(tmp_path):
    import subprocess
    import numpy as np
    from tools.run.cabinet_upper_body import write_saved_scene

    # At the same xy, the low chassis ends before the square but the extended
    # folded arm overlaps it. Exercise the exact evaluator loaded by DWB.
    def rectangle(front):
        return np.array([[-.1,-.1],[front,-.1],[front,.1],[-.1,.1]])

    boxes = [(rectangle(.15), 0., .9), (rectangle(.30), .9, 1.)]
    source = tmp_path/'check.cpp'
    source.write_text('#include "scene.hpp"\nint main(int n, char** v) {'
                      'cabinet::SavedScene s(v[1]); return s.blocked(0,0,0); }')
    checker = tmp_path/'check'
    subprocess.run(['c++','-std=c++17','-I',str(navigation.ROOT/'tools/run/saved_depth_critic'),
                    str(source),'-o',str(checker)], check=True)
    for level, expected in [(12, 0), (18, 1)]:
        path = tmp_path/'scene'
        write_saved_scene(path, np.array([[5,0,level]]), [], boxes, .03)
        assert subprocess.run([checker, path]).returncode == expected


def test_visible_top_still_prefers_a_closer_map_clear_park():
    import math
    # Last field stop, 4 cm short of its former observation waypoint.
    pose = [1.11535, -.55409, 1.11284]
    face = dict(point=[1.14629,-.11313], yaw=.92545, lateral_m=[-.393,.313])
    route = json.loads((navigation.CONFIG/'cabinet_navigation.json').read_text())
    trial = SimpleNamespace(navigation_body={'footprint_xy_m':[[.25,.20],[.25,-.20]], 'padded_outline_xy_m':[[.28,.20],[.28,-.20]]},
        area={'resolution_m':.05, 'origin_xy_m':[0.,0.]}, arrival_xy=.03, arrival_yaw=.1,
        maximum_turn=math.pi/2, distance=.65, map_free=lambda _:True)
    goal = next(navigation.front_view_goals(trial, pose, face, route))
    normal = [math.cos(face['yaw']), math.sin(face['yaw'])]
    assert sum((goal[i]-pose[i])*normal[i] for i in range(2)) > 0
    trial.map_free=lambda p: math.dist(p[:2],pose[:2])<1e-6
    assert not list(navigation.front_view_goals(trial, pose, face, route))
    assert not list(navigation.front_view_goals(trial, [*pose[:2],pose[2]+math.pi], face, route))


@pytest.mark.parametrize('error', [-.12, .12])
@pytest.mark.parametrize('depth', [0., .09])
def test_eraser_alignment_corrects_forward_and_lateral_error(error, depth):
    import math
    pose = [1.,-.5,.2]
    face_yaw = .5
    angle = face_yaw - pose[2]
    side = [-math.sin(angle), math.cos(angle)]
    normal = [math.cos(angle), math.sin(angle)]
    reference = [.341,.124]
    target = [reference[i] + error*side[i] + depth*normal[i] for i in range(2)]
    rows = navigation.placement_candidates(SimpleNamespace(map_free=lambda _:True), pose, target, reference, face_yaw)
    assert rows[0]['delta'] == pytest.approx([target[i]-reference[i] for i in range(2)])
    assert rows[0]['lateral_only'] == (depth == 0.)
    assert any(not r['lateral_only'] for r in rows)

def test_side_slot_can_observe_local_top_and_request_nav2_detour():
    body = {'footprint_xy_m': [[.25,.12],[.25,-.12]], 'padded_outline_xy_m': [[.28,.12],[.28,-.12]]}
    # A strip breaks the direct approach. There is open room around its end.
    def free(p):
        return not (.07 <= p[0] <= .10 and p[1] > -.22) and (p[0] < .17 or p[1] < -.23)
    trial = SimpleNamespace(navigation_body=body, area={'resolution_m':.05, 'origin_xy_m':[0.,0.]},
                            arrival_xy=.02, arrival_yaw=.1, maximum_turn=1., distance=.65, map_free=free)
    face = dict(point=[.49,0.], yaw=0., lateral_m=[-.35,.18])
    pose = [0.,0.,0.]
    goal = next(navigation.front_view_goals(trial, pose, face, {'table_height_m':.8}))
    assert free(goal) and goal[1] < -.23
    assert navigation.cabinet_gap(goal, face, body) == pytest.approx(.06)
    assert not all(free(p) for p in navigation.NearGoal.path(trial, pose, goal))
    assert not navigation.cabinet_gap_reached(pose, face, body)


def test_cabinet_gap_uses_base_front_with_protruding_folded_arm():
    face=dict(point=[.26,0],yaw=0.)
    body=dict(mode='measured_upper_travel',footprint_xy_m=[[.253,0]],
              self_mask_xy_m=[[.205,0]],base_footprint_xy_m=[[.20,0]])
    assert navigation.cabinet_gap((0.,0.,0.),face,body)==pytest.approx(.06)
    assert navigation.cabinet_gap_reached((0.,0.,0.),face,body)
    del body['base_footprint_xy_m']
    assert navigation.cabinet_gap((0.,0.,0.),face,body)==pytest.approx(.055)
