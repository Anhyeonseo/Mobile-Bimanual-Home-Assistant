from copy import deepcopy
import json
import math
from pathlib import Path
import pytest
from home_robot_tasks.application import RobotApplication
from home_robot_tasks.fake_robot import FakeRobot
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.navigation import NavigationMap, NavigateRequest

ROOT=Path(__file__).resolve().parents[1]
def read(name): return json.loads((ROOT/'config'/name).read_text())
def request(): return read('navigate_to.simulation.json')
def grid(): return NavigationMap(read('navigation_map.simulation.json'))
def app(): return RobotApplication(read('home.example.json'),grid(),FakeRobot())


def test_clicked_point_plans_around_obstacles_and_runs_simulation():
    service=app(); goal=request(); result=service.submit(goal,0)
    path=result['plan']['path_xy']
    cells=[service.map._cell(p['x'],p['y']) for p in path]
    assert all(abs(a[0]-b[0])+abs(a[1]-b[1])==1 for a,b in zip(cells,cells[1:]))
    for i in range(10): service.tick(i/10)
    result=service.status(goal['request_id'])
    assert result['status']=='SUCCEEDED' and service.xy==(7.25,4.25)
    assert not result['physical_task_completed'] and result['hardware_commands']==0
    assert result['plan']['goal']['z']==0


@pytest.mark.parametrize('field,value', [('map_revision','old'),('map_id','other'),('floor_id','upstairs'),('frame_id','camera')])
def test_rejects_wrong_map_reference(field,value):
    req=request(); req[field]=value
    with pytest.raises(InvalidTask,match='mismatch'): app().submit(req,0)


@pytest.mark.parametrize('point', [dict(x=4.5,y=2.5,z=0),dict(x=-1,y=1.5,z=0),dict(x=7.25,y=4.25,z=.5),dict(x=0,y=0,z=0)])
def test_blocked_outside_furniture_surface_and_boundary_rejected(point):
    req=request(); req['point']=point
    service=app()
    with pytest.raises(InvalidTask): service.submit(req,0)
    assert service.lease.owner is None and not service.tasks


@pytest.mark.parametrize('value',[float('nan'),float('inf'),True,'2',10**1000])
def test_nonfinite_or_coerced_coordinates_rejected(value):
    req=request(); req['point']['x']=value
    with pytest.raises(InvalidTask): NavigateRequest.from_dict(req)


def test_unknown_cells_block_paths_and_disconnected_goal_rejected():
    data=read('navigation_map.simulation.json')
    data['rows']=[row[:6]+'?'+row[7:] for row in data['rows']]
    with pytest.raises(InvalidTask,match='no_path'):
        NavigationMap(data).plan(NavigateRequest.from_dict(request()),(1.5,1.5))


def test_idempotency_conflict_and_busy_goal_do_not_replace_active():
    service=app(); req=request(); first=service.submit(req,0)
    assert service.submit(deepcopy(req),0)==first
    changed=deepcopy(req); changed['point']['x']=7.5
    with pytest.raises(InvalidTask,match='request_id_conflict'): service.submit(changed,0)
    changed['request_id']='other'
    with pytest.raises(InvalidTask,match='robot_busy'): service.submit(changed,0)
    service.tick(0)
    assert len(service.adapter.log)==1
    result=service.cancel(req['request_id'],.1)
    assert result['status']=='STOPPING'
    with pytest.raises(InvalidTask,match='robot_busy'): service.submit(changed,.1)
    service.tick(.3)
    assert service.status(req['request_id'])['status']=='CANCELLED'
    assert not service.pose_known
    service.localize_simulated((1.5,1.5),.3)
    service.submit(changed,.3); service.tick(.3); service.tick(.6)
    assert service.status('other')['status']=='SUCCEEDED'


def test_map_change_cancels_inflight_and_requires_new_localization():
    service=app(); req=request(); service.submit(req,0); service.tick(0)
    data=read('navigation_map.simulation.json'); data['revision']='synthetic-2'
    old_revision=service.map.revision
    service.replace_map(NavigationMap(data),.1)
    assert service.map.revision==old_revision and service.pending_map is not None
    assert service.status(req['request_id'])['reason']=='map_changed'
    assert service.lease.owner is not None
    service.tick(.3)
    assert service.map.revision=='synthetic-2' and service.pending_map is None
    assert not service.pose_known
    req['request_id']='new';req['map_revision']='synthetic-2'
    with pytest.raises(InvalidTask,match='localization_required'): service.submit(req,.4)


def test_revision_geometry_is_immutable_and_client_cannot_set_safety_settings():
    service=app(); data=read('navigation_map.simulation.json'); data['rows'][3]='#....#.............#'
    with pytest.raises(InvalidTask,match='map_revision_reused'): service.replace_map(NavigationMap(data),0)
    req=request(); req['robot_radius_m']=0
    with pytest.raises(InvalidTask): service.submit(req,0)


def test_descriptors_do_not_mutate_internal_map_and_z_tolerance_is_bounded():
    nav=grid(); copy=nav.descriptor(); copy['rows'][0]='bad'
    assert nav.descriptor()['rows'][0]!='bad'
    data=read('navigation_map.simulation.json'); data['surface_tolerance_m']=10
    with pytest.raises(InvalidTask): NavigationMap(data)


def test_feedback_driven_navigation_requires_localization_after_success():
    service = RobotApplication(read('home.example.json'), grid(), FakeRobot(),
                               exact_goal_simulation=False)
    goal = request()
    service.submit(goal, 0)
    for i in range(10):
        service.tick(i / 10)
    assert service.status(goal['request_id'])['status'] == 'SUCCEEDED'
    assert not service.pose_known and service.xy == (1.5, 1.5)
    goal['request_id'] = 'next'
    with pytest.raises(InvalidTask, match='localization_required'):
        service.submit(goal, 1.0)
    service.localize_simulated((7.20, 4.20), 1.1)
    service.submit(goal, 1.2)
    assert service.pose_known and service.xy == (7.20, 4.20)
