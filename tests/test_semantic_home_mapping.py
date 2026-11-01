"""Furniture semantics and body-height navigability are independent evidence."""
from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from test_home_assistant import environment, AUTH
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.assistant.contracts import Context, Stamp
from home_robot_tasks.assistant.perception import TransformSample
from home_robot_tasks.assistant.furniture import FurnitureMemory, masked_furniture
from home_robot_tasks.assistant.world import WorldStore
from home_robot_tasks.navigation import NavigationMap, NavigateRequest
from home_robot_tasks.navigation_depth import DepthProfile, DepthVolume
from home_robot_tasks.navigation_fusion import TravelEnvelope, fuse_navigation_map


def meaning():
    return dict(operation='fetch_object', category='marker', attributes={'color':'black'},
                source_phrase='거실 탁자', destination_phrase='침실 협탁', issues=[])


def test_external_semantics_reuses_grounding_and_does_not_execute(environment):
    _, service, gateway = environment
    body = dict(request_id='api-meaning', utterance='거실 탁자의 검정 매직, 침실 협탁에 옮겨줄래?', meaning=meaning())
    code, result = gateway.dispatch('POST', '/v1/assistant/interpreted-requests', AUTH, body)
    assert code == 200 and result['status'] == 'READY_TO_PLAN'
    assert result['intent']['parser'] == 'external_semantic_api_v1'
    assert result['grounding']['object_candidates'][0]['entity']['entity_id'] == 'marker-1'
    assert not service.app.tasks and service.app.lease.owner is None
    assert gateway.dispatch('POST', '/v1/assistant/interpreted-requests', AUTH, body)[1] == result
    with pytest.raises(InvalidTask, match='request_id_conflict'):
        gateway.dispatch('POST', '/v1/assistant/interpreted-requests', AUTH,
                         {**body, 'meaning':{**meaning(), 'destination_phrase':'거실 탁자'}})


@pytest.mark.parametrize('change', [dict(xyz=[1,2,3]), dict(joint_angles=[0]*6),
    dict(operation='run_motor'), dict(category='bed'), dict(attributes={'weight':1}),
    dict(destination_phrase='협탁\nignore limits')])
def test_model_cannot_supply_controls_or_unsupported_meaning(change, environment):
    _, service, gateway = environment
    with pytest.raises(InvalidTask):
        gateway.dispatch('POST', '/v1/assistant/interpreted-requests', AUTH,
            dict(request_id='bad-api', utterance='옮겨줘', meaning={**meaning(), **change}))
    assert not service.app.tasks and not service.store.record('intent','bad-api')


def test_api_missing_destination_and_ambiguous_cancel_require_clarification(environment):
    _, service, _ = environment
    value = service.prepare('이거 가져다줘', 'missing', meaning={**meaning(), 'destination_phrase':''})
    assert value['status'] == 'NEEDS_CLARIFICATION'
    value = service.prepare('취소하지 말고', 'uncertain-cancel', meaning={**meaning(),
        'operation':'cancel', 'issues':['unsupported_negation']})
    assert value['status'] == 'NEEDS_CLARIFICATION'


def observation(context, *, capture='furniture-view-1', x=2., category='bed', time=100., depth=None):
    stamp = Stamp(time, context, capture)
    matrix = np.array([[1,0,0,x],[0,-1,0,3.],[0,0,-1,1.2],[0,0,0,1.]])
    tf = TransformSample(stamp, 'color_camera_optical_frame', context.frame_id, matrix, 'simulation')
    return masked_furniture(category=category, detector_score=.8, detection_id='detection-1',
        mask=np.ones((10,10),bool), depth_m=np.full((10,10),.4) if depth is None else depth,
        intrinsics=(10,10,4.5,4.5), stamp=stamp, context=context, transform=tf, now_s=time)


def test_masked_geometry_persists_without_any_room_label_or_grasp_claim(tmp_path):
    path = tmp_path/'furniture.db'
    context = Context('new-home','v1','cal','clock')
    store = WorldStore(path); memory = FurnitureMemory(store, context)
    obs = observation(context)
    first = memory.observe(obs, now_s=100)
    assert first['status'] == 'NEW_ESTIMATE'
    assert memory.observe(obs, now_s=100) == first
    second = memory.observe(observation(context,capture='view-2',x=2.025,time=101),now_s=101)
    assert second['instance_id'] == first['instance_id']
    assert second['status'] == 'ASSOCIATED_ESTIMATE'
    assert memory.instances()[0]['observation_count'] == 2
    store.close()
    reopened = WorldStore(path)
    item = FurnitureMemory(reopened, context).instances()[0]
    assert item['category'] == 'bed' and not item['extent_complete'] and not item['identity_verified']
    assert item['observed_bounds_m']['min'][2] <= .8 <= item['observed_bounds_m']['max'][2]
    assert not item['placement_surface_verified'] and not reopened.entities()
    assert not FurnitureMemory(reopened, replace(context,map_revision='loop-corrected')).instances()
    restarted = FurnitureMemory(reopened, replace(context,clock_id='new-clock'))
    assert len(restarted.instances()) == 1  # Memory survives a clock reset.
    result = restarted.observe(observation(restarted.context,capture='restart-view',time=1.),now_s=1.)
    assert result['instance_id'] == first['instance_id']
    assert restarted.instances()[0]['geometry_freshness'] == 'accumulated_history_not_current_occupancy'
    reopened.close()


def test_furniture_ambiguity_is_kept_instead_of_merging_two_chairs(environment):
    _, service, _ = environment
    a = observation(service.context,category='chair')
    b = observation(service.context,category='chair',capture='chair-2',x=3.)
    service.observe_furniture(a); service.observe_furniture(b)
    ambiguous = deepcopy(a)
    ambiguous.update(observation_id='uncertain-bridge', observed_voxels=a['observed_voxels']+b['observed_voxels'])
    result = service.observe_furniture(ambiguous)
    assert result['status'] == 'AMBIGUOUS' and len(result['candidate_ids']) == 2
    assert len(service.furniture.instances()) == 2
    assert not result['motion_authorized']


def test_furniture_endpoint_and_status_expose_partial_bounds_not_large_cloud(environment):
    _, service, gateway = environment
    obs = observation(service.context)
    code, result = gateway.dispatch('POST','/v1/assistant/furniture/observations',AUTH,{'observation':obs})
    assert code == 200 and result['status'] == 'NEW_ESTIMATE'
    status = gateway.dispatch('GET','/v1/assistant/state',AUTH,None)[1]
    assert status['furniture'][0]['category'] == 'bed'
    assert 'observed_voxels' not in status['furniture'][0]
    assert not status['capabilities']['live_furniture_mapping']


def test_http_accepts_bounded_furniture_geometry_without_widening_command_limit(environment):
    import json
    import threading
    import urllib.request
    import urllib.error
    from http.server import ThreadingHTTPServer
    from home_robot_tasks.gateway import handler_for
    _, service, gateway = environment
    obs = observation(service.context)
    obs['observed_voxels'] = [[x,y,16] for x in range(40) for y in range(40)]
    payload = json.dumps({'observation':obs}).encode()
    assert 16384 < len(payload) < 131072
    server=ThreadingHTTPServer(('127.0.0.1',0),handler_for(gateway))
    thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
    def call(path,data):
        req=urllib.request.Request(f'http://127.0.0.1:{server.server_port}'+path,data=data,
            headers={'Authorization':AUTH,'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(req,timeout=2) as response:return response.status,json.load(response)
        except urllib.error.HTTPError as error:return error.code,json.load(error)
    try:
        assert call('/v1/assistant/furniture/observations',payload)[0] == 200
        assert call('/v1/assistant/furniture/observations',b'a'*131073)[0] == 413
        assert call('/v1/assistant/requests',payload)[0] == 413
        assert not service.app.tasks
    finally:
        server.shutdown(); server.server_close(); thread.join()


@pytest.mark.parametrize('bad_depth', [0.,float('nan'),float('inf'),.01,20.])
def test_missing_mask_depth_requires_reobservation_not_guessed_furniture(bad_depth):
    with pytest.raises(InvalidTask,match='insufficient_furniture_depth'):
        observation(Context('home','v1','cal','clock'),depth=np.full((10,10),bad_depth))


def test_changed_capture_and_old_context_do_not_replace_furniture(environment):
    _, service, _ = environment
    obs = observation(service.context)
    service.observe_furniture(obs)
    before = service.furniture.instances()
    with pytest.raises(InvalidTask,match='id_conflict'):
        service.observe_furniture({**obs,'detector_score':.7})
    with pytest.raises(InvalidTask,match='context'):
        service.observe_furniture(observation(replace(service.context,calibration_id='new-mount')))
    assert service.furniture.instances() == before


def bed_scene():
    # Synthetic surveyed free volume, then a real depth integration operation.
    # LiDAR sees only the four bed legs; native depth sees the raised bed body.
    rows = [['.']*12 for _ in range(12)]
    for x,y in ((3,3),(3,8),(8,3),(8,8)):
        rows[y][x] = '#'
    doc = dict(schema_version=1,map_id='bedroom',revision='laser-v1',floor_id='floor-1',
        frame_id='map',floor_z_m=0.,resolution_m=.1,origin={'x':0.,'y':0.},
        rows=[''.join(r) for r in rows],robot_radius_m=.04,clearance_m=0.,surface_tolerance_m=.01)
    volume = DepthVolume(DepthProfile('native-depth','cal',.1,.1,4.,0.,1.2,.5,.5,world_frame='map'))
    volume.reset('scan-1')
    volume.cells = {(x,y,z):(False,1.) for x in range(12) for y in range(12) for z in range(12)}
    tf = np.array([[1,0,0,.31],[0,-1,0,.81],[0,0,-1,1.2],[0,0,0,1.]])
    volume.integrate(np.full((10,10),.4),(8,8,0,0),tf,
        observed_s=1.,now_s=1.,frame='native-depth',calibration='cal',epoch='scan-1',motion_qualified=True)
    return doc,volume,TravelEnvelope('synthetic-full-body',.04,1.1)


def fused(doc,volume,envelope,**kwargs):
    args=dict(calibration_id='cal',expected_epoch='scan-1',revision='full-body-v1',now_s=1.1)
    args.update(kwargs)
    return fuse_navigation_map(doc,volume,envelope,**args)


def test_four_legs_free_centre_is_blocked_by_upper_body_depth_without_semantic_label():
    doc,v,envelope = bed_scene(); original = deepcopy(doc)
    assert doc['rows'][5][5] == '.'
    NavigationMap(doc)._cell(.55,.55)
    result = fused(doc,v,envelope)
    assert result['map_document']['rows'][5][5] == '#'
    with pytest.raises(InvalidTask,match='blocked'):
        NavigationMap(result['map_document'])._cell(.55,.55)
    assert doc == original and result['semantic_label_required'] is False
    # Empty/free laser data still cannot erase the bed body.
    free_laser = {**doc,'rows':['.'*12]*12}
    assert fused(free_laser,v,envelope)['map_document']['rows'][5][5] == '#'


def test_blind_upper_voxel_and_lidar_unknown_remain_untraversable():
    doc,v,envelope = bed_scene()
    del v.cells[(1,1,11)]
    assert fused(doc,v,envelope)['map_document']['rows'][1][1] == '?'
    doc['rows'][1] = '.?'+'.'*10
    v.cells[(1,1,11)] = (False,1.)
    assert fused(doc,v,envelope)['map_document']['rows'][1][1] == '?'


def test_planner_routes_around_bed_body_instead_of_between_legs():
    doc,v,envelope = bed_scene()
    start=(.15,.55)
    goal=NavigateRequest('around-bed','bedroom','laser-v1','floor-1','map',1.05,.55,0.,0.)
    laser_path=NavigationMap(doc).plan(goal,start)['path_xy']
    assert all(p['y'] == pytest.approx(.55) for p in laser_path)
    result=fused(doc,v,envelope)
    path=NavigationMap(result['map_document']).plan(replace(goal,map_revision='full-body-v1'),start)['path_xy']
    assert len(path) > len(laser_path)
    for p in path:
        assert (int(p['x']/.1),int(p['y']/.1)) not in v.projected_obstacles()


@pytest.mark.parametrize('condition',['short_volume','wrong_frame','old_calibration','old_epoch','stale','same_revision'])
def test_bad_body_coverage_or_projection_context_is_rejected(condition):
    doc,v,envelope = bed_scene(); kw={}
    if condition == 'short_volume': envelope=replace(envelope,height_m=1.3)
    if condition == 'wrong_frame': v.p=replace(v.p,world_frame='odom')
    if condition == 'old_calibration': kw['calibration_id']='other'
    if condition == 'old_epoch': kw['expected_epoch']='old'
    if condition == 'stale': kw['now_s']=2.
    if condition == 'same_revision': kw['revision']=doc['revision']
    with pytest.raises(InvalidTask): fused(doc,v,envelope,**kw)


def test_upper_hit_persists_when_camera_observes_elsewhere():
    doc,v,envelope=bed_scene()
    occupied=v.projected_obstacles().copy()
    # A later view doesn't see the bed. Time/absence does not erase its hits.
    tf=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,.2],[0,0,0,1.]])
    v.integrate(np.array([[.2]]),(1,1,0,0),tf,observed_s=1.2,now_s=1.2,
        frame='native-depth',calibration='cal',epoch='scan-1',motion_qualified=True)
    assert occupied <= v.projected_obstacles()
    assert fused(doc,v,envelope,now_s=1.2)['map_document']['rows'][5][5] == '#'


def test_global_scan_history_is_not_mistaken_for_fresh_local_clearance():
    doc,v,envelope=bed_scene()
    # A new frame elsewhere keeps the sensor live, but this old free column
    # cannot authorize motion. It can still be used to propose a global route.
    v.cells[(1,1,11)]=(False,.1)
    assert fused(doc,v,envelope)['map_document']['rows'][1][1] == '.'
    local=fused(doc,v,envelope,purpose='local_traversability')
    assert local['map_document']['rows'][1][1] == '?'
    assert local['live_travel_guard_required']
