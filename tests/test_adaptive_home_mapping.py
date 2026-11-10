from copy import deepcopy
import numpy as np
import pytest
from test_home_assistant import environment, AUTH, read
from home_robot_tasks.adaptive_mapping import AdaptiveMap
from home_robot_tasks.home_survey import next_view, HiddenSyntheticHouse
from home_robot_tasks.navigation import NavigationMap
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.semantic_mapping import RGBDFurnitureMapper
from home_robot_tasks.assistant.contracts import Stamp
from home_robot_tasks.assistant.perception import TransformSample


def grid():
    doc = read('navigation_map.simulation.json')
    doc.update(rows=['#'*16]+['#'+'.'*14+'#' for _ in range(14)]+['#'*16],
               resolution_m=.1, robot_radius_m=.04, clearance_m=0., origin={'x':0.,'y':0.})
    return doc


def inputs(doc, t, laser=None, depth=None):
    return dict(lidar_rows=laser or doc['rows'], depth_rows=depth or doc['rows'],
                observed_s=t, now_s=t, capture_id=f'capture-{t}', frame_id=doc['frame_id'],
                calibration_id='cal', clock_id='clock', localization_qualified=True,
                full_height_qualified=True)


def test_move_furniture_adds_new_shape_and_clears_old_only_with_independent_free_views():
    doc = grid(); rows = list(doc['rows']); rows[6] = rows[6][:6]+'#'+rows[6][7:]
    doc['rows'] = rows
    updater = AdaptiveMap(doc, calibration_id='cal', clock_id='clock')
    laser, depth = grid()['rows'], grid()['rows']
    depth = list(depth); depth[8] = depth[8][:8]+'#'+depth[8][9:]
    first = updater.update(**inputs(doc, 1., laser, depth))
    assert first['added_cells'] == [[8,8]] and first['cleared_cells'] == []
    second = updater.update(**inputs(doc, 1.2, laser, depth))
    assert second['cleared_cells'] == [[6,6]]
    assert second['map_document']['rows'][8][8] == '#'
    assert updater.route_affected(second, [{'x':.75,'y':.65}])
    assert not updater.route_affected(second, [{'x':1.25,'y':1.25}])


def test_occlusion_stale_duplicate_and_protected_geometry_never_clear():
    doc = grid(); doc['rows'][6] = '#.....#........#'
    updater = AdaptiveMap(doc, calibration_id='cal', clock_id='clock', protected_cells=[(6,6)])
    free = grid()['rows']; unseen = ['?'*16]*16
    updater.update(**inputs(doc, 1., free, unseen))
    for t in (1.2,1.4):
        result = updater.update(**inputs(doc,t,free,free))
        assert result['map_document']['rows'][6][6] == '#'
    with pytest.raises(InvalidTask): updater.update(**inputs(doc,1.4,free,free))
    bad = inputs(doc,2.,free,free);bad['now_s']=3.
    with pytest.raises(InvalidTask): updater.update(**bad)
    bad = inputs(doc,2.,free,free);bad['full_height_qualified']=False
    with pytest.raises(InvalidTask): updater.update(**bad)


def test_frontiers_are_selected_from_partial_map_and_never_cross_unknown():
    sensor = HiddenSyntheticHouse(grid())
    observed, _ = sensor.capture((.35,.35))
    assert any('?' in r for r in observed['rows'])
    proposal = next_view(observed,(.35,.35))
    assert proposal is not None
    assert proposal['cell'] not in NavigationMap(observed).blocked
    changed_hidden = grid(); changed_hidden['rows'][13] = '#...########...#'
    sensor2 = HiddenSyntheticHouse(changed_hidden)
    other, _ = sensor2.capture((.35,.35))
    assert observed['rows'] == other['rows']
    assert proposal == next_view(other,(.35,.35))


def test_one_button_survey_uses_executor_finishes_and_is_idempotent(environment):
    now, service, gateway = environment
    _, permit = gateway.dispatch('GET','/v1/permit',AUTH,None)
    body = {'session_id':'auto-home','name':'우리 집'}
    code, scan = gateway.dispatch('POST','/v1/assistant/surveys',AUTH,body,permit['permit'])
    assert code == 201 and scan['autonomous']
    assert gateway.dispatch('POST','/v1/assistant/surveys',AUTH,body)[1] == scan
    for _ in range(1500):
        now[0] += .1; gateway.tick()
        scan = service.scan.get('auto-home')
        if scan['status'] != 'SCANNING': break
    assert scan['status'] == 'REVIEW', scan
    assert 0 < scan['survey_legs'] <= 65
    assert service.app.lease.owner is None
    assert all(t.result()['hardware_commands'] == 0 for t in service.app.tasks.values())
    assert scan['coverage']['whole_home_fraction'] is None
    assert not scan['navigation_ready']  # synthetic grids are not native-depth qualification


def test_pause_waits_for_shared_stop_resume_needs_permit_and_external_writes_rejected(environment):
    now, service, gateway = environment
    permit = gateway.dispatch('GET','/v1/permit',AUTH,None)[1]['permit']
    gateway.dispatch('POST','/v1/assistant/surveys',AUTH,{'session_id':'auto','name':'집'},permit)
    now[0]+=.1;gateway.tick()
    result = gateway.dispatch('POST','/v1/assistant/scans/auto/pause',AUTH,{})[1]
    assert result['survey_state'] == 'STOPPING'
    assert service.scan.get('auto')['status'] == 'SCANNING'
    for _ in range(20): now[0]+=.1;gateway.tick()
    assert service.scan.get('auto')['status']=='PAUSED'
    assert service.app.lease.owner is None
    with pytest.raises(InvalidTask,match='permit'):
        gateway.dispatch('POST','/v1/assistant/scans/auto/resume',AUTH,{})
    with pytest.raises(InvalidTask,match='owns_observations'):
        gateway.dispatch('POST','/v1/assistant/scans/auto/map',AUTH,{'map_document':grid()})
    permit = gateway.dispatch('GET','/v1/permit',AUTH,None)[1]['permit']
    gateway.dispatch('POST','/v1/assistant/scans/auto/resume',AUTH,{},permit)
    assert service.scan.get('auto')['status']=='SCANNING'


def test_rgb_categories_and_masked_shapes_share_existing_detector(environment):
    now, service, _ = environment
    class Model:
        def detect(self, image, prompt):
            assert 'coffee table' in prompt
            return [{'label':'desk','score':.9,'box_xyxy':[1,1,19,19]}], None
    class Segmenter:
        def segment(self, image, boxes):
            return [(np.ones((20,20),dtype=bool),.9)], None
    mapper = RGBDFurnitureMapper(service.store,service.context,Model(),Segmenter(),model_revision='fixture-v1')
    stamp = Stamp(now[0],service.context,'rgb-frame')
    tf = TransformSample(stamp,'color_camera_optical_frame','map',np.eye(4),'simulation')
    rgb = np.zeros((20,20,3),dtype=np.uint8);depth=np.ones((20,20))
    result = mapper.observe(rgb,depth,(20,20,9.5,9.5),stamp=stamp,transform=tf,now_s=now[0])
    assert result['provenance']['categories']==['desk']
    assert mapper.memory.instances()[0]['category']=='desk'
    assert len(mapper.memory.instances()[0]['observed_voxels'])>4
    assert not result['provenance']['collision_clearance_authority']
    assert mapper.observe(rgb,depth,(20,20,9.5,9.5),stamp=stamp,transform=tf,now_s=now[0])==result


def test_native_depth_and_current_laser_adapter_rejects_history_and_unknown_height():
    from home_robot_tasks.navigation_depth import DepthProfile, DepthVolume
    from home_robot_tasks.navigation_fusion import TravelEnvelope
    doc = grid()
    p = DepthProfile('depth','cal',.1,.1,3.,0.,1.,.5,.5,world_frame='map')
    volume = DepthVolume(p);volume.reset('epoch')
    volume.last_stamp=1.;volume.error=None
    # An elevated hit invisible to the planar laser still blocks the map.
    volume.cells[(6,6,8)] = (True,1.)
    updater=AdaptiveMap(doc,calibration_id='cal',clock_id='clock')
    kwargs=dict(ranges=[.8,.8],angle_min=0.,angle_increment=.1,range_min=.12,
        range_max=3.5,lidar_xy_yaw=(.35,.35,0.),volume=volume,
        envelope=TravelEnvelope('fixture-body',.04,1.),expected_epoch='epoch',
        observed_s=1.,now_s=1.,capture_id='sensor-1',clock_id='clock',localization_qualified=True)
    result=updater.observe_sensors(**kwargs)
    assert [6,6] in result['added_cells']
    assert not result['cleared_cells']
    volume.historical=True
    with pytest.raises(InvalidTask,match='synchronized_full_body'):
        updater.observe_sensors(**kwargs)


def test_map_change_cancels_current_route_and_waits_for_stop(environment):
    now,service,gateway=environment
    app=service.app; doc=app.map._document
    updater=AdaptiveMap(doc,calibration_id='cal',clock_id='clock')
    req=dict(schema_version=1,operation='navigate_to',request_id='travel',map_id=app.map.map_id,
        map_revision=app.map.revision,floor_id=app.map.floor_id,frame_id=app.map.frame_id,
        point=dict(x=3.,y=3.,z=0.),yaw_rad=0.)
    app.submit(req,0.)
    app.tick(.1)
    changed=list(doc['rows']);changed[3]=changed[3][:5]+'#'+changed[3][6:]
    result=updater.update(**inputs(doc,1.,changed,changed))
    updater.apply_to_application(app,result,now_s=.2)
    assert app.pending_map is not None and not app.pose_known
    for i in range(3,30):app.tick(i/10)
    assert app.lease.owner is None and app.pending_map is None
    assert app.map.revision==result['revision']
    assert app.status('travel')['status']=='CANCELLED'
    assert not app.pose_known


def test_restart_pauses_survey_and_can_resume_from_saved_simulation_checkpoint(environment):
    from home_robot_tasks.application import RobotApplication
    from home_robot_tasks.fake_robot import FakeRobot
    from home_robot_tasks.assistant.service import AssistantService
    now,service,gateway=environment
    service.survey.start('restart','집',0.)
    now[0]+=.1;gateway.tick()
    original=service.scan.get('restart')
    restarted_app=RobotApplication(read('home.example.json'),NavigationMap(read('navigation_map.simulation.json')),FakeRobot())
    restarted=AssistantService(restarted_app,service.store,service.context,lambda:now[0])
    assert restarted.scan.get('restart')['status']=='PAUSED'
    assert restarted_app.lease.owner is None
    restarted.survey.action('restart','resume',0.)
    assert restarted.scan.get('restart')['status']=='SCANNING'
    assert restarted.survey.sensor.sequence==original['sensor_sequence']


def test_gateway_fault_never_advances_survey_after_stop(environment):
    now,service,gateway=environment
    service.survey.start('fault','집',0.)
    now[0]+=.1;gateway.tick()
    count=len(service.app.tasks)
    gateway.latch_fault()
    for _ in range(30):now[0]+=.1;gateway.tick()
    assert len(service.app.tasks)==count
    assert service.scan.get('fault')['status']=='PAUSED'
    assert service.app.lease.owner is None


def test_furniture_history_is_kept_but_vacated_shape_is_not_associated_again(environment):
    from home_robot_tasks.assistant.furniture import FurnitureMemory
    now,service,_=environment
    memory=FurnitureMemory(service.store,service.context)
    obs=dict(observation_id='fobs-old',category='desk',detector_score=.9,
        stamp=Stamp(now[0],service.context,'desk-old').to_dict(),source='simulation',
        voxel_size_m=.1,observed_voxels=[[6,6,z] for z in range(2,6)],
        extent_complete=False,depth_valid_fraction=1.)
    old=memory.observe(obs,now_s=now[0])['instance_id']
    doc=grid();doc['rows'][6]='#.....#........#'
    updater=AdaptiveMap(doc,calibration_id=service.context.calibration_id,clock_id=service.context.clock_id)
    free=grid()['rows']
    def observation(t):
        return {**inputs(doc,t,free,free), 'calibration_id':service.context.calibration_id, 'clock_id':service.context.clock_id}
    updater.update(**observation(1.))
    update=updater.update(**observation(1.2))
    result=memory.reconcile_local_geometry(update)
    assert result['affected_instances']==[old]
    previous=memory.instances()[0]
    assert previous['observed_voxels']==obs['observed_voxels']
    assert previous['current_observed_voxels']==[] and previous['location_state']=='vacated'
    assert previous['last_observed_s']==now[0]
    newer={**obs,'observation_id':'fobs-new','stamp':Stamp(now[0]+1,memory.context,'desk-new').to_dict(),
           'observed_voxels':[[8,8,z] for z in range(2,6)]}
    assert memory.observe(newer,now_s=now[0]+1)['instance_id']!=old
