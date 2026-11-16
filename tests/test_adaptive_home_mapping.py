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
    assert updater.update(**bad)['cleared_cells'] == []


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
            assert 'table' in prompt and 'refrigerator' in prompt
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


def living(tmp_path, doc=None, **kwargs):
    from home_robot_tasks.adaptive_mapping import LivingMap
    from home_robot_tasks.assistant.world import WorldStore
    doc = doc or grid()
    store = WorldStore(tmp_path / 'living.sqlite')
    return LivingMap(doc, calibration_id='cal', clock_id='clock',
                     profile_id='folded-body-v1', height_m=1., store=store, **kwargs)


def occupied(doc, cells):
    rows = [list(row) for row in doc['rows']]
    for x, y in cells:
        rows[y][x] = '#'
    return [''.join(row) for row in rows]


def test_person_blocks_immediately_never_persists_and_unknown_does_not_clear(tmp_path):
    doc = grid(); mapping = living(tmp_path)
    person = (8, 8); hit = occupied(doc, [person])
    for t in (1., 2., 3., 4.):
        result = mapping.observe(**inputs(doc, t, hit, hit),
                                 furniture_cells=[person], dynamic_cells=[person])
        assert result['live']['map_document']['rows'][8][8] == '#'
        assert not result['saved']
    assert mapping.store.record('living_map', doc['map_id']) is None
    unseen = ['?' * 16] * 16
    mapping.observe(**inputs(doc, 5., unseen, unseen))
    assert mapping.live.nav.rows[8][8] == '#'
    mapping.observe(**inputs(doc, 6.))
    mapping.observe(**inputs(doc, 6.3))
    assert mapping.live.nav.rows[8][8] == '.'
    assert mapping.stable.rows[8][8] == '.'


def test_unknown_object_is_avoided_without_semantic_inference(tmp_path):
    doc = grid(); mapping = living(tmp_path)
    hit = occupied(doc, [(8, 8)])
    for t in (1., 2., 3.):
        mapping.observe(**inputs(doc, t, hit, hit))
    assert mapping.live.nav.rows[8][8] == '#'
    assert mapping.stable.rows[8][8] == '.'


def identity(t, cells, fid='registered-table'):
    return [dict(furniture_id=fid, cells=cells, capture_id=f'capture-{t}',
                 observed_s=t, identity_verified=True, method='continuous_track')]


def registered_fixture(mapping, doc, cell=(8, 8)):
    hit = occupied(doc, [cell])
    mapping.observe(**inputs(doc, 0., hit, hit))
    mapping.register_furniture('registered-table', [cell], registration_id='user-add-table',
                               source='user_requested_local_scan', now_s=0.)


def test_moved_table_auto_saves_and_restarts_without_transient_state(tmp_path):
    from home_robot_tasks.adaptive_mapping import LivingMap
    doc = grid(); doc['rows'] = occupied(doc, [(6, 6)])
    mapping = living(tmp_path, doc)
    mapping.register_furniture('registered-table', [(6, 6)], registration_id='initial-table',
                               source='accepted_initial_map', now_s=0.)
    new = occupied(grid(), [(9, 9)])
    for t in (1., 2., 3.):
        result = mapping.observe(**inputs(doc, t, new, new), furniture_cells=[(9, 9)], identity_matches=identity(t, [(9, 9)]))
    assert result['saved']
    assert mapping.stable.rows[6][6] == '.'
    assert mapping.stable.rows[9][9] == '#'
    persisted_revision = mapping.stable.revision
    # A later passing person must not enter the durable snapshot.
    hit = occupied({'rows': new}, [(7, 8)])
    mapping.observe(**inputs(doc, 3.1, hit, hit), dynamic_cells=[(7, 8)])
    mapping.store.close()
    restored = living(tmp_path, doc)
    assert restored.stable.revision == persisted_revision
    assert restored.live.nav.rows[7][8] == '.'
    assert restored.last_observation is None and not restored.fresh(3.2)
    assert len(restored.previous_maps) == 2
    with pytest.raises(InvalidTask, match='context_changed'):
        LivingMap(doc, calibration_id='cal', clock_id='reboot-clock',
                  profile_id='different-body', height_m=1., store=restored.store)


def test_table_top_lidar_free_cannot_erase_depth_obstacle_and_no_height_can_add(tmp_path):
    doc = grid(); doc['rows'] = occupied(doc, [(6, 6)])
    mapping = living(tmp_path, doc)
    for t in (1., 2., 3.):
        mapping.observe(**inputs(doc, t, grid()['rows'], doc['rows']))
    assert mapping.stable.rows[6][6] == '#'
    assert mapping.live.nav.rows[6][6] == '#'
    hit = occupied(grid(), [(9, 9)])
    partial = inputs(doc, 4., hit, hit); partial['full_height_qualified'] = False
    mapping.observe(**partial)
    assert mapping.live.nav.rows[9][9] == '#'
    assert mapping.live.nav.rows[6][6] == '#'


def test_confirmation_is_reset_by_occlusion_gap_or_dynamic_evidence(tmp_path):
    doc = grid(); mapping = living(tmp_path)
    registered_fixture(mapping, doc)
    hit = occupied(doc, [(8, 8)]); unknown = ['?' * 16] * 16
    for t in (1., 2.):
        mapping.observe(**inputs(doc, t, hit, hit), furniture_cells=[(8, 8)], identity_matches=identity(t, [(8, 8)]))
    mapping.observe(**inputs(doc, 2.1, unknown, unknown))
    result = mapping.observe(**inputs(doc, 3., hit, hit), furniture_cells=[(8, 8)], identity_matches=identity(3., [(8, 8)]))
    assert not result['saved']
    result = mapping.observe(**inputs(doc, 10., hit, hit), furniture_cells=[(8, 8)], identity_matches=identity(10., [(8, 8)]))
    assert not result['saved']
    mapping.observe(**inputs(doc, 11., hit, hit), furniture_cells=[(8, 8)], dynamic_cells=[(8, 8)])
    assert not mapping.observe(**inputs(doc, 12., hit, hit), furniture_cells=[(8, 8)], identity_matches=identity(12., [(8, 8)]))['saved']


def test_living_map_rejects_forged_semantic_free_stale_and_duplicate_atomically(tmp_path):
    doc = grid(); mapping = living(tmp_path)
    with pytest.raises(InvalidTask, match='occupied_evidence'):
        mapping.observe(**inputs(doc, 1.), furniture_cells=[(8, 8)])
    assert mapping.live.last_stamp is None
    mapping.observe(**inputs(doc, 1.))
    bad = inputs(doc, 2.); bad['now_s'] = 3.
    with pytest.raises(InvalidTask, match='fresh'):
        mapping.observe(**bad)
    with pytest.raises(InvalidTask, match='duplicate'):
        mapping.observe(**inputs(doc, 1.))
    assert mapping.live.last_stamp == 1.


def test_failed_save_keeps_new_hazard_without_committing_persistent_revision(tmp_path, monkeypatch):
    doc = grid(); mapping = living(tmp_path)
    registered_fixture(mapping, doc)
    hit = occupied(doc, [(8, 8)])
    for t in (1., 2.):
        mapping.observe(**inputs(doc, t, hit, hit), furniture_cells=[(8, 8)], identity_matches=identity(t, [(8, 8)]))
    def fail(*args):
        raise OSError('disk full')
    monkeypatch.setattr(mapping.store, 'save_record', fail)
    with pytest.raises(OSError):
        mapping.observe(**inputs(doc, 3., hit, hit), furniture_cells=[(8, 8)], identity_matches=identity(3., [(8, 8)]))
    assert mapping.live.nav.rows[8][8] == '#'
    assert mapping.stable.rows[8][8] == '.'


def mission(tmp_path, **robot_kwargs):
    from home_robot_tasks.application import RobotApplication
    from home_robot_tasks.adaptive_mapping import LivingMapNavigation
    from home_robot_tasks.fake_robot import FakeRobot
    mapping = living(tmp_path)
    app = RobotApplication(read('home.example.json'), mapping.live.nav,
        FakeRobot(duration_s=10., **robot_kwargs), initial_xy=(.35, .35))
    runner = LivingMapNavigation(app, mapping)
    doc = grid()
    mapping.observe(**inputs(doc, 0.))
    req = dict(schema_version=1, operation='navigate_to', request_id='go-to-table',
        map_id=app.map.map_id, map_revision=app.map.revision, floor_id=app.map.floor_id,
        frame_id=app.map.frame_id, point=dict(x=1.15, y=.35, z=0.), yaw_rad=0.)
    runner.start(req, 0.)
    runner.tick(.1); runner.tick(.2)
    return doc, mapping, app, runner


def test_auto_replan_preserves_goal_waits_for_stop_and_fresh_pose(tmp_path):
    doc, mapping, app, runner = mission(tmp_path)
    hit = occupied(doc, [(7, 3)])
    mapping.observe(**inputs(doc, .3, hit, hit), dynamic_cells=[(7, 3)])
    assert runner.tick(.3)['status'] == 'STOPPING'
    for t in (.4, .5, .6):
        runner.tick(t)
    assert runner.state == 'WAITING_FOR_OBSERVATION'
    assert len(app.tasks) == 1
    mapping.observe(**inputs(doc, .7, hit, hit), dynamic_cells=[(7, 3)])
    runner.tick(.7, pose_xy=(.35, .35), pose_observed_s=.2)
    assert len(app.tasks) == 1
    result = runner.tick(.8, pose_xy=(.35, .35), pose_observed_s=.8)
    assert result['status'] == 'MOVING' and result['replans'] == 1
    assert result['request_id'] == 'go-to-table'
    assert app.plans[runner.active]['goal'] == app.plans['go-to-table']['goal']
    assert any(point['y'] != .35 for point in app.plans[runner.active]['path_xy'])
    assert app.status('go-to-table')['simulated_stop_confirmed']
    assert not mapping.store.records('living_map')


def test_off_route_changes_do_not_interrupt_movement(tmp_path):
    doc, mapping, app, runner = mission(tmp_path)
    hit = occupied(doc, [(10, 11)])
    mapping.observe(**inputs(doc, .3, hit, hit))
    assert runner.tick(.3)['status'] == 'MOVING'
    assert not any(row['kind'] == 'stop' for row in app.adapter.log)


def test_blocked_goal_waits_then_automatically_resumes_after_measured_clear(tmp_path):
    doc, mapping, app, runner = mission(tmp_path)
    hit = occupied(doc, [(11, 3)])
    mapping.observe(**inputs(doc, .3, hit, hit))
    runner.tick(.3)
    for t in (.4, .5, .6): runner.tick(t)
    mapping.observe(**inputs(doc, .7, hit, hit))
    assert runner.tick(.7, pose_xy=(.35, .35), pose_observed_s=.7)['status'] == 'WAITING_FOR_PATH'
    mapping.observe(**inputs(doc, .8))
    runner.tick(.8, pose_xy=(.35, .35), pose_observed_s=.8)
    assert len(app.tasks) == 1
    mapping.observe(**inputs(doc, 1.))
    assert runner.tick(1., pose_xy=(.35, .35), pose_observed_s=1.)['status'] == 'MOVING'
    assert len(app.tasks) == 2


def test_cancel_while_waiting_never_restarts(tmp_path):
    doc, mapping, app, runner = mission(tmp_path)
    hit = occupied(doc, [(7, 3)])
    mapping.observe(**inputs(doc, .3, hit, hit)); runner.tick(.3)
    runner.cancel(.4)
    for t in (.5, .6, .7):
        mapping.observe(**inputs(doc, t))
        assert runner.tick(t, pose_xy=(.35, .35), pose_observed_s=t)['status'] == 'CANCELLED'
    assert len(app.tasks) == 1


def test_unconfirmed_stop_never_replans(tmp_path):
    doc, mapping, app, runner = mission(tmp_path)
    app.adapter.never_stop = True
    hit = occupied(doc, [(7, 3)])
    mapping.observe(**inputs(doc, .3, hit, hit)); runner.tick(.3)
    for t in (.4, .7, 1., 2., 3.):
        mapping.observe(**inputs(doc, t, hit, hit))
        runner.tick(t, pose_xy=(.35, .35), pose_observed_s=t)
    assert len(app.tasks) == 1 and runner.state != 'MOVING'


def test_existing_semantic_capture_drives_persistence_without_manual_cell_selection(tmp_path):
    from home_robot_tasks.assistant.contracts import Context
    doc = grid(); mapping = living(tmp_path)
    registered_fixture(mapping, doc)
    hit = occupied(doc, [(8, 8)])
    for t in (1., 2., 3.):
        context = Context(mapping.stable.map_id, mapping.stable.revision, 'cal', 'clock')
        stamp = Stamp(t, context, f'capture-{t}').to_dict()
        capture = dict(provenance=dict(stamp=stamp), observations=[dict(
            observation_id=f'obs-{t}', category='table', detector_score=.9,
            stamp=stamp, source='simulation', voxel_size_m=.1,
            observed_voxels=[[8, 8, z] for z in range(4, 8)],
            extent_complete=False, depth_valid_fraction=1.)])
        result = mapping.observe_capture(semantic_capture=capture, identity_matches=identity(t, [(8, 8)]), **inputs(doc, t, hit, hit))
        assert result['semantic_evidence_rejected'] is None
    assert result['saved'] and mapping.stable.rows[8][8] == '#'


def test_delayed_semantics_cannot_block_current_hazard_update(tmp_path):
    doc = grid(); mapping = living(tmp_path)
    result = mapping.observe_capture(semantic_capture={'provenance': {}},
                                    **inputs(doc, 1., occupied(doc, [(8, 8)])))
    assert result['semantic_evidence_rejected']
    assert mapping.live.nav.rows[8][8] == '#' and mapping.stable.rows[8][8] == '.'


def test_replanned_navigation_finishes_same_mission(tmp_path):
    doc, mapping, app, runner = mission(tmp_path)
    hit = occupied(doc, [(7, 3)])
    mapping.observe(**inputs(doc, .3, hit, hit)); runner.tick(.3)
    for i in range(4, 150):
        t = i/10
        mapping.observe(**inputs(doc, t, hit, hit))
        result = runner.tick(t, pose_xy=(.35, .35), pose_observed_s=t)
        if result['status'] == 'SUCCEEDED':
            break
    assert result['status'] == 'SUCCEEDED'
    assert result['request_id'] == 'go-to-table' and result['replans'] == 1
    assert not app.lease.owner and result['hardware_commands'] == 0


def test_sensor_rays_use_actual_shifted_map_origin():
    from home_robot_tasks.navigation_depth import DepthProfile, DepthVolume
    from home_robot_tasks.navigation_fusion import TravelEnvelope
    doc = grid(); doc['origin'] = dict(x=.03, y=-.07)
    p = DepthProfile('depth','cal',.1,.1,3.,0.,1.,.5,.5,world_frame='map')
    volume = DepthVolume(p); volume.reset('epoch')
    volume.last_stamp = 1.; volume.error = None
    updater = AdaptiveMap(doc, calibration_id='cal', clock_id='clock')
    observation = updater.sensor_observation(ranges=[.8,.8], angle_min=0., angle_increment=.01,
        range_min=.12, range_max=3.5, lidar_xy_yaw=(.35,.35,0.), volume=volume,
        envelope=TravelEnvelope('fixture-body',.04,1.), expected_epoch='epoch',
        observed_s=1., now_s=1., capture_id='shifted-sensor', clock_id='clock',
        localization_qualified=True)
    assert observation['lidar_rows'][4][11] == '#'
    assert observation['lidar_rows'][4][3:11] == '.' * 8
    assert observation['lidar_rows'][3][11] == '?'


def test_living_map_protected_structure_never_erased(tmp_path):
    doc = grid(); doc['rows'] = occupied(doc, [(6, 6)])
    mapping = living(tmp_path, doc, protected_cells=[(6, 6)])
    for t in (1., 2., 3.):
        mapping.observe(**inputs(doc, t, grid()['rows'], grid()['rows']))
    assert mapping.live.nav.rows[6][6] == mapping.stable.rows[6][6] == '#'


def test_hazard_is_handled_before_first_executor_command(tmp_path):
    from home_robot_tasks.application import RobotApplication
    from home_robot_tasks.adaptive_mapping import LivingMapNavigation
    from home_robot_tasks.fake_robot import FakeRobot
    doc = grid(); mapping = living(tmp_path)
    app = RobotApplication(read('home.example.json'), mapping.live.nav, FakeRobot(), initial_xy=(.35,.35))
    runner = LivingMapNavigation(app, mapping)
    mapping.observe(**inputs(doc, 0.))
    runner.start(dict(schema_version=1, operation='navigate_to', request_id='before-start',
        map_id=app.map.map_id, map_revision=app.map.revision, floor_id=app.map.floor_id,
        frame_id=app.map.frame_id, point=dict(x=1.15,y=.35,z=0.),yaw_rad=0.), 0.)
    hit = occupied(doc, [(7, 3)])
    mapping.observe(**inputs(doc, .1, hit, hit))
    runner.tick(.1)
    assert not any(event['kind'] == 'start' for event in app.adapter.log)


def test_new_recognized_furniture_never_auto_registers_even_after_many_views(tmp_path):
    doc = grid(); mapping = living(tmp_path)
    hit = occupied(doc, [(8, 8)])
    for i in range(1, 15):
        t = float(i)
        result = mapping.observe(**inputs(doc, t, hit, hit), furniture_cells=[(8, 8)])
        assert not result['saved']
    assert mapping.live.nav.rows[8][8] == '#'
    assert mapping.stable.rows[8][8] == '.' and not mapping.registry
    assert mapping.store.record('living_map', doc['map_id']) is None


def test_user_add_scan_permits_new_furniture_then_persists_registration(tmp_path):
    doc = grid(); mapping = living(tmp_path)
    hit = occupied(doc, [(8, 8)])
    mapping.observe(**inputs(doc, 1., hit, hit), furniture_cells=[(8, 8)])
    mapping.register_furniture('registered-table', [(8, 8)], registration_id='user-add',
                               source='user_requested_local_scan', now_s=1.)
    for t in (2., 3., 4.):
        result = mapping.observe(**inputs(doc, t, hit, hit), furniture_cells=[(8, 8)],
                                 identity_matches=identity(t, [(8, 8)]))
    assert result['saved'] and mapping.stable.rows[8][8] == '#'
    restored = living(tmp_path)
    assert restored.registry == mapping.registry
    assert restored.registry['registered-table']['registration']['source'] == 'user_requested_local_scan'


def test_unverified_or_unknown_identity_cannot_promote_new_position(tmp_path):
    doc = grid(); mapping = living(tmp_path)
    registered_fixture(mapping, doc)
    hit = occupied(doc, [(9, 9)])
    for t in (1., 2., 3.):
        matches = identity(t, [(9, 9)])
        matches[0]['identity_verified'] = False
        result = mapping.observe(**inputs(doc, t, hit, hit), furniture_cells=[(9, 9)], identity_matches=matches)
        assert result['identity_matches_rejected']
    assert mapping.live.nav.rows[9][9] == '#' and mapping.stable.rows[9][9] == '.'
    result = mapping.observe(**inputs(doc, 4., hit, hit), furniture_cells=[(9, 9)],
                             identity_matches=identity(4., [(9, 9)], 'unregistered-chair'))
    assert result['identity_matches_rejected'] and not result['saved']


def test_unknown_baseline_structure_not_removed_by_furniture_update(tmp_path):
    doc = grid(); doc['rows'] = occupied(doc, [(6, 6)])
    mapping = living(tmp_path, doc)
    for t in (1., 2., 3.):
        mapping.observe(**inputs(doc, t, grid()['rows'], grid()['rows']))
    assert mapping.stable.rows[6][6] == '#'
    assert mapping.live.nav.rows[6][6] == '#'


def test_registration_requires_explicit_source_and_fresh_local_scan(tmp_path):
    doc = grid(); mapping = living(tmp_path)
    registered_fixture(mapping, doc)
    with pytest.raises(InvalidTask, match='explicit_furniture_registration'):
        mapping.register_furniture('new', [(9, 9)], registration_id='auto', source='model_detection', now_s=0.)
    with pytest.raises(InvalidTask, match='fresh_registration_scan'):
        mapping.register_furniture('new', [(9, 9)], registration_id='user-new', source='user_requested_local_scan', now_s=10.)
    with pytest.raises(InvalidTask, match='unambiguous'):
        mapping.register_furniture('new', [(9, 9)], registration_id='user-new', source='accepted_initial_map', now_s=0.)
