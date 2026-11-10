"""Architecture-level regression: ambiguity, revisions, replay, cancellation."""
from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from home_robot_tasks.application import RobotApplication
from home_robot_tasks.fake_robot import FakeRobot
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.gateway import Gateway, GatewayStore
from home_robot_tasks.navigation import NavigationMap
from home_robot_tasks.assistant.contracts import Context, Stamp, make_entity
from home_robot_tasks.assistant.world import WorldStore
from home_robot_tasks.assistant.language import parse_request, ground_intent
from home_robot_tasks.assistant.scanning import ScanManager, infer_room, propose_regions
from home_robot_tasks.assistant.placement import PlacementPlanner, PlacementProfile, Check, REQUIRED_PHASES
from home_robot_tasks.assistant.perception import TransformSample, report_entities, associate_observation
from home_robot_tasks.assistant.service import AssistantService
from home_robot_tasks.assistant.capabilities import Device, DeviceRegistry

ROOT = Path(__file__).resolve().parents[1]
AUTH = 'Bearer ' + 't'*48


def read(name):
    return json.loads((ROOT/'config'/name).read_text())


@pytest.fixture
def environment(tmp_path):
    now = [100.0]
    nav = NavigationMap(read('navigation_map.simulation.json'))
    app = RobotApplication(read('home.example.json'), nav, FakeRobot())
    context = Context(nav.map_id, nav.revision, 'simulation-calibration', 'test-clock')
    store = WorldStore(tmp_path/'world.db')
    service = AssistantService(app, store, context, lambda: now[0])
    service.refresh_demo()
    gateway = Gateway(app, 't'*48, GatewayStore(tmp_path/'requests.db'), assistant=service)
    gateway.now = lambda: now[0]-100
    yield now, service, gateway
    gateway.store.db.close()
    store.close()


def test_korean_sentence_resolves_source_object_destination_without_motion(environment):
    now, service, gateway = environment
    value = service.prepare('거실 탁자에 있는 검은색 마커펜을 침실 협탁 위에 놓아줘', 'request-1')
    assert value['status'] == 'READY_TO_PLAN'
    g = value['grounding']
    assert g['object_candidates'][0]['entity']['entity_id'] == 'marker-1'
    assert g['destination_candidates'][0]['entity_id'] == 'nightstand'
    assert service.app.lease.owner is None
    assert service.prepare(value['utterance'], 'request-1') == value
    with pytest.raises(InvalidTask, match='request_id_conflict'):
        service.prepare('컵 찾아줘', 'request-1')
    preview = service.preview('request-1')
    assert preview['placement']['status'] == 'CANDIDATES_READY'
    assert preview['placement']['evaluated'] == 12  # three distances × four azimuths × one lift height
    assert {c['lift_height_m'] for c in preview['placement']['candidates']} == {.4}
    assert not preview['executable'] and preview['hardware_commands'] == 0
    assert not service.app.tasks


def test_ambiguity_requires_matching_choice_and_missing_destination_can_be_supplied(environment):
    _, service, _ = environment
    value = service.prepare('검은 컵을 협탁에 놓아줘', 'cups')
    assert value['status'] == 'NEEDS_CLARIFICATION'
    assert 'ambiguous_object' in value['grounding']['issues']
    with pytest.raises(InvalidTask):
        service.clarify('cups', {'object_id': 'marker-1'})
    value = service.clarify('cups', {'object_id': 'cup-2'})
    assert value['status'] == 'READY_TO_PLAN'
    assert service.preview('cups')['target']['entity_id'] == 'cup-2'
    value = service.prepare('마커펜 가져다줘', 'missing-place')
    assert 'destination_required' in value['grounding']['issues']
    assert service.clarify('missing-place', {'destination_id': 'nightstand'})['status'] == 'READY_TO_PLAN'


@pytest.mark.parametrize('utterance', ['검은 컵 말고 흰 컵 가져다줘', '컵을 놓지 마', '내 컵을 협탁에 놓아줘',
                                      '작은 컵을 협탁에 놓아줘', '컵 가져오고 그리고 문 열어줘'])
def test_unsupported_constraints_never_silently_drop(utterance, environment):
    _, service, _ = environment
    assert service.prepare(utterance, 'unsupported')['status'] == 'NEEDS_CLARIFICATION'
    assert not service.app.tasks


def test_stale_memory_is_search_hint_not_fresh_execution(environment):
    now, service, _ = environment
    now[0] += 61
    value = service.prepare('검은 마커펜을 협탁에 놓아줘', 'stale')
    assert value['status'] == 'REOBSERVE_REQUIRED'
    assert value['grounding']['object_candidates']
    with pytest.raises(InvalidTask): service.preview('stale')
    context = Context(service.context.map_id, service.context.map_revision, service.context.calibration_id, 'another-clock')
    matches = service.store.candidates(context=context, category='marker', now_s=100)
    assert len(matches) == 1 and not matches[0]['fresh']


def test_world_revision_aliases_and_object_transitions(environment):
    now, service, _ = environment
    store = service.store
    old = store.get('marker-1')
    updated = store.aliases('marker-1', ['내 마커'], old['revision'])
    assert updated['stamp'] == old['stamp']
    with pytest.raises(InvalidTask, match='revision_conflict'):
        store.aliases('marker-1', ['다른 별칭'], old['revision'])
    stamp = Stamp(now[0], service.context, 'carry-proof')
    with pytest.raises(InvalidTask, match='evidence'):
        store.mark_state('marker-1', 'carried', stamp, expected_revision=updated['revision'])
    proof = dict(verified=True, entity_id='marker-1', capture_id=stamp.capture_id)
    carried = store.mark_state('marker-1', 'carried', stamp, expected_revision=updated['revision'], evidence=proof)
    assert carried['position_m'] is None and carried['support_id'] is None
    assert not store.candidates(context=service.context, category='marker', now_s=now[0])
    placed = store.mark_state('marker-1', 'placed', stamp, expected_revision=carried['revision'],
                             evidence=proof, support_id='nightstand', position_m=[7, 3, .8])
    assert placed['room_id'] == 'bedroom' and placed['support_id'] == 'nightstand'
    bad = deepcopy(placed); bad.update(room_id='living_room', revision=placed['revision']+1)
    with pytest.raises(InvalidTask, match='support_room_mismatch'):
        store.put(bad, expected_revision=placed['revision'])
    assert store.get('marker-1') == placed


def test_map_immutable_and_old_context_not_returned(environment):
    _, service, _ = environment
    doc = deepcopy(service.app.map._document)
    doc['rows'][1] = '#??????????????????#'
    with pytest.raises(InvalidTask, match='map_revision_reused'):
        service.store.add_map(doc)
    new_context = Context(service.context.map_id, 'new-map', service.context.calibration_id, service.context.clock_id)
    assert service.store.entities(new_context) == []


def test_scan_lifecycle_review_save_restart_and_stale_evidence(environment):
    now, service, _ = environment
    scan = service.scan
    scan.start('scan-1', '거실 스캔', mode='simulation')
    assert scan.active() and service.app.admission() is False
    scan.ingest_map('scan-1', service.app.map._document)
    observation = dict(stamp=Stamp(now[0], service.context, 'scan-capture').to_dict(),
                       xy_m=[3, 3], categories=['sofa', 'coffee_table'], surface_ids=['coffee_table'])
    value = scan.observe('scan-1', observation, now_s=now[0])
    assert value['coverage']['whole_home_fraction'] is None
    assert value['coverage']['surface_views'] == 1
    assert any(r['classification']['label'] == 'living_room' for r in value['regions'])
    assert scan.observe('scan-1', observation, now_s=now[0]) == value
    other = deepcopy(observation); other['categories'] = ['bed']
    with pytest.raises(InvalidTask, match='capture_id_conflict'):
        scan.observe('scan-1', other, now_s=now[0])
    restarted = ScanManager(service.store, service.context)
    assert restarted.get('scan-1')['status'] == 'PAUSED'
    restarted.resume('scan-1'); restarted.review('scan-1')
    region = restarted.get('scan-1')['regions'][0]['region_id']
    restarted.label('scan-1', region, '거실')
    assert restarted.save('scan-1')['status'] == 'SAVED'
    assert not restarted.active()
    assert restarted.save('scan-1')['status'] == 'SAVED'
    with pytest.raises(InvalidTask): restarted.resume('scan-1')


def test_room_proposals_keep_uncertainty_and_do_not_cross_unknown(environment):
    assert infer_room(['sink'])['status'] == 'uncertain'
    assert infer_room(['bed', 'sofa'])['status'] == 'uncertain'
    assert infer_room(['bed'])['label'] == 'bedroom'
    _, service, _ = environment
    doc = deepcopy(service.app.map._document)
    doc['rows'] = ['####################'] + ['#.......?...#......#']*10 + ['####################']
    proposal = propose_regions(doc)
    for region in proposal['regions']:
        assert all(doc['rows'][y][x] == '.' for x, y in region['cells'])
    assert len(proposal['regions']) >= 2


class Oracle:
    def navigation(self, c): return Check(True, '', cost=1, mode='simulation')
    def view(self, c): return Check(c.lift_height_m > .1, 'visibility', margin=.8, mode='simulation')
    def whole_motion(self, c): return Check(True, '', margin=.7, phases=tuple(REQUIRED_PHASES), mode='simulation')


def test_joint_base_height_selection_rejects_endpoint_only_stale_and_synthetic_proof(environment):
    now, service, _ = environment
    profile = PlacementProfile(0, .6, (0, .2), (.4,), (0,))
    arguments = dict(target_m=[3, 3, .7], stamp=Stamp(now[0], service.context, 'target'),
                     context=service.context, now_s=now[0], scene_revision='scene-1', grasp_ids=['g1'],
                     current_xy_yaw=[1, 1, 0], current_height_m=0)
    planner = PlacementPlanner(profile, Oracle())
    plan = planner.plan(**arguments)
    assert len(plan['candidates']) == 1 and plan['candidates'][0]['lift_height_m'] == .2
    assert plan['evaluated'] == 2 and not plan['executable']
    with pytest.raises(InvalidTask, match='invalidated'):
        planner.validate_handoff(plan, context=service.context, scene_revision='scene-2', now_s=now[0])
    with pytest.raises(InvalidTask, match='stale'):
        planner.plan(**{**arguments, 'now_s': now[0]+3})
    oracle = Oracle(); oracle.whole_motion = lambda c: Check(True, '', phases=('contact',), mode='simulation')
    assert PlacementPlanner(profile, oracle).plan(**arguments)['status'] == 'UNREACHABLE'
    measured = PlacementProfile(0, .6, (.2,), (.4,), (0,), source='measured')
    assert PlacementPlanner(measured, Oracle()).plan(**arguments)['status'] == 'UNREACHABLE'


def test_camera_transform_provenance_and_surface_not_grasp(environment):
    now, service, _ = environment
    stamp = Stamp(now[0], service.context, 'camera-1')
    report = {'schema_version': 2, 'coordinate_frame': 'color_camera_optical_frame',
              'detections': [{'id': 0, 'label': 'black marker pen', 'score': .8,
                              'surface_point': {'valid': True, 'xyz_m': [.1, .2, .5]}}]}
    args = dict(stamp=stamp, context=service.context, now_s=now[0])
    assert report_entities(report, **args)[0]['reason'] == 'camera_extrinsics_required'
    transform = TransformSample(stamp, 'color_camera_optical_frame', 'map',
                                [[1,0,0,1],[0,0,-1,2],[0,1,0,3],[0,0,0,1]], 'simulation')
    result = report_entities(report, transform=transform, **args)[0]
    assert result['entity']['position_m'] == pytest.approx([1.1, 1.5, 3.2])
    assert result['grasp_pose'] is None and result['association_required']
    bad = TransformSample(stamp, transform.source_frame, 'map', np.diag([1,1,-1,1]), 'measured')
    with pytest.raises(InvalidTask, match='rotation'):
        report_entities(report, transform=bad, **args)
    duplicate = deepcopy(result['entity']); duplicate['entity_id'] = 'another'
    association = associate_observation(result['entity'], [result['entity'], duplicate], maximum_distance_m=.1)
    assert len(association['candidate_ids']) == 2 and not association['automatic_identity_verified']


def test_left_only_devices_and_declared_missing_are_distinct():
    left = Device('left_arm', 'uart1', (1,2,3,4,5,6), tuple('joint-'+str(i) for i in range(6)))
    wheels = Device('base', 'uart1', (7,8,9))
    registry = DeviceRegistry([left, wheels])
    observed = {'left_arm': dict(observed_s=1, motor_ids=list(left.motor_ids), healthy=True,
                                positions_rad={name:0 for name in left.joint_names}),
                'base': dict(observed_s=1, motor_ids=[7,8,9], healthy=True, positions_rad={})}
    assert registry.evaluate(observed, now_s=1.1, maximum_age_s=.2)['ready']
    assert 'right_arm' not in registry.evaluate(observed, now_s=1.1, maximum_age_s=.2)['devices']
    del observed['base']
    assert not registry.evaluate(observed, now_s=1.1, maximum_age_s=.2)['ready']
    with pytest.raises(InvalidTask): DeviceRegistry([left, Device('conflict', 'uart1', (6,))])


def test_gateway_end_to_end_same_executor_persistence_and_world_delivery(environment):
    now, service, gateway = environment
    code, _ = gateway.dispatch('POST', '/v1/assistant/requests', AUTH,
                              {'request_id': 'delivery', 'utterance': '검은 마커펜을 협탁에 놓아줘'})
    assert code == 200
    _, preview = gateway.dispatch('POST', '/v1/assistant/requests/delivery/preview', AUTH, {})
    path = '/v1/assistant/requests/delivery/execute'
    body = {'preview_id': preview['preview_id']}
    assert gateway.dispatch('POST', path, AUTH, body)[0] == 409
    permit = gateway.dispatch('GET', '/v1/permit', AUTH, None)[1]['permit']
    code, result = gateway.dispatch('POST', path, AUTH, body, permit)
    assert code == 202 and service.app.lease.owner == result['request_id']
    assert gateway.dispatch('POST', path, AUTH, body)[0] == 200
    for _ in range(150):
        gateway.tick(); now[0] += .05
    done = service.get('delivery')
    assert done['status'] == 'SUCCEEDED'
    assert done['physical_task_completed'] is False
    assert len(service.app.tasks) == 1
    assert service.store.get('marker-1')['support_id'] == 'nightstand'
    assert service.store.get('marker-1')['state'] == 'placed'
    assert not service.app.pose_known  # Synthetic fetch cannot invent final localization.


def test_gateway_auth_and_cancel_shared_ownership(environment):
    now, service, gateway = environment
    assert gateway.dispatch('GET', '/v1/assistant/state', 'bad', None)[0] == 401
    service.prepare('마커펜을 협탁에 놓아줘', 'cancel-1')
    preview = service.preview('cancel-1')
    permit = gateway.dispatch('GET', '/v1/permit', AUTH, None)[1]['permit']
    gateway.dispatch('POST', '/v1/assistant/requests/cancel-1/execute', AUTH, {'preview_id':preview['preview_id']}, permit)
    gateway.tick()
    code, result = gateway.dispatch('POST', '/v1/assistant/requests/cancel-1/cancel', AUTH, {})
    assert code == 202 and result['control_owned']
    now[0] += .2; gateway.tick()
    assert service.app.lease.owner is None
    assert service.get('cancel-1')['status'] == 'CANCELLED'


def test_changed_preview_and_scan_block_motion(environment):
    _, service, gateway = environment
    service.prepare('마커펜을 협탁에 놓아줘', 'change')
    preview = service.preview('change')
    service.store.aliases('marker-1', ['changed'], service.store.get('marker-1')['revision'])
    with pytest.raises(InvalidTask, match='preview_invalidated'):
        service.envelope('change', preview['preview_id'])
    service.scan.start('active-scan', '스캔', mode='simulation')
    fresh = service.preview('change')
    with pytest.raises(InvalidTask, match='scan_active'):
        service.envelope('change', fresh['preview_id'])
    assert not service.app.tasks
