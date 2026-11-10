"""Cross-layer regressions that do not require ROS, cameras or a robot."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as N

import numpy as np
import pytest

from test_home_assistant import environment, AUTH, read, Oracle
from test_moveit_bridge_mapping import NAMES
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.assistant.contracts import Stamp
from home_robot_tasks.assistant.service import AssistantService
from home_robot_tasks.assistant.placement import PlacementProfile
from home_robot_tasks.assistant.placement_binding import TaskPlacementPolicy
from home_robot_tasks.capture_port import detection_samples
from home_robot_tasks.grounding_detector import GroundingSAMDetector
from home_robot_tasks.moveit_bridge import MoveItBridge
from home_robot_tasks.robot_state import RobotStateStore, RobotObservation
from home_robot_tasks.mission_catalog import MissionCatalog
from home_robot_tasks.planning_scene_port import SceneUpdatePort


def start(environment, request_id):
    _, service, gateway = environment
    service.prepare('검은 마커펜을 협탁에 놓아줘', request_id)
    preview = service.preview(request_id)
    permit = gateway.dispatch('GET', '/v1/permit', AUTH, None)[1]['permit']
    code, result = gateway.dispatch('POST', f'/v1/assistant/requests/{request_id}/execute',
        AUTH, {'preview_id': preview['preview_id']}, permit)
    assert code == 202
    return result


@pytest.mark.parametrize('utterance', ['줄무늬 컵을 협탁에 놓아줘', '깨진 컵을 협탁에 놓아줘',
    '검은 마커펜을 협탁 옆에 놓아줘', '검은 마커펜을 협탁 아래에 놓아줘',
    'bring the striped cup to nightstand', '마커펜을 없는 방 협탁에 놓아줘'])
def test_unrecognized_qualifiers_are_not_dropped(utterance, environment):
    _, service, _ = environment
    assert service.prepare(utterance, 'unsupported-detail')['status'] == 'NEEDS_CLARIFICATION'


@pytest.mark.parametrize('utterance', ['bring the black marker to nightstand',
    '검은 마커펜을 침실 협탁에 가져다줘', '거실 탁자 위에 있는 검정 마커펜을 협탁에 놓아줘'])
def test_supported_phrases_remain_usable(utterance, environment):
    _, service, _ = environment
    assert service.prepare(utterance, 'supported')['status'] == 'READY_TO_PLAN'


def test_duplicate_natural_cancel_never_retargets_new_work(environment):
    now, service, gateway = environment
    first = start(environment, 'first')
    cancel = dict(request_id='cancel-once', utterance='멈춰')
    code, result = gateway.dispatch('POST', '/v1/assistant/requests', AUTH, cancel)
    assert code == 200 and result['cancel_target_request_id'] == first['request_id']
    for _ in range(10):
        now[0] += .05; gateway.tick()
    assert service.app.lease.owner is None
    service.app.localize_simulated((1.5, 1.5), gateway.now())
    second = start(environment, 'second')
    gateway.dispatch('POST', '/v1/assistant/requests', AUTH, cancel)
    assert service.app.lease.owner == second['request_id']
    assert service.app.status(second['request_id'])['status'] not in {'CANCELLED', 'STOPPING'}


@pytest.mark.parametrize('status', ['EXECUTION_REQUESTED', 'RUNNING', 'STOPPING', 'STOP_UNCONFIRMED'])
def test_restart_marks_unfinished_work_interrupted_without_submitting(status, environment):
    _, service, _ = environment
    value = service.prepare('마커펜을 협탁에 놓아줘', 'unfinished')
    value.update(status=status, task_request_id='assistant-unfinished')
    service.store.save_record('intent', 'unfinished', value)
    restarted = AssistantService(service.app, service.store, service.context, service.clock)
    assert restarted.get('unfinished')['status'] == 'INTERRUPTED'
    with pytest.raises(InvalidTask): restarted.preview('unfinished')
    assert not service.app.tasks


@pytest.mark.parametrize('path,body', [
    ('scans', dict(session_id='bad',name='bad',mode=[])),
    ('aliases', dict(entity_id=[],aliases=['test'],expected_revision=1)),
    ('aliases', dict(entity_id='marker-1',aliases=['test'],expected_revision=True)),
])
def test_invalid_json_values_are_request_errors_not_controller_faults(path, body, environment):
    _, service, gateway = environment
    previous = service.store.get('marker-1')
    with pytest.raises(InvalidTask):
        gateway.dispatch('POST', '/v1/assistant/'+path, AUTH, body)
    assert not gateway.faulted and not service.app.tasks
    assert service.store.get('marker-1') == previous


def test_scan_publication_has_unconfirmed_semantics_until_reviewed(environment):
    now, service, _ = environment
    scan = service.scan
    scan.start('save-semantic', '집 지도', mode='simulation')
    scan.ingest_map('save-semantic', service.app.map._document)
    scan.observe('save-semantic', dict(stamp=Stamp(now[0], service.context, 'scan-view').to_dict(),
        xy_m=[3,3], categories=['sofa'], surface_ids=[]), now_s=now[0])
    scan.review('save-semantic')
    result = scan.save('save-semantic')
    rooms = [service.store.get(i) for i in result['room_entity_ids']]
    assert rooms and all(r['state'] == 'unconfirmed' for r in rooms)
    assert scan.save('save-semantic') == result
    assert service.prepare(f"{rooms[0]['entity_id']}에 가줘", 'uncertain-room')['status'] == 'NEEDS_CLARIFICATION'


def test_sam_adapter_preserves_rgb_and_excludes_background_depth():
    seen = []
    rgb = np.zeros((10, 12, 3), np.uint8); rgb[:] = [10,20,30]
    mask = np.zeros((10,12), bool); mask[2:8, 4:9] = True
    class Model:
        def detect(self, bgr, prompt):
            seen.append((bgr.copy(), prompt))
            return [dict(box_xyxy=[0,0,12,10], score=.9)], 0
    class Masks:
        def segment(self, bgr, boxes):
            assert boxes == [[0,0,11,9]]
            return [(mask, .95)], 0
    detector = GroundingSAMDetector(Model(), Masks(), prompt='black marker', object_id='marker-query')
    detection = detector(rgb)[0]
    assert seen[0][0][0,0].tolist() == [30,20,10] and rgb[0,0].tolist() == [10,20,30]
    depth = np.full((10,12), 9.0); depth[mask] = .45
    samples = detection_samples(detection, N(depth_m=depth)).samples_uv_depth
    assert samples and all(z == .45 and 4 < x < 8 and 2 < y < 7 for x,y,z in samples)
    mask[:] = False  # Adapter owns a snapshot, not a model's reusable buffer.
    assert detection.mask.any()


def test_left_only_state_and_moveit_never_fabricate_an_absent_arm():
    profile = read('fetch_stack.simulation.json')['state']
    profile['joint_names'] = NAMES[:6]
    for key in ('safe_minimum_rad', 'safe_maximum_rad', 'transport_joints_rad'):
        profile[key] = profile[key][:6]
    state = RobotStateStore(profile, lambda: 1.)
    obs = RobotObservation(1,1,1.,(1.,1.,0.),(0.,)*3,(0.,)*6,10000,0.,
        True,True,True,True,True,True,True,joint_names=tuple(NAMES[:6]))
    state.observe(obs)
    assert state.fresh() and len(state.latest.joints_rad) == 6
    with pytest.raises(InvalidTask):
        RobotStateStore(profile, lambda:1.).observe(replace(obs,joint_names=tuple(NAMES[6:])))
    bridge = MoveItBridge(NAMES[:6], [-1.]*6, [1.]*6, lambda:0.)
    point = lambda q,t: N(positions=q,time_from_start=N(sec=t,nanosec=0))
    trajectory = N(joint_trajectory=N(joint_names=NAMES[:5], points=[point([0.]*5,0),point([.1,0,0,0,0],1)]))
    context = N(anchor_rad=(0.,)*6, joint_names=tuple(NAMES[:6]), frame_id='base_link')
    mapped = bridge.map(trajectory, context)
    assert all(len(p.positions_rad) == 6 and p.positions_rad[5] == 0 for p in mapped)
    with pytest.raises(InvalidTask, match='names mismatch'):
        bridge.map(trajectory, N(anchor_rad=(0.,)*6,joint_names=tuple(NAMES[6:])))
    with pytest.raises(InvalidTask, match='arm/frame'):
        bridge.request(dict(context=context,task={'arm':'right','target':{'frame_id':'base_link'}}))


def test_left_only_scene_rejects_right_arm_before_writing():
    from test_planning_scene_port import Client, change
    client = Client()
    scene = SceneUpdatePort(client, lambda:1., gripper_links={'left':['left_hand','left_finger']})
    with pytest.raises(InvalidTask): scene.start('bad', {**change(), 'arm':'right'})
    assert client.sent == []
    scene.start('good', change())
    assert len(client.sent) == 1


def test_joint_placement_binding_requires_arrival_and_new_grasp_observation(environment):
    now, service, _ = environment
    current = [[1.5,1.5,0], 0.]
    target = {'capture_id':'coarse','position_m':[.1,0,.7],'frame_id':'base_link'}
    catalog = N(map=service.app.map, xy=lambda:current[0][:2], checked_target=lambda placement:deepcopy(target))
    def fresh_grasp(t, candidate, operation, object_id):
        assert candidate['grasp_id'] == 'chosen-grasp'
        return dict(operation=operation, object_id=object_id, arm=candidate['arm'], target=t,
            position_m=[.15,0,.7], quaternion_xyzw=[0,0,0,1])
    policy = TaskPlacementPolicy(profile=PlacementProfile(0,.6,(0,.2,.4),(.4,),(0,)),
        feasibility_factory=lambda *_:Oracle(), target_to_map=lambda t:([3,3,.7],Stamp(now[0],service.context,t['capture_id'])),
        context=lambda:service.context,current_configuration=lambda:current,
        scene_revision=lambda:'scene-1',clock=lambda:now[0],grasp_ids=lambda *_:['chosen-grasp'],
        fresh_grasp_task=fresh_grasp)
    # Exercise MissionCatalog's real delegation, not an alternate executor.
    catalog.placement_policy = policy
    args = {'object_id':'marker-1'}
    with pytest.raises(InvalidTask, match='single_pick_lift_setting_required'):
        MissionCatalog.resolve(catalog, 'align_for_pick', args, now[0])
    policy.pick_height_for = lambda target: .2
    nav = MissionCatalog.resolve(catalog, 'align_for_pick', args, now[0])
    with pytest.raises(InvalidTask, match='base_pose_not_reached'):
        MissionCatalog.resolve(catalog, 'adjust_lift_for_pick', args, now[0])
    current[0] = [nav['goal']['x'],nav['goal']['y'],nav['goal']['yaw_rad']]
    assert MissionCatalog.resolve(catalog,'adjust_lift_for_pick',args,now[0]) == {'height_um':200000}
    with pytest.raises(InvalidTask, match='height_not_reached'):
        MissionCatalog.resolve(catalog,'pick',args,now[0])
    current[1] = .2
    with pytest.raises(InvalidTask, match='reobservation_required'):
        MissionCatalog.resolve(catalog,'pick',args,now[0])
    target['capture_id'] = 'after-alignment'
    task = MissionCatalog.resolve(catalog,'pick',args,now[0])
    assert task['arm'] == 'left' and task['target']['capture_id'] == 'after-alignment'
    assert catalog.manipulation_target == target
    policy.fresh_grasp_task = lambda *a: {**task, 'arm':'right'}
    with pytest.raises(InvalidTask, match='identity_mismatch'):
        MissionCatalog.resolve(catalog,'pick',args,now[0])
