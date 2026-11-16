import json
import threading
import time
from dataclasses import replace

import pytest

from test_home_assistant import environment, AUTH
from home_perception.language.local_model import LocalLanguageModel
from home_perception.language.task_parser import parse_reply
from home_perception.language.target_selection import select_target
from home_robot_tasks.assistant.local_language import LanguageJobs, meaning_from_slots
from home_robot_tasks.fetch import InvalidTask

UTTERANCE = '거실 탁자에 있는 검은 마커펜을 침실 협탁에 놓아줘'
SLOTS = dict(target='검은 마커펜', source='거실 탁자', destination='침실 협탁', excluded=[])


def extracted(text=UTTERANCE, rid='local-1', slots=None):
    return {'extraction': parse_reply(json.dumps(slots or SLOTS), rid, 'test', text).to_dict(),
            'model': 'test-double', 'latency_s': 0.1}


class ControlledModel:
    def __init__(self):
        self.entered, self.release = threading.Event(), threading.Event()
        self.fail = False

    def extract(self, text, rid):
        self.entered.set()
        assert self.release.wait(2)
        if self.fail:
            raise TimeoutError('model not available')
        return extracted(text, rid)


@pytest.fixture
def jobs(environment):
    _, service, gateway = environment
    model = ControlledModel()
    gateway.language = LanguageJobs(service, model)
    yield model, service, gateway
    model.release.set()
    gateway.language.close()
    gateway.language.pool.shutdown(wait=True)


def finish(gateway):
    until = time.monotonic()+2
    while gateway.language.pending and time.monotonic() < until:
        gateway.tick()
        time.sleep(.001)
    assert not gateway.language.pending


def submit(gateway, rid='local-1', text=UTTERANCE):
    return gateway.dispatch('POST', '/v1/assistant/language-requests', AUTH,
                            dict(request_id=rid, utterance=text))


def test_inference_does_not_block_control_or_execute_and_is_idempotent(jobs):
    model, service, gateway = jobs
    assert submit(gateway)[0] == 202
    assert model.entered.wait(1)
    assert gateway.dispatch('GET', '/v1/health', AUTH, None)[0] == 200
    gateway.tick()
    assert submit(gateway)[1]['status'] == 'INTERPRETING'
    with pytest.raises(InvalidTask, match='request_id_conflict'):
        submit(gateway, text='컵 가져와')
    with pytest.raises(InvalidTask, match='language_busy'):
        submit(gateway, rid='local-2')
    model.release.set(); finish(gateway)
    job = gateway.language.get('local-1')
    assert job['status'] == 'READY'
    assert job['result']['status'] == 'READY_TO_PLAN'
    assert job['result']['intent']['parser'] == 'local_language_v1'
    preview = service.preview('local-1')
    assert preview['confirmation']['required']
    assert not service.app.tasks
    code, _ = gateway.dispatch('POST', '/v1/assistant/requests/local-1/execute', AUTH,
                               {'preview_id': preview['preview_id']})
    assert code == 409 and not service.app.tasks


def test_text_cancel_ignores_slow_model_and_discards_late_reply(jobs):
    model, service, gateway = jobs
    submit(gateway); assert model.entered.wait(1)
    code, value = submit(gateway, rid='cancel-now', text='취소')
    assert code == 200 and value['status'] == 'NO_ACTIVE_TASK'
    assert gateway.language.get('local-1')['status'] == 'CANCELLED'
    model.release.set(); finish(gateway)
    assert service.store.record('intent', 'local-1') is None
    assert not service.app.tasks


def test_failed_model_does_not_fall_back_or_make_intent(jobs):
    model, service, gateway = jobs
    model.fail = True
    submit(gateway); model.release.set(); finish(gateway)
    assert gateway.language.get('local-1')['status'] == 'FAILED'
    assert service.store.record('intent', 'local-1') is None


def test_map_changed_while_interpreting_needs_fresh_request(jobs):
    model, service, gateway = jobs
    submit(gateway)
    service.context = replace(service.context, map_revision='new')
    model.release.set(); finish(gateway)
    assert gateway.language.get('local-1')['status'] == 'INTERRUPTED'
    assert service.store.record('intent', 'local-1') is None


def test_restart_preserves_job_as_interrupted_without_inference(environment):
    _, service, gateway = environment
    service.store.save_record('language', 'before-restart', dict(
        request_id='before-restart', utterance=UTTERANCE, status='INTERPRETING'))
    jobs = LanguageJobs(service, None)
    try:
        assert jobs.get('before-restart')['status'] == 'INTERRUPTED'
        assert not jobs.pending and not service.app.tasks
    finally:
        jobs.close()


@pytest.mark.parametrize('url', ['https://127.0.0.1:80', 'http://example.com',
    'http://127.0.0.1@evil.com', 'http://127.0.0.1/v1', 'http://localhost:80'])
def test_model_endpoint_is_local_only(url):
    with pytest.raises(ValueError): LocalLanguageModel(url)


@pytest.mark.parametrize('sentence', ['컵 가져오지 마', '컵 집고 리모컨 가져와', '컵 두 개 가져와'])
def test_original_scope_holes_are_held(sentence):
    item = parse_reply('{"target":"컵"}', 'r', 'u', sentence)
    assert item.action is None and item.unresolved


@pytest.mark.parametrize('reply', ['{"target":"컵","target":"병"}',
    '{"target":"컵","excluded":"리모컨"}', '{"target":"컵","tool_call":"run"}',
    '{"target":"컵","pose":{"x":1}}'])
def test_untrusted_reply_schema_is_bounded(reply):
    assert 'invalid_reply' in parse_reply(reply, 'r', 'u', '컵 집어줘').unresolved


def test_target_ties_never_depend_on_input_order():
    a, b = {'box_xyxy':[0,0,10,10]}, {'box_xyxy':[0,2,10,12]}
    for values in ([a,b], [b,a]):
        assert select_target(values, 'left') == (None, 'AMBIGUOUS_TARGET')


@pytest.mark.parametrize('sentence,slots,issue', [
    ('왼쪽 컵을 협탁에 놓아줘', dict(target='컵', destination='협탁'), 'relative_location_requires_observation'),
    ('내 컵을 협탁에 놓아줘', dict(target='내 컵', destination='협탁'), 'unsupported_object_constraint'),
    ('파란 컵 말고 빨간 컵을 협탁에 놓아줘', dict(target='빨간 컵', destination='협탁', excluded=['파란 컵']), 'exclusion_requires_clarification'),
    ('수첩을 협탁에 놓아줘', dict(target='수첩', destination='협탁'), 'object_category_required')])
def test_bridge_does_not_discard_unsupported_constraints(sentence,slots,issue):
    item = extracted(sentence, slots=slots)['extraction']
    assert issue in meaning_from_slots(item)['issues']


@pytest.mark.parametrize('utterance,target,expected', [
    ('내 컵을 책상으로 옮겨줘','컵','내 컵'),
    ('흰색 컵을 책상으로 옮겨줘','컵','흰색 컵'),
    ('책상 위 작은 빨간 컵을 협탁에 놔줘','컵','작은 빨간 컵')])
def test_model_cannot_omit_literal_modifiers(utterance,target,expected):
    item = parse_reply(json.dumps(dict(target=target,destination='책상')), 'r','u',utterance)
    assert item.target == expected


@pytest.mark.parametrize('utterance,source,destination,expected_source,expected_dest', [
    ('수첩을 선반으로 옮겨줘','선반',None,None,'선반'),
    ('컵을 책상 위에 올려줘','책상','책상',None,'책상'),
    ('책상에 있는 컵을 책상 위에 놔줘','책상','책상','책상','책상'),
    ('책상 위 컵을 협탁에 놔줘','책상','협탁','책상','협탁')])
def test_place_particle_normalization_only_for_clear_single_occurrence(utterance,source,destination,expected_source,expected_dest):
    target = '수첩' if '수첩' in utterance else '컵'
    item = parse_reply(json.dumps(dict(target=target,source=source,destination=destination)), 'r','u',utterance)
    assert (item.source,item.destination) == (expected_source,expected_dest)


def test_cancel_is_available_when_history_full_and_gateway_faulted(jobs):
    model, service, gateway = jobs
    submit(gateway); assert model.entered.wait(1)
    for i in range(128):
        service.store.save_record('intent', f'history-{i}', {'request_id':f'history-{i}'})
    gateway.faulted = True
    code, result = submit(gateway, rid='cancel-full', text='취소')
    assert code == 200 and result['status'] == 'NO_ACTIVE_TASK'
    assert gateway.language.get('local-1')['status'] == 'CANCELLED'
    model.release.set(); finish(gateway)
    assert service.store.record('intent', 'local-1') is None


@pytest.mark.parametrize('text', ['컵과 리모컨을 협탁에 옮겨줘','컵 2개를 협탁에 놓아줘',
    '검은 컵하고 흰 컵을 협탁에 놓아줘'])
def test_dropped_extra_object_or_count_cannot_authorize_single_task(text):
    from home_robot_tasks.assistant.meaning import apply_interaction_policy, intent_from_meaning
    item=extracted(text,slots=dict(target='컵',destination='협탁'))['extraction']
    intent=apply_interaction_policy(intent_from_meaning(text,'r',meaning_from_slots(item)))
    assert 'single_task_required' in intent['issues']
