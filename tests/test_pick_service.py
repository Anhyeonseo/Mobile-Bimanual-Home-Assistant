"""No motors/models: check request ownership, cancellation and truthful status."""
from concurrent.futures import Future
import json
import threading
import time

import pytest

from home_robot_tasks.pick_demo import DemoBackend
from home_robot_tasks.pick_service import PickStation, Rejected


def wait_for(predicate, seconds=3):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.005)
    raise AssertionError('condition did not become true')


class RecordingBackend(DemoBackend):
    def __init__(self):
        super().__init__(step_seconds=0)
        self.calls = []
        self.owners = set()
        self.ticks = 0
        self.aborts = 0

    def prepare(self, ctx):
        self.calls.append('prepare')
        super().prepare(ctx)

    def _step(self, name, ctx, value):
        self.calls.append(name)
        self.owners.add(threading.get_ident())
        return super()._step(name, ctx, value)

    def tick(self):
        self.owners.add(threading.get_ident())
        self.ticks += 1

    def abort(self):
        self.aborts += 1


def start(tmp_path, backend, **kwargs):
    station = PickStation(backend, tmp_path, **kwargs)
    station.start()
    wait_for(lambda: station.snapshot()['accepting'])
    return station


def test_two_requests_reuse_preparation_and_do_not_share_capture_or_results(tmp_path):
    backend = RecordingBackend()
    station = start(tmp_path, backend)
    try:
        first = station.submit('리모컨 집기')
        wait_for(lambda: len(station.snapshot()['history']) == 1 and station.snapshot()['accepting'])
        second = station.submit('리모컨 집어줘')
        wait_for(lambda: len(station.snapshot()['history']) == 2 and station.snapshot()['accepting'])
        assert backend.calls == ['prepare']+['capture', 'detect', 'candidates', 'plan', 'execute', 'verify']*2
        assert len(backend.owners) == 1
        assert first != second
        for identifier in (first, second):
            result = json.loads((tmp_path/identifier/'result.json').read_text())
            assert result['result']['physical_success'] is False
            assert result['result']['status'] == 'SUCCEEDED'
            assert all(row['status'] == 'done' for row in result['stages'])
        snap = station.snapshot()
        snap['current']['text'] = 'mutated'
        assert station.snapshot()['current']['text'] != 'mutated'
    finally:
        station.close()


@pytest.mark.parametrize('stop_kind', ['cancel', 'timeout', 'shutdown'])
def test_worker_wait_services_owner_and_never_executes_late_result(tmp_path, stop_kind):
    class Waiting(RecordingBackend):
        future = Future()

        def detect(self, ctx, value):
            self.calls.append('detect')
            return ctx.wait(self.future)

    backend = Waiting()
    station = start(tmp_path, backend, stage_timeout_s=.1 if stop_kind == 'timeout' else 5)
    try:
        identifier = station.submit('리모컨 집기')
        wait_for(lambda: station.snapshot()['phase'] == 'DETECTING')
        with pytest.raises(Rejected):
            station.submit('리모컨 집기')
        previous = backend.ticks
        wait_for(lambda: backend.ticks >= previous+2)
        if stop_kind == 'cancel':
            station.cancel()
        elif stop_kind == 'shutdown':
            station.shutdown.set()
        wait_for(lambda: (tmp_path/identifier/'result.json').exists())
        backend.future.set_result({'late': True})
        result = station.snapshot()['current']['result']
        assert result['status'] == ('FAILED' if stop_kind == 'timeout' else 'CANCELLED')
        assert backend.aborts == 1
        assert 'execute' not in backend.calls
    finally:
        station.close()


def test_failed_stop_latches_request_gate_even_if_backend_claims_ready(tmp_path):
    class BrokenStop(RecordingBackend):
        def execute(self, ctx, value):
            raise ValueError('feedback lost')

        def abort(self):
            raise RuntimeError('stop acknowledgement missing')

    station = start(tmp_path, BrokenStop())
    try:
        station.submit('리모컨 집기')
        wait_for(lambda: station.snapshot()['history'])
        assert station.snapshot()['last_result']['status'] == 'STOP_UNCONFIRMED'
        wait_for(lambda: station.snapshot()['phase'] == 'BLOCKED')
        with pytest.raises(Rejected):
            station.submit('리모컨 집기')
    finally:
        station.close()


@pytest.mark.parametrize('mode,result', [('demo', {'status':'SUCCEEDED','physical_success':True}),
                                       ('live', {'status':'SUCCEEDED','physical_success':False}),
                                       ('live', {'status':'TRAJECTORY_FINISHED'})])
def test_completion_requires_explicit_consistent_verification(tmp_path, mode, result):
    backend = RecordingBackend()
    backend.mode = mode
    backend.verify = lambda *_: result
    station = start(tmp_path, backend)
    try:
        station.submit('리모컨 집기')
        wait_for(lambda: station.snapshot()['history'])
        assert station.snapshot()['last_result']['status'] == 'FAILED'
    finally:
        station.close()


def test_artifacts_cannot_escape_session_directory(tmp_path):
    station = PickStation(DemoBackend(), tmp_path/'session')
    outside = tmp_path/'private.json'
    outside.write_text('{}')
    (station.output/'link.json').symlink_to(outside)
    with pytest.raises(ValueError):
        station.artifact(station.output/'link.json', 'bad')


@pytest.mark.parametrize(('text','target'), [
    ('노란 테이프를 집어줘', 'yellow tape'), ('물병 집기','water bottle'),
    ('pick up the silicone pastry brush','silicone pastry brush'),
    ('pick the coffee filter up','coffee filter'), ('pick remote','remote control'),
    ('파란 컵을 좀 집어 주세요','blue cup'), ('지우개','eraser'),
])
def test_request_extracts_complete_object_description(text, target):
    from home_robot_tasks.pick_target import parse_target
    assert parse_target(text)[1] == target


@pytest.mark.parametrize('text', ['컵 말고 가위 집어줘', '컵을 집지 마', '', '집어줘',
                                  'pick up the cup and bottle', '컵 옆에 있는 연필 집어줘'])
def test_single_target_parser_does_not_drop_unsupported_constraints(text):
    from home_robot_tasks.pick_target import parse_target
    with pytest.raises(ValueError):
        parse_target(text)


def test_unknown_korean_translates_before_capture_and_translation_error_can_retry(tmp_path,monkeypatch):
    backend = RecordingBackend()
    station = start(tmp_path, backend)
    def translate(phrase):
        assert phrase == '실리콘 붓'
        raise ValueError('offline')
    monkeypatch.setattr('home_robot_tasks.pick_service.translate_target',translate)
    try:
        station.submit('실리콘 붓을 집어줘')
        wait_for(lambda: station.snapshot()['history'] and station.snapshot()['accepting'])
        assert backend.calls == ['prepare']
        assert station.snapshot()['last_result']['status'] == 'FAILED'
        monkeypatch.setattr('home_robot_tasks.pick_service.translate_target',lambda phrase: 'silicone brush')
        station.submit('실리콘 붓을 집어줘')
        wait_for(lambda: len(station.snapshot()['history']) == 2)
        assert station.snapshot()['current']['prompt'] == 'silicone brush'
    finally:
        station.close()


@pytest.mark.parametrize(('reply','expected'), [
    ('{"prompt":"silicone pastry brush"}', 'silicone pastry brush'),
    ('{"prompt":null}', None), ('{"prompt":"brush and cup"}', None),
])
def test_unknown_object_uses_local_translation_contract(monkeypatch, reply, expected):
    import io
    from home_perception.language import local_model
    from home_robot_tasks.pick_target import translate_target
    class Opener:
        def open(self, request, timeout):
            assert request.full_url == 'http://127.0.0.1:18767/v1/chat/completions'
            payload = json.loads(request.data)
            assert payload['response_format']['json_schema']['name'] == 'pick_target'
            assert payload['messages'][1]['content'] == '실리콘 제과용 붓'
            choice = {'finish_reason':'stop','message':{'content':reply}}
            return io.BytesIO(json.dumps({'choices':[choice]}).encode())
    monkeypatch.setattr(local_model, 'build_opener', lambda *_: Opener())
    if expected:
        assert translate_target('실리콘 제과용 붓') == expected
    else:
        with pytest.raises(ValueError):
            translate_target('실리콘 제과용 붓')


@pytest.mark.parametrize('finish', ['complete', 'cancel', 'shutdown', 'fault'])
def test_unlimited_stage_waits_past_sixty_and_three_hundred_seconds(tmp_path, monkeypatch, finish):
    from types import SimpleNamespace
    from home_robot_tasks import pick_service
    offset = [0.]
    monkeypatch.setattr(pick_service, 'time', SimpleNamespace(monotonic=lambda: time.monotonic()+offset[0]))

    class Waiting(RecordingBackend):
        future = Future()
        def capture(self, ctx, value):
            ctx.progress('근거리 RGB-D 촬영 중')
            ctx.wait(self.future)
            return value

    backend = Waiting()
    station = start(tmp_path, backend, stage_timeout_s=0)
    try:
        station.submit('리모컨 집기')
        wait_for(lambda: station.snapshot()['phase_label']=='근거리 RGB-D 촬영 중')
        for elapsed in (61., 361.):
            offset[0] = elapsed
            ticks = backend.ticks
            wait_for(lambda: backend.ticks >= ticks+3)
            assert station.snapshot()['phase'] == 'CAPTURING'
            assert not station.snapshot()['history']
        if finish == 'cancel':
            station.cancel()
        elif finish == 'shutdown':
            station.shutdown.set()
        elif finish == 'fault':
            backend.future.set_exception(ValueError('camera disconnected'))
        else:
            backend.future.set_result(None)
        wait_for(lambda: station.snapshot()['history'])
        expected = {'complete': 'SUCCEEDED', 'cancel': 'CANCELLED', 'shutdown': 'CANCELLED', 'fault': 'FAILED'}
        assert station.snapshot()['last_result']['status'] == expected[finish]
        assert ('execute' in backend.calls) is (finish=='complete')
        if not backend.future.done():
            backend.future.set_result(None)
            assert 'execute' not in backend.calls
    finally:
        station.close()


@pytest.mark.parametrize('mode,extra,expected', [
    ('station', [], 0), ('station', ['--stage-timeout-seconds', '60'], 60), ('demo', [], 300),
])
def test_normal_launcher_resolves_stage_deadline_before_server_start(tmp_path, monkeypatch, mode, extra, expected):
    from tools.run import pick_service as cli
    observed = []
    monkeypatch.setattr(cli, 'PickStation', lambda backend, output, stage_timeout_s: observed.append(stage_timeout_s))
    def finish_before_socket(*args):
        raise LookupError('captured server configuration')
    monkeypatch.setattr(cli, 'create_server', finish_before_socket)
    with pytest.raises(LookupError):
        cli.main(['--mode', mode, '--mobile-mesh-dir', str(tmp_path), '--output', str(tmp_path/'run'), *extra])
    assert observed == [expected]


def test_reset_runs_on_owner_preserves_history_and_rejects_duplicate(tmp_path):
    class Resettable(RecordingBackend):
        release=threading.Event()
        def reset(self,ctx):
            self.owners.add(threading.get_ident())
            while not self.release.is_set():time.sleep(.005)
            self.prepared=True
    b=Resettable();s=start(tmp_path,b)
    try:
        s.submit('지우개');wait_for(lambda:s.snapshot()['history'] and s.snapshot()['accepting'])
        old=list(s.snapshot()['history'])
        assert s.request_reset()
        with pytest.raises(Rejected):s.request_reset()
        with pytest.raises(Rejected):s.submit('지우개')
        b.release.set();wait_for(lambda:s.snapshot()['accepting'])
        assert s.snapshot()['current'] is None and s.snapshot()['history']==old
        assert len(b.owners)==1
    finally:b.release.set();s.close()


def test_reset_recovery_after_startup_failure_and_support_gate(tmp_path):
    class Resettable(RecordingBackend):
        hardware=object()
        def prepare(self,ctx):raise RuntimeError('startup failure')
        def reset(self,ctx):self.prepared=True
    s=PickStation(Resettable(),tmp_path);s.start()
    try:
        wait_for(lambda:s.snapshot()['phase']=='FAULT')
        with pytest.raises(Rejected):s.request_reset()
        s.request_reset(supported_empty=True)
        wait_for(lambda:s.snapshot()['accepting'])
    finally:s.close()


def test_failed_reset_keeps_stop_failure_and_current_record(tmp_path):
    class Resettable(RecordingBackend):
        def reset(self,ctx):raise RuntimeError('stop not confirmed')
    s=start(tmp_path,Resettable())
    try:
        s.stop_error='original stop fault'
        s.request_reset();wait_for(lambda:s.snapshot()['phase']=='FAULT')
        assert not s.snapshot()['accepting'] and s.stop_error=='original stop fault'
    finally:s.close()


def test_idle_tick_fault_retires_queued_request_and_allows_reset(tmp_path):
    entered=threading.Event();release=threading.Event()
    class Resettable(RecordingBackend):
        fail_next=False
        def tick(self):
            super().tick()
            if self.fail_next:
                entered.set();release.wait(2)
                self.fail_next=False
                raise RuntimeError('feedback disappeared before dispatch')
        def reset(self,ctx):self.prepared=True
    b=Resettable();s=start(tmp_path,b)
    try:
        b.fail_next=True;wait_for(entered.is_set)
        s.submit('white eraser')
        release.set()
        wait_for(lambda:s.snapshot()['phase']=='FAULT')
        assert s.pending is None and not s.active and s.snapshot()['reset_available']
        assert b.aborts==1 and 'capture' not in b.calls
        s.request_reset();wait_for(lambda:s.snapshot()['accepting'])
        assert 'capture' not in b.calls  # Failed queued work never revives.
    finally:release.set();s.close()


def test_result_write_failure_stops_owner_and_does_not_trap_reset(tmp_path,monkeypatch):
    class Resettable(RecordingBackend):
        def reset(self,ctx):self.prepared=True
    b=Resettable();s=start(tmp_path,b)
    try:
        def disk_error(*args):raise OSError('disk full')
        monkeypatch.setattr(s,'_finish',disk_error)
        s.submit('white eraser')
        wait_for(lambda:s.snapshot()['phase']=='FAULT')
        assert b.aborts==1 and not s.active and s.snapshot()['reset_available']
    finally:s.close()
