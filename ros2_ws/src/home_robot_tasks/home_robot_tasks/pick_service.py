"""One persistent, single-owner pick loop shared by CLI and local web clients.

The backend owns devices. HTTP threads may only submit/cancel and read copies.
Slow backends use Context.wait(), keeping the device service loop alive. A late
worker result can never dispatch motion after cancellation. No automatic retry,
rearm, release, or inference of physical success from trajectory completion.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor

from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import time
import uuid

from .pick_target import parse_pick_request, translate_target

STAGES = (
    ('CAPTURING', '새 촬영', 'capture'),
    ('DETECTING', '물체 탐지', 'detect'),
    ('GENERATING_GRASPS', '파지 후보 생성', 'candidates'),
    ('PLANNING', '경로 계획·검사', 'plan'),
    ('EXECUTING', '파지 실행', 'execute'),
    ('VERIFYING', '결과 확인', 'verify'),
)


class Cancelled(Exception):
    pass


class Rejected(ValueError):
    """A measured/input rejection; never permission to skip a stage."""


class PlanningRejected(Rejected):
    """Saved planning evidence, with a review-only recovery decision."""
    def __init__(self, message, recovery):
        super().__init__(message)
        self.recovery = deepcopy(recovery)


class Context:
    def __init__(self, station, folder=None):
        self.station, self.folder = station, folder
        self.deadline = None

    def check(self):
        if self.station.shutdown.is_set() or self.station.cancelled.is_set():
            raise Cancelled('중단 요청')
        if self.deadline is not None and time.monotonic() > self.deadline:
            raise TimeoutError('현재 단계의 시간 제한을 초과했습니다.')

    def service(self):
        self.check()
        self.station.backend.tick()
        self.check()

    def wait(self, pending):
        """pending implements done/result; inference has no hardware writer."""
        while not pending.done():
            self.service()
            self.station.shutdown.wait(.01)
        self.service()
        return pending.result()

    def pause(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.service()
            self.station.shutdown.wait(min(.02, max(0, end-time.monotonic())))

    def note(self, message, **values):
        self.station.note(message, **values)

    def progress(self, message):
        self.check()
        with self.station.lock:
            self.station.data['phase_label'] = message
        self.note(message)

    def artifact(self, path, label):
        self.station.artifact(path, label)


class PickStation:
    def __init__(self, backend, output, *, stage_timeout_s=300):
        if not math.isfinite(stage_timeout_s) or stage_timeout_s < 0:
            raise ValueError('nonnegative finite stage timeout required; 0 waits until completion')
        self.backend = backend
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.timeout = stage_timeout_s
        self.lock = threading.RLock()
        self.shutdown, self.cancelled, self.wakeup = (threading.Event() for _ in range(3))
        self.language = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pick-language")
        self.pending = None
        self.active = False
        self.reset_pending = False
        self.reset_supported_empty = False
        self.stop_error = None
        self.events, self.history = deque(maxlen=100), deque(maxlen=20)
        self.assets = {}
        self.data = dict(mode=backend.mode, phase='STARTING', phase_label='프로그램·모델 준비',
                         accepting=False, blocked_reason='', current=None, last_result=None,
                         startup_seconds=None, connected=False, stage_timeout_seconds=stage_timeout_s)
        if backend.mode == 'station':
            self.data['kit_id'] = backend.kit_id
            self.data['candidate_source'] = backend.options.get('candidate_source', 'so101')
        self.data['controls_enabled'] = True
        self.data['mode_detail'] = getattr(backend, 'mode_detail', '')
        self.thread = threading.Thread(target=self._loop, name='pick-device-owner', daemon=True)

    def start(self):
        self.thread.start()

    def note(self, message, **values):
        with self.lock:
            self.events.append(dict(at=datetime.now(timezone.utc).isoformat(),
                                    message=str(message), **values))

    def artifact(self, path, label):
        path = Path(path).resolve()
        if not path.is_relative_to(self.output) or not path.is_file():
            raise ValueError('artifact must be a file inside this station output')
        if path.suffix.lower() not in ('.png', '.jpg', '.jpeg', '.json', '.jsonl'):
            raise ValueError('unsupported monitor artifact')
        with self.lock:
            key = uuid.uuid4().hex
            self.assets[key] = path
            current = self.data['current']
            if current is not None:
                current['artifacts'].append(dict(id=key, label=label, image=path.suffix.lower() in ('.png', '.jpg', '.jpeg')))

    def snapshot(self):
        with self.lock:
            result = deepcopy(self.data)
            result['events'] = deepcopy(list(self.events))
            result['history'] = deepcopy(list(self.history))
            result['reset_available'] = (callable(getattr(self.backend,'reset',None))
                and self.thread.is_alive() and not self.active and self.pending is None
                and not self.reset_pending and self.data['phase'] not in ('STARTING','RESETTING'))
            result['next_test_ready'] = (result['reset_available'] and not self.stop_error
                and bool(getattr(self.backend,'can_prepare_next_test',lambda:False)()))
            result['reset_requires_support'] = getattr(self.backend,'hardware',None) is not None
            result['resume_source'] = deepcopy(getattr(self.backend,'resume_info',None))
            result['server_time_s'] = time.monotonic()
            return result

    def submit(self, text, *, operation=None, supported_empty_bottom=False, supported_empty_top=False,
               supported_startup_release=False, scene_unchanged=False):
        try:
            target, prompt, location = parse_pick_request(text)
        except ValueError as exc:
            raise Rejected(str(exc)) from exc
        with self.lock:
            if not self.data['accepting'] or self.active or self.pending is not None:
                raise Rejected(self.data['blocked_reason'] or '준비 또는 작업 진행 중입니다.')
            if type(supported_startup_release) is not bool or (supported_startup_release and not (supported_empty_bottom or supported_empty_top)):
                raise Rejected('준비 중 모터 힘 해제에는 빈 양팔·리프트 받침 확인이 필요합니다.')
            if operation=='resume-top-pick' and scene_unchanged is not True:
                raise Rejected('베이스·물체가 이전 관측 그대로인지 확인해 주세요.')
            if operation=='resume-top-pick' and location is not None:
                raise Rejected('이전 관측 이어가기는 대상을 바꿀 수 없습니다. 위치를 지정하려면 새 촬영 작업을 선택해 주세요.')
            if self.backend.mode == 'station':
                operation = self.backend.check_operation(operation, supported_empty_bottom, supported_empty_top)
            elif operation is not None or supported_empty_bottom or supported_empty_top or supported_startup_release:
                raise Rejected('현재 실행 모드에서 지원하지 않는 요청입니다.')
            identifier = uuid.uuid4().hex
            self.pending = dict(id=identifier, text=text.strip(), target=target, prompt=prompt,
                                submitted_at=datetime.now(timezone.utc).isoformat())
            if location is not None:
                self.pending['location'] = location
            if operation:
                self.pending.update(operation=operation, supported_empty_bottom=supported_empty_bottom,
                                    supported_empty_top=supported_empty_top, supported_startup_release=supported_startup_release)
                if operation=='resume-top-pick':self.pending['scene_unchanged']=scene_unchanged
            self.data['accepting'] = False
            self.cancelled.clear()
            self.wakeup.set()
            return identifier

    def request_reset(self, *, supported_empty=False):
        with self.lock:
            if not self.snapshot()['reset_available']:
                raise Rejected('작업 중에는 초기화할 수 없습니다. 작업 중단 후 정지 결과를 기다려 주세요.')
            if type(supported_empty) is not bool:
                raise Rejected('받침 확인 값이 잘못됐습니다.')
            if getattr(self.backend,'hardware',None) is not None and not supported_empty:
                raise Rejected('물체를 제거하고 양팔·리프트를 받쳐 고정했는지 확인해 주세요.')
            self.reset_supported_empty=supported_empty
            self.reset_pending=True
            self.data.update(phase='RESETTING',phase_label='픽 스테이션 초기화 중',accepting=False)
            self.wakeup.set()
            return True

    def _reset(self):
        self.cancelled.clear()
        self.note('픽 스테이션 초기화 시작')
        try:
            ctx=Context(self)
            ctx.supported_empty=self.reset_supported_empty
            self.backend.reset(ctx)
            with self.lock:
                self.stop_error=None
                self.data.update(current=None,last_result=None,connected=True)
                self.assets.clear()
            self.note('초기화 완료 · 이전 요청 기록은 보존했습니다.')
        except Exception as exc:
            self.note('초기화 실패: '+str(exc))
            with self.lock:
                self.data.update(phase='FAULT',phase_label='초기화 확인 필요',accepting=False,blocked_reason=str(exc))
        finally:
            with self.lock:
                self.reset_pending=False
                self.reset_supported_empty=False
        if self.data['phase']!='FAULT':self._set_ready()

    def cancel(self):
        with self.lock:
            # Also allow the web stop button to stop an idle physical hold.
            if self.active or self.pending is not None or self.backend.mode == 'live' or getattr(self.backend, 'hardware', None) is not None:
                self.cancelled.set()
                self.wakeup.set()
                self.note('중단 요청을 접수했습니다. 정지 확인 중입니다.')
                return True
            return False

    def _set_ready(self):
        ready, reason = self.backend.ready()
        if self.stop_error:
            ready, reason = False, '정지 확인 실패: '+self.stop_error
        if self.shutdown.is_set():
            ready, reason = False, '실행기를 종료하고 있습니다.'
        with self.lock:
            if self.pending is not None or self.active or self.reset_pending:
                return
            self.data.update(phase='READY' if ready else 'BLOCKED', phase_label='다음 요청 대기' if ready else '확인 필요',
                             accepting=ready, blocked_reason=reason)

    def _finish(self, result, folder):
        with self.lock:
            current = self.data['current']
            current['duration_s'] = time.monotonic()-current['started_s']
            current['result'] = result
            self.data['last_result'] = deepcopy(result)
            self.history.appendleft(dict(id=current['id'], text=current['text'],
                                         duration_s=current['duration_s'], **result))
            report = deepcopy(current)
        # Atomic publication; original camera/plan files are never overwritten.
        temp = folder/'result.tmp'
        temp.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        temp.replace(folder/'result.json')
        self.note(result.get('reason', result['status']), result=result['status'])

    def _run_request(self, request):
        folder = self.output/request['id']
        folder.mkdir(exist_ok=False)
        ctx = Context(self, folder)
        camera_only = self.backend.mode == 'station' and request.get('operation') == 'top-inspect'
        stages = STAGES[:3] if camera_only else STAGES
        if self.backend.mode == 'station' and not camera_only:
            extra = (() if request.get('operation')=='top-pick' else
                     (('PREPARING_PICK', '장치 준비·재관측', 'prepare_pick'),))
            if request.get('operation') == 'observe-rise-pick':
                extra += (('LIFTING', '리프트 상승', 'rise'),)
            stages = STAGES[:3] + extra + STAGES[3:]
            if request.get('operation')=='top-pick':
                stages=stages[:-1]+(('VERIFYING','동작 완료 확인','verify'),)
            if request.get('operation')=='resume-top-pick':
                stages=(('PREPARING_PICK','이전 관측·현재 자세 연결','prepare_resume'),)+STAGES[3:]
        with self.lock:
            self.active = True
            self.assets.clear()
            self.data['current'] = dict(**request, started_s=time.monotonic(), artifacts=[],
                stages=[dict(id=s, label=l, status='waiting', seconds=None) for s,l,_ in stages])
        value = request
        stage = None
        started = None
        try:
            ctx.check()
            ready, reason = self.backend.ready()
            if not ready:
                raise Rejected(reason)
            if request['prompt'] is None:
                with self.lock:
                    self.data.update(phase='INTERPRETING', phase_label='물체 이름 해석')
                ctx.deadline = time.monotonic() + self.timeout if self.timeout else None
                pending = self.language.submit(translate_target, request['target'])
                try:
                    request['prompt'] = ctx.wait(pending)
                except ValueError as exc:
                    raise Rejected(str(exc)) from exc
                finally:
                    pending.cancel()
                with self.lock:
                    self.data['current']['prompt'] = request['prompt']
            ctx.note('탐지 대상: '+request['prompt'])
            if request.get('location'):
                ctx.note('대상 선택: 촬영 화면의 '+('왼쪽' if request['location']=='left' else '오른쪽')+' · 로봇 팔 선택이 아닙니다.')
            for index, (phase, label, method) in enumerate(stages):
                ctx.check()
                started = time.monotonic()
                ctx.deadline = started + self.timeout if self.timeout else None
                with self.lock:
                    self.data.update(phase=phase, phase_label=label, accepting=False)
                    stage = self.data['current']['stages'][index]
                    stage.update(status='running', started_s=started)
                self.note(label+' 시작')
                value = getattr(self.backend, method)(ctx, value)
                ctx.check()
                with self.lock:
                    stage.update(status='done', seconds=time.monotonic()-started)
            allowed = ('OBSERVED_ONLY',) if camera_only else ('SUCCEEDED', 'FAILED', 'UNVERIFIED', 'PLANNED_ONLY')
            if not isinstance(value, dict) or value.get('status') not in allowed:
                raise ValueError('explicit verification outcome required')
            if (camera_only or self.backend.mode not in ('live', 'station')) and value.get('physical_success') is True:
                raise ValueError('nonphysical backend cannot claim physical success')
            if value['status']=='SUCCEEDED' and self.backend.mode in ('live', 'station') and value.get('physical_success') is not True:
                raise ValueError('physical success requires independent verification')
            result = value
        except Exception as exc:
            status = 'CANCELLED' if isinstance(exc, Cancelled) else 'FAILED'
            if stage is not None:
                with self.lock:
                    stage.update(status='cancelled' if status=='CANCELLED' else 'failed',
                                 seconds=time.monotonic()-started)
            with self.lock:
                for row in self.data['current']['stages']:
                    if row['status'] == 'waiting':
                        row['status'] = 'skipped'
            try:
                handler = getattr(self.backend, 'on_failure', None)
                handler(exc) if handler is not None else self.backend.abort()
                stop_error = None
            except Exception as stop:
                stop_error = str(stop)
            result = dict(status=status, reason=str(exc), physical_success=False, stop_error=stop_error)
            if isinstance(exc, PlanningRejected):
                result['recovery'] = deepcopy(exc.recovery)
                if exc.recovery.get('base_reposition_candidate') is True:
                    result['status'] = 'REPOSITION_REVIEW_REQUIRED'
            if stop_error:
                result.update(status='STOP_UNCONFIRMED')
                self.stop_error = stop_error
        finally:
            self.cancelled.clear()
        self._finish(result, folder)
        with self.lock:
            self.active = False
        self._set_ready()

    def _loop(self):
        started = time.monotonic()
        def failed(exc):
            self.note('실행기 중단: '+str(exc))
            # A tick or result-save failure can happen outside _run_request's
            # handler. Retire any queued request and stop the owner before
            # exposing reset; it must never run after recovery.
            try:
                self.backend.abort()
            except Exception as stop:
                self.stop_error=str(stop)
                self.note('정지 확인 실패: '+self.stop_error)
            with self.lock:
                self.pending=None
                self.active=False
                self.data.update(phase='FAULT',phase_label='실행기 오류',accepting=False,blocked_reason=str(exc))
        try:
            try:
                startup=Context(self)
                startup.deadline=started+self.timeout if self.timeout else None
                self.backend.prepare(startup);startup.check()
                with self.lock:self.data.update(startup_seconds=time.monotonic()-started,connected=True)
                self.note('프로그램 준비 완료',mode=self.backend.mode)
                self._set_ready()
            except Exception as exc:failed(exc)
            while not self.shutdown.is_set():
                try:
                    if self.reset_pending:
                        self._reset()
                    elif self.data['phase']!='FAULT':
                        if self.cancelled.is_set() and self.pending is None:
                            self.backend.abort();self.cancelled.clear()
                        self.backend.tick()
                        with self.lock:
                            request,self.pending=self.pending,None
                        if request is not None:self._run_request(request)
                        else:self._set_ready()
                except Exception as exc:failed(exc)
                self.wakeup.wait(.02);self.wakeup.clear()
        finally:
            self.language.shutdown(wait=False,cancel_futures=True)
            try:self.backend.close()
            except Exception as exc:
                self.note('종료 확인 실패: '+str(exc))
                with self.lock:self.data.update(phase='FAULT',phase_label='종료 확인 실패',blocked_reason=str(exc))
            with self.lock:self.data.update(accepting=False,connected=False)

    def close(self):
        self.shutdown.set()
        self.cancelled.set()
        self.wakeup.set()
        self.thread.join(20)
        if self.thread.is_alive():
            raise RuntimeError('pick owner did not finish shutdown')
