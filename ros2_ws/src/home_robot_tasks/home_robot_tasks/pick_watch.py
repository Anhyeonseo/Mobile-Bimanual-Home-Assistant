"""Read-only view of unified_pick's existing atomic state.json output.

This is deliberately not an executor. It never writes execute.json, opens a
controller, or treats a recent file timestamp as proof of robot connectivity.
Absent perception/planning events stay unknown, even if execution has started.
"""
from copy import deepcopy
import json
from pathlib import Path
import threading
import time

from .pick_service import Rejected, STAGES

LABELS = {
    'PREPARING': '컨트롤러 준비',
    'LIFT_RISING': '리프트 높이 조정',
    'WAITING_CHECKED_BUNDLE': '실행 요청 대기 · 검증 번들 필요',
    'EMPTY_RETURN_RUNNING': '빈 그리퍼 복귀 중',
    'EMPTY_RETURNED': '빈 그리퍼 복귀 완료 · 유지 중',
    'CHECKING_EXECUTION_REQUEST': '실행 요청 검사',
    'LOADING_CHECKED_BUNDLE': '검증 번들 준비',
    'CHECKING_CALIBRATION_BINDING': '캘리브레이션 일치 검사',
    'CREATING_SCENE_ADMISSION': '현재 장면 검사 준비',
    'WAITING_LIVE_SCENE_ADMISSION': '현재 장면 일치 확인',
    'GRASP_RUNNING': '파지 실행', 'RUNNING': '파지 실행',
    'RECHECKING_HANDOFF': '다음 동작 진입 조건 재확인',
    'AWAITING_ROUTE_REPLAN': '경로 재계획 필요',
    'AWAITING_DESCENT_CONFIRMATION': '하강 진입 조건 확인 대기',
    'HOLDING': '궤적 완료 · 잡은 자세 유지',
    'STOPPING': '정지 확인 중', 'CANCELLED': '작업 중단',
    'FAILED': '실행 실패', 'REJECTED_BEFORE_CONTROL': '제어 시작 전 거절',
}
EXECUTING = {'GRASP_RUNNING', 'RUNNING', 'RECHECKING_HANDOFF', 'AWAITING_ROUTE_REPLAN',
             'AWAITING_DESCENT_CONFIRMATION', 'EMPTY_RETURN_RUNNING'}


class WatchedPick:
    def __init__(self, folder, *, stale_seconds=5):
        self.output = Path(folder).resolve()
        self.lock = threading.RLock()
        self.assets = {}
        self.stale_seconds = stale_seconds
        self.last_report = None
        self.last_signature = None
        self.last_update = None

    def start(self):
        pass

    def close(self):
        pass

    def submit(self, _):
        raise Rejected('상태 관찰 모드에서는 로봇에 요청을 보내지 않습니다.')

    def cancel(self):
        raise Rejected('상태 관찰 모드에서는 정지 명령을 보내지 않습니다. 기존 실행기를 사용하세요.')

    def _artifacts(self):
        # A fixed allowlist; do not expose arbitrary paths or old sibling runs.
        paths = [('snapshot/color.png', '실행기가 저장한 촬영 이미지'),
                 ('snapshot/detections.jpg', '실행기가 저장한 탐지 이미지'),
                 ('state.json', '실행기 상태 JSON')]
        result = []
        for index, (relative, label) in enumerate(paths):
            path = (self.output/relative).resolve()
            if path.is_relative_to(self.output) and path.is_file():
                key = 'watch-'+str(index)
                self.assets[key] = path
                result.append(dict(id=key, label=label, image=path.suffix != '.json'))
        return result

    def snapshot(self):
        with self.lock:
            return self._snapshot()

    def _snapshot(self):
        error = None
        try:
            path = self.output/'state.json'
            if not path.resolve().is_relative_to(self.output):
                raise ValueError('state.json must remain inside the watched directory')
            # Read from one file handle: an atomic rename cannot mix versions.
            with path.open('rb') as stream:
                import os
                stat = os.fstat(stream.fileno())
                signature = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
                if signature != self.last_signature:
                    data = stream.read(2_000_001)
                    if len(data) > 2_000_000:
                        raise ValueError('state.json exceeds 2 MB')
                    value = json.loads(data)
                    if not isinstance(value, dict) or not isinstance(value.get('status'), str):
                        raise ValueError('status 문자열이 있는 실행기 상태 JSON이 필요합니다.')
                    if not isinstance(value.get('events', []), list):
                        raise ValueError('events must be an array')
                    self.last_report = value
                    self.last_signature = signature
                    self.last_update = stat.st_mtime
        except (OSError, ValueError) as exc:
            error = str(exc)

        report = self.last_report or {}
        age = max(0., time.time()-self.last_update) if self.last_update else None
        fresh = error is None and age is not None and age <= self.stale_seconds
        raw = report.get('status', 'WAITING_STATE')
        current = None
        result = None
        if self.last_report is not None:
            if raw == 'HOLDING':
                result = dict(status='UNVERIFIED', physical_success=False,
                              reason='궤적은 완료됐지만 물체 파지 성공을 확인한 기록은 없습니다.')
            elif raw in ('FAILED', 'REJECTED_BEFORE_CONTROL', 'CANCELLED'):
                result = dict(status='CANCELLED' if raw == 'CANCELLED' else 'FAILED',
                              physical_success=False, reason=str(report.get('error') or report.get('reason') or raw))
            rows = [dict(id=phase, label=label, status='unknown', seconds=None) for phase, label, _ in STAGES]
            if report.get('capture_saved'):
                rows[0]['status'] = 'done'
            if raw in EXECUTING:
                rows[4]['status'] = 'running'
            elif raw == 'HOLDING':
                rows[4]['status'] = 'done'
                rows[5]['status'] = 'unknown'
            current = dict(id='watched-run', text='기존 실행기 관찰', stages=rows, artifacts=self._artifacts(),
                           started_s=None, duration_s=None, result=result)
        labels = LABELS.get(raw, raw)
        phase_label = labels if fresh else ('갱신 끊김 · 마지막 상태: '+labels if self.last_report else '실행기 상태 파일 대기')
        reason = '관찰 전용입니다. 실행·중단은 기존 실행기에서 조작하세요.'
        if not fresh:
            reason = '현재 로봇 상태를 확인할 수 없습니다. '+reason
        if error:
            reason += ' 파일 읽기: '+error
        events = [dict(at=None, message=LABELS.get(e.get('state'), str(e.get('state'))))
                  for e in report.get('events', [])[-100:] if isinstance(e, dict)]
        return dict(mode='watch', phase=raw, phase_label=phase_label, raw_status=raw,
                    detail=str(report.get('stage') or ''), accepting=False, controls_enabled=False,
                    connected=fresh, source_age_s=age, startup_seconds=None, blocked_reason=reason,
                    current=deepcopy(current), last_result=result, history=[], events=events,
                    server_time_s=time.monotonic())
