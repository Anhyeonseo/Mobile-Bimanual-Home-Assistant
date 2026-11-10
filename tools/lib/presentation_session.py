"""Filming timeline, not a robot executor. Only the operator advances shots."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import time
import uuid

from home_robot_tasks.assistant.language import parse_request

STAGES = ('request', 'desk', 'pick', 'carry', 'place')
LABELS = dict(zip(STAGES, ('명령 입력', '책상으로 이동', '리모컨 집기', '침대로 이동', '침대에 놓기')))


def write_json(path, value):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def interpret(raw, request_id):
    """Use the existing bounded parser; expand omitted 을/를 only at a clear boundary."""
    intent = parse_request(raw, request_id)
    normalized = raw
    if intent['issues']:
        match = re.fullmatch(r'\s*(.+?에\s*있는)\s+(.+?)(으로|로|에)\s*'
                             r'(가져다\s*줘|가져다\s*주세요|옮겨\s*줘)[.!?]?\s*', raw)
        possible = []
        if match:
            for boundary in re.finditer(r'\s+', match[2]):
                obj, dest = match[2][:boundary.start()], match[2][boundary.end():]
                if obj.endswith(('을', '를')):
                    continue
                proposed = f'{match[1]} {obj}을 {dest}{match[3]} {match[4]}'
                retry = parse_request(proposed, request_id)
                if not retry['issues']:
                    possible.append((retry, proposed))
        if len(possible) == 1:
            intent, normalized = possible[0]
    intent['utterance'] = raw
    return dict(intent=intent, normalized_for_parser=normalized,
                method='local_bounded_parser', robot_commands=0)


def evidence_summary(report_path, snapshot):
    """Require the exact RGB-D files; attaching an analysis never completes a pick."""
    report_path, snapshot = Path(report_path).resolve(), Path(snapshot).resolve()
    data = json.loads(report_path.read_text())
    expected = data.get('source_sha256', {})
    if set(expected) != {'color.png', 'depth_aligned.png', 'metadata.json'}:
        raise ValueError('동일 촬영의 RGB·깊이·카메라 정보 해시가 필요합니다.')
    for name, digest in expected.items():
        if hashlib.sha256((snapshot / name).read_bytes()).hexdigest() != digest:
            raise ValueError('촬영 파일이 분석 결과와 다릅니다: ' + name)
    if data.get('coordinate_frame') != 'color_camera_optical_frame':
        raise ValueError('이 화면에는 카메라 좌표계 인식 결과를 연결하세요.')
    result = dict(report=str(report_path), snapshot=str(snapshot), source_sha256=expected,
                  report_sha256=hashlib.sha256(report_path.read_bytes()).hexdigest(),
                  origin='recorded_rgbd_analysis', status=data.get('status'),
                  coordinate_frame='color_camera_optical_frame', physical_grasp_verified=False)
    if 'detections' in data:
        result['detections'] = []
        for d in data['detections']:
            segment = d.get('segmentation')
            if segment:
                mask = (report_path.parent / segment['mask_file']).resolve()
                if mask.parent != report_path.parent or hashlib.sha256(mask.read_bytes()).hexdigest() != segment['sha256']:
                    raise ValueError('마스크 파일 불일치')
            point = d.get('surface_point', {})
            result['detections'].append(dict(label=d['label'], confidence=d['score'],
                mask=bool(segment), xyz_m=point.get('xyz_m') if point.get('valid') else None))
    elif data.get('mode') == 'offline_grasp_replay' and 'group' in data:
        group = data['group']
        if group.get('source_sha256') != expected:
            raise ValueError('파지 후보의 촬영 해시 불일치')
        candidates = group['refined_grasps']
        result.update(candidate_count=len(candidates),
            candidate_widths_mm=[round(c['width_m'] * 1000, 1) for c in candidates],
            candidate_semantics='proposals_only_not_a_motion_path')
    else:
        raise ValueError('DINO/SAM 또는 GraspNet 분석 결과 파일이 필요합니다.')
    return result


class FilmSession:
    def __init__(self, output, clip='full', take=1):
        if clip not in (*STAGES, 'full') or take < 1:
            raise ValueError('잘못된 장면 또는 테이크 번호')
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.started = time.monotonic()
        self.state = dict(schema_version=1, session_id=uuid.uuid4().hex, take=take, clip=clip,
            mode='operator_staged_demonstration', created_at_utc=datetime.now(timezone.utc).isoformat(),
            robot_commands=0, status='WAITING_REQUEST', request=None, active=None,
            stages={s: 'waiting' if clip in ('full', s) else 'not_filmed' for s in STAGES},
            evidence=[], events=[])
        self.event('session_created', '촬영 대기', 'session')

    def event(self, kind, message, origin, **fields):
        record = dict(sequence=len(self.state['events']) + 1, type=kind, message=message,
            origin=origin, elapsed_s=round(time.monotonic() - self.started, 3),
            at_utc=datetime.now(timezone.utc).isoformat(), **fields)
        self.state['events'].append(record)
        with (self.output / 'events.jsonl').open('a') as stream:
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
        write_json(self.output / 'state.json', self.state)
        return record

    def request(self, raw):
        if self.state['status'] not in ('WAITING_REQUEST', 'NEEDS_CLARIFICATION'):
            raise ValueError('새 요청은 새 테이크에서 입력하세요.')
        parsed = interpret(raw, 'film-' + self.state['session_id'])
        self.state['request'] = parsed
        intent = parsed['intent']
        if intent['issues'] or intent['operation'] != 'fetch_object':
            self.state['status'] = 'NEEDS_CLARIFICATION'
            return self.event('request_rejected', '요청 확인 필요: ' + ', '.join(intent['issues'] or ['배달 요청 필요']), 'parser')
        self.state['status'] = 'READY'
        if self.state['clip'] in ('request', 'full'):
            self.state['stages']['request'] = 'done'
        return self.event('request_parsed', f"대상 {intent['category']} · 출발 {intent['source_phrase'] or '미지정'} · 도착 {intent['destination_phrase']}", 'parser')

    def start(self):
        if self.state['status'] != 'READY' or self.state['active']:
            raise ValueError('요청 확인 또는 현재 장면 종료가 필요합니다.')
        pending = [s for s in STAGES[1:] if self.state['stages'][s] == 'waiting']
        if not pending:
            raise ValueError('남은 장면이 없습니다. quit으로 저장 후 종료하세요.')
        stage = pending[0]
        self.state['active'] = stage
        self.state['stages'][stage] = 'active'
        return self.event('stage_started', LABELS[stage] + ' · 촬영 시작', 'operator', stage=stage)

    def finish(self, failure=None):
        stage = self.state['active']
        if stage is None:
            raise ValueError('먼저 start로 장면을 시작하세요.')
        self.state['active'] = None
        self.state['stages'][stage] = 'failed' if failure else 'done'
        if failure:
            self.state['status'] = 'FAILED'
        return self.event('stage_failed' if failure else 'stage_confirmed',
            LABELS[stage] + (' · 실패: ' + failure if failure else ' · 운영자 완료 확인'), 'operator', stage=stage)

    def attach(self, report, snapshot):
        result = evidence_summary(report, snapshot)
        self.state['evidence'].append(result)
        return self.event('analysis_attached', '실제 RGB-D 분석 결과 연결 · 녹화 재분석', 'analysis', evidence=result)

    def close(self):
        if self.state['active']:
            self.state['stages'][self.state['active']] = 'interrupted'
            self.state['active'] = None
        finished = all(v in ('done', 'not_filmed') for v in self.state['stages'].values())
        self.state['status'] = 'FILM_MARKERS_COMPLETE' if finished else 'FILM_MARKERS_PARTIAL'
        return self.event('session_closed', '촬영 기록 저장 · 로봇 제어 상태와 별개', 'session')


def render(state):
    rows = ['ALOHA MINI 1  |  HOME ASSISTANT',
            f"테이크 {state['take']:02d} · {state['clip']} · {state['status']}",
            '촬영 진행 · 이동: 텔레옵 / 동작 완료: 운영자 확인', '']
    if state['request']:
        q = state['request']['intent']
        rows += ['요청  ' + q['utterance'], f"대상  {q['category'] or '확인 필요'}   |   {q['source_phrase']} → {q['destination_phrase']}", '']
    symbols = dict(waiting='대기', active='진행', done='완료', not_filmed='별도 촬영', failed='실패', interrupted='중단')
    rows += [f"{i}. {LABELS[s]:12}  {symbols[state['stages'][s]]}" for i, s in enumerate(STAGES, 1)]
    for evidence in state['evidence'][-2:]:
        rows += ['', 'RGB-D 녹화 재분석 · 카메라 기준 XYZ (m)']
        for d in evidence.get('detections', []):
            xyz = '깊이 없음' if d['xyz_m'] is None else ' / '.join(f'{x:+.3f}' for x in d['xyz_m'])
            rows += [f"{d['label']} · 신뢰도 {d['confidence']:.2f} · XYZ {xyz}"]
        if 'candidate_count' in evidence:
            rows += [f"파지 후보 {evidence['candidate_count']}개 · 경로/실물 파지 성공과 별도"]
    rows += ['', '최근 처리 기록']
    rows += [f"{e['elapsed_s']:7.1f}s  {e['message']}" for e in state['events'][-6:]]
    return '\n'.join(rows)
