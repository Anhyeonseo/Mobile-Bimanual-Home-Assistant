import json
from pathlib import Path

import pytest

from tools.lib.presentation_session import FilmSession, evidence_summary, interpret, render


@pytest.mark.parametrize('phrase',[
    '책상에 있는 리모컨 침대로 가져다줘',
    '책상에 있는 리모컨을 침대로 가져다줘',
    '책상에 있는 흰색 리모컨 침대로 가져다줘',
])
def test_real_request_meaning_without_robot_execution(phrase):
    result=interpret(phrase,'test-film')
    intent=result['intent']
    assert not intent['issues']
    assert (intent['category'],intent['source_phrase'],intent['destination_phrase'])==('remote_control','책상','침대')
    assert intent['utterance']==phrase and result['robot_commands']==0


@pytest.mark.parametrize('phrase',[
    '책상에 있는 리모컨 말고 컵 침대로 가져다줘',
    '책상에 있는 리모컨 그리고 계산기 침대로 가져다줘',
    '책상에 있는 리모컨 가져다줘',
])
def test_unknown_or_ambiguous_request_is_not_completed(phrase):
    assert interpret(phrase,'ambiguous')['intent']['issues']


def test_no_timer_or_request_completion_claims_physical_success(tmp_path):
    session=FilmSession(tmp_path/'take1')
    session.request('책상에 있는 리모컨 침대로 가져다줘')
    assert session.state['stages']['desk']=='waiting'
    with pytest.raises(ValueError):session.finish()
    session.start()
    assert session.state['active']=='desk'
    with pytest.raises(ValueError):session.start()
    session.finish()
    assert session.state['stages']['pick']=='waiting'
    session.start();session.close()
    assert session.state['stages']['pick']=='interrupted'
    state=json.loads((session.output/'state.json').read_text())
    assert state['status']=='FILM_MARKERS_PARTIAL' and state['robot_commands']==0
    events=[json.loads(s) for s in (session.output/'events.jsonl').read_text().splitlines()]
    assert events==state['events']
    assert events[-3]['origin']=='operator'


def test_separate_pick_take_does_not_claim_navigation(tmp_path):
    session=FilmSession(tmp_path/'pick',clip='pick',take=3)
    session.request('책상에 있는 리모컨을 침대로 가져다줘')
    session.start();assert session.state['active']=='pick'
    session.finish();session.close()
    assert session.state['stages']['desk']=='not_filmed'
    assert session.state['status']=='FILM_MARKERS_COMPLETE'
    assert '운영자' in render(session.state)
    with pytest.raises(FileExistsError):FilmSession(tmp_path/'pick')


def test_failure_prevents_silent_next_stage(tmp_path):
    session=FilmSession(tmp_path/'fail')
    session.request('책상에 있는 리모컨 침대로 가져다줘')
    session.start();session.finish('촬영 중단')
    with pytest.raises(ValueError):session.start()


def evidence(tmp_path):
    import hashlib
    snapshot=tmp_path/'snapshot';snapshot.mkdir()
    for name in ('color.png','depth_aligned.png','metadata.json'):
        (snapshot/name).write_bytes(name.encode())
    source={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()}
    report=tmp_path/'detection.json'
    report.write_text(json.dumps(dict(source_sha256=source,coordinate_frame='color_camera_optical_frame',
        status='OK',detections=[dict(label='remote control',score=.8,
            surface_point=dict(valid=True,xyz_m=[.1,.2,.3]))])))
    return report,snapshot


def test_analysis_provenance_and_no_motion_status_change(tmp_path):
    report,snapshot=evidence(tmp_path)
    session=FilmSession(tmp_path/'film',clip='pick')
    session.request('책상에 있는 리모컨 침대로 가져다줘')
    session.attach(report,snapshot)
    assert session.state['stages']['pick']=='waiting'
    assert '녹화 재분석' in render(session.state)
    assert session.state['evidence'][0]['physical_grasp_verified'] is False
    (snapshot/'depth_aligned.png').write_bytes(b'changed')
    with pytest.raises(ValueError,match='다릅니다'):evidence_summary(report,snapshot)
