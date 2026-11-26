import http.client
import io
import json
import os
import re
from email.message import Message
from types import SimpleNamespace
from pathlib import Path
import shutil
import subprocess

import pytest

from home_robot_tasks.pick_demo import DemoBackend
from home_robot_tasks.pick_service import PickStation, Rejected
from home_robot_tasks.pick_watch import WatchedPick
from home_robot_tasks.pick_web import handler_for


def test_watch_reports_unknown_stages_and_never_equates_hold_with_grasp_success(tmp_path):
    path = tmp_path/'state.json'
    path.write_text(json.dumps(dict(status='HOLDING', capture_saved=True,
                                   stage='carry', events=[dict(state='GRASP_RUNNING'), dict(state='HOLDING')])))
    source = WatchedPick(tmp_path)
    status = source.snapshot()
    assert status['connected'] and not status['accepting']
    assert status['last_result']['status'] == 'UNVERIFIED'
    assert [row['status'] for row in status['current']['stages']] == ['done','unknown','unknown','unknown','done','unknown']
    assert not status['controls_enabled']
    with pytest.raises(Rejected):
        source.cancel()
    with pytest.raises(Rejected):
        source.submit('리모컨 집기')
    os.utime(path, (1, 1))
    stale = source.snapshot()
    assert not stale['connected']
    assert '갱신 끊김' in stale['phase_label']


def test_watch_missing_corrupt_or_deleted_file_never_shows_fresh_state(tmp_path):
    source = WatchedPick(tmp_path)
    assert source.snapshot()['current'] is None
    path = tmp_path/'state.json'
    path.write_text('{"status":"RUNNING"}')
    assert source.snapshot()['connected']
    path.write_text('{')
    assert not source.snapshot()['connected']
    assert source.snapshot()['raw_status'] == 'RUNNING'  # labelled last known
    path.unlink()
    assert not source.snapshot()['connected']


def test_watch_does_not_expose_symlinks_or_neighbor_runs(tmp_path):
    run = tmp_path/'run'
    run.mkdir()
    private = tmp_path/'private.json'
    private.write_text('{"status":"RUNNING"}')
    (run/'state.json').symlink_to(private)
    source = WatchedPick(run)
    assert source.snapshot()['current'] is None
    assert not source.assets


@pytest.fixture(params=[18768, 18770])
def monitor(tmp_path, request):
    # Run the real handler and response serialization over memory streams. The
    # managed development sandbox forbids even loopback socket creation.
    station = PickStation(DemoBackend(), tmp_path)
    Handler = handler_for(station)
    port = request.param
    def request(method, path, body=None, headers=None):
        handler = Handler.__new__(Handler)
        data = body.encode() if isinstance(body, str) else body or b''
        handler.headers = Message()
        for key, value in {'Host':f'127.0.0.1:{port}', 'Content-Length':str(len(data)), **(headers or {})}.items():
            handler.headers[key] = value
        handler.path, handler.command = path, method
        handler.request_version = 'HTTP/1.1'
        handler.requestline = f'{method} {path} HTTP/1.1'
        handler.server = SimpleNamespace(server_port=port)
        handler.connection = SimpleNamespace(settimeout=lambda _: None)
        handler.rfile, handler.wfile = io.BytesIO(data), io.BytesIO()
        getattr(handler, 'do_'+method)()
        response = http.client.HTTPResponse(SimpleNamespace(makefile=lambda *_: io.BytesIO(handler.wfile.getvalue())))
        response.begin()
        return response.status, response.read(), dict(response.getheaders())
    yield station, request, port


def test_monitor_serves_ui_state_and_only_registered_artifacts(monitor, tmp_path):
    station, request, port = monitor
    code, body, headers = request('GET', '/')
    assert code == 200 and b'__TOKEN__' not in body and 'Pick Station' in body.decode()
    assert headers['Cache-Control'] == 'no-store'
    assert 'frame-ancestors' in headers['Content-Security-Policy']
    assert request('GET', '/api/status')[0] == 200
    assert request('GET', '/api/artifacts/../../state.json')[0] == 404
    path = tmp_path/'report.json'
    path.write_text('{"actual":1}')
    station.artifact(path, 'report')
    key = next(iter(station.assets))
    assert request('GET', '/api/artifacts/'+key)[:2] == (200, path.read_bytes())
    assert request('GET', '/', headers={'Host':'attacker.example'})[0] == 403


def test_monitor_requires_token_same_origin_and_bounded_json(monitor):
    station, request, port = monitor
    token = re.search("const token='([^']+)'", request('GET', '/')[1].decode()).group(1)
    payload = json.dumps({'text':'리모컨 집기'})
    headers = {'Content-Type':'application/json', 'X-Pick-Token':token}
    assert request('POST', '/api/requests', payload, {'Content-Type':'application/json'})[0] == 403
    assert request('POST', '/api/requests', payload, {**headers, 'Origin':'http://attacker.example'})[0] == 403
    assert request('POST', '/api/requests', 'x'*2049, headers)[0] == 400
    assert request('POST', '/api/requests', '[]', headers)[0] == 400
    assert request('POST', '/api/requests', payload, headers)[0] == 409  # preparing
    with station.lock:
        station.data['accepting'] = True
    assert request('POST', '/api/requests', payload, headers)[0] == 202
    assert request('POST', '/api/requests', payload, headers)[0] == 409  # duplicate


def test_page_buttons_mode_labels_elapsed_time_and_disconnection():
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js not available for JS view unit test')
    root = Path(__file__).resolve().parents[1]
    subprocess.run([node, str(root/'tests/fixtures/pick_monitor_events.js'),
                    str(root/'ros2_ws/src/home_robot_tasks/home_robot_tasks/web/pick_monitor.html')],
                   check=True, timeout=10)


def test_reset_endpoint_requires_token_and_forwards_support(monitor):
    station,request,port=monitor
    token=re.search("const token='([^']+)'",request('GET','/')[1].decode()).group(1)
    seen=[]
    station.request_reset=lambda **kw:seen.append(kw) or True
    assert request('POST','/api/reset','{}',{'Content-Type':'application/json'})[0]==403
    headers={'Content-Type':'application/json','X-Pick-Token':token}
    assert request('POST','/api/reset','{"supported_empty":true}',headers)[0]==202
    assert seen==[{'supported_empty':True}]
