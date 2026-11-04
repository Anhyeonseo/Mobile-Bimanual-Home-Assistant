from copy import deepcopy
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

import pytest

from home_robot_tasks.fetch import FetchRequest, InvalidTask, plan_fetch
from home_robot_tasks.offline_lab import load_profile, run_scenario, SCENARIOS
from home_robot_tasks.offline_stack import OfflineStack

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def profile():
    return load_profile(ROOT / 'config/studio.simulation.json')


@pytest.mark.parametrize('scenario', SCENARIOS)
def test_studio_runs_composed_production_fsm_and_records_expected_outcome(profile, scenario):
    events = []
    report = run_scenario(profile, scenario, events.append)
    assert report['passed'], report
    assert events == report['events'] and events
    assert [e['event_id'] for e in events] == list(range(1, len(events) + 1))
    assert all(a['at_s'] <= b['at_s'] for a, b in zip(events, events[1:]))
    assert events[-1]['status'] == report['expected_status']
    assert all(e['evidence_source'] == 'synthetic' for e in events)


def test_source_place_is_preserved_in_navigation_and_search(profile):
    request = FetchRequest.from_dict(profile['request']['request'])
    plan = plan_fetch(request, profile['world'])
    assert plan['request']['source_place'] == 'desk'
    assert plan['steps'][1]['parameters']['source_place'] == 'desk'
    assert plan['steps'][2]['parameters']['locations'] == ['desk']
    # Make the room's default different, then verify actual port starts.
    world = deepcopy(profile['world'])
    world['rooms']['living_room']['search_locations'] = ['coffee_table', 'desk', 'sofa']
    stack = OfflineStack(profile['stack'], world, profile['map'])
    assert stack.run(profile['request'])['status'] == 'SUCCEEDED'
    assert stack.ports['navigation'].starts[0][1]['goal']['x'] == 1.5
    assert stack.ports['navigation'].starts[-1][1]['goal']['x'] == 3.
    assert stack.assembly.catalog.source_place == 'desk'


@pytest.mark.parametrize('source', ['bedroom', 'not_registered', None, True, ''])
def test_invalid_source_place_never_expands_into_room_search(profile, source):
    request = {**profile['request']['request'], 'source_place': source}
    with pytest.raises(InvalidTask):
        plan_fetch(FetchRequest.from_dict(request), profile['world'])


def test_legacy_room_request_still_searches_all_registered_places(profile):
    request = deepcopy(profile['request']['request'])
    request.pop('source_place')
    plan = plan_fetch(FetchRequest.from_dict(request), profile['world'])
    assert 'source_place' not in plan['request']
    assert plan['steps'][2]['parameters']['locations'] == ['desk', 'coffee_table', 'sofa']


def test_profile_rejects_stop_point_in_wall(tmp_path):
    data = json.loads((ROOT / 'config/studio.simulation.json').read_text())
    data['places']['desk']['point']['x'] = 0
    (tmp_path / 'fetch_stack.simulation.json').write_bytes((ROOT / 'config/fetch_stack.simulation.json').read_bytes())
    path = tmp_path / 'profile.json'
    path.write_text(json.dumps(data))
    with pytest.raises(InvalidTask, match='point_blocked'):
        load_profile(path)


def test_lab_cli_writes_trace_and_never_overwrites_a_run(tmp_path, monkeypatch):
    from tools.run import dev_lab
    monkeypatch.setattr(dev_lab, 'provenance', lambda: {})
    output = tmp_path / 'run'
    args = ['run', '--output', str(output)]
    assert dev_lab.main(args) == 0
    report = json.loads((output / 'report.json').read_text())
    assert report['passed'] and report['scenarios'][0]['status'] == 'SUCCEEDED'
    lines = (output / 'success/events.jsonl').read_text().splitlines()
    assert json.loads(lines[-1])['status'] == 'SUCCEEDED'
    original = (output / 'report.json').read_bytes()
    with pytest.raises(SystemExit):
        dev_lab.main(args)
    assert (output / 'report.json').read_bytes() == original


def test_studio_api_launcher_accepts_profile_request_and_persists_cancel(tmp_path):
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    state = tmp_path / 'api'
    process = subprocess.Popen([sys.executable, str(ROOT / 'tools/run/dev_lab.py'),
                                'serve', '--state', str(state), '--port', str(port)],
                               cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    def call(path, body=None, permit=None):
        headers = {'Authorization': 'Bearer ' + (state / 'token').read_text().strip()}
        if body is not None:
            headers['Content-Type'] = 'application/json'
        if permit is not None:
            headers['X-Request-Permit'] = permit
        request = urllib.request.Request(f'http://127.0.0.1:{port}' + path,
                                         data=None if body is None else json.dumps(body).encode(), headers=headers)
        with urllib.request.urlopen(request, timeout=1) as response:
            return response.status, json.load(response)

    try:
        deadline = time.monotonic() + 5
        while True:
            try:
                assert call('/v1/health')[1]['hardware_enabled'] is False
                break
            except (OSError, FileNotFoundError):
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise AssertionError('API startup failed')
                time.sleep(.02)
        assert (state / 'token').stat().st_mode & 0o777 == 0o600
        request = json.loads((state / 'request.json').read_text())
        nonce = call('/v1/permit')[1]['permit']
        code, result = call('/v1/requests', request, nonce)
        assert code == 202 and result['plan']['request']['source_place'] == 'desk'
        assert call('/v1/requests', request)[0] == 200
        call('/v1/requests/studio-fetch/cancel', {})
        deadline = time.monotonic() + 3
        while True:
            result = call('/v1/requests/studio-fetch')[1]
            if result['status'] == 'CANCELLED':
                break
            assert time.monotonic() < deadline, result
            time.sleep(.02)
        assert not result['control_owned'] and result['hardware_commands'] == 0
    finally:
        process.terminate()
        process.communicate(timeout=5)
