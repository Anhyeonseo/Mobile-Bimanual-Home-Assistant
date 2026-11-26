import io
from types import SimpleNamespace as NS
from urllib.error import HTTPError, URLError

import pytest

from tools.run import graspnet_pick as cli
from home_robot_tasks.pick_service import PickStation


def ready(**changes):
    return dict(mode='station', candidate_source='graspnet', accepting=True, **changes)


def test_status_exposes_actual_backend_for_cli_model_check(tmp_path):
    backend = NS(mode='station', kit_id='kit', options={'candidate_source': 'graspnet'})
    station = PickStation(backend, tmp_path)
    try:
        cli.require_ready(dict(station.snapshot(), accepting=True))
    finally:
        station.language.shutdown()


@pytest.mark.parametrize('state', [dict(mode='demo', accepting=True),
    dict(mode='station', candidate_source='so101', accepting=True),
    dict(mode='station', accepting=True),
    dict(mode='station', candidate_source='graspnet', accepting=False, blocked_reason='STOP未確認')])
def test_wrong_model_or_blocked_station_never_receives_pick(monkeypatch, state):
    monkeypatch.setattr(cli, 'Client', lambda _: NS(status=lambda: state,
                        post=lambda *_: pytest.fail('unexpected physical request')))
    assert cli.main(['pick', '리모컨', '--supported-empty-top']) == 2


def test_physical_confirmation_is_required_and_inspect_never_asserts_it():
    with pytest.raises(SystemExit):
        cli.parser().parse_args(['pick', '리모컨'])
    with pytest.raises(SystemExit):
        cli.parser().parse_args(['pick', '리모컨', '--supported-empty-top', '--supported-empty-bottom'])
    assert cli.payload_for(cli.parser().parse_args(['inspect', '리모컨'])) == dict(
        text='리모컨', operation='top-inspect')
    assert cli.payload_for(cli.parser().parse_args(['pick', '리모컨', '--supported-empty-top'])) == dict(
        text='리모컨', operation='top-pick', supported_empty_top=True, supported_empty_bottom=False,
        supported_startup_release=True)
    args = cli.parser().parse_args(['pick', '지우개', '--align', '--supported-empty-top'])
    assert cli.payload_for(args) == dict(text='지우개', operation='top-align-pick', supported_empty_top=True,
                                         supported_empty_bottom=False, supported_startup_release=True)
    with pytest.raises(ValueError):
        cli.payload_for(cli.parser().parse_args(['pick', '지우개', '--align', '--supported-empty-bottom']))


def test_lost_submit_reply_is_not_retried_and_requests_cancel(monkeypatch):
    calls = []
    def post(path, payload):
        calls.append(path)
        if path == '/api/requests':
            raise URLError('reply lost after possible acceptance')
        return {'requested': True}
    monkeypatch.setattr(cli, 'Client', lambda _: NS(status=ready, post=post))
    assert cli.main(['pick', '리모컨', '--supported-empty-top']) == 2
    assert calls == ['/api/requests', '/api/cancel']


def test_explicit_rejection_does_not_cancel_someone_elses_request(monkeypatch):
    calls = []
    def post(path, payload):
        calls.append(path)
        raise HTTPError('http://127.0.0.1', 409, 'busy', {}, io.BytesIO(b'busy'))
    monkeypatch.setattr(cli, 'Client', lambda _: NS(status=ready, post=post))
    assert cli.main(['pick', '리모컨', '--supported-empty-top']) == 2
    assert calls == ['/api/requests']


@pytest.mark.parametrize('result, code', [
    ({'status': 'SUCCEEDED', 'physical_success': True}, 0),
    ({'status': 'SUCCEEDED', 'physical_success': False}, 2),
    ({'status': 'UNVERIFIED', 'physical_success': False}, 2),
    ({'status': 'STOP_UNCONFIRMED', 'physical_success': False}, 2),
    ({'status': 'OBSERVED_ONLY', 'physical_success': False}, 0)])
def test_wait_uses_this_request_result_and_never_equates_unverified_with_success(monkeypatch, result, code):
    states = iter([
        {'phase': 'PLANNING', 'history': [{'id': 'old', 'result': {'status': 'SUCCEEDED', 'physical_success': True}}]},
        {'phase': 'BLOCKED', 'history': [{'id': 'current', 'result': result}]}])
    monkeypatch.setattr(cli.time, 'sleep', lambda _: None)
    assert cli.wait_result(NS(status=lambda: next(states)), 'current') == code


def test_wait_accepts_terminal_history_from_web_station():
    state = {'phase': 'BLOCKED', 'history': [{'id':'current', 'status':'UNVERIFIED', 'physical_success':False}]}
    assert cli.wait_result(NS(status=lambda:state), 'current') == 2


def test_start_is_model_only_and_quotes_remote_root():
    args = cli.parser().parse_args(['start', '--remote-root', '/home/hyper/robot/releases/space $name'])
    command = cli.start_argv(args)
    assert command[0] == 'ssh' and '-tt' in command
    shell = cli.shlex.split(command[-1])[2]
    assert '--candidate-source graspnet' in shell and '--mode station' in shell
    assert '--execute' not in shell and '--supported-empty' not in shell
    assert cli.shlex.split(shell.splitlines()[1]) == ['cd', args.remote_root]
