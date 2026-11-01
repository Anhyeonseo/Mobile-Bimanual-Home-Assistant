"""Legacy identification must never turn a diagnostic into a control session."""
import importlib.util
import json
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location('stm32_link_check',
    Path(__file__).resolve().parents[1] / 'tools/diagnostics/stm32_link_check.py')
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)
INFO = dict(firmware='mobile-base-lift-teleop-v3.1', host_baud=921600,
            servo_baud=1000000, ids=[7, 8, 9, 10], max_raw=200,
            watchdog_ms=400, lift_window_ticks=640)


class Port:
    def __init__(self, response=None, written=5):
        self.response = bytearray(response if response is not None else
                                  json.dumps(INFO).encode() + b'\r\n')
        self.writes = []
        self.written = written
        self.closed = False

    def reset_input_buffer(self): pass
    def __enter__(self): return self
    def __exit__(self, *_): self.closed = True
    def write(self, value): self.writes.append(value); return self.written
    def read(self, size):
        assert size == 1
        result = bytes(self.response[:size]); del self.response[:size]
        return result


def test_reopens_port_and_only_sends_info_without_claiming_motor_health():
    ports = []
    def factory():
        port = Port(); ports.append(port); return port
    result = module.check_link(factory, INFO['firmware'], 5)
    assert result['link_verified'] and len(ports) == 5
    assert all(p.closed and p.writes == [b'INFO\n'] for p in ports)
    assert result['motor_commands_sent'] == 0
    assert not result['motion_authorized'] and not result['motor_ids_verified']


@pytest.mark.parametrize('response', [b'not-json\n', b'{}\n', b'\xff\n',
    b'x' * 1025, json.dumps(INFO | {'ids': [8, 9, 10, 11]}).encode() + b'\n',
    json.dumps(INFO | {'host_baud': 115200}).encode() + b'\n'])
def test_bad_reply_stops_without_retry_or_fallback(response):
    port = Port(response)
    result = module.check_link(lambda: port, INFO['firmware'], 5)
    assert not result['link_verified'] and len(result['results']) == 1
    assert port.closed and port.writes == [b'INFO\n']


def test_partial_write_is_not_resent():
    port = Port(written=2)
    result = module.check_link(lambda: port, INFO['firmware'], 3)
    assert not result['link_verified'] and port.writes == [b'INFO\n']


def test_timeout_keeps_failure_even_if_the_next_open_could_succeed():
    ticks = iter(i * .1 for i in range(100))
    port = Port(b'')
    result = module.check_link(lambda: port, INFO['firmware'], 5, clock=lambda: next(ticks))
    assert not result['link_verified'] and len(result['results']) == 1
    assert '0 response bytes' in result['results'][0]['error']
    assert port.closed and port.writes == [b'INFO\n']


@pytest.mark.parametrize('firmware,attempts', [('unknown', 5), (INFO['firmware'], True),
    (INFO['firmware'], 0), (INFO['firmware'], 11)])
def test_invalid_request_never_opens_port(firmware, attempts):
    def forbidden(): pytest.fail('invalid request opened hardware')
    with pytest.raises(ValueError): module.check_link(forbidden, firmware, attempts)


def test_busy_port_is_reported_without_a_second_open():
    calls = []
    def busy(): calls.append(1); raise OSError('port busy')
    result = module.check_link(busy, INFO['firmware'], 5)
    assert calls == [1] and not result['link_verified']
