import contextlib
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/run'))
sys.path.insert(0, str(ROOT/'tools/diagnostics'))
import teleop_lift as m
import lift_stationary_hold_check as h


class Port:
    def __init__(self, failure=None):
        self.failure = failure
        self.data = bytearray()
        self.writes = []
        self.nonce = 0

    def reset_input_buffer(self):
        pass

    def read(self, n):
        result = bytes(self.data[:n]); del self.data[:n]
        return result

    def write(self, data):
        self.writes.append(data)
        parts = data.decode().split()
        if parts[0] == 'INFO':
            row = m.INFO
        elif parts[0] == 'SNAP':
            eeprom = [0]*40; eeprom[3:7] = [9, 3, 10, 0]; eeprom[33] = 1
            row = dict(type='snapshot', id=10, eeprom=eeprom, goal=0, torque=0, lock=1)
        elif parts[0] == 'STATUS':
            row = dict(type='status', armed=False, fault=False, motor_write_count=0)
        elif parts[0] == 'ARM':
            self.nonce = int(parts[1], 16)
            row = dict(type='armed', armed=True, fault=False, session=self.nonce,
                       sequence=0, nonzero_speed_writes=0)
        elif parts[0] == 'D':
            assert parts[3] == 'Z'
            seq = int(parts[2], 16)
            if self.failure == 'io':
                raise OSError('Simulated communication failure')
            row = dict(type='ack', armed=True, fault=False, session=self.nonce, sequence=seq,
                       nonzero_speed_writes=0, relative_counts=0, sample_tick=seq*50,
                       sample_count=seq+1, speed_raw=0, goal_raw=0, torque_raw=1)
            if self.failure == 'fault':
                row.update(type='fault', armed=False, fault=True, reason='TORQUE_READ_FAILED',
                           failure_register=40, failure_hal_status=3)
        elif parts[0] == 'STOP':
            row = dict(type='stopped', armed=False, fault=self.failure == 'fault',
                       stop_confirmed=True, speed_raw=0, goal_raw=0, torque_raw=1)
        else:
            raise AssertionError(data)
        self.data.extend(json.dumps(row).encode()+b'\n')
        return len(data)


def test_command_guard_rejects_motion_mode_changes_and_corrupted_frames():
    raw = Port(); guard = h.ZeroOnlyPort(raw, m.frame)
    for cmd in [m.frame('D 1234ABCD 00000001 D'), m.frame('PREP 1234ABCD'),
                b'D 1234ABCD 00000001 Z 0000\n', b'ARM 1234ABCD 0000\n',
                b'STOP\nARM 1234ABCD 0000\n']:
        with pytest.raises(RuntimeError): guard.write(cmd)
    assert raw.writes == []


@pytest.mark.parametrize('failure', [None, 'fault', 'io'])
def test_zero_only_hold_and_cleanup(monkeypatch, tmp_path, failure):
    raw = Port(failure)
    now = [0.]
    monkeypatch.setattr(h.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(h.time, 'sleep', lambda dt: now.__setitem__(0, now[0]+dt))
    monkeypatch.setattr(m, 'acquire_backend_lease', lambda *args: contextlib.nullcontext())
    monkeypatch.setattr(m, 'open_exclusive_serial', lambda *args: contextlib.nullcontext(raw))
    report = h.check(m, object(), tmp_path/'hold.json', duration=5)
    assert raw.writes[-1] == b'STOP\n'
    assert report['stop']['stop_confirmed'] is True
    assert report['calibration_valid'] is False
    assert not any(w.startswith(b'D ') and b' D ' in w for w in raw.writes)
    if failure is None:
        assert report['status'] == 'stationary_hold_passed' and report['replies'] == 100
    else:
        assert report['status'] == 'stationary_hold_failed'
        assert 'STOP unconfirmed' not in report['error']
