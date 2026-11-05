import importlib.util
from pathlib import Path

import pytest

from home_robot_tasks.lift_observation import LiftObservation

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('observe_lift_origin', ROOT / 'tools/diagnostics/observe_lift_origin.py')
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


def sample(p=3820, tick=100, **changes):
    return dict(type='snapshot', id=10, telemetry_valid=True, position=p, speed=0,
                sample_tick=tick, telemetry_age_ms=2, motor_status_raw=0,
                goal=0, torque=0, **changes)


def ready(p=3820):
    tracker = LiftObservation(bottom_confirmed=True)
    for i in range(5):
        result = tracker.add(sample(p, 100+i*50), 10+i*.05)
    assert result['lift_height_m'] == 0
    return tracker


def test_low_confirmation_is_required():
    with pytest.raises(ValueError):
        LiftObservation(bottom_confirmed=False)


def test_unwrap_both_directions_at_boundary_and_official_scale():
    t = ready(3)
    assert t.add(sample(4093, 350), 10.25)['lift_raw_unwrapped'] == -3
    assert t.add(sample(4083, 400), 10.30)['lift_height_m'] == pytest.approx(16*.084/4096)
    assert t.add(sample(3, 450), 10.35)['lift_height_m'] == 0


def test_multiple_turns_do_not_collapse_to_modulo_position():
    t = ready()
    for i in range(1, 61):
        row = t.add(sample((3820-i*100) % 4096, 300+i*50), 10.2+i*.05)
    assert row['lift_raw_unwrapped'] == 3820-6000
    assert row['lift_height_m'] == pytest.approx(6000*.084/4096)
    assert row['coordinate_use_allowed'] is False


@pytest.mark.parametrize('change,received', [
    ({'sample_tick': 300}, 10.25),  # duplicate
    ({'sample_tick': 1}, 10.25),    # reset
    ({'sample_tick': 501}, 10.25),  # firmware gap
    ({}, 10.5),                    # host gap
    ({'position': 1000}, 10.25),   # impossible step
    ({'telemetry_valid': False}, 10.25),
    ({'telemetry_age_ms': 101}, 10.25),
    ({'motor_status_raw': 1}, 10.25),
    ({'position': True}, 10.25),
    ({'goal': 100}, 10.25),
])
def test_bad_sample_permanently_invalidates_origin(change, received):
    t = ready()
    s = sample(tick=350)
    s.update(change)
    with pytest.raises(ValueError):
        t.add(s, received)
    with pytest.raises(ValueError, match='origin invalid'):
        t.add(sample(tick=400), 10.3)


def test_nonzero_speed_is_not_silently_accepted_as_stationary():
    t = LiftObservation(bottom_confirmed=True)
    for i in range(8):
        s = sample(tick=100+i*50)
        s['speed'] = 32818
        assert t.add(s, 10+i*.05)['origin_id'] is None


def test_close_and_reconnect_require_new_origin():
    t = ready()
    t.invalidate('port closed')
    with pytest.raises(ValueError):
        t.add(sample(tick=350), 10.25)
    new = LiftObservation(bottom_confirmed=True)
    assert new.origin is None and new.session_id != t.session_id


class Port:
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(data)
        return len(data)


@pytest.mark.parametrize('command', [b'STOP\n', b'ARM 123\n', b'PREP 123\n', b'D 1 2 Z\n', b'RESET\n'])
def test_observer_blocks_all_write_or_reset_commands(command):
    raw = Port()
    with pytest.raises(RuntimeError):
        cli.ReadOnlyPort(raw).write(command)
    assert not raw.writes


class Runtime:
    def __init__(self, fault=False, armed=False):
        self.fault, self.armed = fault, armed
        self.tick = 50

    def inspect(self, port):
        port.write(b'INFO\n')
        return {'firmware': 'lift-teleop-v4'}

    def send(self, port, data):
        port.write(data)

    def receive(self, port, timeout):
        return dict(type='status', armed=self.armed, fault=self.fault, reason='HOST_RX_ERROR',
                    motor_write_count=0, nonzero_speed_writes=0)

    def snapshot(self, port):
        port.write(b'SNAP\n')
        self.tick += 50
        return sample(tick=self.tick)


@pytest.mark.parametrize('fault,armed', [(True, False), (False, True)])
def test_fault_or_armed_prevents_even_snapshot(fault, armed):
    raw = Port()
    report = cli.observe(Runtime(fault, armed), raw, samples=5, bottom_confirmed=True)
    assert report['status'] == 'failed'
    assert raw.writes == [b'INFO\n', b'STATUS\n']
    assert report['runtime_origin_valid'] is False


def test_success_is_evidence_not_persistent_runtime_origin():
    raw = Port()
    report = cli.observe(Runtime(), raw, samples=5, bottom_confirmed=True)
    assert report['status'] == 'bottom_observation_passed'
    assert report['origin_raw_unwrapped'] == 3820
    assert report['session_closed'] and not report['runtime_origin_valid']
    assert set(raw.writes) == {b'INFO\n', b'STATUS\n', b'SNAP\n'}


def test_movement_before_quiet_cannot_redefine_bottom():
    t = LiftObservation(bottom_confirmed=True)
    t.add(sample(tick=100), 10)
    with pytest.raises(ValueError, match='moved before bottom'):
        t.add(sample(p=3810, tick=150), 10.05)
