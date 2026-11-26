from contextlib import contextmanager
from dataclasses import dataclass, replace
from types import SimpleNamespace as NS

import pytest

from tools.run import recover_supported_lift as task
from so101_arm_bridge.lift_wire import LiftCommand, LiftStatus
from so101_arm_bridge.unified_commissioning import CommissionStatus


@dataclass
class Hello:
    firmware_version: int = 0x2492d
    joint_count: int = 12
    stop_latched: bool = True
    left_calibration_hash: int = 1
    right_calibration_hash: int = 2


BASELINE = dict(hello=dict(left_calibration_hash=1, right_calibration_hash=2))


class Clock:
    def __init__(self): self.now = 0.
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


def commission(*, boot=87, flags=127):
    return CommissionStatus(1, 1, 1000, boot, 0, 900, flags, 0, task.EXPECTED_PROFILE)


def preflight_transport(monkeypatch, *, change=None, hello=None, cs=None):
    from so101_arm_bridge import unified_commissioning, system_stop
    clock = Clock()
    events = []
    class Transport:
        def enter_binary_mode(self): events.append('hello'); return hello or Hello()
        def stationary_runtime(self):
            events.append('cached_runtime')
            return dict(stop_latched=True, executor_active=False,
                        mobile_board=dict(source=2, tick_ms=task.EXPECTED_STOP_TICK,
                                          lift_fault=1, position_raw=3269, goal_velocity_raw=0))
        def exchange_mobile(self, packet):
            q = LiftCommand.decode(packet)
            assert q.kind == 4 and q.valid_until_ms == 0 and q.target_um == 0
            events.append('lift_query')
            stamp = 1000 + round(clock.now * 1000)
            value = LiftStatus(4, q.sequence, stamp + 1, 87, 2, 617, 6, 1, 8,
                               stamp, 8141, 10000, 0, 20, 3269, 1, 999)
            return value if change is None else change(value)
    class Client:
        def __init__(self, _): pass
        def query(self): events.append('commission_query'); return cs or commission()
    class Stop:
        def __init__(self, *_): pass
        def status(self):
            events.append('stop_query')
            from so101_arm_bridge.system_stop import SystemStopStatus
            return SystemStopStatus(1, 8000000, 87, task.EXPECTED_STOP_TICK,
                                    task.EXPECTED_STOP_TICK, 3, 95, task.EXPECTED_STOP_TICK-142,
                                    8000000-task.EXPECTED_STOP_TICK+142, 250, 0), 0.
        def request(self): raise AssertionError('no STOP write during preflight')
    monkeypatch.setattr(unified_commissioning, 'CommissionClient', Client)
    monkeypatch.setattr(system_stop, 'SystemStopClient', Stop)
    monkeypatch.setattr(LiftStatus, 'decode', lambda packet: packet)
    return Transport(), clock, events


def test_preflight_is_status_only_and_requires_fresh_position_window(monkeypatch):
    transport, clock, events = preflight_transport(monkeypatch)
    result = task.preflight(transport, BASELINE, clock=clock, sleep=clock.sleep)
    assert clock() >= .4 and len(result['lift_observations']) >= 3
    assert set(events) == {'hello', 'cached_runtime', 'commission_query', 'stop_query', 'lift_query'}


@pytest.mark.parametrize('changes', [
    dict(position_raw=3272), dict(observed_ms=0), dict(command_raw=10),
    dict(flags=11), dict(boot_id=88), dict(state=5, fault=0),
])
def test_preflight_refuses_drift_staleness_motion_new_boot_and_changed_state(monkeypatch, changes):
    transport, clock, _ = preflight_transport(monkeypatch, change=lambda r: replace(r, **changes))
    with pytest.raises(ValueError): task.preflight(transport, BASELINE, clock=clock, sleep=clock.sleep)


@pytest.mark.parametrize('cs', [commission(boot=88), commission(boot=0, flags=0)])
def test_after_reset_or_another_boot_cannot_repeat_recovery(monkeypatch, cs):
    transport, clock, _ = preflight_transport(monkeypatch, cs=cs)
    with pytest.raises(ValueError): task.preflight(transport, BASELINE, clock=clock, sleep=clock.sleep)


def test_stationary_window_cannot_be_built_from_repeated_timestamp(monkeypatch):
    transport, clock, _ = preflight_transport(monkeypatch, change=lambda r: replace(r, observed_ms=1000))
    with pytest.raises(ValueError, match='did not advance'):
        task.preflight(transport, BASELINE, clock=clock, sleep=clock.sleep)


@pytest.mark.parametrize('failure', ['none', 'preflight', 'reset', 'post_reset', 'release'])
def test_recovery_uses_one_reset_and_one_release_without_retry(monkeypatch, failure):
    from so101_arm_bridge import unified_commissioning
    events = []
    @contextmanager
    def connection():
        events.append('open')
        try: yield NS(enter_binary_mode=lambda: Hello(stop_latched=failure=='post_reset'))
        finally: events.append('close')
    def preflight(*_):
        events.append('preflight')
        if failure == 'preflight': raise ValueError('preflight failed')
        return {}
    def reset():
        assert events[-1] == 'save' and events[-2] == 'close'
        events.append('reset')
        if failure == 'reset': raise TimeoutError('reset result unknown')
    def release(_, baseline, report):
        events.append('release')
        if failure == 'release': raise RuntimeError('release failed')
        report['release'] = dict(final_all_torque_off=True, calibration_preserved=True)
    monkeypatch.setattr(task, 'preflight', preflight)
    monkeypatch.setattr(task, 'release_once', release)
    monkeypatch.setattr(unified_commissioning, 'CommissionClient',
                        lambda _: NS(query=lambda: commission(boot=0, flags=0)))
    report = {}
    if failure == 'none':
        task.run_recovery(connection, reset, BASELINE, report, lambda:events.append('save'), sleep=lambda _:None)
        assert report['status']=='ALL_16_TORQUE_OFF_VERIFIED' and not report['lift_reference_valid']
    else:
        with pytest.raises((ValueError, RuntimeError, TimeoutError)):
            task.run_recovery(connection, reset, BASELINE, report, lambda:events.append('save'), sleep=lambda _:None)
    assert events.count('reset') == (failure != 'preflight')
    assert events.count('release') == (failure in ('none', 'release'))
    assert events.count('open') == events.count('close')


def inventory(stamp, *, powered=False, incomplete=False):
    rows = []
    for side, count in [('left',10), ('right',6)]:
        for mid in range(1,count+1):
            mobile = side=='left' and mid>=7
            rows.append(dict(bus=side, requested_id=mid, complete=True,
                             sample_started_tick_ms=stamp,
                             identity=dict(reported_id=mid, model_raw=1),
                             limits=dict(minimum_raw=0, maximum_raw=4095),
                             offset_and_mode=dict(offset_raw=0, operating_mode=int(mobile)),
                             command=dict(torque_enabled_raw=int(powered), goal_speed_raw=0),
                             feedback=dict(position_raw=3269 if mobile else 2000, speed_raw=0,
                                           moving_raw=0, lock_raw=1)))
    return dict(devices=rows, all_snapshots_complete=not incomplete)


@pytest.mark.parametrize('outcome', ['off', 'still_on', 'missing', 'post_ack_failure', 'preflight_failure'])
def test_release_error_never_resends_and_independently_checks_all_motors(monkeypatch, outcome):
    import prepare_unified_controller as prep
    from so101_arm_bridge import device_inventory
    events = []
    before = inventory(0, powered=True)
    def release(*_, report, **kw):
        events.append('release')
        report['inventory_before'] = before
        if outcome!='preflight_failure': report['release_attempted'] = True
        if outcome=='post_ack_failure': report['release'] = dict(kind=5)
        raise RuntimeError('release response or verification error')
    count = 0
    def collect(_):
        nonlocal count
        events.append('readback'); count += 1
        return inventory(count*200, powered=outcome=='still_on', incomplete=outcome=='missing')
    monkeypatch.setattr(prep, 'release_boot_supported', release)
    monkeypatch.setattr(device_inventory, 'collect_inventory', collect)
    report = {}
    clock = Clock()
    if outcome=='off':
        task.release_once(None, BASELINE, report, clock=clock, sleep=clock.sleep)
        assert report['release']['final_all_torque_off']
        assert report['release']['release_error'] and count >= 3
    else:
        with pytest.raises((ValueError, RuntimeError)):
            task.release_once(None, BASELINE, report, clock=clock, sleep=clock.sleep)
        assert not report['release'].get('final_all_torque_off')
        if outcome in ('post_ack_failure','preflight_failure'): assert count == 0
    assert events.count('release') == 1
