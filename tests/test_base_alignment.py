"""Camera-planned lateral base alignment: plan sign, timed strafe order, STOP, station flow."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from home_robot_tasks import base_alignment as ba
from home_robot_tasks.pick_service import Cancelled, PickStation, Rejected
from home_robot_tasks.pick_web_backend import WebPickBackend
from test_pick_pipeline import FakeCompute
from test_pick_service import wait_for


def profile():
    return ba.load_alignment_profile()


def test_released_profile_is_bounded_and_symmetric():
    p = profile()
    assert p['wheel_raw']['left'] == [-50, -50, 100, 0]
    assert p['wheel_raw']['right'] == [50, 50, -100, 0]
    broken = deepcopy(p); broken['wheel_raw']['right'] = [50, 50, -90, 0]
    path = ba.PROFILE.parent/'_test_alignment.json'
    try:
        import json
        path.write_text(json.dumps(broken))
        with pytest.raises(ValueError, match='opposite'):
            ba.load_alignment_profile(path)
    finally:
        path.unlink(missing_ok=True)


def test_object_left_of_target_moves_base_left_by_the_difference():
    p = profile()
    plan = ba.plan_lateral_alignment([.37, .40, .75], p)
    assert plan['direction'] == 'left' and plan['shift_m'] == pytest.approx(.40-p['left_target_y_m'])
    assert plan['wheel_raw'] == p['wheel_raw']['left']
    assert plan['duration_s'] == pytest.approx(plan['shift_m']/p['speed_m_per_s']['left'])
    right = ba.plan_lateral_alignment([.37, .02, .75], p)
    assert right['direction'] == 'right' and right['shift_m'] < 0
    assert right['wheel_raw'] == p['wheel_raw']['right']


def test_within_tolerance_never_moves_and_far_target_is_rejected_not_clipped():
    p = profile()
    near = ba.plan_lateral_alignment([.37, p['left_target_y_m']+.02, .75], p)
    assert near['within_tolerance'] and near['duration_s'] == 0 and not any(near['wheel_raw'])
    with pytest.raises(ValueError, match='허용 최대'):
        ba.plan_lateral_alignment([.37, p['left_target_y_m']+p['maximum_shift_m']+.01, .75], p)
    with pytest.raises(ValueError):
        ba.plan_lateral_alignment([.37, float('nan'), .75], p)


class Clock:
    def __init__(self):self.t = 0.
    def __call__(self):return self.t
    def sleep(self, s):self.t += s


class Reply:
    def __init__(self, tick, feedback, ok=True):
        self.mcu_tick_ms, self.feedback_tick_ms, self.ok = tick, feedback, ok
    def ready_for_motion(self, session, age):
        return self.ok and session == 1


class Mobile:
    def __init__(self, clock, fail_at=None):
        self.clock, self.session, self.commands, self.fail_at = clock, None, [], fail_at
    def velocity(self, values, lifetime_ms):
        assert lifetime_ms <= 200 and self.session == 1
        self.commands.append(tuple(values))
        tick = int(self.clock()*1000)
        return Reply(tick, tick-20, ok=len(self.commands) != self.fail_at)


def strafe(plan, mobile, clock, events, check=lambda: None):
    def admit():
        events.append('admit'); mobile.session = 1
    return ba.run_lateral_shift(plan, mobile=mobile, observe=lambda: events.append('observe'),
        enter_transport=lambda: events.append('transport'), admit=admit, check=check,
        clock=clock, sleep=clock.sleep)


def test_strafe_enters_transport_then_arms_then_commands_then_holds_zero():
    clock = Clock(); mobile = Mobile(clock); events = []
    plan = ba.plan_lateral_alignment([.37, .30, .75], profile())
    result = strafe(plan, mobile, clock, events)
    assert events[:2] == ['transport', 'admit']
    nonzero = [c for c in mobile.commands if any(c)]
    assert set(nonzero) == {tuple(plan['wheel_raw'])} and result['nonzero_commands'] == len(nonzero)
    first_zero = mobile.commands.index((0, 0, 0, 0))
    assert all(any(c) for c in mobile.commands[:first_zero]) and not any(map(any, mobile.commands[first_zero:]))
    assert result['commanded_s'] == pytest.approx(plan['duration_s'], abs=.03)
    assert result['settled_s']-result['commanded_s'] >= .4-1e-6


def test_unhealthy_reply_and_cancellation_propagate_for_the_callers_stop():
    plan = ba.plan_lateral_alignment([.37, .30, .75], profile())
    clock = Clock()
    with pytest.raises(RuntimeError, match='상태 확인'):
        strafe(plan, Mobile(clock, fail_at=3), clock, [])
    clock = Clock(); calls = []
    def check():
        calls.append(1)
        if len(calls) > 5: raise Cancelled('중단 요청')
    with pytest.raises(Cancelled):
        strafe(plan, Mobile(clock), clock, [], check=check)
    with pytest.raises(ValueError):
        strafe(ba.plan_lateral_alignment([.37, .15, .75], profile()), Mobile(clock), clock, [])


def hardware_fixture(monkeypatch, *, fail=False, scope_confirms=True):
    from home_robot_tasks import pick_hardware, platform_preparation
    hw = pick_hardware.UnifiedPickHardware.__new__(pick_hardware.UnifiedPickHardware)
    record = []
    evidence = NS(transport_only=False)
    stop_status = NS(base_confirmed=scope_confirms, scope=1, state=2, flags=31)
    hw.prepared, hw.stopped, hw.stop_confirmed, hw.session, hw.inputs = True, False, False, None, None
    hw.guard = NS(supported_top=True, require_fresh=lambda now: None)
    hw.transport = NS(heartbeat=lambda: NS(stop_latched=False), timing_diagnostics=lambda: {},
                      stationary_runtime=lambda: {}, stationary_feedback_failures=lambda: {})
    hw.evidence = evidence
    hw.mobile = NS(session=None)
    hw.stop = NS(request=lambda: record.append('stop'), status=lambda: (stop_status, 0.))
    hw.observe_preparation = lambda: record.append('arm-observe')
    def enter(observe, ev):
        observe(); ev.transport_only = True; record.append('transport')
    monkeypatch.setattr(platform_preparation, 'enter_base_transport', enter)
    monkeypatch.setattr(platform_preparation, 'arm_observed_stationary',
                        lambda observe, mobile, transport_only: record.append(('admit', transport_only)))
    def run(plan, **kwargs):
        kwargs['enter_transport'](); kwargs['admit']()
        if fail: raise RuntimeError('바퀴 명령 상태 확인 실패')
        return dict(commanded_s=1., nonzero_commands=50, settled_s=1.5)
    monkeypatch.setattr(ba, 'run_lateral_shift', run)
    return hw, record


class Ctx:
    def __init__(self, folder):self.folder, self.notes, self.artifacts = folder, [], []
    def service(self):pass
    def check(self):pass
    def note(self, message, **values):self.notes.append(message)
    def artifact(self, path, label):self.artifacts.append(label)


def test_hardware_shift_always_ends_in_transport_scope_stop(tmp_path, monkeypatch):
    plan = ba.plan_lateral_alignment([.37, .30, .75], profile())
    hw, record = hardware_fixture(monkeypatch)
    hw.lateral_shift(Ctx(tmp_path), plan)
    assert record == ['arm-observe', 'transport', ('admit', True), 'stop']
    assert hw.stopped and hw.stop_confirmed and not hw.base_moving
    hw, record = hardware_fixture(monkeypatch, fail=True)
    failed = tmp_path/'fail'; failed.mkdir()
    with pytest.raises(RuntimeError, match='상태 확인'):
        hw.lateral_shift(Ctx(failed), plan)
    assert record[-1] == 'stop' and hw.stop_confirmed


def test_unconfirmed_base_stop_is_reported_not_hidden(tmp_path, monkeypatch):
    from home_robot_tasks import pick_hardware
    plan = ba.plan_lateral_alignment([.37, .30, .75], profile())
    hw, record = hardware_fixture(monkeypatch, scope_confirms=False)
    clock = Clock()
    monkeypatch.setattr(pick_hardware.time, 'monotonic', clock)
    monkeypatch.setattr(pick_hardware.time, 'sleep', clock.sleep)
    with pytest.raises(RuntimeError, match='base stop not confirmed'):
        hw.lateral_shift(Ctx(tmp_path), plan)
    assert hw.stopped and not hw.stop_confirmed


def test_shift_requires_an_empty_supported_top_hold(tmp_path, monkeypatch):
    plan = ba.plan_lateral_alignment([.37, .30, .75], profile())
    hw, record = hardware_fixture(monkeypatch)
    hw.session = NS(state='HOLDING')
    with pytest.raises(ValueError, match='empty supported top'):
        hw.lateral_shift(Ctx(tmp_path), plan)
    assert record == []


def station_fixture(tmp_path, object_y, shifts):
    baseline = tmp_path/'baseline.json'; baseline.write_text('{}')
    class Fixture(WebPickBackend):
        def _camera_capture(self, ctx, mode):
            return dict(snapshot='camera', camera_mode=mode)
        def _detect_camera(self, ctx, value):
            return value
        def _camera_result(self, ctx, value):
            return dict(status='CAMERA_CANDIDATES_ONLY', reason=None, count=3, depth_fraction=.9,
                        surface_error=None, object_base_xyz_m=[.37, object_y, .75])
    hardware = []
    def factory(options):
        hw = NS(options=options, prepared=False, stopped=False, guard=NS(supported_top=True), tick=lambda: None)
        def prepare(ctx):
            assert options['supported_empty_top'] and options['execute']; hw.prepared = True
        def lateral_shift(ctx, plan):
            shifts.append(plan); return dict(commanded_s=plan['duration_s'], nonzero_commands=1, settled_s=1.)
        hw.prepare, hw.lateral_shift = prepare, lateral_shift
        hw.close = lambda: setattr(hw, 'closed', True)
        hardware.append(hw)
        return hw
    backend = Fixture(dict(mode='station', top_baseline=str(baseline), baseline=str(baseline), camera_mode='wide'),
                      worker_factory=lambda *_: FakeCompute(), hardware_factory=factory)
    return backend, hardware


def test_station_align_moves_once_then_blocks_until_reset(tmp_path):
    shifts = []
    backend, hardware = station_fixture(tmp_path, .40, shifts)
    station = PickStation(backend, tmp_path/'station'); station.start()
    try:
        wait_for(lambda: station.snapshot()['accepting'])
        with pytest.raises(Rejected):
            station.submit('지우개', operation='top-align')  # physical confirmation required
        station.submit('지우개', operation='top-align', supported_empty_top=True)
        wait_for(lambda: station.snapshot()['history'])
        result = station.snapshot()['last_result']
        assert result['status'] == 'UNVERIFIED' and result['physical_success'] is False
        assert result['alignment']['direction'] == 'left' and len(shifts) == 1
        assert hardware[0].closed and backend.hardware is None
        stages = [s['id'] for s in station.snapshot()['current']['stages']]
        assert stages == ['CAPTURING', 'DETECTING', 'GENERATING_GRASPS', 'ALIGNING']
        wait_for(lambda: station.snapshot()['phase'] == 'BLOCKED')
        assert '리셋' in station.snapshot()['blocked_reason']
        station.request_reset()
        wait_for(lambda: station.snapshot()['accepting'])
        station.submit('지우개', operation='top-inspect')
        wait_for(lambda: len(station.snapshot()['history']) == 2)
        assert station.snapshot()['last_result']['status'] == 'OBSERVED_ONLY'
        assert station.snapshot()['last_result']['alignment']['direction'] == 'left'
        assert len(hardware) == 1  # inspect stays camera-only
    finally:
        station.close()


def test_station_align_within_tolerance_never_opens_the_controller(tmp_path):
    shifts = []
    backend, hardware = station_fixture(tmp_path, profile()['left_target_y_m'], shifts)
    station = PickStation(backend, tmp_path/'station'); station.start()
    try:
        wait_for(lambda: station.snapshot()['accepting'])
        station.submit('지우개', operation='top-align', supported_empty_top=True)
        wait_for(lambda: station.snapshot()['history'] and station.snapshot()['accepting'])
        assert station.snapshot()['last_result']['alignment']['within_tolerance']
        assert hardware == [] and shifts == []
    finally:
        station.close()


def test_station_rejects_far_object_before_any_motion(tmp_path):
    shifts = []
    backend, hardware = station_fixture(tmp_path, .90, shifts)
    station = PickStation(backend, tmp_path/'station'); station.start()
    try:
        wait_for(lambda: station.snapshot()['accepting'])
        station.submit('지우개', operation='top-align', supported_empty_top=True)
        wait_for(lambda: station.snapshot()['history'] and station.snapshot()['accepting'])
        result = station.snapshot()['last_result']
        assert result['status'] == 'FAILED' and '허용 최대' in result['reason']
        assert hardware == [] and shifts == []
    finally:
        station.close()
