"""Measured platform age must survive cache ingestion without a second cutoff.

The 2026-11-09 pad retry discarded two 181 ms packets although their full
request/response age was below 200 ms. The last accepted sample then expired.
These are host-time regressions, not a simulation of physical grasp success.
"""
import pytest

from home_robot_tasks.carry_live_inputs import ObservationNotFresh
from test_carry_live_inputs import packet, platform
from test_carry_snapshot_execution import admitted_inputs


def test_recorded_181ms_handoff_packets_replace_the_previous_sample():
    inputs, now, _, _ = admitted_inputs()
    inputs.admit_snapshot(); inputs.set_phase('open')
    # Rebase the recorded MCU ticks to the existing test clock. The exact two
    # age vectors and measured round trips come from retry-1/physical-trial.
    inputs.add_platform(platform(1100, sample_age_ms=(76, 41, 146, 111)), 10.13, 10.133)
    before = inputs.platform.stamp
    for tick, sent, rtt, ages in (
        (1135, 10.165, .003303169, (111, 76, 181, 146)),
        (1170, 10.200, .003333505, (146, 111, 30, 181)),
    ):
        inputs.add_platform(platform(tick, sample_age_ms=ages), sent, sent+rtt)
        now[0] = sent+rtt+.002
        inputs.add_joints(packet(completed_pairs=tick), sent, sent+.003)
        assert inputs.feedback().base_stopped
    assert inputs.platform.stamp > before
    assert inputs.platform.stamp == pytest.approx(10.019)
    assert not inputs.route_start_ready()  # Observe, but don't start a new handoff yet.
    now[0] = 10.220
    with pytest.raises(ValueError, match='platform sample stale'):
        inputs.feedback()  # The very same accepted sample still expires at 200 ms.


@pytest.mark.parametrize('age_ms,rtt_ms', [(181, 3.334), (190, 4), (198, 1), (100, 40)])
def test_freshness_budget_includes_actual_reply_time_at_receipt_and_use(age_ms, rtt_ms):
    inputs, now, _, _ = admitted_inputs(); inputs.admit_snapshot()
    sent = 10.13
    inputs.add_platform(platform(1100, sample_age_ms=(age_ms, 0, 0, 0)), sent, sent+rtt_ms/1000)
    stamp = sent-age_ms/1000
    now[0] = sent+rtt_ms/1000
    inputs.add_joints(packet(completed_pairs=11), sent, sent+.001)
    assert inputs.feedback().base_stopped
    assert inputs.platform.stamp == pytest.approx(stamp)
    now[0] = stamp+.2001
    with pytest.raises(ValueError, match='platform sample stale'):
        inputs.feedback()


@pytest.mark.parametrize('age_ms,rtt_ms', [(181, 20), (199, 2), (201, 1), (100, 101)])
def test_expired_packet_cannot_renew_the_last_measurement(age_ms, rtt_ms):
    inputs, now, _, _ = admitted_inputs(); inputs.admit_snapshot()
    old = inputs.platform.stamp
    with pytest.raises(ObservationNotFresh):
        inputs.add_platform(platform(1100, sample_age_ms=(age_ms, 0, 0, 0)), 10.13, 10.13+rtt_ms/1000)
    assert inputs.platform.stamp == old
    now[0] = 10.25
    with pytest.raises(ValueError, match='platform sample stale'):
        inputs.feedback()


def test_repeated_cached_values_do_not_become_new_encoder_evidence():
    inputs, now, _, _ = admitted_inputs(); inputs.admit_snapshot()
    inputs.add_platform(platform(1100, sample_age_ms=(180, 145, 110, 75)), 10.13, 10.133)
    stamp = inputs.platform.stamp
    inputs.add_platform(platform(1110, sample_age_ms=(190, 155, 120, 85)), 10.14, 10.143)
    assert inputs.platform.stamp == pytest.approx(stamp)
    with pytest.raises(ObservationNotFresh):
        inputs.add_platform(platform(1120, sample_age_ms=(200, 165, 130, 95)), 10.15, 10.153)
    now[0] = 10.153
    with pytest.raises(ValueError, match='platform sample stale'):
        inputs.feedback()


def run_timed_route(p, monkeypatch, phase_ms=0, fault=None, *,
                    pause_s=.045, validation_s=0., oldest_first=False,
                    open_delay_s=0., legacy_start_clock=False):
    """Production session/port/adapter with advancing serial + four-slot cache.

    Reads arrive 3 ms later. Starting a stream interrupts the platform reader
    for 45 ms; the four old per-motor timestamps survive the handoff. This
    covers the complete route, transitions and repeated carry holds without
    pretending that a packet query itself acquired new encoder values.
    """
    import math
    from types import SimpleNamespace as NS
    import numpy as np
    import so101_arm_bridge.bimanual_stream_adapter as bridge
    from so101_arm_bridge.stationary_pick_adapter import StationaryPickAdapter
    from home_robot_tasks.carry_execution import build_carry_session
    from home_robot_tasks.carry_live_inputs import FixedPlatformGuard, LiveCarryInputs, ResidentObservationReader
    from tools.diagnostics.carry_stream_sim import SimClock, SimTransport, SimStop
    from test_carry_live_inputs import scene

    clock = SimClock()
    monkeypatch.setattr(bridge, 'time', NS(monotonic=clock))

    class TimedTransport(SimTransport):
        def __init__(self):
            super().__init__(clock, p.stages[0].points[0].positions_rad, continuous_observation=True)
            last_slot = math.floor((clock()*1000+phase_ms)/35)
            self.stamps = [0.]*4
            for slot in range(last_slot-3, last_slot+1):
                self.stamps[slot % 4] = (slot*35-phase_ms)/1000
            self.next_index = (last_slot+1) % 4
            self.next_read_s = ((last_slot+1)*35-phase_ms)/1000
            self.sequence = 0; self.frozen = False
            self.largest_age_ms = 0

        def _io(self, name):
            clock.advance(.003)
            super()._io(name)

        def open_stream(self, policy):
            clock.advance(open_delay_s)
            self.policy=policy
            return super().open_stream(policy)

        def append(self, batch):
            # Match the firmware's lead and finite-horizon checks, rather
            # than accepting a merely positive or truncated simulated route.
            if batch.samples[0].apply_tick-self.tick < self.policy.minimum_lead_ms+3:
                raise ValueError('initial append rejected: FIRST_SAMPLE_TOO_EARLY')
            assert batch.horizon_end_tick >= self.horizon
            assert batch.samples[-1].apply_tick <= batch.horizon_end_tick
            result=super().append(batch)
            self.horizon=batch.horizon_end_tick
            return result

        def stationary_platform(self, operation=0):
            self._io('platform')
            while self.next_read_s <= clock():
                if not self.frozen:
                    self.stamps[self.next_index] = self.next_read_s; self.sequence += 1
                    self.next_index = (self.next_index+1) % 4
                self.next_read_s += .035
            # Firmware retains s.next through AP2/stream resume; no motor is
            # skipped just because its old wall-clock slot elapsed in a pause.
            if operation == 2:
                self.next_read_s += pause_s
                # Production C regression separately verifies that resume
                # selects the oldest platform slot ahead of six 5 ms arm reads.
                if oldest_first:self.next_read_s -= .030
            ages = tuple(math.ceil((clock()-s)*1000-1e-7) for s in self.stamps)
            self.largest_age_ms = max(self.largest_age_ms, max(ages))
            raw = (20, 0, 0, 4095) if fault == 'base_moved' and self.frozen else (0, 0, 0, 4095)
            return platform(self.tick, raw, sequence=self.sequence, sample_age_ms=ages)

    class TimedAdapter(StationaryPickAdapter):
        # Explicit in-memory identity/ranges; retain production start/poll.
        firmware_version = bridge.F8_FIRMWARE_VERSION
        calibration_hash = bridge.CALIBRATION_HASH
        def __init__(self, transport, limits, **kwargs):
            bridge.ResidentBimanualStreamAdapter.__init__(self, transport, limits, **kwargs)
            # This fixture bypasses StationaryPickAdapter.__init__, including
            # its manifest-derived directions. Its synthetic axes all use +raw.
            self._encoder_directions = (1,)*12
            self.last_runtime_diagnostics = self.last_firmware_runtime = None

        def _validate_finite_plan_points(self, points):
            clock.advance(validation_s)
            return super()._validate_finite_plan_points(points)

        def _initial_batch_base(self, previous_base):
            if legacy_start_clock:
                self._transport.stationary_platform(2)
                return previous_base
            return super()._initial_batch_base(previous_base)

    t = TimedTransport()
    limits = bridge.OperationalLimits(bridge.CANONICAL_JOINT_NAMES,
        tuple(round(v*1e6) for v in p.bounds.minimum), tuple(round(v*1e6) for v in p.bounds.maximum))
    adapter = TimedAdapter(t, limits, motion_authorized=True); adapter.prepare()
    # Initial torque-on setup has finished before starting the observed route.
    t.arm(adapter.calibration_hash); t.enable(); adapter._armed = True
    color, depth, mask, intr = scene()
    inputs = LiveCarryInputs(p, lambda q: np.eye(4), (color, depth), mask, intr, np.eye(4).tolist(),
        FixedPlatformGuard(p.planned_lift_m, -.00002), clock=clock, observation_policy='snapshot_then_verify')
    inputs.validation = {'unchanged': True}
    inputs.scene_time = clock(); inputs.last_pair_stamp = clock()
    # Scene admission itself is covered by the image tests. This integration
    # test exercises the cached-observation production polling order.
    reader = ResidentObservationReader(t, adapter, inputs, clock=clock)
    reader.read(); inputs.admit_snapshot()
    stop = SimStop(adapter)
    # SimStop and ResidentArmPort must use the same single writer identity.
    stop.request = lambda *args: adapter.stop('stationary-pick')
    session = build_carry_session(p, adapter=adapter, coordinated_stop=stop, observe=reader.read,
        clock=clock, validate_current_scene=inputs.validate_stage, simulation=True,
        observation_policy='snapshot_then_verify', ready_for_route=inputs.route_start_ready)
    session.start('timed-route'); started = clock(); hold_start = None; fault_time = None
    while clock()-started < sum(s.timing['duration_s'] for s in p.stages)+40:
        name = p.stages[min(session.index, len(p.stages)-1)].name
        if fault and name == 'pregrasp' and fault_time is None:
            t.frozen = True; fault_time = clock()
        if name == 'close':t.contact = round(sum(p.contact_control_interval)/2*1e6)
        try:
            inputs.set_phase(name); session.poll()
        except Exception as exc:
            session._stop(str(exc)); session.poll()
        if session.state in ('FAILED', 'CANCELLED', 'STOPPING'):break
        if session.state == 'HOLDING':
            if hold_start is None:hold_start = clock()
            if clock()-hold_start > 6:break
        clock.advance(.005)
    return dict(state=session.state, reason=session.reason, stages=[s['stage'] for s in session.history],
        hold_cycle=session.hold_cycle, maximum_packet_age_ms=t.largest_age_ms, deferred_packets=len(reader.deferred),
        simulated_s=clock()-started, stop_calls=t.calls.count('safe_stop'),
        fault_stop_delay_s=None if fault_time is None else clock()-fault_time, hardware_commands=0)


@pytest.mark.parametrize('phase_ms', range(0, 140, 7))
def test_complete_route_with_serial_time_and_reader_handoffs(monkeypatch, phase_ms):
    from test_carry_execution import program
    p = program(); result = run_timed_route(p, monkeypatch, phase_ms)
    assert result['state'] == 'HOLDING', result
    assert result['stages'] == [s.name for s in p.stages]
    assert result['hold_cycle'] >= 2
    assert result['stop_calls'] == 0


@pytest.mark.parametrize('phase_ms', range(0, 140, 7))
def test_slow_handoff_refreshes_oldest_before_arm_sweep(monkeypatch, phase_ms):
    from test_carry_execution import program
    p = program()
    old = run_timed_route(p, monkeypatch, phase_ms, pause_s=.075, validation_s=.013)
    assert old['state'] == 'FAILED' and 'stale' in old['reason'], old
    fixed = run_timed_route(p, monkeypatch, phase_ms, pause_s=.075,
                            validation_s=.013, oldest_first=True)
    assert fixed['state'] == 'HOLDING' and fixed['stop_calls'] == 0, fixed
    assert fixed['stages'] == [s.name for s in p.stages]


@pytest.mark.parametrize('phase_ms',range(0,140,14))
def test_slow_open_does_not_expire_first_command_or_pause_feedback(monkeypatch,phase_ms):
    from test_carry_execution import program
    p=program()
    old=run_timed_route(p,monkeypatch,phase_ms,open_delay_s=.080,
                         oldest_first=True,legacy_start_clock=True)
    assert old['state']=='FAILED' and old['stop_calls']>=1,old
    fixed=run_timed_route(p,monkeypatch,phase_ms,open_delay_s=.080,oldest_first=True)
    assert fixed['state']=='HOLDING' and fixed['stop_calls']==0,fixed
    assert fixed['stages']==[s.name for s in p.stages]


@pytest.mark.parametrize('fault', ['frozen_platform', 'base_moved'])
def test_timed_route_stops_on_actual_frozen_encoders_or_motion(monkeypatch, fault):
    from test_carry_execution import program
    result = run_timed_route(program(), monkeypatch, fault=fault)
    assert result['state'] == 'FAILED', result
    assert result['stop_calls'] >= 1
    assert result['fault_stop_delay_s'] < .25
    assert ('stale' if fault == 'frozen_platform' else 'moved') in result['reason']


def test_stop_diagnostics_distinguish_torque_from_position_and_clear_on_proof():
    from so101_arm_bridge.stationary_runtime import ResidentStop
    from test_carry_live_inputs import StopTransport
    t = StopTransport(); now = [10.]; stop = ResidentStop(t, lambda: now[0])
    stop.request('test', True, now[0]); t.torque = 1; stop.poll('test', now[0])
    d = stop.diagnostics()
    assert d['reason'] == 'arm torque disable unconfirmed'
    assert d['evidence']['motor_id'] == 1 and d['evidence']['bus'] == 0
    t.torque = 0; now[0] += .3; stop.poll('test', now[0])
    t.position += 10; now[0] += .3; stop.poll('test', now[0])
    d = stop.diagnostics()
    assert d['status'] == 'STOPPING' and d['evidence']['maximum_delta_raw'] == 10
    now[0] += .3; stop.poll('test', now[0])
    d = stop.diagnostics()
    assert d['status'] == 'STOPPED' and d['reason'] == ''
    assert d['first_error'] == 'arm torque disable unconfirmed'
