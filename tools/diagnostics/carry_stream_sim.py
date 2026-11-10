"""Explicit in-memory transport for the real resident adapter, never a device.

Models timed batches, encoder contact residual, retained goals and stop ACK loss.
It is not a motor dynamics model or a substitute for the firmware C tests.
"""
from types import SimpleNamespace as NS

from home_robot_tasks.carry_execution import CarryFeedback, CarryRouteBuilder, CarrySession
from home_robot_tasks.device_ports import ResidentArmPort
from so101_arm_bridge.bimanual_stream_adapter import (
    ResidentBimanualStreamAdapter, OperationalLimits, CANONICAL_JOINT_NAMES,
    F8_CAPABILITIES, F8_FIRMWARE_VERSION, CALIBRATION_HASH,
)
from so101_arm_bridge.stationary_pick_adapter import StationaryPickAdapter
from so101_arm_bridge.stream_protocol_v2 import (
    ARM_MASK_BOTH, StreamContractResultV2, StreamStatusCodeV2, StreamExecutorStateV2,
    StreamTerminalReasonV2,
)


class SimClock:
    def __init__(self): self.now = 10.
    def __call__(self): return self.now
    def advance(self, seconds=.05): self.now += seconds


class SimTransport:
    simulated = True

    def __init__(self, clock, initial, *, continuous_observation=False):
        self.clock = clock
        self.continuous_observation = continuous_observation
        self.command = tuple(round(v*1e6) for v in initial)
        self.positions = self.command
        self.samples = []
        self.horizon = 0
        self.stop_latched = False
        self.armed = False
        self.enabled = False
        self.contact = None
        self.fault = None
        self.stop_ack = True
        self.calls = []
        self.goal_log = []

    @property
    def tick(self): return round(self.clock()*1000)

    def _io(self, name):
        self.calls.append(name)
        if self.fault == 'timeout': raise TimeoutError('injected firmware reply timeout')

    def _update(self):
        if not self.samples or self.stop_latched:
            return
        left = self.samples[0]
        for right in self.samples[1:]:
            if self.tick < right[0]:
                u = max(0., (self.tick-left[0])/(right[0]-left[0]))
                self.command = tuple(round(a+u*(b-a)) for a, b in zip(left[1], right[1]))
                break
            left = right
        else:
            self.command = left[1]
        self.positions = tuple(min(v, self.contact) if i == 5 and self.contact is not None else v
                               for i, v in enumerate(self.command))

    def enter_binary_mode(self):
        self._io('hello')
        return NS(protocol_version=2, joint_count=12, stop_latched=self.stop_latched,
                  firmware_version=F8_FIRMWARE_VERSION, left_calibration_hash=CALIBRATION_HASH,
                  right_calibration_hash=CALIBRATION_HASH, capabilities=F8_CAPABILITIES)

    def heartbeat(self):
        self._io('heartbeat'); self._update()
        return NS(stop_latched=self.stop_latched, status_code=0, last_heartbeat_ms=self.tick)

    get_state = heartbeat

    def prepare_shadow(self):
        self._io('shadow')
        return NS(status_code=0, joint_count=12, left_present_mask=63, right_present_mask=63,
                  positions_raw=(2048,)*12, unwrapped_positions_raw=(2048,)*12,
                  anchor_positions_urad=self.positions)

    def get_feedback_snapshot(self):
        self._io('feedback'); self._update()
        return NS(status_code=0, joint_count=12, present_mask=4095, firmware_tick_ms=self.tick,
                  completed_pairs=self.tick, positions_urad=self.positions,
                  sample_age_ms=(1000 if self.fault == 'stale' else 0,)*12)

    def get_dispatch_diagnostics(self):
        self._io('dispatch')
        return NS(status_code=0, active=False, faulted=self.fault == 'dispatch',
                  ready=not (self.continuous_observation and self.enabled),
                  failure_count=int(self.fault == 'dispatch'))

    def get_tracking_diagnostics(self):
        self._io('tracking')
        busy=self.continuous_observation and self.enabled
        return NS(status_code=0, active=busy, pending=busy, failed_pairs=0)

    def get_executor_diagnostics(self):
        self._io('executor'); self._update()
        state = (StreamExecutorStateV2.SUCCEEDED if self.horizon and self.tick >= self.horizon
                 else StreamExecutorStateV2.RUNNING)
        return NS(state=state, terminal_reason=StreamTerminalReasonV2.NONE, safe_stop_required=False,
                  terminal_tick=self.horizon if state==StreamExecutorStateV2.SUCCEEDED else 0)

    def stationary_runtime(self):
        self._io('stationary_runtime')
        return dict(stop_latched=self.stop_latched,
                    ready_for_stream=bool(self.horizon and self.tick>=self.horizon+100))

    def arm(self, calibration_hash):
        self._io('arm')
        if self.stop_latched: raise ValueError('latched stop')
        self.armed = True

    def enable(self):
        self._io('enable'); self.enabled = True
        return NS(stop_latched=False, status_code=0)

    def accepted(self, epoch=0):
        return NS(status_code=StreamStatusCodeV2.OK, contract_result=StreamContractResultV2.OK,
                  arm_mask=ARM_MASK_BOTH, arbiter_epoch=epoch)

    def open_stream(self, policy):
        self._io('open'); self._update()
        if not self.enabled or self.stop_latched: raise ValueError('stream not enabled')
        self.horizon = policy.horizon_end_tick
        self.samples = [(self.tick, self.command)]
        return self.accepted()

    def append(self, batch):
        self._io('append')
        for sample in batch.samples:
            if sample.apply_tick <= self.samples[-1][0] or sample.apply_tick <= self.tick:
                raise ValueError('late/out-of-order simulated firmware sample')
            self.samples.append((sample.apply_tick, sample.positions_urad))
            self.goal_log.append(sample.positions_urad)
        return self.accepted(batch.arbiter_epoch)

    def safe_stop(self):
        self.calls.append('safe_stop')
        if not self.stop_ack: raise TimeoutError('injected missing STOP acknowledgement')
        self._update(); self.stop_latched = True
        return NS(stop_latched=True, status_code=0)


class SimStop:
    def __init__(self, adapter): self.adapter = adapter
    def request(self, goal, immediate, now): self.adapter.stop('carry-lab')
    def poll(self, goal, now):
        return NS(status='STOPPED' if self.adapter.state.value == 'stopped' else 'STOPPING')


class ContinuouslyObservedSimulationAdapter(ResidentBimanualStreamAdapter):
    # Same terminal policy as the exact S3 firmware adapter; synthetic fixture
    # coordinate ranges and in-memory identity remain explicitly simulated.
    _terminal_bus_available = StationaryPickAdapter._terminal_bus_available


def simulate(program, *, fault=None, validator=None, continuous_observation=False,
             observation_policy='continuous_patch',ready_for_route=None,
             pause_before_descent=False,checkpoint_action=None):
    """Exercise production timing/builder/session/ResidentArmPort/stream adapter."""
    clock = SimClock()
    transport = SimTransport(clock, program.stages[0].points[0].positions_rad,
                             continuous_observation=continuous_observation)
    # These limits authorize only this explicitly in-memory adapter instance.
    limits = OperationalLimits(CANONICAL_JOINT_NAMES,
                               tuple(round(v*1e6) for v in program.bounds.minimum),
                               tuple(round(v*1e6) for v in program.bounds.maximum))
    adapter_type=ContinuouslyObservedSimulationAdapter if continuous_observation else ResidentBimanualStreamAdapter
    adapter = adapter_type(transport, limits, motion_authorized=True)
    adapter.prepare()
    stop = SimStop(adapter)
    builder = CarryRouteBuilder(program, validator or (lambda stage, snapshot: True),
                                pause_before_descent=pause_before_descent)
    port = ResidentArmPort(adapter, builder, stop, clock, transport_owner='carry-lab')
    sequence = 0
    events = []
    injected = False

    def observe():
        nonlocal sequence, injected
        sequence += 1
        name = program.stages[min(session.index, len(program.stages)-1)].name
        if name == 'close':
            # Encoder residual represents contact, not perfect command following.
            transport.contact = round(sum(program.contact_control_interval)/2*1e6)
        relation = 'held' if name in ('probe', 'lift', 'carry', 'hold') else 'unknown'
        if observation_policy=='snapshot_then_verify':relation='unknown'
        if fault in ('no_follow', 'empty_close') and name == 'probe': relation = 'unknown'
        if fault == 'slip' and name == 'carry': relation = 'empty'
        if fault in ('timeout', 'dispatch', 'stale') and name == 'lift' and not injected:
            transport.fault = fault; injected = True
        if fault == 'stop_unconfirmed' and name == 'carry':
            relation = 'empty'; transport.stop_ack = False
        if fault == 'empty_close' and name == 'close':
            transport.contact = round(program.close_control_rad*1e6)+200000
        snapshot = adapter.feedback_snapshot()
        positions=list(snapshot.positions_urad)
        if fault=='jaw_outside_envelope' and name in ('probe','lift','carry','hold'):
            positions[5]=round((min(program.contact_control_interval)-.2)*1e6)
        return CarryFeedback(sequence, clock()-max(snapshot.sample_age_ms)/1000, 10., program.capture_id, program.calibration_id,
                             dict(program.source_sha256), tuple(v/1e6 for v in positions),
                             program.planned_lift_m, fault != 'base_moved' or name != 'carry',
                             True, True, True, relation, sequence, clock(),
                             vision_capture_id=program.capture_id, vision_target_id=program.candidate_id,
                             observation_policy=observation_policy)

    session = CarrySession(program, port, stop, observe, clock, simulation=True,
                           observation_policy=observation_policy,
                           pause_before_descent=pause_before_descent,
                           ready_for_route=(lambda:ready_for_route(clock())) if ready_for_route else None)
    session.start('simulated-pick')
    max_s = sum(s.timing['duration_s'] for s in program.stages)+60.
    last_state = None
    checkpoint_ticks=0
    while clock() < 10.+max_s:
        if session.state=='AWAITING_DESCENT_CONFIRMATION':
            checkpoint_ticks+=1
            if checkpoint_ticks==200:
                if checkpoint_action=='resume':session.resume_descent()
                elif checkpoint_action=='cancel':session.cancel()
                elif checkpoint_action=='stale':transport.fault='stale'
        if fault == 'cancel' and session.index >= 4:
            session.cancel()
        session.poll()
        state = (session.state, session.index)
        if state != last_state:
            events.append(dict(at_s=round(clock()-10., 3), state=state[0], stage_index=state[1], reason=session.reason))
            last_state = state
        if session.state == 'HOLDING':
            # The session continues to service its heartbeat after reaching carry.
            for _ in range(100):
                clock.advance(); session.poll()
            break
        if session.state in ('FAILED', 'CANCELLED'):
            break
        if session.state == 'STOPPING' and fault == 'stop_unconfirmed':
            for _ in range(10): clock.advance(); session.poll()
            break
        clock.advance()
    return dict(mode='in_memory_resident_transport', hardware_commands=0, fault=fault,
                observation_policy=observation_policy,
                holding_evidence=session.holding_reason if session.state=='HOLDING' else None,
                state=session.state, reason=session.reason, stages=session.history,
                events=events, simulated_stream_samples=len(transport.goal_log),
                simulated_arm_calls=transport.calls.count('arm'),
                simulated_enable_calls=transport.calls.count('enable'),
                simulated_stop_calls=transport.calls.count('safe_stop'),
                final_command_rad=[v/1e6 for v in transport.command],
                final_measured_rad=[v/1e6 for v in transport.positions],
                simulated_time_s=round(clock()-10., 3),
                checkpoint_ticks=checkpoint_ticks,descent_released=session.descent_released,
                checkpoint_hold_cycles=session.pregrasp_hold_cycle,
                physical_task_completed=False)
