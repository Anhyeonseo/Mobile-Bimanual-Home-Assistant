"""L3 stationary pick supervisor using the existing single-writer resident port.

This module receives observations, not detector scores or commanded poses as
proof of holding. Device-specific acquisition/commissioning are injected at the
application boundary. There is no default hardware connection or auto-rearm.
"""
from dataclasses import dataclass
import math

from .skill_ports import PortResult


def build_carry_session(program, *, adapter, coordinated_stop, observe, clock,
                       validate_current_scene, simulation=False, observation_policy='continuous_patch',
                       ready_for_route=None, pause_before_descent=False):
    """Compose over prepared clients; never open a port, flash or auto-home.

    observe reads bounded MCU feedback; validate_current_scene uses the cached
    scene validation. Slow camera inference and MoveIt run before dispatch in
    their own planning/acquisition layer, never block the stream service loop.
    """
    from .device_ports import ResidentArmPort
    builder = CarryRouteBuilder(program, validate_current_scene,pause_before_descent=pause_before_descent)
    motion = ResidentArmPort(adapter, builder, coordinated_stop, clock,
                             transport_owner='stationary-pick')
    return CarrySession(program, motion, coordinated_stop, observe, clock, simulation=simulation,
                        observation_policy=observation_policy,ready_for_route=ready_for_route,
                        pause_before_descent=pause_before_descent)


@dataclass(frozen=True)
class CarryFeedback:
    sequence: int
    observed_s: float
    capture_observed_s: float
    capture_id: str
    calibration_id: str
    source_sha256: dict
    positions_rad: tuple
    lift_m: float
    base_stopped: bool
    lift_stopped: bool
    controller_ok: bool
    scene_current: bool
    # Derived from independent synchronized vision/FK, e.g. lift_relation().
    payload_relation: str = 'unknown'
    vision_sequence: int = -1
    vision_s: float = -1.
    resident_profile_verified: bool = False
    grip_cap_verified: bool = False
    vision_capture_id: str = ''
    vision_target_id: str = ''
    scene_validated_s: float = -1.
    observation_policy: str = 'continuous_patch'


class CarryRouteBuilder:
    """Same ResidentArmPort for jaw and arm; measured contact never reopens jaw."""
    def __init__(self, program, validate_current_scene, *, pause_before_descent=False):
        self.program = program
        if not callable(validate_current_scene):
            raise ValueError('whole-route current-scene validator required')
        self.validate = validate_current_scene
        self.last_index = -1
        self.pregrasp_hold=None
        if pause_before_descent:
            from .carry_trajectory import retime
            from .carry_program import CarryStage
            if [s.name for s in program.stages[:3]]!=['open','pregrasp','approach']:
                raise ValueError('checked pregrasp endpoint required for descent checkpoint')
            end=program.stages[1].points[-1].positions_rad
            points,timing=retime([end,end],program.bounds,dwell_s=1.)
            self.pregrasp_hold=CarryStage('pregrasp_hold',points,timing,False)

    def __call__(self, parameters, snapshot):
        if parameters=={'pregrasp_hold':True}:
            if self.pregrasp_hold is None or self.last_index!=1:
                raise ValueError('pregrasp hold only after the checked pregrasp stage')
            stage=self.pregrasp_hold
            raw=tuple(snapshot.positions_urad)
            if len(raw)!=12 or any(type(v) is not int for v in raw):
                raise ValueError('fresh twelve-axis measured snapshot required')
            if any(abs(a/1e6-b)>.02 for a,b in zip(raw,stage.points[0].positions_rad)):
                raise ValueError('pregrasp checkpoint anchor changed')
            if self.validate(stage,snapshot) is not True:
                raise ValueError('current-scene checkpoint validation failed')
            return stage.points
        if set(parameters) != {'carry_stage'} or type(parameters['carry_stage']) is not int:
            raise ValueError('explicit compiled stage required')
        index = parameters['carry_stage']
        hold_repeat = index == self.last_index == len(self.program.stages)-1
        if (index != self.last_index+1 and not hold_repeat) or not 0 <= index < len(self.program.stages):
            raise ValueError('carry stages cannot be reordered or replayed')
        raw = tuple(snapshot.positions_urad)
        if len(raw) != 12 or any(type(v) is not int for v in raw):
            raise ValueError('fresh twelve-axis measured snapshot required')
        measured = tuple(v/1e6 for v in raw)
        stage = self.program.stages[index]
        start = stage.points[0].positions_rad
        for i, (a, b) in enumerate(zip(measured, start)):
            if i == 5 and index >= 4:
                lo, hi = self.program.contact_control_interval
                if not lo-1e-6 <= a <= hi+1e-6 or abs(a-b) > .15:
                    raise ValueError('gripper left observed contact envelope')
                if abs(b-self.program.close_control_rad) > 1e-10:
                    raise ValueError('held gripper command changed')
            elif abs(a-b) > .02:
                raise ValueError('measured anchor changed after planning')
        if self.validate(stage, snapshot) is not True:
            raise ValueError('current-scene route validation failed')
        self.last_index = index
        return stage.points


class CarrySession:
    """Own the arm through HOLDING; completion never silently drops torque.

    Stop/cancel always calls the existing coordinated stop backend even between
    child routes. An unconfirmed stop retains ownership and cannot report success.
    A subsequent task must explicitly take over a held payload; this session
    never opens the fingers or resets a latched fault automatically.
    """
    def __init__(self, program, motion, stop, observe, clock, *, simulation=False,
                 observation_policy='continuous_patch',ready_for_route=None,pause_before_descent=False):
        if observation_policy not in ('continuous_patch', 'snapshot_then_verify'):
            raise ValueError('explicit supported observation policy required')
        if type(simulation) is not bool or (simulation and
                not getattr(getattr(motion, 'adapter', None), 'simulation_transport', False)):
            raise ValueError('simulation bypass requires an explicit in-memory transport')
        self.program, self.motion, self.stop = program, motion, stop
        self.observe, self.clock, self.simulation = observe, clock, simulation
        self.observation_policy = observation_policy
        self.ready_for_route = ready_for_route or (lambda: True)
        self.state = 'IDLE'
        self.reason = ''
        self.index = 0
        self.active = False
        self.history = []
        self.last_sequence = -1
        self.last_time = -1.
        self.last_feedback = None
        self.last_vision = -1
        self.held_since = None
        self.started_s = None
        self.phase_s = None
        self.goal = None
        self.hold_cycle = 0
        self.pause_before_descent=pause_before_descent
        self.descent_released=False
        self.pregrasp_hold_cycle=0

    def resume_descent(self):
        # Early input cannot pre-authorize a later checkpoint.
        if self.state!='AWAITING_DESCENT_CONFIRMATION':
            raise ValueError('descent confirmation requires the active pregrasp checkpoint')
        self.descent_released=True

    def _feedback(self, *, initial=False):
        f = self.observe()
        now = self.clock()
        if not isinstance(f, CarryFeedback):
            raise ValueError('independent carry feedback required')
        if f.observation_policy != self.observation_policy:
            raise ValueError('observation policy changed during execution')
        scalars = (now, f.observed_s, f.capture_observed_s, f.lift_m, *f.positions_rad)
        if (len(f.positions_rad) != 12 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                            or not math.isfinite(v) for v in scalars)
                or not 0 <= now-f.observed_s <= .2 or f.observed_s < self.last_time
                or not 0 <= f.capture_observed_s <= now
                or type(f.sequence) is not int or f.sequence < self.last_sequence):
            raise ValueError('stale or invalid joint feedback')
        if f.sequence == self.last_sequence and f != self.last_feedback:
            raise ValueError('feedback mutated without a new acquisition')
        if (f.capture_id != self.program.capture_id or f.calibration_id != self.program.calibration_id
                or f.source_sha256 != self.program.source_sha256):
            raise ValueError('capture or calibration binding changed')
        if (any(v is not True for v in (f.controller_ok, f.base_stopped, f.lift_stopped, f.scene_current))
                or abs(f.lift_m-self.program.planned_lift_m) > .003):
            raise ValueError('stationary scene or fixed lift invalid')
        if initial and not self.simulation:
            if self.program.hypothetical_lift:
                raise ValueError('hypothetical lift cannot authorize hardware')
            recently_captured = 0 <= now-f.capture_observed_s <= 2.
            # Planning may take longer than two seconds. A separate live
            # same-scene check can renew admission without restamping exposure.
            revalidated = (isinstance(f.scene_validated_s, (int,float))
                           and not isinstance(f.scene_validated_s, bool)
                           and math.isfinite(f.scene_validated_s)
                           and 0 <= now-f.scene_validated_s <= .5)
            if not recently_captured and not revalidated:
                raise ValueError('capture requires live scene revalidation before dispatch')
        if not self.simulation and (f.resident_profile_verified is not True or f.grip_cap_verified is not True):
            raise ValueError('resident firmware/profile and gripper output cap must be verified')
        if f.payload_relation not in ('unknown', 'contact', 'held', 'empty'):
            raise ValueError('unknown payload evidence vocabulary')
        self.last_sequence, self.last_time, self.last_feedback = f.sequence, f.observed_s, f
        return f

    def start(self, goal):
        if self.state != 'IDLE' or not isinstance(goal, str) or not goal:
            raise ValueError('new named carry session required')
        # Reject a hypothetical/stale scene before any command or ownership.
        self._feedback(initial=True)
        if not self.motion.ready():
            raise ValueError('resident motion port not prepared')
        self.goal = goal
        self.started_s = self.phase_s = self.clock()
        self.state = 'RUNNING'

    def _stop(self, reason, cancelled=False):
        if self.state in ('IDLE', 'FAILED', 'CANCELLED', 'STOPPING'):
            return
        self.reason = reason
        self.state = 'STOPPING'
        self.destination = 'CANCELLED' if cancelled else 'FAILED'
        try:
            self.stop.request(self.goal, True, self.clock())
        except Exception as exc:
            self.reason += ';stop_unconfirmed:'+str(exc)

    def cancel(self):
        self._stop('operator_cancelled', cancelled=True)

    def _contact(self, f):
        lo, hi = self.program.contact_control_interval
        return lo-1e-6 <= f.positions_rad[5] <= hi+1e-6

    def _held(self, f):
        return (self._contact(f) and f.payload_relation == 'held'
                and f.vision_capture_id == self.program.capture_id
                and f.vision_target_id == self.program.candidate_id
                and type(f.vision_sequence) is int and f.vision_sequence >= 0
                and math.isfinite(f.vision_s) and 0 <= self.clock()-f.vision_s <= .2
                and abs(f.vision_s-f.observed_s) <= .05)

    @property
    def holding_reason(self):
        # Retained arm/jaw goals do not prove that an object came with them.
        return ('HOLDING_UNVERIFIED' if self.observation_policy == 'snapshot_then_verify'
                else 'HOLDING')

    def _retention_allowed(self, f):
        if self.observation_policy == 'continuous_patch':
            return self._held(f)
        # Snapshot execution is a static-scene, supervised experiment. Encoders
        # guard the jaw envelope; grasp success remains pending independent
        # post-pick verification. An explicit negative result still rejects.
        return self._contact(f) and f.payload_relation != 'empty'

    def poll(self):
        if self.state in ('IDLE', 'FAILED', 'CANCELLED'):
            return PortResult(self.state, self.reason)
        try:
            if self.state != 'STOPPING':
                pending = getattr(self.motion, 'pending_stop_reason', lambda: None)()
                if pending:
                    raise ValueError(pending)
                self.motion.maintain()
                f = self._feedback()
                now = self.clock()
                if now < self.started_s:
                    raise ValueError('clock moved backwards')
                if self.state=='AWAITING_DESCENT_CONFIRMATION':
                    child=self.goal+'/pregrasp_hold/'+str(self.pregrasp_hold_cycle)
                    if not self.active:
                        if self.descent_released:
                            self.state='RUNNING';self.phase_s=now
                            return PortResult('RUNNING','DESCENT_CONFIRMED')
                        if not self.ready_for_route():return PortResult('RUNNING','WAIT_FRESH_PLATFORM')
                        self.motion.start(child,{'pregrasp_hold':True});self.active=True
                    result=self.motion.poll(child)
                    pending=getattr(self.motion,'pending_stop_reason',lambda:None)()
                    if pending:raise ValueError(pending)
                    if result.state in ('FAILED','CANCELLED'):raise ValueError(result.reason or 'pregrasp hold failed')
                    if result.state=='SUCCEEDED':
                        self.motion.retire_completed(child);self.pregrasp_hold_cycle+=1;self.active=False
                    return PortResult('RUNNING','WAIT_OPERATOR_HEIGHT_CHECK')
                if self.state == 'HOLDING':
                    if not self._retention_allowed(f):
                        raise ValueError('payload evidence lost during carry hold')
                    child = self.goal+'/hold/'+str(self.hold_cycle)
                    if not self.active:
                        if not self.ready_for_route():
                            return PortResult('RUNNING','WAIT_FRESH_PLATFORM')
                        self.motion.start(child,{'carry_stage':len(self.program.stages)-1})
                        self.active=True
                    result = self.motion.poll(child)
                    pending = getattr(self.motion, 'pending_stop_reason', lambda: None)()
                    if pending:
                        raise ValueError(pending)
                    if result.state in ('FAILED', 'CANCELLED'):
                        raise ValueError(result.reason or 'holding stream failed')
                    if result.state == 'SUCCEEDED':
                        self.motion.retire_completed(child)
                        self.hold_cycle += 1
                        self.active=False
                    return PortResult('RUNNING', self.holding_reason)
                stage = self.program.stages[self.index]
                child = self.goal+'/'+stage.name+('/0' if stage.name == 'hold' else '')
                if now-self.phase_s > stage.timing['duration_s']+4.:
                    raise ValueError('stage timeout:'+stage.name)
                if stage.attached and not self._contact(f):
                    raise ValueError('contact lost:'+stage.name)
                if stage.name in ('lift', 'carry', 'hold') and not self._retention_allowed(f):
                    raise ValueError('independent target-follow evidence lost:'+stage.name)
                if not self.active:
                    if not self.ready_for_route():
                        return PortResult('RUNNING','WAIT_FRESH_PLATFORM')
                    self.motion.start(child, {'carry_stage': self.index})
                    self.active = True
                result = self.motion.poll(child)
                pending = getattr(self.motion, 'pending_stop_reason', lambda: None)()
                if pending:
                    raise ValueError(pending)
                if result.state in ('FAILED', 'CANCELLED'):
                    raise ValueError(result.reason or 'motion failed')
                if stage.name == 'hold' and self.observation_policy == 'continuous_patch' and f.vision_sequence > self.last_vision:
                    self.last_vision = f.vision_sequence
                    self.held_since = now if self.held_since is None else self.held_since
                if result.state == 'SUCCEEDED':
                    if stage.name == 'close' and not self._contact(f):
                        raise ValueError('closure outside contact envelope')
                    if stage.name == 'probe' and not self._retention_allowed(f):
                        # Wait briefly at the probe endpoint, retaining its goal.
                        return PortResult('RUNNING', 'WAIT_TARGET_FOLLOW')
                    if stage.name == 'hold' and self.observation_policy == 'continuous_patch':
                        # Two seconds in the trajectory is not two seconds of
                        # independent retention evidence. Require advancing data.
                        if self.held_since is None or now-self.held_since < .5:
                            return PortResult('RUNNING', 'VERIFY_RETENTION')
                    self.history.append(dict(stage=stage.name, completed_s=now))
                    self.index += 1
                    self.active = False
                    self.phase_s = now
                    if stage.name=='pregrasp' and self.pause_before_descent:
                        self.state='AWAITING_DESCENT_CONFIRMATION'
                    if self.index == len(self.program.stages):
                        self.state = 'HOLDING'
                        self.active = True  # Retire the completed /hold/0 first.
        except Exception as exc:
            self._stop(str(exc))
        if self.state == 'STOPPING':
            try:
                self.stop.request(self.goal, True, self.clock())
                if self.stop.poll(self.goal, self.clock()).status == 'STOPPED':
                    self.state = self.destination
            except Exception:
                pass  # Missing acknowledgement is never proof of stopped motion.
        return PortResult('RUNNING' if self.state == 'HOLDING' else self.state,
                          self.holding_reason if self.state == 'HOLDING' else self.reason)
