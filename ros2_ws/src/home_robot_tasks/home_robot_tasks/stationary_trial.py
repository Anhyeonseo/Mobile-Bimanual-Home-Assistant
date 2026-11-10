"""Synthetic contact and carried-object witnesses for the actual PickPlacePort state machine."""
from dataclasses import asdict

from .execution import Feedback
from .pick_place import PickPlacePort
from .jaw_closure import next_gap, target_gap
from .payload_monitor import GripMeasurement, PayloadMonitor, PayloadObservation
from .probe_evidence import LiftSample, lift_relation
from .skill_ports import PortResult
import numpy as np


class TrialWorld:
    def __init__(self,arm,attempt,width,object_xyz,radius,fault,payload=None,*,guarded=False):
        self.arm=arm;self.q=np.array(attempt['anchor_rad']);self.width=width;self.radius=radius
        self.object=np.array(object_xyz);self.initial=self.object.copy()
        self.fault=fault
        self.payload=payload
        self.nodes={x['stage']:x for x in attempt['stages']}
        self.open=True;self.held=self.attached=self.released=False
        self.support_top=None;self.otp=None
        self.guarded = guarded
        self.probe = attempt.get('test_lift')
        self.closure = attempt.get('jaw_closure')
        if self.closure and self.closure['status'] != 'BOUNDED_CLOSE':
            self.closure = None  # Historical untuned virtual fixtures retain their original gripper model.
        self.close_decisions = []
        self.measured_gap = None
        if self.closure:
            self.width = self.measured_gap = self.closure['entry_gap_m']
        self.start_sample = self.probe_observation = self.probe_reason = None
        self.followed = self.pad_contact = False
        self.sequence = 0
        self.sensor_time = self.sensor_facts = None
        if guarded:
            # Synthetic bands only. The live observer has no calibrated effort/gap thresholds.
            bands = (1.,60.,65.) if self.closure is None else (
                target_gap(self.closure)*1000,
                self.closure['upper_contact_gap_m']*1000, self.width*1000)
            self.monitor = PayloadMonitor(gap_min_mm=bands[0], gap_max_mm=bands[1], open_gap_mm=bands[2],
                effort_min_raw=10, effort_max_raw=80, empty_effort_max_raw=5,
                max_age_s=.25, max_skew_s=.05, dwell_s=.08)
            self.monitor.expect('grasp', 'fixture-target', 0.)
        self.high=self.object[2];self.now=0.;self.last_motion=None;self.trace=[]

    def step(self,stage):
        if stage in self.nodes or stage == 'pick_probe':
            if stage == 'pick_probe':
                if not self.guarded or not self.probe or self.start_sample is not None:
                    raise ValueError('checked one-shot probe required')
                self.start_sample = self._sample()
                self.q = np.array(self.probe['model_rad'])
            else:
                self.q = np.array(self.nodes[stage]['model_rad'])
            self.last_motion=stage
            if self.held:
                if self.fault=='lost_load' and stage=='pick_retreat':
                    self.held=False
                else:
                    pose=self.arm.fk(self.q)[0]
                    self.object=pose[:3,3]+pose[:3,:3]@self.otp
                self.high=max(self.high,self.object[2])
        elif stage=='pick_close':
            pose=self.arm.fk(self.q)[0]
            delta=pose[:3,:3].T@(self.object-pose[:3,3])
            half_width = self.radius
            if self.payload is not None:
                jaw_x = pose[:3, 0]
                half_width = np.abs(jaw_x @ self.payload.base_pose[:3, :3]) @ self.payload.half_extents
            jaw_midpoint_x = -self.arm.jaw.center_from_fixed_m(self.width)
            inside = bool(abs(delta[0] - jaw_midpoint_x) + half_width <= self.width/2
                      and np.max(np.abs(delta[1:])) <= .012 and self.fault != 'empty_close')
            self.pad_contact = bool(inside)
            if self.closure:
                blocked_gap = 2*half_width if inside else None
                if self.fault in ('table_contact', 'one_tip_contact'):
                    blocked_gap = self.closure['visible_span_m']
                elif self.fault == 'early_resistance':
                    blocked_gap = (self.width + self.closure['upper_contact_gap_m']) / 2
                self.pad_contact = self._close_on_section(blocked_gap)
            self.held = self.pad_contact and inside and self.fault not in ('table_contact', 'one_tip_contact')
            self.otp=delta
            self.open=False
        elif stage=='pick_attach':
            self.attached=self.held and (not self.guarded or self.followed)
        elif stage=='place_open':
            self.open=True;self.released=self.fault!='stuck_release';self.held=not self.released
            if self.released:self.object[2]=self.support_top+self.radius
        elif stage=='place_detach':
            if self.released:self.attached=False
        elif stage=='pick_open':self.open=True
        else:raise ValueError('unexpected stage: '+stage)
        self.trace.append(dict(stage=stage,object_xyz_m=self.object.tolist(),held=self.held,
                               attached=self.attached,released=self.released))

    def _close_on_section(self, obstacle_width):
        gap = self.measured_gap
        rising = False
        # This fixture has an independent known width. Only the feedback below sees it;
        # the planner never receives the obstacle dimensions or the object's name.
        for _ in range(100):
            blocked = obstacle_width is not None and gap <= obstacle_width + 1e-9
            effort = ('persistent' if rising else 'rising') if blocked else 'free'
            action, target = next_gap(self.closure, gap, effort)
            self.close_decisions.append(dict(gap_m=gap, action=action, next_gap_m=target,
                                             target_gap_m=target_gap(self.closure)))
            if action == 'WAIT_AT_CURRENT_GAP':
                rising = True
            elif action == 'CLOSE_STEP':
                gap = target if obstacle_width is None else max(target, obstacle_width)
                self.measured_gap = float(gap)
            else:
                return action == 'CONTACT_CANDIDATE'
        raise ValueError('unbounded virtual closure')

    def _sample(self):
        return LiftSample('virtual-base', 'fixture-target', 'center', self.now, self.now,
                          tuple(self.arm.fk(self.q)[0][:3, 3]), tuple(self.object), 0.)

    def _guarded_feedback(self, phase):
        if self.sensor_time == self.now:
            return self.sensor_facts
        relation = 'empty' if self.open else 'unknown'
        follow = False
        # For the one-tip fixture the motor reports a plausible stop but the
        # independent view cannot place the target between both contact faces.
        if self.pad_contact and self.fault != 'one_tip_contact':
            if self.start_sample is None:
                relation = 'contact'
            elif self.last_motion == 'pick_probe':
                observation = None if self.fault == 'occluded_probe' else self._sample()
                relation, reason = lift_relation(self.start_sample, observation, self.now)
                if phase == 'pick_probe':
                    self.probe_observation, self.probe_reason = observation, reason
                follow = relation == 'held'
            elif self.followed and self.held:
                relation = 'held'
        # Grip and vision are independent simulated channels. A mechanical stall can load the
        # pads without translating the observed target, and a scene attachment can outlive a slip.
        self.sequence += 1
        if self.closure:
            gap = self.measured_gap * 1000
        else:
            gap = self.width * 1000 if self.pad_contact else 0.
            if self.open:
                gap = 65.
        effort = 40. if self.pad_contact and (self.held or self.fault in ('table_contact', 'one_tip_contact')) else 0.
        self.monitor.observe_grip(GripMeasurement(self.sequence, self.now, gap, effort, True), self.now)
        image = PayloadObservation(self.sequence, self.now, 'fixture-target', relation)
        self.monitor.observe_vision(image, self.now)
        facts = self.monitor.proof(self.now).conditions
        if follow and facts['grasp_verified']:
            self.followed = True
        self.sensor_time = self.now
        self.sensor_facts = {**facts, 'probe_clearance_verified': bool(self.probe),
                             'probe_follow_verified': follow and self.followed}
        return self.sensor_facts

    def feedback(self,goal,phase,now):
        facts=dict(hardware_ready=True,base_stopped=True,lift_stopped=True,gripper_empty=not self.held,
                   gripper_open=self.open,grasp_verified=self.held,load_retained=self.held,
                   object_attached=self.attached,object_detached=not self.attached,
                   release_verified=self.released,arm_clear=self.last_motion in ('pick_retreat','place_retreat'))
        if self.guarded:
            facts.update(self._guarded_feedback(phase))
        return Feedback(goal,self.now,'READY',facts)


class VirtualStagePort:
    def __init__(self,world):self.world=world;self.states={}
    def ready(self):return True
    def start(self,goal,params):
        self.world.step(params['stage']);self.states[goal]='SUCCEEDED'
    def poll(self,goal):return PortResult(self.states[goal])
    def cancel(self,goal):self.states[goal]='CANCELLED'
    def phase_safe(self,goal):return True


class VirtualProbePort(VirtualStagePort):
    def start(self, goal, params):
        if params != {'arm': self.world.arm.side, 'object_id': 'fixture-target'}:
            raise ValueError('test lift is bound to the selected arm and target')
        self.world.step('pick_probe')
        self.states[goal] = 'SUCCEEDED'


def _start_world(arm, plan, width, object_xyz, radius, fault, payload=None):
    if plan['mode'] != 'nominal_stationary_simulation' or plan['status'] != 'VIRTUAL_PATH_FOUND':
        raise ValueError('simulation-only path required')
    guarded = plan.get('task') == 'pick_lift'
    world = TrialWorld(arm, plan['attempts'][-1], width, object_xyz, radius, fault, payload, guarded=guarded)
    port = PickPlacePort(*(VirtualStagePort(world) for _ in range(3)),
                         resolve_stage=lambda stage, params, tick: {'stage': stage},
                         monitor=world.feedback, clock=lambda: world.now,
                         probe=VirtualProbePort(world) if guarded else None,
                         motion_timeout_s=2, gripper_timeout_s=1, scene_timeout_s=1)
    return world, port


def _run_operation(port, world, operation):
    port.start(operation, dict(operation=operation, arm=world.arm.side, object_id='fixture-target'))
    for _ in range(120):
        world.now += .1
        if world.fault == 'cancel' and world.last_motion == 'pick_pregrasp':
            port.cancel(operation)
        outcome = port.poll(operation)
        if outcome.state != 'RUNNING':
            return outcome
    port.cancel(operation)
    return PortResult('FAILED', 'SIMULATION_POLL_LIMIT')


def _sample_hold(world, minimum_rise):
    """Observe the virtual object's height/contact at lift completion and during a 0.5 s dwell."""
    samples = []
    for tick in range(6):
        if tick:
            world.now += .1
        if tick == 3 and world.fault == 'dropped_during_hold':
            # Physical contact can disappear even if the planning scene still says attached.
            world.held = False
            world.object[2] = world.initial[2]
        rise = float(world.object[2] - world.initial[2])
        samples.append(dict(elapsed_s=tick / 10, rise_m=rise, held=world.held, attached=world.attached))
        if not world.held or not world.attached or world.open:
            return samples, 'LOAD_LOST_DURING_HOLD'
        if rise < minimum_rise:
            return samples, 'INSUFFICIENT_SYNTHETIC_RISE'
    return samples, None


def simulate_pick(arm, plan, width, object_xyz, *, radius=.009, fault=None, minimum_rise=.012, payload=None):
    """Finish with the fixture in the closed gripper; never dispatch a release or place phase."""
    if plan.get('task') != 'pick_lift':
        raise ValueError('pick-and-lift path required')
    world, port = _start_world(arm, plan, width, object_xyz, radius, fault, payload)
    outcome = _run_operation(port, world, 'pick')
    samples = []
    if outcome.state == 'SUCCEEDED':
        samples, reason = _sample_hold(world, minimum_rise)
        status = reason or 'SIMULATED_PICK_HELD'
    else:
        status, reason = 'STOPPED_PICK', outcome.reason
    return dict(status=status, reason=reason, trace=world.trace, hold_samples=samples,
                jaw_closure=world.close_decisions,
                test_lift=dict(plan=world.probe, outcome=world.probe_reason, verified=world.followed,
                               baseline=asdict(world.start_sample) if world.start_sample else None,
                               observation=asdict(world.probe_observation) if world.probe_observation else None),
                final_rise_m=float(world.object[2] - world.initial[2]), minimum_rise_m=minimum_rise,
                retained_at_end=world.held and world.attached and not world.open,
                motion_authorized=False, hardware_commands=0, physical_task_completed=False)


def simulate_trial(arm,plan,width,object_xyz,destination,*,radius=.009,fault=None,minimum_rise=.012):
    world, port = _start_world(arm, plan, width, object_xyz, radius, fault)
    world.support_top=plan['support_top_m']
    reason=None
    status='SIMULATED_SEQUENCE_COMPLETED'
    for operation in ('pick','place'):
        outcome = _run_operation(port, world, operation)
        if outcome.state!='SUCCEEDED':
            status='STOPPED_'+operation.upper();reason=outcome.reason;break
        if operation=='pick' and world.high-world.initial[2]<minimum_rise:
            status='INSUFFICIENT_SYNTHETIC_RISE';break
    lo,hi=np.array(plan['support_box_m'])
    supported=(np.all(world.object[:2]-radius>=lo[:2]) and np.all(world.object[:2]+radius<=hi[:2])
               and abs(world.object[2]-radius-hi[2])<=1e-9)
    if status=='SIMULATED_SEQUENCE_COMPLETED' and (not world.released or world.attached or not supported
            or np.linalg.norm(world.object-destination)>.012):
        status='PLACEMENT_UNCONFIRMED'
    h=plan['place_xy_half_extent_m']
    if h and np.any(np.abs(world.object[:2]-plan['requested_place_xyz_m'][:2])>h):
        if status=='SIMULATED_SEQUENCE_COMPLETED':status='OUTSIDE_PLACE_REGION'
    return dict(status=status,reason=reason,trace=world.trace,supported_at_end=bool(supported),simulated_rise_m=float(world.high-world.initial[2]),
                placed_error_m=float(np.linalg.norm(world.object-destination)),
                motion_authorized=False,hardware_commands=0,physical_task_completed=False)
