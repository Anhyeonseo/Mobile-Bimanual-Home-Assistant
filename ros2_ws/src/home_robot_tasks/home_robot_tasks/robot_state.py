"""Measured robot state, exposure history and independent task/MCU conditions.

Producers must supply observations, never commanded positions or action results.
Joint envelopes and tolerances are explicit profile inputs, not physical defaults.
"""
from collections import deque
from dataclasses import dataclass
import math
from .execution import Feedback
from .fetch import InvalidTask
from .navigation import number
from .stationary_rgbd import MotionEvidence


@dataclass(frozen=True)
class RobotObservation:
    sequence: int
    boot_id: int
    observed_s: float
    xy_yaw: tuple
    base_velocity: tuple
    joints_rad: tuple
    lift_um: int
    lift_velocity: float
    healthy: bool
    localized: bool
    lift_homed: bool
    lift_supported: bool
    surface_valid: bool
    collision_scene_ready: bool
    arm_clear: bool
    reachable: bool = False
    collision_checked: bool = False
    checked_capture_id: str = ""
    checked_scene_revision: str = ""


class RobotStateStore:
    def __init__(self, profile, clock):
        fields = {'maximum_age_s','base_stopped_speed','lift_stopped_speed','joint_tolerance_rad',
                  'transport_joints_rad','lift_transport_um','lift_tolerance_um','hold_dwell_s',
                  'safe_minimum_rad','safe_maximum_rad','arrival_tolerance_m'}
        if not isinstance(profile, dict) or set(profile) != fields:
            raise InvalidTask('complete observation profile required')
        self.p = dict(profile); self.clock = clock
        for k in fields-{'transport_joints_rad','safe_minimum_rad','safe_maximum_rad'}:
            if number(profile[k], k) < 0: raise InvalidTask('negative observation bound')
        if not 0 < profile['maximum_age_s'] <= 2 or not 0 < profile['hold_dwell_s'] <= 2:
            raise InvalidTask('bounded observation age/dwell required')
        for k in ('transport_joints_rad','safe_minimum_rad','safe_maximum_rad'):
            if len(profile[k]) != 12: raise InvalidTask('twelve joint limits required')
            for v in profile[k]: number(v, k)
        if any(a >= b for a,b in zip(profile['safe_minimum_rad'],profile['safe_maximum_rad'])):
            raise InvalidTask('invalid safe joint envelope')
        self.latest = None; self.history = deque(maxlen=512)
        self.boot = None; self.fault = None; self.revision = 0
        self.since = self.hold_since = None; self.last_hold_height = None
        self.catalog = None
        self.lift_target_um = profile["lift_transport_um"]

    def observe(self, value):
        try:
            now = self.clock()
            if not isinstance(value, RobotObservation): raise InvalidTask('robot observation required')
            if type(value.sequence) is not int or not 0 < value.sequence <= 0xFFFFFFFF:
                raise InvalidTask('invalid sensor sequence')
            if type(value.boot_id) is not int or not 0 < value.boot_id <= 0xFFFFFFFF:
                raise InvalidTask('invalid boot identity')
            if self.boot is not None and value.boot_id != self.boot: raise InvalidTask('device restarted')
            if not 0 <= number(value.observed_s,'sensor time') <= now or now-value.observed_s > self.p['maximum_age_s']:
                raise InvalidTask('stale robot observation')
            if self.latest and (value.sequence <= self.latest.sequence or value.observed_s <= self.latest.observed_s):
                raise InvalidTask('repeated robot observation')
            for k,n in (('xy_yaw',3),('base_velocity',3),('joints_rad',12)):
                if len(getattr(value,k)) != n: raise InvalidTask('invalid robot vector')
                for v in getattr(value,k): number(v,k)
            if type(value.lift_um) is not int or not 0 <= value.lift_um <= 1000000:
                raise InvalidTask('invalid lift height')
            number(value.lift_velocity,'lift speed')
            for k in ('healthy','localized','lift_homed','lift_supported','surface_valid','collision_scene_ready','arm_clear','reachable','collision_checked'):
                if type(getattr(value,k)) is not bool: raise InvalidTask('typed measured conditions required')
            if any(not isinstance(getattr(value, k), str) for k in ('checked_capture_id', 'checked_scene_revision')):
                raise InvalidTask('planning proof identities required')
            stopped = all(abs(v) <= self.p['base_stopped_speed'] for v in value.base_velocity) and abs(value.lift_velocity) <= self.p['lift_stopped_speed']
            moved = self.latest and (any(abs(a-b)>1e-6 for a,b in zip(value.xy_yaw,self.latest.xy_yaw)) or value.lift_um != self.latest.lift_um)
            gap = self.latest and value.observed_s-self.latest.observed_s > self.p['maximum_age_s']
            if not stopped or moved or gap:
                self.revision += 1; self.since = None
            # Base motion invalidates camera pose, not a measured stationary lift.
            if gap: self.hold_since = None
            if stopped and self.since is None: self.since = value.observed_s
            if not value.lift_supported or not value.lift_homed or abs(value.lift_velocity)>self.p['lift_stopped_speed']:
                self.hold_since = None
            elif self.hold_since is None:
                self.hold_since, self.last_hold_height = value.observed_s, value.lift_um
            elif abs(value.lift_um-self.last_hold_height)>self.p['lift_tolerance_um']:
                self.hold_since, self.last_hold_height = value.observed_s, value.lift_um
            self.boot, self.latest = value.boot_id, value
            self.history.append(MotionEvidence(value.observed_s, self.since if self.since is not None else value.observed_s,
                f'pose-{self.revision}', all(abs(v)<=self.p['base_stopped_speed'] for v in value.base_velocity),
                abs(value.lift_velocity)<=self.p['lift_stopped_speed']))
        except Exception as error:
            self.fault = str(error)
            raise

    def fresh(self):
        return self.fault is None and self.latest is not None and 0 <= self.clock()-self.latest.observed_s <= self.p['maximum_age_s']

    def motion_at(self, stamp):
        if not self.fresh(): raise InvalidTask('robot state unavailable')
        for value in reversed(self.history):
            if value.observed_s <= stamp:
                if stamp-value.observed_s > self.p['maximum_age_s']: break
                return value
        raise InvalidTask('exposure motion history unavailable')

    def motion(self):
        return self.motion_at(self.clock())

    def conditions(self):
        if not self.fresh(): return {}
        v, p = self.latest, self.p
        safe = all(a <= j <= b for a,j,b in zip(p['safe_minimum_rad'],v.joints_rad,p['safe_maximum_rad']))
        arm_transport = all(abs(a-b)<=p['joint_tolerance_rad'] for a,b in zip(v.joints_rad,p['transport_joints_rad']))
        arrival = False
        if self.catalog and self.catalog.navigation_goal:
            g = self.catalog.navigation_goal
            arrival = v.localized and math.hypot(v.xy_yaw[0]-g['x'],v.xy_yaw[1]-g['y']) <= p['arrival_tolerance_m'] and abs(math.atan2(math.sin(v.xy_yaw[2]-g['yaw_rad']),math.cos(v.xy_yaw[2]-g['yaw_rad']))) <= .1
        target, surface = False, False
        if self.catalog:
            try: self.catalog.checked_target(); target = True
            except InvalidTask: pass
            try: self.catalog.checked_target(True); surface = True
            except InvalidTask: pass
        # Scene availability does not prove IK reachability or collision freedom.
        # Bind independent planning evidence to a still-valid observed target.
        checked = False
        if self.catalog and v.collision_scene_ready:
            for placement in (False, True):
                try:
                    candidate = self.catalog.checked_target(placement)
                    checked |= (v.checked_capture_id == candidate['capture_id'] and
                                v.checked_scene_revision == candidate['scene_revision'])
                except InvalidTask:
                    pass
        return dict(hardware_ready=v.healthy, localized=v.localized,
            base_stopped=self.motion().base_stopped,lift_stopped=self.motion().lift_stopped,
            arm_in_transport_pose=arm_transport,lift_in_transport_position=abs(v.lift_um-p['lift_transport_um'])<=p['lift_tolerance_um'],
            arms_safe_for_lift=safe,arms_safe_for_alignment=safe,lift_homed=v.lift_homed,
            lift_hold_verified=self.hold_since is not None and v.observed_s-self.hold_since>=p['hold_dwell_s'],
            localized_in_source_room=arrival,localized_at_destination=arrival,localized_at_view=arrival,
            alignment_verified=arrival,base_control_available=v.healthy,view_reached=self.motion().lift_stopped and abs(v.lift_um-self.lift_target_um)<=p["lift_tolerance_um"],
            lift_height_verified=self.motion().lift_stopped and abs(v.lift_um-self.lift_target_um)<=p["lift_tolerance_um"],target_observed=target,target_pose_resolved=target,
            fresh_target_pose=target,fresh_observation=target or surface,fresh_work_surface=v.surface_valid,
            placement_surface_valid=surface and v.surface_valid,transform_valid=target or surface,
            reachable=checked and v.reachable,collision_checked=checked and v.collision_checked,arm_clear=v.arm_clear,
            object_at_destination=bool(surface and self.catalog.surface.get('object_at_destination') is True))

    def __call__(self, goal_id, step, now):
        return Feedback(goal_id, self.latest.observed_s if self.latest else now, 'READY', self.conditions())


class EvidencePublisher:
    def __init__(self, state, payload, client, clock):
        self.state,self.payload,self.client,self.clock = state,payload,client,clock
        self.last = None

    def __call__(self):
        c = self.state.conditions(); proof = self.payload.proof(self.clock())
        if not c:
            raise InvalidTask('fresh robot and independent payload state required')
        stamp = min(self.state.latest.observed_s, proof.observed_s)
        if stamp == self.last: return  # Repeated timer calls never refresh cached sensors.
        self.client.publish(stamp, arms_safe=c['arms_safe_for_lift'],
            lift_hold=c['lift_hold_verified'], payload_safe=proof.conditions['payload_safe'])
        self.last = stamp
