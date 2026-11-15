"""One resident arm stream for planned motion and calibrated jaw commands."""
from dataclasses import dataclass
from .classical import route_for_arm
from .fetch import InvalidTask
from .navigation import number
from .manipulation_port import PlanningContext
from .jaw_calibration import JawCalibration
from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint


@dataclass(frozen=True)
class ContactEnvelope:
    axis: int
    minimum_rad: float
    maximum_rad: float
    command_rad: float
    object_id: str
    capture_id: str
    release_goal_rad: float | None = None

    def __post_init__(self):
        if self.axis not in (5,11) or not self.object_id or not self.capture_id:
            raise InvalidTask('identified installed gripper contact required')
        for v in (self.minimum_rad,self.maximum_rad,self.command_rad):number(v,'contact bound')
        if not self.minimum_rad<=self.maximum_rad or self.maximum_rad-self.minimum_rad>.3:
            raise InvalidTask('bounded contact interval required')
        if self.release_goal_rad is not None:number(self.release_goal_rad,'release goal')
        if not self.minimum_rad-.15<=self.command_rad<=self.maximum_rad+.15:
            raise InvalidTask('bounded pressure command required')


class ResidentRouteBuilder:
    """Uses a new measured snapshot for each route, preserving the other arm.

    The collision validator is required for generated joint/jaw routes and must
    check the whole path against the current scene. Planned routes already pass
    through MoveIt; both branches get the same final validation here.
    """
    def __init__(self, *, minimum_rad, maximum_rad, max_speed_rad_s,
                 max_acceleration_rad_s2, max_jerk_rad_s3, validate_route, contact_guard=None):
        if not callable(validate_route):
            raise InvalidTask("current-scene trajectory validator required")
        self.validate = validate_route
        self.contact_guard = contact_guard
        self.limits = dict(minimum_rad=tuple(minimum_rad), maximum_rad=tuple(maximum_rad),
            max_speed_rad_s=tuple(max_speed_rad_s), max_acceleration_rad_s2=tuple(max_acceleration_rad_s2),
            max_jerk_rad_s3=tuple(max_jerk_rad_s3))
        if any(len(v) != 12 for v in self.limits.values()):
            raise InvalidTask("twelve commissioned route bounds required")
        for values in self.limits.values():
            for v in values: number(v, "route bound")
        if (any(a >= b for a,b in zip(self.limits['minimum_rad'], self.limits['maximum_rad']))
                or any(v <= 0 for key, values in self.limits.items() if key.startswith('max_') for v in values)):
            raise InvalidTask("invalid route bounds")

    def __call__(self, parameters, snapshot):
        raw = snapshot.positions_urad
        if len(raw) != 12 or any(type(v) is not int for v in raw):
            raise InvalidTask("measured twelve-axis snapshot required")
        anchor = tuple(v / 1e6 for v in raw)
        contact=parameters.get('contact_envelope')
        if contact is not None:
            if (not isinstance(contact,ContactEnvelope) or not callable(self.contact_guard)
                    or self.contact_guard(contact,snapshot) is not True
                    or not contact.minimum_rad-1e-6<=anchor[contact.axis]<=contact.maximum_rad+1e-6):
                raise InvalidTask('live gripper contact envelope required')
        if set(parameters) in ({'trajectory','context'},{'trajectory','context','contact_envelope'}):
            c = parameters['context']
            if not isinstance(c, PlanningContext) or len(c.anchor_rad) != 12:
                raise InvalidTask("planning context required")
            if any(abs(a-b) > .02 for a,b in zip(anchor, c.anchor_rad)):
                raise InvalidTask("measured anchor moved after planning")
            points = tuple(parameters['trajectory'])
        elif set(parameters) == {'arm', 'steps'}:
            arm = parameters['arm']
            if arm not in ('left', 'right'): raise InvalidTask("named arm required")
            other = 6 if arm == 'left' else 0
            route = route_for_arm(anchor, anchor[other:other+6], arm, parameters['steps'], **self.limits)
            points = tuple(TimedJointPoint(p['offset_ms'], p['positions_rad']) for p in route)
        else:
            raise InvalidTask("explicit planned trajectory or joint steps required")
        if not 2 <= len(points) <= 20000:
            raise InvalidTask("bounded resident route required")
        previous = None
        for p in points:
            if (not isinstance(p, TimedJointPoint) or type(p.offset_ms) is not int or p.offset_ms <= 0
                    or p.offset_ms % 5 or len(p.positions_rad) != 12):
                raise InvalidTask("invalid resident point")
            for lo,v,hi in zip(self.limits['minimum_rad'],p.positions_rad,self.limits['maximum_rad']):
                if not lo <= number(v, 'joint') <= hi: raise InvalidTask("joint limit")
            if previous:
                dt = (p.offset_ms - previous.offset_ms) / 1000
                if dt <= 0 or any(abs(a-b)/dt > v+1e-6 for a,b,v in zip(
                        p.positions_rad,previous.positions_rad,self.limits['max_speed_rad_s'])):
                    raise InvalidTask("route time or speed bound")
            previous = p
        if contact is not None:
            values=[p.positions_rad[contact.axis] for p in points]
            if contact.release_goal_rad is None:
                if any(abs(v-contact.command_rad)>1e-6 for v in values):
                    raise InvalidTask('held pressure must remain constant')
            else:
                delta=contact.release_goal_rad-contact.command_rad
                if abs(delta)<1e-6:raise InvalidTask('nonzero calibrated release required')
                progress=[(v-contact.command_rad)/delta for v in values]
                if (abs(progress[0])>1e-5 or abs(progress[-1]-1)>1e-5
                        or any(v< -1e-5 or v>1+1e-5 for v in progress)
                        or any(b<a-1e-6 for a,b in zip(progress,progress[1:]))
                        or any(abs(p.positions_rad[i]-points[0].positions_rad[i])>1e-6
                               for p in points for i in range(12) if i!=contact.axis)):
                    raise InvalidTask('release must only open the supported gripper monotonically')
        if any(abs(a-b) > (.15 if contact is not None and i==contact.axis else .02)
               for i,(a,b) in enumerate(zip(points[0].positions_rad,anchor))):
            raise InvalidTask("route does not start at measured anchor")
        if self.validate(points, snapshot) is not True:
            raise InvalidTask("current-scene route validation failed")
        return points


class SharedGripperPort:
    """Delegates to the SAME ResidentArmPort; no second writer or serial client."""
    def __init__(self, arm_port, calibrations):
        if set(calibrations) != {'left', 'right'} or any(not isinstance(v, JawCalibration) for v in calibrations.values()):
            raise InvalidTask("two calibrated grippers required")
        self.arm, self.calibrations = arm_port, dict(calibrations)

    def ready(self): return self.arm.ready()

    def start(self, goal, parameters):
        if set(parameters) != {'arm', 'action'} or parameters['arm'] not in self.calibrations or parameters['action'] not in ('open','close'):
            raise InvalidTask("explicit gripper arm/action required")
        calibration = self.calibrations[parameters['arm']]
        value = calibration.open if parameters['action'] == 'open' else calibration.close
        self.arm.start(goal, {'arm':parameters['arm'], 'steps':[{'kind':'gripper','target_position_rad':value}]})

    def poll(self, goal): return self.arm.poll(goal)
    def cancel(self, goal): self.arm.cancel(goal)
