"""One resident arm stream for planned motion and calibrated jaw commands."""
from .classical import route_for_arm
from .fetch import InvalidTask
from .navigation import number
from .manipulation_port import PlanningContext
from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint


class JawCalibration:
    """Measured monotonic angle/gap samples, with no extrapolation."""
    def __init__(self, samples, *, open_rad, close_rad):
        if not isinstance(samples, (tuple, list)) or not 2 <= len(samples) <= 64:
            raise InvalidTask("bounded measured jaw calibration required")
        self.samples = tuple((number(a, "jaw angle"), number(g, "jaw gap")) for a, g in samples)
        angles, gaps = zip(*self.samples)
        if (any(a >= b for a, b in zip(angles, angles[1:])) or min(gaps) < 0
                or not (all(a < b for a, b in zip(gaps, gaps[1:]))
                        or all(a > b for a, b in zip(gaps, gaps[1:])))):
            raise InvalidTask("strictly monotonic jaw angle/gap samples required")
        self.open = number(open_rad, "open angle")
        self.close = number(close_rad, "close angle")
        if self.gap(self.open) <= self.gap(self.close):
            raise InvalidTask("open jaw must have the larger measured gap")

    def gap(self, angle):
        angle = number(angle, "measured jaw angle")
        if not self.samples[0][0] <= angle <= self.samples[-1][0]:
            raise InvalidTask("jaw angle outside calibration")
        for (a, x), (b, y) in zip(self.samples, self.samples[1:]):
            if a <= angle <= b:
                return x + (y-x) * (angle-a) / (b-a)
        raise InvalidTask("jaw interpolation unavailable")


class ResidentRouteBuilder:
    """Uses a new measured snapshot for each route, preserving the other arm.

    The collision validator is required for generated joint/jaw routes and must
    check the whole path against the current scene. Planned routes already pass
    through MoveIt; both branches get the same final validation here.
    """
    def __init__(self, *, minimum_rad, maximum_rad, max_speed_rad_s,
                 max_acceleration_rad_s2, max_jerk_rad_s3, validate_route):
        if not callable(validate_route):
            raise InvalidTask("current-scene trajectory validator required")
        self.validate = validate_route
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
        if set(parameters) == {'trajectory', 'context'}:
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
        if any(abs(a-b) > .02 for a,b in zip(points[0].positions_rad,anchor)):
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
