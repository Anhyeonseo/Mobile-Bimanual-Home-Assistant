"""Hardware-free three-wheel omni geometry and measured-velocity odometry.

Configuration describes each wheel's rolling direction and centre in base XY.
No AlohaMini dimensions, motor signs or servo speed scale are assumed.
"""
from dataclasses import dataclass
import math
from .navigation import number
from .fetch import InvalidTask
from .replay import _seconds


@dataclass(frozen=True)
class Wheel:
    x_m: float
    y_m: float
    rolling_angle_rad: float
    radius_m: float


def _cross(a,b):
    return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])

def _dot(a,b): return sum(x*y for x,y in zip(a,b,strict=True))

def _vector(values):
    if not isinstance(values,(tuple,list)) or len(values)!=3:
        raise InvalidTask('three motion components required')
    return tuple(number(v,'motion component') for v in values)


class OmniKinematics:
    def __init__(self, wheels: tuple[Wheel,Wheel,Wheel]):
        if len(wheels)!=3:
            raise InvalidTask('three wheels required')
        rows=[]
        for wheel in wheels:
            x,y,angle,radius=(number(v,'wheel geometry') for v in
                              (wheel.x_m,wheel.y_m,wheel.rolling_angle_rad,wheel.radius_m))
            if not 1e-6 <= radius <= 10:
                raise InvalidTask('wheel radius outside supported scale')
            tx,ty=math.cos(angle),math.sin(angle)
            rows.append((tx/radius,ty/radius,(-tx*y+ty*x)/radius))
        determinant=_dot(rows[0],_cross(rows[1],rows[2]))
        scale=math.prod(math.sqrt(_dot(row,row)) for row in rows)
        if scale==0 or abs(determinant)/scale<1e-6:
            raise InvalidTask('singular wheel geometry')
        self.rows=tuple(rows)
        self.inverse_columns=tuple(tuple(v/determinant for v in column) for column in
                                   (_cross(rows[1],rows[2]),_cross(rows[2],rows[0]),_cross(rows[0],rows[1])))

    def wheel_rates(self, twist, max_wheel_rad_s=None):
        """vx/vy m/s, yaw rate rad/s -> three wheel rad/s; common saturation."""
        values=_vector(twist)
        rates=tuple(_dot(row,values) for row in self.rows)
        scale=1.0
        if max_wheel_rad_s is not None:
            limits=_vector(max_wheel_rad_s)
            if any(limit<=0 for limit in limits):
                raise InvalidTask('positive wheel speed limits required')
            scale=max(1.0,*(abs(rate)/limit for rate,limit in zip(rates,limits,strict=True)))
        return tuple(rate/scale for rate in rates)

    def body_twist(self, measured_wheel_rad_s):
        values=_vector(measured_wheel_rad_s)
        return tuple(sum(self.inverse_columns[j][i]*values[j] for j in range(3)) for i in range(3))


class WheelOdometry:
    """SE(2) integration with previous measured velocity held between samples.

Caller supplies synchronized acquisition time, never command echo. A gap
invalidates the integration baseline and cannot invent motion over missing data.
This estimate has wheel-slip error; localization/covariance are separate work.
"""
    def __init__(self, geometry: OmniKinematics, *, max_gap_s: float):
        self.geometry=geometry
        self.max_gap=_seconds(max_gap_s,'max_gap_s')
        if not self.max_gap:
            raise InvalidTask('positive observation gap required')
        self.pose=(0.0,0.0,0.0)
        self.previous=None
        self.discontinuities=0
        self.last_stamp=None

    def observe(self, measured_wheel_rad_s, observed_s: float):
        stamp=_seconds(observed_s,'observed_s')
        twist=self.geometry.body_twist(measured_wheel_rad_s)
        if self.last_stamp is not None and stamp<=self.last_stamp:
            raise InvalidTask("odometry observation must advance")
        if self.previous is None:
            self.previous=(stamp,twist)
            self.last_stamp=stamp
            return self.pose
        dt=stamp-self.previous[0]
        if dt<=0:
            raise InvalidTask('odometry observation must advance')
        self.last_stamp=stamp
        if dt>self.max_gap:
            self.previous=None
            self.discontinuities+=1
            raise InvalidTask('odometry_feedback_gap')
        vx,vy,wz=self.previous[1]
        theta=wz*dt
        if abs(theta)<1e-8:
            a,b=dt,0.5*wz*dt*dt
        else:
            a,b=math.sin(theta)/wz,(1-math.cos(theta))/wz
        dx,dy=a*vx-b*vy,b*vx+a*vy
        x,y,yaw=self.pose
        self.pose=(x+math.cos(yaw)*dx-math.sin(yaw)*dy,
                   y+math.sin(yaw)*dx+math.cos(yaw)*dy,
                   (yaw+theta+math.pi)%(2*math.pi)-math.pi)
        self.previous=(stamp,twist)
        return self.pose
