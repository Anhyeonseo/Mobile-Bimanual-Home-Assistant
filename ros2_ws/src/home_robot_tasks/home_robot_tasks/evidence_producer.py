"""Measured source fusion; no action result or command can stand in for input.

Input clocks have already been mapped into the local monotonic time domain.
The oldest source timestamp is retained. Polling never refreshes old samples.
"""
from copy import deepcopy
from dataclasses import dataclass
from .fetch import InvalidTask
from .navigation import number
from .robot_state import RobotObservation
from .payload_monitor import GripMeasurement


@dataclass(frozen=True)
class SourceSample:
    sequence: int
    observed_s: float
    boot_id: int
    profile_sha256: str
    values: dict


FIELDS = {
    'arm': {'joints_rad','healthy'},
    'base': {'base_velocity'},
    'localization': {'xy_yaw','localized'},
    'lift': {'lift_um','lift_velocity','lift_homed','lift_supported','healthy'},
    'scene': {'surface_valid','collision_scene_ready','arm_clear','reachable','collision_checked',
              'checked_capture_id','checked_scene_revision'},
}
BOOLEANS = {'healthy','localized','lift_homed','lift_supported','surface_valid',
            'collision_scene_ready','arm_clear','reachable','collision_checked'}


class EvidenceProducer:
    def __init__(self, clock, *, boot_id, profile_sha256, maximum_age_s, maximum_skew_s):
        if type(boot_id) is not int or not 0 < boot_id <= 0xffffffff:
            raise InvalidTask("explicit device boot identity required")
        if (not isinstance(profile_sha256,str) or len(profile_sha256)!=64
                or any(c not in '0123456789abcdef' for c in profile_sha256)):
            raise InvalidTask("profile SHA-256 required")
        self.age, self.skew = number(maximum_age_s,'age'), number(maximum_skew_s,'skew')
        if not 0 < self.skew <= self.age <= 2: raise InvalidTask("bounded observation timing required")
        self.clock,self.boot,self.profile = clock,boot_id,profile_sha256
        self.samples = {}; self.sequence = 0; self.emitted_s = None; self.fault = None

    def observe(self, source, sample):
        try:
            if self.fault: raise InvalidTask(self.fault)
            if source not in FIELDS or not isinstance(sample,SourceSample) or set(sample.values)!=FIELDS[source]:
                raise InvalidTask("explicit measured source fields required")
            now = number(self.clock(),'clock');stamp=number(sample.observed_s,'observation')
            previous=self.samples.get(source)
            if (sample.boot_id!=self.boot or sample.profile_sha256!=self.profile
                    or type(sample.sequence) is not int or not 0 < sample.sequence <= 0xffffffff
                    or not 0 <= stamp <= now or now-stamp > self.age
                    or previous and (sample.sequence<=previous.sequence or stamp<=previous.observed_s)):
                raise InvalidTask("source reboot, identity or timestamp mismatch")
            v=sample.values
            for key in FIELDS[source] & BOOLEANS:
                if type(v[key]) is not bool: raise InvalidTask("typed observation flags required")
            for key,n in (('joints_rad',12),('base_velocity',3),('xy_yaw',3)):
                if key in v:
                    if not isinstance(v[key],(tuple,list)) or len(v[key])!=n: raise InvalidTask("source vector shape")
                    for value in v[key]: number(value,key)
            if source=='lift':
                if type(v['lift_um']) is not int or not 0 <= v['lift_um'] <= 1000000: raise InvalidTask("lift height")
                number(v['lift_velocity'],'lift velocity')
            if source=='scene':
                if any(not isinstance(v[k],str) for k in ('checked_capture_id','checked_scene_revision')):
                    raise InvalidTask("scene proof identities required")
                if (v['reachable'] or v['collision_checked']) and not (v['collision_scene_ready'] and v['checked_capture_id'] and v['checked_scene_revision']):
                    raise InvalidTask("scene readiness is not collision or reachability proof")
            self.samples[source]=deepcopy(sample)
        except Exception as error:
            self.fault=str(error)
            raise

    def produce(self):
        if self.fault: raise InvalidTask(self.fault)
        if set(self.samples)!=set(FIELDS): return None
        now=number(self.clock(),'clock');stamps=[s.observed_s for s in self.samples.values()]
        stamp=min(stamps)
        if not 0 <= stamp <= max(stamps) <= now or now-stamp>self.age or max(stamps)-stamp>self.skew:
            raise InvalidTask("source observations stale or unsynchronized")
        if stamp==self.emitted_s: return None
        if self.emitted_s is not None and stamp<self.emitted_s: raise InvalidTask("source clock reversed")
        values={k:v for s in self.samples.values() for k,v in s.values.items() if k!='healthy'}
        values['healthy']=all(self.samples[k].values['healthy'] for k in ('arm','lift'))
        for key in ('joints_rad','base_velocity','xy_yaw'):values[key]=tuple(map(float,values[key]))
        values['lift_velocity']=float(values['lift_velocity'])
        if self.sequence==0xffffffff: raise InvalidTask("evidence sequence exhausted")
        self.sequence+=1;self.emitted_s=stamp
        return RobotObservation(self.sequence,self.boot,stamp,**values)


def measured_gripper(sample, calibration, *, now_s, maximum_age_s):
    """Convert measured angle/effort. Do not pass a target angle or torque limit."""
    if (not isinstance(sample,SourceSample) or set(sample.values)!={'position_rad','effort_raw','healthy'}
            or type(sample.sequence) is not int or not 0<sample.sequence<=0xffffffff
            or type(sample.values['healthy']) is not bool):
        raise InvalidTask("measured gripper source required")
    age=number(now_s,'now')-number(sample.observed_s,'gripper stamp')
    if not 0<=age<=number(maximum_age_s,'gripper age'): raise InvalidTask("stale gripper measurement")
    effort=number(sample.values['effort_raw'],'measured effort')
    if effort<0: raise InvalidTask("unsigned measured effort magnitude required")
    return GripMeasurement(sample.sequence,sample.observed_s,
        calibration.gap(sample.values['position_rad']),effort,sample.values['healthy'])


class RosEvidencePublisher:
    """Publishes validated observations using source time, never timer time.

    Pair with RosEvidenceSource. Low-level device drivers feed EvidenceProducer;
    existing ROS JointState/odometry or recorded observations can be used alike.
    """
    def __init__(self,node,clock,profile_sha256,*,world_frame='map',namespace='robot_evidence'):
        from so101_interfaces.msg import RobotStateEvidence,GripperObservation,PayloadVisualObservation
        self.types=(RobotStateEvidence,GripperObservation,PayloadVisualObservation)
        self.node,self.clock,self.profile,self.frame=node,clock,profile_sha256,world_frame
        self.publishers=tuple(node.create_publisher(kind,namespace+'/'+topic,10) for kind,topic in zip(
            self.types,('state','gripper','payload')))

    def publish(self, robot=None, grip=None, visual=None):
        from dataclasses import fields
        from builtin_interfaces.msg import Time
        from .payload_monitor import PayloadObservation
        for index,(sample,kind) in enumerate(zip((robot,grip,visual),(RobotObservation,GripMeasurement,PayloadObservation))):
            if sample is None: continue
            if not isinstance(sample,kind): raise InvalidTask("typed source observation required")
            age=number(self.clock(),'clock')-number(sample.observed_s,'stamp')
            if not 0<=age<=2: raise InvalidTask("stale ROS producer input")
            ns=self.node.get_clock().now().nanoseconds-round(age*1e9)
            if ns<0: raise InvalidTask("ROS source timestamp before epoch")
            msg=self.types[index]();msg.header.stamp=Time(sec=ns//1000000000,nanosec=ns%1000000000)
            msg.header.frame_id=sample.camera_frame if index==2 and sample.camera_frame else self.frame
            msg.profile_sha256=self.profile
            for field in fields(sample):
                if field.name in ('observed_s','camera_frame'):continue
                value=getattr(sample,field.name)
                if field.name=='destination' and value is None:value=''
                setattr(msg,field.name,value)
            self.publishers[index].publish(msg)
