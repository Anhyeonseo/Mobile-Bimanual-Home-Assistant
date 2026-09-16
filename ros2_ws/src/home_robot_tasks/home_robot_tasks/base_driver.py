"""Nav2 SI velocity -> existing STM32 mobile client; measured wheel odometry.

An explicitly prepared MobileClient is injected on the same serialized worker
as the arms/lift. Never opens a UART, provisions modes, homes, arms or reconnects.
Firmware applies physical motor signs; this layer must not apply them twice.
"""
from dataclasses import dataclass
import math

from .base_motion import WheelOdometry
from .fetch import InvalidTask
from .navigation import identifier, number


@dataclass(frozen=True)
class BaseProfile:
    raw_per_rad_s: tuple
    maximum_wheel_rad_s: tuple
    maximum_acceleration_rad_s2: tuple
    maximum_command_age_s: float
    maximum_feedback_age_s: float
    maximum_cycle_s: float

    def __post_init__(self):
        for vector in (self.raw_per_rad_s,self.maximum_wheel_rad_s,self.maximum_acceleration_rad_s2):
            if len(vector)!=3 or any(number(v,'wheel scale/limit')<=0 for v in vector):
                raise InvalidTask('three explicit positive wheel scales/limits required')
        if any(s*v>32767 for s,v in zip(self.raw_per_rad_s,self.maximum_wheel_rad_s)):
            raise InvalidTask('wheel command exceeds mobile wire range')
        for value in (self.maximum_command_age_s,self.maximum_feedback_age_s,self.maximum_cycle_s):
            if not 0 < number(value,'base time budget') <= .5:
                raise InvalidTask('invalid base time budget')


@dataclass(frozen=True)
class BaseObservation:
    observed_s: float
    pose: tuple
    twist: tuple
    boot_id: int
    feedback_tick_ms: int


class MobileBaseDriver:
    def __init__(self,client,geometry,profile,clock,guard,travel_evidence,request_stop):
        self.client,self.geometry,self.p,self.clock = client,geometry,profile,clock
        self.guard,self.travel_evidence,self.request_stop = guard,travel_evidence,request_stop
        self.odometry = WheelOdometry(geometry,max_gap_s=profile.maximum_feedback_age_s)
        self.owner = self.source = self.epoch = None
        self.command = None
        self.granted = self.last_cycle = None
        self.last_feedback_tick = self.feedback_time = None
        self.observation = None
        self.last_rates = (0.,0.,0.)
        self.fault = None
        self.stop_error = None
        self.braking = False

    def acquire(self,owner,source,epoch):
        if self.owner is not None or self.fault or self.client.session is None:
            raise InvalidTask('base requires unowned prepared session')
        # Task/phase goal IDs contain '/' and ':'; map identifiers do not.
        # Validate all fields before assigning any ownership.
        if not isinstance(owner,str) or not 1 <= len(owner) <= 256 or any(ord(c)<32 for c in owner):
            raise InvalidTask('invalid base owner')
        source,epoch=(identifier(v,'base authority') for v in (source,epoch))
        self.owner,self.source,self.epoch=owner,source,epoch
        self.braking = False
        self.granted = number(self.clock(),'grant time')
        self.command = None
        self.last_cycle = self.granted
        self.last_rates = (0.,0.,0.)

    def submit(self,twist,*,observed_s,owner,source,epoch):
        try:
            now = self.clock()
            if self.fault or self.owner is None or (owner,source,epoch)!=(self.owner,self.source,self.epoch):
                raise InvalidTask('base command has no current owner')
            stamp = number(observed_s,'command acquisition time')
            values = tuple(number(v,'SI twist') for v in twist)
            if (len(values)!=3 or not 0 <= now-stamp <= self.p.maximum_command_age_s
                    or stamp <= self.granted or (self.command and stamp <= self.command[0])):
                raise InvalidTask('stale, replayed or invalid base command')
            if self.braking and any(values):
                raise InvalidTask('navigation completed; only stop commands accepted')
            self.command = (stamp,values)
        except Exception as error:
            # A stale/wrong publisher cannot replace a live owner's command.
            if self.owner is not None:
                self._fault(str(error))
            raise

    def _fault(self,reason):
        self.fault = self.fault or reason
        self.command = None
        try:
            self.request_stop(self.fault)
        except Exception as error:
            self.stop_error = str(error)
        # Retain owner. New driver/session and explicit recovery are required.

    def observe(self):
        sent = self.clock()
        status = self.client.status()
        received = self.clock()
        if (received < sent or received-sent > self.p.maximum_cycle_s
                or not status.ready_for_motion(self.client.session,round(self.p.maximum_feedback_age_s*1000))):
            raise InvalidTask('mobile feedback unhealthy, unowned or late')
        if self.observation and status.boot_id != self.observation.boot_id:
            raise InvalidTask('mobile device restarted')
        tick = status.feedback_tick_ms
        age = ((status.mcu_tick_ms-tick)&0xffffffff)/1000
        stamp = sent-age  # conservative oldest host acquisition bound
        if self.last_feedback_tick is not None:
            delta = (tick-self.last_feedback_tick)&0xffffffff
            if delta == 0:
                if received-self.feedback_time > self.p.maximum_feedback_age_s:
                    raise InvalidTask('repeated wheel sample expired')
                return self.observation
            if delta >= 0x80000000:
                raise InvalidTask('wheel acquisition clock regressed')
            # Preserve MCU sample spacing; status polling must not retime wheels.
            stamp = min(stamp,self.feedback_time+delta/1000)
        if not 0 <= received-stamp <= self.p.maximum_feedback_age_s:
            raise InvalidTask('wheel sample aged during transport')
        rates = tuple(v/s for v,s in zip(status.velocity_raw[:3],self.p.raw_per_rad_s))
        if any(abs(v)>limit*1.2 for v,limit in zip(rates,self.p.maximum_wheel_rad_s)):
            raise InvalidTask('measured wheel speed outside profile')
        self.odometry.observe(rates,stamp)
        self.last_feedback_tick,self.feedback_time = tick,stamp
        self.observation = BaseObservation(stamp,self.odometry.pose,self.geometry.body_twist(rates),
                                           status.boot_id,tick)
        return self.observation

    def tick(self):
        if self.fault:
            self._fault(self.fault)
            return None
        if self.owner is None:
            if self.client.session is None:
                return None
            try:
                return self.observe()
            except Exception as error:
                self._fault(str(error))
                return None
        try:
            now = self.clock()
            dt = now-self.last_cycle
            if not 0 < dt <= self.p.maximum_cycle_s:
                raise InvalidTask('base control cycle missed')
            self.last_cycle = now
            observation = self.observe()
            now = self.clock()
            if self.command is None:
                if now-self.granted > self.p.maximum_command_age_s:
                    raise InvalidTask('first base command timed out')
                requested = (0.,0.,0.)
            elif not 0 <= now-self.command[0] <= self.p.maximum_command_age_s:
                raise InvalidTask('base command expired')
            else:
                requested = self.command[1]
            target = self.geometry.wheel_rates(requested,self.p.maximum_wheel_rad_s)
            # One interpolation factor preserves the wheel-vector direction.
            scale = max(1.,*(abs(b-a)/(limit*dt) for a,b,limit in
                            zip(self.last_rates,target,self.p.maximum_acceleration_rad_s2)))
            rates = tuple(a+(b-a)/scale for a,b in zip(self.last_rates,target))
            raw = tuple(round(rate*factor) for rate,factor in zip(rates,self.p.raw_per_rad_s))
            # Check the quantized command that the board will actually receive.
            applied_rates = tuple(v/s for v,s in zip(raw,self.p.raw_per_rad_s))
            applied = self.geometry.body_twist(applied_rates)
            evidence = self.travel_evidence()
            if evidence.epoch != self.epoch:
                raise InvalidTask('travel mode changed under base owner')
            self.guard.check(applied,observation.twist,observation.pose,evidence,now)
            now = self.clock()
            if now-self.last_cycle > self.p.maximum_cycle_s:
                raise InvalidTask('base control processing deadline missed')
            remaining = self.p.maximum_command_age_s-(now-(self.command[0] if self.command else self.granted))
            lifetime = min(200,math.floor(remaining*1000))
            if lifetime < 1:
                raise InvalidTask('base command expired during observation')
            result = self.client.velocity(raw+(0,),lifetime_ms=lifetime)
            if not result.ready_for_motion(self.client.session,round(self.p.maximum_feedback_age_s*1000)):
                raise InvalidTask('mobile command acknowledgement not active')
            self.last_rates = applied_rates
            return observation
        except Exception as error:
            self._fault(str(error))
            return None

    def release(self,*,stop_confirmed):
        if type(stop_confirmed) is not bool or not stop_confirmed:
            raise InvalidTask('independent whole-stop confirmation required')
        self.owner = self.source = self.epoch = None
        self.command = None


class RosBaseIO:
    """Single-executor ROS endpoint; tick is owned by the existing runtime.

    Stamped input only: a queued Twist must not acquire a new timestamp on receipt.
    The caller chooses the sole post-safety cmd_vel topic and current authority.
    Covariance is explicit; zero covariance cannot claim perfect wheel odometry.
    """
    def __init__(self,node,driver,*,command_topic,odom_topic,odom_frame,base_frame,
                 pose_variance,twist_variance):
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile
        from tf2_ros import TransformBroadcaster
        for vector in (pose_variance,twist_variance):
            if len(vector)!=6 or any(number(v,'odometry variance')<=0 for v in vector):
                raise InvalidTask('six positive measured/candidate covariance entries required')
        if odom_frame!=driver.guard.volume.p.world_frame:
            raise InvalidTask('odometry and depth volume frame mismatch')
        if odom_frame==base_frame:
            raise InvalidTask('distinct odometry frames required')
        self.node,self.driver,self.odom_frame,self.base_frame=node,driver,odom_frame,base_frame
        self.variance=pose_variance,twist_variance
        self.publisher=node.create_publisher(Odometry,odom_topic,10)
        self.tf=TransformBroadcaster(node)
        self.last_published=None
        self.command_authority=None
        self.last_ros_command=None
        self.subscription=node.create_subscription(TwistStamped,command_topic,self.receive,QoSProfile(depth=1))

    def receive(self,msg):
        try:
            if self.driver.owner is None:
                return
            if msg.header.frame_id!=self.base_frame or any(v!=0 for v in
                (msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y)):
                raise InvalidTask('planar body-frame stamped command required')
            authority=(self.driver.owner,self.driver.epoch,self.driver.granted)
            if authority!=self.command_authority:
                self.command_authority=authority
                self.last_ros_command=None
            stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            if (not 0<=msg.header.stamp.nanosec<1000000000
                    or (self.last_ros_command is not None and stamp_ns<=self.last_ros_command)):
                raise InvalidTask('replayed ROS command timestamp')
            age=(self.node.get_clock().now().nanoseconds-stamp_ns)/1e9
            if not 0<=age<=self.driver.p.maximum_command_age_s:
                raise InvalidTask('stale or future ROS command')
            self.last_ros_command=stamp_ns
            self.driver.submit((msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z),
                observed_s=self.driver.clock()-age,owner=self.driver.owner,
                source=self.driver.source,epoch=self.driver.epoch)
        except Exception as error:
            self.driver._fault(str(error))

    def tick(self):
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.time import Time
        observation=self.driver.tick()
        if observation is None or observation.observed_s==self.last_published:
            return
        age=self.driver.clock()-observation.observed_s
        stamp=Time(nanoseconds=self.node.get_clock().now().nanoseconds-round(age*1e9)).to_msg()
        msg=Odometry();msg.header.stamp=stamp;msg.header.frame_id=self.odom_frame;msg.child_frame_id=self.base_frame
        x,y,yaw=observation.pose
        msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z=observation.twist
        for covariance,variance in ((msg.pose.covariance,self.variance[0]),(msg.twist.covariance,self.variance[1])):
            for i,value in enumerate(variance):covariance[7*i]=float(value)
        self.publisher.publish(msg)
        tf=TransformStamped();tf.header=msg.header;tf.child_frame_id=self.base_frame
        tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.rotation=msg.pose.pose.orientation
        self.tf.sendTransform(tf)
        self.last_published=observation.observed_s
