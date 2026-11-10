"""LiDAR cadence and localization qualification before shared base ownership.

Source timestamps survive callbacks; receiving an old packet cannot refresh it.
This supplements measured wheel/posture/depth evidence, never replaces it.
"""
from dataclasses import dataclass
import math
import numpy as np
from .fetch import InvalidTask
from .navigation import identifier, number


@dataclass(frozen=True)
class NavigationHealthProfile:
    scan_frame: str
    map_frame: str = 'map'
    rays: int = 360
    maximum_scan_age_s: float = .35
    maximum_scan_gap_s: float = .35
    maximum_localization_age_s: float = .5
    maximum_xy_std_m: float = .15
    maximum_yaw_std_rad: float = .2
    minimum_valid_fraction: float = .1
    consecutive_scans: int = 3

    def __post_init__(self):
        identifier(self.scan_frame,'scan frame');identifier(self.map_frame,'map frame')
        if (type(self.rays) is not int or not 16 <= self.rays <= 4096
                or type(self.consecutive_scans) is not int or not 1 <= self.consecutive_scans <= 20):
            raise InvalidTask('bounded scan health profile required')
        for key in ('maximum_scan_age_s','maximum_scan_gap_s','maximum_localization_age_s',
                    'maximum_xy_std_m','maximum_yaw_std_rad','minimum_valid_fraction'):
            if not 0 < number(getattr(self,key),key) <= 1:raise InvalidTask('invalid navigation health limit')


class NavigationHealth:
    def __init__(self, profile, clock):
        self.p,self.clock=profile,clock
        self.epoch=None
        self.reset('initial')

    def reset(self,epoch):
        identifier(epoch,'navigation health epoch')
        if epoch==self.epoch:raise InvalidTask('new navigation health epoch required')
        self.accept_after=self.clock()
        self.epoch=epoch;self.scan_stamp=None;self.localization_stamp=None;self.scans=0
        self.scan_error='awaiting scan';self.localization_error='awaiting localization'
        self.maximum_gap_s=0.

    def scan(self, *, observed_s, frame, ranges, minimum_range_m, maximum_range_m):
        try:
            stamp=number(observed_s,'scan time');age=self.clock()-stamp
            lo,hi=number(minimum_range_m,'minimum range'),number(maximum_range_m,'maximum range')
            if (frame!=self.p.scan_frame or stamp<=self.accept_after or not 0 <= age <= self.p.maximum_scan_age_s
                    or not isinstance(ranges,(list,tuple)) or len(ranges)!=self.p.rays or not 0<lo<hi<=30):
                raise InvalidTask('scan frame, age, size or range invalid')
            valid=0
            for value in ranges:
                if type(value) not in (int,float):raise InvalidTask('invalid scan range type')
                valid += math.isfinite(value) and lo<=value<=hi
            if valid/len(ranges)<self.p.minimum_valid_fraction:raise InvalidTask('insufficient valid scan ranges')
            if self.scan_stamp is not None:
                gap=stamp-self.scan_stamp
                if gap<=0:raise InvalidTask('replayed or reordered scan')
                self.maximum_gap_s=max(self.maximum_gap_s,gap)
                if gap>self.p.maximum_scan_gap_s:self.scans=0
            self.scan_stamp=stamp;self.scans+=1
            self.scan_error=None if self.scans>=self.p.consecutive_scans else 'awaiting consecutive scans'
        except (ValueError,TypeError) as error:
            self.scans=0;self.scan_error=str(error)
            raise InvalidTask(self.scan_error) from error

    def localization(self, *, observed_s, frame, covariance_xy_yaw, transform_valid):
        try:
            stamp=number(observed_s,'localization time')
            covariance=np.asarray(covariance_xy_yaw,dtype=float)
            if (frame!=self.p.map_frame or stamp<=self.accept_after or transform_valid is not True
                    or not 0<=self.clock()-stamp<=self.p.maximum_localization_age_s
                    or self.localization_stamp is not None and stamp<=self.localization_stamp
                    or covariance.shape!=(3,3) or not np.isfinite(covariance).all()
                    or not np.allclose(covariance,covariance.T,atol=1e-10)
                    or np.min(np.linalg.eigvalsh(covariance)) < -1e-10):
                raise InvalidTask('localization identity, transform, sequence or covariance invalid')
            if (np.max(np.linalg.eigvalsh(covariance[:2,:2]))>self.p.maximum_xy_std_m**2
                    or covariance[2,2]>self.p.maximum_yaw_std_rad**2):
                raise InvalidTask('localization uncertainty exceeds travel limit')
            self.localization_stamp=stamp;self.localization_error=None
        except (ValueError,TypeError) as error:
            self.localization_error=str(error)
            raise InvalidTask(self.localization_error) from error

    def report(self):
        now=self.clock()
        scan_ok=self.scan_error is None and self.scan_stamp is not None and 0<=now-self.scan_stamp<=self.p.maximum_scan_age_s
        pose_ok=self.localization_error is None and self.localization_stamp is not None and 0<=now-self.localization_stamp<=self.p.maximum_localization_age_s
        return dict(epoch=self.epoch,ready=scan_ok and pose_ok,scan_ready=scan_ok,
                    localization_ready=pose_ok,scan_error=self.scan_error,localization_error=self.localization_error,
                    consecutive_scans=self.scans,maximum_scan_gap_s=self.maximum_gap_s,
                    motion_authorized=False)

    def ready(self):return self.report()['ready']


class RosNavigationHealth:
    """Read-only ROS source; qualification errors invalidate health, not executor."""
    def __init__(self,node,health,tf_buffer,*,scan_topic,localization_topic,base_frame):
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import qos_profile_sensor_data
        self.node,self.health,self.tf,self.base=node,health,tf_buffer,base_frame
        self.source_stamps={}
        identifier(base_frame,'base frame')
        self.subscriptions=[node.create_subscription(LaserScan,scan_topic,self.scan,qos_profile_sensor_data),
            node.create_subscription(PoseWithCovarianceStamped,localization_topic,self.localization,10)]

    def time(self,message,kind):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        if stamp<=self.source_stamps.get(kind,-1):raise InvalidTask('replayed ROS observation')
        self.source_stamps[kind]=stamp
        return self.health.clock()-(self.node.get_clock().now().nanoseconds/1e9-stamp)

    def scan(self,message):
        try:self.health.scan(observed_s=self.time(message,'scan'),frame=message.header.frame_id,
            ranges=list(message.ranges),minimum_range_m=message.range_min,maximum_range_m=message.range_max)
        except InvalidTask as error:
            self.health.scans=0;self.health.scan_error=str(error)

    def localization(self,message):
        from rclpy.time import Time
        try:
            covariance=np.asarray(message.pose.covariance).reshape(6,6)[np.ix_([0,1,5],[0,1,5])]
            self.health.localization(observed_s=self.time(message,'localization'),frame=message.header.frame_id,
                covariance_xy_yaw=covariance,transform_valid=bool(self.tf.can_transform(
                    self.health.p.map_frame,self.base,Time.from_msg(message.header.stamp))))
        except (InvalidTask,ValueError) as error:
            self.health.localization_error=str(error)
