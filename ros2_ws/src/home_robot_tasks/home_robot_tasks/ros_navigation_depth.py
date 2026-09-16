"""Native rectified depth -> bounded occupancy worker -> Nav2 OccupancyGrid.

One pending image and one worker job, no backlog. Exposure TF is mandatory.
Only the serialized runtime's poll commits a volume; the control worker never
raycasts. RGB and object detection are intentionally not dependencies.
"""
from concurrent.futures import ThreadPoolExecutor
from copy import copy
import math
import numpy as np
from .fetch import InvalidTask
from .navigation import number
from .ros_rgbd_source import image_array, quaternion_transform, RosRGBDSource


def projected_grid(volume, *, origin_xy, width, height, now):
    """Three-valued full-height projection: any occupied wins; unknown stays -1."""
    if (type(width) is not int or type(height) is not int or min(width,height)<1
            or width*height>250000):
        raise InvalidTask('bounded projection required')
    r=volume.p.resolution_m
    origin=tuple(number(v,'grid origin') for v in origin_xy)
    if len(origin)!=2 or any(abs(v/r-round(v/r))>1e-6 for v in origin):
        raise InvalidTask('grid origin must align with voxel lattice')
    ox,oy=(round(v/r) for v in origin)
    lo,hi=math.floor(volume.p.minimum_height_m/r),math.ceil(volume.p.maximum_height_m/r)
    occupied=volume.projected_obstacles()
    return [100 if (x,y) in occupied else
            (0 if all(volume.free((x,y,z),now) for z in range(lo,hi)) else -1)
            for y in range(oy,oy+height) for x in range(ox,ox+width)]


class RosNavigationDepth:
    def __init__(self,node,tf_buffer,volume,clock,qualified_at,*,depth_topic,info_topic,
                 grid_topic,world_frame,depth_scale_m,origin_xy,width,height,camera_model):
        from sensor_msgs.msg import Image,CameraInfo
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
        if world_frame!=volume.p.world_frame:raise InvalidTask('depth volume world frame mismatch')
        if not 0<number(depth_scale_m,'depth scale')<=.01:raise InvalidTask('explicit depth scale required')
        projected_grid(volume,origin_xy=origin_xy,width=width,height=height,now=clock())
        self.node,self.tf,self.volume,self.clock,self.qualified_at=node,tf_buffer,volume,clock,qualified_at
        self.world,self.scale,self.origin,self.width,self.height=world_frame,depth_scale_m,origin_xy,width,height
        if len(camera_model)!=6:raise InvalidTask('pinned camera dimensions and intrinsics required')
        self.camera_model=tuple(number(v,'camera model') for v in camera_model)
        if any(v<=0 for v in self.camera_model[:4]):raise InvalidTask('invalid pinned camera model')
        self.info=self.pending=self.future=None
        self.reject_generation=0
        self.accept_after=clock()
        self.last_received=None
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='navigation-depth')
        self.publisher=node.create_publisher(OccupancyGrid,grid_topic,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscriptions=[node.create_subscription(Image,depth_topic,self.receive,qos_profile_sensor_data),
                            node.create_subscription(CameraInfo,info_topic,self.receive_info,qos_profile_sensor_data)]

    def reset(self,epoch):
        self.volume.reset(epoch)
        self.accept_after=self.clock()
        self.pending=None
        self.last_received=None

    def receive_info(self,message):
        self.info=message

    def receive(self,message):
        # Keep only the most recent original sample; never invent a timestamp.
        stamp=RosRGBDSource.stamp(message)
        if self.last_received is not None and stamp<=self.last_received:
            self.volume.error='replayed depth image'
            self.reject_generation+=1
            self.pending=None
            return
        self.last_received=stamp
        self.pending=(message,self.info)

    def poll(self):
        from rclpy.time import Time
        from rclpy.duration import Duration
        from nav_msgs.msg import OccupancyGrid
        if self.future is not None and self.future.done():
            try:
                epoch,after,generation,candidate,data=self.future.result()
                if epoch==self.volume.epoch and after==self.accept_after and generation==self.reject_generation:
                    if not candidate.fresh(self.clock()):raise InvalidTask('depth processing deadline missed')
                    self.volume.cells,self.volume.last_stamp,self.volume.error=candidate.cells,candidate.last_stamp,None
                    msg=OccupancyGrid();msg.header.frame_id=self.world
                    age=self.clock()-candidate.last_stamp
                    msg.header.stamp=Time(nanoseconds=self.node.get_clock().now().nanoseconds-round(age*1e9)).to_msg()
                    msg.info.resolution=float(candidate.p.resolution_m)
                    msg.info.width,msg.info.height=self.width,self.height
                    msg.info.origin.position.x,msg.info.origin.position.y=map(float,self.origin)
                    msg.info.origin.orientation.w=1.
                    msg.data=data
                    self.publisher.publish(msg)
            except Exception as error:
                self.volume.error=str(error)
                self.reject_generation+=1
            self.future=None
        if self.future is not None or self.pending is None:return
        message,info=self.pending;self.pending=None
        try:
            if info is None:raise InvalidTask('native depth CameraInfo unavailable')
            stamp=RosRGBDSource.stamp(message)
            age=self.node.get_clock().now().nanoseconds/1e9-stamp
            observed=self.clock()-age
            p=self.volume.p
            if (not 0<=age<=p.maximum_age_s or observed<=self.accept_after
                    or abs(stamp-RosRGBDSource.stamp(info))>.02
                    or message.header.frame_id!=p.frame or info.header.frame_id!=p.frame
                    or message.encoding not in ('16UC1','32FC1')
                    or (message.width,message.height)!=(info.width,info.height)
                    or message.width*message.height>1280*720
                    or not np.allclose((info.width,info.height,info.p[0],info.p[5],info.p[2],info.p[6]),self.camera_model,rtol=0,atol=1e-6)
                    or any(abs(float(v))>1e-9 for v in (info.p[1],info.p[3],info.p[4],info.p[7],info.p[8],info.p[9],info.p[11]))
                    or abs(float(info.p[10])-1)>1e-9):
                raise InvalidTask('native depth metadata/age invalid')
            at=Time.from_msg(message.header.stamp)
            tf=self.tf.lookup_transform(self.world,p.frame,at,timeout=Duration()).transform
            transform=quaternion_transform(tf.translation,tf.rotation)
            qualified=self.qualified_at(observed)
            candidate=copy(self.volume);candidate.cells=dict(self.volume.cells)
            image=image_array(message,depth_scale_m=self.scale)
            intrinsics=tuple(float(info.p[i]) for i in (0,5,2,6))
            epoch,after,generation=self.volume.epoch,self.accept_after,self.reject_generation
            def process():
                candidate.integrate(image,intrinsics,transform,observed_s=observed,now_s=self.clock(),
                    frame=p.frame,calibration=p.calibration,epoch=epoch,motion_qualified=qualified)
                data=projected_grid(candidate,origin_xy=self.origin,width=self.width,height=self.height,now=self.clock())
                return epoch,after,generation,candidate,data
            self.future=self.pool.submit(process)
        except Exception as error:
            self.volume.error=str(error)
            self.reject_generation+=1

    def close(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
