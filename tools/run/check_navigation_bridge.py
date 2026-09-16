#!/usr/bin/env python3
"""Actual DDS -> reusable stamped SI bridge -> C mobile parser/plant -> DDS odom.

Native rectified depth also traverses DDS/exposure TF/worker/grid publication.
All images, body clearance, geometry and wheel feedback are SYNTHETIC. This
checks software wiring, not Nav2 planning or D415/vibration/traction performance.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,default=ROOT/'build/offline-core/libactuator_host_simulator.so')
    args=parser.parse_args()
    import rclpy
    from geometry_msgs.msg import TwistStamped,TransformStamped
    from sensor_msgs.msg import Image,CameraInfo
    from nav_msgs.msg import Odometry,OccupancyGrid
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster
    from home_robot_tasks.base_motion import Wheel,OmniKinematics
    from home_robot_tasks.base_driver import BaseProfile,MobileBaseDriver,RosBaseIO
    from home_robot_tasks.navigation_depth import DepthProfile,DepthVolume,TravelGuard,TravelEvidence
    from home_robot_tasks.ros_navigation_depth import RosNavigationDepth
    from so101_arm_bridge.host_simulator import NativeHostSerial
    from so101_arm_bridge.mobile_client import MobileClient
    from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2,MobileV2Exchange
    rclpy.init();node=rclpy.create_node('navigation_bridge_contract',namespace='/offline_bridge')
    clock=time.monotonic
    source=None
    try:
        volume=DepthVolume(DepthProfile('depth','synthetic',.1,.1,4.,0.,1.,.5,.5))
        volume.reset('travel-1')
        buffer=Buffer();listener=TransformListener(buffer,node);broadcaster=StaticTransformBroadcaster(node)
        # optical x -> -world y; optical y -> -world z; optical z -> world x
        tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='depth'
        tf.transform.translation.z=.5
        tf.transform.rotation.x=-.5;tf.transform.rotation.y=.5;tf.transform.rotation.z=-.5;tf.transform.rotation.w=.5
        broadcaster.sendTransform(tf)
        source=RosNavigationDepth(node,buffer,volume,clock,lambda _:True,depth_topic='depth',info_topic='info',
            grid_topic='depth_grid',world_frame='odom',depth_scale_m=.001,origin_xy=(-1.,-1.),width=40,height=20,
            camera_model=(1,1,1,1,0,0))
        image_pub=node.create_publisher(Image,'depth',qos_profile_sensor_data)
        info_pub=node.create_publisher(CameraInfo,'info',qos_profile_sensor_data)
        grids=[]
        node.create_subscription(OccupancyGrid,'depth_grid',grids.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def spin_until(check,limit=3.):
            deadline=clock()+limit
            while not check():
                if clock()>deadline:raise TimeoutError('DDS contract deadline')
                rclpy.spin_once(node,timeout_sec=.005);source.poll()
        spin_until(lambda:image_pub.get_subscription_count()>0 and info_pub.get_subscription_count()>0 and buffer.can_transform('odom','depth',rclpy.time.Time()))
        def publish_depth(mm):
            stamp=node.get_clock().now().to_msg()
            info=CameraInfo();info.header.frame_id='depth';info.header.stamp=stamp;info.width=info.height=1
            info.p=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.]
            image=Image();image.header=info.header;image.width=image.height=1;image.encoding='16UC1';image.step=2
            image.data=list(mm.to_bytes(2,'little'));info_pub.publish(info)
            # Deliver CameraInfo before image; no artificial timestamp refresh.
            for _ in range(3):rclpy.spin_once(node,timeout_sec=.005)
            image_pub.publish(image)
        publish_depth(1000);spin_until(lambda:len(grids)>=1)
        assert any(v==100 for v in grids[-1].data),volume.error
        original=volume.last_stamp
        publish_depth(2000);spin_until(lambda:len(grids)>=2)
        assert (10,0) not in volume.projected_obstacles()
        assert (20,0) in volume.projected_obstacles(),volume.projected_obstacles()
        assert volume.last_stamp>original
        # Separate, explicit synthetic full-volume fixture for all-direction travel.
        travel=DepthVolume(volume.p);travel.reset('base-1')
        def refresh_clear_fixture():
            now=clock();travel.last_stamp=now;travel.error=None
            travel.cells={(x,y,z):(False,now) for x in range(-15,16) for y in range(-15,16) for z in range(10)}
        refresh_clear_fixture()
        serial=NativeHostSerial(args.library,lambda:int(clock()*1000),boot_id=42)
        client=MobileClient(MobileV2Exchange(StreamValidationTransportV2(serial,response_timeout_s=.05)),lambda:int(clock()*1000))
        # Explicit harness setup, never performed by the reusable bridge.
        client.synchronize();client.arm(1)
        geometry=OmniKinematics(tuple(Wheel(.2*math.cos(a),.2*math.sin(a),a+math.pi/2,.05) for a in (0,2*math.pi/3,4*math.pi/3)))
        stops=[]
        def stop(reason):
            # Native plant has mobile feedback only. Whole-system SAFE_STOP
            # binding/proof is covered separately by factory/HAL stop tests.
            stops.append(reason);client.stop()
        guard=TravelGuard(travel,radius_m=.2,height_m=1.,margin_m=.02,reaction_s=.1,braking_m_s2=1.,maximum_speed_m_s=.4,maximum_yaw_rad_s=.8)
        driver=MobileBaseDriver(client,geometry,BaseProfile((100.,)*3,(10.,)*3,(100.,)*3,.2,.15,.1),clock,
            guard,lambda:TravelEvidence(travel.last_stamp,'base-1',True,True,True),stop)
        io=RosBaseIO(node,driver,command_topic='cmd_vel_safe',odom_topic='odom',odom_frame='odom',base_frame='base_footprint',
                     pose_variance=(.01,)*6,twist_variance=(.02,)*6)
        publisher=node.create_publisher(TwistStamped,'cmd_vel_safe',1)
        odoms=[];node.create_subscription(Odometry,'odom',odoms.append,10)
        spin_until(lambda:publisher.get_subscription_count()>0 and io.publisher.get_subscription_count()>0)
        driver.acquire('bridge-test','nav2','base-1')
        deadline=clock()+.25
        while clock()<deadline:
            refresh_clear_fixture()
            msg=TwistStamped();msg.header.frame_id='base_footprint';msg.header.stamp=node.get_clock().now().to_msg();msg.twist.linear.x=.2
            publisher.publish(msg)
            for _ in range(2):rclpy.spin_once(node,timeout_sec=.003)
            io.tick()
            assert driver.fault is None,driver.fault
        spin_until(lambda:len(odoms)>2)
        assert max(m.pose.pose.position.x for m in odoms)>0
        assert all(m.pose.covariance[0]>.0 for m in odoms)
        assert driver.client.status().velocity_raw[3]==0
        stale=TwistStamped();stale.header.frame_id='base_footprint'
        stale.header.stamp.sec=node.get_clock().now().to_msg().sec-2;stale.twist.linear.x=.2
        publisher.publish(stale);spin_until(lambda:driver.fault is not None)
        spin_until(lambda:client.status().velocity_raw==(0,0,0,0))
        assert stops and driver.owner=='bridge-test'
        print(json.dumps(dict(passed=True,mode='simulation',hardware_commands=0,physical_task_completed=False,
            real_dds=True,native_c_plant=True,depth_grid_messages=len(grids),odom_messages=len(odoms),
            stale_command_mobile_stop=True,depth_motion_quality='synthetic qualification only')))
        return 0
    finally:
        if source is not None:source.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
