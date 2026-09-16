#!/usr/bin/env python3
"""Real Nav2 local costmap: a planar laser cannot erase a separate depth obstacle.

Synthetic laser and projected occupancy grids only; no hardware or navigation
controller. Tests the actual plugins and candidate overlay merge semantics.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import yaml
ROOT=Path(__file__).resolve().parents[2]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output/depth-costmap')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.srv import GetCostmap
    from lifecycle_msgs.srv import ChangeState
    from tf2_ros import StaticTransformBroadcaster
    from rclpy.qos import QoSProfile,DurabilityPolicy
    rclpy.init();node=rclpy.create_node('depth_costmap_contract')
    tf=StaticTransformBroadcaster(node)
    transforms=[]
    for parent,child in (('odom','base_footprint'),('base_footprint','laser')):
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;transforms.append(t)
    tf.sendTransform(transforms)
    publisher=node.create_publisher(OccupancyGrid,'/offline_nav2/depth_grid',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    laser=node.create_publisher(LaserScan,'/offline_depth_costmap/scan',10)
    grid=OccupancyGrid();grid.header.frame_id='odom';grid.info.resolution=.05;grid.info.width=grid.info.height=80
    grid.info.origin.position.x=grid.info.origin.position.y=-2.;grid.info.origin.orientation.w=1.
    grid.data=[-1]*6400
    for y in range(39,42):
        for x in range(59,62):grid.data[y*80+x]=100
    scan_distance=[3.]
    last_map=[None]
    def publish():
        grid.header.stamp=node.get_clock().now().to_msg();publisher.publish(grid)
        scan=LaserScan();scan.header.frame_id='laser';scan.header.stamp=grid.header.stamp
        scan.angle_min=-.1;scan.angle_max=.1;scan.angle_increment=.1
        scan.range_min=.05;scan.range_max=4.;scan.ranges=[scan_distance[0]]*3;laser.publish(scan)
    timer=node.create_timer(.05,publish)
    overlay=yaml.safe_load((ROOT/'config/nav2.depth.simulation.yaml').read_text())['/offline_nav2/local_costmap/local_costmap']['ros__parameters']
    params=dict(use_sim_time=False,global_frame='odom',robot_base_frame='base_footprint',rolling_window=True,
        width=4,height=4,resolution=.05,update_frequency=20.,publish_frequency=20.,robot_radius=.2,
        transform_tolerance=.2,track_unknown_space=overlay['track_unknown_space'],use_maximum=overlay['use_maximum'],always_send_full_costmap=True,
        plugins=['obstacle_layer','depth_layer'],depth_layer=overlay['depth_layer'],
        obstacle_layer=dict(plugin='nav2_costmap_2d::ObstacleLayer',observation_sources='scan',
            scan=dict(topic='/offline_depth_costmap/scan',data_type='LaserScan',marking=True,clearing=True,
                obstacle_max_range=3.5,raytrace_max_range=3.5)))
    log=(args.output/'nav2_costmap.log').open('w');process=None
    try:
        with tempfile.TemporaryDirectory(prefix='alohamini-depth-costmap-') as tmp:
            path=Path(tmp)/'params.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
            binary=Path(get_package_prefix('nav2_costmap_2d'))/'lib/nav2_costmap_2d/nav2_costmap_2d'
            process=subprocess.Popen([str(binary),'--ros-args','-r','__ns:=/offline_depth_costmap','--params-file',str(path)],stdout=log,stderr=log)
            def until(check,seconds=5):
                deadline=time.monotonic()+seconds
                while not check():
                    if process.poll() is not None:raise RuntimeError('Nav2 costmap exited; inspect log')
                    if time.monotonic()>deadline:raise TimeoutError('Nav2 costmap response deadline')
                    rclpy.spin_once(node,timeout_sec=.01)
            services={}
            def discover():
                (args.output/'services.json').write_text(json.dumps(node.get_service_names_and_types()))
                for name,types in node.get_service_names_and_types():
                    if name.startswith('/offline_depth_costmap/'):
                        if 'lifecycle_msgs/srv/ChangeState' in types:services['state']=name
                        if 'nav2_msgs/srv/GetCostmap' in types and name=='/offline_depth_costmap/get_costmap':services['cost']=name
                return 'state' in services
            until(discover)
            lifecycle=node.create_client(ChangeState,services['state']);until(lifecycle.service_is_ready)
            for transition in (1,3):
                request=ChangeState.Request();request.transition.id=transition
                future=lifecycle.call_async(request);until(future.done)
                assert future.result().success,(transition,services)
            from rcl_interfaces.srv import GetParameters
            pc=node.create_client(GetParameters,services['state'].rsplit('/',1)[0]+'/get_parameters')
            until(pc.service_is_ready)
            query=GetParameters.Request();query.names=['plugins','depth_layer.enabled','use_maximum','lethal_cost_threshold','unknown_cost_value','track_unknown_space']
            pf=pc.call_async(query);until(pf.done)
            (args.output/'parameters.txt').write_text(str(list(zip(query.names,pf.result().values))))
            until(lambda:discover() and 'cost' in services)
            getter=node.create_client(GetCostmap,services['cost']);until(getter.service_is_ready)
            def value_at():
                f=getter.call_async(GetCostmap.Request());until(f.done)
                m=f.result().map
                last_map[0]=m
                x=int((1.025-m.metadata.origin.position.x)/m.metadata.resolution)
                y=int((.025-m.metadata.origin.position.y)/m.metadata.resolution)
                (args.output/'last_sample.json').write_text(json.dumps(dict(
                    origin=[m.metadata.origin.position.x,m.metadata.origin.position.y],resolution=m.metadata.resolution,
                    size=[m.metadata.size_x,m.metadata.size_y],query=[x,y],value=int(m.data[y*m.metadata.size_x+x]),
                    counts={str(v):list(m.data).count(v) for v in set(m.data)},
                    lethal_cells=[[i%m.metadata.size_x,i//m.metadata.size_x] for i,v in enumerate(m.data) if v==254][:20])))
                return m.data[y*m.metadata.size_x+x]
            assert grid.data[3260]==100
            until(lambda:value_at()==254)
            # Repeated planar rays cross this cell but cannot clear depth layer.
            start=time.monotonic();until(lambda:time.monotonic()-start>.3)
            assert value_at()==254,'laser incorrectly erased upper depth obstacle'
            # Explicit new depth projection clears A. This is never caused by a
            # timeout, empty depth frame or planar ray.
            for y in range(39,42):
                for x in range(59,62):grid.data[y*80+x]=0
            until(lambda:value_at()==0)
            scan_distance[0]=.7
            def lidar_retained():
                value_at()
                return any(v==254 for v in last_map[0].data)
            until(lidar_retained)
            print(json.dumps(dict(passed=True,mode='simulation',hardware_commands=0,real_nav2_costmap=True,
                lidar_cannot_clear_depth=True,explicit_depth_clear_applied=True,depth_free_preserves_lidar_hit=True)))
            return 0
    finally:
        if process is not None:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
