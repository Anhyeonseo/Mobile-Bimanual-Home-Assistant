#!/usr/bin/env python3
"""Real slam_toolbox, synthetic LaserScan/TF, graph save and process reload.

No device interfaces, robot commands or replacement SLAM implementation. Uses
an isolated ROS domain chosen by the caller, just like check_nav2_closed_loop.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=ROOT/'output/s4-local/slam')
    a=p.parse_args();a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=True)
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from lifecycle_msgs.srv import ChangeState
    from slam_toolbox.srv import SerializePoseGraph,DeserializePoseGraph
    from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
    from home_robot_tasks.map_bundle import from_occupancy_grid,write_nav2_map
    from home_robot_tasks.navigation_fusion import TravelEnvelope
    rclpy.init();node=rclpy.create_node('slam_replay',namespace='/offline_s4')
    publisher=node.create_publisher(LaserScan,'scan',10)
    broadcaster=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
    fixed=TransformStamped();fixed.header.frame_id='base_link';fixed.child_frame_id='laser'
    fixed.transform.rotation.w=1.;fixed.header.stamp=node.get_clock().now().to_msg();static.sendTransform(fixed)
    maps=[]
    node.create_subscription(OccupancyGrid,'map',lambda m:maps.append(m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    index=[0];moving=[False];scans=[0];scan_enabled=[True]
    # A closed square, fixed yaw, holonomic odometry; every ray is synthetic.
    route=[(-.6+.02*i,-.6) for i in range(40)]+[(.2,-.6+.02*i) for i in range(40)]+[(.2-.02*i,.2) for i in range(40)]+[(-.6,.2-.02*i) for i in range(40)]
    def occupied(x,y):return abs(x)>=1.9 or abs(y)>=1.9 or (.7<=x<=1.1 and -.3<=y<=.6)
    def publish():
        x,y=route[min(index[0],len(route)-1)]
        now=node.get_clock().now().to_msg()
        tf=TransformStamped();tf.header.stamp=now;tf.header.frame_id='odom';tf.child_frame_id='base_link'
        tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.rotation.w=1.;broadcaster.sendTransform(tf)
        if not scan_enabled[0]:return
        msg=LaserScan();msg.header.stamp=now;msg.header.frame_id='laser';msg.angle_min=-math.pi
        msg.angle_increment=math.tau/360;msg.angle_max=math.pi-msg.angle_increment
        msg.range_min=.12;msg.range_max=3.5;msg.scan_time=.1
        msg.ranges=[]
        for i in range(360):
            angle=msg.angle_min+i*msg.angle_increment;dx,dy=math.cos(angle),math.sin(angle)
            distance=float('inf')
            for n in range(12,351):
                radius=n*.01
                if occupied(x+radius*dx,y+radius*dy):distance=radius;break
            msg.ranges.append(distance)
        publisher.publish(msg);scans[0]+=1
        if moving[0]:index[0]+=1
    node.create_timer(.1,publish)
    phase='startup';service_namespace='/offline_s4'
    process=None;slam_active=False;logs=[];report=dict(mode='real_slam_synthetic_scan_tf',hardware_commands=0,physical_map_verified=False,passed=False)
    def until(predicate,seconds):
        deadline=time.monotonic()+seconds
        while not predicate():
            if process is not None and process.poll() is not None:raise RuntimeError('SLAM process exited')
            if time.monotonic()>deadline:raise TimeoutError('SLAM replay deadline')
            rclpy.spin_once(node,timeout_sec=.02)
    def service(kind,name,request):
        client=node.create_client(kind,service_namespace+'/'+name)
        try:
            until(lambda:client.service_is_ready(),15)
            future=client.call_async(request);until(future.done,15)
            return future.result()
        finally:node.destroy_client(client)
    def launch(number):
        nonlocal process,service_namespace,slam_active
        service_namespace='/offline_s4' if number==1 else '/offline_s4_reload'
        path=Path(get_package_prefix('slam_toolbox'))/'lib/slam_toolbox/async_slam_toolbox_node'
        log=(a.output/f'slam-{number}.log').open('w');logs.append(log)
        process=subprocess.Popen([str(path),'--ros-args','-r','__ns:='+service_namespace,'-r','scan:=/offline_s4/scan','-r','/map:=/offline_s4/map','-r','/map_metadata:=/offline_s4/map_metadata','--params-file',str(ROOT/'ros2_ws/src/home_robot_tasks/config/slam.replay.yaml')],stdout=log,stderr=subprocess.STDOUT)
        for transition in (1,3):
            request=ChangeState.Request();request.transition.id=transition
            if not service(ChangeState,'slam_toolbox/change_state',request).success:raise RuntimeError('SLAM lifecycle transition failed')
        slam_active=True
    def stop():
        nonlocal process,slam_active
        if process is not None:
            if slam_active and process.poll() is None:
                try:
                    for transition in (4,2):
                        request=ChangeState.Request();request.transition.id=transition
                        if not service(ChangeState,'slam_toolbox/change_state',request).success:
                            raise RuntimeError('SLAM cleanup transition failed')
                except Exception as error:report.update(passed=False,error=str(error))
                slam_active=False
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:process.kill();process.wait()
            report.setdefault('process_exit_codes',[]).append(process.returncode)
            if process.returncode!=0:report.update(passed=False,error='local process did not exit cleanly')
            process=None
    try:
        launch(1);moving[0]=True;phase='mapping'
        until(lambda:index[0]>=len(route) and bool(maps),30);moving[0]=False
        until(lambda:len(maps)>=3,10)
        grid=maps[-1]
        document=from_occupancy_grid(grid,map_id='s4-synthetic-room',revision='slam-replay-1',floor_id='floor',
            envelope=TravelEnvelope('synthetic-body',.15,1.),clearance_m=.03)
        write_nav2_map(a.output,'localization',document)
        phase='save_graph'
        graph=a.output/'posegraph'
        response=service(SerializePoseGraph,'slam_toolbox/serialize_map',SerializePoseGraph.Request(filename=str(graph)))
        if response.result!=0:raise RuntimeError('SLAM graph save failed')
        graph_files=[graph.with_suffix('.data'),graph.with_suffix('.posegraph')]
        if not all(f.is_file() and f.stat().st_size>0 for f in graph_files):raise RuntimeError('graph files absent')
        from home_robot_tasks.assistant.contracts import Context
        from home_robot_tasks.assistant.world import WorldStore
        from home_robot_tasks.assistant.scanning import ScanManager
        from home_robot_tasks.mapping_checkpoint import MappingCheckpoints
        checkpoint_db=a.output/'restart-checkpoint.db'
        checkpoint_store=WorldStore(checkpoint_db)
        try:
            checkpoint_context=Context(document['map_id'],document['revision'],'synthetic-map-calibration','before-restart')
            checkpoint_scans=ScanManager(checkpoint_store,checkpoint_context)
            checkpoint_scans.start('graph-restart','Interrupted SLAM',mode='replay')
            checkpoint_scans.ingest_map('graph-restart',document)
            MappingCheckpoints(checkpoint_store).save('graph-restart',stopped=True,
                graph={'slam.data':graph.with_suffix('.data').read_bytes(),
                       'slam.posegraph':graph.with_suffix('.posegraph').read_bytes()})
        finally:checkpoint_store.close()
        checkpoint_store=WorldStore(checkpoint_db)
        try:
            graph=MappingCheckpoints(checkpoint_store).materialize_graph('graph-restart',a.output/'restored-graph')
        finally:checkpoint_store.close()
        report['pose_graph_loaded_from_atomic_checkpoint']=True
        scan_enabled[0]=False;stop();maps.clear();phase='reload_start'
        launch(2);phase='deserialize'
        request=DeserializePoseGraph.Request(filename=str(graph),match_type=1)
        service(DeserializePoseGraph,'slam_toolbox/deserialize_map',request)
        until(lambda:bool(maps),15)
        reloaded=maps[-1]
        known=sum(v>=0 for v in reloaded.data);obstacles=sum(v>=65 for v in reloaded.data)
        if known<100 or obstacles<10:raise RuntimeError('reloaded map lost observations')
        report.update(passed=all(c==0 for c in report.get('process_exit_codes',[])),scan_frames=scans[0],saved_map_shape=[grid.info.width,grid.info.height],
            reloaded_map_shape=[reloaded.info.width,reloaded.info.height],reloaded_known_cells=known,
            reloaded_occupied_cells=obstacles,graph_reloaded_after_restart=True,
            graph_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in graph_files},
            localization_required_for_navigation=True,full_height_map_ready=False)
        stop();phase='height_mapping'
        import numpy as np
        from home_robot_tasks.assistant.contracts import Context
        from home_robot_tasks.assistant.world import WorldStore
        from home_robot_tasks.navigation_depth import DepthProfile
        from home_robot_tasks.mapping_session import MappingSession
        from home_robot_tasks.map_bundle import read_bundle,occupancy_values
        bundle=a.output/'height-map'
        if bundle.exists():raise RuntimeError('use a new output directory for immutable height map')
        store=WorldStore(a.output/'world.db')
        try:
            profile=DepthProfile('synthetic_native_depth','synthetic-map-calibration',.05,.1,3.5,0,1.,.3,.5,world_frame='map')
            context=Context(document['map_id'],document['revision'],profile.calibration,'synthetic-depth-clock')
            mapping=MappingSession(store,context,profile,TravelEnvelope('synthetic-body',.15,1.),
                session_id='slam-depth',name='SLAM and native-depth replay',map_document=document,mode='simulation')
            vv,uu=np.indices((48,64));intrinsics=(48.,48.,31.5,23.5)
            for frame,(x,y) in enumerate([(x,y) for x in (-.6,0.,.6) for y in (-.6,0.,.6)],1):
                pose=np.diag([1.,-1.,-1.,1.]);pose[:3,3]=[x,y,1.2]
                depth=np.full((48,64),1.25)
                # Elevated tabletop absent from the planar LiDAR fixture.
                hit_x=x+(uu-31.5)*.7/48;hit_y=y-(vv-23.5)*.7/48
                elevated=(hit_x>-.9)&(hit_x<-.3)&(hit_y>.35)&(hit_y<.9)
                depth[elevated]=.7
                mapping.depth(depth,intrinsics,pose,observed_s=float(frame),now_s=float(frame),
                    frame=profile.frame,calibration_id=profile.calibration,clock_id=context.clock_id,motion_qualified=True)
            mapping.finish(now_s=9.,output=bundle)
            checkpoints=MappingCheckpoints(store)
            checkpoints.save('slam-depth',stopped=True,graph={
                'slam.data':graph.with_suffix('.data').read_bytes(),
                'slam.posegraph':graph.with_suffix('.posegraph').read_bytes()})
            recovered=checkpoints.load('slam-depth')
            if not any(n.startswith('depth/') for n in recovered['assets']):
                raise RuntimeError('native depth missing from restart checkpoint')
            report['full_mapping_checkpoint_verified']=True
            volume=mapping.scan.archived_volume('slam-depth')
            if volume.fresh(9.):raise RuntimeError('archive improperly authorized live clearance')
            metadata,saved=read_bundle(bundle)
            added=sum(a=='.' and b=='#' for r,s in zip(saved['localization']['rows'],saved['navigation']['rows']) for a,b in zip(r,s))
            if added<1:raise RuntimeError('height obstacle disappeared from saved navigation map')
            report.update(native_depth_frames=9,persisted_depth_voxels=len(volume.cells),
                lidar_free_cells_blocked_by_height=added,depth_restored_as_historical=True,
                full_height_map_ready=True,whole_home_coverage_verified=False)
        finally:store.close()
        phase='saved_map_publication';published=[]
        node.create_subscription(OccupancyGrid,'/offline_s4_saved/navigation_map',lambda m:published.append(m),
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        log=(a.output/'saved-maps.log').open('w');logs.append(log)
        process=subprocess.Popen([sys.executable,'-m','home_robot_tasks.saved_map_node','--bundle',str(bundle),
            '--ros-args','-r','__ns:=/offline_s4_saved'],stdout=log,stderr=subprocess.STDOUT)
        until(lambda:bool(published),10)
        if list(published[-1].data)!=occupancy_values(saved['navigation']):raise RuntimeError('saved map ROS data differs')
        report['saved_body_map_published']=True
    except Exception as error:report.update(passed=False,error=str(error),phase=phase,route_index=index[0],maps_received=len(maps),scan_frames=scans[0])
    finally:
        stop()
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
        (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
