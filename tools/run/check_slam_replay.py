#!/usr/bin/env python3
"""Real SLAM graph save/reload and recorded-data saved-map AMCL comparison.

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


def recorded_replay(a):
    """Real captured scans and nominal encoder odometry in an isolated ROS domain."""
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from lifecycle_msgs.srv import ChangeState
    from slam_toolbox.srv import SerializePoseGraph, DeserializePoseGraph
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from home_robot_tasks.s4_recorded_mapping import encoder_path
    from home_robot_tasks.map_bundle import from_occupancy_grid, write_nav2_map
    from home_robot_tasks.navigation_fusion import TravelEnvelope
    from home_robot_tasks.assistant.contracts import Context
    from home_robot_tasks.assistant.world import WorldStore
    from home_robot_tasks.assistant.scanning import ScanManager
    from home_robot_tasks.mapping_checkpoint import MappingCheckpoints
    if a.output.exists():raise ValueError('new output directory required')
    a.output.mkdir(parents=True)
    records=[json.loads(s) for s in a.teleop.read_text().splitlines()]
    capture=json.loads((a.recorded_capture/'report.json').read_text())
    if capture.get('passed') is not True or capture.get('teleop_finished')!=records[-1]:
        raise ValueError('successful matching real capture required')
    odom,encoder=encoder_path(records,json.loads((ROOT/'config/home_assistant/s4_base.nominal.json').read_text()))
    all_scans=json.loads((a.recorded_capture/'scans.json').read_text())
    scans=[s for s in all_scans if s['observed_s']+359*s['time_increment']>=odom[0,0] and s['observed_s']<=odom[-1,0]]
    mount=json.loads((ROOT/'config/home_assistant/lidar_mount.review.json').read_text())
    localize = a.localization_map is not None
    rclpy.init();node=rclpy.create_node('recorded_replay',namespace='/offline_s4_recorded')
    pub=node.create_publisher(LaserScan,'scan',10);dynamic=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
    maps=[];poses=[]
    node.create_subscription(OccupancyGrid,'map',lambda m:maps.append(m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def save_pose(m):
        p=m.pose.pose;q=p.orientation
        poses.append(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
            xy_yaw=[p.position.x,p.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)],
            covariance=list(m.pose.covariance)))
    node.create_subscription(PoseWithCovarianceStamped,'amcl_pose' if localize else 'pose',save_pose,10)
    map_pub = node.create_publisher(OccupancyGrid,'map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)) if localize else None
    initial_pub = node.create_publisher(PoseWithCovarianceStamped,'initialpose',10) if localize else None
    fixed=TransformStamped();fixed.header.frame_id='base_link';fixed.child_frame_id='lidar_a'
    fixed.header.stamp=node.get_clock().now().to_msg()
    for key,value in zip(('x','y','z'),mount['base_from_lidar_translation_m']):setattr(fixed.transform.translation,key,value)
    fixed.transform.rotation.z=math.sin(mount['yaw_rad']/2);fixed.transform.rotation.w=math.cos(mount['yaw_rad']/2);static.sendTransform(fixed)
    proc=None;namespace='';active=False;logs=[]
    node_name='amcl' if localize else 'slam_toolbox'
    report=dict(mode='real_amcl_recorded_scan_nominal_encoder_tf' if localize else 'real_slam_recorded_scan_nominal_encoder_tf',hardware_commands=0,
        physical_map_verified=False,floor_scale_qualified=False,whole_home_coverage_verified=False,
        autonomous_navigation_authorized=False,passed=False,encoder=encoder,scans_selected=len(scans),
        export_envelope_scope='placeholder needed by map document schema; not measured robot clearance',
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (a.teleop,a.recorded_capture/'scans.json',a.recorded_capture/'report.json',
             ROOT/'config/home_assistant/s4_base.nominal.json',ROOT/'config/home_assistant/lidar_mount.review.json',
             ROOT/'ros2_ws/src/home_robot_tasks/config'/('localization.replay.yaml' if localize else 'slam.replay.yaml'))})
    def until(predicate,seconds):
        end=time.monotonic()+seconds
        while not predicate():
            if proc is not None and proc.poll() is not None:raise RuntimeError('SLAM process exited')
            if time.monotonic()>end:raise TimeoutError('recorded SLAM deadline')
            rclpy.spin_once(node,timeout_sec=.01)
    def service(kind,name,request):
        client=node.create_client(kind,namespace+'/'+name)
        try:
            until(client.service_is_ready,20);future=client.call_async(request);until(future.done,30)
            return future.result()
        finally:node.destroy_client(client)
    def launch(index):
        nonlocal proc,namespace,active
        namespace='/offline_s4_recorded' if index==1 else '/offline_s4_recorded_reload'
        log=(a.output/f'{"amcl" if localize else "slam"}-{index}.log').open('w');logs.append(log)
        package='nav2_amcl' if localize else 'slam_toolbox'
        executable=Path(get_package_prefix(package))/'lib'/package/('amcl' if localize else 'async_slam_toolbox_node')
        proc=subprocess.Popen([str(executable),'--ros-args','-r','__ns:='+namespace,
            '-r','scan:=/offline_s4_recorded/scan','-r','/map:=/offline_s4_recorded/map',
            '-r','/map_metadata:=/offline_s4_recorded/map_metadata','--params-file',
            str(ROOT/'ros2_ws/src/home_robot_tasks/config'/('localization.replay.yaml' if localize else 'slam.replay.yaml'))],stdout=log,stderr=subprocess.STDOUT)
        for transition in (1,3):
            req=ChangeState.Request();req.transition.id=transition
            if not service(ChangeState,node_name+'/change_state',req).success:raise RuntimeError('lifecycle failed')
        active=True
    def stop():
        nonlocal proc,active
        if proc is None:return
        if active and proc.poll() is None:
            for transition in (4,2):
                req=ChangeState.Request();req.transition.id=transition
                if not service(ChangeState,node_name+'/change_state',req).success:raise RuntimeError('cleanup failed')
            active=False
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=10)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        report.setdefault('process_exit_codes',[]).append(proc.returncode);proc=None
    try:
        launch(1)
        from rcl_interfaces.srv import GetParameters
        if localize:
            parameters=service(GetParameters,'amcl/get_parameters',GetParameters.Request(names=['robot_model_type','alpha1','alpha2','alpha3','alpha4','alpha5']))
            report['motion_model']=parameters.values[0].string_value
            report['odometry_noise_alphas']=[p.double_value for p in parameters.values[1:]]
            if report['motion_model']!='nav2_amcl::OmniMotionModel':raise RuntimeError('omnidirectional localization required')
            raw=a.localization_map.read_bytes();report['localization_map_sha256']=hashlib.sha256(raw).hexdigest()
            document=json.loads(raw);grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=node.get_clock().now().to_msg()
            grid.info.width=document['width'];grid.info.height=document['height'];grid.info.resolution=document['resolution']
            grid.info.origin.position.x=document['origin'][0];grid.info.origin.position.y=document['origin'][1];grid.info.origin.orientation.w=1.
            grid.data=document['data'];map_pub.publish(grid)
            until(lambda:initial_pub.get_subscription_count()>0,10)
            deadline=time.monotonic()+.5;until(lambda:time.monotonic()>deadline,2)
            initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.header.stamp=node.get_clock().now().to_msg()
            initial.pose.pose.position.x=a.initial_pose[0];initial.pose.pose.position.y=a.initial_pose[1]
            initial.pose.pose.orientation.z=math.sin(a.initial_pose[2]/2);initial.pose.pose.orientation.w=math.cos(a.initial_pose[2]/2)
            initial.pose.covariance[0]=initial.pose.covariance[7]=.2**2;initial.pose.covariance[35]=math.radians(15)**2
            initial_pub.publish(initial);report['initial_pose_xy_yaw']=a.initial_pose
            report['same_capture_map_replay_not_independent_accuracy']=True
        else:
            parameters=service(GetParameters,'slam_toolbox/get_parameters',GetParameters.Request(names=['check_min_dist_and_heading_precisely']))
            report['rotation_scan_selection_enabled']=bool(parameters.values[0].bool_value)
            if not report['rotation_scan_selection_enabled']:raise RuntimeError('rotation scan selection not enabled')
        clock_start=node.get_clock().now().nanoseconds/1e9+.5;wall_start=time.monotonic()+.5
        source_start=odom[0,0];i=j=0
        def stamp(t):
            from builtin_interfaces.msg import Time
            ns=round((clock_start+t-source_start)*1e9);return Time(sec=ns//10**9,nanosec=ns%10**9)
        end=wall_start+(odom[-1,0]-source_start)+2
        while time.monotonic()<end:
            at=source_start+time.monotonic()-wall_start
            while i<len(odom) and odom[i,0]<=at:
                t,x,y,yaw=odom[i];tf=TransformStamped();tf.header.stamp=stamp(t)
                tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.transform.translation.x=x;tf.transform.translation.y=y
                tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2);dynamic.sendTransform(tf);i+=1
            while j<len(scans) and scans[j]['observed_s']<=at-.1:
                s=scans[j];msg=LaserScan();msg.header.stamp=stamp(s['observed_s']);msg.header.frame_id='lidar_a'
                msg.angle_min=s['angle_min'];msg.angle_increment=s['angle_increment'];msg.angle_max=s['angle_min']+359*s['angle_increment']
                msg.range_min=.12;msg.range_max=3.5;msg.scan_time=s['scan_time'];msg.time_increment=s['time_increment']
                msg.ranges=[float('inf') if v is None else float(v) for v in s['ranges']];pub.publish(msg);j+=1
            rclpy.spin_once(node,timeout_sec=.005)
        if localize:
            until(lambda:len(poses)>=3,10)
            stop()
            reference=json.loads((a.localization_map.parent/'slam-poses.json').read_text())
            reference_report=json.loads((a.localization_map.parent/'report.json').read_text())
            from home_robot_tasks.s4_recorded_mapping import localization_comparison
            comparison=localization_comparison(poses,clock_start-source_start,reference,reference_report['replay_clock_offset_s'])
            report.update(comparison,scans_published=j,pose_messages=len(poses),
                passed=all(c==0 for c in report['process_exit_codes']) and comparison['tracking_sanity_passed'],
                replay_clock_offset_s=clock_start-source_start,
                limitations=['same recorded session used to build map and replay localization; not independent accuracy',
                    'initial pose hypothesis provided, not global localization from an unknown room',
                    'no live device, controller, motor commands, or automatic navigation authorization'])
            print(json.dumps(report));return report
        until(lambda:len(maps)>=2,10)
        # Allow processing to finish before serializing the graph.
        deadline=time.monotonic()+2
        until(lambda:time.monotonic()>=deadline,4)
        grid=maps[-1]
        def grid_dict(g):
            return dict(width=g.info.width,height=g.info.height,resolution=g.info.resolution,
                origin=[g.info.origin.position.x,g.info.origin.position.y],data=list(g.data))
        original=grid_dict(grid);(a.output/'occupancy.json').write_text(json.dumps(original))
        document=from_occupancy_grid(grid,map_id='s4-recorded-room-review',revision='recorded-06',floor_id='floor',
            envelope=TravelEnvelope('nominal-review-only',.22,1.2),clearance_m=.03)
        write_nav2_map(a.output,'localization',document)
        graph=a.output/'posegraph';req=SerializePoseGraph.Request(filename=str(graph))
        if service(SerializePoseGraph,'slam_toolbox/serialize_map',req).result!=0:raise RuntimeError('graph save failed')
        files=[graph.with_suffix('.data'),graph.with_suffix('.posegraph')]
        if not all(f.is_file() and f.stat().st_size for f in files):raise RuntimeError('missing graph assets')
        db=a.output/'checkpoint.db';store=WorldStore(db)
        try:
            ctx=Context(document['map_id'],document['revision'],'nominal-recorded-calibration','recorded-06')
            manager=ScanManager(store,ctx);manager.start('recorded-06','Recorded room review',mode='replay');manager.ingest_map('recorded-06',document)
            MappingCheckpoints(store).save('recorded-06',stopped=True,graph={'slam.data':files[0].read_bytes(),'slam.posegraph':files[1].read_bytes()})
        finally:store.close()
        stop();maps.clear();store=WorldStore(db)
        try:restored=MappingCheckpoints(store).materialize_graph('recorded-06',a.output/'restored-graph')
        finally:store.close()
        launch(2);service(DeserializePoseGraph,'slam_toolbox/deserialize_map',DeserializePoseGraph.Request(filename=str(restored),match_type=1))
        until(lambda:bool(maps),20);loaded=grid_dict(maps[-1]);stop()
        same=original==loaded
        known=sum(v>=0 for v in loaded['data']);occupied=sum(v>=65 for v in loaded['data'])
        report.update(passed=known>100 and occupied>10 and same and all(c==0 for c in report['process_exit_codes']),
            scans_published=j,map_shape=[loaded['width'],loaded['height']],known_cells=known,occupied_cells=occupied,
            known_area_m2=known*loaded['resolution']**2,graph_restored_from_checkpoint=True,
            map_cells_identical_after_restart=same,slam_pose_messages=len(poses),
            graph_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
            replay_clock_offset_s=clock_start-source_start,
            limitations=['recorded-room coverage only, not whole-home mapping',
                'nominal encoder and mounting calibration; no surveyed map accuracy',
                'process restart passed is not physical power-reconnect relocalization'])
    except Exception as error:
        report['error']=str(error)
        raise
    finally:
        if proc is not None:
            try:stop()
            except Exception as error:
                report['cleanup_error']=str(error);report['passed']=False
                if proc is not None and proc.poll() is None:proc.kill();proc.wait()
        for f in logs:f.close()
        (a.output/('amcl-poses.json' if localize else 'slam-poses.json')).write_text(json.dumps(poses))
        (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(report))
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=ROOT/'output/s4-local/slam')
    p.add_argument('--reverse-ray-timing',action='store_true',help='exercise signed ray-time metadata used by the LDS driver')
    p.add_argument('--recorded-capture',type=Path)
    p.add_argument('--teleop',type=Path)
    p.add_argument('--localization-map',type=Path,help='saved occupancy.json with report.json/slam-poses.json from the same capture: compare AMCL against SLAM')
    p.add_argument('--initial-pose',nargs=3,type=float,default=[0.,0.,0.],metavar=('X','Y','YAW'))
    a=p.parse_args();a.output=a.output.resolve()
    if a.localization_map and not a.recorded_capture:p.error('--localization-map requires --recorded-capture')
    if not all(math.isfinite(v) for v in a.initial_pose):p.error('finite initial pose required')
    if a.recorded_capture:
        if not a.teleop:p.error('--recorded-capture requires --teleop')
        return 0 if recorded_replay(a)["passed"] else 1
    a.output.mkdir(parents=True,exist_ok=True)
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
        if a.reverse_ray_timing:msg.time_increment=-msg.scan_time/360
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
    process=None;slam_active=False;logs=[];report=dict(mode='real_slam_synthetic_scan_tf',hardware_commands=0,
        reverse_ray_timing=a.reverse_ray_timing,physical_deskew_verified=False,physical_map_verified=False,passed=False)
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
