#!/usr/bin/env python3
"""Compare DWB footprints against a recorded costmap, with no motor bridge.

This is a retrospective frozen-scene replay, not a physical success claim.
Use a separate ROS_DOMAIN_ID; publishes only synthetic stationary odometry.
"""
import argparse,json,math,os,subprocess,time
from pathlib import Path
import numpy as np
import yaml


def replay(folder,root,output,variant,recorded_parameters=None):
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rclpy.action import ActionClient
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid,Odometry
    from geometry_msgs.msg import TransformStamped,PoseStamped,TwistStamped
    from nav2_msgs.action import FollowPath
    from dwb_msgs.msg import LocalPlanEvaluation
    from tf2_ros import TransformBroadcaster
    from ament_index_python.packages import get_package_prefix
    from home_robot_tasks.nav2_trial import nav2_trial_config
    ns='/costmap_replay';saved=json.loads((folder/'obstacle-wait-status.json').read_text());pose=saved['pose']
    meta=json.loads((folder/'block-costmaps.json').read_text())['local'];grid=np.load(folder/'block-costmaps.npz')['local']
    route=saved.get('path') or (saved.get('nav2') or {}).get('path')
    if not route:raise ValueError('recorded nonempty route required; cancelled previews may have no route')
    if recorded_parameters is None:
        config=nav2_trial_config(yaml.safe_load((root/'config/nav2.alohamini.candidate.yaml').read_text()),root,namespace=ns,map_target=True)
    else:
        config=yaml.safe_load(recorded_parameters.read_text().replace('/s4_nav2_trial',ns))
    controller=config[ns+'/controller_server']['ros__parameters'];controller['FollowPath']['debug_trajectory_details']=True
    local=config[ns+'/local_costmap/local_costmap']['ros__parameters']
    if variant=='previous':
        local['footprint']=json.dumps(saved['navigation_body']['footprint_xy_m'])
        local['footprint_padding']=saved['navigation_body']['clearance_m']+math.sqrt(2)*local['resolution']/2
    elif variant=='raster-padded':
        from home_robot_tasks.nav2_trial import offset_convex_footprint
        local['footprint']=json.dumps(offset_convex_footprint(saved['navigation_body']['footprint_xy_m'],
            saved['navigation_body']['clearance_m']+math.sqrt(2)*local['resolution']/2))
        local['footprint_padding']=0.
    # Preserve recorded lattice exactly; recompute inflation from lethal cells.
    local.update(rolling_window=False,global_frame='odom',plugins=['static_layer','inflation_layer'],
        track_unknown_space=False,use_maximum=False)
    local['static_layer']=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic=ns+'/map',map_subscribe_transient_local=True)
    config={k:v for k,v in config.items() if k in (ns+'/controller_server',ns+'/local_costmap/local_costmap')}
    path=output/(variant+'-params.yaml');path.write_text(yaml.safe_dump(config))
    rclpy.init();node=rclpy.create_node('frozen_costmap_replay');tf=TransformBroadcaster(node)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    mp=node.create_publisher(OccupancyGrid,ns+'/map',qos);op=node.create_publisher(Odometry,ns+'/odom',10)
    evaluations=[];commands=[];invalid_critics={}
    def evaluation(m):
        evaluations.append((sum(t.total>=0 for t in m.twists),len(m.twists)))
        for trajectory in m.twists:
            if trajectory.total>=0:continue
            for score in trajectory.scores:
                if score.raw_score<0:
                    invalid_critics[score.name]=invalid_critics.get(score.name,0)+1
    sub=node.create_subscription(LocalPlanEvaluation,ns+'/evaluation',evaluation,1)
    cmd=node.create_subscription(TwistStamped,ns+'/cmd_vel',lambda m:commands.append((m.twist.linear.x,m.twist.linear.y,m.twist.angular.z)),1)
    m=OccupancyGrid();m.header.frame_id='odom';m.info.resolution=meta['resolution'];m.info.width=grid.shape[1];m.info.height=grid.shape[0]
    m.info.origin.position.x=meta['origin']['position']['x'];m.info.origin.position.y=meta['origin']['position']['y'];m.info.origin.orientation.w=1.
    m.data=np.where(grid==254,100,np.where(grid==255,-1,0)).astype(int).ravel().tolist();mp.publish(m)
    def tick():
        now=node.get_clock().now().to_msg();o=Odometry();o.header.frame_id='odom';o.header.stamp=now;o.child_frame_id='base_footprint'
        o.pose.pose.position.x,o.pose.pose.position.y=pose[:2];o.pose.pose.orientation.z=math.sin(pose[2]/2);o.pose.pose.orientation.w=math.cos(pose[2]/2);op.publish(o)
        t=TransformStamped();t.header=o.header;t.child_frame_id=o.child_frame_id;t.transform.translation.x,t.transform.translation.y=pose[:2];t.transform.rotation=o.pose.pose.orientation;tf.sendTransform(t)
    timer=node.create_timer(.025,tick)
    log=(output/(variant+'.log')).open('w');proc=subprocess.Popen([str(Path(get_package_prefix('nav2_controller'))/'lib/nav2_controller/controller_server'),'--ros-args','-r','__ns:='+ns,'--params-file',str(path)],stdout=log,stderr=subprocess.STDOUT)
    def wait(future,limit=10):
        end=time.monotonic()+limit
        while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.025)
        if not future.done():raise TimeoutError('replay ROS request')
        return future.result()
    try:
        cl=node.create_client(ChangeState,ns+'/controller_server/change_state');assert cl.wait_for_service(timeout_sec=10)
        for transition in (1,3):
            request=ChangeState.Request();request.transition.id=transition;assert wait(cl.call_async(request)).success
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.025)
        action=ActionClient(node,FollowPath,ns+'/follow_path');assert action.wait_for_server(timeout_sec=5)
        goal=FollowPath.Goal();goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker';goal.path.header.frame_id='odom';goal.path.header.stamp=node.get_clock().now().to_msg()
        for x,y,yaw in route:
            p=PoseStamped();p.header=goal.path.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);goal.path.poses.append(p)
        handle=wait(action.send_goal_async(goal));assert handle.accepted
        end=time.monotonic()+3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.025)
        wait(handle.cancel_goal_async())
        result=dict(variant=variant,evaluations=evaluations,invalid_critics=invalid_critics,valid_max=max((v for v,n in evaluations),default=0),nonzero_candidate_commands=sum(any(abs(v)>1e-6 for v in c) for c in commands),motor_bridge=False)
        (output/(variant+'-result.json')).write_text(json.dumps(result,indent=2));return result
    finally:
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--recording',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--variant',choices=['previous','raster-padded','corrected'],required=True)
    p.add_argument('--recorded-parameters',type=Path,help='reuse recorded Nav2 parameters, including manipulation arrival tolerances')
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='190':p.error('isolated ROS_DOMAIN_ID=190 required')
    a.output.mkdir(parents=True,exist_ok=True)
    print(json.dumps(replay(a.recording,Path(__file__).resolve().parents[2],a.output,a.variant,a.recorded_parameters)))
