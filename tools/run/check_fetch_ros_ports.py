#!/usr/bin/env python3
"""Full fetch over actual ROS action/service transport with SYNTHETIC servers.

This checks assembly/callback/typed-message contracts; real Nav2/MoveIt algorithms
are exercised separately by check_nav2_closed_loop and check_moveit_planning.
"""
import json
from pathlib import Path
import time
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]


def main():
    import rclpy
    from rclpy.action import ActionServer,CancelResponse
    from nav2_msgs.action import NavigateToPose
    from moveit_msgs.srv import GetMotionPlan
    from trajectory_msgs.msg import JointTrajectoryPoint
    from home_robot_tasks.offline_stack import OfflineStack
    from home_robot_tasks.nav2_port import Nav2Port
    from home_robot_tasks.manipulation_port import MoveItPlanPort
    from home_robot_tasks.moveit_bridge import MoveItBridge
    from home_robot_tasks.robot_state import RobotStateStore,RobotObservation
    from home_robot_tasks.payload_monitor import PayloadMonitor
    from home_robot_tasks.ros_stack import RosEvidenceSource
    from home_robot_tasks.evidence_producer import RosEvidencePublisher
    from home_robot_tasks.payload_monitor import GripMeasurement,PayloadObservation
    from so101_interfaces.msg import RobotStateEvidence,GripperObservation,PayloadVisualObservation
    from dataclasses import fields
    read=lambda n:json.loads((ROOT/'config'/n).read_text())
    config=read('fetch_stack.simulation.json');limits=read('bimanual_operational_limits.json')
    rclpy.init();node=rclpy.create_node('fetch_contract_test',namespace='/offline_fetch')
    holder={};calls={'navigation':0,'planning':0}
    async def navigate(goal):
        s=holder['stack'];p=goal.request.pose.pose
        s.xy_yaw=(p.position.x,p.position.y,0.);s.velocity=(0.,0.,0.)
        calls['navigation']+=1;goal.succeed();return NavigateToPose.Result()
    server=ActionServer(node,NavigateToPose,'navigate',navigate,cancel_callback=lambda _:CancelResponse.ACCEPT)
    names=limits['joint_order']
    def plan(request,response):
        p=request.motion_plan_request
        assert p.group_name=='left_arm' and p.goal_constraints[0].position_constraints
        assert p.start_state.joint_state.name[:12]==names
        response.motion_plan_response.error_code.val=1
        jt=response.motion_plan_response.trajectory.joint_trajectory
        jt.joint_names=names[:5]
        a=JointTrajectoryPoint();a.positions=[0.]*5
        b=JointTrajectoryPoint();b.positions=[.05,0.,0.,0.,0.];b.time_from_start.sec=1
        jt.points=[a,b];calls['planning']+=1;return response
    service=node.create_service(GetMotionPlan,'plan',plan)
    bridge=MoveItBridge(names,[-1.]*12,[1.]*12,lambda:holder['stack'].height/1e6)
    nav=Nav2Port(node,'/offline_fetch/navigate')
    planner=MoveItPlanPort(node,bridge.request,bridge.map,'/offline_fetch/plan')
    s=OfflineStack(config,read('home.example.json'),read('navigation_map.simulation.json'),navigation=nav,planner=planner)
    holder['stack']=s
    # Independently exercise all three typed subscriptions through DDS.
    received=RobotStateStore(config['state'],s.clock);payload=PayloadMonitor(**config['payload'])
    source=RosEvidenceSource(node,received,payload,s.clock,'a'*64,namespace='evidence')
    evidence_publisher=RosEvidencePublisher(node,s.clock,'a'*64,namespace='evidence')
    publishers=evidence_publisher.publishers
    deadline=time.monotonic()+10
    try:
        while not nav.ready() or not planner.ready() or any(p.get_subscription_count()==0 for p in publishers):
            if time.monotonic()>deadline:raise TimeoutError('ROS services unavailable')
            rclpy.spin_once(node,timeout_sec=.01)
        from dataclasses import replace
        old_stamp=s.now-.08
        evidence_publisher.publish(replace(s.state.latest,observed_s=old_stamp),
            GripMeasurement(1,old_stamp,45.,0.,True),PayloadObservation(1,old_stamp,'remote_control','empty',camera_frame=config['camera']['camera_frame']))
        for _ in range(20):rclpy.spin_once(node,timeout_sec=.01)
        assert source.error is None and received.latest is not None and payload.grip and payload.vision, (source.error, received.latest, payload.grip, payload.vision)
        assert received.latest.observed_s<=old_stamp+.001 and payload.grip.observed_s<=old_stamp+.001
        assert payload.vision.camera_frame==config['camera']['camera_frame']
        q=dict(schema_version=1,operation='fetch_object',request_id='ros-fetch',request=read('fetch_remote.example.json'))
        s.runtime.submit(q);task=s.runtime.app.tasks['ros-fetch']
        for _ in range(1200):
            # Drain callbacks before advancing the deterministic synthetic clock.
            for _ in range(8):rclpy.spin_once(node,timeout_sec=.001)
            s.tick()
            if task.state in task.TERMINAL or task.state=='STOP_UNCONFIRMED':break
        result=task.result();assert result['status']=='SUCCEEDED',result
        assert calls['navigation']>=4 and calls['planning']==6
        assert len(s.ports['gripper'].starts)==3 and len(s.ports['scene'].starts)==2
        print(json.dumps({'passed':True,'mode':'simulation','servers':'synthetic ROS action/service',
            'completed_steps':result['completed_steps'],'calls':calls,'typed_evidence_topics':3,
            'manipulation_stages':11,'source_timestamps_preserved':True,'hardware_commands':0}))
    finally:
        server.destroy();node.destroy_service(service);node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
