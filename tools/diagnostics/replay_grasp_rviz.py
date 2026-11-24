#!/usr/bin/env python3
"""RViz playback of saved, complete MoveIt routes. No controller/action clients."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from tools.diagnostics.benchmark_grasp_feasibility import capture_models, verify_inputs, save
from home_robot_tasks.carry_model import parameters
from home_robot_tasks import s2_geometry as geo


def first_plan(validation):
    for attempt in validation.get('attempts',[]):
        if attempt['status']=='OFFLINE_ROUTE_FOUND':
            plan=attempt['plan']
            if plan.get('motion_authorized') is not False or plan.get('physical_task_completed') is not False:
                raise ValueError('offline route required')
            if [s['label'] for s in plan['stages']]!=['open_geometry','pregrasp','approach','close_geometry','lift','carry']:
                raise ValueError('complete six-stage MoveIt route required')
            return plan
    raise ValueError('no successful MoveIt route to replay')


def playback_frames(plan,joints,*,fps=30,stage_seconds=2.):
    """Interpolation is for visualization only, not a retimed executable path."""
    candidate=plan['resolved_grasp'];frames=[]
    previous_jaw=joints['left_model_gripper']
    count=max(2,int(round(fps*stage_seconds)))
    for stage in plan['stages']:
        q=np.asarray(stage['joints'],dtype=float)
        if q.ndim!=2 or q.shape[1]!=5 or len(q)==0 or not np.isfinite(q).all():
            raise ValueError('finite five-joint path required')
        label=stage['label'];attached=label in ('lift','carry')
        target_jaw=candidate['held_jaw_model_rad'] if attached or label=='close_geometry' else candidate['open_jaw_model_rad']
        for step,u in enumerate(np.linspace(0,len(q)-1,count)):
            lo=int(np.floor(u));hi=min(lo+1,len(q)-1);fraction=u-lo
            arm=(1-fraction)*q[lo]+fraction*q[hi]
            ratio=step/(count-1)
            jaw=(1-ratio)*previous_jaw+ratio*target_jaw if label in ('open_geometry','close_geometry') else target_jaw
            frames.append(dict(stage=label,arm=arm.tolist(),jaw=float(jaw),attached=attached))
        previous_jaw=target_jaw
    return frames


def triangles(box):
    if box.mesh:
        vertices=np.asarray(box.mesh['vertices']);faces=np.asarray(box.mesh['faces'])
    else:
        vertices=np.array(list(itertools.product((-1,1),repeat=3)))*np.asarray(box.size_m)/2
        faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                        [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    world=vertices@box.base_pose[:3,:3].T+box.base_pose[:3,3]
    return world[faces].reshape(-1,3)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--set',type=int,choices=(1,2),default=1)
    parser.add_argument('--backend',choices=('compare','graspnet','so101'),default='compare')
    parser.add_argument('--results',type=Path,default=ROOT/'output/so101_photo_experiment/fair-comparison-all')
    parser.add_argument('--inputs',type=Path,default=ROOT/'output/so101_photo_experiment')
    parser.add_argument('--mobile-mesh-dir',type=Path,required=True)
    parser.add_argument('--stage-seconds',type=float,default=2.)
    parser.add_argument('--dry-run',action='store_true',help='validate routes and export RViz config without opening ROS or windows')
    args=parser.parse_args()
    if not np.isfinite(args.stage_seconds) or not .5<=args.stage_seconds<=10:
        parser.error('stage-seconds must be .5..10')
    names=['graspnet','so101'] if args.backend=='compare' else [args.backend]
    source=args.inputs/f'set-{args.set}';output=args.inputs/f'rviz-set-{args.set}-{args.backend}'
    output.mkdir(parents=True,exist_ok=True)
    routes=[];models=None;reference=None
    for name in names:
        folder=args.results/f'set-{args.set}'/name
        prepared=json.loads((folder/'prepared.json').read_text())
        verify_inputs(source/prepared['capture_id'],prepared)
        current=capture_models(prepared,source/'capture-model')
        identity={k:prepared[k] for k in ('capture_id','source_sha256','review','observed_scene')}
        if reference is not None and identity!=reference:
            raise ValueError('comparison routes have different captured scenes/states')
        reference=identity;models=current
        plan=first_plan(json.loads((folder/'validation.json').read_text()))
        candidate=plan['resolved_grasp']
        if candidate['scene_id']!=prepared['capture_id'] or candidate['calibration_id']!=current[0]['calibration_id']:
            raise ValueError('saved route calibration/scene mismatch')
        frames=playback_frames(plan,current[-1],stage_seconds=args.stage_seconds)
        if not np.allclose(frames[0]['arm'],[current[-1]['left_model_'+j] for j in geo.JOINTS[:5]]):
            raise ValueError('route does not start at captured arm state')
        routes.append(dict(name=name,plan=plan,frames=frames))
    profile,geometry,arms,installed,boxes,payload,joints=models
    config=parameters(ROOT,profile,geometry,arms,args.mobile_mesh_dir)
    param_path=output/'robot_parameters.yaml'
    param_path.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':config['robot_description']}}}))
    (output/'robot.urdf').write_text(config['robot_description'])
    topic='/offline_grasp_view'
    rviz={'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'}],
        'Visualization Manager':{'Class':'','Global Options':{'Fixed Frame':'base_link','Background Color':'32; 36; 43','Frame Rate':30},
            'Displays':[
                {'Class':'rviz_default_plugins/Grid','Name':'Grid','Enabled':True,'Cell Size':.1,'Plane Cell Count':30},
                {'Class':'rviz_default_plugins/RobotModel','Name':'SO101 robot','Enabled':True,
                 'Description Source':'Topic','Description Topic':{'Value':topic+'/robot_description','Durability Policy':'Transient Local','Reliability Policy':'Reliable','History Policy':'Keep Last','Depth':1}},
                {'Class':'rviz_default_plugins/MarkerArray','Name':'Captured scene and playback','Enabled':True,
                 'Topic':{'Value':topic+'/markers','Durability Policy':'Transient Local','Reliability Policy':'Reliable','History Policy':'Keep Last','Depth':1}},
                {'Class':'rviz_default_plugins/MarkerArray','Name':'Static captured obstacles','Enabled':True,
                 'Topic':{'Value':topic+'/static_markers','Durability Policy':'Transient Local','Reliability Policy':'Reliable','History Policy':'Keep Last','Depth':1}}],
            'Views':{'Current':{'Class':'rviz_default_plugins/Orbit','Distance':1.8,'Pitch':.4,'Yaw':.7,
                'Focal Point':dict(zip(('X','Y','Z'),map(float,payload.base_from_object[:3,3])))}}},
        'Window Geometry':{'Width':1280,'Height':900}}
    rviz_path=output/'playback.rviz';rviz_path.write_text(yaml.safe_dump(rviz))
    save(output/'playback.json',dict(mode='saved_MoveIt_kinematic_playback',set=args.set,
        hardware_commands=0,physical_simulation=False,stage_seconds=args.stage_seconds,
        routes=[dict(backend=r['name'],candidate_id=r['plan']['candidate_id'],frames=len(r['frames'])) for r in routes],
        caveats=['jaw opening/closing and path interpolation use illustrative timing',
                 'payload follows TCP after lift begins; attachment is assumed, not simulated',
                 'playback interpolation is not a new collision check']))
    print('Validated playback: '+str(output),flush=True)
    if args.dry_run:return
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from sensor_msgs.msg import JointState
    from visualization_msgs.msg import Marker,MarkerArray
    from geometry_msgs.msg import Point
    from scipy.spatial.transform import Rotation
    rclpy.init();node=rclpy.create_node('saved_route_player',namespace=topic)
    joint_pub=node.create_publisher(JointState,'joint_states',10)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    marker_pub=node.create_publisher(MarkerArray,'markers',qos)
    static_pub=node.create_publisher(MarkerArray,'static_markers',qos)
    def marker(index,kind,color):
        m=Marker();m.header.frame_id='base_link';m.ns='offline';m.id=index;m.type=kind;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=1.
        m.color.r,m.color.g,m.color.b,m.color.a=color
        return m
    static=[]
    for index,role,color in [(0,'support',(.55,.45,.3,.35)),(1,'obstacle',(.9,.35,.15,.65))]:
        m=marker(index,Marker.TRIANGLE_LIST,color)
        for box in boxes:
            if box.role==role:
                m.points.extend(Point(x=float(x),y=float(y),z=float(z)) for x,y,z in triangles(box))
        static.append(m)
    # Dense captured obstacle meshes are latched once, never serialized at 30 Hz.
    static_pub.publish(MarkerArray(markers=static))
    target=marker(2,Marker.TRIANGLE_LIST,(.15,.85,.95,.9))
    if payload.mesh:
        local=np.asarray(payload.mesh['vertices'])[np.asarray(payload.mesh['faces'])].reshape(-1,3)
    else:
        from types import SimpleNamespace
        local=triangles(SimpleNamespace(mesh=None,size_m=payload.size_m,base_pose=np.eye(4)))
    target.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in local]
    label=marker(3,Marker.TEXT_VIEW_FACING,(1.,1.,1.,1.));label.scale.z=.035
    label.pose.position.x,label.pose.position.y,label.pose.position.z=map(float,payload.base_from_object[:3,3]+[0,0,.25])
    clock={'route':0,'frame':0,'hold':0}
    def tick():
        r=routes[clock['route']];f=r['frames'][clock['frame']]
        state=dict(joints)
        for name,angle in zip(geo.JOINTS[:5],f['arm']):state['left_model_'+name]=angle
        state['left_model_gripper']=f['jaw']
        message=JointState();message.header.stamp=node.get_clock().now().to_msg()
        message.name=list(state);message.position=list(map(float,state.values()));joint_pub.publish(message)
        pose=payload.base_from_object
        if f['attached']:pose=arms['left'].fk(f['arm'])[0]@np.asarray(r['plan']['resolved_grasp']['tcp_from_object'])
        target.pose.position.x,target.pose.position.y,target.pose.position.z=map(float,pose[:3,3])
        quaternion=Rotation.from_matrix(pose[:3,:3]).as_quat()
        target.pose.orientation.x,target.pose.orientation.y,target.pose.orientation.z,target.pose.orientation.w=map(float,quaternion)
        label.text=f"{r['name']} | {f['stage']}\nSaved MoveIt path - no physics / no hardware"
        marker_pub.publish(MarkerArray(markers=[target,label]))
        if clock['frame']<len(r['frames'])-1:clock['frame']+=1
        else:
            clock['hold']+=1
            if clock['hold']>=60:clock.update(route=(clock['route']+1)%len(routes),frame=0,hold=0)
    processes=[]
    try:
        processes.append(subprocess.Popen(['ros2','run','robot_state_publisher','robot_state_publisher',
            '--ros-args','-r','__ns:='+topic,'--params-file',str(param_path)]))
        processes.append(subprocess.Popen(['rviz2','-d',str(rviz_path)]))
        node.create_timer(1/30,tick)
        print('RViz playback started. Ctrl+C stops the viewer. Each backend repeats in turn.',flush=True)
        while rclpy.ok() and processes[-1].poll() is None:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
