#!/usr/bin/env python3
"""Plan/check an explicitly empty stationary hand return using the existing scene."""
import argparse,json,sys
from dataclasses import asdict,replace
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from tools.diagnostics.run_s3_stationary_lab import model
from tools.diagnostics.carry_moveit_session import session
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.carry_model import parameters
from home_robot_tasks.carry_scene import SceneBox,apply_boxes
from home_robot_tasks.carry_moveit import MoveItCarryPlanner
from home_robot_tasks.carry_planning import GraspRouteCandidate
from home_robot_tasks.carry_trajectory import MotionBounds
from home_robot_tasks.carry_program import _control
from home_robot_tasks.carry_bundle import sha,write_json
from home_robot_tasks.empty_return import compile_empty_return,load_empty_return
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--scene',type=Path,required=True);p.add_argument('--anchor',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
r=json.loads((a.scene/'report.json').read_text());profile=json.loads((a.scene/'profile.json').read_text())
reading=json.loads(a.anchor.read_text());anchor=np.array(reading['records'][-1]['joints']['positions_urad'])/1e6
arms={s:model(0,side=s) for s in ('left','right')};arm=arms['left'];joints={'s2_lift_joint':0.}
for side,offset in [('left',0),('right',6)]:
 for i,n in enumerate(geo.JOINTS):joints[f'{side}_model_{n}']=geo.model_from_control(float(anchor[offset+i]),profile['arms'][side]['alignment'][n])
start=np.array([joints['left_model_'+n] for n in geo.JOINTS[:5]])
goal=np.array([r['initial_model_joint_positions_si']['left_model_'+n] for n in geo.JOINTS[:5]])
c=GraspRouteCandidate(**r['plan']['resolved_grasp'])
c=replace(c,base_from_grasp_tcp=np.asarray(c.base_from_grasp_tcp),tcp_from_object=np.asarray(c.tcp_from_object),pregrasp_model_rad=np.asarray(c.pregrasp_model_rad))
opening=min(float(arm.jaw.gaps[-1])-.001,.061)
c=replace(c,open_jaw_model_rad=float(np.interp(opening,arm.jaw.gaps,arm.jaw.angles)))
boxes=[SceneBox(b['name'],np.array(b['base_pose']),tuple(b['size_m']),b['role'],b['mesh']) for b in json.loads((a.scene/'scene.json').read_text())]
state=RobotState(joint_state=JointState(name=list(joints),position=list(joints.values())))
geometry=geo.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
params=parameters(ROOT,profile,geometry,arms,ROOT/'output/carry-model-meshes')
report=dict(status='PLANNING',motor_commands=0,actual_lift_m=0.,empty_gripper_confirmation_required=True)
try:
 with session(params,state,a.output/'moveit') as call:
  apply_boxes(call,boxes);planner=MoveItCarryPlanner(arm,state,call)
  planner.open(start,c)
  target=arm.fk(start)[0].copy();target[2,3]+=.025
  up=planner.cartesian(start,target,c,attached=False,label='rise_empty')
  back=planner.connect(up.end,goal,c,attached=False,label='return_empty')
  ranges=json.loads((ROOT/'config/home_assistant/arm_ranges.development.json').read_text());lo=[];hi=[]
  for side in ('left','right'):
   for n in geo.JOINTS:
    item=ranges['arms'][side][n];lo.append(item['minimum_urad']/1e6);hi.append(item['maximum_urad']/1e6)
  bounds=MotionBounds(tuple(lo),tuple(hi),(.3,)*5+(.08,)+(.3,)*5+(.08,),(.6,)*5+(.2,)+(.6,)*5+(.2,),(3.,)*5+(1.,)+(3.,)*5+(1.,))
  program=compile_empty_return(anchor,up.joints,back.joints,_control(profile,'left','gripper',c.open_jaw_model_rad),profile,bounds)
  plane=np.array(r['observed_support_plane_base']);plane=plane/np.linalg.norm(plane[:3]);support=np.eye(4);z=plane[:3];x=np.array([1.,0.,0.]);x-=z*(x@z);x/=np.linalg.norm(x);support[:3,:3]=np.column_stack([x,np.cross(z,x),z]);support[:3,3]=-plane[3]*z
  checked=0;minimum=np.inf;rise_end=program['stages'][1]['end_ms']
  for point in program['points']:
   control=np.round(point['positions_rad'],6);q=np.array([geo.model_from_control(float(control[i]),profile['arms']['left']['alignment'][n]) for i,n in enumerate(geo.JOINTS)])
   planner.valid(q[:5],c,False,jaw=q[5]);checked+=1
   if point['offset_ms']<=rise_end:
    clear=arm.jaw.plane_clearance(arm.fk(q[:5])[0]@arm.tcp_from_stock,float(q[5]),support)
    minimum=min(minimum,*(v['clearance_m'] for v in clear.values()))
    if min(v['clearance_m'] for v in clear.values())<-.002:raise ValueError('opening/rise crosses support surface')
  m=dict(profile=profile,range_sha256=sha(ROOT/'config/home_assistant/arm_ranges.development.json'),scene_sha256=sha(a.scene/'scene.json'),source_anchor_sha256=sha(a.anchor))
  proof=dict(program_sha256=geo.digest(program),model_sha256=geo.digest(m),observed_empty_required=True,base_commands=0,lift_commands=0,
             native_geometry=dict(status='PASSED',checked_states=checked,planner_checked_states=planner.checked_states,
                                  minimum_open_rise_surface_clearance_m=float(minimum),continuous_collision_proof=False))
  data=dict(kind='checked_empty_return',schema_version=1,program=program,model=m,proof=proof,planned_lift_m=0.,motion_authorized=False)
  write_json(a.output/'return.json',data);load_empty_return(a.output/'return.json')
  report.update(status='CHECKED_EMPTY_RETURN',duration_s=program['duration_s'],samples=len(program['points']),native_geometry=proof['native_geometry'],goal_model_rad=goal.tolist(),opened_gap_m=opening)
except Exception as error:
 report.update(status='RETURN_REJECTED',error=str(error),diagnostics=getattr(error,'diagnostics',None));raise
finally:
 write_json(a.output/'report.json',report);print(json.dumps(report),flush=True)
