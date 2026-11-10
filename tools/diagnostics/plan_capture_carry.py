#!/usr/bin/env python3
"""Plan a capture-bound pick/lift/carry through local MoveIt. Never opens hardware."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.camera_evidence import near_depth_profile
from home_robot_tasks.carry_binding import bind_grasp
from home_robot_tasks.carry_model import parameters
from home_robot_tasks.carry_moveit import MoveItCarryPlanner
from home_robot_tasks.carry_planning import plan_pick_carry
from home_robot_tasks.carry_scene import from_prepared_capture, apply_boxes
from home_robot_tasks.installed_jaw import InstalledLeftJaw, StockLeftJaw
from home_robot_tasks.grasp_preferences import TabletopGraspPolicy
from home_robot_tasks.stationary_pick_sim import StationaryArm
from home_robot_tasks.so101_jaw import load_stock_jaw
from tools.diagnostics.carry_moveit_session import session


def serial(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)


def load_models(prepared, *, nominal_obstacles=None, obstacle_padding_m=.001, jaw_basis='measured_front'):
    read=lambda name:json.loads((ROOT/'config/home_assistant'/name).read_text())
    profile=read('s2_coordinates.development.json')
    if jaw_basis not in ('measured_front','nominal_stock_tcp_section'):raise ValueError('unknown jaw basis')
    measured_profile=profile
    if prepared['review']['calibration_id']!=profile['calibration_id']:
        profile=near_depth_profile(profile,read('camera_device.observed.json'),read('camera_device.near.observed.json'))
    boxes,payload,joints=from_prepared_capture(prepared,profile,nominal_obstacles=nominal_obstacles,obstacle_padding_m=obstacle_padding_m)
    geometry=geo.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    jaw=load_stock_jaw(ROOT/'config/home_assistant/official_sources/so101_new_calib.urdf',ROOT/'ros2_ws/src/so101_description/meshes')
    installed=(StockLeftJaw(jaw,profile,read('arm_ranges.development.json')['arms']['left']['gripper'])
               if jaw_basis=='nominal_stock_tcp_section' else InstalledLeftJaw(read('left_pad_gap.observed.json'),measured_profile))
    arms={side:StationaryArm(profile,geometry,read('arm_ranges.development.json'),joints['s2_lift_joint'],jaw,side)
          for side in ('left','right')}
    return profile,geometry,arms,installed,boxes,payload,joints


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared',type=Path,required=True)
    parser.add_argument('--mobile-mesh-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--maximum-grasps',type=int,default=8,choices=range(1,33))
    parser.add_argument('--approach-mode',choices=('tool_axis','vertical'),default='tool_axis',
                        help='vertical aligns XY above the contact without forcing an upright wrist')
    parser.add_argument('--nominal-jaw',action='store_true',help='explicit stock-STL fingers; use nominal TCP-section width, not measured pads')
    parser.add_argument('--grasp-posture',choices=('tabletop','legacy'),default='tabletop',
                        help='prefer easy tabletop contact; legacy is for historical offline comparisons')
    parser.add_argument('--entry-clearance-mm',type=int,default=4,choices=range(4,21),
                        help='extra opening beyond observed span; final close target unchanged')
    parser.add_argument('--close-allowance-mm',type=int,default=5,choices=(1,2,3,4,5),
                        help='fixed close target below observed width; position preload, not calibrated force')
    parser.add_argument('--fixed-finger-clearance-mm',type=int,default=0,choices=range(0,11),
                        help='pregrasp-only fixed-side offset, converging to the original contact')
    parser.add_argument('--preview-lift',type=float,help='offline hypothetical height; preserves original capture state')
    envelope=parser.add_mutually_exclusive_group()
    envelope.add_argument('--nominal-obstacles',action='store_true',default=None,help='use actual point bounds + 1 mm (default for new ownership scenes)')
    envelope.add_argument('--rounded-obstacles',dest='nominal_obstacles',action='store_false',help='compare the legacy 5 mm rounded columns')
    parser.add_argument('--obstacle-padding-mm',type=int,choices=(0,1),default=1,help='offline point-envelope comparison, not execution clearance')
    args=parser.parse_args()
    if args.approach_mode=='vertical' and args.fixed_finger_clearance_mm:
        parser.error('vertical descent requires --fixed-finger-clearance-mm 0; align XY before lowering')
    if args.fixed_finger_clearance_mm > args.entry_clearance_mm-2:
        parser.error('entry opening must retain at least 2 mm on the moving side')
    prepared=json.loads(args.prepared.read_text())
    if args.nominal_obstacles is None:
        args.nominal_obstacles=prepared['observed_scene'].get('preferred_obstacle_envelope')=='point_bounds'
    jaw_basis='nominal_stock_tcp_section' if args.nominal_jaw else 'measured_front'
    grasp_policy=TabletopGraspPolicy() if args.grasp_posture=='tabletop' else None
    profile,geometry,arms,installed,boxes,payload,joints=load_models(prepared,nominal_obstacles=args.nominal_obstacles,
        obstacle_padding_m=args.obstacle_padding_mm/1000,jaw_basis=jaw_basis)
    captured_joints=dict(joints)
    if args.preview_lift is not None:
        if not np.isfinite(args.preview_lift) or not 0<=args.preview_lift<=profile['lift']['travel_m']:
            parser.error('preview lift outside model travel')
        ranges=json.loads((ROOT/'config/home_assistant/arm_ranges.development.json').read_text())
        arms={side:StationaryArm(profile,geometry,ranges,args.preview_lift,arms[side].jaw,side) for side in arms}
        joints['s2_lift_joint']=args.preview_lift
    arm=arms['left'];start=np.array([joints['left_model_'+n] for n in geo.JOINTS[:5]])
    report=dict(mode='offline_capture_moveit',jaw_basis=jaw_basis,capture_id=prepared['capture_id'],source_sha256=prepared['source_sha256'],
        approach_mode=args.approach_mode,
        close_allowance_m=args.close_allowance_mm/1000,
        grasp_posture_policy=asdict(grasp_policy) if grasp_policy else None,
        prepared_sha256=hashlib.sha256(args.prepared.read_bytes()).hexdigest(),calibration_id=profile['calibration_id'],
        hardware_commands=0,motion_authorized=False,physical_task_completed=False,continuous_collision_proof=False,
        environment_coverage='observed_only',initial_model_joint_positions_si=joints,
        captured_model_joint_positions_si=captured_joints,hypothetical_lift_m=args.preview_lift,
        obstacle_envelope='point_bounds' if args.nominal_obstacles else 'rounded_5mm_columns',obstacle_padding_mm=args.obstacle_padding_mm,
        observed_scene_boxes=len(boxes),candidates=[],binding_failures=[],status='NO_COMPLETE_ROUTE')
    args.output.mkdir(parents=True,exist_ok=True)
    candidates=[]
    for index,jaw_sign in ((i,s) for i in range(min(64,len(prepared['group']['refined_grasps']))) for s in (1,-1)):
        try:
            candidate,jaw=bind_grasp(arm,prepared['review'],prepared['group'],index,payload,installed,
                                     hypothetical_lift_m=args.preview_lift,jaw_sign=jaw_sign,
                                     entry_clearance_m=args.entry_clearance_mm/1000,
                                     fixed_finger_clearance_m=args.fixed_finger_clearance_mm/1000,
                                     grasp_policy=grasp_policy,close_allowance_m=args.close_allowance_mm/1000,
                                     approach_mode=args.approach_mode)
            candidates.append(candidate)
            report['candidates'].append(dict(candidate=asdict(candidate),jaw_preview=jaw,
                source=prepared['group']['refined_grasps'][index]['source']))
            # Let the coordinator prioritize balance before spending its route
            # budget. Detector order must not discard centred bound candidates.
        except ValueError as exc:
            report['binding_failures'].append(dict(index=index,jaw_sign=jaw_sign,reason=str(exc),diagnostics=getattr(exc,'diagnostics',None)))
    try:
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity
        config=parameters(ROOT,profile,geometry,arms,args.mobile_mesh_dir)
        (args.output/'robot.urdf').write_text(config['robot_description'])
        state=RobotState();state.joint_state.name=list(joints);state.joint_state.position=list(joints.values())
        with session(config,state,args.output) as call:
            apply_boxes(call,boxes)
            validity=call('check_state_validity',GetStateValidity.Request(robot_state=state))
            report['initial_state_valid']=validity.valid
            report['initial_contacts']=[(c.contact_body_1,c.contact_body_2) for c in validity.contacts]
            report['initial_contact_details']=[dict(bodies=[c.contact_body_1,c.contact_body_2],
                depth_m=c.depth,position_m=[c.position.x,c.position.y,c.position.z])
                for c in validity.contacts]
            if validity.valid:
                backend=MoveItCarryPlanner(arm,state,call)
                plan=plan_pick_carry(arm,backend,start,candidates,scene_id=prepared['capture_id'],maximum_grasps=args.maximum_grasps,
                                     grasp_policy=grasp_policy)
                report['plan']=asdict(plan);report['status']=plan.status;report['checked_states']=backend.checked_states
            else:
                report['reason']='captured robot state collides in the current model/observed scene'
    except Exception as exc:
        report['status']='PLANNER_ERROR';report['reason']=str(exc)
    if grasp_policy is not None and report['status']=='NO_COMPLETE_ROUTE':
        report['next_action']='REPOSITION_BASE_AND_REOBSERVE'
        report['next_action_motion_authorized']=False
    (args.output/'report.json').write_text(json.dumps(report,default=serial,indent=2)+'\n')
    summary={k:report.get(k) for k in ('status','capture_id','hypothetical_lift_m','obstacle_envelope','checked_states','reason')}
    summary.update(bound_candidates=len(candidates),rejected_candidates=len(report['binding_failures']),hardware_commands=0)
    print(json.dumps(summary,default=serial))
    for failure in report.get('plan',{}).get('failures',[]):print(failure['stage']+': '+failure['reason'])
    return 0 if report['status']=='OFFLINE_ROUTE_FOUND' else 2


if __name__=='__main__':raise SystemExit(main())
