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


def tabletop_lift_directions(normal, distance_m=.02):
    """Prefer separation normal to the observed table; retain base-up fallback.

    Keep the existing probe envelope (2 mm lateral travel). The complete
    attached-object path and rounded commands still require collision checks.
    """
    n=np.asarray(normal,dtype=float)
    if n.shape!=(3,) or not np.isfinite(n).all() or n[2]<=0:
        raise ValueError('finite upward observed support normal required')
    n=n/np.linalg.norm(n);vertical=np.array([0.,0.,1.])
    if np.linalg.norm(n[:2])*distance_m>.002 or np.allclose(n,vertical,atol=1e-8):
        return [vertical.tolist()]
    return [n.tolist(),vertical.tolist()]


def load_models(prepared, *, nominal_obstacles=None, obstacle_padding_m=.001, jaw_basis='measured_front'):
    read=lambda name:json.loads((ROOT/'config/home_assistant'/name).read_text())
    profile=read('s2_coordinates.development.json')
    if jaw_basis not in ('measured_front','nominal_stock_tcp_section'):raise ValueError('unknown jaw basis')
    measured_profile=profile
    if prepared['review']['calibration_id']!=profile['calibration_id']:
        wide=read('s2_coordinates.wide.development.json')
        profile=(wide if prepared['review']['calibration_id']==wide['calibration_id'] else
                 near_depth_profile(profile,read('camera_device.observed.json'),read('camera_device.near.observed.json')))
    if prepared['review']['calibration_id']!=profile['calibration_id']:
        raise ValueError('unknown prepared camera calibration')
    boxes,payload,joints=from_prepared_capture(prepared,profile,nominal_obstacles=nominal_obstacles,obstacle_padding_m=obstacle_padding_m)
    if prepared.get('start_revalidation') is not None:
        from home_robot_tasks.pick_lift_transition import planning_review
        joints=planning_review(prepared,profile)['model_joint_positions_si']
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
    parser.add_argument('--compare-all-routes',action='store_true',
                        help='offline comparison only: evaluate alternatives after the first complete route')
    parser.add_argument('--approach-mode',choices=('tool_axis','vertical'),default='tool_axis',
                        help='vertical aligns XY above the contact without forcing an upright wrist')
    parser.add_argument('--nominal-jaw',action='store_true',help='explicit stock-STL fingers; use nominal TCP-section width, not measured pads')
    parser.add_argument('--grasp-posture',choices=('tabletop','legacy'),default='tabletop',
                        help='prefer easy tabletop contact; legacy is for historical offline comparisons')
    parser.add_argument('--entry-clearance-mm',type=int,default=4,choices=range(4,21),
                        help='extra opening beyond observed span; final close target unchanged')
    parser.add_argument('--contact-clearance-mm',type=int,default=0,choices=range(0,11),
                        help='fixed-side gap retained through descent; closing may slide the object')
    parser.add_argument('--close-allowance-mm',type=int,default=5,choices=(1,2,3,4,5),
                        help='fixed close target below observed width; position preload, not calibrated force')
    parser.add_argument('--close-extra-raw',type=int,default=0,
                        help='additional stock-jaw closing preload; clamped at its existing raw limit')
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
    report=plan(args)
    summary={k:report.get(k) for k in ('status','capture_id','hypothetical_lift_m','obstacle_envelope','checked_states','reason')}
    summary.update(bound_candidates=len(report['candidates']),rejected_candidates=len(report['binding_failures']),hardware_commands=0)
    print(json.dumps(summary,default=serial))
    for failure in report.get('plan',{}).get('failures',[]):print(failure['stage']+': '+failure['reason'])
    return 0 if report['status']=='OFFLINE_ROUTE_FOUND' else 2


def plan(args, *, session_factory=session, preview_base_xy=None):
    """Callable planner; session lifetime may be owned by a resident worker."""
    if args.approach_mode=='vertical' and args.fixed_finger_clearance_mm:
        raise ValueError('vertical requires no fixed-finger offset')
    if args.fixed_finger_clearance_mm > args.entry_clearance_mm-2:
        raise ValueError('moving side clearance below 2 mm')
    prepared=json.loads(args.prepared.read_text())
    if args.nominal_obstacles is None:
        args.nominal_obstacles=prepared['observed_scene'].get('preferred_obstacle_envelope')=='point_bounds'
    jaw_basis='nominal_stock_tcp_section' if args.nominal_jaw else 'measured_front'
    grasp_policy=TabletopGraspPolicy() if args.grasp_posture=='tabletop' else None
    profile,geometry,arms,installed,boxes,payload,joints=load_models(prepared,nominal_obstacles=args.nominal_obstacles,
        obstacle_padding_m=args.obstacle_padding_mm/1000,jaw_basis=jaw_basis)
    from home_robot_tasks.pick_calibration import TOP_BASIS, TOP_BASES, check_top_surface
    from home_robot_tasks.pick_lift_transition import planning_review
    review=planning_review(prepared,profile)
    group = prepared['group']
    if preview_base_xy is not None:
        from home_robot_tasks.tabletop_alignment import translated_observation
        review, group, boxes, payload = translated_observation(
            preview_base_xy, profile, review, group, boxes, payload)
    calibrated_top = review.get('lift_reference_basis') in TOP_BASES
    if calibrated_top:
        if args.preview_lift is not None:
            raise ValueError('상단 실측 경로는 촬영 때의 등록 높이를 사용합니다.')
        points = prepared['review']['base_points_m']
        check_top_surface(prepared['support_plane'], points.get('center_xyz_m', points.get('surface_xyz_m')), profile)
    captured_joints=dict(prepared['review']['model_joint_positions_si'])
    if args.preview_lift is not None:
        if not np.isfinite(args.preview_lift) or not 0<=args.preview_lift<=profile['lift']['travel_m']:
            raise ValueError('preview lift outside model travel')
        ranges=json.loads((ROOT/'config/home_assistant/arm_ranges.development.json').read_text())
        arms={side:StationaryArm(profile,geometry,ranges,args.preview_lift,arms[side].jaw,side) for side in arms}
        joints['s2_lift_joint']=args.preview_lift
    arm=arms['left'];start=np.array([joints['left_model_'+n] for n in geo.JOINTS[:5]])
    report=dict(mode='offline_capture_moveit',jaw_basis=jaw_basis,capture_id=prepared['capture_id'],source_sha256=prepared['source_sha256'],
        approach_mode=args.approach_mode,
        cartesian_orientation_tolerance_deg=5. if args.approach_mode=='vertical' and grasp_policy is not None else 2.,
        contact_clearance_m=getattr(args,'contact_clearance_mm',0)/1000,
        close_allowance_m=args.close_allowance_mm/1000,
        close_extra_raw=getattr(args,'close_extra_raw',0),
        grasp_posture_policy=asdict(grasp_policy) if grasp_policy else None,
        prepared_sha256=hashlib.sha256(args.prepared.read_bytes()).hexdigest(),calibration_id=profile['calibration_id'],
        hardware_commands=0,motion_authorized=False,physical_task_completed=False,continuous_collision_proof=False,
        environment_coverage='observed_only',initial_model_joint_positions_si=joints,
        captured_model_joint_positions_si=captured_joints,hypothetical_lift_m=args.preview_lift,
        obstacle_envelope='point_bounds' if args.nominal_obstacles else 'rounded_5mm_columns',obstacle_padding_mm=args.obstacle_padding_mm,
        observed_scene_boxes=len(boxes),candidates=[],binding_failures=[],status='NO_COMPLETE_ROUTE')
    if calibrated_top:
        from home_robot_tasks.pick_calibration import REVISION
        transform = np.asarray(profile['base_from_optical'])
        plane = prepared['support_plane']
        normal = transform[:3,:3] @ plane['normal']
        point = transform[:3,:3] @ plane['point_m'] + transform[:3,3]
        if preview_base_xy is not None:
            point[:2] -= preview_base_xy
        report.update(lift_reference_basis=review['lift_reference_basis'], calibration_revision=REVISION,
                      observed_support_plane_base=[*normal, -float(normal @ point)])
    directions=(tabletop_lift_directions(normal) if calibrated_top and args.approach_mode=='vertical'
                else [[0.,0.,1.]])
    report['lift_directions_base']=directions
    if prepared.get('start_revalidation') is not None:
        report['start_revalidation']=prepared['start_revalidation']
    args.output.mkdir(parents=True,exist_ok=True)
    if preview_base_xy is not None:
        from home_robot_tasks.tabletop_alignment import projected_target_visible
        from home_robot_tasks.camera_evidence import load_capture_profile
        for mode in ('near', 'wide', 'legacy'):
            _, device = load_capture_profile(ROOT, mode)
            if device['camera_device'] == profile['camera_device']:
                break
        else:
            raise ValueError('no matching camera intrinsics for tabletop preview')
        report.update(mode='hypothetical_tabletop_alignment', preview_base_xy_m=list(preview_base_xy),
                      within_camera_view=projected_target_visible(profile, payload,
                                               device['sdk_calibration']['color_intrinsics']))
        if not report['within_camera_view']:
            report['reason'] = 'target outside camera view'
            (args.output/'report.json').write_text(json.dumps(report, default=serial, indent=2)+'\n')
            return report
    candidates=[]
    for index,jaw_sign in ((i,s) for i in range(min(64,len(prepared['group']['refined_grasps']))) for s in (1,-1)):
        try:
            candidate,jaw=bind_grasp(arm,review,group,index,payload,installed,
                                     hypothetical_lift_m=args.preview_lift,jaw_sign=jaw_sign,
                                     entry_clearance_m=args.entry_clearance_mm/1000,
                                     fixed_finger_clearance_m=args.fixed_finger_clearance_mm/1000,
                                     grasp_policy=grasp_policy,close_allowance_m=args.close_allowance_mm/1000,
                                     close_extra_raw=getattr(args,'close_extra_raw',0),
                                     approach_mode=args.approach_mode,
                                     contact_clearance_m=getattr(args,'contact_clearance_mm',0)/1000,
                                     support_normal_base=normal if calibrated_top else (0.,0.,1.))
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
        with session_factory(config,state,args.output) as call:
            validity_request = GetStateValidity.Request(robot_state=state)
            if preview_base_xy is None:
                apply_boxes(call, boxes)
                validity = call('check_state_validity', validity_request)
            else:
                from home_robot_tasks.tabletop_alignment import scene_sweep
                for fraction, moved_boxes in scene_sweep(boxes, preview_base_xy):
                    apply_boxes(call, moved_boxes)
                    validity = call('check_state_validity', validity_request)
                    if not validity.valid:
                        break
                report['observed_sweep_clear'] = bool(validity.valid)
                report['sweep_fraction'] = float(fraction)
            report['initial_state_valid']=validity.valid
            report['initial_contacts']=[(c.contact_body_1,c.contact_body_2) for c in validity.contacts]
            report['initial_contact_details']=[dict(bodies=[c.contact_body_1,c.contact_body_2],
                depth_m=c.depth,position_m=[c.position.x,c.position.y,c.position.z])
                for c in validity.contacts]
            if validity.valid:
                backend=MoveItCarryPlanner(arm,state,call,vertical_grasp_policy=grasp_policy if args.approach_mode=='vertical' else None)
                plan=plan_pick_carry(arm,backend,start,candidates,scene_id=prepared['capture_id'],maximum_grasps=args.maximum_grasps,
                                     grasp_policy=grasp_policy,stop_after_first=not args.compare_all_routes,top_calibrated=calibrated_top,
                                     exhaust_candidates=getattr(args,'exhaust_candidates',False),lift_directions_base=directions)
                report['plan']=asdict(plan);report['status']=plan.status;report['checked_states']=backend.checked_states
            else:
                report['reason']='captured robot state collides in the current model/observed scene'
    except Exception as exc:
        report['status']='PLANNER_ERROR';report['reason']=str(exc)
    from home_robot_tasks.pick_recovery import planning_recovery
    from tools.lib.pick_contact_review import contact_height_review
    report['contact_height_review']=contact_height_review(report,prepared)
    report['recovery']=planning_recovery(report)
    report['next_action']=report['recovery']['action']
    report['next_action_motion_authorized']=False
    if preview_base_xy is not None and report['status'] == 'OFFLINE_ROUTE_FOUND':
        report['status'] = 'HYPOTHETICAL_ROUTE_FOUND'
    (args.output/'report.json').write_text(json.dumps(report,default=serial,indent=2)+'\n')
    return report


if __name__=='__main__':raise SystemExit(main())
