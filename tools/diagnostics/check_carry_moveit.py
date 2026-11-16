#!/usr/bin/env python3
"""Exercise current nominal robot + dynamic carry through real offline MoveIt.

Synthetic grasp poses validate planning integration, NOT perception performance.
No device/SSH/controller is opened. Source ROS Jazzy before running.
"""
import argparse
from dataclasses import asdict, replace
from itertools import product
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'ros2_ws/src/home_robot_tasks'), str(ROOT/'ros2_ws/src/so101_arm_bridge'), str(ROOT)]
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.carry_model import parameters
from home_robot_tasks.carry_planning import GraspRouteCandidate, plan_pick_carry, plan_place_carry, PlanRejected
from home_robot_tasks.carry_posture import ObjectAxisConstraint,CarryPolicy
from home_robot_tasks.carry_moveit import MoveItCarryPlanner
from home_robot_tasks.stationary_pick_sim import StationaryArm
from home_robot_tasks.so101_jaw import load_stock_jaw
from tools.diagnostics.carry_moveit_session import session


def serial(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    raise TypeError(type(value).__name__)


def synthetic_grasp(arm, roll, *, upright=False):
    """Place a synthetic box against the stock fixed finger at a reachable pose.

    The object is constructed for this offline test; no observed object or
    robot target is moved to hide a contact residual. Apertures follow the
    actual closing-line section, including the rotated upright fixture.
    """
    pre = np.deg2rad([0., -45., 45., 60., 90. + roll])
    pose = arm.fk(pre)[0]
    pose[:3, 3] += .006 * pose[:3, 2]
    size = (.01, .012, .012)
    attachment = arm.tcp_from_stock.copy()
    attachment[:3, 3] += attachment[:3, :3] @ [-arm.jaw.center_from_fixed_m(size[0]), 0., 0.]
    if upright:
        attachment[:3, :3] = pose[:3, :3].T
    object_pose = pose @ attachment
    vertices = np.array(list(product(*[(-v / 2, v / 2) for v in size])))
    world = vertices @ object_pose[:3, :3].T + object_pose[:3, 3]
    section = arm.jaw.contact_section(pose @ arm.tcp_from_stock, world)
    attachment[:3, 3] -= pose[:3, :3].T @ np.asarray(section['tcp_correction_base_m'])
    held_gap = section['width_m']
    open_gap = held_gap + .004
    if not arm.jaw.accepts(held_gap) or not arm.jaw.accepts(open_gap):
        raise ValueError('synthetic fixture exceeds stock jaw aperture')
    constraint = (ObjectAxisConstraint(attachment, maximum_tilt_rad=np.deg2rad(10),
                                      evidence='synthetic upright task requirement')
                  if upright else None)
    return GraspRouteCandidate(
        'upright-fixture' if upright else 'fixture-roll-' + str(roll),
        arm.profile['calibration_id'], 'fixture-scene', pre, pose, attachment, size,
        float(np.interp(open_gap, arm.jaw.gaps, arm.jaw.angles)),
        float(np.interp(held_gap, arm.jaw.gaps, arm.jaw.angles)), constraint,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mobile-mesh-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'output/home-assistant/s3/carry-moveit')
    parser.add_argument('--include-place', action='store_true', help='also plan payload lowering, release sweep and retreat')
    parser.add_argument('--compile-task-programs', action='store_true', help='compile and verify the new shared-runtime pick/place stage programs')
    args = parser.parse_args()
    from moveit_msgs.msg import RobotState, CollisionObject
    from moveit_msgs.srv import GetPositionFK, GetStateValidity, ApplyPlanningScene
    from shape_msgs.msg import SolidPrimitive
    from scipy.spatial.transform import Rotation
    read = lambda name: json.loads((ROOT/'config/home_assistant'/name).read_text())
    profile = read('s2_coordinates.development.json')
    geometry = geo.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    jaw = load_stock_jaw(ROOT/'config/home_assistant/official_sources/so101_new_calib.urdf', ROOT/'ros2_ws/src/so101_description/meshes')
    arms = {side: StationaryArm(profile, geometry, read('arm_ranges.development.json'), 0., jaw, side) for side in ('left','right')}
    arm = arms['left']
    config = parameters(ROOT, profile, geometry, arms, args.mobile_mesh_dir)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'robot.urdf').write_text(config['robot_description'])
    (args.output/'robot.srdf').write_text(config['robot_description_semantic'])
    state = RobotState()
    state.joint_state.name = [j.get('name') for j in ET.fromstring(config['robot_description']).findall('joint') if j.get('type') != 'fixed']
    state.joint_state.position = [0.]*len(state.joint_state.name)
    for side, model in arms.items():
        for name, value in profile['arms'][side]['reference_model_rad'].items():
            state.joint_state.position[state.joint_state.name.index(side+'_model_'+name)] = float(value)
    report = dict(mode='offline_real_moveit_synthetic_grasps', calibration_id=profile['calibration_id'],
                  hardware_commands=0, motion_authorized=False, physical_task_completed=False,
                  continuous_collision_proof=False, mtc_used=False, checks=[], routes=[])
    try:
        with session(config, state, args.output) as call:
            result = call('check_state_validity', GetStateValidity.Request(robot_state=state))
            report['initial_contacts'] = [(c.contact_body_1,c.contact_body_2) for c in result.contacts]
            if not result.valid:
                raise RuntimeError('nominal Home collides: '+str(report['initial_contacts']))
            report['checks'].append('nominal_home_robot_mesh_collision_free')
            from copy import deepcopy
            reference_state=deepcopy(state)
            for side in arms:
                for name,value in zip(geo.JOINTS[:5],np.deg2rad(CarryPolicy().reference_model_deg)):
                    reference_state.joint_state.position[reference_state.joint_state.name.index(side+'_model_'+name)]=float(value)
            reference_valid=call('check_state_validity',GetStateValidity.Request(robot_state=reference_state))
            if not reference_valid.valid:raise RuntimeError('common two-arm reference collides')
            report['checks'].append('two_arm_common_carry_reference_collision_free')
            # Verify both-arm FK in nonzero model coordinates, with current lift geometry.
            for side, model in arms.items():
                req = GetPositionFK.Request(robot_state=state, fk_link_names=[side+'_tcp'])
                req.header.frame_id='base_link'
                fk=call('compute_fk',req)
                if fk.error_code.val != 1: raise RuntimeError('MoveIt FK failed')
                p=fk.pose_stamped[0].pose
                expected=model.fk(model.anchor)[0]
                pos=np.array([p.position.x,p.position.y,p.position.z])
                rot=Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix()
                if np.linalg.norm(pos-expected[:3,3])>1e-8 or np.linalg.norm(rot-expected[:3,:3])>1e-8:
                    raise RuntimeError('current coordinate/MoveIt FK mismatch')
                report['checks'].append(side+'_official_fk_matches')
            floor = CollisionObject(id='floor', operation=CollisionObject.ADD)
            floor.header.frame_id='base_link'
            floor.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[4.,4.,.1])]
            floor_pose=np.eye(4); floor_pose[2,3]=profile['ground_z_m']-.051
            floor.primitive_poses=[MoveItCarryPlanner.pose(floor_pose)]
            scene=ApplyPlanningScene.Request(); scene.scene.is_diff=True
            scene.scene.world.collision_objects=[floor]
            if not call('apply_planning_scene',scene).success: raise RuntimeError('floor scene failed')
            backend = MoveItCarryPlanner(arm,state,call)
            # Absolute official model angles keep these synthetic fixtures
            # independent of the installation's recalibrated Home reference.
            for roll in (0., 30.):
                candidate = synthetic_grasp(arm, roll)
                pre = candidate.pregrasp_model_rad
                plan=plan_pick_carry(arm,backend,arm.anchor,[candidate],scene_id='fixture-scene')
                report['routes'].append(asdict(plan))
            if args.include_place:
                successful = plan
                if successful.status != 'OFFLINE_ROUTE_FOUND':
                    raise RuntimeError('pick/carry prerequisite failed')
                held = successful.resolved_grasp
                object_pose = held.base_from_grasp_tcp@held.tcp_from_object
                # A synthetic narrow support beneath the payload leaves the
                # stock fingertip sweep visible to the real collision checker.
                vertices = np.array(list(product(*[(-v/2,v/2) for v in held.object_size_m])))
                world = vertices@object_pose[:3,:3].T+object_pose[:3,3]
                top = float(world[:,2].min()-.0015)
                lo, hi = world[:,:2].min(axis=0)-.004, world[:,:2].max(axis=0)+.004
                support = CollisionObject(id='place_support',operation=CollisionObject.ADD)
                support.header.frame_id='base_link'
                support.primitives=[SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(hi[0]-lo[0]),float(hi[1]-lo[1]),.01])]
                support_pose=np.eye(4);support_pose[:3,3]=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,top-.005]
                support.primitive_poses=[backend.pose(support_pose)]
                scene.scene.world.collision_objects=[support]
                if not call('apply_planning_scene',scene).success: raise RuntimeError('place support scene failed')
                place=plan_place_carry(arm,backend,successful.stages[-1].end,held,
                    scene_id='fixture-scene',base_from_object=object_pose,
                    support_bounds_m=[[float(lo[0]),float(lo[1]),top],[float(hi[0]),float(hi[1]),top]],
                    support_observation_id='synthetic-support',support_observed_s=1.,now_s=1.1,
                    empty_transport_model_rad=arm.anchor,clearance_m=.02)
                report['place_route']=asdict(place)
                if place.status != 'OFFLINE_ROUTE_FOUND':
                    raise RuntimeError('place route failed: '+str(place.failures))
                report['checks'].append('place_lower_release_retreat_full_mesh_checked')
                if args.compile_task_programs:
                    from home_robot_tasks.assistant.alignment_contracts import TaskScene
                    from home_robot_tasks.assistant.contracts import Context, Stamp
                    from home_robot_tasks.task_route_program import compile_task_route
                    from home_robot_tasks.carry_program import _control
                    from home_robot_tasks.carry_trajectory import MotionBounds
                    ranges=read('arm_ranges.development.json')
                    minimum=[];maximum=[]
                    for side in ('left','right'):
                        for name in geo.JOINTS:
                            minimum.append(ranges['arms'][side][name]['minimum_urad']/1e6)
                            maximum.append(ranges['arms'][side][name]['maximum_urad']/1e6)
                    bounds=MotionBounds(tuple(minimum),tuple(maximum),(.3,)*12,(.6,)*12,(3.,)*12)
                    anchor=np.array([_control(profile,side,name,profile['arms'][side]['reference_model_rad'][name])
                                     for side in ('left','right') for name in geo.JOINTS])
                    context=Context('fixture-map','v1',profile['calibration_id'],'fixture-clock')
                    observed=TaskScene('fixture-object','fixture-table',Stamp(1.,context,'fixture-capture'),
                        'fixture-scene','fixture-model','fixture-pose',(0.,0.,0.),0.,
                        np.tile(world-[0,0,profile['ground_z_m']],(3,1)),top-profile['ground_z_m'],{'fixture':'a'*64})
                    open_control=_control(profile,'left','gripper',held.open_jaw_model_rad)
                    close_control=_control(profile,'left','gripper',held.held_jaw_model_rad)
                    pick_program=compile_task_route('pick',successful,scene=observed,arm=arm,anchor_control_rad=anchor,
                        bounds=bounds,open_control=open_control,close_control=close_control,
                        contact_control_interval=(close_control,close_control))
                    place_program=compile_task_route('place',place,scene=observed,arm=arm,
                        anchor_control_rad=pick_program.stages[-1].points[-1].positions_rad,bounds=bounds,
                        open_control=open_control,close_control=close_control,contact_control_interval=(close_control,close_control))
                    report['task_programs']=[]
                    # Remove the support while validating the pick; it was added
                    # after the synthetic pick so would otherwise be a new obstacle.
                    support.operation=CollisionObject.REMOVE
                    scene.scene.world.collision_objects=[support]
                    if not call('apply_planning_scene',scene).success:raise RuntimeError('task-program support cleanup failed')
                    from home_robot_tasks.assistant.alignment_contracts import HeldObject
                    from home_robot_tasks.mobile_route_planning import CaptureRouteInput
                    from home_robot_tasks.loaded_carry_planning import LoadedSnapshot, plan_measured_carry
                    measured_attachment=successful.resolved_grasp.tcp_from_object.copy()
                    measured_attachment[0,3]+=.008  # measured shift must trigger a newly checked tail
                    measured=HeldObject('fixture-object',successful.resolved_grasp.candidate_id,context,'fixture-model',1.,
                        measured_attachment,held.object_size_m,.001)
                    data=CaptureRouteInput(arm,backend,bounds,pick_program.stages[4].points[-1].positions_rad,None,
                        {},{},held=measured,held_candidate=successful.resolved_grasp,open_control=open_control,
                        hold_control=close_control,contact_control_interval=(close_control,close_control))
                    loaded_program=plan_measured_carry(LoadedSnapshot(data,observed,measured,'fixture-epoch',1.))
                    for program in (pick_program,loaded_program,place_program):
                        if program.operation=='place':
                            support.operation=CollisionObject.ADD;scene.scene.world.collision_objects=[support]
                            if not call('apply_planning_scene',scene).success:raise RuntimeError('task-program support restore failed')
                        candidate_for_check=program.resolved_grasp
                        total=0
                        for stage in program.stages:
                            backend.prepare(candidate_for_check,stage.attached)
                            for point in stage.points:
                                q=np.array([geo.model_from_control(float(point.positions_rad[i]),profile['arms']['left']['alignment'][n])
                                            for i,n in enumerate(geo.JOINTS)])
                                backend.valid(q[:5],candidate_for_check,stage.attached,jaw=q[5]);total+=1
                        report['task_programs'].append(dict(operation=program.operation,stages=[s.name for s in program.stages],
                            checked_wire_points=total,physical_task_completed=False))
                    report['checks'].append('new_pick_and_place_programs_retimed_and_full_mesh_rechecked')

                support.operation=CollisionObject.REMOVE
                scene.scene.world.collision_objects=[support]
                if not call('apply_planning_scene',scene).success: raise RuntimeError('support cleanup failed')
            # Explicit upright task: the object stays upright although the TCP is tilted.
            upright = synthetic_grasp(arm, 30., upright=True)
            result=plan_pick_carry(arm,backend,arm.anchor,[upright],scene_id='fixture-scene')
            report['routes'].append(asdict(result))
            # Same carry pose with an oversized attached object must fail, even
            # though the unladen arm was valid. Only finger contact is allowed.
            huge=replace(candidate,candidate_id='oversized-payload',object_size_m=(1.,1.,1.))
            backend.prepare(huge,True)
            try:
                backend.valid(pre,huge,True)
            except PlanRejected:
                report['checks'].append('attached_payload_collision_rejected')
            else:
                raise RuntimeError('attached payload collision was ignored')
            # Environment persists across target/attachment scene updates.
            obstacle=CollisionObject(id='blocked_goal',operation=CollisionObject.ADD)
            obstacle.header.frame_id='base_link'
            obstacle.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.1,.1,.1])]
            obstacle.primitive_poses=[backend.pose(arm.fk(pre)[0])]
            scene.scene.world.collision_objects=[obstacle]
            if not call('apply_planning_scene',scene).success: raise RuntimeError('obstacle scene failed')
            backend.prepare(candidate,False)
            try:
                backend.valid(pre,candidate,False)
            except PlanRejected:
                report['checks'].append('environment_collision_rejected_after_scene_update')
            else:
                raise RuntimeError('environment collision was ignored')
            report['checked_states']=backend.checked_states
            report['passed']=all(p['status']=='OFFLINE_ROUTE_FOUND' for p in report['routes'])
    except Exception as exc:
        report['passed']=False; report['error']=str(exc)
        import traceback
        report['error_traceback']=traceback.format_exc()
    (args.output/'report.json').write_text(json.dumps(report,default=serial,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('routes','place_route')},default=serial))
    for route in report['routes']:
        print(route['status'], route['failures'])
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
