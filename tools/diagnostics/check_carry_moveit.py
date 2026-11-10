#!/usr/bin/env python3
"""Exercise current nominal robot + dynamic carry through real offline MoveIt.

Synthetic grasp poses validate planning integration, NOT perception performance.
No device/SSH/controller is opened. Source ROS Jazzy before running.
"""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'ros2_ws/src/home_robot_tasks'), str(ROOT/'ros2_ws/src/so101_arm_bridge'), str(ROOT)]
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.carry_model import parameters
from home_robot_tasks.carry_planning import GraspRouteCandidate, plan_pick_carry, PlanRejected
from home_robot_tasks.carry_posture import ObjectAxisConstraint,CarryPolicy
from home_robot_tasks.carry_moveit import MoveItCarryPlanner
from home_robot_tasks.stationary_pick_sim import StationaryArm
from home_robot_tasks.so101_jaw import load_stock_jaw
from carry_moveit_session import session


def serial(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    raise TypeError(type(value).__name__)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mobile-mesh-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'output/home-assistant/s3/carry-moveit')
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
                pre=np.deg2rad([0.,-45.,45.,60.,90.+roll])
                pose=arm.fk(pre)[0]
                pose[:3,3] += .006*pose[:3,2]
                attachment=np.eye(4); attachment[:3,3]=[-.018,0.,0.]
                candidate=GraspRouteCandidate('fixture-roll-'+str(roll),profile['calibration_id'],'fixture-scene',pre,
                    pose,attachment,(.01,.012,.012),.5,.3)
                plan=plan_pick_carry(arm,backend,arm.anchor,[candidate],scene_id='fixture-scene')
                report['routes'].append(asdict(plan))
            # Explicit upright task: the object stays upright although the TCP is tilted.
            upright_attachment=attachment.copy()
            upright_attachment[:3,:3]=pose[:3,:3].T
            upright=replace(candidate,candidate_id='upright-fixture',tcp_from_object=upright_attachment,
                object_constraint=ObjectAxisConstraint(upright_attachment,maximum_tilt_rad=np.deg2rad(10),
                                                       evidence='synthetic upright task requirement'))
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
    (args.output/'report.json').write_text(json.dumps(report,default=serial,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('routes',)},default=serial))
    for route in report['routes']:
        print(route['status'], route['failures'])
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
