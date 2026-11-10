#!/usr/bin/env python3
"""Compile and rehearse a capture-bound carry program; no hardware execution."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks'), str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.carry_program import compile_program, manifest
from home_robot_tasks.carry_trajectory import MotionBounds
from home_robot_tasks.stationary_pick_sim import StationaryArm
from tools.diagnostics.plan_capture_carry import load_models, serial
from tools.diagnostics.carry_stream_sim import simulate


def load_inputs(report_path, prepared_path):
    report = json.loads(report_path.read_text())
    if hashlib.sha256(prepared_path.read_bytes()).hexdigest() != report['prepared_sha256']:
        raise ValueError('prepared capture changed after planning')
    prepared = json.loads(prepared_path.read_text())
    if prepared['source_sha256'] != report['source_sha256'] or prepared['capture_id'] != report['capture_id']:
        raise ValueError('capture provenance mismatch')
    profile, geometry, arms, installed, boxes, payload, joints = load_models(
        prepared, nominal_obstacles=report['obstacle_envelope'] == 'point_bounds',
        obstacle_padding_m=report['obstacle_padding_mm']/1000,jaw_basis=report.get('jaw_basis','measured_front'))
    ranges = json.loads((ROOT/'config/home_assistant/arm_ranges.development.json').read_text())
    lift = report['initial_model_joint_positions_si']['s2_lift_joint']
    arms = {side: StationaryArm(profile, geometry, ranges, lift, arm.jaw, side) for side, arm in arms.items()}
    minimum, maximum = [], []
    for side in ('left', 'right'):
        for name in geo.JOINTS:
            spec = ranges['arms'][side][name]
            # Match the MCU's integer envelope exactly at range boundaries.
            minimum.append(spec['minimum_urad']/1e6)
            maximum.append(spec['maximum_urad']/1e6)
    # Development command-profile limits, not measured motor performance.
    bounds = MotionBounds(tuple(minimum), tuple(maximum),
                          (.3,)*5+(.08,)+(.3,)*5+(.08,),
                          (.6,)*5+(.2,)+(.6,)*5+(.2,),
                          (3.,)*5+(1.,)+(3.,)*5+(1.,))
    program = compile_program(report, profile, installed, arms['left'], bounds=bounds)
    return program, (report, profile, geometry, arms, boxes)


def validate_native(program, inputs, mesh_dir, output):
    from home_robot_tasks.grasp_preferences import TabletopGraspPolicy, contact_posture
    from moveit_msgs.msg import RobotState
    from home_robot_tasks.carry_model import parameters
    from home_robot_tasks.carry_moveit import MoveItCarryPlanner
    from home_robot_tasks.carry_planning import GraspRouteCandidate
    from home_robot_tasks.carry_posture import ObjectAxisConstraint
    from home_robot_tasks.carry_scene import apply_boxes
    from tools.diagnostics.carry_moveit_session import session
    report, profile, geometry, arms, boxes = inputs
    surface_audit=audit_native_surfaces(program,profile,arms['left'],boxes,
                                      approach_mode=report.get('approach_mode','tool_axis'))
    posture_spec=report.get('grasp_posture_policy')
    posture_policy=TabletopGraspPolicy(**posture_spec) if posture_spec else None
    joints = report['initial_model_joint_positions_si']
    state = RobotState()
    state.joint_state.name = list(joints)
    state.joint_state.position = list(joints.values())
    candidate = dict(report['plan']['resolved_grasp'])
    for name in ('pregrasp_model_rad', 'base_from_grasp_tcp', 'tcp_from_object'):
        candidate[name] = np.asarray(candidate[name])
    if candidate['object_constraint']:
        candidate['object_constraint'] = ObjectAxisConstraint(**candidate['object_constraint'])
    candidate = GraspRouteCandidate(**candidate)
    config = parameters(ROOT, profile, geometry, arms, mesh_dir)
    output.mkdir(parents=True, exist_ok=True)
    results = []
    with session(config, state, output) as call:
        apply_boxes(call, boxes)
        planner = MoveItCarryPlanner(arms['left'], state, call)
        for stage in program.stages:
            planner.prepare(candidate, stage.attached)
            before = planner.checked_states
            previous = None
            for point in stage.points:
                # Validate the actual integer micro-radian wire representation,
                # including extra numerical stage bridges and the probe split.
                controls = np.round(point.positions_rad, 6)
                model = np.array([geo.model_from_control(float(controls[i]), profile['arms']['left']['alignment'][n])
                                  for i, n in enumerate(geo.JOINTS)])
                if previous is not None and np.max(abs(model-previous)) > np.deg2rad(1):
                    raise ValueError('retimed sampling gap exceeds previous collision resolution')
                previous = model
                if posture_policy and stage.name in ('approach','close','probe','lift'):
                    if not contact_posture(arms['left'].fk(model[:5])[0],posture_policy)['accepted']:
                        raise ValueError('wire contact/lift route exceeds tabletop posture envelope')
                # Rigid contact blocks the physical finger at observed width.
                # The smaller command is pressure intent, not a smaller object.
                jaw = max(model[5], candidate.held_jaw_model_rad) if stage.name in ('close','probe','lift','carry','hold') else model[5]
                planner.valid(model[:5], candidate, stage.attached, jaw=jaw)
            results.append(dict(stage=stage.name, checked_states=planner.checked_states-before))
        # Negative control: a new independent obstacle intersecting the held
        # object must still fail. This never changes the recorded scene or
        # permits a collision; the temporary local object is removed afterward.
        from moveit_msgs.msg import CollisionObject
        from moveit_msgs.srv import ApplyPlanningScene
        from shape_msgs.msg import SolidPrimitive
        from home_robot_tasks.carry_planning import PlanRejected
        point=(candidate.payload_mesh['vertices'][0] if candidate.payload_mesh else [0.,0.,0.])
        payload_pose=arms['left'].fk(model[:5])[0]@candidate.tcp_from_object
        obstacle_pose=np.eye(4);obstacle_pose[:3,3]=(payload_pose@np.r_[point,1])[:3]
        obstacle=CollisionObject(id='validation_independent_obstacle',operation=CollisionObject.ADD)
        obstacle.header.frame_id='base_link'
        obstacle.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.01]*3)]
        obstacle.primitive_poses=[planner.pose(obstacle_pose)]
        request=ApplyPlanningScene.Request();request.scene.is_diff=True
        request.scene.world.collision_objects=[obstacle]
        if not call('apply_planning_scene',request).success:raise ValueError('negative-control obstacle insertion failed')
        rejected=False
        try:
            try:planner.valid(model[:5],candidate,True,jaw=jaw)
            except PlanRejected as exc:
                rejected=any(obstacle.id in c['bodies'] for c in getattr(exc,'diagnostics',{}).get('contacts',[]))
            if not rejected:raise ValueError('independent obstacle was not rejected')
        finally:
            obstacle.operation=CollisionObject.REMOVE
            if not call('apply_planning_scene',request).success:raise ValueError('negative-control obstacle removal failed')
        planner.valid(model[:5],candidate,True,jaw=jaw)
    return dict(status='PASSED', stages=results, continuous_collision_proof=False,
                independent_obstacle_rejected=rejected,scene_restored_and_rechecked=True,
                stock_finger_surfaces=surface_audit)


def audit_native_surfaces(program,profile,arm,boxes, *, approach_mode='tool_axis'):
    """Reuse the verified STL, checking wire FK and actual commanded jaw sweep.

    The plane is extended only for local descent/closure/lift acceptance. A
    global plane test would wrongly prohibit reaching from below a table edge.
    Positive nominal clearance does not prove real-robot height calibration.
    """
    from home_robot_tasks.carry_cartesian import validate_vertical_descent
    if approach_mode not in ('tool_axis','vertical'):raise ValueError('unknown approach mode')
    box=next(b for b in boxes if b.role=='support')
    support=box.base_pose.copy()
    support[:3,3]+=support[:3,2]*box.size_m[2]/2
    results=[]
    for stage in program.stages:
        minima={name:dict(clearance_m=float('inf')) for name in ('fixed','moving')}
        poses=[]
        for index,point in enumerate(stage.points):
            controls=np.round(point.positions_rad,6)
            model=np.array([geo.model_from_control(float(controls[i]),profile['arms']['left']['alignment'][n])
                            for i,n in enumerate(geo.JOINTS)])
            pose=arm.fk(model[:5])[0];poses.append(pose)
            evidence=arm.jaw.plane_clearance(pose@arm.tcp_from_stock,model[5],support)
            for name,record in evidence.items():
                if record['clearance_m']<minima[name]['clearance_m']:
                    minima[name]=dict(record,point_index=index,offset_ms=point.offset_ms)
        row=dict(stage=stage.name,samples=len(stage.points),minimum_by_finger=minima)
        if approach_mode=='vertical':
            if stage.name=='approach':row['vertical_descent']=validate_vertical_descent(poses)
            if stage.name in ('approach','close','probe','lift') and min(v['clearance_m'] for v in minima.values())<=0:
                raise ValueError('stock finger surface crosses the support plane during '+stage.name)
        results.append(row)
    return dict(basis='verified_official_STL_surface_vertices',base_from_support=support.tolist(),
                approach_mode=approach_mode,stages=results,physical_accuracy_verified=False,
                commanded_jaw_including_preload=True,continuous_collision_proof=False,
                note='signed distance to extended observed plane; real arm/table discrepancy remains unmeasured')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--prepared', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--mobile-mesh-dir', type=Path, help='also recheck retimed/rounded geometry with native MoveIt')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    program, inputs = load_inputs(args.report, args.prepared)
    summary = manifest(program)
    summary['model_assumptions'] = {key: inputs[0].get(key) for key in (
        'environment_coverage','obstacle_envelope','obstacle_padding_mm','continuous_collision_proof')}
    summary['native_geometry'] = (validate_native(program, inputs, args.mobile_mesh_dir, args.output/'moveit')
                                  if args.mobile_mesh_dir else dict(status='NOT_RUN'))
    cases = (None, 'no_follow', 'slip', 'empty_close', 'timeout', 'dispatch', 'stale',
             'cancel', 'base_moved', 'stop_unconfirmed')
    summary['rehearsals'] = [simulate(program, fault=fault) for fault in cases]
    expected = ['HOLDING']+['FAILED']*6+['CANCELLED','FAILED','STOPPING']
    summary['simulation_passed'] = [r['state'] for r in summary['rehearsals']] == expected
    summary['status'] = 'OFFLINE_PREFLIGHT_PASSED' if summary['simulation_passed'] else 'OFFLINE_PREFLIGHT_FAILED'
    (args.output/'report.json').write_text(json.dumps(summary, default=serial, indent=2)+'\n')
    (args.output/'program.json').write_text(json.dumps(asdict(program), default=serial, indent=2)+'\n')
    print(json.dumps({k: summary[k] for k in ('status','hypothetical_lift','candidate_source','native_geometry')}, default=serial))
    print(json.dumps([(r['fault'], r['state'], r['reason']) for r in summary['rehearsals']]))
    return 0 if summary['simulation_passed'] else 2


if __name__ == '__main__': raise SystemExit(main())
