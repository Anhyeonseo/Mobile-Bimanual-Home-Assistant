#!/usr/bin/env python3
"""Bounded common carry-reference comparison using native offline MoveIt.

One observed remote attachment plus explicit synthetic shape/orientation cases.
No inference of graspability, measured mass, motor capacity or driving stability.
No robot transport/controller is opened. Reference poses are model coordinates.
"""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from tools.diagnostics.plan_capture_carry import load_models,serial
from tools.diagnostics.carry_moveit_session import session
from home_robot_tasks.carry_model import parameters
from home_robot_tasks.carry_planning import GraspRouteCandidate, PlanRejected
from home_robot_tasks.carry_posture import carry_goals,ObjectAxisConstraint,CarryPolicy
from home_robot_tasks.carry_moveit import MoveItCarryPlanner
from home_robot_tasks.carry_load import CarriedGeometry,payload_load
from home_robot_tasks.stationary_pick_sim import StationaryArm

REFERENCES={'low_home':[0,-90,70,90,90], 'low_fold':[0,-75,50,90,90],
            'raised_fold':[0,-70,35,90,90], 'high_fold':[0,-60,25,90,90],
            'forward_high':[0,-45,10,90,90]}

def fixtures(arm,report):
    source=report['plan']['resolved_grasp'];source=dict(source)
    for k in ('pregrasp_model_rad','base_from_grasp_tcp','tcp_from_object'):source[k]=np.array(source[k])
    remote=GraspRouteCandidate(**source)
    lifted=np.array(next(s for s in report['plan']['stages'] if s['label']=='lift')['joints'][-1])
    yield 'observed_remote',remote,lifted
    # Same reachable lifted TCP, new explicitly synthetic carried envelopes.
    # These transport fixtures do not claim that the preceding grasps succeeded.
    for name,size,upright,offset in [
        ('small_box',(.05,.04,.06),False,0.),
        ('upright_cup',(.05,.05,.09),True,0.),
        ('upright_bottle',(.045,.045,.15),True,0.),
        ('long_centered',(.25,.025,.025),False,0.),
        ('long_offset',(.25,.025,.025),False,.06)]:
        relative=remote.tcp_from_object.copy()
        if upright:relative[:3,:3]=arm.fk(lifted)[0][:3,:3].T
        relative[:3,3]+=relative[:3,0]*offset
        constraint=ObjectAxisConstraint(relative,evidence='synthetic upright carry fixture') if upright else None
        candidate=replace(remote,candidate_id=name,base_from_grasp_tcp=arm.fk(lifted)[0],
            tcp_from_object=relative,object_size_m=size,payload_mesh=None,object_constraint=constraint)
        yield name,candidate,lifted

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared',type=Path,required=True);p.add_argument('--previous-report',type=Path,required=True)
    p.add_argument('--mobile-mesh-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--references',nargs='+',choices=REFERENCES,default=list(REFERENCES))
    p.add_argument('--adaptive',action='store_true',help='validate selected default family instead of comparing fixed references')
    a=p.parse_args();prepared=json.loads(a.prepared.read_text());previous=json.loads(a.previous_report.read_text())
    if previous['capture_id']!=prepared['capture_id'] or previous['source_sha256']!=prepared['source_sha256']:
        raise ValueError('remote report and capture mismatch')
    profile,geometry,arms,_,_,_,_=load_models(prepared,nominal_obstacles=True,obstacle_padding_m=0)
    joints=previous['initial_model_joint_positions_si'];lift=joints['s2_lift_joint']
    ranges=json.loads((ROOT/'config/home_assistant/arm_ranges.development.json').read_text())
    arms={side:StationaryArm(profile,geometry,ranges,lift,arms[side].jaw,side) for side in arms}
    arm=arms['left'];config=parameters(ROOT,profile,geometry,arms,a.mobile_mesh_dir)
    from moveit_msgs.msg import RobotState
    state=RobotState();state.joint_state.name=list(joints);state.joint_state.position=list(joints.values())
    references={'adaptive_common':CarryPolicy().reference_model_deg} if a.adaptive else {n:REFERENCES[n] for n in a.references}
    policy=CarryPolicy() if a.adaptive else CarryPolicy(exploration_offsets_deg=(),maximum_results=4)
    report=dict(mode='offline_common_carry_comparison',hardware_commands=0,motion_authorized=False,
        physical_task_completed=False,environment='robot_only_no_furniture',lift_m=lift,
        initial_model_joint_positions_si=joints,capture_id=prepared['capture_id'],
        source_sha256=prepared['source_sha256'],references=references,policy=asdict(policy),cases=[],results=[])
    a.output.mkdir(parents=True,exist_ok=True)
    with session(config,state,a.output) as call:
        backend=MoveItCarryPlanner(arm,state,call)
        for name,candidate,start in fixtures(arm,previous):
            report['cases'].append(dict(name=name,candidate=asdict(candidate),start=start,
                evidence='observed_attachment' if name=='observed_remote' else 'synthetic_transport_fixture'))
            payload=CarriedGeometry(candidate.tcp_from_object,candidate.object_size_m,candidate.payload_mesh)
            for reference,angles in references.items():
                goals=carry_goals(arm,start,candidate.base_from_grasp_tcp,reference_model_rad=np.deg2rad(angles),
                    object_constraint=candidate.object_constraint,payload=payload,
                    policy=policy)
                result=dict(case=name,reference=reference,status='NO_ROUTE',failures=[],alternatives=[])
                for goal in goals:
                    try:
                        backend.prepare(candidate,True);backend.valid(goal.model_rad,candidate,True)
                        clearance=backend.payload_clearance(goal.model_rad,candidate)
                        load=payload_load(arm,goal.model_rad,payload)
                        merit=goal.cost+.4*(1-clearance['passed_margin_m']/.02)
                        result['alternatives'].append(dict(goal=asdict(goal),load=load,clearance=clearance,merit=merit))
                    except PlanRejected as exc:result['failures'].append(str(exc))
                for alternative in sorted(result['alternatives'],key=lambda x:x['merit']):
                    try:
                        route=backend.connect(start,np.array(alternative['goal']['model_rad']),candidate,attached=True,label='carry')
                        result.update(status='OFFLINE_ROUTE_FOUND',selected=alternative,path=asdict(route))
                        break
                    except PlanRejected as exc:result['failures'].append(str(exc))
                report['results'].append(result)
                (a.output/'report.json').write_text(json.dumps(report,default=serial,indent=2)+'\n')
                selected=result.get('selected',{})
                print(json.dumps(dict(case=name,reference=reference,status=result['status'],
                    shoulder_cm=100*selected.get('load',{}).get('shoulder_horizontal_lever_m',0),
                    clearance_mm=1000*selected.get('clearance',{}).get('passed_margin_m',0))),flush=True)
        report['checked_states']=backend.checked_states
    (a.output/'report.json').write_text(json.dumps(report,default=serial,indent=2)+'\n')
    return 0

if __name__=='__main__':raise SystemExit(main())
