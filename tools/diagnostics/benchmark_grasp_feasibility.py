#!/usr/bin/env python3
"""Fresh generators -> identical capture-bound SO101/MoveIt checks. No execution."""
import argparse
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.diagnostics.replay_s3_grasp import load_scene
from tools.diagnostics.plan_capture_carry import serial
from tools.diagnostics.carry_moveit_session import session
from home_perception.geometry.mask_cloud import measure
from home_perception.geometry.plane import Plane
from home_perception.grasp import graspnet
from home_perception.grasp.so101 import candidates as so101_candidates
from home_perception.grasp.handoff import observed_contact_line
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.carry_binding import bind_grasp
from home_robot_tasks.carry_model import parameters
from home_robot_tasks.carry_scene import apply_boxes, from_prepared_capture
from home_robot_tasks.carry_moveit import MoveItCarryPlanner
from home_robot_tasks.carry_planning import plan_pick_carry
from home_robot_tasks.grasp_preferences import TabletopGraspPolicy
from home_robot_tasks.jaw_closure import closure_plan
from home_robot_tasks.installed_jaw import StockLeftJaw
from home_robot_tasks.stationary_pick_sim import StationaryArm
from home_robot_tasks.so101_jaw import load_stock_jaw


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, default=serial, indent=2, allow_nan=False))


def verify_inputs(snapshot, prepared):
    if prepared['capture_id'] != snapshot.name:
        raise ValueError('capture identity mismatch')
    for name in ('color.png', 'depth_aligned.png', 'metadata.json'):
        if hashlib.sha256((snapshot/name).read_bytes()).hexdigest() != prepared['source_sha256'][name]:
            raise ValueError('snapshot digest mismatch: '+name)
    if hashlib.sha256((snapshot/'mask.png').read_bytes()).hexdigest() != prepared['group']['mask_sha256']:
        raise ValueError('mask digest mismatch')


def capture_models(prepared, directory):
    """Historical camera/arm calibration; explicit nominal CAD jaw for both methods."""
    bundle=json.loads((directory/'bundle.json').read_text())
    data=(directory/'model.json').read_bytes()
    if hashlib.sha256(data).hexdigest()!=bundle['files_sha256']['model.json']:
        raise ValueError('historical model digest mismatch')
    model=json.loads(data);profile=model['profile']
    if profile['calibration_id']!=prepared['review']['calibration_id']:
        raise ValueError('historical model/capture calibration mismatch')
    for name,digest in prepared['source_sha256'].items():
        if bundle['files_sha256'].get(name)!=digest:
            raise ValueError('model bundle belongs to another snapshot')
    data=(directory/'ranges.json').read_bytes()
    if hashlib.sha256(data).hexdigest()!=model['range_sha256']:
        raise ValueError('historical joint range digest mismatch')
    ranges=json.loads(data)
    boxes,payload,joints=from_prepared_capture(prepared,profile)
    geometry=geo.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    jaw=load_stock_jaw(ROOT/'config/home_assistant/official_sources/so101_new_calib.urdf',
                       ROOT/'ros2_ws/src/so101_description/meshes')
    arms={side:StationaryArm(profile,geometry,ranges,joints['s2_lift_joint'],jaw,side)
          for side in ('left','right')}
    if not np.allclose(arms['left'].mount,model['fk']['mount']) or not np.allclose(arms['left'].tcp,model['fk']['tcp']):
        raise ValueError('historical FK model differs')
    installed=StockLeftJaw(jaw,profile,ranges['arms']['left']['gripper'])
    return profile,geometry,arms,installed,boxes,payload,joints


def common_sections(group, points, jaw):
    """Same evidence/entry-width policy for both generators, no score comparison."""
    result = copy.deepcopy(group)
    kept, rejected = [], []
    for index, c in enumerate(result['refined_grasps']):
        section = observed_contact_line(c, points)
        c['closing_section'] = section
        if section['status'] == 'VISIBLE_SPAN':
            c['width_m'] = section['high_m']-section['low_m']
        check = closure_plan(jaw, c)
        if check['status'] != 'BOUNDED_CLOSE':
            rejected.append(dict(index=index, reason=check['status']))
            continue
        c['common_input_index'] = index
        kept.append(c)
    result['refined_grasps'] = kept
    result['common_checks'] = dict(rejected=rejected, passed=len(kept))
    return result


def bound_prepared(template, group):
    """Replace generator data, preserving only verified capture/scene identity."""
    result = copy.deepcopy(template)
    for key in ('capture_id', 'observation_index', 'label', 'source_sha256', 'coordinate_frame',
                'perception_status', 'session_id', 'calibration_id', 'mask_sha256'):
        group[key] = copy.deepcopy(template['group'][key])
    result['group'] = group
    result['candidate_source'] = group['benchmark_backend']
    for key in ('candidate_diagnostics', 'bounded_closure_candidates', 'measured_closure_candidates',
                'first_measured_closure_source', 'first_width_fit_source', 'planned_source', 'path'):
        result.pop(key, None)
    return result


def scan_candidates(count, bind, plan, *, exhaustive=False, on_attempt=None):
    """Same index/sign ordering and stop rule for both methods; failures stay typed."""
    attempts = []
    start = perf_counter()
    first_ms = None
    for index in range(count):
        for sign in (1, -1):
            row = dict(index=index, jaw_sign=sign)
            tick = perf_counter()
            try:
                candidate, preview = bind(index, sign)
            except ValueError as exc:
                row.update(status='BINDING_REJECTED', reason=str(exc),
                           diagnostics=getattr(exc, 'diagnostics', None),
                           binding_ms=1000*(perf_counter()-tick), route_ms=0.)
            else:
                row.update(binding_ms=1000*(perf_counter()-tick), jaw_preview=preview)
                tick = perf_counter()
                route = plan(candidate)
                row.update(route_ms=1000*(perf_counter()-tick), status=route.status,
                           plan=asdict(route))
                if route.status == 'OFFLINE_ROUTE_FOUND' and first_ms is None:
                    first_ms = 1000*(perf_counter()-start)
            attempts.append(row)
            if on_attempt:
                on_attempt(row)
            if first_ms is not None and not exhaustive:
                break
        if first_ms is not None and not exhaustive:
            break
    success = sum(a['status']=='OFFLINE_ROUTE_FOUND' for a in attempts)
    complete = len(attempts)==2*count
    return dict(status='OFFLINE_ROUTE_FOUND' if success else 'NO_COMPLETE_ROUTE',
        attempts=attempts, tested_signed_candidates=len(attempts), available_signed_candidates=2*count,
        bound_signed_candidates=sum(a['status']!='BINDING_REJECTED' for a in attempts),
        feasible_signed_candidates=success, full_pool_evaluated=complete,
        full_pool_feasible_fraction=success/(2*count) if complete and count else None,
        first_feasible_validation_ms=first_ms, validation_ms=1000*(perf_counter()-start),
        motion_authorized=False, hardware_commands=0)


def validate(prepared, output, mesh_dir, model_dir, *, exhaustive=False,
             acceleration='baseline', planning_budget_s=5., planning_attempts=2, verify_batch=False,
             request_window=8):
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    started = perf_counter()
    profile, geometry, arms, installed, boxes, payload, joints = capture_models(prepared, model_dir)
    arm = arms['left']; policy = TabletopGraspPolicy()
    start = np.array([joints['left_model_'+n] for n in geo.JOINTS[:5]])
    state = RobotState(); state.joint_state.name=list(joints); state.joint_state.position=list(joints.values())
    config = parameters(ROOT, profile, geometry, arms, mesh_dir)
    output.mkdir(parents=True, exist_ok=True)
    metrics = []
    with session(config, state, output, acceleration=acceleration, verify_batch=verify_batch,
                 request_window=request_window) as call:
        apply_boxes(call, boxes)
        valid = call('check_state_validity', GetStateValidity.Request(robot_state=state))
        setup_ms = 1000*(perf_counter()-started)
        if not valid.valid:
            return dict(status='INVALID_INITIAL_STATE', setup_ms=setup_ms,
                contacts=[(c.contact_body_1,c.contact_body_2) for c in valid.contacts],
                first_feasible_validation_ms=None, hardware_commands=0)
        def bind(index, sign):
            return bind_grasp(arm, prepared['review'], prepared['group'], index, payload, installed,
                              jaw_sign=sign, grasp_policy=policy)
        def plan(candidate):
            backend = MoveItCarryPlanner(arm, state, call, planning_budget_s=planning_budget_s,
                                         planning_attempts=planning_attempts)
            stages = []
            def timed_stage(name, method):
                def invoke(*args, **kwargs):
                    begin = perf_counter()
                    status = 'ERROR'
                    try:
                        result = method(*args, **kwargs)
                        status = 'OK'
                        return result
                    finally:
                        stages.append(dict(stage=kwargs.get('label', name), status=status,
                                           wall_ms=1000*(perf_counter()-begin)))
                return invoke
            for name in ('open', 'connect', 'cartesian', 'close', 'payload_clearance'):
                setattr(backend, name, timed_stage(name, getattr(backend, name)))
            tick = perf_counter()
            try:
                route = plan_pick_carry(arm, backend, start, [candidate], scene_id=prepared['capture_id'],
                                      maximum_grasps=1, grasp_policy=policy, stop_after_first=True)
                return route
            finally:
                metrics.append(dict(candidate_id=candidate.candidate_id,
                    wall_ms=1000*(perf_counter()-tick), services=backend.timings,
                    stages=stages, checked_states=backend.checked_states))
        def progress(row):
            print(f"  candidate {row['index']} sign {row['jaw_sign']}: {row['status']}", flush=True)
            with (output/'attempts.jsonl').open('a') as stream:
                stream.write(json.dumps(row,default=serial,allow_nan=False)+'\n')
        report = scan_candidates(len(prepared['group']['refined_grasps']), bind, plan,
                                 exhaustive=exhaustive, on_attempt=progress)
        report.update(setup_ms=setup_ms, acceleration=acceleration,
                      planning_budget_s=planning_budget_s, planning_attempts=planning_attempts,
                      reference_batch_verification=verify_batch,
                      request_window=request_window if acceleration == 'pipelined' else None,
                      planner_metrics=metrics, collision_proof='sampled_states_only',
                      environment_coverage='observed_only', physical_grasp_success_verified=False)
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, required=True)
    parser.add_argument('--mobile-mesh-dir', type=Path, required=True)
    parser.add_argument('--inputs', type=Path, default=ROOT/'output/so101_photo_experiment')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'output/so101_photo_experiment/fair-comparison')
    parser.add_argument('--maximum-candidates', type=int, choices=range(1,33), default=8)
    parser.add_argument('--repeats', type=int, choices=range(1,11), default=3)
    parser.add_argument('--exhaustive', action='store_true', help='test the whole signed pool for a feasible fraction')
    parser.add_argument('--check-only', action='store_true', help='verify inputs/models without GPU or ROS services')
    args = parser.parse_args()
    if args.output_dir.exists() and not args.check_only:
        parser.error('output directory already exists; use a new --output-dir')
    cases=[]
    for number,capture in [(1,'rgbd-99'),(2,'rgbd-103')]:
        snapshot=args.inputs/f'set-{number}'/capture
        template=json.loads((args.inputs/f'set-{number}'/'comparison/archive.json').read_text())
        verify_inputs(snapshot,template)
        models=capture_models(template,args.inputs/f'set-{number}'/'capture-model')
        meta,depth,scene,_=load_scene(snapshot)
        p=template['support_plane']
        plane=Plane(np.array(p['normal']),np.array(p['point_m']),p['rms_m'],p['inlier_fraction'])
        mask=np.asarray(Image.open(snapshot/'mask.png'))>0
        observed=measure(mask,depth,meta['aligned_depth_intrinsics'],plane)
        if observed.status!='accepted':raise ValueError('target geometry not accepted: '+str(observed.reason))
        cases.append((number,template,models,meta,depth,scene,plane,mask,observed))
        print(f'Capture {capture}: hashes, robot state and scene verified; {len(observed.points)} cleaned points',flush=True)
    if args.check_only:
        print('CHECK_OK (no GPU inference or MoveIt executed)');return
    import torch
    if not torch.cuda.is_available():parser.exit(2,'CUDA unavailable: run the shell launcher in your normal terminal.\n')
    graspnet.ARTIFACTS=args.artifacts.resolve();checkpoint=graspnet.ARTIFACTS/'models/checkpoint-rs.tar'
    torch.cuda.synchronize();load_start=perf_counter()
    if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=graspnet.CHECKPOINT_SHA256:
        raise ValueError('checkpoint digest mismatch')
    model_type,decode,detector=graspnet._baseline()
    net=model_type(input_feature_dim=0,num_view=300,num_angle=12,num_depth=4,cylinder_radius=.05,
                   hmin=-.02,hmax_list=[.01,.02,.03,.04],is_training=False)
    net.load_state_dict(torch.load(checkpoint,map_location='cpu',weights_only=True)['model_state_dict'],strict=True)
    net.cuda().eval();torch.cuda.synchronize();load_ms=1000*(perf_counter()-load_start)
    original=graspnet.infer
    def cached(points, _checkpoint):
        indices=np.random.default_rng(0).choice(len(points),20000,replace=len(points)<20000)
        torch.manual_seed(0)
        cloud=torch.as_tensor(points[indices][None],dtype=torch.float32,device='cuda')
        with torch.inference_mode():rows=decode(net({'point_clouds':cloud}))[0].cpu().numpy()
        return rows,detector
    graspnet.infer=cached
    args.output_dir.mkdir(parents=True)
    report=dict(mode='fresh_generator_common_robot_validation',gpu=torch.cuda.get_device_name(0),
        model_load_ms=load_ms,maximum_candidates=args.maximum_candidates,exhaustive=args.exhaustive,
        repeats=args.repeats,complete=False,hardware_commands=0,motion_authorized=False,results=[],
        policy=dict(audit_enabled=False,trial_offset_m=0.,jaw_signs=[1,-1],entry_clearance_m=.004,
                    calibration='hash_bound_capture_time_model',jaw_basis='nominal_stock_tcp_section',
                    posture='tabletop',stop='first_complete_route_per_signed_candidate',
                    route='open_pregrasp_approach_close_lift_carry',ompl_randomness='not_fixed'),
        notes=['generation median excludes warmup/model load/preprocessing',
               'total uses actual final generation sample + common check + validation; not median plus one route',
               'initial MoveIt/model setup and physical success excluded; setup reported separately',
               'full-pool success fraction is null after early stop; signed candidates may be geometrically redundant'])
    save(args.output_dir/'summary.json',report)
    try:
        for number,template,models,meta,depth,scene,plane,mask,observed in cases:
            jaw=models[2]['left'].jaw;opening=(jaw.gaps[0],jaw.gaps[-1]);points=observed.points
            def generate(name):
                if name=='graspnet':
                    g=graspnet.candidates(depth,meta['aligned_depth_intrinsics'],[mask],0,scene,plane,
                        observed.support_height_m,opening,checkpoint=checkpoint,section_points=points,audit_enabled=False)
                else:g=so101_candidates(points,plane,opening,limit=32)
                g['refined_grasps']=g['refined_grasps'][:args.maximum_candidates]
                g['benchmark_backend']=name
                return g
            names=['graspnet','so101'] if number==1 else ['so101','graspnet']
            for name in names:generate(name)
            samples={name:[] for name in names};groups={}
            for repeat in range(args.repeats):
                for name in (names if repeat%2==0 else names[::-1]):
                    torch.cuda.synchronize();t=perf_counter();groups[name]=generate(name);torch.cuda.synchronize()
                    samples[name].append(1000*(perf_counter()-t))
            for name in names:
                output=args.output_dir/f'set-{number}'/name
                t=perf_counter();group=common_sections(groups[name],points,jaw);common_ms=1000*(perf_counter()-t)
                prepared=bound_prepared(template,group)
                save(output/'prepared.json',prepared)
                row=dict(set=number,backend=name,generation_samples_ms=samples[name],
                    generation_median_ms=float(np.median(samples[name])),common_check_ms=common_ms,
                    generated_returned=len(groups[name]['refined_grasps']),common_passed=len(group['refined_grasps']))
                print(f'Set {number} {name}: common robot validation',flush=True)
                try:validation=validate(prepared,output/'validation',args.mobile_mesh_dir,
                                        args.inputs/f'set-{number}'/'capture-model',exhaustive=args.exhaustive)
                except Exception as exc:
                    validation=dict(status='VALIDATION_ERROR',reason=str(exc),first_feasible_validation_ms=None,hardware_commands=0)
                save(output/'validation.json',validation)
                first=validation.get('first_feasible_validation_ms')
                row.update(status=validation['status'],first_feasible_validation_ms=first,
                    generation_to_first_feasible_ms=None if first is None else samples[name][-1]+common_ms+first,
                    validation_setup_ms=validation.get('setup_ms'),
                    tested_signed_candidates=validation.get('tested_signed_candidates'),
                    feasible_signed_candidates=validation.get('feasible_signed_candidates'),
                    full_pool_evaluated=validation.get('full_pool_evaluated',False),
                    full_pool_feasible_fraction=validation.get('full_pool_feasible_fraction'))
                report['results'].append(row);save(args.output_dir/'summary.json',report)
        report['complete']=True
        report['valid_comparison']=all(r['status'] in ('OFFLINE_ROUTE_FOUND','NO_COMPLETE_ROUTE') for r in report['results'])
        save(args.output_dir/'summary.json',report)
    finally:graspnet.infer=original
    print('Saved: '+str(args.output_dir/'summary.json'))


if __name__=='__main__':main()
