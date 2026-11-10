#!/usr/bin/env python3
"""Export the exact native-checked route and original target pixels for Jetson."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import sys
import numpy as np
from PIL import Image
import cv2

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.carry_bundle import sha,write_json,load_bundle
from home_robot_tasks.s2_geometry import JOINTS,digest,model_from_control
from home_robot_tasks.carry_scene_guard import route_background_depth_guard
from tools.diagnostics.prepare_carry_execution import load_inputs


def bundle(report_path,prepared_path,proof_dir,snapshot,mask,output):
    program,inputs=load_inputs(report_path,prepared_path)
    old=json.loads((proof_dir/'program.json').read_text());data=asdict(program)
    # Independent Python/numpy versions can differ below a micro-radian.
    # Bind the geometry check to exactly the integer wire route it validated.
    for new,previous in zip(data['stages'],old['stages'],strict=True):
        if new['name']!=previous['name'] or len(new['points'])!=len(previous['points']):raise ValueError('native plan differs')
        for a,b in zip(new['points'],previous['points']):
            if a['offset_ms']!=b['offset_ms'] or not np.array_equal(np.round(a['positions_rad'],6),np.round(b['positions_rad'],6)):
                raise ValueError('native-validated wire route changed')
    proof=json.loads((proof_dir/'report.json').read_text())
    if proof['native_geometry'].get('status')!='PASSED' or proof['report_sha256']!=program.report_sha256:
        raise ValueError('native proof does not match plan')
    prepared=json.loads(prepared_path.read_text())
    index=prepared['review']['observation_index']
    if sha(mask)!=prepared['observations'][index]['mask_sha256']:raise ValueError('target mask changed')
    for name,checksum in program.source_sha256.items():
        if sha(snapshot/name)!=checksum:raise ValueError('capture changed')
    report,profile,geometry,arms,boxes=inputs
    arm=arms['left'];alignment=profile['arms']['left']['alignment']
    model=dict(profile=profile,jaw_basis=report.get('jaw_basis','measured_front'),
        approach_mode=report.get('approach_mode','tool_axis'),
        grasp_posture_policy=report.get('grasp_posture_policy'),
        jaw_geometry_physically_verified=False,
        range_sha256=sha(ROOT/'config/home_assistant/arm_ranges.development.json'),
        fk=dict(mount=arm.mount.tolist(),tcp=arm.tcp.tolist(),joints=[dict(origin=o.tolist(),axis=a.tolist()) for o,a in arm.joints],
                home_model_rad=[alignment[n]['home_model_rad'] for n in JOINTS[:5]],
                control_to_model_sign=[alignment[n]['control_to_model_sign'] for n in JOINTS[:5]]),
        error_budget_m=.0006,error_budget_status='provisional_combined_delta_budget_not_physical_accuracy_verification',
        scene_coverage='observed_only; moving robot image region excluded after admission',
        scene_guard=route_background_depth_guard(arms,profile,report['plan']['resolved_grasp'],
            ROOT/'config/home_assistant/official_sources/so101_new_calib.urdf',
            ROOT/'ros2_ws/src/so101_description/meshes',program))
    if report.get('start_revalidation') is not None:
        model['start_revalidation']=report['start_revalidation']
    md=json.loads((snapshot/'metadata.json').read_text());intr=md['aligned_depth_intrinsics']
    sweep=np.zeros((intr['height'],intr['width']),np.uint8);optical=np.linalg.inv(np.asarray(profile['base_from_optical']))
    def draw(points):
        pts=(optical@np.c_[points,np.ones(len(points))].T).T
        for a,b in zip(pts,pts[1:]):
            if min(a[2],b[2])<=.05:continue
            pixel=lambda p:(round(intr['fx']*p[0]/p[2]+intr['ppx']),round(intr['fy']*p[1]/p[2]+intr['ppy']))
            radius=min(200,max(3,round(intr['fx']*.065/min(a[2],b[2]))))
            cv2.line(sweep,pixel(a),pixel(b),255,2*radius)
    for stage in program.stages:
        for point in stage.points:
            for side,offset in (('left',0),('right',6)):
                q=[model_from_control(float(point.positions_rad[offset+i]),profile['arms'][side]['alignment'][n]) for i,n in enumerate(JOINTS[:5])]
                draw(arms[side].fk(q)[1])
    output.mkdir(parents=True,exist_ok=False)
    for name in program.source_sha256:shutil.copyfile(snapshot/name,output/name)
    shutil.copyfile(mask,output/'mask.png');Image.fromarray(sweep).save(output/'robot_sweep.png')
    write_json(output/'program.json',data);write_json(output/'model.json',model)
    write_json(output/'proof.json',dict(program_sha256=digest(data),model_sha256=digest(model),
        native_geometry=proof['native_geometry'],simulation_passed=proof['simulation_passed'],
        motion_authorized=False,physical_task_completed=False))
    files={p.name:sha(p) for p in sorted(output.iterdir())}
    write_json(output/'bundle.json',dict(kind='stationary_carry_bundle',schema_version=1,files_sha256=files))
    load_bundle(output)
    return dict(status='BUNDLE_VERIFIED',hypothetical_lift=program.hypothetical_lift,hardware_commands=0)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('report','prepared','proof-dir','snapshot','mask','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();print(json.dumps(bundle(a.report,a.prepared,a.proof_dir,a.snapshot,a.mask,a.output)))
