#!/usr/bin/env python3
"""Read-only model/observed-scene/path viewer; failed IK is never a trajectory."""
import argparse
import base64
from copy import deepcopy
from io import BytesIO
import hashlib
import html
import itertools
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from tools.diagnostics.plan_capture_carry import load_models
from tools.diagnostics.s2_visualize import bundle
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.stationary_pick_sim import StationaryArm
from home_robot_tasks.snapshot_io import hashes


def corners(pose,size):
    return [(pose[:3,:3]@(np.array(sign)*np.array(size)/2)+pose[:3,3]).tolist()
            for sign in itertools.product((-1,1),repeat=3)]


def visualization_data(prepared,reports):
    nominal=all(r.get('obstacle_envelope') in ('point_bounds','point_bounds_plus_1mm') for r in reports)
    padding=min(r.get('obstacle_padding_mm',1) for r in reports)/1000
    profile,geometry,arms,_,boxes,payload,joints=load_models(prepared,nominal_obstacles=nominal,obstacle_padding_m=padding)
    robot=bundle(geometry,profile);robot['display'].update(inspection_lighting=True,highlight_platform=True)
    state=lambda q:{side:{n:q[f'{side}_model_{n}'] for n in geo.JOINTS} for side in geo.ARMS}
    support=[];obstacles=[]
    for b in boxes:
        if b.role=='support':
            # Show measured top only; collision extrusion remains in MoveIt.
            local=np.array([[-1,-1,1],[-1,1,1],[1,1,1],[1,-1,1]])*np.array(b.size_m)/2
            support.append((local@b.base_pose[:3,:3].T+b.base_pose[:3,3]).tolist())
        else:
            obstacles.append((b.base_pose[:3,3]+b.base_pose[:3,:3]@np.array([0,0,b.size_m[2]/2])).tolist())
    actual=dict(label='촬영 당시 · 리프트 최저점',lift=joints['s2_lift_joint'],angles=state(joints),
        frames=[dict(label='captured')],message='촬영 당시 자세 · 이동 경로 아님')
    scenarios=[actual]
    for report in reports:
        if (report['capture_id']!=prepared['capture_id'] or report['source_sha256']!=prepared['source_sha256']
                or report['calibration_id']!=profile['calibration_id'] or report['motion_authorized'] is not False):
            raise ValueError('viewer report belongs to another capture')
        q=report['initial_model_joint_positions_si'];lift=q['s2_lift_joint']
        simulated=report.get('hypothetical_lift_m') is not None
        label=f"가상 높이 +{lift*100:.0f}cm" if simulated else '촬영 높이에서 계산'
        s=dict(label=label,lift=lift,angles=state(q),frames=[dict(label='captured')])
        plan=report.get('plan',{});candidate=plan.get('resolved_grasp') or next((c['candidate'] for c in report['candidates'] if c['candidate']['candidate_id']==plan.get('candidate_id')),None)
        if report['status']=='OFFLINE_ROUTE_FOUND' and candidate:
            ranges=json.loads((ROOT/'config/home_assistant/arm_ranges.development.json').read_text())
            arm=StationaryArm(profile,geometry,ranges,lift,arms['left'].jaw,'left')
            s['attachment']=candidate['tcp_from_object'];s['message']='접근 → 집게 닫기 → 2cm 들기 → 운반 자세 · 계산 통과, 실물 미검증'
            for stage in plan['stages']:
                attached=stage['label'] in ('lift','carry')
                jaw=candidate['held_jaw_model_rad'] if attached or stage['label']=='close_geometry' else candidate['open_jaw_model_rad']
                path=stage['joints'];ids=np.unique(np.linspace(0,len(path)-1,min(60,len(path))).astype(int))
                for i in ids:s['frames'].append(dict(label=stage['label'],q=path[i],jaw=jaw,attached=attached,tcp=arm.fk(path[i])[0][:3,3].tolist()))
        else:
            s['message']='완전한 경로 없음 · 후보의 집게 위치·방향을 동시에 맞추지 못함'
            if not report.get('initial_state_valid',True):s['message']='촬영 자세와 주변 장애물의 모델 간섭 확인 필요'
            elif report.get('reason'):s['message']='계산 중단: '+report['reason']
            elif plan.get('failures'):
                failure=plan['failures'][0];s['message']='경로 검사 중단: '+failure['stage']
                if 'capture_obstacle' in failure['reason']:s['message']='진입 중 손가락과 관측 장애물 겹침 · 실행 경로 없음'
                elif 'capture_support' in failure['reason']:s['message']='집게와 탁자 모델 간섭 · 실행 경로 없음'
                info=failure.get('diagnostics') or {}
                failed=next((c['candidate'] for c in report['candidates'] if c['candidate']['candidate_id']==failure['candidate']),None)
                if info.get('model_rad') and failed:
                    s['frames'].append(dict(label='간섭 확인 자세 · 이동 경로 아님',q=info['model_rad'],jaw=info.get('jaw_model_rad') if info.get('jaw_model_rad') is not None else failed['open_jaw_model_rad'],
                        contacts=[c['position_m'] for c in info.get('contacts',[])]))
        scenarios.append(s)
    if len(obstacles)>2500:obstacles=[obstacles[i] for i in np.linspace(0,len(obstacles)-1,2500).astype(int)]
    if payload.mesh:
        local=np.array(payload.mesh['vertices']);target=(local@payload.base_from_object[:3,:3].T+payload.base_from_object[:3,3]).tolist()
        n=len(local)//2;edges=[[i,(i+1)%n] for i in range(n)]+[[n+i,n+(i+1)%n] for i in range(n)]+[[i,n+i] for i in range(n)]
        local=local.tolist()
    else:
        local=corners(np.eye(4),payload.size_m);target=corners(payload.base_from_object,payload.size_m)
        edges=[[i,i^bit] for i in range(8) for bit in (1,2,4) if (i^bit)>i]
    return dict(robot=robot,capture=prepared['capture_id'],ground=profile['ground_z_m'],arm_names=list(geo.JOINTS[:5]),
                support=np.round(support,5).tolist(),obstacles=np.round(obstacles,5).tolist(),
                target=target,target_edges=edges,payload_corners=local,scenarios=scenarios)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared',type=Path,required=True);p.add_argument('--report',type=Path,action='append',required=True)
    p.add_argument('--snapshot',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();prepared=json.loads(a.prepared.read_text());reports=[json.loads(r.read_text()) for r in a.report]
    if hashes(a.snapshot)!=prepared['source_sha256']:raise ValueError('viewer image does not belong to capture')
    digest=hashlib.sha256(a.prepared.read_bytes()).hexdigest()
    if any(r['prepared_sha256']!=digest for r in reports):raise ValueError('planning input was changed')
    data=visualization_data(prepared,reports)
    from PIL import Image
    with Image.open(a.snapshot/'color.png') as image:
        image.thumbnail((320,240));stream=BytesIO();image.save(stream,format='JPEG',quality=80)
    here=Path(__file__).parent
    content=(here/'carry_viewer/template.html').read_text()
    replacements={'__DATA__':json.dumps(data,ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c'),
        '__PHOTO__':'data:image/jpeg;base64,'+base64.b64encode(stream.getvalue()).decode(),
        '__CAPTURE__':html.escape(prepared['capture_id']),
        '__KINEMATICS__':(here/'s2_viewer/viewer.js').read_text(),'__RENDERER__':(here/'s2_viewer/mesh-renderer.js').read_text(),
        '__VIEWER__':(here/'carry_viewer/viewer.js').read_text()}
    for key,value in replacements.items():content=content.replace(key,value)
    if len(content.encode())>1_000_000:raise ValueError('viewer exceeds inline size budget')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(content)
    print(a.output)


if __name__=='__main__':main()
