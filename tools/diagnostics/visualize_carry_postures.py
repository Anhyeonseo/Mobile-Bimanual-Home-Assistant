#!/usr/bin/env python3
"""Render selected native-planner carry fixtures with the same official model."""
import argparse,json,sys
from pathlib import Path
from itertools import product,combinations
import numpy as np
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from tools.diagnostics.plan_capture_carry import load_models
from tools.diagnostics.s2_visualize import bundle
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.stationary_pick_sim import StationaryArm

LABELS={'observed_remote':'리모컨 · 촬영 형상','small_box':'작은 상자 · 가상',
        'upright_cup':'컵 형태 · 가상','upright_bottle':'병 형태 · 가상',
        'long_centered':'긴 물체 25cm · 중앙 파지 · 가상','long_offset':'긴 물체 25cm · 치우친 파지 · 가상'}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepared',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();prepared=json.loads(a.prepared.read_text());report=json.loads(a.report.read_text())
    if report['capture_id']!=prepared['capture_id'] or report['source_sha256']!=prepared['source_sha256']:
        raise ValueError('report/capture mismatch')
    profile,geometry,arms,*_=load_models(prepared,nominal_obstacles=True,obstacle_padding_m=0)
    robot=bundle(geometry,profile);robot['display'].update(inspection_lighting=True,highlight_platform=True)
    j=report['initial_model_joint_positions_si'];state=dict(lift=report['lift_m'],point=[0,0,0],
        angles={side:{n:j[side+'_model_'+n] for n in geo.JOINTS} for side in geo.ARMS})
    ranges=json.loads((ROOT/'config/home_assistant/arm_ranges.development.json').read_text())
    arm=StationaryArm(profile,geometry,ranges,report['lift_m'],arms['left'].jaw,'left')
    reference=np.deg2rad(report['policy']['reference_model_deg']).tolist()
    cases=[dict(name='reference',label='공통 기준 · 빈 집게',q=reference,
                jaw=j['left_model_gripper'],attachment=None)]
    for result in report['results']:
        if result['status']!='OFFLINE_ROUTE_FOUND':raise ValueError('viewer requires validated selected goals')
        source=next(c for c in report['cases'] if c['name']==result['case']);c=source['candidate'];s=result['selected'];q=s['goal']['model_rad']
        vertices=np.array(c['payload_mesh']['vertices']) if c['payload_mesh'] else np.array(list(product((-1,1),repeat=3)))*np.array(c['object_size_m'])/2
        hull=ConvexHull(vertices);edges=set()
        # Suppress coplanar triangulation diagonals so payload shape remains legible.
        edge_faces={}
        for i,face in enumerate(hull.simplices):
            for pair in combinations(face,2):edge_faces.setdefault(tuple(sorted(map(int,pair))),[]).append(i)
        for edge,faces in edge_faces.items():
            if len(faces)!=2 or not np.allclose(hull.equations[faces[0],:3],hull.equations[faces[1],:3],atol=1e-6):edges.add(edge)
        cases.append(dict(name=source['name'],label=LABELS[source['name']],q=q,jaw=c['held_jaw_model_rad'],
            attachment=c['tcp_from_object'],vertices=vertices.tolist(),faces=hull.simplices.tolist(),edges=sorted(edges),
            quality=s['load'],shoulder=arm.fk(q)[1][2].tolist(),upright=c['object_constraint'] is not None))
    data=dict(id='carry-reference-20261108',robot=robot,ground=profile['ground_z_m'],state=state,names=list(geo.JOINTS[:5]),cases=cases)
    here=Path(__file__).parent;content=(here/'carry_viewer/posture-template.html').read_text()
    values={'__DATA__':json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False).replace('<','\\u003c'),
        '__KINEMATICS__':(here/'s2_viewer/viewer.js').read_text(),'__RENDERER__':(here/'s2_viewer/mesh-renderer.js').read_text(),
        '__VIEWER__':(here/'carry_viewer/posture-viewer.js').read_text()}
    for key,value in values.items():content=content.replace(key,value)
    if len(content.encode())>1_000_000:raise ValueError('inline size budget exceeded')
    a.output.write_text(content);print(a.output)

if __name__=='__main__':main()
