#!/usr/bin/env python3
"""Reproduce the nominal Mini 1 -> SO101 adapter from pinned shared mesh geometry."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import cKDTree
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks import s2_commissioning as s2

EXPECTED = {
    'left_base.STL': 'd9d81d3df21305adc748346e5d5ea6700c70a6d433f1ff8b6e08ce423cd6ac17',
    'right_base.STL': '7992f74647898d037d94c1b41d084904986992c1dac75e867a09007f60b6448d',
    'base_so101_v2.stl': 'bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d',
}

def vertices(path):
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=EXPECTED[path.name]:
        raise ValueError('mesh differs from pinned official source: '+path.name)
    n=int.from_bytes(raw[80:84], 'little')
    if len(raw)!=84+50*n: raise ValueError('invalid binary STL')
    triangles=np.frombuffer(raw,offset=84,dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attribute','<u2')]))
    return np.unique(np.round(triangles['v'].reshape(-1,3).astype(float),8),axis=0)

def derive(cad_dir):
    g=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    sources=ROOT/'config/home_assistant/official_sources'
    so=ET.parse(sources/'so101_new_calib.urdf')
    cad=ET.parse(sources/'alohamini1.urdf')
    visual=next(v for v in so.findall('./link[@name="base_link"]/visual')
                if v.find('geometry/mesh').get('filename').endswith('/base_so101_v2.stl'))
    model_from_mesh=s2.origin({'origin':dict(visual.find('origin').attrib)})
    points=vertices(ROOT/'ros2_ws/src/so101_description/meshes/base_so101_v2.stl')
    tree=cKDTree(points)
    result={'schema_version':1, 'status':'official_shared_mesh_frame_adapter',
            'geometry_sha256':s2.digest(g), 'source_mesh_sha256':EXPECTED,
            'method':'shared vertices in identical mesh coordinates; invert SO101 base visual origin, preserve Mini1 base visual origin',
            'physical_mount_verified':False, 'motion_authorized':False, 'arms':{}}
    js={j['name']:j for j in g['mobile_reference_joints']}
    shoulder=next(j for j in g['arm_joints'] if j['name']=='shoulder_pan')
    for side in s2.ARMS:
        vp=cad.find('./link[@name="'+side+'_base"]/visual')
        cad_from_mesh=s2.origin({'origin':dict(vp.find('origin').attrib)})
        a=vertices(cad_dir/(side+'_base.STL'))
        distances,_=tree.query(a)
        matched=a[distances<1e-6]
        span=np.ptp(matched,axis=0)
        if len(matched)<1500 or np.min(span)<.07:
            raise ValueError('insufficient shared 3D geometry')
        adapter=cad_from_mesh@np.linalg.inv(model_from_mesh)
        delta=(adapter@s2.origin(shoulder))[:3,3]-s2.origin(js[side+'_joint1'])[:3,3]
        if np.linalg.norm(delta)>.002: raise ValueError('first joint centers disagree')
        result['arms'][side]={'cad_arm_from_model_root':adapter.tolist(),
            'shared_vertex_count':len(matched), 'shared_span_m':span.tolist(),
            'maximum_shared_residual_m':float(distances[distances<1e-6].max()),
            'shared_examples_m':matched[np.linspace(0,len(matched)-1,5,dtype=int)].tolist(),
            'first_joint_center_difference_m':delta.tolist(),
            'first_joint_center_distance_m':float(np.linalg.norm(delta)),
            'note':'Meshes are not identical; shared fixed-base geometry establishes a nominal frame, not installed accuracy.'}
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cad-meshes',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args()
    with a.output.open('x') as f: json.dump(derive(a.cad_meshes),f,ensure_ascii=False,indent=2);f.write('\n')
