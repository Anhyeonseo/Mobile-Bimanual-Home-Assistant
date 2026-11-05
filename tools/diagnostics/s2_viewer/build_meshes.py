#!/usr/bin/env python3
"""Reduce upstream STL surfaces for the self-contained S2 viewer (not collision meshes)."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks import s2_commissioning as s2


def read_stl(path):
    raw=path.read_bytes();n=struct.unpack_from('<I',raw,80)[0]
    if len(raw)!=84+n*50:raise ValueError(f'binary STL required: {path.name}')
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    vertices=np.frombuffer(raw[84:],dtype)['vertices'].reshape(-1,3).astype(float)
    return vertices,raw


def cluster(vertices,step):
    _,inverse=np.unique(np.round(vertices/step).astype(np.int32),axis=0,return_inverse=True)
    counts=np.bincount(inverse)
    pts=np.column_stack([np.bincount(inverse,weights=vertices[:,i])/counts for i in range(3)])
    faces=inverse.reshape(-1,3)
    faces=faces[(faces[:,0]!=faces[:,1])&(faces[:,0]!=faces[:,2])&(faces[:,1]!=faces[:,2])]
    _,unique=np.unique(np.sort(faces,axis=1),axis=0,return_index=True)
    faces=faces[np.sort(unique)]
    used,indices=np.unique(faces,return_inverse=True)
    return pts[used],indices.reshape(-1,3)


def pack(path,target,preserve_bounds=False):
    vertices,raw=read_stl(path);step=.0008
    pts,faces=cluster(vertices,step)
    while len(faces)>target:
        step*=1.3;pts,faces=cluster(vertices,step)
    low=pts.min(0);span=pts.max(0)-low;span=np.maximum(span,1e-9)
    quantized=np.round((pts-low)/span*65535).astype('<u2')
    source_min,source_max=vertices.min(0),vertices.max(0)
    if preserve_bounds:
        # Keep wheel radius/width despite display-only vertex clustering.
        low=source_min;span=np.maximum(source_max-source_min,1e-9)
    if len(pts)>65535:raise ValueError('mesh too large')
    return dict(min=low.tolist(),span=span.tolist(),vertices=base64.b64encode(quantized.tobytes()).decode(),
                faces=base64.b64encode(faces.astype('<u2').tobytes()).decode(),triangle_count=len(faces),
                vertex_count=len(pts),cluster_m=step,original_triangles=len(vertices)//3,
                source_min_m=source_min.tolist(),source_max_m=source_max.tolist(),
                bounds_preserved=preserve_bounds,
                source_sha256=hashlib.sha256(raw).hexdigest(),
                source_git_blob=hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest())


def build(mini1_meshes):
    geometry=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    urdf=ET.parse(ROOT/'config/home_assistant/official_sources/so101_new_calib.urdf').getroot()
    result={'schema_version':1,'geometry_sha256':s2.digest(geometry),'meshes':{},'arm_visuals':[],
            'purpose':'decimated display surfaces only; not collision or installed geometry',
            'sources':geometry['sources'][:2]}
    for link in urdf.findall('link'):
        for visual in link.findall('visual'):
            filename=Path(visual.find('geometry/mesh').get('filename')).name
            if filename not in result['meshes']:
                result['meshes'][filename]=pack(ROOT/'ros2_ws/src/so101_description/meshes'/filename,1600)
            matrix=s2.origin({'origin':visual.find('origin').attrib}).tolist()
            result['arm_visuals'].append(dict(link=link.get('name'),mesh=filename,origin=matrix,
                                              material='motor' if filename.startswith('sts') else 'arm'))
    for filename,target in [('base_link.STL',18000),('vertical_link.STL',3500),('wheel1.STL',1800)]:
        result['meshes'][filename]=pack(mini1_meshes/filename,target,preserve_bounds=filename=='wheel1.STL')
    result['cad_from_wheels']=[s2.origin(j).tolist() for j in geometry['mobile_reference_joints'] if j['name'] in ('wheel1','wheel2','wheel3')]
    raw_wheel,_=read_stl(mini1_meshes/'wheel1.STL')
    result['wheel_contact_z_m']=[float((raw_wheel@np.asarray(m)[:3,:3].T+np.asarray(m)[:3,3])[:,2].min()) for m in result['cad_from_wheels']]
    # CAD x points across the two arm mounts. This is a visual reference rotation,
    # not a calibrated base_link mapping or an arm-root adapter.
    result['display_from_cad']=[[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]]
    result['platform_mesh_top_m']=result['meshes']['vertical_link.STL']['min'][2]+result['meshes']['vertical_link.STL']['span'][2]
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mini1-meshes',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    d=build(a.mini1_meshes);a.output.write_text(json.dumps(d,separators=(',',':'))+'\n')
    print(a.output, a.output.stat().st_size, 'bytes',sum(m['triangle_count'] for m in d['meshes'].values()),'unique triangles')
