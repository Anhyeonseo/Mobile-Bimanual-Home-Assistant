#!/usr/bin/env python3
"""Keep full stop and carriage surfaces in the compact endpoint inspection view."""
import argparse
import base64
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s2_lift_contacts import read_triangles, stop_components
from build_meshes import cluster


def pack(triangles,digest,preserved,original):
    vertices,faces=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
    lo=vertices.min(0);span=np.maximum(vertices.max(0)-lo,1e-9)
    q=np.round((vertices-lo)/span*65535).astype('<u2')
    if len(vertices)>65535:raise ValueError('index overflow')
    return dict(min=lo.tolist(),span=span.tolist(),vertices=base64.b64encode(q.tobytes()).decode(),
                faces=base64.b64encode(faces.astype('<u2').tobytes()).decode(),vertex_count=len(vertices),
                triangle_count=len(triangles),source_sha256=digest,original_triangles=original,
                full_resolution_triangles=preserved,quantization_max_axis_error_m=(span/131070).tolist())


def build(directory):
    fixed,digest=read_triangles(directory/'base_link.STL');masks=stop_components(fixed)
    preserve=masks['low']|masks['high'];other=fixed[~preserve].reshape(-1,3)
    step=.001
    while True:
        v,f=cluster(other,step)
        if len(f)<=12000:break
        step*=1.3
    result={'base_link.STL':pack(np.concatenate([fixed[preserve],v[f]]),digest,int(preserve.sum()),len(fixed))}
    moving,digest=read_triangles(directory/'vertical_link.STL')
    result['vertical_link.STL']=pack(moving,digest,len(moving),len(moving))
    return dict(purpose='display only; full stop solids and carriage faces, quantized vertices; other fixed parts reduced',meshes=result)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mesh-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.write_text(json.dumps(build(a.mesh_dir),separators=(',',':'))+'\n')
    print(a.output,a.output.stat().st_size)
