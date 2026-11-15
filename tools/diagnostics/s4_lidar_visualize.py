#!/usr/bin/env python3
"""Review a proposed LDS mount using the existing Mini 1 viewer and real scans.

This never writes a runtime transform or range correction. Original STL triangles
are used for the stationary self-occlusion comparison; reduced meshes are display only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from s2_lift_visualize import endpoint_bundle
from s2_visualize import ROOT, bundle, s2
sys.path.insert(0, str(Path(__file__).with_name('s2_viewer')))
from build_meshes import read_stl, pack


def ray_hits(triangles, origin, directions):
    """First positive intersection of each unit ray with original mesh triangles."""
    a = triangles[:, 0]; e1 = triangles[:, 1]-a; e2 = triangles[:, 2]-a
    delta = origin-a; q = np.cross(delta, e1); numerator = np.einsum('ij,ij->i', e2, q)
    result = []
    for direction in directions:
        h = np.cross(np.broadcast_to(direction,e2.shape), e2)
        det = np.einsum('ij,ij->i',e1,h)
        inv = np.divide(1.,det,out=np.zeros_like(det),where=abs(det)>1e-12)
        u = np.einsum('ij,ij->i',delta,h)*inv; v = (q@direction)*inv; t = numerator*inv
        valid = (abs(det)>1e-12)&(u>=-1e-8)&(v>=-1e-8)&(u+v<=1+1e-8)&(t>1e-5)
        result.append(float(t[valid].min()) if valid.any() else None)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--lds-mesh',type=Path,required=True)
    p.add_argument('--mobile-mesh-dir',type=Path,required=True)
    p.add_argument('--scans',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True)
    a=p.parse_args()
    official=json.loads((ROOT/'config/home_assistant/lds01_official_geometry.json').read_text())
    if hashlib.sha256(a.lds_mesh.read_bytes()).hexdigest()!=official['mesh_sha256']:
        raise ValueError('official LDS mesh hash mismatch')
    geometry=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    data=bundle(geometry);end=endpoint_bundle();low=end['endpoint_poses']['low']
    data['display'].update(lift_only=True,software_rendering=False)
    data['surfaces']['meshes']={k:v for k,v in data['surfaces']['meshes'].items() if k in ('base_link.STL','vertical_link.STL','wheel1.STL','d415.stl')}
    # The STL is in mm; the official URDF scales it by 0.001.
    mesh=pack(a.lds_mesh,2400)
    for key in ('min','span','source_min_m','source_max_m'):
        mesh[key]=[v*.001 for v in mesh[key]]
    mesh['cluster_m']*=.001
    data['surfaces']['meshes']['lds01.stl']=mesh
    cad=np.asarray(data['surfaces']['display_from_cad'])
    wheels=[cad@np.asarray(m) for m in data['display']['cad_from_wheels']]
    front=max(wheels,key=lambda m:m[0,3])
    # Select the horizontal cap on the existing front wheel/motor case,
    # above the chassis deck, from the original pinned triangles.
    base_vertices,base_raw=read_stl(a.mobile_mesh_dir/'base_link.STL')
    if hashlib.sha256(base_raw).hexdigest()!=data['surfaces']['meshes']['base_link.STL']['source_sha256']:
        raise ValueError('Mini 1 base source mismatch')
    tri=base_vertices.reshape(-1,3,3);cent=tri.mean(1)
    normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);area=np.linalg.norm(normal,axis=1)/2
    selected=(cent[:,1]<-.10)&(abs(cent[:,0])<.08)&(cent[:,2]>.10)&(abs(normal[:,2])>.999*np.linalg.norm(normal,axis=1))&(area>1e-9)
    top_z=float(cent[selected,2].max())
    cap=selected&(abs(cent[:,2]-top_z)<2e-6)
    case_cad=np.average(cent[cap],axis=0,weights=area[cap])
    case_base=(cad@np.r_[case_cad,1])[:3]
    cover_z=float(case_base[2])
    # -17.8 mm is a provisional STL flange datum, not measured installation.
    lidar=case_base+np.array([0,0,.0178])
    captured=json.loads(a.scans.read_text())
    if not captured or any(r['count']!=360 for r in captured):raise ValueError('360 ray capture required')
    values=np.asarray([[np.nan if x is None else x for x in r['ranges']] for r in captured[-20:]])
    ranges=[float(np.median(col[np.isfinite(col)])) if np.isfinite(col).any() else None for col in values.T]
    directions=np.array([[np.cos(i*np.pi/180+np.pi),np.sin(i*np.pi/180+np.pi),0] for i in range(360)])
    full=[]
    for name,transform in [('base_link.STL',cad),('vertical_link.STL',np.asarray(low['platform']))]:
        vertices,raw=read_stl(a.mobile_mesh_dir/name)
        if hashlib.sha256(raw).hexdigest()!=data['surfaces']['meshes'][name]['source_sha256']:
            raise ValueError('Mini 1 mesh hash mismatch')
        world=vertices@transform[:3,:3].T+transform[:3,3]
        full.append(world.reshape(-1,3,3))
    hits=ray_hits(np.concatenate(full),lidar,directions)
    residuals=[ranges[i]-d for i,d in enumerate(hits) if d is not None and .12<=d<=3.5 and ranges[i] is not None]
    report=dict(mode='mount_review_only',hardware_enabled=False,lift='operator reported lowest; CAD lower-contact pose',
        arms='current pose unknown; omitted from geometry comparison',mount_confirmed=False,range_calibration_applied=False,
        mount_basis='area-weighted top cap of front wheel/motor case from original CAD triangles; provisional STL flange z=-17.8 mm; yaw=180 deg suggested by missing-ray overlap, unconfirmed',
        case_top_cad_m=case_cad.tolist(),case_top_triangle_count=int(cap.sum()),
        base_from_lidar_translation_m=lidar.tolist(),yaw_rad=float(np.pi),scan_height_above_floor_m=float(lidar[2]-data['display']['ground_z_m']),
        scan_source=str(a.scans),scan_source_sha256=hashlib.sha256(a.scans.read_bytes()).hexdigest(),frames=len(captured),
        observed_ranges_m=ranges,model_first_hit_m=hits,valid_self_hit_comparisons=len(residuals),
        median_self_hit_residual_m=float(np.median(residuals)) if residuals else None,
        model_hits_inside_minimum_range=sum(d is not None and d<.12 for d in hits),
        missing_inside_model_shadow=sum(d is not None and ranges[i] is None for i,d in enumerate(hits)),
        missing_inside_shadow_at_yaw_zero=sum(d is not None and ranges[(i+180)%360] is None for i,d in enumerate(hits)),
        orientation_evidence='Missingness overlap supports a review candidate, not exact yaw or range bias.',
        accuracy_qualified=False,limitation='Self-hit residual is conditional on unconfirmed mount/pose. Environment geometry is unknown; no metric range calibration or model alignment is inferred.')
    data['lidar_review']=dict(mount=lidar.tolist(),yaw_deg=180,cover_z=cover_z,platform=low['platform'],ranges=ranges,model_hits=hits,
        plate_height_m=low['floor_to_platform_m'],official=official,comparisons=len(residuals))
    directory=Path(__file__).with_name('s2_viewer')
    encoded=json.dumps(data,ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c')
    html=(directory/'lidar-review.html').read_text().replace('/*__DATA__*/',encoded).replace('/*__SCRIPT__*/',
        (directory/'mesh-renderer.js').read_text()+'\n'+(directory/'lidar-review.js').read_text())
    if len(html.encode())>=1000000:raise ValueError('inline viewer exceeds size limit')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(html)
    a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('observed_ranges_m','model_first_hit_m')}))


if __name__=='__main__':main()
