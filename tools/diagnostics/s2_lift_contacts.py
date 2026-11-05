#!/usr/bin/env python3
"""Contact limits for two selected lift stop surfaces, not a whole-robot collision check."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from s2_visualize import ROOT, s2


def read_triangles(path):
    raw=path.read_bytes()
    count=int.from_bytes(raw[80:84],'little')
    if len(raw)!=84+50*count:raise ValueError('expected binary STL')
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    return np.frombuffer(raw[84:],dtype)['vertices'].astype(float),hashlib.sha256(raw).hexdigest()


def cross2(a,b):return a[0]*b[1]-a[1]*b[0]


def connected_triangle_labels(triangles):
    """Identify complete solids; rounding only joins STL seams, never moves faces."""
    _,index=np.unique(np.round(triangles.reshape(-1,3),6),axis=0,return_inverse=True)
    faces=index.reshape(-1,3);parent=np.arange(index.max()+1)
    def root(i):
        while parent[i]!=i:
            parent[i]=parent[parent[i]];i=parent[i]
        return i
    for a,b,c in faces:
        ra=root(a)
        for v in (b,c):
            rb=root(v)
            if ra!=rb:parent[rb]=ra
    return np.array([root(a) for a in faces[:,0]])


def stop_components(fixed):
    """Seeds belong to the pinned STL, whose hash is checked by derive()."""
    labels=connected_triangle_labels(fixed)
    return {key:labels==labels[seed] for key,seed in [('low',62420),('high',22082)]}


def clip_triangles(subject,against):
    """Convex polygon intersection in the plane transverse to the lift axis."""
    out=list(subject)
    for a,b in zip(against,np.roll(against,-1,axis=0)):
        if not out:return []
        old=out;out=[];u=old[-1];du=cross2(b-a,u-a)
        for v in old:
            dv=cross2(b-a,v-a)
            if (du>=-1e-12)!=(dv>=-1e-12):out.append(u+(v-u)*du/(du-dv))
            if dv>=-1e-12:out.append(v)
            u,du=v,dv
    return out


def projected_faces(triangles,axis,sign,mask):
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    ids=np.where((sign*(normals@axis)>1e-10)&mask)[0]
    v=triangles[ids].copy()
    xy=v[:,:,:2]-v[:,:,2,None]*axis[:2]/axis[2]
    d=xy[:,1]-xy[:,0];e=xy[:,2]-xy[:,0];reverse=d[:,0]*e[:,1]-d[:,1]*e[:,0]<0
    xy[reverse]=xy[reverse][:,[0,2,1]];v[reverse]=v[reverse][:,[0,2,1]]
    coefficients=np.linalg.solve(np.concatenate([xy,np.ones((*xy.shape[:2],1))],axis=2),v[:,:,2])
    return ids,v,xy,coefficients


def contact_limit(fixed,moving,axis,mask,direction):
    """First selected-surface contact from q=0, for a constant prismatic axis.

    Triangle footprints must overlap. The height difference is affine on the
    intersection polygon; its extremum is attained at a polygon vertex. This
    handles the small official axis tilt without inventing a vertical axis.
    """
    axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis)
    if axis[2]<=.99:raise ValueError('this lift solver expects a near-vertical positive axis')
    if direction not in ('low','high'):raise ValueError('invalid direction')
    low=direction=='low'
    fi,fv,fp,fc=projected_faces(fixed,axis,1 if low else -1,mask)
    mi,mv,mp,mc=projected_faces(moving,axis,-1 if low else 1,np.ones(len(moving),bool))
    fl,fh=fp.min(1),fp.max(1);fzlo,fzhi=fv[:,:,2].min(1),fv[:,:,2].max(1)
    best=-np.inf if low else np.inf;record=None
    for k,p in enumerate(mp):
        candidate=(fl<=p.max(0)+1e-10).all(1)&(fh>=p.min(0)-1e-10).all(1)
        possible=(fzhi-mv[k,:,2].min())/axis[2] if low else (fzlo-mv[k,:,2].max())/axis[2]
        candidate &= possible>best-1e-10 if low else possible<best+1e-10
        for j in np.where(candidate)[0]:
            poly=clip_triangles(p,fp[j])
            if not poly:continue
            pts=np.asarray(poly);v=np.column_stack([pts,np.ones(len(pts))])
            delta=v@(fc[j]-mc[k])/axis[2]
            n=np.argmax(delta) if low else np.argmin(delta);q=float(delta[n])
            if (low and q>best) or (not low and q<best):
                best=q;xy=pts[n];z=float(np.r_[xy,1]@fc[j]);xyz=np.r_[xy+z*axis[:2]/axis[2],z]
                record=dict(q_m=q,fixed_triangle=int(fi[j]),moving_triangle=int(mi[k]),contact_cad_m=xyz.tolist(),
                            fixed_vertices=fixed[fi[j]].tolist(),moving_vertices_at_zero=moving[mi[k]].tolist())
    if record is None or (low and best>=0) or (not low and best<=0):
        raise ValueError('selected contact does not bound the starting lift pose')
    return record


def derive(mesh_dir):
    geometry=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    surfaces=json.loads((Path(__file__).with_name('s2_viewer')/'meshes.json').read_text())
    fixed,base_hash=read_triangles(mesh_dir/'base_link.STL');local,moving_hash=read_triangles(mesh_dir/'vertical_link.STL')
    for name,digest in [('base_link.STL',base_hash),('vertical_link.STL',moving_hash)]:
        if digest!=surfaces['meshes'][name]['source_sha256']:raise ValueError('source mesh revision mismatch')
    j=next(j for j in geometry['mobile_reference_joints'] if j['name']=='vertical_move');origin=s2.origin(j)
    axis=np.array([float(v) for v in j['axis']['xyz'].split()]);axis=origin[:3,:3]@axis;axis/=np.linalg.norm(axis)
    moving=local@origin[:3,:3].T+origin[:3,3]
    # Whole connected solids, including walls and thick underside. A Z crop
    # previously omitted the antenna foot below 880 mm and allowed penetration.
    masks=stop_components(fixed)
    contacts={key:contact_limit(fixed,moving,axis,mask,key) for key,mask in masks.items()}
    # Independently include all low base bodies (guards, wheel motors, plates).
    # Posts/rack are mating interfaces, so they are not treated as stop solids.
    labels=connected_triangle_labels(fixed)
    lower_assembly=np.zeros(len(fixed),bool)
    for label in np.unique(labels):
        part=labels==label
        if fixed[part].max((0,1))[2]<.12:lower_assembly|=part
    lower_audit=contact_limit(fixed,moving,axis,lower_assembly,'low')
    if abs(lower_audit['q_m']-contacts['low']['q_m'])>1e-7:
        raise ValueError('another base component blocks descent before the selected deck')
    ground=min(surfaces['wheel_contact_z_m'])
    # User height datum: the upper flange where each SO-ARM is attached.
    # Fit the pinned STL mounting plane, then evaluate it at the official
    # corresponding arm mount XY. Do not use the whole-carriage maximum Z.
    arm_plates={}
    for side,seed in [('left',19718),('right',17518)]:
        tri=local[seed]
        normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);normal/=np.linalg.norm(normal)
        mask=np.abs((local-tri[0])@normal).max(1)<2e-6
        points=local[mask].reshape(-1,3);center=points.mean(0)
        _,_,vt=np.linalg.svd(points-center,full_matrices=False)
        normal=vt[-1]*np.sign(vt[-1,2])
        mount=next(j for j in geometry['mobile_reference_joints'] if j['name']==side+'_base')
        point=s2.origin(mount)[:3,3].copy()
        point[2]=center[2]-np.dot(normal[:2],point[:2]-center[:2])/normal[2]
        arm_plates[side]=dict(name=side+' SO-ARM mounting plate upper face',seed_triangle=seed,
            local_point_m=point.tolist(),local_vertices_m=tri.tolist(),plane_normal_local=normal.tolist(),
            plane_center_local_m=center.tolist(),plane_triangle_count=int(mask.sum()),
            definition='floor to upper mounting face at official arm attachment XY')
    plate=arm_plates['left']
    plate_local=np.array(plate['local_point_m'])
    for key,record in contacts.items():
        at_contact=moving+axis*record['q_m']
        record['moving_envelope_top_cad_z_m']=float(at_contact[:,:,2].max())
        plate_at_contact=origin[:3,:3]@plate_local+origin[:3,3]+axis*record['q_m']
        record['platform_top_cad_z_m']=float(plate_at_contact[2])
        record['platform_top_cad_point_m']=plate_at_contact.tolist()
        record['floor_to_platform_top_m']=record['platform_top_cad_z_m']-ground
        record['arm_mounting_plates']={}
        for side,datum in arm_plates.items():
            point=origin[:3,:3]@np.array(datum['local_point_m'])+origin[:3,3]+axis*record['q_m']
            record['arm_mounting_plates'][side]=dict(cad_point_m=point.tolist(),floor_height_m=float(point[2]-ground))
        # Preserve actual mating triangles for visual highlighting and audit.
        record['moving_vertices_at_contact']=(np.array(record['moving_vertices_at_zero'])+axis*record['q_m']).tolist()
    selections={key:dict(seed_triangle=62420 if key=='low' else 22082,triangle_count=int(mask.sum()),
                        source_triangle_indices_sha256=hashlib.sha256(np.flatnonzero(mask).astype('<u4').tobytes()).hexdigest(),
                        bounds_min_m=fixed[mask].min((0,1)).tolist(),bounds_max_m=fixed[mask].max((0,1)).tolist(),
                        selection='entire vertex-connected component; 1 micrometre seam matching, original coordinates')
                for key,mask in masks.items()}
    return dict(schema_version=4,height_datum=plate,arm_height_datums=arm_plates,primary_height_side='left',geometry_sha256=s2.digest(geometry),source_mesh_sha256={'base_link.STL':base_hash,'vertical_link.STL':moving_hash},
                method='exact triangle projection intersection and affine gap extrema on selected stop surfaces',
                scope='complete base deck and antenna foot solids versus the complete moving carriage; not global collision clearance or hardware operating limits',
                lower_surface_selection=selections['low'],upper_surface_selection=selections['high'],
                axis_cad=axis.tolist(),ground_cad_z_m=ground,contacts=contacts,
                assembly_origin_cad_m=origin[:3,3].tolist(),
                lower_base_clearance_audit=dict(selection='all complete fixed components with max CAD Z below 0.12 m',
                    triangle_count=int(lower_assembly.sum()),q_m=lower_audit['q_m'],agrees_with_deck=True),
                assembly_reference='https://github.com/liyiteng/AlohaMini/blob/17c6a98d79881a45ab869c1f392ed89c0723a298/AlohaMini1/docs/hardware_assembly.md',
                historical_height_residual_m={key:contacts[key]['floor_to_platform_top_m']-height for key,height in [('low',.26),('high',.90)]},
                height_consistency_verified=False,
                historical_arm_height_residual_m={key:{side:v['floor_height_m']-height for side,v in contacts[key]['arm_mounting_plates'].items()} for key,height in [('low',.26),('high',.90)]},
                travel_m=contacts['high']['q_m']-contacts['low']['q_m'],historical_heights_used=False,
                physical_verified=False,motion_authorized=False)


def floor_height_comparison(report, underside_clearance_m, *, underside_cad_z_m=0.):
    """Re-reference heights only; never infer new wheel/motor mount transforms.

    The input datum is floor -> chassis bottom-plate UNDERSIDE. Keep the
    official CAD contact calculation unchanged and state remaining uncertainty.
    """
    import copy
    if not np.isfinite(underside_clearance_m) or not 0 < underside_clearance_m < .2:
        raise ValueError('underside clearance must be finite and between 0 and 0.2 m')
    if not np.isfinite(underside_cad_z_m):raise ValueError('invalid underside datum')
    ground=float(underside_cad_z_m-underside_clearance_m)
    correction=float(report['ground_cad_z_m']-ground)
    endpoints={key:{side:dict(floor_height_m=float(value['cad_point_m'][2]-ground),
                             cad_point_m=copy.deepcopy(value['cad_point_m']))
                    for side,value in contact['arm_mounting_plates'].items()}
               for key,contact in report['contacts'].items()}
    return dict(schema_version=1,source_contact_report_sha256=s2.digest(report),
                datum='floor to chassis bottom plate underside',
                underside_clearance_m=float(underside_clearance_m),underside_cad_z_m=float(underside_cad_z_m),
                nominal_ground_cad_z_m=report['ground_cad_z_m'],referenced_ground_cad_z_m=ground,
                height_correction_m=correction,endpoints=endpoints,travel_m=report['travel_m'],
                wheel_and_motor_mounts_inferred=False,full_kinematic_calibration_verified=False,
                mechanical_contacts_physically_verified=False,motion_authorized=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mesh-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=derive(a.mesh_dir);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v['floor_to_platform_top_m'] for k,v in result['contacts'].items()}));print('travel_m',result['travel_m'])
