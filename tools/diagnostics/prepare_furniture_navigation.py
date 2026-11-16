#!/usr/bin/env python3
"""Bind immutable SLAM/voxel evidence and the reviewed fixed robot body.

Exports existing STL height slices for live incremental recomputation. Does not
connect to hardware, remove map cells, or replace the original mapping files.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks')]
from tools.diagnostics.review_s4_sensor_capture import travel_geometry,slab_footprints,endpoint_obstacles
from home_robot_tasks.navigation_trial import validate_navigation_body


def observed_surface_cells(baseline,measured,reference,size):
    """Associate recorded voxel volumes, rather than exact rounded point keys.

    A voxel represents a volume: its centre can be half a cell diagonal from
    the surface return. The bound comes from the evidence resolution, not from
    robot clearance. The reference-distance gate matches the appearance layer.
    """
    from scipy.spatial import cKDTree
    baseline=np.asarray(sorted(baseline),int).reshape(-1,3)
    if not len(measured) or not len(baseline):return set()
    centers=(baseline+.5)*size
    eligible=cKDTree(reference).query(centers)[0]<.08
    observed=cKDTree(measured).query(centers)[0]<=np.sqrt(3)*size/2+1e-9
    return set(map(tuple,baseline[eligible&observed]))


def complete_surface_ownership(folder,reference_path,report_path):
    """Recover omitted furniture cells from its immutable mapping exposures.

    The original semantic geometry selects the connected measured surface. Only
    actual depth returns in that component can claim existing baseline volumes.
    This does not grow a mask, clear neighboring cells, or infer unseen volume.
    """
    from scipy.spatial import cKDTree
    from home_robot_tasks.furniture_geometry import planar_surfaces
    from home_robot_tasks.furniture_relocation import image_points
    from home_robot_tasks.recorded_semantics import register_native_depth
    folder,reference_path,report_path=map(Path,(folder,reference_path,report_path))
    destination=folder/'ownership.json'
    if destination.exists():raise ValueError('ownership evidence already exists')
    config=json.loads((folder/'binding.json').read_text())
    reference=json.loads(reference_path.read_text())
    if (reference['revision']!=config['reference_revision'] or
            reference['furniture_id']!=config['furniture_id'] or
            hashlib.sha256(report_path.read_bytes()).hexdigest()!=reference['source_semantic_report_sha256'] or
            hashlib.sha256((folder/'baseline.npz').read_bytes()).hexdigest()!=config['baseline_sha256']):
        raise ValueError('original furniture mapping evidence mismatch')
    points=np.asarray(config['reference_geometry']);tree=cKDTree(points);size=config['voxel_size_m']
    with np.load(folder/'baseline.npz') as data:baseline=set(map(tuple,data['voxels']))
    original=set(map(tuple,np.floor(points/size+1e-8).astype(int)))&baseline
    votes={};sources=[]
    for index,frame in enumerate(json.loads(report_path.read_text())['frames']):
        path=Path(frame['capture']['source_path'])
        if hashlib.sha256(path.read_bytes()).hexdigest()!=frame['capture']['sha256']:
            raise ValueError('original RGB-D capture changed')
        metadata_path=path.parent/'metadata.json';meta=json.loads(metadata_path.read_text())
        with np.load(path,allow_pickle=False) as data:
            color,native=data['color_bgr'],data['native_depth']
        surfaces=planar_surfaces(color,native,meta,frame['camera_to_map'],height_m=float(np.median(points[:,2])))
        selected=[s for s in surfaces if (tree.query(s['cloud_xyz_m'])[0]<.08).mean()>=.7]
        if not selected:continue
        world=image_points(register_native_depth(native,meta),meta['color_intrinsics'],frame['camera_to_map'])
        cells=set()
        for surface in selected:
            measured=world[surface['mask']>0]
            # Bound each return to the existing identity, even if two surfaces
            # touch in the image. No recursive connected-component expansion.
            measured=measured[tree.query(measured)[0]<.08]
            cells|=observed_surface_cells(baseline,measured,points,size)
        for cell in cells-original:votes.setdefault(cell,[]).append(index)
        sources.append(dict(view=index,capture=frame['capture'],camera_to_map=frame['camera_to_map'],
            metadata_sha256=hashlib.sha256(metadata_path.read_bytes()).hexdigest()))
    evidence=dict(schema_version=1,method='original_rgbd_surface_membership',
        furniture_id=config['furniture_id'],reference_revision=config['reference_revision'],
        binding_sha256=hashlib.sha256((folder/'binding.json').read_bytes()).hexdigest(),
        reference_sha256=hashlib.sha256(reference_path.read_bytes()).hexdigest(),
        report_sha256=hashlib.sha256(report_path.read_bytes()).hexdigest(),sources=sources,
        voxel_surface_radius_m=float(np.sqrt(3)*size/2),
        members=[dict(voxel=list(map(int,k)),views=v) for k,v in sorted(votes.items())])
    destination.write_text(json.dumps(evidence,indent=2)+'\n')
    return dict(original_voxels=len(original),added_owned_voxels=len(votes),source_views=len(sources),motor_commands=0)


def prepare_geometry_cache(folder):
    """Precompute invariant official-body kernels; leave evidence binding intact."""
    from tools.diagnostics.review_s4_sensor_capture import footprint_kernel
    folder=Path(folder);config=json.loads((folder/'binding.json').read_text())
    grid=json.loads((folder/'occupancy.json').read_text());res=grid['resolution'];origin=np.array(grid['origin'])
    with np.load(folder/'baseline.npz') as source:count=len(source['feasible_by_yaw'])
    body=config['body'];kernels={}
    center=np.array([.5,.5])*res
    anchor=np.floor((center-origin)/res)
    phase=origin+(anchor+.5)*res-center
    for i in range(count):
        yaw=i*2*np.pi/count
        kernels['base_'+str(i)]=footprint_kernel([body['footprint_xy_m']],res,yaw,body['clearance_m'])
        for z,polygons in config['slabs'].items():
            if polygons:kernels[f'slab_{i}_{z}']=footprint_kernel(polygons,res,yaw,body['clearance_m'],
                obstacle_size_m=config['voxel_size_m'],lattice_offset_xy_m=phase)
    archive=folder/'geometry-cache.npz';np.savez_compressed(archive,**kernels)
    (folder/'geometry-cache.json').write_text(json.dumps(dict(
        binding_sha256=hashlib.sha256((folder/'binding.json').read_bytes()).hexdigest(),
        cache_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),kernels=len(kernels)),indent=2)+'\n')
    return dict(kernels=len(kernels),motor_commands=0)


def prepare(map_path,area_path,depth_path,body_path,geometry_report,reference_path,output):
    paths=list(map(Path,(map_path,area_path,depth_path,body_path,geometry_report,reference_path)))
    map_path,area_path,depth_path,body_path,geometry_report,reference_path=paths
    output=Path(output)
    if output.exists():raise ValueError('new navigation evidence directory required')
    saved=json.loads(map_path.read_text());area=dict(np.load(area_path));body=json.loads(body_path.read_text())
    validate_navigation_body(body,area,map_path)
    reference=json.loads(reference_path.read_text());report=json.loads(geometry_report.read_text())
    if hashlib.sha256(map_path.read_bytes()).hexdigest()!=reference['source_map_sha256']:raise ValueError('reference map mismatch')
    depth=dict(np.load(depth_path));occupied=depth['voxels'][depth['occupied']]
    grid=np.asarray(saved['data']).reshape(saved['height'],saved['width']);origin=np.asarray(saved['origin']);res=saved['resolution']
    projection,_=endpoint_obstacles(occupied,.05,grid.shape,origin,res,bottom=.1,top=body['height_above_floor_m']+body['clearance_m'])
    if not np.array_equal(projection,area['rgbd_obstacles']):raise ValueError('voxel provenance differs from reviewed area')
    output.mkdir(parents=True)
    old=report['geometry'];joints=('base','shoulder','elbow','wrist_flex','wrist_roll','gripper')
    axes=[dict(slot=offset+i,position=old['arm_raw'][side][joint]) for side,offset in [('left',1),('right',7)] for i,joint in enumerate(joints)]
    axes.append(dict(slot=13,position=old['lift_raw']))
    hold=output/'derived-model-posture.json';hold.write_text(json.dumps(dict(executed=False,plan=dict(ok=True,axes=axes),source=str(geometry_report))))
    geometry,_,surfaces=travel_geometry(ROOT,hold,lift_m=0,
        mobile_mesh_dir=ROOT/'tmp/s2-official/AlohaMini/AlohaMini1/simulation/src/Aloha/meshes',
        d415_mesh=ROOT/'tmp/s2-official/realsense/d415.stl',lds_mesh=ROOT/'output/s4-lidar-fix/lds01-official.stl',
        include_surfaces=True,exclude_arms=True)
    if not np.allclose(geometry['footprint_xy_m'],body['footprint_xy_m'],atol=1e-9):raise ValueError('reviewed body differs')
    slices={}
    for z in range(int(np.ceil((geometry['height_above_floor_m']+body['clearance_m'])/.05))):
        slices[str(z)]=[p.tolist() for p in slab_footprints(surfaces,max(0,z*.05-body['clearance_m']),min(geometry['height_above_floor_m'],(z+1)*.05+body['clearance_m']))]
    np.savez_compressed(output/'baseline.npz',voxels=occupied,**area)
    (output/'occupancy.json').write_bytes(map_path.read_bytes())
    config=dict(schema_version=1,voxel_size_m=.05,body=body,slabs=slices,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        baseline_sha256=hashlib.sha256((output/'baseline.npz').read_bytes()).hexdigest(),
        map_sha256=hashlib.sha256(map_path.read_bytes()).hexdigest(),
        reference_revision=reference['revision'],furniture_id=reference['furniture_id'],
        reference_geometry=reference['observed_geometry_xyz_m'])
    (output/'binding.json').write_text(json.dumps(config,indent=2)+'\n')
    prepare_geometry_cache(output)
    return dict(voxels=len(occupied),height_bands=len(slices),motor_commands=0)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cache-existing',type=Path,help='add kernels to a copied, existing bound evidence folder')
    p.add_argument('--complete-ownership',type=Path,help='bind omitted surface cells using original RGB-D records')
    p.add_argument('--original-report',type=Path)
    names=('map','area','depth','body','geometry-report','reference','output')
    for name in names:p.add_argument('--'+name,type=Path)
    a=p.parse_args()
    if a.complete_ownership:
        if not a.reference or not a.original_report or a.cache_existing or any(getattr(a,n.replace('-','_')) for n in names if n!='reference'):
            p.error('complete-ownership requires only reference and original-report')
        print(json.dumps(complete_surface_ownership(a.complete_ownership,a.reference,a.original_report)))
    elif a.cache_existing:
        if any(getattr(a,n.replace('-','_')) for n in names):p.error('cache-existing cannot be mixed with new evidence inputs')
        print(json.dumps(prepare_geometry_cache(a.cache_existing)))
    else:
        if not all(getattr(a,n.replace('-','_')) for n in names):p.error('all new evidence inputs required')
        print(json.dumps(prepare(a.map,a.area,a.depth,a.body,a.geometry_report,a.reference,a.output)))
