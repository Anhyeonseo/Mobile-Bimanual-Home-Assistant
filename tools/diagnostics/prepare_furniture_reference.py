#!/usr/bin/env python3
"""Extract a landmark's appearance from immutable accepted SLAM RGB-D assets."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))


def prepare(profile,report_path,furniture_id,output):
    import cv2
    import numpy as np
    from scipy.spatial import cKDTree
    from home_robot_tasks.recorded_semantics import register_native_depth
    from home_robot_tasks.furniture_relocation import image_points
    from home_robot_tasks.furniture_geometry import orthographic_features
    profile,report_path,output=map(Path,(profile,report_path,output))
    if output.exists():raise ValueError('new reference directory required')
    labels=json.loads((profile/'household-labels.json').read_text())
    if hashlib.sha256(report_path.read_bytes()).hexdigest()!=labels['source_report_sha256']:
        raise ValueError('semantic source hash mismatch')
    entity=next(e for e in labels['entities'] if e['id']==furniture_id)
    geometry=np.asarray(json.loads((profile/'review-geometry.json').read_text())[furniture_id])
    tree=cKDTree(geometry);report=json.loads(report_path.read_text());features={};sources=[];views=[]
    sift=cv2.SIFT_create(nfeatures=1600,contrastThreshold=.015)
    planar_views=[]
    for index,frame in enumerate(report['frames']):
        source=Path(frame['capture']['source_path'])
        if hashlib.sha256(source.read_bytes()).hexdigest()!=frame['capture']['sha256']:
            raise ValueError('recorded capture hash mismatch')
        metadata=json.loads((source.parent/'metadata.json').read_text())
        with np.load(source,allow_pickle=False) as raw:
            color=raw['color_bgr'];raw_native=raw['native_depth'];depth=register_native_depth(raw_native,metadata)
        world=image_points(depth,metadata['color_intrinsics'],frame['camera_to_map'])
        mask=((depth>.15)&(tree.query(world.reshape(-1,3))[0].reshape(depth.shape)<.065)).astype('uint8')*255
        if np.count_nonzero(mask)<1000:continue
        keys,des=sift.detectAndCompute(color,cv2.erode(mask,np.ones((5,5),np.uint8)))
        if des is None or len(keys)<10:continue
        uv=np.array([k.pt for k in keys]);pixels=np.rint(uv).astype(int)
        points=world[pixels[:,1],pixels[:,0]]
        valid=(depth[pixels[:,1],pixels[:,0]]>.15)&(tree.query(points)[0]<.065)
        if valid.sum()<10:continue
        for name,value in (('uv',uv[valid]),('des',des[valid]),('xyz',points[valid])):
            features[f'{index}_{name}']=value
        views.append(index);sources.append(dict(view=index,capture=frame['capture'],camera_to_map=frame['camera_to_map']))
        # The immutable landmark geometry selects the original surface. Never
        # replace the reference with a photograph of the moved furniture.
        for surface in orthographic_features(color,raw_native,metadata,frame['camera_to_map'],
                height_m=float(np.median(geometry[:,2]))):
            member=tree.query(surface['xyz'])[0]<.08
            if member.sum()<10 or member.mean()<.5:continue
            name=f'{index}_planar_{len(planar_views)}'
            features[name+'_des']=surface['des'][member]
            features[name+'_xyz']=surface['xyz'][member]
            features[name+'_boundary']=surface['boundary_xy_m']
            planar_views.append(dict(name=name,view=index,height_m=surface['height_m']))
    if not views:raise ValueError('no sufficiently textured recorded furniture view')
    output.mkdir(parents=True);np.savez(output/'features.npz',**features)
    ref=dict(schema_version=1,furniture_id=furniture_id,label=entity['label'],context=labels['context'],
        anchor_xyz_m=entity['observed_anchor_xyz_m'],observed_geometry_xyz_m=geometry.tolist(),
        views=views,sources=sources,source_map_sha256=labels['source_map_sha256'],
        source_semantic_report_sha256=labels['source_report_sha256'],
        features_sha256=hashlib.sha256((output/'features.npz').read_bytes()).hexdigest(),
        method='SIFT mutual ratio matches + measured RGB-D rigid XY fit',
        planar_views=planar_views,
        geometry_is_partial=True,occupancy_clearance_authorized=False)
    ref['revision']='fref-'+hashlib.sha256(json.dumps(ref,sort_keys=True).encode()).hexdigest()[:24]
    (output/'reference.json').write_text(json.dumps(ref,ensure_ascii=False,indent=2)+'\n')
    return dict(furniture_id=furniture_id,views=views,revision=ref['revision'],motion_commands=0)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',type=Path,required=True);p.add_argument('--report',type=Path,required=True)
    p.add_argument('--furniture-id',required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(json.dumps(prepare(a.profile,a.report,a.furniture_id,a.output),ensure_ascii=False))
