#!/usr/bin/env python3
"""Replay saved instance masks/depth; report ownership conflicts, never move hardware.

This is a consistency audit, not labelled segmentation accuracy or grasp success.
Each accepted instance keeps other nonoverlapping instance masks protected.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import ConvexHull

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_perception.geometry.pick_scene import build_pick_scene,InsufficientScene
from home_perception.geometry.plane import backproject
from home_robot_tasks.snapshot_io import hashes
from tools.diagnostics.replay_s3_grasp import load_scene,grounding_observations


def audit(snapshot):
    source=hashes(snapshot)
    reports=sorted(snapshot.glob('grounding_*/detections.json'))
    matching=[p for p in reports if json.loads(p.read_text()).get('source_sha256')==source]
    if not matching:return dict(snapshot=str(snapshot),status='NO_BOUND_SEGMENTATION',instances=[])
    detection=matching[-1]
    profile=json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())
    meta,depth,_,global_plane=load_scene(snapshot);clouds={};planes={}
    observations,masks=grounding_observations(snapshot,detection,meta,depth,global_plane,source,clouds,
        support_planes=planes,up_in_optical=np.asarray(profile['base_from_optical'])[2,:3])
    results=[]
    for i,observation in enumerate(observations):
        row=dict(index=i,label=observation['label'],perception_status=observation['status'])
        if observation['status']!='accepted':
            results.append(dict(row,status='PERCEPTION_REOBSERVE',reason=observation['reason']));continue
        mask=masks[i];protected=np.zeros_like(mask)
        for j,other in enumerate(masks):
            overlap=np.count_nonzero(mask&other)/max(1,min(mask.sum(),other.sum()))
            if j!=i and overlap<=.05:protected|=other&~mask
        try:
            scene=build_pick_scene(depth,meta['aligned_depth_intrinsics'],planes.get(i,global_plane),mask,clouds[i],protected=protected)
        except InsufficientScene as exc:
            results.append(dict(row,status='SCENE_REOBSERVE',reason=str(exc)));continue
        points=backproject(depth,meta['aligned_depth_intrinsics'],depth>0)
        support=points@scene.support_from_optical[:3,:3].T+scene.support_from_optical[:3,3]
        world=(~scene.target_pixels[depth>0])&(abs(support[:,2])>.005)
        transform=np.linalg.inv(scene.support_from_payload)
        local=support@transform[:3,:3].T+transform[:3,3]
        hull=ConvexHull(scene.payload_mesh['vertices'])
        inside=np.all(local@hull.equations[:,:3].T+hull.equations[:,3]<=1e-10,axis=1)
        protected_swallowed=int((protected&scene.target_pixels).sum())
        conflict=int((inside&world).sum())
        status='OWNERSHIP_CONSISTENT' if not protected_swallowed and not conflict else 'MODEL_CONFLICT'
        results.append(dict(row,status=status,target_points=int(scene.target_pixels.sum()),
            retained_world_returns=int(world.sum()),protected_pixels=int(protected.sum()),
            protected_pixels_swallowed=protected_swallowed,world_returns_inside_payload=conflict,
            ownership=scene.ownership_evidence))
    return dict(snapshot=str(snapshot),source_sha256=source,detections=str(detection),status='AUDITED',instances=results)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshots',type=Path,nargs='+',required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();folders=set()
    for path in args.snapshots:
        if (path/'metadata.json').is_file():folders.add(path)
        else:folders.update(m.parent for m in path.rglob('metadata.json'))
    rows=[]
    for folder in sorted(folders):
        result=audit(folder);rows.append(result)
        print(folder.name,[(r['label'],r['status']) for r in result['instances']],flush=True)
    instances=[r for f in rows for r in f['instances']]
    counts={status:sum(r['status']==status for r in instances) for status in sorted({r['status'] for r in instances})}
    report=dict(mode='saved_rgbd_ownership_consistency_audit',hardware_commands=0,motion_authorized=False,
        segmentation_accuracy_measured=False,physical_grasp_success_measured=False,counts=counts,captures=rows)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(counts))


if __name__=='__main__':main()
