"""Offline ray evidence for recorded RGB-D review, never a live safety volume.

Reuses native projection and DepthVolume's voxel traversal. Sampling leaves
unknown space unknown; floor endpoints can clear rays above them. A hit wins
within a frame, while later measured rays can contradict an old endpoint.
"""
import math
import hashlib
import json
from pathlib import Path
import numpy as np

from .navigation_depth import DepthProfile, DepthVolume, native_endpoints


class RecordedRayEvidence:
    def __init__(self, *, resolution=.05, minimum_height=.10, maximum_height=1.15,
                 minimum_depth=.12, maximum_depth=3.5):
        self.profile = DepthProfile('depth', 'recorded', resolution, minimum_depth,
            maximum_depth, minimum_height, maximum_height, 1., .01,
            maximum_voxels=500000, maximum_ray_steps=2000000, world_frame='map')
        self.traversal = DepthVolume(self.profile)
        self.cells = {}
        self.frames = 0

    def integrate_points(self, points, origin, optical_forward):
        points=np.asarray(points,float);origin=np.asarray(origin,float)
        forward=np.asarray(optical_forward,float);p=self.profile
        if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():
            raise ValueError('finite measured points required')
        if origin.shape!=(3,) or forward.shape!=(3,) or not np.isfinite([origin,forward]).all():
            raise ValueError('finite camera origin and direction required')
        if not np.isclose(np.linalg.norm(forward),1.):raise ValueError('unit optical direction required')
        low=math.floor(p.minimum_height_m/p.resolution_m)
        high=math.ceil(p.maximum_height_m/p.resolution_m)
        keys,indices=np.unique(np.floor(points/p.resolution_m).astype(np.int64),axis=0,return_index=True)
        hits,cleared=set(),set();work=0
        if len(keys)>p.maximum_voxels:raise ValueError('endpoint budget exceeded')
        for key,point in zip(keys,points[indices]):
            key=tuple(int(v) for v in key)
            if low<=key[2]<high:hits.add(key)
            for cell in self.traversal._ray(origin,point):
                work+=1
                if work>p.maximum_ray_steps:raise ValueError('offline ray budget exceeded')
                if low<=cell[2]<high:
                    optical_z=sum(float(forward[i])*((cell[i]+.5)*p.resolution_m-float(origin[i])) for i in range(3))
                    if optical_z>=p.minimum_depth_m+math.sqrt(3)*p.resolution_m:cleared.add(cell)
        if len(self.cells.keys()|hits|cleared)>p.maximum_voxels:raise ValueError('offline volume budget exceeded')
        for key in cleared-hits:
            value=self.cells.setdefault(key,[0,0,0.]);value[1]+=1;value[2]=max(-2.,value[2]-.4)
        for key in hits:
            value=self.cells.setdefault(key,[0,0,0.]);value[0]+=1;value[2]=min(3.5,value[2]+.85)
        self.frames+=1
        return dict(hits=len(hits),cleared=len(cleared-hits),ray_steps=work)

    def integrate(self, depth_m, intrinsics, camera_to_map, *, pixel_stride=8):
        if type(pixel_stride) is not int or pixel_stride<1:raise ValueError('positive sampling stride required')
        depth=np.asarray(depth_m,float);matrix=np.asarray(camera_to_map,float)
        if depth.ndim!=2 or matrix.shape!=(4,4) or not np.isfinite(matrix).all():
            raise ValueError('depth image and finite transform required')
        valid=np.zeros(depth.shape,bool);valid[::pixel_stride,::pixel_stride]=True
        valid&=np.isfinite(depth)&(depth>=self.profile.minimum_depth_m)&(depth<=self.profile.maximum_depth_m)
        points=native_endpoints(depth,valid,intrinsics,matrix)
        return self.integrate_points(points,matrix[:3,3],matrix[:3,2])

    def arrays(self):
        keys=np.array(sorted(self.cells),dtype=np.int32).reshape(-1,3)
        values=np.array([self.cells[tuple(k)] for k in keys],float).reshape(-1,3)
        occupied=(values[:,0]>=2)&(values[:,2]>=.7)
        cleared=(values[:,1]>=2)&(values[:,2]<=-.4)
        return dict(voxels=keys,hit_frames=values[:,0].astype(np.int32),
                    clear_frames=values[:,1].astype(np.int32),evidence_score=values[:,2],
                    occupied=occupied,cleared=cleared,uncertain=~(occupied|cleared))


def review_recorded_depth(capture, slam, trajectory, output):
    from .recorded_campaign import verify_rgbd_attachment
    from .recorded_semantics import RecordedMapPoses
    capture,slam,trajectory,output=map(Path,(capture,slam,trajectory,output))
    if output.exists():raise ValueError('new depth review output required')
    source=json.loads((capture/'report.json').read_text())
    sr=json.loads((slam/'report.json').read_text())
    if not sr.get('passed') or not sr.get('final_graph_poses'):raise ValueError('verified final graph required')
    verify_rgbd_attachment(capture,sr,trajectory)
    metadata=json.loads((capture/'rgbd/metadata.json').read_text())
    intr=metadata['native_depth_intrinsics']
    if any(abs(v)>1e-12 for v in intr['coeffs']):raise ValueError('rectified native depth required')
    with np.load(trajectory/'trajectories.npz') as data:prior=data['encoder'].copy()
    poses=RecordedMapPoses(prior,json.loads((slam/'slam-poses.json').read_text()),sr['replay_clock_offset_s'],final_graph=True)
    transforms={t['child']:np.array(t['matrix']) for t in source['sensor_frames']['transforms']}
    camera=transforms['color_camera_optical_frame']@transforms['depth_camera_optical_frame']
    camera[2,3]-=transforms['base_footprint'][2,3]
    frames=[json.loads(line) for line in (capture/'rgbd/frames.jsonl').read_text().splitlines()]
    eligible=[];excluded=[]
    for f in frames:
        if f.get('pose_covered') is False:
            excluded.append(dict(file=f['file'],reason='outside own lidar source coverage'));continue
        try:T,gap=poses.at(f['depth_time_s'])
        except ValueError as error:
            excluded.append(dict(file=f['file'],reason=str(error)));continue
        eligible.append((f,T))
    selected=[];previous=None;last_epoch=None
    for f,T in eligible:
        yaw=math.atan2(T[1,0],T[0,0])
        if (previous is None or f['source_epoch']!=last_epoch or
            np.linalg.norm(T[:2,2]-previous[0])>=.10 or
            abs(math.atan2(math.sin(yaw-previous[1]),math.cos(yaw-previous[1])))>=math.radians(10)):
            selected.append((f,T));previous=(T[:2,2],yaw);last_epoch=f['source_epoch']
    # Include the last image of each source, without fabricating intermediate data.
    ids={f['file'] for f,T in selected}
    for i,(f,T) in enumerate(eligible):
        if i==len(eligible)-1 or eligible[i+1][0]['source_epoch']!=f['source_epoch']:
            if f['file'] not in ids:selected.append((f,T));ids.add(f['file'])
    selected.sort(key=lambda row:row[0]['depth_time_s'])
    output.mkdir(parents=True)
    print(json.dumps(dict(selected=len(selected),eligible=len(eligible),excluded=len(excluded))),flush=True)
    evidence=RecordedRayEvidence();records=[]
    for n,(f,T) in enumerate(selected):
        path=(capture/'rgbd'/f['file']).resolve()
        if not path.is_relative_to((capture/'rgbd').resolve()):raise ValueError('archive path escapes capture')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=f['sha256']:raise ValueError('archive hash mismatch')
        with np.load(path,allow_pickle=False) as raw:depth=raw['native_depth'].astype(float)*metadata['depth_scale_m']
        world=np.eye(4);world[:2,:2]=T[:2,:2];world[:2,3]=T[:2,2];world=world@camera
        result=evidence.integrate(depth,[intr[k] for k in ('fx','fy','ppx','ppy')],world,pixel_stride=8)
        records.append(dict(capture=f,camera_to_map=world.tolist(),**result))
        if (n+1)%20==0:print(json.dumps(dict(processed=n+1,total=len(selected),voxels=len(evidence.cells))),flush=True)
    arrays=evidence.arrays();np.savez_compressed(output/'ray-evidence.npz',**arrays)
    np.savez_compressed(output/'preview.npz',depth_voxels=arrays['voxels'][arrays['occupied']],
                        slam_poses=poses.rows,wheel_poses=np.empty((0,3)))
    report=dict(mode='offline_recorded_ray_evidence_review',hardware_commands=0,live_map_modified=False,
        navigation_authorized=False,depth_motion_qualified=False,free_space_certified=False,
        map_sha256=hashlib.sha256((slam/'occupancy.json').read_bytes()).hexdigest(),
        selected_frames=len(selected),eligible_frames=len(eligible),raw_frames_retained=len(frames),excluded_frames=excluded,
        source_epochs=sorted({f['source_epoch'] for f,T in selected}),pixel_stride=8,
        selection=dict(minimum_translation_m=.10,minimum_yaw_degrees=10,source_endpoints_included=True),
        evidence_policy=dict(hit_increment=.85,clear_increment=-.4,score_min=-2.,score_max=3.5,
                             occupied_min_score=.7,occupied_min_hit_frames=2,cleared_max_score=-.4,cleared_min_frames=2),
        minimum_height_m=.10,maximum_height_m=1.15,voxel_size_m=.05,
        occupied_voxels=int(arrays['occupied'].sum()),cleared_voxels=int(arrays['cleared'].sum()),
        uncertain_voxels=int(arrays['uncertain'].sum()),records=records,
        limitations=['Offline sampled rays; not full-resolution, full-height free-space certification.',
                     'Scores combine historical hit/clear evidence and are not calibrated probabilities.',
                     'Camera extrinsics and final graph alignment retain their recorded uncertainty.',
                     'No blanket removal along the driving path; trajectory is used for independent contradiction checks.'],
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [capture/'report.json',capture/'rgbd/metadata.json',capture/'rgbd/frames.jsonl',
             slam/'occupancy.json',slam/'slam-poses.json',trajectory/'trajectories.npz']})
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return {k:report[k] for k in ['selected_frames','eligible_frames','occupied_voxels','cleared_voxels','uncertain_voxels']}
