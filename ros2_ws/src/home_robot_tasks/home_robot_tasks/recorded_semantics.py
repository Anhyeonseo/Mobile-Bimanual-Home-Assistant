"""Offline recorded RGB/native-depth registration and semantic map review.

Reuses measured camera calibration, encoder interpolation, SLAM map corrections
and RGBDFurnitureMapper. No motor/device connection or live occupancy authority.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from .navigation_depth import native_endpoints


def clip_recorded_boxes(detections,shape):
    """Bound model boxes to visible pixels; keep unclipped boxes for audit."""
    height,width=shape[:2];result=[]
    for detection in detections:
        a,b,c,d=detection['box_xyxy']
        if not np.isfinite([a,b,c,d]).all() or a>=c or b>=d:raise ValueError('invalid model box')
        box=[max(0.,a),max(0.,b),min(float(width),c),min(float(height),d)]
        if box[0]>=box[2] or box[1]>=box[3]:continue
        result.append(dict(detection,box_xyxy=box,unclipped_box_xyxy=[a,b,c,d]))
    return result


def register_native_depth(raw, metadata):
    """Project measured pixel centres; nearest Z wins, holes remain unobserved.

    This is a conservative pinhole registration, not an SDK rs.align export.
    Nonzero distortion is refused rather than silently ignored.
    """
    source=metadata['native_depth_intrinsics'];target=metadata['color_intrinsics']
    for intr in (source,target):
        values=[intr[k] for k in ('fx','fy','ppx','ppy')]
        if (not np.isfinite(values).all() or min(values[:2])<=0 or
                not 0<=intr['ppx']<intr['width'] or not 0<=intr['ppy']<intr['height'] or
                any(abs(c)>1e-12 for c in intr['coeffs'])):
            raise ValueError('rectified finite camera intrinsics required')
    if raw.dtype!=np.uint16 or raw.shape!=(source['height'],source['width']):
        raise ValueError('native depth shape/type mismatch')
    scale=metadata['depth_scale_m']
    if not np.isfinite(scale) or not 0<scale<.1:raise ValueError('invalid depth scale')
    extr=metadata['depth_to_color'];matrix=np.eye(4)
    matrix[:3,:3]=np.asarray(extr['rotation_column_major']).reshape(3,3,order='F')
    matrix[:3,3]=extr['translation_m']
    from .s2_geometry import rigid
    rigid(matrix)
    depth=raw.astype(float)*scale
    points=native_endpoints(depth,(depth>=.12)&(depth<=5.),
        [source[k] for k in ('fx','fy','ppx','ppy')],matrix)
    points=points[points[:,2]>.01]
    uv=np.rint(points[:,:2]/points[:,2:]*[target['fx'],target['fy']]+[target['ppx'],target['ppy']]).astype(int)
    valid=(uv[:,0]>=0)&(uv[:,0]<target['width'])&(uv[:,1]>=0)&(uv[:,1]<target['height'])
    aligned=np.full(target['height']*target['width'],np.inf)
    np.minimum.at(aligned,uv[valid,1]*target['width']+uv[valid,0],points[valid,2])
    aligned[~np.isfinite(aligned)]=0
    return aligned.reshape(target['height'],target['width'])


class RecordedMapPoses:
    def __init__(self, odom, slam_poses, clock_offset, *, final_graph=False):
        from .s4_recorded_mapping import interpolate, transform, pose
        self.odom=odom
        self.final_graph=final_graph
        self.rows=np.array([[p['stamp']-clock_offset,*p['xy_yaw']] for p in slam_poses])
        self.rows[:,3]=np.unwrap(self.rows[:,3])
        wheel=interpolate(odom,self.rows[:,0])
        self.corrections=np.array([[r[0],*pose(transform(r[1:])@np.linalg.inv(transform(w)))]
            for r,w in zip(self.rows,wheel)])
        self.corrections[:,3]=np.unwrap(self.corrections[:,3])

    def at(self, stamp, maximum_correction_gap_s=.75):
        from .s4_recorded_mapping import interpolate, transform, pose
        index=int(np.searchsorted(self.rows[:,0],stamp,side='right'))
        if not 0<index<len(self.rows):raise ValueError('outside recorded map pose coverage')
        gap=self.rows[index,0]-self.rows[index-1,0]
        if gap>maximum_correction_gap_s:
            # A final optimized graph intentionally keeps sparse keyframes.
            # Dense recorded lidar motion may bridge these; an online stream
            # still uses the original conservative freshness requirement.
            left,right=self.rows[index-1,0],self.rows[index,0]
            sample_times=self.odom[(self.odom[:,0]>=left)&(self.odom[:,0]<=right),0]
            # Evaluate a correction at the robot, not at the arbitrary odom
            # origin. A small rotation far from that origin otherwise looks
            # like a large translation and wrongly drops stationary restarts.
            anchor=transform(interpolate(self.odom,[(left+right)/2])[0])
            correction_delta=pose(np.linalg.inv(transform(self.corrections[index-1,1:])@anchor)
                                  @(transform(self.corrections[index,1:])@anchor))
            if (not self.final_graph or len(sample_times)<2
                    or np.max(np.diff(np.r_[left,sample_times,right]))>2.
                    or np.linalg.norm(correction_delta[:2])>.5 or abs(correction_delta[2])>np.deg2rad(20)):
                raise ValueError('sparse SLAM correction interval')
        matrix=transform(interpolate(self.corrections,[stamp])[0])@transform(interpolate(self.odom,[stamp])[0])
        return matrix,float(gap)


def choose_recorded_views(eligible, poses, count):
    """Cover each source recording, then add spatially different viewpoints."""
    from .s4_recorded_mapping import pose
    groups={}
    for i,row in enumerate(eligible):groups.setdefault(row.get('source_epoch','single'),[]).append(i)
    if len(groups)>count:
        return [eligible[i] for i in np.unique(np.linspace(0,len(eligible)-1,count,dtype=int))]
    chosen=[indices[len(indices)//2] for indices in groups.values()]
    locations=np.array([pose(poses.at(row['color_time_s'])[0]) for row in eligible])
    while len(chosen)<min(count,len(eligible)):
        distance=np.full(len(eligible),np.inf)
        for index in chosen:
            yaw=(locations[:,2]-locations[index,2]+np.pi)%(2*np.pi)-np.pi
            score=np.linalg.norm(locations[:,:2]-locations[index,:2],axis=1)/.5+np.abs(yaw)/(np.pi/3)
            distance=np.minimum(distance,score)
        distance[chosen]=-1
        chosen.append(int(np.argmax(distance)))
    return [eligible[i] for i in sorted(chosen)]


def landmark_view_support(store,instances):
    """Separate repeated frames from viewpoint diversity; neither certifies identity."""
    captures={c['stamp']['capture_id']:c for c in store.records('semantic_capture')}
    observations=store.records('furniture_observation');result=[]
    for item in instances:
        ids=sorted({o['observation']['stamp']['capture_id'] for o in observations
                    if o['association']['instance_id']==item['instance_id']})
        representatives=[]
        for capture_id in ids:
            if capture_id not in captures:continue
            matrix=np.array(captures[capture_id]['camera_to_map']);origin=matrix[:3,3];direction=matrix[:3,2]
            if not any(np.linalg.norm(origin-p)<.08 and direction@d>np.cos(np.deg2rad(10)) for p,d in representatives):
                representatives.append((origin,direction))
        value=dict(instance_id=item['instance_id'],category=item['category'],distinct_captures=len(ids),
            viewpoint_groups=len(representatives),minimum_translation_m=.08,minimum_rotation_deg=10.,
            status='MULTIVIEW_ESTIMATE' if len(representatives)>=2 else 'SINGLE_VIEW_CANDIDATE',
            identity_verified=False,independent_correctness_verified=False)
        store.save_record('furniture_view_support',item['instance_id'],value);result.append(value)
    return result


def spatial_landmark_review(instances, frames):
    """Group unique overlapping storage hypotheses; retain raw instances and conflicts.

    This is a display/memory review, not learned re-identification. A broad mask
    touching two objects never joins them into a transitive mega-object.
    """
    storage={'wardrobe','cabinet','bookshelf','refrigerator','unclassified_furniture'}
    cells=[{tuple(c) for c in i['observed_voxels']} for i in instances]
    adjacent=[[] for _ in instances]
    for a,left in enumerate(instances):
        for b in range(a+1,len(instances)):
            right=instances[b]
            if (left['category'] not in storage or right['category'] not in storage or
                    'unclassified_furniture' not in (left['category'],right['category']) or
                    left['voxel_size_m']!=right['voxel_size_m'] or left['context']!=right['context']):continue
            overlap=len(cells[a]&cells[b])
            if overlap/min(len(cells[a]),len(cells[b]))>=.6 and overlap/max(len(cells[a]),len(cells[b]))>=.1:
                adjacent[a].append(b);adjacent[b].append(a)
    groups=[];used=set()
    for i in range(len(instances)):
        if i in used:continue
        group=[i]
        if len(adjacent[i])==1 and adjacent[adjacent[i][0]]==[i]:group+=adjacent[i]
        used.update(group);groups.append(group)
    result=[]
    for group in groups:
        members=sorted(instances[i]['instance_id'] for i in group)
        categories={instances[i]['category'] for i in group};observed=[];alternatives=set(categories)-{'unclassified_furniture'}
        for frame in frames:
            mapping=frame.get('mapping',{});associations=mapping.get('associations',[])
            accepted=mapping.get('provenance',{}).get('accepted_detections',[])
            if not any(a.get('instance_id') in members for a in associations):continue
            observed.append(frame)
            for detection,association in zip(accepted,associations):
                if association.get('instance_id') not in members:continue
                corners=np.asarray(detection['corners_uv']);box=np.r_[corners.min(0),corners.max(0)]
                matches=[d for d in frame['detections'] if np.max(abs(np.asarray(d['box_xyxy'])-box))<=1.01]
                if len(matches)==1:
                    evidence=matches[0];original=evidence.get('original_label',evidence['label']).replace('_',' ').split()
                    alternatives.update(set(original)&storage-{'unclassified_furniture'})
                    for ranking in evidence.get('household_review',{}).get('alternatives',[]):
                        alternatives.update(v['label'] for v in ranking[:2] if v['label'] in storage and v['label']!='unclassified_furniture')
        representatives=[]
        for frame in observed:
            matrix=np.array(frame['camera_to_map']);origin=matrix[:3,3];direction=matrix[:3,2]
            if not any(np.linalg.norm(origin-p)<.08 and direction@d>np.cos(np.deg2rad(10)) for p,d in representatives):
                representatives.append((origin,direction))
        merged=set().union(*(cells[i] for i in group));size=instances[group[0]]['voxel_size_m']
        category=next(iter(categories)) if len(categories)==1 else 'unclassified_furniture'
        result.append(dict(landmark_id='landmark-'+hashlib.sha256(':'.join(members).encode()).hexdigest()[:24],
            member_instance_ids=members,category=category,label_alternatives=sorted(alternatives),
            observed_voxels=[list(c) for c in sorted(merged)],voxel_size_m=size,
            distinct_captures=len({f['capture']['capture_id'] for f in observed}),viewpoint_groups=len(representatives),
            association='unique_overlapping_storage_hypotheses' if len(group)>1 else 'existing_instance',
            label_state='NEEDS_REVIEW' if category=='unclassified_furniture' else 'MODEL_ESTIMATE',
            identity_verified=False,physical_extent_complete=False,collision_clearance_authority=False,
            ambiguous_neighbor_ids=sorted(instances[j]['instance_id'] for i in group for j in adjacent[i] if j not in group)))
    return result


def review_semantics(root,capture,slam,trajectory,output,model,segmenter,*,model_revision,frames=6,household_review=False):
    from .camera_evidence import from_metadata,match_observed_identity
    from .assistant.contracts import Context,Stamp
    from .assistant.perception import TransformSample
    from .assistant.world import WorldStore
    from .semantic_mapping import RGBDFurnitureMapper
    from .semantic_mapping import furniture_category
    from .assistant.furniture import FurnitureMemory
    from .fetch import InvalidTask
    import cv2
    root,capture,slam,trajectory,output=map(Path,(root,capture,slam,trajectory,output))
    if output.exists() or type(frames) is not int or not 1<=frames<=120:raise ValueError('new output and 1..120 frames required')
    sr=json.loads((slam/'report.json').read_text());tr=json.loads((trajectory/'report.json').read_text())
    if sr.get('passed') is not True or not sr.get('map_cells_identical_after_restart'):raise ValueError('verified saved SLAM required')
    capture_report=json.loads((capture/'report.json').read_text())
    if capture_report.get('mode')=='offline_lidar_rgbd_attachment':
        from .recorded_campaign import verify_rgbd_attachment
        verify_rgbd_attachment(capture,sr,trajectory)
    else:
        for report in (sr,tr):
            for name in ('scans.json','report.json'):
                expected=[v for k,v in report['source_sha256'].items() if k.endswith('/'+name)]
                if expected!=[hashlib.sha256((capture/name).read_bytes()).hexdigest()]:raise ValueError('cross-capture map/trajectory refused')
    metadata=json.loads((capture/'rgbd/metadata.json').read_text())
    match_observed_identity(from_metadata(metadata),json.loads((root/'config/home_assistant/camera_device.observed.json').read_text()))
    with np.load(trajectory/'trajectories.npz',allow_pickle=False) as data:odom=data['encoder']
    poses=RecordedMapPoses(odom,json.loads((slam/'slam-poses.json').read_text()),sr['replay_clock_offset_s'],
        final_graph=sr.get('final_graph_poses') is True and tr.get('trajectory_basis','').startswith('lidar_'))
    manifest=[json.loads(l) for l in (capture/'rgbd/frames.jsonl').read_text().splitlines()]
    eligible=[];excluded=[]
    for row in manifest:
        try:
            if row.get('pose_covered') is False:raise ValueError('outside source segment lidar pose coverage')
            if abs(row['color_time_s']-row['depth_time_s'])>.05:raise ValueError('RGB/depth exposure skew')
            poses.at(row['color_time_s']);eligible.append(row)
        except ValueError as e:excluded.append(dict(file=row['file'],reason=str(e)))
    if not eligible:raise ValueError('no capture-time map poses')
    chosen=choose_recorded_views(eligible,poses,frames)
    report=json.loads((capture/'report.json').read_text())
    transforms={t['child']:np.array(t['matrix']) for t in report['sensor_frames']['transforms']}
    optical=transforms['color_camera_optical_frame'].copy()
    optical[2,3]-=transforms['base_footprint'][2,3]
    map_sha=hashlib.sha256((slam/'occupancy.json').read_bytes()).hexdigest()
    context=Context('recorded-semantic-room','map-'+map_sha[:24],report['sensor_frames']['calibration_id'],'recorded-rgbd-review')
    output.mkdir(parents=True);store=WorldStore(output/'furniture.db')
    mapper=RGBDFurnitureMapper(store,context,model,segmenter,model_revision=model_revision,floor_z_m=0. if household_review else None)
    results=[]
    try:
        for number,row in enumerate(chosen):
            path=(capture/'rgbd'/row['file']).resolve()
            if not path.is_relative_to((capture/'rgbd').resolve()) or hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:
                raise ValueError('archive path/hash mismatch')
            with np.load(path,allow_pickle=False) as data:rgb=data['color_bgr'][...,::-1].copy();raw=data['native_depth'].copy()
            registered=register_native_depth(raw,metadata)
            matrix,gap=poses.at(row['color_time_s']);world=np.eye(4);world[:2,:2]=matrix[:2,:2];world[:2,3]=matrix[:2,2];world=world@optical
            capture_id=(row['source_epoch']+'-'+row['capture_id']) if 'source_epoch' in row else row['capture_id']
            stamp=Stamp(row['color_time_s'],context,capture_id)
            tf=TransformSample(stamp,'color_camera_optical_frame','map',world,'measured')
            entry=dict(capture=row,correction_gap_s=gap,registration='native pixel centres to RGB, nearest color-Z, no filling',
                aligned_valid_fraction=float(np.mean(registered>0)),camera_to_map=world.tolist())
            try:
                entry['mapping']=mapper.observe(rgb,registered,[metadata['color_intrinsics'][k] for k in ('fx','fy','ppx','ppy')],stamp=stamp,transform=tf,now_s=stamp.observed_s)
            except InvalidTask as e:entry['reobserve_reason']=str(e)
            # Save the exact source view; detector output is recorded by the wrapper.
            canvas=rgb[...,::-1].copy();detections=getattr(model,'last_detections',[])
            entry['detections']=detections
            entry['unresolved_labels']=[d['label'] for d in detections if furniture_category(d['label']) is None]
            for det in entry.get('mapping',{}).get('provenance',{}).get('accepted_detections',[]):
                corners=np.array(det['corners_uv']);x1,y1=np.floor(corners.min(0)).astype(int);x2,y2=np.ceil(corners.max(0)).astype(int)
                cv2.rectangle(canvas,(x1,y1),(x2,y2),(0,180,255),2)
                cv2.putText(canvas,f"{det['category']} {det['score']:.2f}",(max(0,x1),max(16,y1)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,80,255),1)
            cv2.imwrite(str(output/f'view-{number:02d}.jpg'),canvas)
            entry['image']=f'view-{number:02d}.jpg';results.append(entry)
            (output/'progress.json').write_text(json.dumps(results,indent=2)+'\n')
        instances=mapper.memory.instances()
        view_support=landmark_view_support(store,instances)
        landmarks=spatial_landmark_review(instances,results)
        for landmark in landmarks:store.save_record('furniture_landmark_review',landmark['landmark_id'],landmark)
    finally:store.close()
    reopened=WorldStore(output/'furniture.db')
    try:
        restored=FurnitureMemory(reopened,context).instances()
        restored_view_support=reopened.records('furniture_view_support')
        restored_landmarks=reopened.records('furniture_landmark_review')
    finally:reopened.close()
    from .assistant.furniture import FURNITURE_PROMPT
    conflicts=[]
    for i,left in enumerate(instances):
        a={tuple(c) for c in left['observed_voxels']}
        for right in instances[i+1:]:
            b={tuple(c) for c in right['observed_voxels']}
            overlap=len(a&b)/min(len(a),len(b))
            if left['category']!=right['category'] and overlap>=.2:
                conflicts.append(dict(instance_ids=[left['instance_id'],right['instance_id']],
                    categories=[left['category'],right['category']],overlap_fraction=overlap,
                    resolution='unresolved; do not treat as two confirmed objects'))
    result=dict(mode='offline_real_rgbd_semantic_review',context=context.to_dict(),model_revision=model_revision,
        landmarks=landmarks,persisted_landmarks_identical=sorted(landmarks,key=lambda x:x['landmark_id'])==restored_landmarks,
        prompt=FURNITURE_PROMPT,floor_filter_enabled=household_review,classifier_review_enabled=getattr(model,'household',None) is not None,view_support=view_support,model_versions=getattr(model,'versions',{}),cross_category_conflicts=conflicts,
        landmark_policy='major household furniture; unnamed objects remain in independent native depth',
        repeated_view_landmarks=[i['instance_id'] for i in instances if i['observation_count']>=2],
        tentative_single_view_landmarks=[i['instance_id'] for i in instances if i['observation_count']<2],
        frames=results,instances=instances,persisted_instances_identical=instances==restored,persisted_view_support_identical=view_support==restored_view_support,excluded_frames=excluded,
        eligible_frames=len(eligible),map_sha256=map_sha,motor_commands=0,live_map_modified=False,
        semantic_names_physically_confirmed=False,full_furniture_extent_verified=False,self_geometry_removed=False,
        obstacle_clearing_authorized=False,autonomous_navigation_authorized=False,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (capture/'rgbd/metadata.json',capture/'rgbd/frames.jsonl',slam/'report.json',trajectory/'report.json')},
        limitations=['Recorded-time interpolation with nominal sensor mounting, not surveyed object locations.',
            'Pixel-centre registration leaves holes and uses measured color-Z; no SDK alignment identity claimed.',
            'Semantic voxel overlap estimates instance identity; moving furniture requires new geometry observations.',
            'Robot self-geometry and full-height free space unqualified; detections cannot clear obstacles.'])
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
