#!/usr/bin/env python3
"""Project a saved real sensor capture into floor XY. No live-map or free-space claims."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.camera_evidence import from_metadata, match_observed_identity
from home_robot_tasks.navigation_depth import native_endpoints
from home_robot_tasks.s4_sensor_frames import load_sensor_frames


def review(capture, output):
    capture,output=Path(capture),Path(output)
    report=json.loads((capture/'report.json').read_text())
    if report.get('mode')!='real_sensor' or report.get('passed') is not True:
        raise ValueError('passed real sensor capture required')
    metadata=json.loads((capture/'rgbd-metadata.json').read_text())
    reference=json.loads((ROOT/'config/home_assistant/camera_device.observed.json').read_text())
    match_observed_identity(from_metadata(metadata),reference)
    intr=metadata['native_depth_intrinsics']
    if any(abs(c)>1e-12 for c in intr['coeffs']):
        raise ValueError('rectified native depth required; distorted pinhole projection refused')
    with np.load(capture/'rgbd-last.npz',allow_pickle=False) as images:
        raw=images['native_depth']
        if raw.shape!=(intr['height'],intr['width']) or raw.dtype!=np.uint16:
            raise ValueError('native depth shape or type mismatch')
        depth=raw.astype(float)*metadata['depth_scale_m']
    entries=load_sensor_frames(ROOT,depth_to_color=metadata['depth_to_color'],navigation_root=True)['transforms']
    matrices={t['child']:np.array(t['matrix']) for t in entries}
    floor_from_depth=matrices['base_link']@matrices['color_camera_optical_frame']@matrices['depth_camera_optical_frame']
    valid=(depth>=.12)&(depth<=4.)
    points=native_endpoints(depth,valid,[intr[k] for k in ('fx','fy','ppx','ppy')],floor_from_depth)
    scans=json.loads((capture/'scans.json').read_text())
    scan=scans[-1]
    if scan['frame']!='lidar_a':raise ValueError('unexpected scan frame')
    ranges=np.array([np.nan if r is None else r for r in scan['ranges']])
    angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    laser=np.column_stack((ranges*np.cos(angles),ranges*np.sin(angles),np.zeros(len(ranges))))
    transform=matrices['base_link']@matrices['lidar_a']
    laser=laser@transform[:3,:3].T+transform[:3,3]
    laser=laser[np.isfinite(laser).all(axis=1)]
    resolution=.05
    above=points[(points[:,2]>transform[2,3]+.05)&(points[:,2]<1.5)]
    cells=np.unique(np.floor(above[:,:2]/resolution).astype(int),axis=0)
    output.mkdir(parents=True,exist_ok=False)
    # Endpoints only: unknown cells are never labelled free by this review.
    result=dict(mode='offline_stationary_endpoint_review',hardware_enabled=False,motion_authorized=False,
        live_map=False,localization_supplied=False,free_space_inferred=False,precision_calibration_verified=False,
        native_valid_points=len(points),lidar_valid_points=len(laser),
        above_lidar_plane_points=len(above),above_lidar_plane_xy_cells=len(cells),resolution_m=resolution,
        height_percentiles_m=np.percentile(points[:,2],[5,50,95]).tolist() if len(points) else [],
        source_sha256={name:hashlib.sha256((capture/name).read_bytes()).hexdigest() for name in
            ('report.json','rgbd-metadata.json','rgbd-last.npz','scans.json')},
        limitations=['current accepted nominal mounting, not independently calibrated',
                     'above-plane depth endpoints demonstrate extra height observations, not collision-free travel',
                     'snapshot geometry only; no moving-frame synchronization or realtime processing acceptance'])
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    np.savez_compressed(output/'endpoints.npz',depth_floor_m=points,lidar_floor_m=laser,
                        above_lidar_xy_cells=cells)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(8,7))
    if len(cells):ax.scatter((cells[:,0]+.5)*resolution,(cells[:,1]+.5)*resolution,s=8,c='#3e81c4',label='Depth endpoints above lidar plane')
    ax.scatter(laser[:,0],laser[:,1],s=10,c='#d97723',label='Lidar endpoints')
    ax.scatter([0],[0],marker='>',s=70,c='black',label='Robot reference (+X forward)')
    ax.set(xlabel='Forward X (m)',ylabel='Left Y (m)',title='Stationary sensor geometry — unknown space remains unknown',aspect='equal')
    ax.grid(alpha=.25);ax.legend(loc='best');fig.tight_layout();fig.savefig(output/'sensor-endpoints.png',dpi=160);plt.close(fig)
    return result


def review_moving(capture, slam, trajectory_review, output):
    """Visual quality review only; do not ingest unqualified motion into live maps."""
    from scipy.spatial import cKDTree
    from home_robot_tasks.s4_recorded_mapping import interpolate, transform, pose, scan_points
    capture,slam,trajectory_review,output=map(Path,(capture,slam,trajectory_review,output))
    if output.exists():raise ValueError('new output directory required')
    sr=json.loads((slam/'report.json').read_text())
    if sr.get('passed') is not True:raise ValueError('completed SLAM checkpoint replay required')
    capture_report=json.loads((capture/'report.json').read_text())
    if capture_report.get('mode')=='offline_lidar_rgbd_attachment':
        from home_robot_tasks.recorded_campaign import verify_rgbd_attachment
        verify_rgbd_attachment(capture,sr,trajectory_review)
    prior_basis=sr.get('trajectory_prior',sr.get('encoder',{}).get('source','nominal_wheel_encoder'))
    prior_label='Lidar prior' if prior_basis.startswith('lidar_') else 'Wheel prior'
    saved=json.loads((slam/'occupancy.json').read_text());poses=json.loads((slam/'slam-poses.json').read_text())
    clock=sr['replay_clock_offset_s']
    rows=np.array([[p['stamp']-clock,*p['xy_yaw']] for p in poses]);rows[:,3]=np.unwrap(rows[:,3])
    if len(rows)<2:raise ValueError('SLAM pose coverage insufficient')
    with np.load(trajectory_review/'trajectories.npz') as data:odom=data['encoder']
    baseline=transform(interpolate(odom,[rows[0,0]])[0]);inv=np.linalg.inv(baseline)
    wheel=np.array([pose(inv@transform(p)) for p in interpolate(odom,rows[:,0])]);wheel[:,2]=np.unwrap(wheel[:,2])
    error=rows[:,1:]-wheel
    # Interpolate sparse map-to-odom corrections, retaining measured encoder
    # motion between processed scans; never extrapolate beyond SLAM coverage.
    corrections=np.array([[r[0],*pose(transform(r[1:])@np.linalg.inv(transform(p)))]
                          for r,p in zip(rows,interpolate(odom,rows[:,0]))]);corrections[:,3]=np.unwrap(corrections[:,3])
    def map_pose(t):
        return transform(interpolate(corrections,[t])[0])@transform(interpolate(odom,[t])[0])
    grid=np.array(saved['data']).reshape(saved['height'],saved['width']);resolution=saved['resolution'];origin=np.array(saved['origin'])
    yy,xx=np.nonzero(grid>=65);occupied=origin+(np.column_stack((xx,yy))+.5)*resolution;tree=cKDTree(occupied)
    report=json.loads((capture/'report.json').read_text())
    transforms={x['child']:np.array(x['matrix']) for x in report['sensor_frames']['transforms']}
    m=transforms['lidar_a'];mount=np.array([[m[0,0],m[0,1],m[0,3]],[m[1,0],m[1,1],m[1,3]],[0,0,1]])
    distances=[];nominal_distances=[];heldout=0
    scans=json.loads((capture/'scans.json').read_text())
    for s in scans:
        t=s['observed_s']
        if not rows[0,0]<=t<=rows[-1,0] or t+359*s['time_increment']<odom[0,0]:continue
        if np.min(abs(rows[:,0]-t))<.02:continue
        p=scan_points(s,mount,odom);T=map_pose(t);W=inv@transform(interpolate(odom,[t])[0])
        q=p@T[:2,:2].T+T[:2,2];distances.extend(tree.query(q)[0].tolist())
        nominal_distances.extend(tree.query(p@W[:2,:2].T+W[:2,2])[0].tolist());heldout+=1
    def stats(d):
        return dict(endpoints=len(d),median_m=float(np.median(d)),p90_m=float(np.percentile(d,90)),within_10cm_fraction=float(np.mean(np.array(d)<=.1))) if len(d) else None
    metadata=json.loads((capture/'rgbd/metadata.json').read_text());intr=metadata['native_depth_intrinsics']
    if any(abs(v)>1e-12 for v in intr['coeffs']):raise ValueError('rectified native depth required')
    # Static lift height is the recorded lowest setting; no invented lift motion.
    base_from_depth=transforms['color_camera_optical_frame']@transforms['depth_camera_optical_frame']
    base_from_depth[2,3]-=transforms['base_footprint'][2,3]
    voxels=[];covered=0
    for line in (capture/'rgbd/frames.jsonl').read_text().splitlines():
        frame=json.loads(line);t=frame['depth_time_s']
        if frame.get('pose_covered') is False:continue
        if not rows[0,0]<=t<=rows[-1,0]:continue
        path=(capture/'rgbd'/frame['file']).resolve()
        if not path.is_relative_to((capture/'rgbd').resolve()):raise ValueError('invalid archive path')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=frame['sha256']:raise ValueError('archive changed')
        with np.load(path,allow_pickle=False) as image:depth=image['native_depth'].astype(float)*metadata['depth_scale_m']
        valid=np.zeros(depth.shape,dtype=bool);valid[::8,::8]=True;valid&=(depth>=.12)&(depth<=3.5)
        p=native_endpoints(depth,valid,[intr[k] for k in ('fx','fy','ppx','ppy')],base_from_depth)
        p=p[(p[:,2]>=0)&(p[:,2]<=1.5)];T=map_pose(t);p[:,:2]=p[:,:2]@T[:2,:2].T+T[:2,2]
        voxels.append(np.unique(np.floor(p/.05).astype(int),axis=0));covered+=1
    cells=np.unique(np.vstack(voxels),axis=0) if voxels else np.empty((0,3),dtype=int)
    points=(cells+.5)*.05;above=points[points[:,2]>report['scan_height_above_floor_m']+.05]
    inside=np.floor((above[:,:2]-origin)/resolution).astype(int);valid=(inside[:,0]>=0)&(inside[:,0]<grid.shape[1])&(inside[:,1]>=0)&(inside[:,1]<grid.shape[0]);inside=inside[valid]
    free_cells=np.unique(inside[grid[inside[:,1],inside[:,0]]==0],axis=0)
    result=dict(mode='offline_recorded_map_quality_preview',hardware_commands=0,live_map_modified=False,
        physical_accuracy_verified=False,depth_motion_qualified=False,navigation_authorized=False,
        slam_pose_count=len(rows),slam_pose_time_coverage_s=float(rows[-1,0]-rows[0,0]),
        maximum_slam_pose_gap_s=float(np.max(np.diff(rows[:,0]))),
        nominal_path_length_m=sr['encoder']['path_length_m'],
        nominal_endpoint_displacement_m=float(np.linalg.norm(sr['encoder']['final_pose'][:2])),
        compared_encoder_pose=wheel[-1].tolist(),compared_slam_pose=rows[-1,1:].tolist(),
        scan_vs_encoder_final_translation_difference_m=float(np.linalg.norm(error[-1,:2])),
        scan_vs_encoder_final_heading_difference_deg=float(np.rad2deg(error[-1,2])),
        heldout_scans=heldout,heldout_endpoint_to_map=stats(distances),nominal_endpoint_to_map=stats(nominal_distances),
        depth_preview_frames=covered,preview_voxels=len(cells),above_lidar_voxels=len(above),
        lidar_free_cells_with_upper_depth_endpoints=len(free_cells),
        limitations=['map fit is internal consistency, not an independent surveyed distance',
          ('final optimized graph corrections interpolated with recorded motion' if sr.get('final_graph_poses') else
           'map-to-odom corrections interpolated between online SLAM poses; not a final optimized graph trajectory'),
          '8-pixel stride is for preview only; original full-resolution native depth retained',
          'upper endpoints are review candidates; no free space, obstacle clearing or semantic labels inferred'])
    result['trajectory_prior_basis']=prior_basis
    if prior_basis.startswith('lidar_'):
        for old,new in [('nominal_path_length_m','lidar_prior_path_length_m'),
                ('nominal_endpoint_displacement_m','lidar_prior_endpoint_displacement_m'),
                ('compared_encoder_pose','compared_lidar_prior_pose'),
                ('scan_vs_encoder_final_translation_difference_m','slam_vs_lidar_prior_final_translation_difference_m'),
                ('scan_vs_encoder_final_heading_difference_deg','slam_vs_lidar_prior_final_heading_difference_deg'),
                ('nominal_endpoint_to_map','lidar_prior_endpoint_to_map')]:result[new]=result.pop(old)
    output.mkdir(parents=True);(output/'report.json').write_text(json.dumps(result,indent=2)+'\n');np.savez_compressed(output/'preview.npz',depth_voxels=cells,slam_poses=rows,wheel_poses=wheel)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    fig,axes=plt.subplots(1,3,figsize=(15,5));extent=[origin[0],origin[0]+grid.shape[1]*resolution,origin[1],origin[1]+grid.shape[0]*resolution]
    display=np.where(grid<0,0,np.where(grid>=65,2,1))
    for ax in axes[:2]:ax.imshow(display,origin='lower',extent=extent,cmap=ListedColormap(['#aeb8c2','#fafafa','#17222c']),vmin=0,vmax=2);ax.set(xlabel='X (m)',ylabel='Y (m)',aspect='equal')
    axes[0].plot(rows[:,1],rows[:,2],c='#008a76',label='SLAM pose');axes[0].set_title('Recorded lidar map / 5 cm cells');axes[0].legend()
    axes[1].scatter(above[:,0],above[:,1],s=.6,c='#cf711f',alpha=.35);axes[1].set_title('Upper depth endpoints / preview only')
    t=rows[:,0]-rows[0,0];axes[2].plot(t,np.rad2deg(wheel[:,2]),label=prior_label+' yaw');axes[2].plot(t,np.rad2deg(rows[:,3]),label='SLAM yaw');axes[2].set(xlabel='Time (s)',ylabel='Yaw (degrees)',title='Rotation consistency');axes[2].grid(alpha=.2);axes[2].legend()
    fig.suptitle('Recorded-data review — no autonomous navigation approval');fig.tight_layout();fig.savefig(output/'map-review.png',dpi=160);plt.close(fig)
    return result


def review_lidar(capture, slam, output):
    """Lidar-only map rendering and source accounting, without opening RGB-D."""
    from home_robot_tasks.recorded_campaign import load_prepared, digest, mask_body_shadow
    from home_robot_tasks.s4_recorded_mapping import localization_map_review, transform, interpolate, pose, scan_points
    capture, slam, output = map(Path,(capture,slam,output))
    odom, scans, prepared = load_prepared(capture)
    sr = json.loads((slam/'report.json').read_text())
    if not (prepared.get('lidar_only') and sr.get('lidar_only') and sr.get('passed') and sr.get('final_graph_poses')):
        raise ValueError('verified lidar-only map with final graph poses required')
    if output.exists():raise ValueError('new review directory required')
    for path, expected in prepared['source_sha256'].items():
        if digest(path) != expected:raise ValueError('raw source or calibration changed: '+path)
    grid = json.loads((slam/'occupancy.json').read_text())
    poses = json.loads((slam/'slam-poses.json').read_text())
    md = json.loads((ROOT/'config/home_assistant/lidar_mount.review.json').read_text())
    mount = transform([*md['base_from_lidar_translation_m'][:2],md['yaw_rad']])
    coverage = []
    for source in prepared['sources']:
        segment = source['segment']
        source_path = next(Path(p) for p in prepared['source_sha256'] if p.endswith(segment+'/capture/scans.json'))
        raw = json.loads(source_path.read_text())
        used = [s for s in scans if s['source_segment']==segment]
        identical = len(raw)==len(used) and all(
            s['source_observed_s']==r['observed_s'] and s['ranges']==mask_body_shadow(r,prepared['visibility'])[0]['ranges']
            for r,s in zip(raw,used))
        if not identical:raise ValueError('unaccounted scan or unmasked range modification: '+segment)
        coverage.append(dict(segment=segment,raw_scans=len(raw),used_scans=len(used),
            visible_ranges_identical=True,source_sha256=digest(source_path)))
    metrics = localization_map_review(poses,sr['replay_clock_offset_s'],scans,odom,mount,grid)
    result = dict(mode='offline_lidar_only_map_review',lidar_only=True,semantic_used=False,rgbd_used=False,
        wheel_feedback_used=False,source_coverage=coverage,all_recorded_scans_retained=True,
        input_scans=len(scans),graph_keyframes=len(poses),visibility=prepared['visibility'],
        graph_reload_identical=sr['map_cells_identical_after_restart'],
        map_directory=str(slam.resolve()),prepared_directory=str(capture.resolve()),
        quality=metrics,physical_accuracy_verified=False,navigation_authorized=False,
        limitations=['Map fit uses the same observations, not surveyed ground truth.',
            'All scans are retained as inputs; SLAM spatial keyframes are a smaller set.',
            'Unknown cells are not free space. Body-occluded rays contribute neither hits nor free rays.'])
    output.mkdir(parents=True)
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    cells=np.asarray(grid['data']).reshape(grid['height'],grid['width'])
    origin=np.asarray(grid['origin']);res=grid['resolution']
    extent=[origin[0],origin[0]+grid['width']*res,origin[1],origin[1]+grid['height']*res]
    display=np.where(cells<0,0,np.where(cells>=65,2,1))
    cmap=ListedColormap(['#c6ccd2','#ffffff','#17212b'])
    def background(ax):
        ax.imshow(display,origin='lower',extent=extent,cmap=cmap,vmin=0,vmax=2,interpolation='nearest')
        ax.set(xlabel='X (m)',ylabel='Y (m)',aspect='equal')
    fig,ax=plt.subplots(figsize=(10,9));background(ax)
    ax.set_title('Lidar-only 2D map | all 9 recording chunks | 5 cm cells',pad=14)
    fig.text(.5,.025,'Black: observed obstacle   White: observed free   Gray: unknown',ha='center')
    fig.tight_layout(rect=[0,.04,1,1]);fig.savefig(output/'lidar-map.png',dpi=180);plt.close(fig)
    rows=np.array([[p['stamp']-sr['replay_clock_offset_s'],*p['xy_yaw']]for p in poses]);rows[:,3]=np.unwrap(rows[:,3])
    prior=interpolate(odom,rows[:,0])
    corrections=np.array([[r[0],*pose(transform(r[1:])@np.linalg.inv(transform(p)))]for r,p in zip(rows,prior)])
    corrections[:,3]=np.unwrap(corrections[:,3])
    fig,axes=plt.subplots(3,3,figsize=(15,14))
    for ax,source in zip(axes.flat,prepared['sources']):
        background(ax)
        selected=[s for s in scans if s['source_segment']==source['segment'] and rows[0,0]<=s['observed_s']<=rows[-1,0]]
        points=[]
        for s in selected[::max(1,len(selected)//100)]:
            t=s['observed_s'];T=transform(interpolate(corrections,[t])[0])@transform(interpolate(odom,[t])[0])
            p=scan_points(s,mount,odom);points.append(p@T[:2,:2].T+T[:2,2])
        if points:
            p=np.concatenate(points);ax.scatter(p[:,0],p[:,1],s=.2,c='#d45e16',alpha=.35)
        ax.set_title(source['segment'].rsplit('-',1)[-1]+f" | {source['scans']} scans")
    fig.tight_layout();fig.savefig(output/'chunk-alignment.png',dpi=150);plt.close(fig)
    return {k:result[k] for k in ('input_scans','graph_keyframes','all_recorded_scans_retained','graph_reload_identical')} | dict(scan_to_map=metrics['all_scan_to_map'])


def review_fused_layers(slam, depth, body, semantic, output):
    """Export inspectable layers on one immutable map lattice, not a live costmap."""
    from scipy.ndimage import binary_dilation, label
    slam,depth,body,semantic,output=map(Path,(slam,depth,body,semantic,output))
    if output.exists():raise ValueError('new fused review required')
    sm=json.loads((slam/'report.json').read_text());dr=json.loads((depth/'report.json').read_text())
    br=json.loads((body/'report.json').read_text());sr=json.loads((semantic/'report.json').read_text())
    occupancy=json.loads((slam/'occupancy.json').read_text())
    if not sm.get('passed') or not sm.get('final_graph_poses'):raise ValueError('verified final map required')
    if sr['map_sha256']!=hashlib.sha256((slam/'occupancy.json').read_bytes()).hexdigest():raise ValueError('semantic map mismatch')
    for p in (slam/'occupancy.json',slam/'report.json',slam/'slam-poses.json',depth/'preview.npz',depth/'report.json'):
        if br['source_sha256'].get(str(p))!=hashlib.sha256(p.read_bytes()).hexdigest():raise ValueError('body/depth map mismatch')
    grid=np.asarray(occupancy['data']).reshape(occupancy['height'],occupancy['width'])
    resolution=occupancy['resolution'];origin=np.asarray(occupancy['origin'])
    with np.load(body/'candidate-layers.npz',allow_pickle=False) as data:
        upper=data['upper_hits'].copy();forbidden=data['body_forbidden'].copy()
    with np.load(depth/'preview.npz',allow_pickle=False) as data:voxels=data['depth_voxels'].copy()
    if upper.shape!=grid.shape or forbidden.shape!=grid.shape:raise ValueError('fusion lattice mismatch')
    # Coarse display families avoid presenting dozens of partial-view fragments
    # as independent named objects. Raw instances and exact model labels remain.
    families={'bed':'bed','sofa':'sofa','table':'table','nightstand':'table','chair':'chair',
        'refrigerator':'storage_review','cabinet':'storage_review','wardrobe':'storage_review',
        'bookshelf':'storage_review','unclassified_furniture':'storage_review'}
    masks={name:np.zeros(grid.shape,dtype=bool) for name in sorted(set(families.values()))}
    member_cells=[]
    for item in sr['landmarks']:
        family=families.get(item['category'])
        if family is None:continue
        xyz=(np.asarray(item['observed_voxels'])+.5)*item['voxel_size_m']
        xy=np.floor((xyz[:,:2]-origin)/resolution).astype(int)
        xy=xy[(xy[:,0]>=0)&(xy[:,0]<grid.shape[1])&(xy[:,1]>=0)&(xy[:,1]<grid.shape[0])]
        masks[family][xy[:,1],xy[:,0]]=True
        member_cells.append((item,family,xy))
    regions=[]
    for family,mask in masks.items():
        # Dilation groups annotations only; never changes geometry or free cells.
        expanded=binary_dilation(mask,iterations=2)
        labels,n=label(expanded,structure=np.ones((3,3),dtype=int))
        for component in range(1,n+1):
            cells=mask&(labels==component);yy,xx=np.nonzero(cells)
            if not len(xx):continue
            members=[item for item,f,xy in member_cells if f==family and len(xy) and np.any(labels[xy[:,1],xy[:,0]]==component)]
            center=origin+(np.array([np.median(xx),np.median(yy)])+.5)*resolution
            regions.append(dict(id=f'{family}-{component}',family=family,center_xy_m=center.tolist(),
                observed_xy_cells=int(len(xx)),observed_area_m2=float(len(xx)*resolution**2),
                member_landmark_ids=[m['landmark_id']for m in members],
                model_labels=sorted({m['category']for m in members}),
                maximum_member_viewpoint_groups=max(m['viewpoint_groups']for m in members),
                object_count_claimed=False,identity_confirmed=False))
    output.mkdir(parents=True)
    np.savez_compressed(output/'layers.npz',lidar_occupancy=grid,rgbd_upper_surfaces=upper,
        body_clearance_candidate=forbidden,depth_voxels=voxels,origin_xy_m=origin,resolution_m=resolution,
        **{'semantic_'+k:v for k,v in masks.items()})
    (output/'furniture-regions.json').write_text(json.dumps(regions,ensure_ascii=False,indent=2)+'\n')
    result=dict(mode='offline_lidar_rgbd_semantic_fusion',source_map_changed=False,
        source_map_sha256=sr['map_sha256'],depth_frames=dr['depth_preview_frames'],
        semantic_views=len(sr['frames']),semantic_raw_instances=len(sr['instances']),
        semantic_viewpoint_supported_instances=sum(x['viewpoint_groups']>=2 for x in sr['view_support']),
        recording_chunks=len({f['capture']['source_epoch'] for f in sr['frames']}),
        height_interval_m=br['vertical_interval_m'],rgbd_added_on_planar_free_cells=int((upper&(grid==0)).sum()),
        robot_height_m=br['geometry']['height_above_floor_m'],robot_width_depth_m=br['geometry']['dimensions_xyz_m'][:2],
        region_policy='Connected coarse display families only; dilated annotations are not obstacle geometry or object count.',
        semantics_clears_obstacles=False,full_height_free_space_certified=False,self_geometry_removed=False,
        navigation_authorized=False,deployed_to_jetson=False,
        limitations=['White is lidar-plane free space, not verified full-height passage.',
            'Depth endpoints add observed shapes; they do not certify unseen space as free.',
            'Furniture labels are model estimates; refrigerator/storage distinctions remain under review.',
            'Body clearance uses original STL at the recorded final yaw, not a rotation sweep.'],
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in
            (slam/'occupancy.json',slam/'report.json',depth/'preview.npz',depth/'report.json',body/'candidate-layers.npz',body/'report.json',semantic/'report.json')})
    (output/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    from matplotlib import font_manager
    font=next((f.name for f in font_manager.fontManager.ttflist if f.name.startswith('Noto Sans CJK')),None)
    if font:plt.rcParams['font.family']=font
    plt.rcParams['axes.unicode_minus']=False
    names={'bed':'침대','sofa':'소파','table':'탁자·책상','chair':'의자','storage_review':'수납·기타 가구*'}
    colors={'bed':'#8468c2','sofa':'#368cbe','table':'#219b7b','chair':'#c66a75','storage_review':'#bd933e'}
    extent=[origin[0],origin[0]+grid.shape[1]*resolution,origin[1],origin[1]+grid.shape[0]*resolution]
    display=np.where(grid<0,0,np.where(grid>=65,2,1))
    fig,axes=plt.subplots(1,2,figsize=(13,10))
    for ax in axes:
        ax.imshow(display,origin='lower',extent=extent,cmap=ListedColormap(['#c5cbd0','#fff','#26323a']),vmin=0,vmax=2,interpolation='nearest')
        ax.set(xlabel='X (m)',ylabel='Y (m)',aspect='equal')
    axes[0].imshow(np.ma.masked_where(~upper,upper),origin='lower',extent=extent,cmap=ListedColormap(['#e79b50']),interpolation='nearest',alpha=.9)
    axes[0].set_title('라이다 + RGB-D 형상',fontsize=15,pad=12)
    axes[1].imshow(np.ma.masked_where(~upper,upper),origin='lower',extent=extent,cmap=ListedColormap(['#e6d7bf']),interpolation='nearest',alpha=.6)
    for family,mask in masks.items():
        axes[1].imshow(np.ma.masked_where(~mask,mask),origin='lower',extent=extent,cmap=ListedColormap([colors[family]]),interpolation='nearest',alpha=.85)
    shown=[]
    for region in sorted(regions,key=lambda r:(r['maximum_member_viewpoint_groups'],r['observed_xy_cells']),reverse=True):
        if region['observed_xy_cells']<24:continue
        center=np.array(region['center_xy_m'])
        if any(np.linalg.norm(center-p)<.5 for p in shown):continue
        shown.append(center)
        axes[1].annotate(names[region['family']],center,ha='center',fontsize=8.5,
            bbox=dict(boxstyle='round,pad=.25',facecolor='white',edgecolor=colors[region['family']],alpha=.94))
        if len(shown)>=16:break
    axes[1].set_title('주요 가구 시멘틱 관측',fontsize=15,pad=12)
    fig.legend(handles=[Patch(color='#e79b50',label='RGB-D 상부 형상')]+[Patch(color=colors[k],label=names[k])for k in names],loc='lower center',ncol=3,bbox_to_anchor=(.5,.045))
    fig.suptitle(f"새 라이다 지도 기준 · RGB-D {result['depth_frames']:,}장 합성 · 주요 가구 {result['semantic_views']}개 시점 인식",fontsize=14)
    fig.text(.5,.012,'회색: 미관측  |  흰색: 라이다 기준 빈 공간  |  * 냉장고·수납장 등 이름 불일치 포함 · 실물 주행 판정 전',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.13,1,.94]);fig.savefig(output/'fused-map.png',dpi=170);plt.close(fig)
    return result


def travel_geometry(root, hold_plan, *, lift_m, mobile_mesh_dir, d415_mesh, lds_mesh, include_surfaces=False, exclude_arms=False):
    """Original STL bounds at an observed posture; not an installed-body certificate."""
    from home_robot_tasks import s2_geometry as geom, s2_development as dev
    from home_robot_tasks.d415_geometry import nominal_model
    from tools.diagnostics.s2_viewer.build_meshes import read_stl
    from scipy.spatial import ConvexHull
    root=Path(root);plan=json.loads(Path(hold_plan).read_text())
    if plan.get('executed') is not False or plan.get('plan',{}).get('ok') is not True:
        raise ValueError('successful read-only hold plan required')
    axes=plan['plan']['axes']
    if len(axes)!=13 or {a['slot'] for a in axes}!=set(range(1,14)):
        raise ValueError('all thirteen distinct posture axes required')
    positions={a['slot']:a['position'] for a in axes}
    if any(type(v) is not int or not 0<=v<4096 for v in positions.values()):
        raise ValueError('single-turn held posture required')
    geometry=geom.load_geometry(root/'config/home_assistant/official_geometry.json')
    profile=dev.released_profile(root,geometry)
    if not np.isfinite(lift_m) or not 0<=lift_m<=profile['lift']['travel_m']:
        raise ValueError('explicit lift position in model travel required')
    manifest=json.loads((root/'tools/diagnostics/s2_viewer/meshes.json').read_text())
    if manifest['geometry_sha256']!=geom.digest(geometry):raise ValueError('mesh geometry mismatch')
    mounts=json.loads((root/'config/home_assistant/arm_mounts.nominal.json').read_text())
    base=np.array(profile['base_from_cad']);platform=np.array(profile['lift']['platform_at_lift_zero'])
    displacement=np.array(profile['lift']['axis_base'])*lift_m;platform[:3,3]+=displacement
    joints={j['name']:j for j in geometry['mobile_reference_joints']}
    clouds=[];parts=[];sources={};surfaces=[]
    def add(name,path,matrix,sha,scale=1.):
        vertices,raw=read_stl(Path(path))
        if hashlib.sha256(raw).hexdigest()!=sha:raise ValueError('original mesh hash mismatch: '+name)
        matrix=geom.rigid(matrix)
        vertices=(vertices*scale)@matrix[:3,:3].T+matrix[:3,3]
        vertices[:,2]-=profile['ground_z_m']
        clouds.append(vertices);surfaces.append(vertices.reshape(-1,3,3))
        parts.append(dict(name=name,minimum_m=vertices.min(0).tolist(),maximum_m=vertices.max(0).tolist()))
        sources[str(path)]=sha
    mobile_mesh_dir=Path(mobile_mesh_dir)
    for name,matrix in [('base_link.STL',base),('vertical_link.STL',platform)]:
        add(name,mobile_mesh_dir/name,matrix,manifest['meshes'][name]['source_sha256'])
    for i in range(1,4):
        add('wheel'+str(i),mobile_mesh_dir/'wheel1.STL',base@geom.origin(joints['wheel'+str(i)]),manifest['meshes']['wheel1.STL']['source_sha256'])
    raw_arms={}
    for side,offset in [('left',1),('right',7)]:
        raw_arms[side]={joint:positions[offset+i] for i,joint in enumerate(geom.JOINTS)}
        root_from_arm=np.array(profile['arms'][side]['base_from_arm_at_lift_zero'])
        root_from_arm[:3,3]+=displacement
        name=side+'_base.STL'
        add(side+' mounting base',mobile_mesh_dir/name,platform@geom.origin(joints[side+'_base']),mounts['source_mesh_sha256'][name])
        # Explicit offline scenario: operator intends to stow the arms. Fixed
        # mounting structure stays; no claim is made about a measured stow pose.
        if exclude_arms:continue
        frames=geom.arm_transforms(geometry,geom.model_angles(raw_arms[side],profile['arms'][side]['alignment']))
        for visual in manifest['arm_visuals']:
            name=visual['mesh'];matrix=root_from_arm@frames[visual['link']]@np.array(visual['origin'])
            add(side+'/'+visual['link']+'/'+name,root/'ros2_ws/src/so101_description/meshes'/name,matrix,manifest['meshes'][name]['source_sha256'])
    camera=nominal_model(root/'config/home_assistant/d415_official_geometry.json')
    add('D415',d415_mesh,np.array(profile['base_from_optical'])@np.array(camera['color_optical_from_mesh']),camera['mesh_sha256'])
    lidar=json.loads((root/'config/home_assistant/lds01_official_geometry.json').read_text())
    transforms=load_sensor_frames(root)['transforms']
    lidar_tf=next(t['matrix'] for t in transforms if t['child']=='lidar_a')
    add('LDS-01',lds_mesh,np.array(lidar_tf),lidar['mesh_sha256'],lidar['mesh_scale_m'])
    points=np.vstack(clouds);hull=ConvexHull(points[:,:2])
    low,high=points.min(0),points.max(0)
    result=dict(mode='original_STL_observed_posture_review',geometry_id='mini1-'+geom.digest(dict(profile=profile,raw=raw_arms,lift=lift_m))[:24],
        calibration_id=profile['calibration_id'],arm_raw=raw_arms,lift_position_m=lift_m,lift_raw=positions[13],
        lift_basis='explicit operator-referenced height; not inferred from raw position after power cycles',
        minimum_m=low.tolist(),maximum_m=high.tolist(),dimensions_xyz_m=(high-low).tolist(),
        height_above_floor_m=float(high[2]),circumscribed_radius_m=float(np.linalg.norm(points[:,:2],axis=1).max()),
        footprint_xy_m=points[hull.vertices,:2].tolist(),parts=parts,source_sha256=sources,
        hold_plan_sha256=hashlib.sha256(Path(hold_plan).read_bytes()).hexdigest(),
        source_profile_sha256=geom.digest(profile),original_meshes_used=True,
        physical_body_verified=False,runtime_authorized=False,
        unmodelled=['external power cable','any support or accessory absent from official CAD','payload (empty grippers last confirmed)'])
    result['arm_geometry_included']=not exclude_arms
    if exclude_arms:
        result.update(mode='original_STL_body_only_assumed_stowed_arms_review',
            geometry_id='mini1-'+geom.digest(dict(profile=profile,lift=lift_m,exclude_arms=True))[:24],
            arm_policy='operator-requested exclusion of articulated arms; fixed mounts retained',
            folded_arm_envelope_verified=False)
        result['unmodelled'].append('folded arm envelope intentionally excluded by operator request')
    # Display sample only: all dimensions above were calculated from full surfaces.
    sample=points[::max(1,len(points)//18000)]
    return (result,sample,surfaces) if include_surfaces else (result,sample)


def endpoint_obstacles(voxels, voxel_size, grid_shape, origin, resolution, *, bottom, top):
    """Endpoint-only review. No free cells, self filtering or live authority."""
    if not np.isfinite([bottom,top,voxel_size,resolution]).all() or not 0<=bottom<top or voxel_size<=0 or resolution<=0:raise ValueError('invalid height or resolution')
    cells=np.asarray(voxels)
    if cells.ndim!=2 or cells.shape[1]!=3 or not np.issubdtype(cells.dtype,np.integer):
        raise ValueError('integer endpoint voxels required')
    selected=cells[(cells[:,2]*voxel_size<top)&((cells[:,2]+1)*voxel_size>bottom)]
    if len(grid_shape)!=2 or min(grid_shape)<1 or np.prod(grid_shape)>250000 or np.shape(origin)!=(2,) or not np.isfinite(origin).all():
        raise ValueError('bounded grid with finite origin required')
    blocked=np.zeros(grid_shape,dtype=bool)
    # Rasterize the complete voxel XY extent; the SLAM grid need not share its origin.
    for x,y,z in selected:
        lo=np.floor(np.round((np.array([x,y])*voxel_size-origin)/resolution,12)).astype(int)
        hi=np.ceil(np.round(((np.array([x,y])+1)*voxel_size-origin)/resolution,12)).astype(int)
        x0,y0=np.maximum(lo,0);x1,y1=np.minimum(hi,[grid_shape[1],grid_shape[0]])
        if x0<x1 and y0<y1:blocked[y0:y1,x0:x1]=True
    return blocked,len(selected)


def slab_footprints(surfaces, bottom, top):
    """Per-part convex XY bounds of triangles clipped to a height interval.

    Intersect triangle edges with both planes; merely selecting nearby vertices
    misses a tall face crossing the band. Separate parts do not fill each other's gaps.
    """
    from scipy.spatial import ConvexHull
    if not np.isfinite([bottom,top]).all() or bottom>=top:raise ValueError('invalid slab')
    polygons=[]
    for triangles in surfaces:
        triangles=np.asarray(triangles,float)
        if triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not np.isfinite(triangles).all():
            raise ValueError('finite original triangles required')
        if not len(triangles):continue
        tri=triangles[(triangles[:,:,2].min(1)<=top)&(triangles[:,:,2].max(1)>=bottom)]
        if not len(tri):continue
        pts=tri.reshape(-1,3);inside=pts[(pts[:,2]>=bottom)&(pts[:,2]<=top)]
        pieces=[inside[:,:2]]
        for a,b in ((0,1),(1,2),(2,0)):
            start,end=tri[:,a],tri[:,b];dz=end[:,2]-start[:,2]
            for z in (bottom,top):
                valid=(abs(dz)>1e-12)&(np.minimum(start[:,2],end[:,2])<=z)&(np.maximum(start[:,2],end[:,2])>=z)
                t=(z-start[valid,2])/dz[valid]
                pieces.append((start[valid]+t[:,None]*(end[valid]-start[valid]))[:,:2])
        xy=np.unique(np.vstack(pieces),axis=0)
        if not len(xy):continue
        if len(xy)>=3 and np.linalg.matrix_rank(xy-xy[0],tol=1e-10)==2:xy=xy[ConvexHull(xy).vertices]
        polygons.append(xy)
    return polygons


def footprint_kernel(polygons, resolution, yaw, clearance, *, obstacle_size_m=None, lattice_offset_xy_m=(0.,0.)):
    """Obstacle cell -> robot centre offsets for one yaw (reflected footprint).

    Expand by the obstacle cell square and metric clearance, without rounding
    clearance up to a whole number of cells or assuming all possible yaw angles.
    """
    from scipy.spatial import ConvexHull
    if not np.isfinite([resolution,yaw,clearance]).all() or resolution<=0 or clearance<0:raise ValueError('invalid footprint parameters')
    rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    obstacle_size_m=resolution if obstacle_size_m is None else obstacle_size_m
    offset=np.asarray(lattice_offset_xy_m,float)
    if offset.shape!=(2,) or not np.isfinite(offset).all() or not np.isfinite(obstacle_size_m) or obstacle_size_m<=0:
        raise ValueError('invalid obstacle size or lattice offset')
    square=obstacle_size_m/2*np.array([[-1,-1],[-1,1],[1,1],[1,-1]])
    expanded=[]
    for poly in polygons:
        poly=np.asarray(poly,float)
        if poly.ndim!=2 or poly.shape[1]!=2 or not len(poly) or not np.isfinite(poly).all():raise ValueError('finite XY polygon required')
        points=(-poly@rotation.T)[:,None,:]+square[None,:,:]
        points=points.reshape(-1,2);hull=ConvexHull(points)
        expanded.append(points[hull.vertices])
    if not expanded:return np.zeros((1,1),bool)
    extent=max(abs(np.vstack(expanded)).max()+clearance+abs(offset).max(),resolution)
    steps=int(np.ceil(extent/resolution))
    if steps>200:raise ValueError('bounded footprint kernel required')
    yy,xx=np.mgrid[-steps:steps+1,-steps:steps+1];points=np.column_stack((xx.ravel(),yy.ravel()))*resolution+offset
    selected=np.zeros(len(points),bool)
    for poly in expanded:
        hull=ConvexHull(poly)
        inside=np.all(points@hull.equations[:,:2].T+hull.equations[:,2]<=1e-10,axis=1)
        distance=np.full(len(points),np.inf)
        for a,b in zip(poly,np.roll(poly,-1,axis=0)):
            edge=b-a;t=np.clip((points-a)@edge/(edge@edge),0,1)
            distance=np.minimum(distance,np.linalg.norm(points-a-t[:,None]*edge,axis=1))
        selected|=inside|(distance<=clearance+1e-10)
    return selected.reshape(xx.shape)


def voxel_body_centres(voxels_xy, polygons, *, voxel_size, grid_shape, origin,
                       resolution, yaw, clearance):
    """Project voxel/body overlap directly onto robot-centre samples.

    Rasterizing a voxel to intersecting map cells and then expanding those
    cells pads the same evidence twice when the two grid origins differ.
    Instead, retain its original square and account for the lattice phase.
    """
    cells=np.asarray(voxels_xy)
    if cells.ndim!=2 or cells.shape[1]!=2 or not np.issubdtype(cells.dtype,np.integer):
        raise ValueError('integer XY voxel cells required')
    if not np.isclose(voxel_size,resolution,rtol=0,atol=1e-8):
        raise ValueError('matching voxel and map resolution required')
    result=np.zeros(grid_shape,bool)
    if not len(cells) or not polygons:return result
    centers=(cells+.5)*voxel_size
    anchors=np.floor((centers-np.asarray(origin))/resolution).astype(int)
    offsets=np.asarray(origin)+(anchors+.5)*resolution-centers
    if np.max(abs(offsets-offsets[0]))>1e-6:
        raise ValueError('nonuniform voxel lattice phase')
    kernel=footprint_kernel(polygons,resolution,yaw,clearance,
        obstacle_size_m=voxel_size,lattice_offset_xy_m=offsets[0])
    yy,xx=np.nonzero(kernel);cy,cx=np.array(kernel.shape)//2
    for dx,dy in zip(xx-cx,yy-cy):
        target=anchors+[dx,dy]
        valid=(target[:,0]>=0)&(target[:,0]<grid_shape[1])&(target[:,1]>=0)&(target[:,1]<grid_shape[0])
        target=target[valid];result[target[:,1],target[:,0]]=True
    return result


def height_matched_candidates(occupied, voxels, surfaces, *, resolution, origin,
                              voxel_size, lidar_top, height, yaw, clearance, navigation_footprint=None):
    from scipy.ndimage import binary_dilation
    base_polys=([np.asarray(navigation_footprint,float)] if navigation_footprint is not None
                else slab_footprints(surfaces,0,lidar_top))
    base_kernel=footprint_kernel(base_polys,resolution,yaw,clearance)
    result=binary_dilation(occupied,structure=base_kernel)
    bands=[]
    for z in np.unique(voxels[:,2]):
        low,high=float(z*voxel_size),float((z+1)*voxel_size)
        if high<=lidar_top or low>=height+clearance:continue
        polygons=slab_footprints(surfaces,max(0,low-clearance),min(height,high+clearance))
        if not polygons:continue
        result|=voxel_body_centres(voxels[voxels[:,2]==z,:2],polygons,voxel_size=voxel_size,
            grid_shape=occupied.shape,origin=origin,resolution=resolution,yaw=yaw,clearance=clearance)
        bands.append(dict(height_interval_m=[low,high],body_polygons=len(polygons),
                          rasterization='direct_original_voxel_to_body_centre'))
    return result,base_polys,bands


def movement_area_review(output, depth_review, grid, origin, resolution, geometry, surfaces, poses,
                         *, lidar_top, clearance, context_review=None, navigation_footprint=None):
    """Orientation-aware robot-centre candidates from native ray evidence.

    No route is carved from teleop history. It is evaluated afterwards as an
    independent consistency check, and unresolved depth remains separate.
    """
    from scipy.ndimage import binary_dilation
    report=json.loads((depth_review/'report.json').read_text())
    if report.get('mode')!='offline_recorded_ray_evidence_review':
        raise ValueError('movement area requires ray evidence, not endpoint union')
    with np.load(depth_review/'ray-evidence.npz',allow_pickle=False) as a:
        confirmed=a['voxels'][a['occupied']].copy()
        uncertain=a['voxels'][a['uncertain']&(a['hit_frames']>0)].copy()
    base_polys=([np.asarray(navigation_footprint,float)] if navigation_footprint is not None
                else slab_footprints(surfaces,0,lidar_top))
    height=geometry['height_above_floor_m']
    bands=[]
    for z in np.unique(np.r_[confirmed[:,2],uncertain[:,2]]):
        low,high=z*.05,(z+1)*.05
        if low>=height+clearance:continue
        polygons=slab_footprints(surfaces,max(0,low-clearance),min(height,high+clearance))
        if polygons:bands.append((polygons,confirmed[confirmed[:,2]==z,:2],uncertain[uncertain[:,2]==z,:2]))
    feasible=[];review_free=[];lidar_feasible=[]
    for degrees in range(0,360,5):
        yaw=np.deg2rad(degrees)
        kernel=footprint_kernel(base_polys,resolution,yaw,clearance)
        base_blocked=binary_dilation(grid>=65,structure=kernel)
        unknown=binary_dilation(grid<0,structure=kernel,border_value=1)
        blocked=base_blocked.copy();disputed=np.zeros(grid.shape,bool)
        for polygons,hits,maybe in bands:
            blocked|=voxel_body_centres(hits,polygons,voxel_size=.05,grid_shape=grid.shape,
                origin=origin,resolution=resolution,yaw=yaw,clearance=clearance)
            disputed|=voxel_body_centres(maybe,polygons,voxel_size=.05,grid_shape=grid.shape,
                origin=origin,resolution=resolution,yaw=yaw,clearance=clearance)
        valid=(grid==0)&~blocked&~unknown
        lidar_feasible.append((grid==0)&~base_blocked&~unknown)
        feasible.append(valid);review_free.append(valid&~disputed)
    feasible=np.array(feasible);review_free=np.array(review_free);lidar_feasible=np.array(lidar_feasible)
    any_valid=feasible.any(0);candidate=review_free.any(0)
    needs_review=any_valid&~candidate
    xy=np.floor((poses[:,1:3]-origin)/resolution).astype(int)
    bins=np.rint(np.rad2deg(poses[:,3])/5).astype(int)%72
    path_pass=feasible[bins,xy[:,1],xy[:,0]]
    path_clear=review_free[bins,xy[:,1],xy[:,0]]
    ray_map,_=endpoint_obstacles(confirmed,.05,grid.shape,origin,resolution,bottom=.10,top=height+clearance)
    stats=dict(mode='orientation_aware_lidar_rgbd_movement_candidates',
        source_map_sha256=report['map_sha256'],ray_evidence_sha256=hashlib.sha256((depth_review/'ray-evidence.npz').read_bytes()).hexdigest(),
        selected_depth_frames=report['selected_frames'],source_epochs=len(report['source_epochs']),
        lidar_footprint_policy='complete_fixed_projection' if navigation_footprint is not None else 'lidar_height_slice',
        navigation_footprint_xy_m=None if navigation_footprint is None else np.asarray(navigation_footprint).tolist(),
        yaw_step_deg=5,clearance_m=clearance,articulated_arms_excluded=not geometry['arm_geometry_included'],
        robot_height_m=height,fixed_body_dimensions_xyz_m=geometry['dimensions_xyz_m'],
        rasterization='direct original voxel squares; no intermediate map-cell enlargement',
        candidate_center_cells=int(candidate.sum()),uncertain_center_cells=int(needs_review.sum()),
        candidate_center_area_m2=float(candidate.sum()*resolution**2),
        free_some_orientation_cells=int(any_valid.sum()),free_all_sampled_orientations_cells=int(feasible.all(0).sum()),
        recorded_pose_count=len(poses),recorded_pose_lidar_body_conflicts=int((~lidar_feasible[bins,xy[:,1],xy[:,0]]).sum()),
        recorded_pose_rgbd_body_conflicts=int((~path_pass).sum()),recorded_pose_depth_uncertain=int((path_pass&~path_clear).sum()),
        trajectory_used_to_clear_obstacles=False,source_geometry_modified=False,
        connected_route_verified=False,continuous_rotation_sweep_verified=False,full_height_free_space_certified=False,
        navigation_authorized=False,robot_commands=0,
        limitations=['Green is a candidate at one or more sampled headings, not a certified connected route.',
                    'Yellow has unresolved hit/clear depth evidence; it is not declared free.',
                    'Historical sampled ray evidence retains calibration, registration and sampling uncertainty.'])
    np.savez_compressed(output/'movement-area.npz',feasible_by_yaw=feasible,review_free_by_yaw=review_free,
        candidate_centers=candidate,uncertain_centers=needs_review,rgbd_obstacles=ray_map,
        origin_xy_m=origin,resolution_m=resolution,poses=poses,path_pass=path_pass,path_clear=path_clear,
        source_map_sha256=report['map_sha256'],
        **({} if navigation_footprint is None else dict(navigation_footprint_xy_m=np.asarray(navigation_footprint),clearance_m=clearance)))
    (output/'movement-area-report.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    from matplotlib import font_manager
    from matplotlib.lines import Line2D
    font=next((f.name for f in font_manager.fontManager.ttflist if f.name.startswith('Noto Sans CJK')),None)
    if font:plt.rcParams['font.family']=font
    plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(1,2,figsize=(13,10))
    extent=[origin[0],origin[0]+grid.shape[1]*resolution,origin[1],origin[1]+grid.shape[0]*resolution]
    display=np.where(grid<0,0,np.where(grid>=65,2,1))
    axes[0].imshow(display,origin='lower',extent=extent,cmap=ListedColormap(['#dce2e7','#fff','#34434d']),vmin=0,vmax=2,interpolation='nearest')
    axes[0].imshow(np.ma.masked_where(~ray_map,ray_map),origin='lower',extent=extent,cmap=ListedColormap(['#d88938']),alpha=.9,interpolation='nearest')
    view=np.zeros(grid.shape,np.uint8);view[grid==0]=2;view[needs_review]=3;view[candidate]=4;view[grid>=65]=1
    axes[1].imshow(view,origin='lower',extent=extent,cmap=ListedColormap(['#dce2e7','#34434d','#e8b8af','#efd17e','#53ad8b']),vmin=0,vmax=4,interpolation='nearest')
    # Highlight contradictions, never erase an obstacle merely because a
    # historical robot centre passed near it.
    conflicting_xy=np.unique(xy[~path_pass],axis=0)
    conflicting_centres=origin+(conflicting_xy+.5)*resolution
    axes[1].scatter(conflicting_centres[:,0],conflicting_centres[:,1],s=12,
                    facecolors='none',edgecolors='#235ab4',linewidths=.8,alpha=.85)
    for ax in axes:ax.set(xlabel='X (m)',ylabel='Y (m)',aspect='equal')
    axes[0].set_title('① 2D 지도 + RGB-D 장애물',fontsize=14)
    axes[1].set_title('② 베이스·고정 구조 크기 반영',fontsize=14)
    if context_review:
        names=json.loads((context_review/'household-labels.json').read_text())
        if names['source_map_sha256']!=report['map_sha256']:raise ValueError('names belong to a different map')
        wanted={'home-coffee-table-01','home-bed-01','home-sofa-01','home-fridge-01','home-work-desk-01','home-vanity-01'}
        for item in names['entities']:
            if item['id'] not in wanted:continue
            for ax in axes:ax.text(*item['observed_anchor_xyz_m'][:2],item['label'],fontsize=8,ha='center',
                bbox=dict(boxstyle='round,pad=.2',fc='white',ec='#7b8992',alpha=.92))
    fig.suptitle('RGB-D 상부 장애물과 차폭을 반영한 이동 후보',fontsize=18)
    fig.legend(handles=[Patch(color='#d88938',label='RGB-D 장애물'),Patch(color='#53ad8b',label='중심 위치 후보'),
        Patch(color='#efd17e',label='깊이 관측 확인 필요'),Patch(color='#e8b8af',label='물체·차폭 제한'),
        Patch(color='#34434d',label='라이다 장애물'),Patch(color='#dce2e7',label='미관측'),
        Line2D([],[],marker='o',mfc='none',mec='#235ab4',linestyle='none',label='주행 기록과 불일치 · 재확인')],loc='lower center',ncol=3,bbox_to_anchor=(.5,.055))
    fig.text(.5,.018,'팔 제외 · 리프트 최하단 · 여유 3cm · 5° 간격 방향 검토\n초록: 가능한 방향이 있는 위치 후보 / 회전·연결 경로는 별도 검증',ha='center',fontsize=10)
    fig.tight_layout(rect=[0,.14,1,.94]);fig.savefig(output/'movement-area.png',dpi=150);plt.close(fig)
    return stats


def review_traversability(args):
    from scipy.ndimage import binary_dilation
    import math
    geometry,body,surfaces=travel_geometry(ROOT,args.hold_plan,lift_m=args.lift_m,
        mobile_mesh_dir=args.mobile_mesh_dir,d415_mesh=args.d415_mesh,lds_mesh=args.lds_mesh,
        include_surfaces=True,exclude_arms=getattr(args,'exclude_arms',False))
    navigation_footprint=geometry['footprint_xy_m'] if getattr(args,'fixed_body_projection',False) else None
    output=args.output
    if output.exists():raise ValueError('new review output required')
    occupancy=json.loads((args.slam_review/'occupancy.json').read_text())
    grid=np.asarray(occupancy['data']).reshape(occupancy['height'],occupancy['width'])
    resolution=occupancy['resolution'];origin=np.array(occupancy['origin'])
    with np.load(args.depth_review/'preview.npz',allow_pickle=False) as d:voxels=d['depth_voxels'];poses=d['slam_poses']
    slam_report=json.loads((args.slam_review/'report.json').read_text())
    if slam_report.get('passed') is not True:raise ValueError('passed saved SLAM required')
    saved_poses=json.loads((args.slam_review/'slam-poses.json').read_text())
    expected=np.array([[p['stamp']-slam_report['replay_clock_offset_s'],*p['xy_yaw']] for p in saved_poses])
    expected[:,3]=np.unwrap(expected[:,3])
    if poses.shape!=expected.shape or not np.allclose(poses,expected,atol=1e-8,rtol=0):
        raise ValueError('depth preview belongs to a different SLAM trajectory')
    depth_report=json.loads((args.depth_review/'report.json').read_text())
    if depth_report.get('mode') not in ('offline_recorded_map_quality_preview','offline_recorded_ray_evidence_review'):
        raise ValueError('recorded depth review required')
    if depth_report.get('mode')=='offline_recorded_ray_evidence_review' and depth_report['map_sha256']!=hashlib.sha256((args.slam_review/'occupancy.json').read_bytes()).hexdigest():
        raise ValueError('ray evidence belongs to a different map')
    # This comparison intentionally uses only heights above the lidar plane.
    # Floor/self-body points remain unresolved; do not turn endpoint absence into free space.
    lidar=json.loads((ROOT/'config/home_assistant/lidar_mount.review.json').read_text())
    bottom=lidar['scan_height_above_floor_m']+.05
    top=geometry['height_above_floor_m']+args.clearance_m
    upper,n=endpoint_obstacles(voxels,.05,grid.shape,origin,resolution,bottom=bottom,top=top)
    occupied=grid>=65;combined=occupied|upper
    radius=geometry['circumscribed_radius_m']+args.clearance_m
    steps=math.ceil(radius/resolution);y,x=np.mgrid[-steps:steps+1,-steps:steps+1]
    kernel=x*x+y*y<=steps*steps
    legacy=binary_dilation(combined,structure=kernel)
    yaw=float(poses[-1,3])
    forbidden,base_polys,bands=height_matched_candidates(occupied,voxels,surfaces,
        resolution=resolution,origin=origin,voxel_size=.05,lidar_top=bottom,
        height=geometry['height_above_floor_m'],yaw=yaw,clearance=args.clearance_m,navigation_footprint=navigation_footprint)
    extra=(grid==0)&upper
    report=dict(mode='recorded_body_clearance_candidate_review',geometry=geometry,
        candidate_clearance_m=args.clearance_m,vertical_interval_m=[bottom,top],
        upper_endpoint_voxels=n,upper_endpoint_xy_cells=int(upper.sum()),
        lidar_free_cells_with_upper_hits=int(extra.sum()),
        body_dilated_obstacle_cells=int(forbidden.sum()),
        comparison_policy='height-matched per-part STL slabs, fixed recorded final yaw',
        yaw_rad=yaw,base_footprint_xy_m=[p.tolist() for p in base_polys],height_bands=bands,
        previous_circle_cells=int(legacy.sum()),
        previous_circle_on_lidar_free_cells=int((legacy&(grid==0)).sum()),
        height_matched_on_lidar_free_cells=int((forbidden&(grid==0)).sum()),
        rotation_sweep_verified=False,
        free_space_certified=False,self_geometry_removed=False,depth_motion_qualified=False,
        runtime_authorized=False,motor_commands=0,live_map_modified=False,
        current_posture_applied_to_recorded_environment=not getattr(args,'exclude_arms',False),
        articulated_arms_excluded=getattr(args,'exclude_arms',False),
        limitations=['Recorded endpoint preview, not a full-height ray-cleared traversability map.',
            'Current observed joint posture does not certify the pose at every historical depth exposure.',
            'Self returns and unsupported accessories remain unresolved; no obstacle clearing performed.',
            'Height-specific part hulls at one fixed yaw; turns require a swept-orientation check.',
            'LiDAR-only geometry is tested with the lower body; unknown upper geometry is not certified clear.',
            'Clearance is an explicit review setting, not measured braking or installation uncertainty.'],
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [args.slam_review/'occupancy.json',args.slam_review/'report.json',args.slam_review/'slam-poses.json',args.depth_review/'preview.npz',args.depth_review/'report.json']})
    if getattr(args,'exclude_arms',False):
        report['limitations'][1]='Articulated arm geometry excluded by operator request; physical folded envelope not verified.'
    output.mkdir(parents=True);(output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savez_compressed(output/'candidate-layers.npz',lidar_occupied=occupied,upper_hits=upper,body_forbidden=forbidden,previous_circle=legacy)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Polygon, Patch
    from matplotlib import font_manager
    families=[f.name for f in font_manager.fontManager.ttflist if f.name.startswith('Noto Sans CJK')]
    korean=bool(families)
    if korean:
        plt.rcParams['font.family']=families[0]
        plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(1,3,figsize=(16,6))
    extent=[origin[0],origin[0]+grid.shape[1]*resolution,origin[1],origin[1]+grid.shape[0]*resolution]
    display=np.where(grid<0,0,np.where(occupied,2,1))
    for ax in axes:
        ax.imshow(display,origin='lower',extent=extent,cmap=ListedColormap(['#aeb8c2','#f7f5ee','#23323d']),vmin=0,vmax=2)
        ax.set(xlabel='X (m)',ylabel='Y (m)',aspect='equal')
    axes[0].imshow(np.ma.masked_where(~upper,upper),origin='lower',extent=extent,cmap=ListedColormap(['#df802f']),alpha=.9)
    axes[0].set_title('관측된 물체 형상 · 여유 폭 확장 전' if korean else 'Observed surfaces / no padding')
    for ax,mask,title in [(axes[1],legacy,'이전: 모든 높이에 최대 원 반경'),(axes[2],forbidden,'수정: 높이별 외형 · 표시된 방향 기준')]:
        # Shade only lidar-known free floor, not unseen grey regions or objects.
        visible=mask&(grid==0)&~upper
        ax.imshow(np.ma.masked_where(~visible,visible),origin='lower',extent=extent,cmap=ListedColormap(['#cf6666']),alpha=.5)
        ax.imshow(np.ma.masked_where(~upper,upper),origin='lower',extent=extent,cmap=ListedColormap(['#df802f']),alpha=.9)
        ax.set_title(title if korean else ('Previous circle' if ax is axes[1] else 'Height-matched body / fixed yaw'))
    rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    for poly in base_polys:axes[2].add_patch(Polygon(poly@rot.T+poses[-1,1:3],fill=False,color='#16837a',linewidth=1.2))
    axes[2].arrow(poses[-1,1],poses[-1,2],.27*np.cos(yaw),.27*np.sin(yaw),color='#16837a',width=.015,head_width=.10)
    legends=[Patch(color='#23323d',label='라이다 물체' if korean else 'Lidar surface'),
        Patch(color='#df802f',label='상부 깊이 형상' if korean else 'Upper surface'),
        Patch(color='#e4aaaa',label='로봇 중심의 여유 공간 · 실제 물체 아님' if korean else 'Centre clearance / not a solid object'),
        Patch(color='#aeb8c2',label='미관측' if korean else 'Unobserved')]
    fig.legend(handles=legends,loc='lower center',ncol=4,fontsize=10)
    fig.suptitle('물체 영역과 로봇 중심의 여유 영역을 구분한 비교 · 실물 주행 판정 전' if korean else 'Surfaces and centre clearance shown separately / offline review',fontsize=13)
    fig.tight_layout(rect=[0,.13,1,.94]);fig.savefig(output/'traversability-review.png',dpi=160);plt.close(fig)
    if getattr(args,'movement_area',False):
        report['movement_area']=movement_area_review(output,args.depth_review,grid,origin,resolution,geometry,surfaces,poses,
            lidar_top=bottom,clearance=args.clearance_m,context_review=args.context_review,navigation_footprint=navigation_footprint)
        (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


def review_living_map(slam_review, body_review, output):
    """Synthetic change injection on the recorded lattice, never live evidence."""
    import time
    from home_robot_tasks.adaptive_mapping import LivingMap
    from home_robot_tasks.assistant.world import WorldStore
    slam_review, body_review, output = map(Path, (slam_review, body_review, output))
    source = slam_review/'occupancy.json'
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    raw = json.loads(source.read_text())
    body = json.loads((body_review/'movement-area-report.json').read_text())
    if body['source_map_sha256'] != digest:
        raise ValueError('movement review and lidar map mismatch')
    with np.load(body_review/'movement-area.npz', allow_pickle=False) as layers:
        upper, candidates = layers['rgbd_obstacles'].copy(), layers['candidate_centers'].copy()
        if (not np.allclose(layers['origin_xy_m'], raw['origin'], atol=1e-10, rtol=0) or
                abs(float(layers['resolution_m'])-raw['resolution']) > 1e-10):
            raise ValueError('recorded lattice mismatch')
    cells = np.asarray(raw['data']).reshape(raw['height'], raw['width'])
    if upper.shape != cells.shape or candidates.shape != cells.shape:
        raise ValueError('movement grid shape mismatch')
    rows = np.where((cells == 100) | upper, '#', np.where(cells == 0, '.', '?'))
    free = np.argwhere(candidates & (rows == '.'))
    if len(free) < 30:
        raise ValueError('insufficient fixture space')
    # These are synthetic single-cell changes, not furniture identified in the
    # recording. The accepted map and full yaw/body candidate masks stay intact.
    old, new, person = [tuple(map(int, free[i][::-1])) for i in (len(free)//4, len(free)//2, 3*len(free)//4)]
    fixture = rows.copy(); fixture[old[1], old[0]] = '#'
    doc = dict(schema_version=1, map_id='recorded-living-review', revision='fixture-1',
        floor_id='ground', frame_id='map', floor_z_m=0., resolution_m=raw['resolution'],
        origin=dict(zip(('x','y'), raw['origin'])), rows=[''.join(r) for r in fixture],
        robot_radius_m=.2252786, clearance_m=.03, surface_tolerance_m=.03)
    output.mkdir(parents=True, exist_ok=False)
    store = WorldStore(output/'living-map.sqlite')
    config = dict(calibration_id='recorded-review-calibration', clock_id='synthetic-change-clock',
                  profile_id='folded-body-review', height_m=body['robot_height_m'], store=store)
    mapping = LivingMap(doc, **config)
    mapping.register_furniture('fixture-table', [old], registration_id='initial-fixture',
                               source='accepted_initial_map', now_s=0.)
    events, elapsed = [], []
    def step(t, *, old_clear=False, person_free=False, unseen=False):
        laser = np.full(rows.shape, '?'); depth = laser.copy()
        if not unseen:
            for x,y in (new, person): laser[y,x] = depth[y,x] = '#'
            laser[old[1],old[0]] = '.'
            if old_clear: depth[old[1],old[0]] = '.'
            if person_free: laser[person[1],person[0]] = depth[person[1],person[0]] = '.'
        started = time.perf_counter()
        result = mapping.observe(lidar_rows=[''.join(r) for r in laser], depth_rows=[''.join(r) for r in depth],
            observed_s=t, now_s=t, capture_id=f'synthetic-{t}', frame_id='map',
            calibration_id=config['calibration_id'], clock_id=config['clock_id'],
            localization_qualified=True, full_height_qualified=True,
            furniture_cells=[] if unseen else [new],
            identity_matches=[] if unseen else [dict(furniture_id='fixture-table', cells=[new],
                capture_id=f'synthetic-{t}', observed_s=t, identity_verified=True, method='continuous_track')],
            dynamic_cells=[] if unseen or person_free else [person])
        elapsed.append(time.perf_counter()-started)
        events.append(dict(time_s=t, saved=result['saved'],
            added=result['persistent']['added_cells'], cleared=result['persistent']['cleared_cells'],
            live_person=mapping.live.nav.rows[person[1]][person[0]],
            stored_person=mapping.stable.rows[person[1]][person[0]]))
    step(1.)
    assert mapping.live.nav.rows[new[1]][new[0]] == '#'
    assert mapping.stable.rows[old[1]][old[0]] == '#'
    step(2., old_clear=True); step(3., old_clear=True)
    assert mapping.stable.rows[old[1]][old[0]] == '.'
    assert mapping.stable.rows[new[1]][new[0]] == '#'
    step(4., unseen=True)
    assert mapping.live.nav.rows[person[1]][person[0]] == '#'
    step(5., old_clear=True, person_free=True); step(5.3, old_clear=True, person_free=True)
    assert mapping.live.nav.rows[person[1]][person[0]] == '.'
    saved = mapping.stable._document
    store.close()
    store = WorldStore(output/'living-map.sqlite')
    restored = LivingMap(doc, **{**config, 'store': store, 'clock_id':'synthetic-reboot-clock'})
    assert restored.stable._document == saved and not restored.fresh(0.)
    store.close()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == digest
    result = dict(mode='synthetic_changes_on_recorded_map_lattice', passed=True,
        source_map_sha256=digest, movement_review_sha256=hashlib.sha256(
            (body_review/'movement-area-report.json').read_bytes()).hexdigest(),
        persistence_policy='registered_furniture_only_v1',
        map_dimensions=[raw['width'],raw['height']], injected_cells=dict(old=old,new=new,person=person),
        events=events, update_wall_ms=dict(maximum=1000*max(elapsed), median=1000*float(np.median(elapsed))),
        stable_restart_identical=True, restored_live_evidence_fresh=False,
        source_map_modified=False, hardware_commands=0, physical_validation=False,
        connected_route_verified=False, navigation_authorized=False,
        limitations=['synthetic observations, not observed furniture movement',
                     'PC processing time is not Jetson latency',
                     'recorded map still has unresolved body/depth boundary conflicts',
                     'no connected yaw-aware route or physical navigation tested'])
    (output/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--slam-review',type=Path);p.add_argument('--trajectory-review',type=Path)
    p.add_argument('--lidar-only',action='store_true',help='Review a prepared lidar-only campaign without reading RGB-D')
    p.add_argument('--semantic',action='store_true',help='Offline DINO/SAM furniture annotations; never modifies live occupancy')
    p.add_argument('--semantic-floor-filter',action='store_true',help='Ablation: keep DINO names and filter semantic floor geometry')
    p.add_argument('--household-review',action='store_true',help='Local household crop review plus floor filtering; original geometry unchanged')
    p.add_argument('--semantic-frames',type=int,default=6)
    p.add_argument('--model-cache',type=Path,help='Existing pinned DINO/SAM cache; semantic review uses offline model loading')
    p.add_argument('--traversability',action='store_true')
    p.add_argument('--living-map-review',action='store_true',help='Inject synthetic local changes on the recorded map lattice; no hardware')
    p.add_argument('--movement-area',action='store_true',help='Compare 72 headings using ray-qualified depth and render robot-centre candidates')
    p.add_argument('--context-review',type=Path,help='Optional household names on the same map')
    p.add_argument('--ray-depth-review',action='store_true',help='Replay sampled native depth rays offline; preserve hit, clear and uncertain evidence separately')
    p.add_argument('--exclude-arms',action='store_true',help='Offline body-only scenario: assume arms will be stowed; retain fixed mounts, lift and sensors')
    p.add_argument('--fused-review',action='store_true',help='Compose verified lidar, depth, body and semantic review layers')
    p.add_argument('--semantic-review',type=Path)
    p.add_argument('--body-review',type=Path)
    p.add_argument('--fixed-body-projection',action='store_true',help='use full fixed structure outline on lidar and unknown layers')
    p.add_argument('--hold-plan',type=Path)
    p.add_argument('--depth-review',type=Path)
    p.add_argument('--mobile-mesh-dir',type=Path)
    p.add_argument('--d415-mesh',type=Path)
    p.add_argument('--lds-mesh',type=Path)
    p.add_argument('--lift-m',type=float)
    p.add_argument('--clearance-m',type=float,default=.03)
    args=p.parse_args()
    if args.living_map_review:
        if args.slam_review is None or args.body_review is None:
            p.error('--living-map-review requires --slam-review and --body-review')
        print(json.dumps(review_living_map(args.slam_review,args.body_review,args.output)))
        raise SystemExit(0)
    if args.movement_area and not args.traversability:p.error('--movement-area requires --traversability')
    if args.ray_depth_review:
        if args.traversability or args.semantic or args.exclude_arms or any(v is None for v in (args.capture,args.slam_review,args.trajectory_review)):
            p.error('--ray-depth-review requires capture, final SLAM and trajectory inputs')
        from home_robot_tasks.recorded_depth import review_recorded_depth
        print(json.dumps(review_recorded_depth(args.capture,args.slam_review,args.trajectory_review,args.output)))
        raise SystemExit(0)
    if args.fixed_body_projection and not (args.traversability and args.exclude_arms):p.error('--fixed-body-projection requires --traversability --exclude-arms')
    if args.exclude_arms and not args.traversability:p.error('--exclude-arms requires --traversability')
    if args.fused_review:
        if any(v is None for v in (args.slam_review,args.depth_review,args.body_review,args.semantic_review)):
            p.error('--fused-review requires --slam-review, --depth-review, --body-review, --semantic-review')
        result=review_fused_layers(args.slam_review,args.depth_review,args.body_review,args.semantic_review,args.output)
        print(json.dumps({k:result[k]for k in ('depth_frames','semantic_views','recording_chunks','rgbd_added_on_planar_free_cells')}))
        raise SystemExit(0)
    if args.traversability:
        if args.semantic or any(v is None for v in (args.hold_plan,args.depth_review,args.slam_review,args.mobile_mesh_dir,args.d415_mesh,args.lds_mesh,args.lift_m)):
            p.error('--traversability requires recorded maps, hold plan, original meshes and explicit lift height')
        if not np.isfinite(args.clearance_m) or not 0<=args.clearance_m<=.2:p.error('invalid clearance')
        result=review_traversability(args)
        print(json.dumps(dict(dimensions_xyz_m=result['geometry']['dimensions_xyz_m'],height_m=result['geometry']['height_above_floor_m'],radius_m=result['geometry']['circumscribed_radius_m'],extra_upper_cells=result['lidar_free_cells_with_upper_hits'])))
        raise SystemExit(0)
    if (args.household_review or args.semantic_floor_filter) and not args.semantic:p.error('--household-review requires --semantic')
    if not args.capture:p.error('--capture required')
    if args.lidar_only:
        if not args.slam_review or args.semantic:p.error('--lidar-only requires --slam-review and excludes --semantic')
        print(json.dumps(review_lidar(args.capture,args.slam_review,args.output)))
    elif args.semantic:
        if not args.slam_review or not args.trajectory_review or not args.model_cache:
            p.error('--semantic requires saved SLAM, trajectory review and existing model cache')
        import os
        os.environ['HF_HUB_OFFLINE']='1';os.environ['TRANSFORMERS_OFFLINE']='1'
        sys.path.insert(0,str(ROOT/'tools/lib'))
        from grounding_rgbd import GroundingRGBD,MODEL_REVISION as DINO_REVISION
        from sam_rgbd import SAMMasks,MODEL_REVISION as SAM_REVISION
        from home_robot_tasks.recorded_semantics import review_semantics,clip_recorded_boxes
        class RecordedDetector(GroundingRGBD):
            def detect(self,*args):
                detections,elapsed=super().detect(*args)
                self.last_detections=clip_recorded_boxes(detections,args[0].shape)
                if getattr(self,'household',None) is not None:
                    self.last_detections=self.household.refine(args[0],self.last_detections)
                return self.last_detections,elapsed
        model=RecordedDetector(cache_dir=args.model_cache)
        semantic_revision='dino-'+DINO_REVISION[:12]+'-sam-'+SAM_REVISION[:12]
        if args.household_review:
            from household_semantics import HouseholdCropReviewer,MODEL_REVISION as HOUSEHOLD_REVISION
            semantic_revision+='-siglip2-'+HOUSEHOLD_REVISION[:12]
            model.household=HouseholdCropReviewer(args.model_cache,model.device)
        segmenter=SAMMasks(model.device,cache_dir=args.model_cache)
        if args.household_review or args.semantic_floor_filter:semantic_revision+='-floor-v1'
        result=review_semantics(ROOT,args.capture,args.slam_review,args.trajectory_review,args.output,
            model,segmenter,model_revision=semantic_revision,frames=args.semantic_frames,household_review=args.household_review or args.semantic_floor_filter)
        print(json.dumps(dict(frames=len(result['frames']),instances=len(result['instances']),
                             persisted_instances_identical=result['persisted_instances_identical'])))
    elif args.slam_review:
        if not args.trajectory_review:p.error('--trajectory-review required')
        print(json.dumps(review_moving(args.capture,args.slam_review,args.trajectory_review,args.output)))
    else:print(json.dumps(review(args.capture,args.output)))
