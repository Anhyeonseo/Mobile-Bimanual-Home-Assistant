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
          'map-to-odom corrections interpolated between online SLAM poses; not a final optimized graph trajectory',
          '8-pixel stride is for preview only; original full-resolution native depth retained',
          'upper endpoints are review candidates; no free space, obstacle clearing or semantic labels inferred'])
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
    t=rows[:,0]-rows[0,0];axes[2].plot(t,np.rad2deg(wheel[:,2]),label='Wheel yaw');axes[2].plot(t,np.rad2deg(rows[:,3]),label='SLAM yaw');axes[2].set(xlabel='Time (s)',ylabel='Yaw (degrees)',title='Rotation consistency');axes[2].grid(alpha=.2);axes[2].legend()
    fig.suptitle('Recorded-data review — no autonomous navigation approval');fig.tight_layout();fig.savefig(output/'map-review.png',dpi=160);plt.close(fig)
    return result


def travel_geometry(root, hold_plan, *, lift_m, mobile_mesh_dir, d415_mesh, lds_mesh):
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
    clouds=[];parts=[];sources={}
    def add(name,path,matrix,sha,scale=1.):
        vertices,raw=read_stl(Path(path))
        if hashlib.sha256(raw).hexdigest()!=sha:raise ValueError('original mesh hash mismatch: '+name)
        matrix=geom.rigid(matrix)
        vertices=(vertices*scale)@matrix[:3,:3].T+matrix[:3,3]
        vertices[:,2]-=profile['ground_z_m']
        clouds.append(vertices)
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
        frames=geom.arm_transforms(geometry,geom.model_angles(raw_arms[side],profile['arms'][side]['alignment']))
        name=side+'_base.STL'
        add(side+' mounting base',mobile_mesh_dir/name,platform@geom.origin(joints[side+'_base']),mounts['source_mesh_sha256'][name])
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
    # Display sample only: all dimensions above were calculated from full surfaces.
    return result,points[::max(1,len(points)//18000)]


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


def review_traversability(args):
    from scipy.ndimage import binary_dilation
    import math
    geometry,body=travel_geometry(ROOT,args.hold_plan,lift_m=args.lift_m,
        mobile_mesh_dir=args.mobile_mesh_dir,d415_mesh=args.d415_mesh,lds_mesh=args.lds_mesh)
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
    if depth_report.get('mode')!='offline_recorded_map_quality_preview':raise ValueError('recorded endpoint review required')
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
    forbidden=binary_dilation(combined,structure=kernel)
    extra=(grid==0)&upper
    report=dict(mode='recorded_body_clearance_candidate_review',geometry=geometry,
        candidate_clearance_m=args.clearance_m,vertical_interval_m=[bottom,top],
        upper_endpoint_voxels=n,upper_endpoint_xy_cells=int(upper.sum()),
        lidar_free_cells_with_upper_hits=int(extra.sum()),
        body_dilated_obstacle_cells=int(forbidden.sum()),
        free_space_certified=False,self_geometry_removed=False,depth_motion_qualified=False,
        runtime_authorized=False,motor_commands=0,live_map_modified=False,
        current_posture_applied_to_recorded_environment=True,
        limitations=['Recorded endpoint preview, not a full-height ray-cleared traversability map.',
            'Current observed joint posture does not certify the pose at every historical depth exposure.',
            'Self returns and unsupported accessories remain unresolved; no obstacle clearing performed.',
            'Circumcircle supports any yaw but can reject a narrow passage a fixed-yaw footprint could fit.',
            'Clearance is an explicit review setting, not measured braking or installation uncertainty.'],
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [args.slam_review/'occupancy.json',args.slam_review/'report.json',args.slam_review/'slam-poses.json',args.depth_review/'preview.npz',args.depth_review/'report.json']})
    output.mkdir(parents=True);(output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savez_compressed(output/'candidate-layers.npz',lidar_occupied=occupied,upper_hits=upper,body_forbidden=forbidden)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Circle
    from matplotlib import font_manager
    korean=any(f.name=='Noto Sans CJK KR' for f in font_manager.fontManager.ttflist)
    if korean:
        plt.rcParams['font.family']='Noto Sans CJK KR'
        plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(1,3,figsize=(16,5.5))
    extent=[*([origin[0],origin[0]+grid.shape[1]*resolution]),origin[1],origin[1]+grid.shape[0]*resolution]
    display=np.where(grid<0,0,np.where(occupied,2,1))
    for ax in axes[:2]:
        ax.imshow(display,origin='lower',extent=extent,cmap=ListedColormap(['#aeb8c2','#f7f5ee','#23323d']),vmin=0,vmax=2)
        ax.plot(poses[:,1],poses[:,2],color='#16837a',linewidth=1.5)
        ax.set(xlabel='Map X (m)',ylabel='Map Y (m)',aspect='equal')
    axes[0].set_title('라이다 + 상부 깊이 형상 (주황)' if korean else 'Lidar + observed upper surfaces')
    axes[0].imshow(np.ma.masked_where(~upper,upper),origin='lower',extent=extent,cmap=ListedColormap(['#df802f']),alpha=.8)
    axes[1].imshow(np.ma.masked_where(~forbidden,forbidden),origin='lower',extent=extent,cmap=ListedColormap(['#c95252']),alpha=.5)
    axes[1].add_patch(Circle(poses[-1,1:3],radius,fill=False,color='#16837a',linewidth=2))
    axes[1].set_title('로봇 중심 접근 제한 후보 (빨강)' if korean else 'Body clearance candidates / any yaw')
    axes[2].scatter(body[:,0],body[:,2],s=.35,c='#354d61',alpha=.3)
    axes[2].axhline(bottom,color='#df802f',linestyle='--',label='Upper review starts')
    axes[2].axhline(geometry['height_above_floor_m'],color='#16837a',linestyle='--',label='Model body height')
    axes[2].set(xlabel='Robot forward X (m)',ylabel='Height above floor (m)',aspect='equal',title='원본 STL · 현재 팔 자세' if korean else 'Original STL / current arm posture')
    axes[2].legend(fontsize=8)
    fig.suptitle('기록 지도 검토 · 밝은 영역도 아직 통행 가능으로 확인한 곳은 아님' if korean else 'Recorded-data comparison: pale areas are NOT certified traversable',fontsize=13)
    fig.tight_layout();fig.savefig(output/'traversability-review.png',dpi=160);plt.close(fig)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--slam-review',type=Path);p.add_argument('--trajectory-review',type=Path)
    p.add_argument('--semantic',action='store_true',help='Offline DINO/SAM furniture annotations; never modifies live occupancy')
    p.add_argument('--semantic-frames',type=int,default=6)
    p.add_argument('--model-cache',type=Path,help='Existing pinned DINO/SAM cache; semantic review uses offline model loading')
    p.add_argument('--traversability',action='store_true')
    p.add_argument('--hold-plan',type=Path)
    p.add_argument('--depth-review',type=Path)
    p.add_argument('--mobile-mesh-dir',type=Path)
    p.add_argument('--d415-mesh',type=Path)
    p.add_argument('--lds-mesh',type=Path)
    p.add_argument('--lift-m',type=float)
    p.add_argument('--clearance-m',type=float,default=.03)
    args=p.parse_args()
    if args.traversability:
        if args.semantic or any(v is None for v in (args.hold_plan,args.depth_review,args.slam_review,args.mobile_mesh_dir,args.d415_mesh,args.lds_mesh,args.lift_m)):
            p.error('--traversability requires recorded maps, hold plan, original meshes and explicit lift height')
        if not np.isfinite(args.clearance_m) or not 0<=args.clearance_m<=.2:p.error('invalid clearance')
        result=review_traversability(args)
        print(json.dumps(dict(dimensions_xyz_m=result['geometry']['dimensions_xyz_m'],height_m=result['geometry']['height_above_floor_m'],radius_m=result['geometry']['circumscribed_radius_m'],extra_upper_cells=result['lidar_free_cells_with_upper_hits'])))
        raise SystemExit(0)
    if not args.capture:p.error('--capture required')
    if args.semantic:
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
                return self.last_detections,elapsed
        model=RecordedDetector(cache_dir=args.model_cache)
        segmenter=SAMMasks(model.device,cache_dir=args.model_cache)
        result=review_semantics(ROOT,args.capture,args.slam_review,args.trajectory_review,args.output,
            model,segmenter,model_revision='dino-'+DINO_REVISION[:12]+'-sam-'+SAM_REVISION[:12],frames=args.semantic_frames)
        print(json.dumps(dict(frames=len(result['frames']),instances=len(result['instances']),
                             persisted_instances_identical=result['persisted_instances_identical'])))
    elif args.slam_review:
        if not args.trajectory_review:p.error('--trajectory-review required')
        print(json.dumps(review_moving(args.capture,args.slam_review,args.trajectory_review,args.output)))
    else:print(json.dumps(review(args.capture,args.output)))
