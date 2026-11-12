#!/usr/bin/env python3
"""Project a saved real sensor capture into floor XY. No live-map or free-space claims."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
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


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--slam-review',type=Path);p.add_argument('--trajectory-review',type=Path)
    args=p.parse_args()
    if args.slam_review:
        if not args.trajectory_review:p.error('--trajectory-review required')
        print(json.dumps(review_moving(args.capture,args.slam_review,args.trajectory_review,args.output)))
    else:print(json.dumps(review(args.capture,args.output)))
