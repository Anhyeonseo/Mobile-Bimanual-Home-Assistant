"""Offline recorded encoder/scan analysis; never a live calibration or drive gate.

Reuses nominal CAD kinematics and the strict teleop record reviewer. The trimmed
ICP method extends the earlier small-motion diagnostic, retaining explicit
residual/overlap failures instead of turning uncertain registration into motion.
"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from .base_motion import OmniKinematics, Wheel
from .s4_base_preparation import review_teleop_capture


def transform(pose):
    x,y,a=pose;c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,x],[s,c,y],[0.,0.,1.]])


def pose(matrix):
    return np.array([matrix[0,2],matrix[1,2],math.atan2(matrix[1,0],matrix[0,0])])


def interpolate(rows, times):
    rows=np.asarray(rows);times=np.asarray(times)
    if (rows.ndim!=2 or rows.shape[1]!=4 or len(rows)<2 or not np.isfinite(rows).all()
            or not np.isfinite(times).all() or np.any(np.diff(rows[:,0])<=0)):
        raise ValueError("finite ordered pose samples required")
    if np.any(times<rows[0,0]) or np.any(times>rows[-1,0]):
        raise ValueError('pose extrapolation refused')
    return np.column_stack([np.interp(times,rows[:,0],rows[:,i]) for i in range(1,4)])


def encoder_path(records, geometry):
    review=review_teleop_capture(records)
    if review['verified_pauses']:raise ValueError('split paused drive epochs before mapping')
    model=OmniKinematics(tuple(Wheel(**{k:w[k] for k in
        ('x_m','y_m','rolling_angle_rad','radius_m')}) for w in geometry['wheels']))
    replies=[r for r in records if r['event']=='reply']
    times=[];positions=[];acquisition=[]
    commands=[r for r in records if r['event']=='command']
    for c,r in zip(commands,replies,strict=True):
        # No hardware time synchronization claim: host request/reply midpoint.
        times.append((c['host_monotonic_ns']+r['host_monotonic_ns'])/2e9)
        acquisition.append((r['host_monotonic_ns']-c['host_monotonic_ns'])/1e9)
        positions.append([w['position_raw'] for w in r['reply']['wheel_feedback']])
    positions=np.array(positions);deltas=(np.diff(positions,axis=0)+2048)%4096-2048
    if np.any(abs(deltas)>=1024):raise ValueError('ambiguous encoder unwrap')
    current=np.eye(3);rows=[[times[0],0.,0.,0.]];length=turn=0.
    for at,delta in zip(times[1:],deltas):
        dx,dy,da=model.body_twist((delta*math.tau/4096).tolist())
        if abs(da)<1e-9:a,b=1.,da/2
        else:a,b=math.sin(da)/da,(1-math.cos(da))/da
        current=current@transform((a*dx-b*dy,b*dx+a*dy,da))
        rows.append([at,*pose(current)]);length+=math.hypot(dx,dy);turn+=abs(da)
    rows=np.array(rows);rows[:,3]=np.unwrap(rows[:,3])
    return rows,dict(review=review,path_length_m=length,absolute_turn_rad=turn,
        final_pose=rows[-1,1:].tolist(),maximum_host_acquisition_s=max(acquisition),
        timestamp_basis='host command/reply midpoint, wheel acquisitions are sequential')


def scan_points(scan,mount,odom=None):
    ranges=np.array(scan['ranges'],dtype=float)
    angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    valid=np.isfinite(ranges)&(ranges>=.25)&(ranges<=3.5)
    p=np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid]),np.ones(valid.sum())))
    p=(mount@p.T).T
    if odom is not None:
        # LDS arrays have decreasing acquisition time as angle increases.
        times=scan['observed_s']+np.flatnonzero(valid)*scan['time_increment']
        reference=transform(interpolate(odom,[scan['observed_s']])[0])
        inverse=np.linalg.inv(reference)
        ray_poses=interpolate(odom,times)
        p=np.array([inverse@transform(q)@point for q,point in zip(ray_poses,p)])
    return p[:,:2]


def align(source,target,initial=None):
    """Trimmed point-to-point ICP; source coordinates mapped into target."""
    if min(len(source),len(target))<60:raise ValueError('insufficient scan endpoints')
    matrix=np.eye(3) if initial is None else np.array(initial,copy=True)
    tree=cKDTree(target)
    for _ in range(45):
        p=source@matrix[:2,:2].T+matrix[:2,2]
        distances,indices=tree.query(p)
        keep=distances<=min(.15,float(np.quantile(distances,.85)))
        if keep.sum()<60:raise ValueError('insufficient registration overlap')
        pp=p[keep];q=target[indices[keep]];pc=pp.mean(axis=0);qc=q.mean(axis=0)
        u,_,vt=np.linalg.svd((pp-pc).T@(q-qc));rotation=vt.T@u.T
        if np.linalg.det(rotation)<0:vt[-1]*=-1;rotation=vt.T@u.T
        step=np.eye(3);step[:2,:2]=rotation;step[:2,2]=qc-rotation@pc
        matrix=step@matrix
        if np.linalg.norm(step[:2,2])<1e-7 and abs(pose(step)[2])<1e-7:break
    p=source@matrix[:2,:2].T+matrix[:2,2];distances,_=tree.query(p)
    keep=distances<=min(.15,float(np.quantile(distances,.85)))
    return matrix,dict(rmse_m=float(np.sqrt(np.mean(distances[keep]**2))),
                       overlap_fraction=float(keep.mean()),inliers=int(keep.sum()))


def review_recorded(root,teleop,capture,output):
    root,teleop,capture,output=map(Path,(root,teleop,capture,output))
    if output.exists():raise ValueError('new immutable review directory required')
    records=[json.loads(line) for line in teleop.read_text().splitlines()]
    source_report=json.loads((capture/'report.json').read_text())
    if source_report.get('mode')!='real_sensor' or source_report.get('passed') is not True:
        raise ValueError('successful real sensor capture required')
    if source_report.get('teleop_finished')!=records[-1]:raise ValueError('capture belongs to different teleop session')
    geometry=json.loads((root/'config/home_assistant/s4_base.nominal.json').read_text())
    odom,encoder=encoder_path(records,geometry)
    mount_data=json.loads((root/'config/home_assistant/lidar_mount.review.json').read_text())
    mount=transform([*mount_data['base_from_lidar_translation_m'][:2],mount_data['yaw_rad']])
    scans=json.loads((capture/'scans.json').read_text())
    selected=[]
    for s in scans:
        if s['frame']!='lidar_a' or s['count']!=360:raise ValueError('unexpected scan geometry')
        if not -.001<s['time_increment']<0:raise ValueError('signed LDS ray timing required')
        times=[s['observed_s'],s['observed_s']+359*s['time_increment']]
        if min(times)>=odom[0,0] and max(times)<=odom[-1,0]:selected.append(s)
    if len(selected)<10:raise ValueError('insufficient synchronized scans')
    matrices=[];diagnostics=[];zero=np.eye(3);previous=None
    # Adjacent scans start at identity, independently of encoder displacement.
    # Deskew uses nominal encoders; report that dependence explicitly.
    for i,s in enumerate(selected):
        points=scan_points(s,mount,odom)
        if previous is not None:
            delta,quality=align(points,previous)
            if quality['rmse_m']>.06 or quality['overlap_fraction']<.6:
                raise ValueError('scan registration not reliable enough to join')
            zero=zero@delta;diagnostics.append(dict(time=s['observed_s'],**quality))
        matrices.append(zero.copy());previous=points
    registered=np.array([[s['observed_s'],*pose(m)] for s,m in zip(selected,matrices)])
    registered[:,3]=np.unwrap(registered[:,3])
    baseline=transform(interpolate(odom,[selected[0]['observed_s']])[0]);inverse=np.linalg.inv(baseline)
    sampled=np.array([pose(inverse@transform(q)) for q in interpolate(odom,registered[:,0])]);sampled[:,2]=np.unwrap(sampled[:,2])
    error=registered[:,1:]-sampled
    scan_path=float(np.linalg.norm(np.diff(registered[:,1:3],axis=0),axis=1).sum())
    # Compare accumulated and direct first/last registration to expose drift.
    try:
        direct,quality=align(scan_points(selected[-1],mount,odom),scan_points(selected[0],mount,odom),matrices[-1])
        closure=pose(np.linalg.inv(direct)@matrices[-1]).tolist()
    except ValueError as alignment_error:
        quality={'available':False,'reason':str(alignment_error)};closure=None
    manifest=[json.loads(line) for line in (capture/'rgbd/frames.jsonl').read_text().splitlines()]
    metadata=json.loads((capture/'rgbd/metadata.json').read_text());valid_archive=0
    for row in manifest:
        path=(capture/'rgbd'/row['file']).resolve()
        if not path.is_relative_to((capture/'rgbd').resolve()):raise ValueError('archive path outside root')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('archive hash mismatch')
        with np.load(path,allow_pickle=False) as frame:
            if frame['native_depth'].dtype!=np.uint16 or frame['native_depth'].shape!=(metadata['native_depth_intrinsics']['height'],metadata['native_depth_intrinsics']['width']):raise ValueError('native depth changed')
            if frame['color_bgr'].dtype!=np.uint8 or frame['color_bgr'].shape!=(metadata['color_intrinsics']['height'],metadata['color_intrinsics']['width'],3):raise ValueError('color changed')
        valid_archive+=1
    covered=[r for r in manifest if registered[0,0]<=r['depth_time_s']<=registered[-1,0]]
    result=dict(mode='offline_real_capture_review',hardware_commands=0,calibration_applied=False,
        physical_map_verified=False,floor_scale_qualified=False,encoder=encoder,
        source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (teleop,capture/'scans.json',capture/'report.json',root/'config/home_assistant/s4_base.nominal.json',root/'config/home_assistant/lidar_mount.review.json')},
        scan_registration_consistent_with_encoders=bool(np.linalg.norm(error[-1,:2])<.1 and abs(error[-1,2])<.15),
        selected_scans=len(selected),scan_path_length_m=scan_path,scan_final_pose=registered[-1,1:].tolist(),
        scan_vs_encoder_final_error=error[-1].tolist(),scan_vs_encoder_position_rmse_m=float(np.sqrt(np.mean(np.sum(error[:,:2]**2,axis=1)))),
        scan_pair_median_rmse_m=float(np.median([d['rmse_m'] for d in diagnostics])),
        first_last_registration=quality,accumulation_vs_direct_final_error=closure,
        native_archive_verified=valid_archive,native_frames_with_pose_coverage=len(covered),
        limitations=['nominal CAD wheel scale and reviewed lidar mount, not surveyed ground truth',
         'adjacent ICP initialized at identity but ray deskew uses nominal encoder motion',
         'short travel and uneven direction coverage do not qualify floor scale',
         'capture clock offsets and sensor exposure latency not independently calibrated'])
    output.mkdir(parents=True)
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    (output/'scans.json').write_text(json.dumps(selected)+'\n')
    np.savez_compressed(output/'trajectories.npz',encoder=odom,registered=registered,sampled_encoder=sampled)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(15,4))
    axes[0].plot(sampled[:,0],sampled[:,1],label='Nominal wheel encoders');axes[0].plot(registered[:,1],registered[:,2],label='Adjacent lidar alignment');axes[0].axis('equal');axes[0].set(xlabel='X (m)',ylabel='Y (m)',title='Recorded base trajectory');axes[0].legend()
    t=registered[:,0]-registered[0,0];axes[1].plot(t,np.rad2deg(sampled[:,2]),label='Encoders');axes[1].plot(t,np.rad2deg(registered[:,3]),label='Lidar');axes[1].set(xlabel='Time (s)',ylabel='Yaw (degrees)',title='Rotation comparison');axes[1].legend()
    for s,m in zip(selected[::4],matrices[::4]):
        pts=scan_points(s,mount,odom);world=pts@m[:2,:2].T+m[:2,2];axes[2].scatter(world[:,0],world[:,1],s=.4,alpha=.2)
    axes[2].axis('equal');axes[2].set(xlabel='X (m)',ylabel='Y (m)',title='Lidar endpoint overlay (not a navigation map)')
    for ax in axes:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(output/'movement-review.png',dpi=160);plt.close(fig)
    return result


def localization_comparison(poses,clock_offset,reference,reference_offset):
    """Replay consistency against SLAM, never surveyed physical accuracy."""
    rows=np.array([[p['stamp']-reference_offset,*p['xy_yaw']] for p in reference])
    rows[:,3]=np.unwrap(rows[:,3])
    actual=np.array([[p['stamp']-clock_offset,*p['xy_yaw']] for p in poses])
    actual=actual[(actual[:,0]>=rows[0,0])&(actual[:,0]<=rows[-1,0])]
    if len(actual)<3:raise ValueError('insufficient shared localization timeline')
    expected=interpolate(rows,actual[:,0]);errors=actual[:,1:]-expected
    errors[:,2]=(errors[:,2]+math.pi)%math.tau-math.pi
    xy=np.linalg.norm(errors[:,:2],axis=1);yaw=abs(errors[:,2]);tail=actual[:,0]>=actual[-1,0]-5
    # Diagnostic only. These thresholds never authorize motion and do not
    # compare AMCL against encoder position or demand agreement with it.
    return dict(compared_pose_messages=len(actual),
        final_slam_difference_m=float(xy[-1]),final_slam_difference_deg=float(np.rad2deg(yaw[-1])),
        last_5s_median_slam_difference_m=float(np.median(xy[tail])),
        last_5s_median_slam_difference_deg=float(np.rad2deg(np.median(yaw[tail]))),
        tracking_sanity_passed=bool(np.median(xy[tail])<.15 and np.median(yaw[tail])<.2),
        physical_localization_qualified=False,encoder_agreement_required=False)


def localization_map_review(poses, clock_offset, scans, odom, mount, saved):
    """Score a new recording against a fixed older map, without invented truth.

    Report the first ten seconds separately for global initialization. The
    diagnostic threshold is not a live-motion gate or surveyed pose accuracy.
    """
    rows=np.array([[p['stamp']-clock_offset,*p['xy_yaw']] for p in poses])
    if len(rows)<3:raise ValueError('insufficient localization poses')
    rows[:,3]=np.unwrap(rows[:,3])
    wheel=interpolate(odom,rows[:,0])
    corrections=np.array([[r[0],*pose(transform(r[1:])@np.linalg.inv(transform(w)))]
                          for r,w in zip(rows,wheel)])
    corrections[:,3]=np.unwrap(corrections[:,3])
    grid=np.array(saved['data']).reshape(saved['height'],saved['width'])
    origin=np.array(saved['origin']);resolution=saved['resolution']
    yy,xx=np.nonzero(grid>=65)
    if len(xx)<10:raise ValueError('insufficient occupied map cells')
    tree=cKDTree(origin+(np.column_stack((xx,yy))+.5)*resolution)
    metrics=[];distances=[];steady=[]
    for scan in scans:
        t=scan['observed_s']
        if not rows[0,0]<=t<=rows[-1,0]:continue
        if t+(len(scan['ranges'])-1)*scan['time_increment']<odom[0,0]:continue
        points=scan_points(scan,mount,odom)
        if not len(points):continue
        T=transform(interpolate(corrections,[t])[0])@transform(interpolate(odom,[t])[0])
        world=points@T[:2,:2].T+T[:2,2]
        d=tree.query(world)[0];distances.extend(d)
        elapsed=float(t-rows[0,0])
        if elapsed>=10:steady.extend(d)
        metrics.append(dict(elapsed_s=elapsed,endpoints=len(d),median_m=float(np.median(d)),
            p90_m=float(np.percentile(d,90)),within_10cm_fraction=float(np.mean(d<=.1))))
    if len(metrics)<10 or not steady:raise ValueError('insufficient scan/map review coverage')
    def stats(values):
        d=np.asarray(values)
        return dict(endpoints=len(d),median_m=float(np.median(d)),p90_m=float(np.percentile(d,90)),
                    within_10cm_fraction=float(np.mean(d<=.1)))
    steady_stats=stats(steady)
    # The pose at +10 s is an analysis baseline, not a measured start marker.
    start=interpolate(rows,[rows[0,0]+10])[0]
    delta=pose(np.linalg.inv(transform(start))@transform(rows[-1,1:]))
    steps=np.diff(corrections[:,1:],axis=0)
    after=rows[1:,0]>=rows[0,0]+10
    return dict(compared_scans=len(metrics),pose_time_coverage_s=float(rows[-1,0]-rows[0,0]),
        all_scan_to_map=stats(distances),after_10s_scan_to_map=steady_stats,per_scan=metrics,
        maximum_pose_message_gap_s=float(np.max(np.diff(rows[:,0]))),
        maximum_map_correction_step_after_10s_m=float(np.max(np.linalg.norm(steps[after,:2],axis=1))),
        maximum_map_correction_step_after_10s_deg=float(np.rad2deg(np.max(abs(steps[after,2])))),
        final_pose_xy_yaw=rows[-1,1:].tolist(),relative_to_pose_at_10s=delta.tolist(),
        tracking_sanity_passed=bool(steady_stats['within_10cm_fraction']>=.8 and steady_stats['p90_m']<=.2),
        physical_localization_qualified=False,encoder_agreement_required=False,
        measured_return_error_available=False)


def map_initial_pose_candidates(scan, odom, mount, saved):
    """Offline hypotheses from one initial scan; never a motor enable decision.

    Search free map cells at coarse resolution, then reuse bounded ICP. Keep
    distinct alternatives so repeated room geometry is not hidden by one fit.
    No subsequent trajectory or final pose is used to select the hypothesis.
    """
    grid=np.array(saved['data']).reshape(saved['height'],saved['width'])
    origin=np.array(saved['origin']);resolution=saved['resolution']
    yy,xx=np.nonzero(grid>=65);target=origin+(np.column_stack((xx,yy))+.5)*resolution
    if len(target)<60:raise ValueError('insufficient occupied map cells for initialization')
    tree=cKDTree(target);source=scan_points(scan,mount,odom)
    if len(source)<60:raise ValueError('insufficient initial scan endpoints')
    step=max(1,round(.1/resolution));yy,xx=np.indices(grid.shape)
    free=(grid==0)&(xx%step==0)&(yy%step==0)
    translations=origin+(np.column_stack((xx[free],yy[free]))+.5)*resolution
    if not len(translations):raise ValueError('no free cells for initialization')
    sparse=source[::4];coarse=[]
    for yaw in np.arange(-math.pi,math.pi,math.radians(10)):
        rotated=sparse@transform((0,0,yaw))[:2,:2].T
        distances=tree.query(translations[:,None,:]+rotated[None,:,:])[0]
        costs=np.minimum(distances,.2).mean(axis=1)
        for index in np.argsort(costs)[:3]:
            coarse.append((float(costs[index]),[*translations[index],float(yaw)]))
    candidates=[]
    for _,seed in sorted(coarse)[:40]:
        try:matrix,quality=align(source,target,transform(seed))
        except ValueError:continue
        candidate=pose(matrix)
        if any(np.linalg.norm(candidate[:2]-np.array(c['xy_yaw'][:2]))<.15 and
               abs((candidate[2]-c['xy_yaw'][2]+math.pi)%math.tau-math.pi)<.15 for c in candidates):continue
        world=source@matrix[:2,:2].T+matrix[:2,2];distances=tree.query(world)[0]
        candidates.append(dict(xy_yaw=candidate.tolist(),clipped_mean_distance_m=float(np.minimum(distances,.2).mean()),
            within_10cm_fraction=float(np.mean(distances<=.1)),registration=quality))
    candidates.sort(key=lambda c:c['clipped_mean_distance_m'])
    if not candidates:raise ValueError('no initial alignment hypothesis')
    return dict(method='first_scan_coarse_free_cell_search_then_existing_icp',
        source_scan_observed_s=scan['observed_s'],later_scans_used=False,
        candidates=candidates[:5],physical_pose_verified=False,automatic_motion_authorized=False)
