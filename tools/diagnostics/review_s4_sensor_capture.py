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


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();print(json.dumps(review(args.capture,args.output)))
