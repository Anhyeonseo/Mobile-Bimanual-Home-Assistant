#!/usr/bin/env python3
"""Offline D415 bag -> aligned RGB-D -> conservative, independent 3D fragments.

No hardware camera is opened. No poses are invented across tracking failures.
This diagnostic is local RGB-D odometry, NOT loop-closed whole-house SLAM.
"""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time

os.environ.setdefault('OMP_NUM_THREADS', '4')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

if __package__:
    from .room_scan_quality import image_quality, summarize_quality
else:
    from room_scan_quality import image_quality, summarize_quality


def save(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def scaled_intrinsics(intr, width, height):
    sx, sy = width / intr['width'], height / intr['height']
    return {'width': width, 'height': height, 'fx': intr['fx'] * sx, 'fy': intr['fy'] * sy,
            'cx': (intr['cx'] + 0.5) * sx - 0.5, 'cy': (intr['cy'] + 0.5) * sy - 0.5}


def export_bag(bag, output, fps=10, realtime=False):
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    if output.exists():
        raise RuntimeError(f'이미 있는 데이터 폴더입니다: {output}; 새 폴더를 지정하세요.')
    output.mkdir(parents=True)
    (output / 'color').mkdir()
    (output / 'depth').mkdir()
    cfg = rs.config()
    cfg.enable_device_from_file(str(bag.resolve()), repeat_playback=False)
    pipeline = rs.pipeline()
    # Resolve only the recorded device. This never falls back to a live camera.
    profile = cfg.resolve(rs.pipeline_wrapper(pipeline))
    device = profile.get_device()
    playback = device.as_playback()
    playback.set_real_time(realtime)
    scale = device.first_depth_sensor().get_depth_scale()
    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_profile.get_intrinsics()
    if any(abs(c) > 1e-8 for c in intr.coeffs) and intr.model != rs.distortion.brown_conrady:
        raise RuntimeError(f'지원하지 않는 색상 왜곡 모델: {intr.model}')
    native = dict(width=intr.width, height=intr.height, fx=intr.fx, fy=intr.fy, cx=intr.ppx, cy=intr.ppy)
    calibration = scaled_intrinsics(native, 640, 360)
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.]])
    maps = cv2.initUndistortRectifyMap(K, np.array(intr.coeffs), None, K,
                                     (intr.width, intr.height), cv2.CV_32FC1)
    align = rs.align(rs.stream.color)
    rows, errors = [], []
    first = None
    previous = -1e20
    seen = set()
    last_seen = None
    duration = playback.get_duration().total_seconds()
    done = threading.Event()

    def read(frame):
        nonlocal first, previous, last_seen
        try:
            frames = frame.as_frameset()
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if not color or not depth:
                return
            number = color.get_frame_number()
            stamp = color.get_timestamp()
            if first is None:
                first = stamp
            last_seen = (stamp - first) / 1000
            if number in seen or stamp - previous < 1000 / fps - 2:
                return
            seen.add(number)
            previous = stamp
            skew = abs(stamp - depth.get_timestamp())
            if skew > 40:
                rows.append({'seconds': last_seen, 'excluded': 'rgb_depth_skew', 'skew_ms': skew})
                return
            aligned = align.process(frames)
            rgb = np.asanyarray(aligned.get_color_frame().get_data())
            if color_profile.format() == rs.format.rgb8:
                rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif color_profile.format() != rs.format.bgr8:
                raise RuntimeError('Only RGB8/BGR8 color is supported')
            dep = np.asanyarray(aligned.get_depth_frame().get_data())
            rgb = cv2.remap(rgb, *maps, cv2.INTER_LINEAR)
            dep = cv2.remap(dep, *maps, cv2.INTER_NEAREST)
            rgb = cv2.resize(rgb, (640, 360), interpolation=cv2.INTER_AREA)
            dep = cv2.resize(dep, (640, 360), interpolation=cv2.INTER_NEAREST_EXACT)
            # Store millimetres for a portable dataset with explicit depth scale.
            dep = np.rint(dep.astype(np.float64) * scale * 1000).clip(0, 65535).astype(np.uint16)
            quality = image_quality(rgb, dep)
            name = f'{len(rows):06d}.png'
            for kind, pixels in [('color', rgb), ('depth', dep)]:
                if not cv2.imwrite(str(output / kind / name), pixels):
                    raise RuntimeError(f'이미지 저장 실패: {kind}/{name}')
            rows.append({'seconds': last_seen, 'frame_number': number, 'skew_ms': skew,
                         'color': f'color/{name}', 'depth': f'depth/{name}', 'quality': quality})
            if len(rows) % 100 == 0:
                print(f'RGB-D 추출 {last_seen:.1f}/{duration:.1f}초 · {len(rows)}장', flush=True)
        except Exception as error:
            errors.append(str(error))
            done.set()

    try:
        active = pipeline.start(cfg)
        playback = active.get_device().as_playback()
        playback.set_real_time(realtime)
        deadline = time.monotonic() + max(180, duration * 12)
        while not done.is_set():
            if time.monotonic() > deadline:
                raise RuntimeError('녹화파일 추출 시간 초과')
            try:
                frames = pipeline.wait_for_frames(1000)
            except RuntimeError:
                if rows and playback.current_status() == rs.playback_status.stopped:
                    break
                raise
            # Do not pause a playback device already reaching EOF (SDK can wait forever).
            color = frames.get_color_frame()
            near_end = first is not None and color and (color.get_timestamp() - first) / 1000 >= duration - .20
            if near_end:
                read(frames)
                break
            if realtime:
                # On ARM, frequent pause/resume can repeatedly decode large bag chunks.
                # Play at the original rate instead; record actual export gaps below.
                read(frames)
                continue
            # Pause while aligning/writing so disk latency cannot silently skip the scene.
            playback.pause()
            read(frames)
            playback.resume()
    finally:
        pipeline.stop()
    if errors:
        raise RuntimeError(errors[0])
    if last_seen is None or last_seen < duration - 1:
        raise RuntimeError('끝까지 읽지 못했습니다. 추출 데이터는 완전하지 않습니다.')
    qualities = [r['quality'] for r in rows if 'quality' in r]
    exported = [r for r in rows if 'color' in r]
    gaps = [b['seconds'] - a['seconds'] for a, b in zip(exported, exported[1:])]
    report = {'schema_version': 1, 'source_bag': bag.name, 'source_bytes': bag.stat().st_size,
              'duration_seconds': duration, 'requested_fps': fps,
              'realtime_playback': realtime,
              'export_max_gap_seconds': max(gaps, default=0),
              'export_gaps_over_250ms': sum(g > .25 for g in gaps),
              'effective_export_fps': (len(exported) - 1) / (exported[-1]['seconds'] - exported[0]['seconds']) if len(exported) > 1 else 0,
              'last_read_seconds': last_seen, 'last_exported_seconds': rows[-1]['seconds'],
              'unexported_tail_seconds': max(0, duration - rows[-1]['seconds']),
              'tail_policy': 'finish within final 0.20s to avoid librealsense pause-at-EOF deadlock',
              'depth_scale_units_per_meter': 1000, 'depth_aligned_to_color': True,
              'color_rectified': True, 'native_intrinsics': native, 'intrinsics': calibration,
              'quality_summary': summarize_quality(qualities), 'frames': rows}
    save(output / 'dataset.json', report)
    return report


def valid_transform(T, seconds):
    import numpy as np
    if T.shape != (4, 4) or not np.isfinite(T).all():
        return False
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-3) or abs(np.linalg.det(R) - 1) > 1e-3:
        return False
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6):
        return False
    angle = math.acos(float(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    return float(np.linalg.norm(T[:3, 3])) <= max(0.12, seconds * 1.5) and angle <= max(0.20, seconds * 2.0)


def reconstruct(dataset, output):
    import cv2
    import numpy as np
    import open3d as o3d
    cv2.setNumThreads(2)
    data = json.loads((dataset / 'dataset.json').read_text())
    if output.exists():
        raise RuntimeError(f'이미 있는 결과 폴더입니다: {output}; 새 폴더를 지정하세요.')
    output.mkdir(parents=True)
    p = data['intrinsics']
    intrinsic = o3d.camera.PinholeCameraIntrinsic(p['width'], p['height'], p['fx'], p['fy'], p['cx'], p['cy'])
    K = intrinsic.intrinsic_matrix
    sift = cv2.SIFT_create(nfeatures=1800)
    matcher = cv2.BFMatcher()
    options = o3d.pipelines.odometry.OdometryOption()
    options.depth_min = 0.45
    options.depth_max = 3.0
    options.depth_diff_max = 0.07
    segments, events, trajectory = [], [], []
    last = None
    pose = np.eye(4)
    volume = None
    current = []

    def finish():
        nonlocal volume, current
        if volume is None:
            return
        index = len(segments)
        pointcloud = volume.extract_point_cloud()
        mesh = volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        pointcloud.colors = o3d.utility.Vector3dVector(np.clip(np.asarray(pointcloud.colors), 0, 1))
        mesh.vertex_colors = o3d.utility.Vector3dVector(np.clip(np.asarray(mesh.vertex_colors), 0, 1))
        mesh_path = f'fragment-{index:02d}.ply'
        cloud_path = f'fragment-{index:02d}-points.ply'
        if not o3d.io.write_triangle_mesh(str(output / mesh_path), mesh):
            raise RuntimeError('Mesh write failed')
        if not o3d.io.write_point_cloud(str(output / cloud_path), pointcloud):
            raise RuntimeError('Point cloud write failed')
        extent = pointcloud.get_axis_aligned_bounding_box().get_extent().tolist() if len(pointcloud.points) else [0, 0, 0]
        segments.append({'id': index, 'start_seconds': current[0]['seconds'], 'end_seconds': current[-1]['seconds'],
                         'frames': len(current), 'points': len(pointcloud.points), 'triangles': len(mesh.triangles),
                         'extent_m': extent, 'mesh': mesh_path, 'pointcloud': cloud_path,
                         'frame': 'first_camera_of_this_fragment'})
        print(f'복원 조각 {index}: {len(current)}장 · {current[0]["seconds"]:.1f}~{current[-1]["seconds"]:.1f}초', flush=True)
        volume, current = None, []

    for idx, row in enumerate(data['frames']):
        if 'excluded' in row or row['quality']['useful_depth_fraction'] < 0.20:
            events.append({'seconds': row['seconds'], 'reason': row.get('excluded', 'insufficient_depth')})
            finish()
            last = None
            continue
        rgb = cv2.imread(str(dataset / row['color']))
        depth = cv2.imread(str(dataset / row['depth']), cv2.IMREAD_UNCHANGED)
        depth[(depth < 450) | (depth > 3000)] = 0
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = sift.detectAndCompute(gray, (depth > 0).astype(np.uint8) * 255)
        rgbd_color = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)), o3d.geometry.Image(depth),
            depth_scale=1000, depth_trunc=3.0, convert_rgb_to_intensity=False)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)), o3d.geometry.Image(depth),
            depth_scale=1000, depth_trunc=3.0)
        cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_color, intrinsic).voxel_down_sample(0.04)
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.10, max_nn=20))
        success, reason = False, 'first_frame'
        stats = {}
        if last is not None and row['seconds'] - last['row']['seconds'] <= 0.25:
            dt = row['seconds'] - last['row']['seconds']
            initial = np.eye(4)
            # Feature correspondences provide a larger motion basin than identity-only odometry.
            if descriptors is not None and last['descriptors'] is not None and len(descriptors) >= 2:
                matches = matcher.knnMatch(last['descriptors'], descriptors, k=2)
                object_points, image_points = [], []
                for pair in matches:
                    if len(pair) != 2 or pair[0].distance >= .7 * pair[1].distance:
                        continue
                    m = pair[0]
                    u, v = last['keypoints'][m.queryIdx].pt
                    x, y = round(u), round(v)
                    if not (0 <= x < p['width'] and 0 <= y < p['height']):
                        continue
                    z = float(last['depth'][y, x]) / 1000
                    if not .45 <= z <= 3:
                        continue
                    object_points.append([(u - p['cx']) * z / p['fx'], (v - p['cy']) * z / p['fy'], z])
                    image_points.append(keypoints[m.trainIdx].pt)
                if len(object_points) >= 12:
                    ok, rvec, tvec, inliers = cv2.solvePnPRansac(np.float32(object_points), np.float32(image_points), K, None,
                                                              iterationsCount=150, reprojectionError=2.5, confidence=.999)
                    if ok and inliers is not None and len(inliers) >= 12:
                        initial[:3, :3], initial[:3, 3] = cv2.Rodrigues(rvec)[0], tvec[:, 0]
                        stats['pnp_inliers'] = len(inliers)
            ok, T, information = o3d.pipelines.odometry.compute_rgbd_odometry(
                last['rgbd'], rgbd, intrinsic, initial,
                o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), options)
            if ok and valid_transform(T, dt):
                check = o3d.pipelines.registration.evaluate_registration(last['cloud'], cloud, 0.06, T)
                stats.update(fitness=float(check.fitness), rmse_m=float(check.inlier_rmse))
                # Depth overlap alone can align a featureless plane incorrectly.
                # Require feature support OR sufficiently informative spatial geometry.
                evals = np.linalg.eigvalsh(information)
                stats['information_ratio'] = float(evals[0] / max(evals[-1], 1))
                normals = np.asarray(cloud.normals)
                normal_evals = np.linalg.eigvalsh(normals.T @ normals)
                stats['normal_diversity'] = float(normal_evals[0] / max(normal_evals[-1], 1))
                # Point-to-point information can look well-conditioned on a flat wall.
                # Require visual support or normals spanning three directions instead.
                success = check.fitness >= .55 and check.inlier_rmse <= .035 and (stats.get('pnp_inliers', 0) >= 12 or stats['normal_diversity'] >= .02)
                reason = 'low_overlap_or_ambiguous_geometry' if not success else 'tracked'
            else:
                reason = 'odometry_failed_or_implausible_motion'
            if success:
                # T maps previous-camera points into current-camera coordinates.
                pose = pose @ np.linalg.inv(T)
        elif last is not None:
            reason = 'timestamp_gap'
        if not success:
            if last is not None:
                events.append({'seconds': row['seconds'], 'reason': reason, **stats})
            finish()
            pose = np.eye(4)
            volume = o3d.pipelines.integration.ScalableTSDFVolume(
                voxel_length=.015, sdf_trunc=.045, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        volume.integrate(rgbd_color, intrinsic, np.linalg.inv(pose))
        current.append(row)
        trajectory.append({'seconds': row['seconds'], 'color': row['color'], 'fragment': len(segments),
                           'camera_to_fragment': pose.tolist(), 'tracking': reason, **stats})
        last = dict(row=row, depth=depth, keypoints=keypoints, descriptors=descriptors, rgbd=rgbd, cloud=cloud)
        if idx % 50 == 0:
            print(f'경로 분석 {row["seconds"]:.1f}/{data["duration_seconds"]:.1f}초', flush=True)
    finish()
    report = {'schema_version': 1, 'status': 'partial_reconstruction' if segments else 'no_reconstruction',
              'algorithm': 'SIFT/PnP initialized Open3D hybrid RGB-D odometry + per-fragment TSDF',
              'loop_closure': False, 'global_map_valid': False, 'robot_navigation_valid': False,
              'coordinate_frame': 'each fragment has an independent first-camera frame; metres',
              'source_duration_seconds': data['duration_seconds'], 'segments': segments,
              'events': events, 'trajectory': trajectory,
              'limitations': ['No global relocalization or loop closure', 'No ground truth metric accuracy validation',
                              'No inferred geometry behind surfaces; not a traversability map']}
    save(output / 'reconstruction.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    extract = sub.add_parser('extract')
    extract.add_argument('bag', type=Path)
    extract.add_argument('output', type=Path)
    extract.add_argument('--fps', type=float, default=10)
    extract.add_argument('--realtime', action='store_true', help='Replay at recorded rate without per-frame pause/resume; useful on Jetson. Export gaps are reported.')
    rec = sub.add_parser('reconstruct')
    rec.add_argument('dataset', type=Path)
    rec.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.command == 'extract':
        if not 5 <= args.fps <= 30:
            parser.error('--fps must be between 5 and 30')
        export_bag(args.bag, args.output, args.fps, args.realtime)
    else:
        reconstruct(args.dataset, args.output)


if __name__ == '__main__':
    main()
