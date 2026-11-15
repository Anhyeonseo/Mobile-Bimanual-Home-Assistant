"""Join immutable recording chunks for offline review, never for live control.

Faulted sessions retain their failure. All-sensor mode tracks their observations
with lidar; valid wheel feedback supplies optional initialization and ray deskew.
"""
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .base_motion import OmniKinematics, Wheel
from .s4_base_preparation import _review_samples
from .s4_recorded_mapping import align, interpolate, pose, scan_points, transform


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def acknowledged_epochs(records):
    """Return source-backed continuous samples, without inventing a final stop."""
    if not records or records[0].get('event') != 'session':
        raise ValueError('session header required')
    firmware = records[0].get('firmware', {})
    if firmware.get('firmware') != 'mobile-base-hold-teleop-v7' or firmware.get('wheel_feedback') is not True:
        raise ValueError('identified feedback firmware required')
    epochs, current, rejected = [], [], []
    pending = None
    active = None
    last_stamp = -1

    def flush():
        nonlocal current
        if current:
            epochs.append(current)
            current = []

    for line, row in enumerate(records, 1):
        stamp = row.get('host_monotonic_ns')
        if type(stamp) is not int or stamp < last_stamp:
            raise ValueError('source timestamp regression')
        last_stamp = stamp
        event = row.get('event')
        if event == 'armed':
            flush()
            reply = row['reply']
            active = reply.get('session') if (reply.get('armed') is True and
                reply.get('fault') is False and reply.get('hold_confirmed') is True) else None
        elif event == 'command':
            if pending is not None:
                flush()
                rejected.append(dict(line=line, reason='missing previous reply'))
            pending = (line, row)
        elif event == 'reply':
            reply = row.get('reply', {})
            command = pending[1] if pending else {}
            if not (pending and active is not None and reply.get('type') == 'ack'
                    and command.get('session') == active == reply.get('session')
                    and command.get('sequence') == reply.get('sequence')
                    and reply.get('armed') is True and reply.get('fault') is False
                    and reply.get('hold_confirmed') is True):
                flush()
                rejected.append(dict(line=line, reason='unpaired or rejected feedback'))
                pending = None
                continue
            sample = dict(sequence=reply['sequence'],
                host_started_s=command['host_monotonic_ns']/1e9,
                host_received_s=stamp/1e9,
                source_lines=[pending[0], line],
                wheels=[dict(id=w['id'], ok=True, hal_status=0, position_raw=w['position_raw'],
                    speed_raw=w['speed_raw'], started_tick_ms=w['tick_begin_ms'],
                    ended_tick_ms=w['tick_end_ms']) for w in reply.get('wheel_feedback', [])])
            pending = None
            try:
                _review_samples([sample])
            except (ValueError, KeyError) as error:
                flush()
                rejected.append(dict(line=line, reason=str(error)))
                continue
            if current:
                try:
                    _review_samples([current[-1], sample])
                except ValueError as error:
                    flush()
                    rejected.append(dict(line=line, reason='interval split: '+str(error)))
            current.append(sample)
        elif event in ('paused', 'finished'):
            flush()
            pending = None
            active = None
    flush()
    return epochs, rejected


def sample_odometry(samples, geometry):
    _review_samples(samples)
    model = OmniKinematics(tuple(Wheel(**{k: w[k] for k in
        ('x_m', 'y_m', 'rolling_angle_rad', 'radius_m')}) for w in geometry['wheels']))
    times = [(s['host_started_s']+s['host_received_s'])/2 for s in samples]
    positions = np.array([[w['position_raw'] for w in s['wheels']] for s in samples])
    deltas = (np.diff(positions, axis=0)+2048) % 4096-2048
    current = np.eye(3)
    rows = [[times[0], 0., 0., 0.]]
    for at, delta in zip(times[1:], deltas):
        dx, dy, da = model.body_twist((delta*math.tau/4096).tolist())
        a, b = (1., da/2) if abs(da) < 1e-9 else (math.sin(da)/da, (1-math.cos(da))/da)
        current = current @ transform((a*dx-b*dy, b*dx+a*dy, da))
        rows.append([at, *pose(current)])
    rows = np.array(rows)
    rows[:, 3] = np.unwrap(rows[:, 3])
    return rows


def join_boundary(source, target):
    """A near-stationary restart hypothesis must fit both scans unambiguously."""
    solutions = []
    target_tree, source_tree = cKDTree(target), cKDTree(source)
    for yaw in (-.25, 0., .25):
        for x, y in ((0., 0.), (.15, 0.), (-.15, 0.), (0., .15), (0., -.15)):
            try:
                matrix, quality = align(source, target, transform((x, y, yaw)))
            except ValueError:
                continue
            p = pose(matrix)
            if np.linalg.norm(p[:2]) > .45 or abs(p[2]) > math.radians(30):
                continue
            forward = target_tree.query(source@matrix[:2, :2].T+matrix[:2, 2])[0]
            inverse = np.linalg.inv(matrix)
            backward = source_tree.query(target@inverse[:2, :2].T+inverse[:2, 2])[0]
            distances = np.r_[forward, backward]
            solutions.append((float(np.minimum(distances, .2).mean()), matrix,
                dict(quality, within_10cm_fraction=float(np.mean(distances <= .1)),
                     p90_m=float(np.percentile(distances, 90)))))
    if not solutions:
        raise ValueError('no bounded lidar boundary match')
    solutions.sort(key=lambda s: s[0])
    score, matrix, quality = solutions[0]
    alternatives = [s for s in solutions[1:] if
        np.linalg.norm(pose(np.linalg.inv(matrix)@s[1])[:2]) > .10 or
        abs(pose(np.linalg.inv(matrix)@s[1])[2]) > math.radians(5)]
    gap = min((s[0]-score for s in alternatives), default=None)
    if quality['rmse_m'] > .06 or quality['within_10cm_fraction'] < .75 or (gap is not None and gap < .015):
        raise ValueError('ambiguous or inconsistent lidar boundary: '+json.dumps(dict(quality, alternative_gap_m=gap)))
    return matrix, dict(quality, clipped_symmetric_mean_m=score, alternative_gap_m=gap,
                        transform_new_base_into_previous_base=pose(matrix).tolist())


def sensor_odometry(scans, feedback, mount):
    """Lidar-first tracking of every observation, even without wheel feedback."""
    rows, qualities = [], []
    current = np.eye(3)
    previous = previous_scan = previous_feedback = None
    for scan in scans:
        if (scan.get('frame') != 'lidar_a' or scan.get('count') != 360 or len(scan['ranges']) != 360
                or not math.isfinite(scan['observed_s']) or not -.001 < scan['time_increment'] < 0):
            raise ValueError('invalid source scan geometry')
        at = scan['observed_s']
        track = next((t for t in feedback if t[0, 0] <= at+359*scan['time_increment'] and at <= t[-1, 0]), None)
        points = scan_points(scan, mount, track)
        if previous is not None:
            dt = at-previous_scan['observed_s']
            if dt <= 0:raise ValueError('source scan time regression')
            if dt > .35:
                delta, quality = join_boundary(points, previous)
                quality['gap_s'] = dt
            else:
                seeds = [np.eye(3)]
                if track is not None and track is previous_feedback:
                    poses = interpolate(track, [previous_scan['observed_s'], at])
                    seeds.append(np.linalg.inv(transform(poses[0]))@transform(poses[1]))
                candidates = []
                tree = cKDTree(previous)
                for seed in seeds:
                    try:
                        delta, quality = align(points, previous, seed)
                        distances = tree.query(points@delta[:2, :2].T+delta[:2, 2])[0]
                        quality['within_10cm_fraction'] = float(np.mean(distances <= .1))
                        candidates.append((float(np.minimum(distances, .2).mean()), delta, quality))
                    except ValueError:pass
                if not candidates:raise ValueError('lidar tracking unavailable at '+str(at))
                _, delta, quality = min(candidates, key=lambda c: c[0])
                if (quality['rmse_m'] > .06 or quality['within_10cm_fraction'] < .70
                        or np.linalg.norm(pose(delta)[:2]) > .15 or abs(pose(delta)[2]) > .20):
                    raise ValueError('unresolved lidar step at '+str(at)+': '+json.dumps(quality))
            current = current@delta
            qualities.append(dict(observed_s=at, encoder_covered=track is not None, **quality))
        rows.append([at, *pose(current)])
        previous, previous_scan, previous_feedback = points, scan, track
    if len(rows) < 3:raise ValueError('insufficient recorded scans')
    # No prior motion observation exists for the first scan's negative-time rays.
    # Hold its initial transform for that one scan; this is not encoder evidence.
    rows.insert(0, [scans[0]['observed_s']+359*scans[0]['time_increment'], *rows[0][1:]])
    result = np.array(rows)
    result[:, 3] = np.unwrap(result[:, 3])
    return result, qualities


def prepare_campaign(root, segment_paths, output, *, all_sensor_observations=False):
    root, output = Path(root), Path(output)
    if output.exists():
        raise ValueError('new output directory required')
    output.mkdir(parents=True)
    geometry_file = root/'config/home_assistant/s4_base.nominal.json'
    mount_file = root/'config/home_assistant/lidar_mount.review.json'
    geometry = json.loads(geometry_file.read_text())
    mount_data = json.loads(mount_file.read_text())
    mount = transform([*mount_data['base_from_lidar_translation_m'][:2], mount_data['yaw_rad']])
    report = dict(mode='offline_campaign_preparation', passed=False, hardware_commands=0,
        navigation_authorized=False, physical_map_verified=False, sources=[], epochs=[], boundaries=[],
        all_sensor_observations=all_sensor_observations,
        trajectory_basis="lidar_scan_matching" if all_sensor_observations else "bounded_wheel_feedback",
        source_sha256={str(f): digest(f) for f in (geometry_file, mount_file)})
    tracks, scans_out, frame_rows = [], [], []
    previous = None
    source_capture = metadata = None
    try:
        for segment in map(Path, segment_paths):
            logs = list((segment/'teleop').glob('*.jsonl'))
            if len(logs) != 1:
                raise ValueError('one immutable teleop log required per segment')
            capture = segment/'capture'
            records = [json.loads(line) for line in logs[0].read_text().splitlines()]
            sensor = json.loads((capture/'report.json').read_text())
            files = [logs[0], capture/'report.json', capture/'scans.json', capture/'rgbd/metadata.json', capture/'rgbd/frames.jsonl']
            report['source_sha256'].update({str(f): digest(f) for f in files})
            epochs, rejected = acknowledged_epochs(records)
            source = dict(segment=segment.name, original_final_event=records[-1],
                original_capture_passed=sensor.get('passed'), rejected_feedback=rejected,
                acknowledged_epochs=len(epochs), used_epochs=0)
            report['sources'].append(source)
            if not epochs and not all_sensor_observations:
                source['excluded_reason'] = 'no acknowledged drive interval; archive retained'
                continue
            if sensor.get('mode') != 'real_sensor' or sensor.get('teleop_finished') != records[-1]:
                raise ValueError('successful matching sensor capture required: '+segment.name)
            if not all_sensor_observations and sensor.get('passed') is not True:
                raise ValueError('successful sensor capture required')
            if (sensor.get('error') or sensor.get('camera_cleanup_error')
                    or sensor.get('receipt_timing', {}).get('passed') is not True
                    or not sensor.get('tf_lookup_checks')
                    or any(c['maximum_matrix_error'] >= 1e-7 for c in sensor['tf_lookup_checks'])):
                raise ValueError('unqualified sensor timing or transform')
            camera_metadata = json.loads((capture/'rgbd/metadata.json').read_text())
            if metadata is None:
                metadata, source_capture = camera_metadata, sensor
            elif any(camera_metadata[k] != metadata[k] for k in ('native_depth_intrinsics', 'color_intrinsics', 'depth_scale_m', 'depth_to_color')):
                raise ValueError('camera calibration changed between chunks')
            if sensor['sensor_frames']['transforms'] != source_capture['sensor_frames']['transforms']:
                raise ValueError('sensor mount changed between chunks')
            scans = json.loads((capture/'scans.json').read_text())
            manifest = [json.loads(line) for line in (capture/'rgbd/frames.jsonl').read_text().splitlines()]
            work = []
            if all_sensor_observations:
                feedback = [sample_odometry(e, geometry) for e in epochs if len(e) >= 3]
                odom, quality = sensor_odometry(scans, feedback, mount)
                source['lidar_tracking'] = dict(scans=len(scans),
                    median_rmse_m=float(np.median([q['rmse_m'] for q in quality])),
                    maximum_rmse_m=max(q['rmse_m'] for q in quality),
                    scans_without_encoder_coverage=sum(not q['encoder_covered'] for q in quality),
                    gap_registrations=[q for q in quality if 'gap_s' in q])
                (output/(segment.name+'-scan-tracking.json')).write_text(json.dumps(quality))
                work.append((odom, scans, [1, len(records)], sum(len(e) for e in epochs)))
            for samples in ([] if all_sensor_observations else epochs):
                if len(samples) < 3:
                    continue
                odom = sample_odometry(samples, geometry)
                selected = [s for s in scans if s['observed_s']+359*s['time_increment'] >= odom[0, 0]
                            and s['observed_s'] <= odom[-1, 0]]
                if len(selected) < 3:
                    continue
                work.append((odom, selected, [samples[0]['source_lines'][0], samples[-1]['source_lines'][1]], len(samples)))
            for index, (odom, selected, lines, sample_count) in enumerate(work):
                if any(s.get('frame') != 'lidar_a' or s.get('count') != 360 or len(s['ranges']) != 360
                       or not -.001 < s['time_increment'] < 0 for s in selected):
                    raise ValueError('invalid scan geometry')
                gaps = np.diff([s['observed_s'] for s in selected])
                if not all_sensor_observations and not np.all((gaps > 0) & (gaps <= .35)):
                    raise ValueError('scan gap in selected epoch')
                first = scan_points(selected[0], mount, None if all_sensor_observations else odom)
                if previous is None:
                    start_pose, start_time = np.eye(3), 1.
                else:
                    delta, quality = join_boundary(first, previous['points'])
                    report['boundaries'].append(dict(previous=previous['id'], current=f'{segment.name}:{index}', **quality))
                    start_pose = previous['pose']@delta
                    start_time = tracks[-1][-1, 0]+1.
                origin = transform(interpolate(odom, [selected[0]['observed_s']])[0])
                world = start_pose@np.linalg.inv(origin)
                transformed = np.array([[r[0], *pose(world@transform(r[1:]))] for r in odom])
                transformed[:, 3] = np.unwrap(transformed[:, 3])
                offset = start_time-odom[0, 0]
                transformed[:, 0] += offset
                tracks.append(transformed)
                epoch = dict(id=f'{segment.name}:{index}', segment=segment.name,
                    source_interval_s=odom[[0, -1], 0].tolist(), replay_interval_s=transformed[[0, -1], 0].tolist(),
                    time_offset_s=float(offset), source_lines=lines,
                    acknowledged_samples=sample_count, scans=len(selected))
                report['epochs'].append(epoch)
                source['used_epochs'] += 1
                for scan in selected:
                    scans_out.append(dict(scan, observed_s=scan['observed_s']+offset,
                        source_observed_s=scan['observed_s'], source_segment=segment.name, source_epoch=epoch['id']))
                for row in manifest:
                    covered = all(odom[0, 0] <= row[k] <= odom[-1, 0] for k in ('color_time_s', 'depth_time_s'))
                    if covered or all_sensor_observations:
                        frame_rows.append(dict(row, file=segment.name+'-'+row['file'],
                            color_time_s=row['color_time_s']+offset, depth_time_s=row['depth_time_s']+offset,
                            source_color_time_s=row['color_time_s'], source_depth_time_s=row['depth_time_s'],
                            source_path=str((capture/'rgbd'/row['file']).resolve()), source_epoch=epoch['id'], pose_covered=covered))
                previous = dict(id=epoch['id'], points=scan_points(selected[-1], mount, None if all_sensor_observations else odom),
                    pose=world@transform(interpolate(odom, [selected[-1]['observed_s']])[0]))
        if not tracks:
            raise ValueError('no usable campaign intervals')
        joined = np.vstack(tracks)
        joined[:, 3] = np.unwrap(joined[:, 3])
        np.savez_compressed(output/'trajectories.npz', encoder=joined)
        (output/'scans.json').write_text(json.dumps(scans_out))
        rgbd = output/'rgbd'
        rgbd.mkdir()
        (rgbd/'metadata.json').write_text(json.dumps(metadata, indent=2))
        for frame in frame_rows:
            original = Path(frame['source_path'])
            if digest(original) != frame['sha256']:
                raise ValueError('RGB-D original hash mismatch: '+str(original))
            os.link(original, rgbd/frame['file'])
        (rgbd/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in frame_rows))
        report.update(passed=True, scans=len(scans_out), rgbd_frames=len(frame_rows),
            rgbd_frames_with_pose=sum(r['pose_covered'] for r in frame_rows),
            sensor_frames=source_capture['sensor_frames'], scan_height_above_floor_m=source_capture['scan_height_above_floor_m'],
            path_length_m=float(sum(np.linalg.norm(np.diff(t[:, 1:3], axis=0), axis=1).sum() for t in tracks)),
            final_pose=joined[-1, 1:].tolist(),
            artifact_sha256={name: digest(output/name) for name in ('trajectories.npz', 'scans.json', 'rgbd/frames.jsonl', 'rgbd/metadata.json')},
            limitations=['faulted source sessions remain faulted; raw sensor observations retained',
                'restart transforms are local lidar matches, not surveyed poses or global relocalization',
                'nominal wheel feedback initializes scan matching only where valid; physical scale remains unverified',
                'gaps compressed in offline timeline; no observations invented across gaps'])
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def mask_body_shadow(scan, visibility):
    """Keep angular indices/timing intact; hidden rays are unknown, never free."""
    if (scan.get('frame') != visibility['frame'] or scan.get('count') != 360
            or len(scan['ranges']) != 360 or abs(scan['angle_min']) > 1e-7
            or abs(scan['angle_increment']-math.pi/180) > 1e-8):
        raise ValueError('visibility mask does not match scan geometry')
    ranges = list(scan['ranges'])
    removed = sum(ranges[i] is not None for i in visibility['masked_ray_indices'])
    for i in visibility['masked_ray_indices']:
        ranges[i] = None
    return dict(scan, ranges=ranges), removed


def surface_model(points):
    """Local line normals; corners and isolated points do not define a surface."""
    tree = cKDTree(points)
    distances, indices = tree.query(points, k=7)
    neighbors = points[indices]
    centered = neighbors-neighbors.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered)
    values, vectors = np.linalg.eigh(covariance)
    valid = (distances[:,-1] < .18) & (values[:,1] > 1e-6) & (values[:,0] < .12*values[:,1])
    return tree, vectors[:,:,0], valid


def align_surfaces(source, target, initial, model):
    """Robust point-to-line ICP avoids bias from sliding along sampled walls."""
    tree, normals, valid = model
    matrix = initial.copy()
    for _ in range(30):
        rotated = source@matrix[:2,:2].T
        points = rotated+matrix[:2,2]
        distances, indices = tree.query(points)
        keep = (distances < .15) & valid[indices]
        if keep.sum() < 40:
            return align(source,target,initial)
        n = normals[indices[keep]]
        residual = np.einsum('ij,ij->i',n,points[keep]-target[indices[keep]])
        jacobian = np.column_stack((n, -n[:,0]*rotated[keep,1]+n[:,1]*rotated[keep,0]))
        weight = np.sqrt(np.minimum(1., .025/np.maximum(abs(residual),1e-9)))
        weighted = jacobian*weight[:,None]
        delta, _, rank, _ = np.linalg.lstsq(weighted,-residual*weight,rcond=None)
        if rank < 3 or np.linalg.cond(weighted) > 100:
            return align(source,target,initial)
        matrix[:2,2] += delta[:2]
        matrix[:2,:2] = transform((0.,0.,delta[2]))[:2,:2]@matrix[:2,:2]
        if np.linalg.norm(delta) < 1e-6:
            break
    distances = tree.query(source@matrix[:2,:2].T+matrix[:2,2])[0]
    keep = distances <= min(.15,float(np.quantile(distances,.85)))
    return matrix,dict(rmse_m=float(np.sqrt(np.mean(distances[keep]**2))),
        overlap_fraction=float(keep.mean()),inliers=int(keep.sum()))


def lidar_submap_odometry(scans, mount, *, surface_matching=True):
    """Pure lidar local prior. Retain all scans, match to recent spatial keyframes.

    Wheel feedback, images and semantic labels are intentionally not inputs.
    Spatial keyframes avoid a stationary scan stream replacing all moving views.
    This prior is still subject to global SLAM corrections, not ground truth.
    """
    current = np.eye(3)
    previous = previous_scan = last_keyframe = None
    keyframes, rows, quality_rows = [], [], []
    target = model = None
    for index, scan in enumerate(scans):
        at = scan['observed_s']
        points = scan_points(scan, mount)
        quality = dict(rmse_m=0., within_10cm_fraction=1.)
        if previous is not None:
            dt = at-previous_scan['observed_s']
            if dt <= 0:
                raise ValueError('source scan time regression')
            seed = current
            if dt > .35 or scan['source_segment'] != previous_scan['source_segment']:
                delta, boundary = join_boundary(points, previous)
                seed = current@delta
                quality['boundary'] = boundary
                quality['gap_s'] = dt
            fitted, fit_quality = (align_surfaces(points,target,seed,model) if surface_matching
                                   else align(points,target,seed))
            distances = cKDTree(target).query(points@fitted[:2, :2].T+fitted[:2, 2])[0]
            quality.update(fit_quality, within_10cm_fraction=float(np.mean(distances <= .1)))
            step = pose(np.linalg.inv(current)@fitted)
            if (quality['rmse_m'] > .08 or quality['within_10cm_fraction'] < .5
                    or np.linalg.norm(step[:2]) > (.5 if dt > .35 else .2)
                    or abs(step[2]) > (.55 if dt > .35 else .25)):
                raise ValueError(f'unresolved local scan {index}: '+json.dumps(quality))
            current = fitted
        movement = pose(np.linalg.inv(last_keyframe)@current) if last_keyframe is not None else None
        if movement is None or np.linalg.norm(movement[:2]) >= .08 or abs(movement[2]) >= .08:
            keyframes.append(points@current[:2, :2].T+current[:2, 2])
            keyframes = keyframes[-30:]
            merged = np.concatenate(keyframes)
            _, unique = np.unique(np.floor(merged/.025).astype(int), axis=0, return_index=True)
            target = merged[unique]
            model = surface_model(target) if surface_matching else None
            last_keyframe = current.copy()
        rows.append([at, *pose(current)])
        quality_rows.append(dict(index=index, observed_s=at, source_segment=scan['source_segment'], **quality))
        previous, previous_scan = points, scan
        if index % 1000 == 0:
            print(json.dumps(dict(lidar_scans_processed=index+1, total=len(scans))), flush=True)
    rows.insert(0, [scans[0]['observed_s']+359*scans[0]['time_increment'], *rows[0][1:]])
    result = np.asarray(rows)
    result[:, 3] = np.unwrap(result[:, 3])
    return result, quality_rows


def prepare_lidar_campaign(root, segment_paths, output, *, surface_matching=False):
    """Independent lidar-only input: no teleop, RGB-D or semantic files read."""
    root, output = Path(root), Path(output)
    if output.exists():
        raise ValueError('new output directory required')
    output.mkdir(parents=True)
    mount_file = root/'config/home_assistant/lidar_mount.review.json'
    visibility_file = root/'config/home_assistant/lidar_visibility.review.json'
    mount_data = json.loads(mount_file.read_text())
    visibility = json.loads(visibility_file.read_text())
    if (not np.allclose(mount_data['base_from_lidar_translation_m'], visibility['base_from_lidar_translation_m'], atol=1e-9)
            or not math.isclose(mount_data['yaw_rad'], visibility['yaw_rad'], abs_tol=1e-9)):
        raise ValueError('visibility mask mount mismatch')
    mount = transform([*mount_data['base_from_lidar_translation_m'][:2], mount_data['yaw_rad']])
    report = dict(mode='offline_campaign_preparation', passed=False, hardware_commands=0,
        navigation_authorized=False, physical_map_verified=False, lidar_only=True,
        wheel_feedback_used=False, rgbd_used=False, semantic_used=False, rgbd_frames=0,
        trajectory_basis='lidar_local_surface_submap_only' if surface_matching else 'lidar_local_submap_only',
        all_sensor_observations=True,
        visibility=visibility, sources=[], epochs=[], boundaries=[],
        source_sha256={str(p): digest(p) for p in (mount_file, visibility_file)})
    scans_out = []
    try:
        for segment in map(Path, segment_paths):
            path = segment/'capture/scans.json'
            scans = json.loads(path.read_text())
            times = np.array([s['observed_s'] for s in scans])
            if len(scans) < 3 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
                raise ValueError('invalid raw scan timeline: '+str(path))
            if any(not -.001 < s['time_increment'] < 0 for s in scans):
                raise ValueError('invalid signed ray timing')
            start = scans_out[-1]['observed_s']+1. if scans_out else 1.-359*scans[0]['time_increment']
            offset = start-times[0]
            removed = 0
            for scan in scans:
                masked, count = mask_body_shadow(scan, visibility)
                removed += count
                scans_out.append(dict(masked, observed_s=scan['observed_s']+offset,
                    source_observed_s=scan['observed_s'], source_segment=segment.name, source_epoch=segment.name+':0'))
            report['source_sha256'][str(path)] = digest(path)
            report['sources'].append(dict(segment=segment.name, scans=len(scans), all_scans_retained=True,
                masked_finite_returns=removed, used_epochs=1))
            report['epochs'].append(dict(id=segment.name+':0', segment=segment.name, scans=len(scans),
                source_interval_s=times[[0,-1]].tolist(), replay_interval_s=[float(start),float(times[-1]+offset)],
                time_offset_s=float(offset)))
        odom, quality = lidar_submap_odometry(scans_out, mount, surface_matching=surface_matching)
        report['epochs'][0]['replay_interval_s'][0] = float(odom[0,0])
        report['epochs'][0]['source_interval_s'][0] = float(odom[0,0]-report['epochs'][0]['time_offset_s'])
        report['boundaries'] = [q for q in quality if 'boundary' in q]
        np.savez_compressed(output/'trajectories.npz', encoder=odom)
        (output/'scans.json').write_text(json.dumps(scans_out))
        (output/'scan-tracking.json').write_text(json.dumps(quality))
        report.update(passed=True, scans=len(scans_out),
            path_length_m=float(np.linalg.norm(np.diff(odom[:,1:3],axis=0),axis=1).sum()), final_pose=odom[-1,1:].tolist(),
            artifact_sha256={name:digest(output/name) for name in ('trajectories.npz','scans.json','scan-tracking.json')},
            limitations=['Only visible lidar rays are matched; masked/missing rays remain unknown.',
                'All recorded scans retained, including interrupted sessions; unrecorded gaps are not fabricated.',
                'Local lidar submap prior may drift; final SLAM and boundary review required.',
                'Nominal mounting transform, no surveyed pose accuracy.'])
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def retime_rgbd_frame(row, segment, epoch, source_interval):
    """Use original within-boot exposure time, never another prepared timeline."""
    offset = epoch['time_offset_s']
    covered = all(source_interval[0] <= row[k] <= source_interval[1]
                  for k in ('color_time_s','depth_time_s'))
    return dict(row, file=segment+'-'+row['file'], source_epoch=epoch['id'],
        source_color_time_s=row['color_time_s'], source_depth_time_s=row['depth_time_s'],
        color_time_s=row['color_time_s']+offset, depth_time_s=row['depth_time_s']+offset,
        pose_covered=covered)


def attach_rgbd_campaign(prepared, segment_paths, output):
    """Attach immutable original RGB-D to a completed lidar-only time axis."""
    prepared, output = Path(prepared).resolve(), Path(output).resolve()
    _, scans, parent = load_prepared(prepared)
    if not parent.get('lidar_only') or output.exists():
        raise ValueError('lidar-only parent and new attachment directory required')
    segments = {Path(p).name:Path(p).resolve() for p in segment_paths}
    if set(segments) != {e['segment'] for e in parent['epochs']}:
        raise ValueError('RGB-D segments do not match lidar campaign')
    output.mkdir(parents=True);(output/'rgbd').mkdir()
    report = dict(mode='offline_lidar_rgbd_attachment',passed=False,hardware_commands=0,
        lidar_preparation=dict(path=str(prepared),report_sha256=digest(prepared/'report.json')),
        trajectory_basis=parent['trajectory_basis'],sources=[],source_sha256={},
        lidar_map_changed=False,source_archives_modified=False)
    frames=[];metadata=reference=None
    parent_sources={str(Path(p).resolve()):sha for p,sha in parent['source_sha256'].items()}
    try:
        for epoch in parent['epochs']:
            segment=segments[epoch['segment']];capture=segment/'capture'
            # Require this exact raw scan source to have produced the lidar parent.
            raw_scan=capture/'scans.json'
            if parent_sources.get(str(raw_scan)) != digest(raw_scan):
                raise ValueError('RGB-D attachment raw lidar source mismatch')
            sensor=json.loads((capture/'report.json').read_text())
            camera=json.loads((capture/'rgbd/metadata.json').read_text())
            if metadata is None:metadata,reference=camera,sensor
            elif (sensor['sensor_frames']['transforms']!=reference['sensor_frames']['transforms']
                    or any(camera[k]!=metadata[k] for k in ('native_depth_intrinsics','color_intrinsics','depth_scale_m','depth_to_color'))):
                raise ValueError('camera calibration or mounting changed between recordings')
            subset=[s for s in scans if s['source_segment']==segment.name]
            interval=[subset[0]['source_observed_s']+359*subset[0]['time_increment'],subset[-1]['source_observed_s']]
            manifest=[json.loads(line) for line in (capture/'rgbd/frames.jsonl').read_text().splitlines()]
            for row in manifest:
                original=(capture/'rgbd'/row['file']).resolve()
                if not original.is_relative_to(capture/'rgbd') or digest(original)!=row['sha256']:
                    raise ValueError('RGB-D original path/hash mismatch')
                timed=retime_rgbd_frame(row,segment.name,epoch,interval)
                timed['source_path']=str(original)
                os.link(original,output/'rgbd'/timed['file']);frames.append(timed)
            report['sources'].append(dict(segment=segment.name,rgbd_frames=len(manifest),
                source_capture_passed=sensor.get('passed'),source_lidar_interval_s=interval))
            report['source_sha256'].update({str(p):digest(p) for p in
                (capture/'report.json',capture/'rgbd/metadata.json',capture/'rgbd/frames.jsonl',raw_scan)})
        for name in ('scans.json','trajectories.npz'):os.link(prepared/name,output/name)
        (output/'rgbd/metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
        (output/'rgbd/frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in frames))
        report.update(passed=True,scans=len(scans),rgbd_frames=len(frames),
            rgbd_frames_with_pose=sum(r['pose_covered'] for r in frames),
            sensor_frames=reference['sensor_frames'],scan_height_above_floor_m=reference['scan_height_above_floor_m'],
            artifact_sha256={name:digest(output/name) for name in ('scans.json','trajectories.npz','rgbd/metadata.json','rgbd/frames.jsonl')},
            limitations=['Original RGB-D retained even outside lidar pose coverage; no cross-boot time matching.',
                         'Image attachment does not alter the saved lidar trajectory or map.'])
    except Exception as error:
        report['error']=str(error);raise
    finally:(output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def verify_rgbd_attachment(capture, slam_report, trajectory):
    """Verify the parent SLAM evidence instead of pretending it used RGB-D."""
    capture,trajectory=Path(capture),Path(trajectory)
    report=json.loads((capture/'report.json').read_text())
    if report.get('mode')!='offline_lidar_rgbd_attachment' or report.get('passed') is not True:
        raise ValueError('completed RGB-D attachment required')
    parent=Path(report['lidar_preparation']['path'])
    if digest(parent/'report.json')!=report['lidar_preparation']['report_sha256']:
        raise ValueError('lidar parent report changed')
    _,_,prepared=load_prepared(parent)
    sources={str(Path(p).resolve()):v for p,v in slam_report['source_sha256'].items()}
    for name in ('scans.json','report.json','trajectories.npz'):
        if sources.get(str((parent/name).resolve()))!=digest(parent/name):
            raise ValueError('map was built from a different lidar preparation')
    for name,sha in report['artifact_sha256'].items():
        if digest(capture/name)!=sha:raise ValueError('RGB-D attachment changed')
    for name in ('scans.json','trajectories.npz'):
        if digest(capture/name)!=digest(parent/name):raise ValueError('attachment changed lidar inputs')
    if digest(trajectory/'trajectories.npz')!=digest(parent/'trajectories.npz'):
        raise ValueError('RGB-D trajectory belongs to a different map')
    return prepared


def load_prepared(path):
    path = Path(path)
    report = json.loads((path/'report.json').read_text())
    if report.get('mode') != 'offline_campaign_preparation' or report.get('passed') is not True:
        raise ValueError('completed campaign preparation required')
    for name, expected in report['artifact_sha256'].items():
        if digest(path/name) != expected:
            raise ValueError('prepared campaign artifact changed')
    with np.load(path/'trajectories.npz', allow_pickle=False) as data:
        odom = data['encoder']
    scans = json.loads((path/'scans.json').read_text())
    if (odom.ndim != 2 or odom.shape[1] != 4 or len(odom) < 3
            or not np.isfinite(odom).all() or np.any(np.diff(odom[:, 0]) <= 0)):
        raise ValueError('invalid prepared trajectory')
    times = np.array([s['observed_s'] for s in scans])
    if (len(times) < 3 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0)
            or times[0] < odom[0, 0] or times[-1] > odom[-1, 0]):
        raise ValueError('invalid prepared scan timeline')
    return odom, scans, report
