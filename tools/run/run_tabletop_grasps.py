#!/usr/bin/env python3
"""GraspNet candidates for saved tabletop perception runs; no robot or downloads.

Reuses masks from run_tabletop_perception.py. Candidates use the baseline
parallel gripper in the camera frame: no SO101 fit, IK, reach or execution check.
Scores rank candidates and are not success probabilities. Objects held by
perception keep their candidates for inspection but are never marked executable.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from home_perception.geometry.plane import backproject, fit_plane
from home_perception.geometry.workspace import workspace_mask
from home_perception.grasp.refine import FLAT_OBJECT_HEIGHT_M, flat_object_grasp, refine
from home_perception.io.snapshot import load_snapshot
from tools.lib.run_manifest import provenance, sha256, write_json

SCRIPT = ROOT / 'tools/diagnostics/graspnet_candidates.py'


def intrinsic_matrix(intrinsics):
    return [[intrinsics['fx'], 0., intrinsics['ppx']], [0., intrinsics['fy'], intrinsics['ppy']], [0., 0., 1.]]


def write_cloud(path, snapshot, keep):
    import cv2
    import open3d as o3d
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(backproject(snapshot.depth_m(), snapshot.intrinsics, keep))
    colors = cv2.cvtColor(snapshot.bgr, cv2.COLOR_BGR2RGB)[keep].astype(float) / 255
    cloud.colors = o3d.utility.Vector3dVector(colors)
    if not o3d.io.write_point_cloud(str(path), cloud):
        raise RuntimeError(f'Failed to save {path}')


def prepare(snapshot, observations, masks_from, folder):
    """Write the sample and objects.json that graspnet_candidates.py reads."""
    import cv2
    import numpy as np
    from scipy.io import savemat
    depth = snapshot.depth_m()
    plane = fit_plane(backproject(depth, snapshot.intrinsics, depth > 0))
    masks = [cv2.imread(str(masks_from / f'mask_{i:03}.png'), cv2.IMREAD_UNCHANGED) > 0 for i in range(len(observations))]
    workspace = workspace_mask(depth, snapshot.intrinsics, plane, masks)
    sample, objects = folder / 'sample', folder / 'objects'
    sample.mkdir(parents=True)
    objects.mkdir()
    shutil.copyfile(snapshot.folder / 'color.png', sample / 'color.png')
    shutil.copyfile(snapshot.folder / 'depth_aligned.png', sample / 'depth.png')
    cv2.imwrite(str(sample / 'workspace_mask.png'), workspace.astype(np.uint8) * 255)
    k = intrinsic_matrix(snapshot.intrinsics)
    savemat(sample / 'meta.mat', {'intrinsic_matrix': k, 'factor_depth': 1 / snapshot.depth_scale_m})
    valid = depth > 0
    write_cloud(objects / 'scene.ply', snapshot, valid)
    write_cloud(objects / 'workspace.ply', snapshot, valid & workspace)
    records = []
    for index, (observation, mask) in enumerate(zip(observations, masks)):
        keep = mask & valid & workspace
        name = f'mask_{index:03}.png'
        shutil.copyfile(masks_from / name, objects / name)
        cloud = f'object_{index:03}.ply'
        write_cloud(objects / cloud, snapshot, keep)
        records.append({'index': index, 'label': observation['label'], 'status': 'OK', 'mask': name, 'cloud': cloud,
                        'perception_status': observation['status'], 'points': int(keep.sum())})
    write_json(objects / 'objects.json', {
        'sample': str(sample.resolve()), 'coordinate_frame': snapshot.frame_id, 'intrinsic_matrix': k,
        'input_sha256': {name: sha256(sample / name) for name in
                         ('color.png', 'depth.png', 'meta.mat', 'workspace_mask.png')},
        'objects': records})
    return objects, records, float(workspace.mean()), plane


def read_points(path):
    import numpy as np
    import open3d as o3d
    return np.asarray(o3d.io.read_point_cloud(str(path)).points)


def finish(folder, objects, record, observation, plane):
    """Apply geometry checks, add the flat-object rule, and save the final list."""
    import numpy as np
    target = read_points(objects / record['cloud'])
    height = observation['support_height_m'] or 0.
    kept, rejected = refine(np.load(folder / 'grasps_filtered.npy'), target, plane, height)
    rule, rule_reason = None, 'not_flat'
    if height <= FLAT_OBJECT_HEIGHT_M:
        rule, rule_reason = flat_object_grasp(target, read_points(objects / 'scene.ply'), plane)
    sources = ['rule'] * (rule is not None) + ['graspnet'] * len(kept)
    final = np.vstack(([rule] if rule is not None else []) + [kept]) if len(sources) else np.zeros((0, 17))
    np.save(folder / 'grasps_refined.npy', final)
    return {'refined': len(final), 'rejected_by_check': rejected, 'flat_rule': rule_reason or 'added',
            'executable': observation['status'] == 'accepted' and len(final) > 0,
            'refined_grasps': [{'source': source, 'score': float(row[0]), 'width_m': float(row[1]),
                                'depth_m': float(row[3]), 'rotation_matrix': row[4:13].reshape(3, 3).tolist(),
                                'translation_m': row[13:16].tolist()} for source, row in zip(sources, final)]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--perception', type=Path, default=ROOT / 'tmp/tabletop-perception-gpu-r2')
    parser.add_argument('--dataset', type=Path, default=ROOT / 'data/tabletop_rgbd_20261101')
    parser.add_argument('--checkpoint', type=Path, default=ROOT / 'artifacts/models/checkpoint-rs.tar')
    parser.add_argument('--output', type=Path, default=ROOT / 'tmp/tabletop-grasps')
    parser.add_argument('--focus-radius-m', type=float, default=.15)
    parser.add_argument('--collision-voxel-m', type=float, default=.005)
    parser.add_argument('--stage-timeout-s', type=int, default=180)
    args = parser.parse_args(argv)
    source = json.loads((args.perception / 'report.json').read_text(encoding='utf-8'))
    if source['status'] != 'COMPLETED':
        parser.error('perception run did not complete')
    output = args.output.resolve()
    try:
        output.mkdir(parents=True, exist_ok=False)
    except OSError as error:
        parser.exit(2, f'Cannot create new output directory: {error}\n')
    report = {'schema_version': 1, 'mode': 'offline_grasp_candidates', 'status': 'RUNNING', 'hardware_commands': 0,
              'physical_task_completed': False, 'robot_feasibility_checked': False, 'robot_transform': None,
              'perception': str(args.perception.resolve()), 'checkpoint_sha256': sha256(args.checkpoint),
              'objects': [], 'provenance': provenance()}
    started = time.monotonic()
    try:
        for capture in source['captures']:
            name, observations = capture['capture'], capture['observations']
            snapshot = load_snapshot(args.dataset / name)
            objects, records, fraction, plane = prepare(snapshot, observations, args.perception / name, output / name)
            for record in records:
                folder = output / name / f'grasps_{record["index"]:03}'
                entry = {'capture': name, 'workspace_fraction': fraction, **record}
                command = [sys.executable, str(SCRIPT), '--objects', str(objects), '--index', str(record['index']),
                           '--checkpoint', str(args.checkpoint.resolve()), '--output', str(folder),
                           '--focus-radius-m', str(args.focus_radius_m),
                           '--collision-voxel-m', str(args.collision_voxel_m)]
                log_path = output / name / f'grasps_{record["index"]:03}.log'
                with log_path.open('w', encoding='utf-8') as log:
                    completed = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                               timeout=args.stage_timeout_s)
                entry['exit_code'] = completed.returncode
                if completed.returncode == 0:
                    grasp = json.loads((folder / 'grasps.json').read_text(encoding='utf-8'))
                    entry.update({key: grasp[key] for key in ('generated', 'target_associated',
                                                              'collision_and_empty_filtered', 'saved',
                                                              'inference_seconds')})
                    entry['grasps'] = grasp['grasps']
                    entry.update(finish(folder, objects, record, observations[record['index']], plane))
                report['objects'].append(entry)
                write_json(output / 'report.json', report)
                print(name, record['label'], entry.get('refined', 'FAILED'), flush=True)
        report['status'] = 'COMPLETED' if all(item['exit_code'] == 0 for item in report['objects']) else 'PARTIAL'
    except KeyboardInterrupt:
        report.update(status='CANCELLED', error='interrupted_by_user')
    except Exception as error:
        report.update(status='FAILED', error=f'{type(error).__name__}: {error}')
    finally:
        report['wall_seconds'] = time.monotonic() - started
        write_json(output / 'report.json', report)
    print(json.dumps({'status': report['status'], 'report': str(output / 'report.json')}, ensure_ascii=False))
    return 130 if report['status'] == 'CANCELLED' else 1 if report['status'] == 'FAILED' else 0


if __name__ == '__main__':
    raise SystemExit(main())
