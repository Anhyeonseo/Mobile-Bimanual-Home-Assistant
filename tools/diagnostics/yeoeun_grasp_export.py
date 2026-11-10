"""Read a colocated/copyable pair of Yeoeun tabletop reports, including GraspNet rows."""
import hashlib
import json

import numpy as np
from scipy.io import loadmat

from home_perception.grasp.graspnet import BASELINE_REVISION, CHECKPOINT_SHA256
from home_perception.grasp.handoff import CONVENTION


def read(path):
    return json.loads(path.read_text(encoding='utf8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def unique(items, **fields):
    matches = [item for item in items if all(item.get(k) == v for k, v in fields.items())]
    if len(matches) != 1:
        raise ValueError(f'expected a unique Yeoeun export entry for {fields}')
    return matches[0]


def original_observations(root, capture):
    report = read(root / 'report.json')
    if (report['mode'] != 'offline_perception_smoke' or report['status'] != 'COMPLETED'
            or report['robot_transform'] is not None or report['hardware_commands'] != 0):
        raise ValueError('completed Yeoeun tabletop perception run required')
    frame = unique(report['captures'], capture=capture)
    observations = read(root / capture / 'observations.json')['observations']
    if observations != frame['observations']:
        raise ValueError('perception summary and per-capture observations disagree')
    return observations


def refined_group(root, perception_root, capture, index, observation, source, meta, *, candidate_source='graspnet'):
    """Join the original observation, indexed SAM mask, prepared RGB-D and final 17-column rows."""
    report = read(root / 'report.json')
    if (report['mode'] != 'offline_grasp_candidates' or report['status'] not in ('COMPLETED', 'PARTIAL')
            or report['hardware_commands'] != 0 or report['robot_transform'] is not None):
        raise ValueError('offline Yeoeun grasp export required')
    entry = unique(report['objects'], capture=capture, index=index)
    if entry['exit_code'] != 0:
        raise ValueError('selected GraspNet inference did not complete')
    frame = root / capture
    target_name = f'mask_{index:03}.png'
    objects = read(frame / 'objects/objects.json')
    record = unique(objects['objects'], index=index)
    folder = frame / f'grasps_{index:03}'
    inference = read(folder / 'grasps.json')
    if (inference['target'] != record or inference['coordinate_frame'] != observation['frame_id']
            or objects['coordinate_frame'] != inference['coordinate_frame']
            or record['status'] != 'OK' or record['perception_status'] != 'accepted'
            or any(entry.get(k) != v for k, v in record.items())
            or record['mask'] != target_name or record['label'] != observation['label']
            or inference['checkpoint_sha256'] != report['checkpoint_sha256']):
        raise ValueError('selected grasp/observation identity mismatch')
    mask_hash = digest(perception_root / capture / target_name)
    if mask_hash != digest(frame / 'objects' / target_name) or observation.get('mask_sha256', mask_hash) != mask_hash:
        raise ValueError('observation and GraspNet masks differ')
    sample = frame / 'sample'
    expected = objects['input_sha256']
    if set(expected) != {'color.png', 'depth.png', 'meta.mat', 'workspace_mask.png'} or any(
            digest(sample / name) != sha for name, sha in expected.items()):
        raise ValueError('GraspNet input digest mismatch')
    if expected['color.png'] != source['color.png'] or expected['depth.png'] != source['depth_aligned.png']:
        raise ValueError('GraspNet used different RGB-D')
    sample_meta = loadmat(sample / 'meta.mat')
    intr = meta['aligned_depth_intrinsics']
    camera = [[intr['fx'], 0., intr['ppx']], [0., intr['fy'], intr['ppy']], [0., 0., 1.]]
    if (not np.array_equal(sample_meta['intrinsic_matrix'], camera)
            or not np.array_equal(objects['intrinsic_matrix'], camera)
            or sample_meta['factor_depth'].size != 1
            or not np.isclose(sample_meta['factor_depth'].item(), 1/meta['depth_scale_m'])):
        raise ValueError('GraspNet used different camera intrinsics or depth scale')
    if observation['source_sha256'] != source or observation['capture_id'] != capture:
        raise ValueError('perception used different snapshot metadata')
    candidates = entry['refined_grasps']
    if not isinstance(candidates, list) or len(candidates) > 64 or entry['refined'] != len(candidates):
        raise ValueError('invalid refined candidate count')
    rows = np.load(folder / 'grasps_refined.npy', allow_pickle=False)
    if rows.shape != (len(candidates), 17) or not np.isfinite(rows).all():
        raise ValueError('invalid GraspNet array')
    for row, c in zip(rows, candidates, strict=True):
        if c['source'] not in ('rule', 'graspnet'):
            raise ValueError('unknown upstream candidate source')
        fields = [c['score'], c['width_m'], c['depth_m'], *np.asarray(c['rotation_matrix']).ravel(), *c['translation_m']]
        if not np.array_equal(row[np.r_[0, 1, 3:16]], fields):
            raise ValueError('refined JSON and NPY candidate disagree')
    if candidate_source == 'graspnet':
        if (inference.get('baseline_revision') != BASELINE_REVISION or inference.get('checkpoint_sha256') != CHECKPOINT_SHA256
                or inference.get('model') != 'official GraspNet baseline (RealSense)'):
            raise ValueError('expected the RealSense GraspNet model and checkpoint')
        filtered = np.load(folder / 'grasps_filtered.npy', allow_pickle=False)
        if filtered.ndim != 2 or filtered.shape[1] != 17 or not np.isfinite(filtered).all():
            raise ValueError('invalid collision-filtered network output')
        for row, candidate in zip(rows, candidates, strict=True):
            if candidate['source'] == 'graspnet' and not (filtered == row).all(axis=1).any():
                raise ValueError('refined candidate missing from collision-filtered network output')
        candidates = [c for c in candidates if c['source'] == 'graspnet']
        reason = 'graspnet_candidates' if candidates else 'no_refined_graspnet_candidate'
    elif candidate_source == 'flat-rule':
        candidates = [c for c in candidates if c['source'] == 'rule']
        reason = entry['flat_rule']
    else:
        raise ValueError('unknown candidate source')
    return dict(tcp_convention=CONVENTION, mask_sha256=mask_hash,
                checkpoint_sha256=report['checkpoint_sha256'], candidate_reason=reason,
                refined_grasps=candidates)
