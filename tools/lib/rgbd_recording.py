"""Portable aligned RGB-D recordings. Validation opens files, never devices.

RGB is uint8 HxWx3, depth is raw uint16 HxW; Z[m] = depth * depth_scale_m.
Optional camera_to_base transforms map camera optical coordinates into base_link.
Missing transforms remain unknown and never become an identity calibration.
"""
import math
from pathlib import Path
import re

import numpy as np

from tools.lib.run_manifest import sha256, write_json


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite numeric')
    return float(value)


def _identifier(value, name):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', value):
        raise ValueError(f'invalid {name}')
    return value


def _asset(root, descriptor):
    if not isinstance(descriptor, dict) or set(descriptor) != {'path', 'sha256'}:
        raise ValueError('asset requires path and sha256')
    relative = descriptor['path']
    if not isinstance(relative, str) or Path(relative).is_absolute():
        raise ValueError('recording paths must be relative')
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError('asset outside recording or missing')
    if sha256(path) != descriptor['sha256']:
        raise ValueError('asset checksum mismatch: ' + relative)
    return path


def _transform(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('camera_to_base must be a finite 4x4 matrix')
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5)):
        raise ValueError('camera_to_base must be a proper rigid transform')


def validate_recording(path):
    import json
    path = Path(path).resolve()
    root = path.parent
    data = json.loads(path.read_text(encoding='utf-8'))
    if type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise ValueError('recording schema_version must be 1')
    if data.get('evidence_source') not in ('synthetic', 'public_dataset', 'measured'):
        raise ValueError('recording evidence_source required')
    _identifier(data.get('recording_id'), 'recording_id')
    _identifier(data.get('clock_domain'), 'clock_domain')
    camera = data['camera']
    if camera.get('depth_alignment') != 'color':
        raise ValueError('depth must already be aligned to color')
    _identifier(camera.get('frame_id'), 'camera frame')
    _identifier(camera.get('calibration_id'), 'calibration_id')
    width, height = camera['width'], camera['height']
    if type(width) is not int or type(height) is not int or not 1 <= min(width, height) <= max(width, height) <= 4096:
        raise ValueError('invalid image dimensions')
    k = np.asarray(camera['intrinsic_matrix'], dtype=float)
    if (k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
            or not np.allclose(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0
            or not 0 <= k[0, 2] < width or not 0 <= k[1, 2] < height):
        raise ValueError('invalid rectified camera intrinsics')
    scale = _number(camera['depth_scale_m'], 'depth scale')
    if not 0 < scale <= .1:
        raise ValueError('invalid depth scale in meters')
    frames = data['frames']
    if not isinstance(frames, list) or not 1 <= len(frames) <= 10000:
        raise ValueError('recording requires 1..10000 frames')
    seen, previous, summaries = set(), -1., []
    for frame in frames:
        capture = _identifier(frame['capture_id'], 'capture_id')
        observed = _number(frame['observed_s'], 'observation time')
        if capture in seen or observed < 0 or observed <= previous:
            raise ValueError('capture IDs must be unique and timestamps strictly increasing')
        seen.add(capture)
        previous = observed
        rgb = np.load(_asset(root, frame['rgb']), allow_pickle=False, mmap_mode='r')
        depth = np.load(_asset(root, frame['depth']), allow_pickle=False, mmap_mode='r')
        if rgb.dtype != np.uint8 or rgb.shape != (height, width, 3):
            raise ValueError('RGB must be uint8 HxWx3')
        if depth.dtype != np.uint16 or depth.shape != (height, width):
            raise ValueError('depth must be raw uint16 HxW')
        if frame.get('workspace') is not None:
            mask = np.load(_asset(root, frame['workspace']), allow_pickle=False, mmap_mode='r')
            if mask.shape != depth.shape or mask.dtype not in (np.dtype('uint8'), np.dtype('bool')) or not np.isin(mask, [0, 1, 255]).all():
                raise ValueError('workspace must be a binary HxW mask')
        calibrated = frame.get('camera_to_base') is not None
        if calibrated:
            _transform(frame['camera_to_base'])
        valid = depth > 0
        summaries.append({'capture_id': capture, 'observed_s': observed,
                          'valid_depth_fraction': float(valid.mean()),
                          'median_valid_depth_m': float(np.median(depth[valid]) * scale) if valid.any() else None,
                          'camera_to_base_available': calibrated,
                          'workspace_available': frame.get('workspace') is not None})
    return {'schema_version': 1, 'valid': True, 'recording_id': data['recording_id'],
            'evidence_source': data['evidence_source'], 'frame_count': len(frames),
            'manifest_sha256': sha256(path), 'frames': summaries,
            'physical_task_completed': False}


def write_recording(output, rgb, depth, intrinsic_matrix, depth_scale_m, *,
                    evidence_source, recording_id, workspace=None, camera_to_base=None,
                    frame_id='camera_color_optical_frame', calibration_id='unmeasured', source=None):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    frame = {'capture_id': 'frame-000000', 'observed_s': 0., 'camera_to_base': camera_to_base}
    for name, array in (('rgb', rgb), ('depth', depth), ('workspace', workspace)):
        if array is not None:
            path = output / (name + '.npy')
            np.save(path, array, allow_pickle=False)
            frame[name] = {'path': path.name, 'sha256': sha256(path)}
    data = {'schema_version': 1, 'recording_id': recording_id, 'evidence_source': evidence_source,
            'clock_domain': 'recording_relative', 'source': source,
            'camera': {'frame_id': frame_id, 'calibration_id': calibration_id,
                       'width': int(rgb.shape[1]), 'height': int(rgb.shape[0]),
                       'intrinsic_matrix': np.asarray(intrinsic_matrix).tolist(),
                       'depth_scale_m': depth_scale_m, 'depth_alignment': 'color'},
            'frames': [frame]}
    write_json(output / 'manifest.json', data)
    validate_recording(output / 'manifest.json')
    return output / 'manifest.json'


def import_graspnet(sample, output):
    from PIL import Image
    from scipy.io import loadmat
    sample = Path(sample)
    meta = loadmat(sample / 'meta.mat')
    factor = float(np.asarray(meta['factor_depth']).item())
    if not math.isfinite(factor) or factor <= 0:
        raise ValueError('invalid factor_depth')
    files = ('color.png', 'depth.png', 'workspace_mask.png', 'meta.mat')
    return write_recording(output, np.asarray(Image.open(sample / 'color.png').convert('RGB')),
                           np.asarray(Image.open(sample / 'depth.png')).astype(np.uint16),
                           meta['intrinsic_matrix'], 1 / factor,
                           workspace=(np.asarray(Image.open(sample / 'workspace_mask.png')) > 0).astype(np.uint8),
                           evidence_source='public_dataset', recording_id='graspnet-example',
                           calibration_id='graspnet-example-intrinsics',
                           source={'kind': 'graspnet_example', 'input_sha256': {name: sha256(sample / name) for name in files}})


def export_graspnet(manifest, output, index=0):
    import json
    from PIL import Image
    from scipy.io import savemat
    validate_recording(manifest)
    manifest = Path(manifest).resolve()
    data = json.loads(manifest.read_text(encoding='utf-8'))
    if type(index) is not int or not 0 <= index < len(data['frames']):
        raise ValueError('frame index outside recording')
    frame = data['frames'][index]
    if frame.get('workspace') is None:
        raise ValueError('explicit workspace mask required for GraspNet export')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    for name, filename in (('rgb', 'color.png'), ('depth', 'depth.png'), ('workspace', 'workspace_mask.png')):
        array = np.load(_asset(manifest.parent, frame[name]), allow_pickle=False)
        if name == 'workspace':
            array = (array > 0).astype(np.uint8) * 255
        Image.fromarray(array).save(output / filename)
    savemat(output / 'meta.mat', {'intrinsic_matrix': data['camera']['intrinsic_matrix'],
                                'factor_depth': 1 / data['camera']['depth_scale_m']})
    write_json(output / 'recording_source.json', {'manifest_sha256': sha256(manifest),
               'capture_id': frame['capture_id'], 'evidence_source': data['evidence_source'],
               'camera_to_base': frame.get('camera_to_base')})
    return output
