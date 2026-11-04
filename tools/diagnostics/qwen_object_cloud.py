#!/usr/bin/env python3
"""Qwen detections -> SAM masks -> camera-frame object clouds for the demo."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('detections', type=Path, help='Qwen detections.json')
    p.add_argument('--sample', type=Path, default=Path('artifacts/datasets/graspnet-example'))
    p.add_argument('--output', type=Path, default=Path('tmp/qwen-object-cloud'))
    p.add_argument('--index', type=int, help='Only process this zero-based detection index')
    p.add_argument('--device', choices=['cuda', 'cpu'], default='cuda')
    p.add_argument('--view', action='store_true')
    args = p.parse_args()
    import cv2
    import numpy as np
    import open3d as o3d
    from PIL import Image
    from scipy.io import loadmat
    from tools.lib.sam_rgbd import SAMMasks, MODEL_ID, MODEL_REVISION

    data = json.loads(args.detections.read_text())
    rgb = Image.open(args.sample/'color.png').convert('RGB')
    # Qwen hashes its saved RGB PNG, not the original file's compression bytes.
    from io import BytesIO
    encoded = BytesIO()
    rgb.save(encoded, format='PNG')
    if hashlib.sha256(encoded.getvalue()).hexdigest() != data['source']['rgb_sha256']:
        raise ValueError('Detection RGB does not match this RGB-D sample')
    depth = np.asarray(Image.open(args.sample/'depth.png'))
    workspace = np.asarray(Image.open(args.sample/'workspace_mask.png')) > 0
    meta = loadmat(args.sample/'meta.mat')
    k = np.asarray(meta['intrinsic_matrix'], dtype=float)
    factor = float(np.asarray(meta['factor_depth']).item())
    if depth.shape != (rgb.height, rgb.width) or workspace.shape != depth.shape:
        raise ValueError('RGB-D dimensions mismatch')
    if not np.isfinite(factor) or factor <= 0 or k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0,0],k[1,1]) <= 0:
        raise ValueError('Invalid camera metadata')
    detections = data['detections']
    indices = list(range(len(detections))) if args.index is None else [args.index]
    if not indices or any(i < 0 or i >= len(detections) for i in indices):
        raise ValueError('No detections or invalid --index')
    boxes = [detections[i]['bbox_xyxy'] for i in indices]
    for b in boxes:
        if len(b) != 4 or not np.isfinite(b).all() or not (0 <= b[0] < b[2] <= rgb.width and 0 <= b[1] < b[3] <= rgb.height):
            raise ValueError(f'Invalid box {b}')
    arr = np.asarray(rgb)
    masks, seconds = SAMMasks(args.device).segment(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR), boxes)
    z = depth.astype(float)/factor
    u, v = np.meshgrid(np.arange(rgb.width), np.arange(rgb.height))
    xyz = np.stack([(u-k[0,2])*z/k[0,0], (v-k[1,2])*z/k[1,1], z], axis=-1)
    valid = depth > 0
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    def cloud(keep, name):
        c = o3d.geometry.PointCloud()
        c.points = o3d.utility.Vector3dVector(xyz[keep])
        c.colors = o3d.utility.Vector3dVector(arr[keep].astype(float)/255)
        if len(c.points) and not o3d.io.write_point_cloud(str(out/name), c):
            raise RuntimeError(f'Failed to save {name}')
        return c
    scene = cloud(valid, 'scene.ply')
    cloud(valid & workspace, 'workspace.ply')
    records, objects = [], []
    overlay = arr.copy()
    for i, (mask, score) in zip(indices, masks):
        Image.fromarray(mask.astype('uint8')*255).save(out/f'mask_{i:03}.png')
        keep = mask & valid & workspace
        count = int(keep.sum())
        accepted = score >= 0.5 and count >= 3
        record = {'index': i, 'label': detections[i]['label'], 'bbox_xyxy': detections[i]['bbox_xyxy'],
                  'sam_predicted_iou': score, 'mask_pixels': int(mask.sum()),
                  'valid_workspace_depth_points': count, 'status': 'OK' if accepted else 'REJECTED',
                  'mask': f'mask_{i:03}.png', 'cloud': None}
        if accepted:
            name = f'object_{i:03}.ply'
            objects.append(cloud(keep, name))
            record['cloud'] = name
        else:
            (out/f'object_{i:03}.ply').unlink(missing_ok=True)
        overlay[mask] = (0.5*overlay[mask]+0.5*np.array([0,255,0])).astype('uint8')
        records.append(record)
    Image.fromarray(overlay).save(out/'masks.jpg')
    report = {'detections_path': str(args.detections.resolve()),
              'detections_sha256': hashlib.sha256(args.detections.read_bytes()).hexdigest(),
              'sample': str(args.sample.resolve()),
              'input_sha256': {f:hashlib.sha256((args.sample/f).read_bytes()).hexdigest()
                               for f in ['color.png','depth.png','meta.mat','workspace_mask.png']},
              'sam_model': MODEL_ID, 'sam_revision': MODEL_REVISION, 'sam_seconds': seconds,
              'coordinate_frame': 'camera optical: X right, Y down, Z forward; meters',
              'depth_scale_m': 1/factor, 'intrinsic_matrix': k.tolist(),
              'point_filter': 'SAM mask AND raw depth > 0 AND official workspace ROI',
              'quality_threshold': 0.5, 'objects': records}
    (out/'objects.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if args.view and objects:
        # Display only; saved PLY files keep original colors and coordinates.
        scene.paint_uniform_color([0.5,0.5,0.5])
        for c in objects: c.paint_uniform_color([0,1,0])
        o3d.visualization.draw_geometries([scene, *objects], window_name='Scene gray / target green')


if __name__ == '__main__':
    main()
