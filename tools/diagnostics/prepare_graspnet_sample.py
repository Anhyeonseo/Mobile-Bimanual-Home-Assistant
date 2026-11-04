#!/usr/bin/env python3
"""Prepare the official GraspNet demo RGB-D; optionally export/view its cloud."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

REVISION = '280c215129f759ed8649cb4e89fc5dfee55f4f80'
BASE = f'https://raw.githubusercontent.com/graspnet/graspnet-baseline/{REVISION}/doc/example_data/'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, default=Path('artifacts/datasets/graspnet-example'))
    p.add_argument('--pointcloud', action='store_true', help='Export full scene and workspace PLY files')
    p.add_argument('--view', action='store_true', help='Open the workspace cloud in Open3D')
    args = p.parse_args()
    import numpy as np
    from PIL import Image
    from scipy.io import loadmat

    root = args.output
    root.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for name in ['color.png', 'depth.png', 'workspace_mask.png', 'meta.mat']:
        path = root / name
        if not path.exists():
            print(f'Downloading {name}', flush=True)
            temp = path.with_suffix(path.suffix + '.download')
            with urllib.request.urlopen(BASE + name, timeout=60) as response:
                temp.write_bytes(response.read())
            temp.replace(path)
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    rgb = np.asarray(Image.open(root / 'color.png').convert('RGB'))
    depth = np.asarray(Image.open(root / 'depth.png'))
    mask = np.asarray(Image.open(root / 'workspace_mask.png'))
    meta = loadmat(root / 'meta.mat')
    k = np.asarray(meta['intrinsic_matrix'], dtype=float)
    factor = float(np.asarray(meta['factor_depth']).item())
    if (depth.ndim != 2 or depth.shape != rgb.shape[:2] or mask.shape != depth.shape
            or k.shape != (3, 3) or not np.isfinite(k).all()
            or not np.isfinite(factor) or factor <= 0 or k[0, 0] <= 0 or k[1, 1] <= 0):
        raise ValueError('Invalid RGB-D dimensions or camera metadata')
    z = depth.astype(np.float32) / factor
    np.save(root / 'depth_m.npy', z)
    valid = depth > 0
    source = {'repository': 'https://github.com/graspnet/graspnet-baseline',
              'revision': REVISION, 'source_url': BASE,
              'width': rgb.shape[1], 'height': rgb.shape[0],
              'intrinsic_matrix': k.tolist(), 'factor_depth': factor,
              'depth_scale_m': 1 / factor, 'depth_formula': 'Z_m = raw_depth / factor_depth',
              'coordinate_frame': 'camera optical: X right, Y down, Z forward; meters',
              'sha256': hashes, 'valid_depth_fraction': float(valid.mean()),
              'median_valid_depth_m': float(np.median(z[valid])),
              'workspace_mask': 'Scene workspace ROI, not an object segmentation mask'}
    (root / 'source.json').write_text(json.dumps(source, indent=2))
    v = np.clip(z / 1.5, 0, 1)
    preview = np.stack([255*v, 255*(1-abs(2*v-1)), 255*(1-v)], axis=-1).astype('uint8')
    preview[~valid] = 0
    Image.fromarray(preview).save(root / 'depth_preview.png')
    print(json.dumps(source, indent=2), flush=True)
    if args.pointcloud or args.view:
        import open3d as o3d
        u, v = np.meshgrid(np.arange(z.shape[1]), np.arange(z.shape[0]))
        xyz = np.stack([(u-k[0, 2])*z/k[0, 0], (v-k[1, 2])*z/k[1, 1], z], axis=-1)
        clouds = {}
        for name, keep in [('scene', valid), ('workspace', valid & (mask > 0))]:
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(xyz[keep])
            cloud.colors = o3d.utility.Vector3dVector(rgb[keep].astype(float)/255)
            if not o3d.io.write_point_cloud(str(root / f'{name}.ply'), cloud):
                raise RuntimeError(f'Could not write {name}.ply')
            clouds[name] = cloud
            print(f'{name}.ply: {len(cloud.points)} points', flush=True)
        if args.view:
            o3d.visualization.draw_geometries([clouds['workspace']],
                                             window_name='GraspNet workspace (camera frame)')


if __name__ == '__main__':
    main()
