#!/usr/bin/env python3
"""4단계: 윤곽 안의 픽셀을 3D 점구름으로 만들고 크기 재기. GPU 불필요."""
from pathlib import Path
import sys

import cv2
import numpy as np
import open3d as o3d

sys.path.insert(0, '.')
from home_perception.geometry.plane import backproject, fit_plane
from home_perception.io.snapshot import load_snapshot

OUT = Path('tmp/tutorial')
snapshot = load_snapshot('data/tabletop_rgbd_20261101/05_remote_only')
depth = snapshot.depth_m()
mask = cv2.imread(str(OUT / 'mask_000.png'), 0) > 0
plane = fit_plane(backproject(depth, snapshot.intrinsics, depth > 0))

points = backproject(depth, snapshot.intrinsics, mask & (depth > 0))
print(f'윤곽 안 유효 깊이: {(mask & (depth > 0)).sum() / mask.sum() * 100:.1f}%')
heights = plane.height(points)
points = points[heights > .003]     # 윤곽 경계에 섞인 책상 점을 뺀다.
print(f'물체 높이: {np.percentile(plane.height(points), 95) * 1000:.1f}mm')

cloud = o3d.geometry.PointCloud()
cloud.points = o3d.utility.Vector3dVector(points)
box = cloud.get_minimal_oriented_bounding_box()
print('감싸는 상자 크기(mm):', sorted(np.round(np.asarray(box.extent) * 1000).tolist()))
o3d.io.write_point_cloud(str(OUT / 'step4_object.ply'), cloud)
print('결과:', OUT / 'step4_object.ply')
if '--view' in sys.argv:
    o3d.visualization.draw_geometries([cloud, box])
