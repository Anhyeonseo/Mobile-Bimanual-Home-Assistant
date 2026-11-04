#!/usr/bin/env python3
"""1단계: 라이브러리 없이 RANSAC으로 책상면 찾기. GPU 불필요."""
import json
from pathlib import Path

import cv2
import numpy as np

SAMPLE = Path('data/tabletop_rgbd_20261101/05_remote_only')
OUT = Path('tmp/tutorial')
OUT.mkdir(parents=True, exist_ok=True)

# 사진, 거리, 카메라 값을 읽는다. 거리 값이 0이면 측정 실패다.
bgr = cv2.imread(str(SAMPLE / 'color.png'))
meta = json.loads((SAMPLE / 'metadata.json').read_text())
camera = meta['aligned_depth_intrinsics']
depth = cv2.imread(str(SAMPLE / 'depth_aligned.png'), cv2.IMREAD_UNCHANGED) * meta['depth_scale_m']

# 픽셀(u, v)과 거리(Z)를 실제 3D 좌표(X, Y, Z)로 바꾼다.
v, u = np.nonzero(depth > 0)
z = depth[v, u]
points = np.column_stack(((u - camera['ppx']) * z / camera['fx'], (v - camera['ppy']) * z / camera['fy'], z))
print('3D 점 개수:', len(points))

# RANSAC: 점 3개로 평면을 만들고, 그 평면에 가까운 점이 가장 많은 것을 고른다.
THRESHOLD_M, ITERATIONS = .005, 300
rng = np.random.default_rng(0)
sample = points[rng.choice(len(points), 40000, replace=False)]
best_count, best_normal, best_point = 0, None, None
for _ in range(ITERATIONS):
    a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
    normal = np.cross(b - a, c - a)
    if np.linalg.norm(normal) < 1e-9:
        continue
    normal /= np.linalg.norm(normal)
    count = (np.abs((sample - a) @ normal) < THRESHOLD_M).sum()
    if count > best_count:
        best_count, best_normal, best_point = count, normal, a
print(f'가장 좋은 평면: 표본 {len(sample)}개 중 {best_count}개가 5mm 이내')

# 법선이 카메라 쪽을 향하게 뒤집으면 책상 위 물체의 높이가 양수가 된다.
if best_normal[2] > 0:
    best_normal = -best_normal
height = (points - best_point) @ best_normal
table = np.abs(height) < THRESHOLD_M
print(f'책상 점 비율: {table.mean() * 100:.1f}%, 카메라에서 책상까지: {abs(best_normal @ best_point) * 1000:.1f}mm')

# 초록은 책상, 빨강은 책상보다 5mm 이상 높은 점이다.
image = bgr.copy()
image[v[table], u[table]] = (.5 * image[v[table], u[table]] + (0, 128, 0)).astype(np.uint8)
above = height > THRESHOLD_M
image[v[above], u[above]] = (0, 0, 255)
cv2.imwrite(str(OUT / 'step1_table.jpg'), image)
print('결과:', OUT / 'step1_table.jpg')
