#!/usr/bin/env python3
"""리모컨 후보가 왜 탈락하는지 물체별로 센다. 보완 전 결과 폴더를 넣는다."""
import importlib.util
import json
from pathlib import Path
import sys
import types

import numpy as np
import open3d as o3d

sys.path.insert(0, '.')
from home_perception.geometry.plane import backproject, fit_plane
from home_perception.io.snapshot import load_snapshot

G = Path(sys.argv[1] if len(sys.argv) > 1 else 'tmp/followup/grasps-before')
baseline = Path('artifacts/graspnet-baseline')
for part in ('models', 'utils', 'pointnet2'):
    sys.path.insert(0, str(baseline / part))
package = types.ModuleType('graspnetAPI')
package.__path__ = [str(Path(importlib.util.find_spec('graspnetAPI').origin).parent)]
sys.modules['graspnetAPI'] = package
from graspnetAPI.grasp import GraspGroup
from collision_detector import ModelFreeCollisionDetector

totals = {}
for item in json.loads((G / 'report.json').read_text())['objects']:
    capture = item['capture']
    grasps = GraspGroup(np.load(G / capture / f'grasps_{item["index"]:03}/grasps_target.npy'))
    if not len(grasps):
        continue
    snapshot = load_snapshot(Path('data/tabletop_rgbd_20261101') / capture)
    depth = snapshot.depth_m()
    plane = fit_plane(backproject(depth, snapshot.intrinsics, depth > 0))
    scene = np.asarray(o3d.io.read_point_cloud(str(G / capture / 'objects/scene.ply')).points)
    detector = ModelFreeCollisionDetector(scene, voxel_size=.01)
    collisions, empties = [], []
    for start in range(0, len(grasps), 32):
        collision, empty = detector.detect(grasps[start:start + 32], approach_dist=.05, collision_thresh=.01,
                                           return_empty_grasp=True)
        collisions += collision.tolist()
        empties += empty.tolist()
    collisions, empties = np.array(collisions), np.array(empties)
    tips = grasps.translations + grasps.rotation_matrices[:, :, 0] * grasps.depths[:, None]
    total = totals.setdefault(item['label'], [0, 0, 0, 0, 0.])
    total[0] += len(grasps)
    total[1] += int(collisions.sum())
    total[2] += int((~collisions & empties).sum())
    total[3] += int((~collisions & ~empties).sum())
    total[4] += float((plane.height(tips) < 0).sum())
for label, (count, collision, empty, kept, below) in totals.items():
    print(f'{label}: 연결 후보 {count}, 충돌 탈락 {collision}, 빈손 탈락 {empty}, 남음 {kept}, '
          f'집게 끝이 책상 아래 {below / count * 100:.0f}%')
