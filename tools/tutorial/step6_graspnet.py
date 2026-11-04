#!/usr/bin/env python3
"""6단계: 앞 단계의 윤곽으로 GraspNet을 돌리고 후보를 필터링하기."""
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

sys.path.insert(0, '.')
from home_perception.grasp.refine import flat_object_grasp, refine
from home_perception.io.snapshot import load_snapshot
from tools.run.run_tabletop_grasps import prepare, read_points

OUT = Path('tmp/tutorial')
folder = OUT / 'grasp'
shutil.rmtree(folder, ignore_errors=True)
snapshot = load_snapshot('data/tabletop_rgbd_20261101/05_remote_only')
objects, records, _, plane = prepare(snapshot, [{'label': 'remote', 'status': 'accepted'}], OUT, folder)

subprocess.run([sys.executable, 'tools/diagnostics/graspnet_candidates.py', '--objects', str(objects), '--index', '0',
                '--focus-radius-m', '0.15', '--collision-voxel-m', '0.005', '--output', str(folder / 'result')],
               check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
candidates = np.load(folder / 'result/grasps_filtered.npy')
target = read_points(objects / records[0]['cloud'])
print('충돌 검사를 통과한 GraspNet 후보:', len(candidates))
kept, rejected = refine(candidates, target, plane, .012)
print('탈락 이유:', rejected, '-> 남은 후보:', len(kept))
rule, reason = flat_object_grasp(target, read_points(objects / 'scene.ply'), plane)
print('규칙 후보:', '없음 ' + str(reason) if rule is None else f'집게 폭 {rule[0, 1] * 100:.1f}cm')
print('GraspNet 결과 그림:', folder / 'result/grasps.jpg')
