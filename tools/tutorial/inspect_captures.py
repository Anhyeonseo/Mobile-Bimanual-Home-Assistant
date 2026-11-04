#!/usr/bin/env python3
"""받은 사진을 처음 검사했을 때의 계산. 파일 형식, 깊이, 밝기, 책상 평면, 물체 덩어리를 본다."""
import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from home_perception.geometry.plane import backproject, fit_plane

DATA = Path(sys.argv[1] if len(sys.argv) > 1 else 'data/tabletop_rgbd_20261101')
manifest = {c['folder']: c for c in json.loads((DATA / 'manifest.json').read_text())['captures']}
settings = set()
for name in sorted(x for x in os.listdir(DATA) if (DATA / x).is_dir()):
    folder = DATA / name
    color = cv2.imread(str(folder / 'color.png'), cv2.IMREAD_UNCHANGED)
    depth = cv2.imread(str(folder / 'depth_aligned.png'), cv2.IMREAD_UNCHANGED)
    meta = json.loads((folder / 'metadata.json').read_text())
    k = meta['aligned_depth_intrinsics']
    settings.add((k['fx'], k['fy'], meta['depth_scale_m'], meta['device']['serial'], str(meta['depth_sensor_options'])))
    valid = depth > 0
    z = depth * meta['depth_scale_m']
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    plane = fit_plane(backproject(z, k, valid))
    print(f'{name:20} valid {valid.mean() * 100:5.2f}% (manifest {manifest[name]["valid_aligned_depth_percent"]:5.2f}) '
          f'Z {z[valid].min():.3f}~{z[valid].max():.2f}m bright {gray.mean():.0f} saturated {(color.max(2) >= 250).mean() * 100:.2f}% '
          f'plane rms {plane.rms_m * 1e3:.2f}mm tilt {np.degrees(np.arccos(abs(plane.normal[2]))):.1f}deg '
          f'skew {meta["color_frame"]["timestamp_ms"] - meta["depth_frame"]["timestamp_ms"]:.2f}ms')
print('카메라 설정 종류 수:', len(settings))
