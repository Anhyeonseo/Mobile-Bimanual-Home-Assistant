#!/usr/bin/env python3
"""Read-only checks for the tabletop captures and saved perception runs.

data: file format, valid depth and table plane per capture.
z: repeatability of table distance and object sizes. No ruler data is used,
so this shows consistency, never absolute depth accuracy.
grasp: why GraspNet candidates were rejected. Needs a run made with the
default arguments of run_tabletop_grasps.py.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import numpy as np

from home_perception.geometry.plane import backproject, fit_plane
from home_perception.io.snapshot import load_snapshot, snapshot_folders


def table(snapshot):
    depth = snapshot.depth_m()
    return fit_plane(backproject(depth, snapshot.intrinsics, depth > 0))


def check_data(args):
    for folder in snapshot_folders(args.dataset):
        snapshot = load_snapshot(folder)
        plane = table(snapshot)
        print(f'{snapshot.capture_id:20} {snapshot.bgr.shape[1]}x{snapshot.bgr.shape[0]} {snapshot.depth_raw.dtype} '
              f'valid {(snapshot.depth_raw > 0).mean() * 100:5.1f}%  table rms {plane.rms_m * 1e3:.2f}mm  '
              f'objects {", ".join(snapshot.objects or [])}')


def check_z(args):
    report = json.loads((args.perception / 'report.json').read_text(encoding='utf-8'))
    sizes = {}
    for capture in report['captures']:
        plane = table(load_snapshot(args.dataset / capture['capture']))
        distance = abs(plane.normal @ plane.point) * 1e3
        print(f'{capture["capture"]:20} camera to table {distance:.1f}mm  rms {plane.rms_m * 1e3:.2f}mm')
        for item in capture['observations']:
            if item['status'] == 'accepted':
                sizes.setdefault(item['label'], []).append(
                    (max(item['obb']['extent_m']) * 1e3, item['support_height_m'] * 1e3))
    for label, values in sizes.items():
        values = np.array(values)
        print(f'{label}: n={len(values)} length {values[:, 0].mean():.1f}+-{values[:, 0].std():.1f}mm '
              f'height {values[:, 1].mean():.1f}+-{values[:, 1].std():.1f}mm')


def check_grasp(args):
    report = json.loads((args.grasps / 'report.json').read_text(encoding='utf-8'))
    totals = {}
    for item in report['objects']:
        total = totals.setdefault(item['label'], dict(objects=0, associated=0, collision_free=0, final=0, rule=0))
        total['objects'] += 1
        total['associated'] += item['target_associated']
        total['collision_free'] += item['collision_and_empty_filtered']
        total['final'] += item['refined']
        total['rule'] += item['flat_rule'] == 'added'
    for label, total in totals.items():
        print(label, total)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('check', choices=['data', 'z', 'grasp'])
    parser.add_argument('--dataset', type=Path, default=ROOT / 'data/tabletop_rgbd_20261101')
    parser.add_argument('--perception', type=Path, default=ROOT / 'tmp/tabletop-perception')
    parser.add_argument('--grasps', type=Path, default=ROOT / 'tmp/tabletop-grasps')
    args = parser.parse_args()
    {'data': check_data, 'z': check_z, 'grasp': check_grasp}[args.check](args)


if __name__ == '__main__':
    main()
