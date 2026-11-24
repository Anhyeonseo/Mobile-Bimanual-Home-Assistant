#!/usr/bin/env python3
"""Saved aligned RGB-D + binary mask -> SO101 contact overlay and JSON; no model/device."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.diagnostics.replay_s3_grasp import load_scene
from tools.diagnostics.run_s3_stationary_lab import stock_jaw
from home_perception.geometry.plane import backproject
from home_perception.grasp.so101 import candidates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--mask', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    meta, depth, _, plane = load_scene(args.snapshot)
    with Image.open(args.mask) as image:
        mask = np.asarray(image)
    if mask.shape != depth.shape or not np.isin(mask, [0, 255]).all():
        parser.error('mask must be binary 0/255 and aligned with depth')
    intr = meta['aligned_depth_intrinsics']
    points = backproject(depth, intr, (mask > 0) & (depth > 0))
    jaw = stock_jaw()
    result = candidates(points, plane, (jaw.gaps[0], jaw.gaps[-1]))
    result.update(mode='offline_contact_preview', support_plane=plane.to_dict(),
        support_plane_source='global_fit_preview_only', hardware_commands=0,
        mask_sha256=hashlib.sha256(args.mask.read_bytes()).hexdigest(),
        source_snapshot=str(args.snapshot.resolve()), coordinate_frame=meta['coordinate_frame'])
    with Image.open(args.snapshot/'color.png') as image:
        overlay = image.convert('RGB')
    draw = ImageDraw.Draw(overlay)
    def pixel(point):
        x, y, z = point
        return (float(x/z*intr['fx']+intr['ppx']), float(y/z*intr['fy']+intr['ppy']))
    # Highest-ranked candidate only: all tilts project onto the same contact line.
    if result['refined_grasps']:
        c = result['refined_grasps'][0]
        a, b = pixel(c['fixed_contact_optical_m']), pixel(c['moving_contact_optical_m'])
        draw.line([a, b], fill='yellow', width=3)
        for (x, y), color in ((a, 'red'), (b, 'cyan')):
            draw.ellipse((x-4, y-4, x+4, y+4), fill=color)
    draw.text((10, 10), 'RED fixed / CYAN moving | observation only', fill='white')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir/'candidates.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    overlay.save(args.output_dir/'contacts.png')
    print(f"{len(result['refined_grasps'])} candidates; generation {result['candidate_generation_seconds']:.4f}s")


if __name__ == '__main__':
    main()
