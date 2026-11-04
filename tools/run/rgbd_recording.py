#!/usr/bin/env python3
"""Create/inspect portable RGB-D data without accessing a camera."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.lib.rgbd_recording import import_graspnet, export_graspnet, validate_recording, write_recording


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    demo = sub.add_parser('demo')
    demo.add_argument('--output', type=Path, required=True)
    check = sub.add_parser('validate')
    check.add_argument('manifest', type=Path)
    imp = sub.add_parser('import-graspnet', help='Import the existing official example; no download')
    imp.add_argument('--sample', type=Path, default=ROOT / 'artifacts/datasets/graspnet-example')
    imp.add_argument('--output', type=Path, required=True)
    exp = sub.add_parser('export-graspnet')
    exp.add_argument('manifest', type=Path)
    exp.add_argument('--index', type=int, default=0)
    exp.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == 'validate':
            result = validate_recording(args.manifest)
        elif args.command == 'demo':
            import numpy as np
            rgb = np.zeros((48, 64, 3), np.uint8)
            rgb[18:30, 20:40] = [200, 180, 20]
            depth = np.full((48, 64), 1000, np.uint16)
            depth[18:30, 20:40] = 950
            path = write_recording(args.output, rgb, depth, [[50, 0, 32], [0, 50, 24], [0, 0, 1]], .001,
                                   evidence_source='synthetic', recording_id='synthetic-studio',
                                   workspace=np.ones_like(depth, np.uint8), calibration_id='synthetic-only')
            result = validate_recording(path)
        elif args.command == 'import-graspnet':
            result = validate_recording(import_graspnet(args.sample, args.output))
        else:
            result = {'sample': str(export_graspnet(args.manifest, args.output, args.index))}
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        parser.exit(2, f'Invalid recording: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
