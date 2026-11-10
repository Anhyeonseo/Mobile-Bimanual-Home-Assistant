#!/usr/bin/env python3
"""Report a stationary lift target from a support height, in centimetres; no device access."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.stationary_lift import LiftHeightRule


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--support-cm', required=True, type=float, help='height from room floor to supporting surface')
    parser.add_argument('--above-support-cm', default=0., type=float, help='optional working level above the surface')
    parser.add_argument('--tabletop-calibration', action='store_true', help='experimental left-arm high tabletop offset')
    parser.add_argument('--previous-preset', metavar='PRESET_ID',
                        help='last preset on this support, e.g. 45cm; omit for a new support')
    parser.add_argument('--side', choices=('left','right'), default='left', help='static reference, default left')
    args = parser.parse_args()
    if args.tabletop_calibration and args.side != 'left':
        parser.error('tabletop calibration evaluated only with the left arm')
    try:
        config = ROOT/'config/home_assistant'
        profile = json.loads((config/'s2_coordinates.development.json').read_text())
        rule = LiftHeightRule.from_profile(profile, side=args.side)
        answer = rule.setting(args.support_cm/100, args.above_support_cm/100, previous_preset=args.previous_preset,
                              tabletop_calibration=args.tabletop_calibration)
        print(json.dumps(answer, indent=2, allow_nan=False))
    except (ValueError, KeyError, OSError) as exc:
        parser.exit(2, str(exc)+'\n')


if __name__ == '__main__':
    main()
