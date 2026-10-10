#!/usr/bin/env python3
"""Run Grounding DINO and camera-coordinate conversion on a saved D415 capture."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.lib.grounding_rgbd import GroundingRGBD


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--prompt", required=True, help="English object description")
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--box-threshold", type=float, default=0.30)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--sam", action="store_true", help="Segment boxes with SAM 2.1 and sample depth inside masks")
    args = parser.parse_args()
    detector = GroundingRGBD(args.device, args.box_threshold, args.text_threshold, use_sam=args.sam)
    detector.analyze_snapshot(args.snapshot, args.prompt)


if __name__ == "__main__":
    main()
