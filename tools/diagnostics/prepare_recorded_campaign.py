#!/usr/bin/env python3
"""Prepare recorded multi-part mapping evidence locally; no robot connection."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'ros2_ws/src/home_robot_tasks'), str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.recorded_campaign import prepare_campaign, prepare_lidar_campaign, attach_rgbd_campaign


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--all-sensor-observations', action='store_true')
    parser.add_argument('--lidar-only', action='store_true', help='All lidar scans, body shadow masked; no wheel feedback or RGB-D')
    parser.add_argument('--surface-matching', action='store_true', help='Lidar-only comparison: use local surface normals instead of point correspondence')
    parser.add_argument('--attach-rgbd-to', type=Path, help='Attach original RGB-D to an existing lidar-only preparation; no SLAM rerun')
    args = parser.parse_args()
    if args.surface_matching and not args.lidar_only:
        parser.error('--surface-matching requires --lidar-only')
    campaign = json.loads(args.campaign.read_text())
    segments = [args.campaign.parent/s['id'] for s in campaign['segments']]
    if args.attach_rgbd_to:
        if args.lidar_only or args.all_sensor_observations or args.surface_matching:
            parser.error('RGB-D attachment excludes trajectory preparation options')
        report=attach_rgbd_campaign(args.attach_rgbd_to,segments,args.output)
    elif args.lidar_only:
        report = prepare_lidar_campaign(ROOT, segments, args.output.resolve(), surface_matching=args.surface_matching)
    else:
        report = prepare_campaign(ROOT, segments, args.output.resolve(), all_sensor_observations=args.all_sensor_observations)
    print(json.dumps({k: report[k] for k in ('passed', 'scans', 'rgbd_frames', 'path_length_m') if k in report}))


if __name__ == '__main__':
    main()
