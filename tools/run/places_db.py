#!/usr/bin/env python3
"""Create and inspect the local furniture DB. Opens no robot devices."""
import argparse
import json
from pathlib import Path
import sqlite3
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.place_registry import PlaceRegistry


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database', type=Path, default=ROOT / 'data/home.sqlite3')
    sub = parser.add_subparsers(dest='command', required=True)
    init = sub.add_parser('init', help='Register names; coordinates remain NULL')
    init.add_argument('--seed', type=Path, default=ROOT / 'config/home_places.json')
    sub.add_parser('list', help='List places and registration status')
    show = sub.add_parser('show', help='Look up an ID, Korean label or alias')
    show.add_argument('place')
    stop = sub.add_parser('set-stop', help='Register measured robot stop coordinates later')
    stop.add_argument('place')
    for name in ('map-id', 'map-revision', 'frame-id'):
        stop.add_argument('--' + name, required=True)
    for name in ('x', 'y', 'yaw'):
        stop.add_argument('--' + name, type=float, required=True)
    args = parser.parse_args(argv)
    try:
        seed = json.loads(args.seed.read_text(encoding='utf-8')) if args.command == 'init' else None
        with PlaceRegistry(args.database, create=args.command == 'init') as registry:
            if args.command == 'init':
                registry.seed(seed)
                result = {'database': str(args.database.resolve()), 'places': registry.list_places()}
            elif args.command == 'list':
                result = registry.list_places()
            elif args.command == 'show':
                result = registry.get(args.place)
            else:
                result = registry.set_stop(args.place, map_id=args.map_id, map_revision=args.map_revision,
                                           frame_id=args.frame_id, x_m=args.x, y_m=args.y, yaw_rad=args.yaw)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, KeyError, TypeError, sqlite3.Error) as error:
        parser.exit(2, f'Place DB error: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
