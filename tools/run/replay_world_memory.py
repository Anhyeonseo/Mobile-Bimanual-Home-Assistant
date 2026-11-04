#!/usr/bin/env python3
"""Replay a perception run through object memory in capture order.

All captures share one fixed camera, so the camera frame stands in for a map.
The objects were rearranged by hand between captures, which tests association
and moves. It does not test multiple viewpoints or real map coordinates.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from home_perception.contracts import ObjectObservation
from home_perception.memory.world_memory import WorldMemory
from tools.lib.run_manifest import write_json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--perception', type=Path, default=ROOT / 'tmp/tabletop-perception')
    parser.add_argument('--output', type=Path, default=ROOT / 'tmp/world-memory/report.json')
    args = parser.parse_args(argv)
    report = json.loads((args.perception / 'report.json').read_text(encoding='utf-8'))
    memory = WorldMemory('color_camera_optical_frame')
    timeline = []
    for capture in report['captures']:
        memory.observe([ObjectObservation.from_dict(item) for item in capture['observations']])
        timeline.append({'capture': capture['capture'], 'entities': [
            {'id': entity.entity_id, 'kind': entity.kind, 'state': entity.state, 'position_m': entity.position_m}
            for entity in memory.entities.values()]})
        print(capture['capture'][:2], ' '.join(f'{e.kind}:{e.state[:3]}' for e in memory.entities.values()), flush=True)
    write_json(args.output, {'schema_version': 1, 'timeline': timeline, 'map_coordinates': False,
                             'entities': [entity.to_dict() for entity in memory.entities.values()]})
    print(len(memory.entities), 'entities')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
