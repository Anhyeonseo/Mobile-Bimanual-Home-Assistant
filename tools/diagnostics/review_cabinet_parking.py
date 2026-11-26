"""Export a self-contained cabinet navigation replay from a recorded trial.

Use --help for inputs. Detection IDs identify surface points reviewed in the
pictures. They do not select a target for the robot or authorize motion.
"""
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.camera_evidence import from_metadata, load_capture_profile, match_observed_identity
from home_robot_tasks.s2_geometry import optical_to_base


def read(path):
    return json.loads(path.read_text())


def image_uri(image, fmt):
    buf = io.BytesIO()
    image.save(buf, fmt)
    return 'data:image/' + fmt.lower() + ';base64,' + base64.b64encode(buf.getvalue()).decode()


def exposure(report_path, detection_id):
    report = read(report_path)
    snapshot = report_path.parent.parent
    for name in ('color.png', 'depth_aligned.png', 'metadata.json'):
        if hashlib.sha256((snapshot / name).read_bytes()).hexdigest() != report['source_sha256'][name]:
            raise ValueError(f'촬영과 탐지 기록 불일치: {name}')
    meta = read(snapshot / 'metadata.json')
    profile, reference = load_capture_profile(ROOT, 'near')
    match_observed_identity(from_metadata(meta), reference)
    image = Image.open(snapshot / 'color.png').convert('RGB')
    draw = ImageDraw.Draw(image)
    target = next((o for o in report['detections'] if o['id'] == detection_id), None)
    if not target or not target['surface_point']['valid']:
        raise ValueError('실측 깊이가 있는 대상 ID를 지정하세요.')
    for obj in report['detections']:
        xy = obj['box_xyxy']
        draw.rectangle(xy, outline='#008077' if obj == target else '#ed9951', width=3)
        draw.text((xy[0], xy[3]+2), str(obj['id']), fill='white', stroke_width=2, stroke_fill='#243940')
    point = target['surface_point']
    x, y = point['pixel']
    draw.ellipse((x-3,y-3,x+3,y+3), fill='white', outline='black')
    return dict(photo=image_uri(image, 'JPEG'), xyz=optical_to_base(profile['base_from_optical'], point['xyz_m']),
                date=report['captured_at_utc'], label=target['label'], count=len(report['detections']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('trial', 'map', 'household', 'output'):
        parser.add_argument('--'+key, required=True, type=Path)
    parser.add_argument('--observation', type=Path, help='후속 근거리 촬영 detections.json')
    parser.add_argument('--reference', type=Path, help='성공한 근거리 촬영 detections.json')
    parser.add_argument('--object-id', type=int, default=0)
    parser.add_argument('--reference-id', type=int, default=0)
    args = parser.parse_args()
    route = read(ROOT/'config/home_assistant/cabinet_navigation.json')
    map_hash = hashlib.sha256(args.map.read_bytes()).hexdigest()
    labels = read(args.household/'household-labels.json')
    if map_hash != labels['source_map_sha256'] or map_hash != route['files']['map']['sha256']:
        raise ValueError('수납장과 주행 지도의 좌표 기준이 일치하지 않습니다.')
    grid = read(args.map)
    values = np.array(grid['data']).reshape(grid['height'], grid['width'])
    shade = np.where(values < 0, 210, np.where(values >= 50, 97, 249)).astype('uint8')
    cabinet = read(args.household/'review-geometry.json')[route['support_id']]
    nav = args.trial/'navigation'
    result = read(args.trial/'result.json')
    events = [json.loads(line) for f in nav.glob('*.jsonl') for line in f.read_text().splitlines() if line.strip()]
    start = next((e for e in events if e['event'] == 'started'), None)
    if not start:
        parser.error('주행 시작 기록이 없습니다.')
    t0 = start['host_monotonic_ns']
    loc = sorted((e for e in events if e['event']=='localization' and e['host_monotonic_ns'] >= t0),
                 key=lambda e:e['host_monotonic_ns'])
    poses = [[0, *start['pose']]] + [[round((e['host_monotonic_ns']-t0)/1e9, 3), *e['pose']] for e in loc]
    body = nav/'upper-body.json'
    data = dict(map=image_uri(Image.fromarray(shade[::-1]), 'PNG'), origin=grid['origin'], cell=grid['resolution'],
                size=[grid['width'],grid['height']], cabinet=sorted(set((x,y) for x,y,z in cabinet)),
                goal=start['goal'], poses=poses, footprint=read(body)['footprint_xy_m'] if body.exists() else [],
                status=result['result']['reason'], trial=result['id'],
                observation=exposure(args.observation,args.object_id) if args.observation else None,
                reference=exposure(args.reference,args.reference_id) if args.reference else None)
    if data['observation'] and data['reference'] and data['observation']['label'] != data['reference']['label']:
        raise ValueError('비교 대상 이름이 다릅니다.')
    template = Path(__file__).with_suffix('.html').read_text()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(template.replace('__REPLAY_DATA__', json.dumps(data, allow_nan=False).replace('<',r'\u003c')))


if __name__ == '__main__':
    main()
