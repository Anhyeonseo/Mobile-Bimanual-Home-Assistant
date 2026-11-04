#!/usr/bin/env python3
"""Export a perception run as an auto-labelled YOLO segmentation dataset.

Labels are unreviewed model output. One capture session cannot give an honest
test set, so the validation split only proves the training path works.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import cv2

from home_perception.eval.yolo_labels import label_line, mask_polygon

CLASSES = {'컵': 'cup', '리모컨': 'remote control', '계산기': 'calculator', '마커펜': 'marker pen', '커터칼': 'utility knife'}
NAMES = list(CLASSES.values())


def class_id(label):
    for korean, english in CLASSES.items():
        if korean in label or english in label.lower():
            return NAMES.index(english)
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--perception', type=Path, default=ROOT / 'tmp/tabletop-perception-r4')
    parser.add_argument('--dataset', type=Path, default=ROOT / 'data/tabletop_rgbd_20261101')
    parser.add_argument('--output', type=Path, default=ROOT / 'tmp/yolo-tabletop')
    parser.add_argument('--validation', nargs='*', default=['04_similar_objects', '09_misc_objects'])
    parser.add_argument('--include-held', action='store_true', help='Masks are usable even when depth is poor')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = json.loads((args.perception / 'report.json').read_text(encoding='utf-8'))
    counts = {name: 0 for name in NAMES}
    skipped = []
    for capture in report['captures']:
        name = capture['capture']
        split = 'val' if name in args.validation else 'train'
        for folder in ('images', 'labels'):
            (output / folder / split).mkdir(parents=True, exist_ok=True)
        lines = []
        for index, item in enumerate(capture['observations']):
            identifier = class_id(item['label'])
            usable = item['status'] == 'accepted' or (args.include_held and item['status'] == 'held')
            polygon = None
            if identifier is not None and usable:
                mask = cv2.imread(str(args.perception / name / f'mask_{index:03}.png'), cv2.IMREAD_UNCHANGED) > 0
                polygon = mask_polygon(mask)
            if polygon is None:
                skipped.append({'capture': name, 'label': item['label'], 'status': item['status']})
                continue
            lines.append(label_line(identifier, polygon))
            counts[NAMES[identifier]] += 1
        shutil.copyfile(args.dataset / name / 'color.png', output / 'images' / split / f'{name}.png')
        (output / 'labels' / split / f'{name}.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    (output / 'dataset.yaml').write_text(
        f'path: {output}\ntrain: images/train\nval: images/val\nnames:\n'
        + ''.join(f'  {index}: {name}\n' for index, name in enumerate(NAMES)), encoding='utf-8')
    (output / 'export.json').write_text(json.dumps(
        {'reviewed_by_human': False, 'source': str(args.perception), 'counts': counts, 'skipped': skipped,
         'validation': args.validation}, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'counts': counts, 'skipped': len(skipped)}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
