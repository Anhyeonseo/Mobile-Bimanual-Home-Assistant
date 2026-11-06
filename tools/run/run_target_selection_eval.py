#!/usr/bin/env python3
"""Detect, verify and select one target for fixed cases; reports pass or fail per case.

Expected boxes come from pipeline output on the dev captures, so this
is a regression set for those captures, not a measure of general accuracy.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools/diagnostics'))
from home_perception.detect.verify import crop_box, parse_answer, question_english, resize_for_model
from home_perception.eval.robustness import box_iou
from home_perception.io.snapshot import load_snapshot
from home_perception.language.target_selection import select_target
from tools.lib.run_manifest import write_json

KOREAN = {'cup': '컵', 'remote control': '리모컨', 'calculator': '계산기', 'marker pen': '마커펜', 'utility knife': '커터칼'}
INSTRUCTION = ('\nFind every matching object, each with its own box. Return only a JSON array where each item has '
               '"label" and "bbox_2d": [x1,y1,x2,y2] as integers normalized to 0..1000. Return [] if nothing matches.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, default=ROOT / 'config/perception_language_cases.json')
    parser.add_argument('--dataset', type=Path, default=ROOT / 'data/tabletop_rgbd_20261101')
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/models/qwen3-vl-2b')
    parser.add_argument('--output', type=Path, default=ROOT / 'tmp/target-selection/report.json')
    args = parser.parse_args(argv)
    import cv2
    from PIL import Image
    from qwen3_vl_rgbd import parse_boxes
    import torch
    from transformers import AutoProcessor, Qwen3VLForConditionalGeneration
    model = Qwen3VLForConditionalGeneration.from_pretrained(
        str(args.model), dtype=torch.bfloat16, attn_implementation='sdpa').to('cuda').eval()
    processor = AutoProcessor.from_pretrained(str(args.model))

    def ask(image, text, tokens):
        messages = [{'role': 'user', 'content': [{'type': 'image', 'image': image}, {'type': 'text', 'text': text}]}]
        inputs = processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                               return_dict=True, return_tensors='pt').to('cuda')
        with torch.inference_mode():
            output = model.generate(**inputs, max_new_tokens=tokens, do_sample=False)
        return processor.batch_decode(output[:, inputs['input_ids'].shape[1]:], skip_special_tokens=True)[0]

    rows = []
    for case in json.loads(args.cases.read_text(encoding='utf-8')):
        snapshot = load_snapshot(args.dataset / case['capture'])
        image = Image.fromarray(cv2.cvtColor(snapshot.bgr, cv2.COLOR_BGR2RGB))
        raw = ask(image, f'{KOREAN[case["object"]]}을(를) 모두 찾아줘.' + INSTRUCTION, 512)
        boxes = [item['bbox_xyxy'] for item in parse_boxes(raw, image.width, image.height)]
        candidates, answers = [], []
        for box in boxes:
            crop = resize_for_model(crop_box(snapshot.bgr, box))
            answer = ask(Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)), question_english(case['object']), 16)
            answers.append(answer)
            if parse_answer(answer):
                candidates.append({'box_xyxy': box})
        chosen, failure = select_target(candidates, case['location'])
        expected = case['expect']
        passed = failure == expected if isinstance(expected, str) else bool(chosen and box_iou(chosen['box_xyxy'], expected) >= .5)
        rows.append({**case, 'passed': passed, 'failure': failure, 'boxes': boxes, 'answers': answers,
                     'chosen': chosen and chosen['box_xyxy']})
        print('PASS' if passed else 'FAIL', case['capture'][:2], case['object'], case['location'], failure,
              [round(v) for v in (chosen['box_xyxy'] if chosen else [])], answers, flush=True)
    write_json(args.output, {'schema_version': 1, 'model': str(args.model), 'passed': sum(r['passed'] for r in rows),
                             'total': len(rows), 'rows': rows})
    print(sum(row['passed'] for row in rows), '/', len(rows))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
