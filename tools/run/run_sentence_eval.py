#!/usr/bin/env python3
"""Korean sentence -> TaskRequest -> detect -> verify -> select, scored per sentence.

The parsed target text, modifiers included, is both the detection query and the
verification question. A request the parser leaves unresolved stops before
detection. Expected boxes come from pipeline output on the dev captures, so this
is a regression set for those captures, not a measure of general accuracy.
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools/diagnostics'))
from home_perception.detect.verify import crop_box, parse_answer, question, resize_for_model
from home_perception.eval.robustness import box_iou
from home_perception.io.snapshot import load_snapshot
from home_perception.language.target_selection import select_target
from home_perception.language.task_parser import PROMPT, parse_reply
from tools.lib.run_manifest import write_json
from run_target_selection_eval import INSTRUCTION


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, default=ROOT / 'config/perception_sentence_cases.json')
    parser.add_argument('--dataset', type=Path, default=ROOT / 'data/tabletop_rgbd_20261101')
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/models/qwen3-vl-4b')
    parser.add_argument('--output', type=Path, default=ROOT / 'tmp/sentence-eval/report.json')
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
        content = ([{'type': 'image', 'image': image}] if image is not None else []) + [{'type': 'text', 'text': text}]
        inputs = processor.apply_chat_template([{'role': 'user', 'content': content}], tokenize=True,
                                               add_generation_prompt=True, return_dict=True,
                                               return_tensors='pt').to('cuda')
        with torch.inference_mode():
            output = model.generate(**inputs, max_new_tokens=tokens, do_sample=False)
        return processor.batch_decode(output[:, inputs['input_ids'].shape[1]:], skip_special_tokens=True)[0]

    rows, started = [], time.monotonic()
    for index, case in enumerate(json.loads(args.cases.read_text(encoding='utf-8'))):
        request = parse_reply(ask(None, PROMPT + case['text'], 128), f'case-{index}', 'eval', case['text'])
        boxes, answers, chosen = [], [], None
        if request.unresolved:
            failure = 'UNRESOLVED:' + ','.join(request.unresolved)
        else:
            snapshot = load_snapshot(args.dataset / case['capture'])
            image = Image.fromarray(cv2.cvtColor(snapshot.bgr, cv2.COLOR_BGR2RGB))
            try:
                boxes = [item['bbox_xyxy'] for item in parse_boxes(
                    ask(image, f'{request.target}을(를) 모두 찾아줘.' + INSTRUCTION, 512), image.width, image.height)]
            except (ValueError, KeyError, TypeError):
                boxes = []
            candidates = []
            for box in boxes:
                crop = resize_for_model(crop_box(snapshot.bgr, box))
                answer = ask(Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)), question(request.target), 16)
                answers.append(answer)
                if parse_answer(answer):
                    candidates.append({'box_xyxy': box})
            chosen, failure = select_target(candidates, request.location)
        expected = case['expect']
        passed = failure == expected if isinstance(expected, str) else bool(chosen and box_iou(chosen['box_xyxy'], expected) >= .5)
        rows.append({**case, 'passed': passed, 'failure': failure, 'request': request.to_dict(),
                     'boxes': boxes, 'answers': answers, 'chosen': chosen and chosen['box_xyxy']})
        print('PASS' if passed else 'FAIL', case['capture'][:2], case['text'], '->', request.target, request.location,
              failure, [round(v) for v in (chosen['box_xyxy'] if chosen else [])], answers, flush=True)
    passed = sum(row['passed'] for row in rows)
    write_json(args.output, {'schema_version': 1, 'model': str(args.model), 'passed': passed, 'total': len(rows),
                             'seconds': time.monotonic() - started, 'rows': rows})
    print(passed, '/', len(rows))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
