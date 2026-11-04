#!/usr/bin/env python3
"""Measure whether Qwen reports absent objects, with and without crop verification.

Every capture is asked about every object. capture_record.json says which are
present, so this is a real presence check, but on ten captures from one session
it is a dev measurement. It does not check box or location accuracy.
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools/diagnostics'))
from home_perception.detect.verify import crop_box, parse_answer, question, question_english, resize_for_model
from home_perception.io.snapshot import load_snapshot, snapshot_folders
from tools.lib.run_manifest import provenance, write_json

OBJECTS = {'cup': '컵', 'remote control': '리모컨', 'calculator': '계산기',
           'marker pen': '마커펜', 'utility knife': '커터칼'}
INSTRUCTION = ('\nReturn only a JSON array of visible matching objects. '
               'Each object must have "label" (English name) and "bbox_2d": [x1,y1,x2,y2]. '
               'Coordinates must be integers normalized to 0..1000, relative to the full image. '
               'Use top-left and bottom-right corners. Return [] if no objects match.')


def summarise(rows):
    def count(present, key):
        return sum(1 for row in rows if row['present'] == present and row[key])
    positives, negatives = sum(row['present'] for row in rows), sum(not row['present'] for row in rows)
    return {'present_questions': positives, 'absent_questions': negatives,
            'detector_only': {'found_present': count(True, 'detected'), 'false_detections': count(False, 'detected')},
            'with_verification': {'found_present': count(True, 'verified'), 'false_detections': count(False, 'verified')}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=ROOT / 'data/tabletop_rgbd_20261101')
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/models/qwen3-vl-4b')
    parser.add_argument('--output', type=Path, default=ROOT / 'tmp/presence-eval')
    parser.add_argument('--detector', choices=['qwen', 'dino'], default='qwen')
    parser.add_argument('--verify-language', choices=['ko', 'en'], default='en')
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    import cv2
    import torch
    from PIL import Image
    from qwen3_vl_rgbd import parse_boxes
    from transformers import AutoProcessor, Qwen3VLForConditionalGeneration
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA is not visible to this process')
    model = Qwen3VLForConditionalGeneration.from_pretrained(
        str(args.model), dtype=torch.bfloat16, attn_implementation='sdpa').to('cuda').eval()
    processor = AutoProcessor.from_pretrained(str(args.model))

    def ask(image, text, tokens):
        messages = [{'role': 'user', 'content': [{'type': 'image', 'image': image}, {'type': 'text', 'text': text}]}]
        inputs = processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                               return_dict=True, return_tensors='pt').to('cuda')
        with torch.inference_mode():
            generated = model.generate(**inputs, max_new_tokens=tokens, do_sample=False)
        return processor.batch_decode(generated[:, inputs['input_ids'].shape[1]:], skip_special_tokens=True)[0]

    dino = None
    if args.detector == 'dino':
        from home_perception.detect.dino import DinoDetector
        dino = DinoDetector('cuda')
    rows, started = [], time.monotonic()
    for folder in snapshot_folders(args.dataset):
        snapshot = load_snapshot(folder)
        image = Image.fromarray(cv2.cvtColor(snapshot.bgr, cv2.COLOR_BGR2RGB))
        for english, korean in OBJECTS.items():
            error = None
            if dino:
                boxes = [item.box_xyxy for item in dino.detect(snapshot.bgr, english)]
            else:
                raw = ask(image, f'{korean}을(를) 찾아줘.' + INSTRUCTION, 1024)
                try:
                    boxes = [item['bbox_xyxy'] for item in parse_boxes(raw, image.width, image.height)]
                except (ValueError, KeyError, TypeError) as exc:
                    boxes, error = [], str(exc)
            answers = []
            for number, box in enumerate(boxes):
                crop = resize_for_model(crop_box(snapshot.bgr, box))
                text = (question(korean) if args.verify_language == 'ko'
                        else question_english(english))
                answer = ask(Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)), text, 16)
                answers.append({'box': box, 'answer': answer, 'yes': parse_answer(answer)})
                cv2.imwrite(str(args.output / f'{snapshot.capture_id[:2]}_{english.replace(" ", "_")}_{number}.jpg'), crop)
            rows.append({'capture': snapshot.capture_id, 'object': english, 'present': english in snapshot.objects,
                         'detected': bool(boxes), 'verified': any(item['yes'] for item in answers),
                         'answers': answers, 'error': error})
            print(snapshot.capture_id[:2], english, rows[-1]['present'], len(boxes), rows[-1]['verified'], flush=True)
    write_json(args.output / 'report.json', {
        'schema_version': 1, 'mode': 'offline_presence_eval', 'hardware_commands': 0, 'accuracy_of_location_checked': False,
        'dataset': str(args.dataset), 'wall_seconds': time.monotonic() - started, 'summary': summarise(rows),
        'rows': rows, 'provenance': provenance()})
    print(json.dumps(summarise(rows), ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
