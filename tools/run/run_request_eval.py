#!/usr/bin/env python3
"""Measure the Korean request parser on the fixed sentence set; text only, no robot."""
import argparse
import collections
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from home_perception.language.task_parser import PROMPT, parse_reply
from tools.lib.run_manifest import write_json

SLOTS = ('action', 'target', 'source', 'destination', 'location')


def normalise(value):
    return value.replace(' ', '') if isinstance(value, str) else value


def score(case, item):
    wrong = [name for name in SLOTS if normalise(getattr(item, name)) != normalise(case[name])]
    if [normalise(v) for v in item.excluded] != [normalise(v) for v in case['excluded']]:
        wrong.append('excluded')
    return wrong


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, default=ROOT / 'config/perception_request_cases.json')
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/models/qwen3-vl-4b')
    parser.add_argument('--output', type=Path, default=ROOT / 'tmp/request-eval/report.json')
    args = parser.parse_args(argv)
    import torch
    from transformers import AutoProcessor, Qwen3VLForConditionalGeneration
    model = Qwen3VLForConditionalGeneration.from_pretrained(
        str(args.model), dtype=torch.bfloat16, attn_implementation='sdpa').to('cuda').eval()
    processor = AutoProcessor.from_pretrained(str(args.model))
    rows, slots_total, slots_right, started = [], 0, 0, time.monotonic()
    by_group = collections.defaultdict(lambda: [0, 0])
    unsafe = []
    for index, case in enumerate(json.loads(args.cases.read_text(encoding='utf-8'))):
        messages = [{'role': 'user', 'content': [{'type': 'text', 'text': PROMPT + case['text']}]}]
        inputs = processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                               return_dict=True, return_tensors='pt').to('cuda')
        with torch.inference_mode():
            output = model.generate(**inputs, max_new_tokens=128, do_sample=False)
        reply = processor.batch_decode(output[:, inputs['input_ids'].shape[1]:], skip_special_tokens=True)[0]
        item = parse_reply(reply, f'case-{index}', 'eval', case['text'])
        if case['supported']:
            wrong = score(case, item)
            slots_total += len(SLOTS) + 1
            slots_right += len(SLOTS) + 1 - len(wrong)
        else:
            wrong = [] if item.action is None else ['executable']
            if item.action is not None:
                unsafe.append(case['text'])
        by_group[case['group']][0] += not wrong
        by_group[case['group']][1] += 1
        rows.append({'text': case['text'], 'group': case['group'], 'wrong': wrong, 'reply': reply, 'result': item.to_dict()})
        if wrong:
            print('FAIL', case['group'], case['text'], wrong, json.dumps(item.to_dict(), ensure_ascii=False)[:150], flush=True)
    summary = {'sentences': len(rows), 'sentences_right': sum(not row['wrong'] for row in rows),
               'slot_accuracy': slots_right / slots_total, 'unsupported_marked_executable': unsafe,
               'by_group': dict(by_group), 'seconds': time.monotonic() - started}
    write_json(args.output, {'schema_version': 1, 'summary': summary, 'rows': rows})
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
