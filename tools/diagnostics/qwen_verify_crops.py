#!/usr/bin/env python3
"""Ask Qwen3-VL whether each saved crop shows the named object."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from home_perception.detect.verify import question


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('jobs', type=Path, help='JSON list of {"crop": path, "name": object name}')
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--device', choices=['cuda', 'cpu'], default='cuda')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import torch
    from PIL import Image
    from transformers import AutoProcessor, Qwen3VLForConditionalGeneration
    model = Qwen3VLForConditionalGeneration.from_pretrained(
        str(args.model), dtype=torch.float32 if args.device == 'cpu' else torch.bfloat16,
        attn_implementation='sdpa').to(args.device).eval()
    processor = AutoProcessor.from_pretrained(str(args.model))
    answers = []
    for job in json.loads(args.jobs.read_text(encoding='utf-8')):
        messages = [{'role': 'user', 'content': [{'type': 'image', 'image': Image.open(job['crop']).convert('RGB')},
                                                 {'type': 'text', 'text': question(job['name'])}]}]
        inputs = processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                               return_dict=True, return_tensors='pt').to(args.device)
        with torch.inference_mode():
            generated = model.generate(**inputs, max_new_tokens=16, do_sample=False)
        answers.append(processor.batch_decode(generated[:, inputs['input_ids'].shape[1]:], skip_special_tokens=True)[0])
    args.output.write_text(json.dumps(answers, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
