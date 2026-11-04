#!/usr/bin/env python3
"""5단계: 찾은 칸을 잘라서 맞는 물체인지 다시 묻기."""
import json
from pathlib import Path
import sys

import cv2
from PIL import Image
import torch
from transformers import AutoProcessor, Qwen3VLForConditionalGeneration

sys.path.insert(0, '.')
from home_perception.detect.verify import crop_box, parse_answer, question, resize_for_model

MODEL = 'artifacts/models/qwen3-vl-2b'
OUT = Path('tmp/tutorial')
bgr = cv2.imread('data/tabletop_rgbd_20261101/05_remote_only/color.png')
crop = resize_for_model(crop_box(bgr, json.loads((OUT / 'step2_box.json').read_text())))
cv2.imwrite(str(OUT / 'step5_crop.jpg'), crop)
image = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))

model = Qwen3VLForConditionalGeneration.from_pretrained(
    MODEL, dtype=torch.bfloat16, attn_implementation='sdpa').to('cuda').eval()
processor = AutoProcessor.from_pretrained(MODEL)
for name in ('리모컨', '컵', '계산기'):
    messages = [{'role': 'user', 'content': [{'type': 'image', 'image': image},
                                             {'type': 'text', 'text': question(name)}]}]
    inputs = processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                           return_dict=True, return_tensors='pt').to('cuda')
    with torch.inference_mode():
        generated = model.generate(**inputs, max_new_tokens=16, do_sample=False)
    answer = processor.batch_decode(generated[:, inputs['input_ids'].shape[1]:], skip_special_tokens=True)[0]
    print(f'{name}? 답변: {answer} -> 통과: {parse_answer(answer)}')
