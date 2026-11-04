#!/usr/bin/env python3
"""2단계: Qwen 모델을 GPU에 올리고 물체 찾기."""
import json
from pathlib import Path
import re

from PIL import Image, ImageDraw
import torch
from transformers import AutoProcessor, Qwen3VLForConditionalGeneration

MODEL = 'artifacts/models/qwen3-vl-2b'
PROMPT = '리모컨을 찾아줘.'     # '컵을 찾아줘.'로 바꾸면 없는 물체에도 박스를 내는 것을 볼 수 있다.
OUT = Path('tmp/tutorial')

print('GPU:', torch.cuda.get_device_name(0))
model = Qwen3VLForConditionalGeneration.from_pretrained(
    MODEL, dtype=torch.bfloat16, attn_implementation='sdpa').to('cuda').eval()
processor = AutoProcessor.from_pretrained(MODEL)
print(f'GPU 메모리 사용: {torch.cuda.memory_allocated() / 1e9:.1f}GB')

image = Image.open('data/tabletop_rgbd_20261101/05_remote_only/color.png').convert('RGB')
question = (PROMPT + '\nReturn only a JSON array. Each item has "label" and "bbox_2d": [x1,y1,x2,y2] '
            'with integers normalized to 0..1000. Return [] if nothing matches.')
messages = [{'role': 'user', 'content': [{'type': 'image', 'image': image}, {'type': 'text', 'text': question}]}]
inputs = processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                       return_dict=True, return_tensors='pt').to('cuda')
with torch.inference_mode():
    generated = model.generate(**inputs, max_new_tokens=256, do_sample=False)
answer = processor.batch_decode(generated[:, inputs['input_ids'].shape[1]:], skip_special_tokens=True)[0]
print('모델 답변 원문:', answer)

# 좌표는 0~1000 비율이므로 실제 픽셀로 바꾼다.
items = json.loads(re.sub(r'^```(json)?|```$', '', answer.strip(), flags=re.M))
if not items:
    raise SystemExit('박스 없음')
x1, y1, x2, y2 = items[0]['bbox_2d']
box = [x1 * image.width / 1000, y1 * image.height / 1000, x2 * image.width / 1000, y2 * image.height / 1000]
ImageDraw.Draw(image).rectangle(box, outline='red', width=3)
image.save(OUT / 'step2_box.jpg')
(OUT / 'step2_box.json').write_text(json.dumps(box))
print('박스:', [round(v) for v in box], '결과:', OUT / 'step2_box.jpg')
