#!/usr/bin/env python3
"""Open3D RGB-D sample -> natural-language Qwen3-VL grounding experiment."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import time


def parse_boxes(text, width, height):
    """Validate Qwen's normalized xyxy boxes before converting to pixels."""
    text = text.strip()
    if text.startswith('```'):
        text = text.split('\n', 1)[1].rsplit('```', 1)[0].strip()
    items = json.loads(text)
    if not isinstance(items, list):
        raise ValueError('Expected a JSON array')
    result = []
    for item in items:
        box = item['bbox_2d']
        if (len(box) != 4 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                               or not math.isfinite(v) or not 0 <= v <= 1000 for v in box)
                or box[0] >= box[2] or box[1] >= box[3]):
            raise ValueError(f'Invalid normalized xyxy box: {box}')
        label = item['label']
        if not isinstance(label, str) or not label.strip():
            raise ValueError('Missing object label')
        pixels = [box[0]*width/1000, box[1]*height/1000,
                  box[2]*width/1000, box[3]*height/1000]
        result.append({'label': label, 'bbox_1000': box, 'bbox_xyxy': pixels})
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sample', choices=['tum', 'sun', 'redwood'], default='tum')
    p.add_argument('--image', type=Path, help='Use a local RGB image instead of downloading a sample')
    p.add_argument('--prompt', default='책상 위에 있는 물체들을 각각 찾아줘.')
    p.add_argument('--model', default='Qwen/Qwen3-VL-2B-Instruct')
    p.add_argument('--revision', default='89644892e4d85e24eaac8bacfd4f463576704203')
    p.add_argument('--device', choices=['cpu', 'cuda'], default='cuda')
    p.add_argument('--threads', type=int, default=8)
    p.add_argument('--max-new-tokens', type=int, default=512)
    p.add_argument('--prepare-only', action='store_true')
    p.add_argument('--output', type=Path, default=Path('tmp/qwen3-vl-rgbd'))
    args = p.parse_args()
    from PIL import Image, ImageDraw, ImageFont
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    source = {'dataset': None}
    if args.image:
        color = args.image.resolve()
    else:
        import open3d as o3d
        root = str(Path('artifacts/datasets/open3d').resolve())
        if args.sample == 'tum':
            data = o3d.data.SampleTUMRGBDImage(data_root=root)
            color, depth = Path(data.color_path), Path(data.depth_path)
            rgbd = o3d.geometry.RGBDImage.create_from_tum_format(
                o3d.io.read_image(str(color)), o3d.io.read_image(str(depth)),
                convert_rgb_to_intensity=False)
        elif args.sample == 'sun':
            data = o3d.data.SampleSUNRGBDImage(data_root=root)
            color, depth = Path(data.color_path), Path(data.depth_path)
            rgbd = o3d.geometry.RGBDImage.create_from_sun_format(
                o3d.io.read_image(str(color)), o3d.io.read_image(str(depth)),
                convert_rgb_to_intensity=False)
        else:
            data = o3d.data.SampleRedwoodRGBDImages(data_root=root)
            color, depth = Path(data.color_paths[0]), Path(data.depth_paths[0])
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.io.read_image(str(color)), o3d.io.read_image(str(depth)),
                convert_rgb_to_intensity=False)
        import numpy as np
        depth_m = np.asarray(rgbd.depth)
        np.save(out / 'depth_m.npy', depth_m)
        v = np.clip(depth_m / 3.0, 0, 1)
        preview = np.stack([255*v, 255*(1-abs(2*v-1)), 255*(1-v)], axis=-1).astype('uint8')
        preview[depth_m == 0] = 0
        Image.fromarray(preview).save(out / 'depth_preview.png')
        shutil.copyfile(depth, out / 'depth_original.png')
        source = {'dataset': type(data).__name__, 'color_path': str(color),
                  'depth_path': str(depth), 'depth_conversion': 'Open3D dataset-specific RGBD conversion',
                  'depth_truncation_m': 3.0}
    image = Image.open(color).convert('RGB')
    image.save(out / 'rgb.png')
    source.update({'width': image.width, 'height': image.height,
                   'rgb_sha256': hashlib.sha256((out / 'rgb.png').read_bytes()).hexdigest()})
    (out / 'source.json').write_text(json.dumps(source, indent=2), encoding='utf-8')
    print(f'Prepared RGB {image.size}: {out}', flush=True)
    if args.prepare_only:
        return

    import torch
    from transformers import AutoProcessor, Qwen3VLForConditionalGeneration
    torch.set_num_threads(args.threads)
    cache = 'artifacts/models/huggingface'
    start = time.perf_counter()
    model = Qwen3VLForConditionalGeneration.from_pretrained(
        args.model, revision=args.revision, cache_dir=cache,
        dtype=torch.float32 if args.device == 'cpu' else torch.bfloat16,
        attn_implementation='sdpa').to(args.device).eval()
    processor = AutoProcessor.from_pretrained(args.model, revision=args.revision, cache_dir=cache)
    load_seconds = time.perf_counter() - start
    instruction = (args.prompt + '\nReturn only a JSON array of visible matching objects. '
                   'Each object must have "label" (English name) and "bbox_2d": [x1,y1,x2,y2]. '
                   'Coordinates must be integers normalized to 0..1000, relative to the full image. '
                   'Use top-left and bottom-right corners. Return [] if no objects match.')
    messages = [{'role': 'user', 'content': [
        {'type': 'image', 'image': image}, {'type': 'text', 'text': instruction}]}]
    inputs = processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                          return_dict=True, return_tensors='pt').to(args.device)
    print(f'Model loaded in {load_seconds:.1f}s. Generating...', flush=True)
    start = time.perf_counter()
    with torch.inference_mode():
        generated = model.generate(**inputs, max_new_tokens=args.max_new_tokens, do_sample=False)
    seconds = time.perf_counter() - start
    tokens = generated[:, inputs['input_ids'].shape[1]:]
    raw = processor.batch_decode(tokens, skip_special_tokens=True)[0]
    (out / 'response.txt').write_text(raw, encoding='utf-8')
    result = {'model': args.model, 'revision': getattr(model.config, '_commit_hash', None) or args.revision,
              'prompt': args.prompt, 'instruction': instruction, 'device': args.device,
              'load_seconds': load_seconds, 'inference_seconds': seconds,
              'generated_tokens': tokens.shape[1], 'source': source,
              'input_modalities': ['RGB', 'text'], 'raw_response': raw}
    try:
        result['detections'] = parse_boxes(raw, image.width, image.height)
    except (ValueError, KeyError, TypeError) as exc:
        result.update(status='INVALID_MODEL_OUTPUT', error=str(exc))
        (out / 'detections.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        raise
    result['status'] = 'OK' if result['detections'] else 'NO_DETECTIONS'
    draw = ImageDraw.Draw(image)
    font_path = Path('/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf')
    font = ImageFont.truetype(str(font_path), 14) if font_path.exists() else ImageFont.load_default()
    for item in result['detections']:
        box = item['bbox_xyxy']
        draw.rectangle(box, outline='red', width=3)
        draw.text((box[0], max(0, box[1]-16)), item['label'], font=font,
                  fill='red', stroke_width=1, stroke_fill='white')
    image.save(out / 'detections.jpg')
    (out / 'detections.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
