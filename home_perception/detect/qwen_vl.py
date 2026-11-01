"""Qwen3-VL boxes through tools/diagnostics/qwen3_vl_rgbd.py in its own process."""
import json
from pathlib import Path
import subprocess
import sys

from home_perception.detect.base import Detection

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'tools/diagnostics/qwen3_vl_rgbd.py'


def command(image, prompt, output, model, device='cuda'):
    return [sys.executable, str(SCRIPT), '--image', str(image), '--model', str(model),
            '--device', device, '--prompt', prompt, '--max-new-tokens', '1024',
            '--output', str(output)]


def parse_detections(path):
    """Qwen emits no confidence score, so score stays None."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    name = data['model']
    return [Detection(item['label'], [float(v) for v in item['bbox_xyxy']], name)
            for item in data['detections']]


def detect(image, prompt, output, model, env=None, timeout=180, device='cuda'):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    with (output / 'qwen.log').open('w', encoding='utf-8') as log:
        subprocess.run(command(image, prompt, output, model, device), cwd=ROOT, env=env, stdout=log,
                       stderr=subprocess.STDOUT, timeout=timeout, check=True)
    return parse_detections(output / 'detections.json')
