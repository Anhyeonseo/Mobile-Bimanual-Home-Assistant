#!/usr/bin/env python3
"""3단계: SAM 2.1을 GPU에 올리고 박스 안의 물체 윤곽 따기."""
import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

os.environ['HF_HUB_OFFLINE'] = '1'      # 받아둔 모델만 쓴다.
sys.path.insert(0, '.')
from tools.lib.sam_rgbd import SAMMasks

OUT = Path('tmp/tutorial')
bgr = cv2.imread('data/tabletop_rgbd_20261101/05_remote_only/color.png')
box = json.loads((OUT / 'step2_box.json').read_text())
segmenter = SAMMasks('cuda')
(mask, quality), = segmenter.segment(bgr, [box])[0]
print(f'윤곽 픽셀 {mask.sum()}개, SAM 품질 예측 {quality:.2f}')
cv2.imwrite(str(OUT / 'mask_000.png'), mask.astype(np.uint8) * 255)
image = bgr.copy()
image[mask] = (.5 * image[mask] + (0, 128, 0)).astype(np.uint8)
cv2.imwrite(str(OUT / 'step3_mask.jpg'), image)
print('결과:', OUT / 'step3_mask.jpg')
