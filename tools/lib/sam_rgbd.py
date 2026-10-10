"""Box-prompted SAM 2.1 masks in the original RGB pixel coordinates."""
from pathlib import Path
import time

import cv2
import numpy as np

MODEL_ID = "facebook/sam2.1-hiera-tiny"
MODEL_REVISION = "de431c4043854a71d8101e17995dfe596bf101a5"
ROOT = Path(__file__).resolve().parents[2]


class SAMMasks:
    def __init__(self, device):
        from transformers import Sam2Model, Sam2Processor

        self.device = device
        options = {"revision": MODEL_REVISION, "cache_dir": str(ROOT / "artifacts/models/huggingface")}
        print(f"Loading {MODEL_ID} on {device} ...", flush=True)
        self.processor = Sam2Processor.from_pretrained(MODEL_ID, **options)
        self.model = Sam2Model.from_pretrained(MODEL_ID, use_safetensors=True, **options).to(device).eval()

    def segment(self, rgb, boxes):
        import torch
        from PIL import Image

        start = time.monotonic()
        image = Image.fromarray(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
        inputs = self.processor(images=image, input_boxes=[boxes], return_tensors="pt").to(self.device)
        with torch.inference_mode():
            outputs = self.model(**inputs, multimask_output=False)
        masks = self.processor.post_process_masks(outputs.pred_masks.cpu(), inputs["original_sizes"].cpu())[0]
        scores = outputs.iou_scores.detach().cpu().reshape(-1).tolist()
        masks = masks[:, 0].numpy().astype(bool)
        if masks.shape != (len(boxes), *rgb.shape[:2]) or len(scores) != len(boxes):
            raise ValueError("SAM mask dimensions do not match source RGB/detection boxes")
        if not all(np.isfinite(scores)):
            raise ValueError("SAM returned a non-finite quality score")
        return list(zip(masks, scores)), time.monotonic() - start
