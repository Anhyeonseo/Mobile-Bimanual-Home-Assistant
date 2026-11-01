"""Grounding DINO through the existing tools/lib/grounding_rgbd.py detector."""
from home_perception.detect.base import Detection


class DinoDetector:
    def __init__(self, device='auto', **options):
        from tools.lib.grounding_rgbd import GroundingRGBD
        self.model = GroundingRGBD(device=device, **options)

    def detect(self, bgr, prompt):
        from tools.lib.grounding_rgbd import MODEL_ID
        detections, _ = self.model.detect(bgr, prompt)
        return [Detection(item['label'], item['box_xyxy'], MODEL_ID, item['score'])
                for item in detections]
