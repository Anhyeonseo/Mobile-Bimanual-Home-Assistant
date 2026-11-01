"""DINO + SAM adapter for the existing asynchronous RGB-D CapturePort.

Models are injected by the caller. Importing this module never downloads model
weights, opens a camera or touches a robot. Instance identity remains a separate
semantic grounding decision; all detections are returned, including ambiguity.
"""
import numpy as np

from .capture_port import ImageDetection
from .fetch import InvalidTask
from .navigation import identifier, number


class GroundingSAMDetector:
    def __init__(self, model, segmenter, *, prompt, object_id, minimum_mask_quality=.5):
        if not isinstance(prompt, str) or not prompt.strip() or len(prompt) > 256:
            raise InvalidTask('bounded text prompt required')
        if not callable(getattr(model, 'detect', None)) or not callable(getattr(segmenter, 'segment', None)):
            raise InvalidTask('explicit loaded detector and segmenter required')
        self.model, self.segmenter, self.prompt = model, segmenter, prompt.strip()
        self.object_id = identifier(object_id, 'query object id')
        self.quality = number(minimum_mask_quality, 'mask threshold')
        if not 0 <= self.quality <= 1:
            raise InvalidTask('mask threshold outside [0,1]')

    def __call__(self, rgb):
        if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or not 1 <= min(rgb.shape[:2]) <= max(rgb.shape[:2]) <= 8192:
            raise InvalidTask('bounded uint8 RGB image required')
        # Existing GroundingRGBD and SAMMasks accept OpenCV BGR. CapturePort is RGB.
        bgr = rgb[..., ::-1].copy()
        detections, _ = self.model.detect(bgr, self.prompt)
        if not isinstance(detections, list) or len(detections) > 64:
            raise InvalidTask('bounded detection list required')
        if not detections:
            return []
        h, w = rgb.shape[:2]
        boxes = []
        for value in detections:
            if not isinstance(value, dict) or not isinstance(value.get('box_xyxy'), (list, tuple)) or len(value['box_xyxy']) != 4:
                raise InvalidTask('detector boxes required')
            a, b, c, d = [number(v, 'detector pixel') for v in value['box_xyxy']]
            if not 0 <= a < c <= w or not 0 <= b < d <= h:
                raise InvalidTask('detector box outside source image')
            # Detectors commonly use an exclusive right/bottom image boundary.
            # CapturePort corners are actual pixel centres.
            c, d = min(c, w-1), min(d, h-1)
            if not a < c or not b < d:
                raise InvalidTask('detector box has no pixel interior')
            if not 0 <= number(value.get('score'), 'detector score') <= 1:
                raise InvalidTask('detector score outside [0,1]')
            boxes.append([a,b,c,d])
        segmented, _ = self.segmenter.segment(bgr, boxes)
        if len(segmented) != len(detections):
            raise InvalidTask('one mask per detection required')
        results = []
        for detection, (mask, quality), (a,b,c,d) in zip(detections, segmented, boxes):
            if not isinstance(mask, np.ndarray) or mask.dtype != np.bool_ or mask.shape != (h,w):
                raise InvalidTask('aligned boolean SAM mask required')
            quality = number(quality, 'mask quality')
            if not 0 <= quality <= 1:
                raise InvalidTask('invalid mask quality')
            if quality < self.quality or not mask.any():
                continue
            # Box orientation is not claimed as a physical grasp orientation.
            results.append(ImageDetection(self.object_id, detection['score'],
                ((a,b),(c,b),(c,d),(a,d)), mask.copy()))
        return results
