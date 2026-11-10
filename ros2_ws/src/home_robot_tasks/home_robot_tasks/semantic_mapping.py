"""RGB identification plus aligned depth shapes, using the existing DINO/SAM path.

RGB names are annotations only. Detection failures never remove obstacles.
Model loading/inference belongs to the capture worker, outside the control loop.
"""
import hashlib
import numpy as np

from .assistant.furniture import (FURNITURE_CATEGORIES, FURNITURE_PROMPT,
                                 FurnitureMemory, masked_furniture)
from .fetch import InvalidTask
from .grounding_detector import GroundingSAMDetector
from .navigation import identifier


def furniture_category(label):
    if not isinstance(label, str):
        return None
    category = label.strip().lower().rstrip('.').strip().replace(' ', '_')
    return category if category in FURNITURE_CATEGORIES else None


class RGBDFurnitureMapper:
    def __init__(self, store, context, model, segmenter, *, model_revision):
        self.revision = identifier(model_revision, 'semantic model revision')
        self.memory = FurnitureMemory(store, context)
        self.detector = GroundingSAMDetector(model, segmenter, prompt=FURNITURE_PROMPT,
                                            object_id='furniture', label_resolver=furniture_category)

    def observe(self, rgb, aligned_depth, intrinsics, *, stamp, transform, now_s):
        stamp.check(self.memory.context, now_s, 5.)
        if (not isinstance(rgb, np.ndarray) or not isinstance(aligned_depth, np.ndarray) or aligned_depth.ndim != 2 or
                rgb.shape[:2] != aligned_depth.shape or aligned_depth.size > 1280*720):
            raise InvalidTask('registered RGB depth required')
        transform.transform((0,0,0), context=self.memory.context, observation_stamp=stamp)
        detections = self.detector(rgb)
        observations = [masked_furniture(category=d.object_id, detector_score=d.confidence,
            detection_id=f'detection-{i}', mask=d.mask, depth_m=aligned_depth,
            intrinsics=intrinsics, stamp=stamp, context=self.memory.context,
            transform=transform, now_s=now_s) for i, d in enumerate(detections)]
        provenance = dict(stamp=stamp.to_dict(), model_revision=self.revision,
            rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),
            depth_sha256=hashlib.sha256(aligned_depth.tobytes()).hexdigest(),
            image_shape=list(rgb.shape), depth_dtype=str(aligned_depth.dtype),
            intrinsics=list(intrinsics), camera_to_map=np.asarray(transform.matrix).tolist(),
            source_frame=transform.source_frame, source=transform.source,
            categories=sorted({o['category'] for o in observations}),
            observation_ids=[o['observation_id'] for o in observations],
            collision_clearance_authority=False)
        previous = self.memory.store.record('semantic_capture', stamp.capture_id)
        if previous is None and len(self.memory.store.records('semantic_capture')) >= 2048:
            raise InvalidTask('semantic_capture_budget_exceeded')
        if previous is not None and previous != provenance:
            raise InvalidTask('semantic_capture_id_conflict')
        associations = [self.memory.observe(o, now_s=now_s) for o in observations]
        self.memory.store.save_record('semantic_capture', stamp.capture_id, provenance)
        return dict(provenance=provenance, associations=associations)
