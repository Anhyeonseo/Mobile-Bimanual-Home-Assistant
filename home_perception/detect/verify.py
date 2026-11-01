"""Second look at a detected crop, so a box alone never means the object exists."""
import re

import cv2

CROP_PADDING = .25


def crop_box(bgr, box, padding=CROP_PADDING):
    """Padded crop so the model sees the object edges and a little context."""
    height, width = bgr.shape[:2]
    x1, y1, x2, y2 = box
    pad_x, pad_y = (x2 - x1) * padding, (y2 - y1) * padding
    left, right = max(0, int(x1 - pad_x)), min(width, int(x2 + pad_x) + 1)
    top, bottom = max(0, int(y1 - pad_y)), min(height, int(y2 + pad_y) + 1)
    if right <= left or bottom <= top:
        raise ValueError('empty crop')
    return bgr[top:bottom, left:right]


def question(name):
    return (f'이 사진의 가운데에 있는 물체가 {name} 맞나요? 다른 물체이거나, 물체가 없거나, '
            "확실하지 않으면 '아니오'라고 답하세요. '예' 또는 '아니오'로만 답하세요.")


def parse_answer(text):
    """Only a clear yes passes; anything unclear counts as no."""
    text = text.strip().lower()
    yes = bool(re.match(r"['\"]?(예|네|yes)\b", text))
    no = bool(re.search(r'아니|no\b', text))
    return yes and not no


def resize_for_model(crop, minimum=224):
    """Very small crops are enlarged; pixels are not otherwise altered."""
    shortest = min(crop.shape[:2])
    if shortest >= minimum:
        return crop
    scale = minimum / shortest
    return cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)


def split_verified(detections, answers):
    """Boxes with a clear yes stay; the rest never reach segmentation."""
    if len(detections) != len(answers):
        raise ValueError('one answer per detection required')
    kept = [item for item, answer in zip(detections, answers) if parse_answer(answer)]
    rejected = [item for item, answer in zip(detections, answers) if not parse_answer(answer)]
    return kept, rejected


def missing_objects(expected, kept, names):
    """Requested objects without a verified box. names maps object to Korean."""
    labels = [item.label for item in kept]
    return [name for name in dict.fromkeys(expected)
            if not any(names[name] in label or name in label.lower() for label in labels)]
