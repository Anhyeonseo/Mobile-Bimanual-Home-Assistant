"""Evaluation-only image perturbations. Not training augmentation."""
import cv2
import numpy as np

PERTURBATIONS = ('gamma_dark', 'gamma_bright', 'blur', 'noise')


def perturb(bgr, kind, seed=0):
    if kind == 'gamma_dark' or kind == 'gamma_bright':
        gamma = 2. if kind == 'gamma_dark' else .5
        table = np.uint8(np.clip((np.arange(256) / 255.)**gamma * 255, 0, 255))
        return cv2.LUT(bgr, table)
    if kind == 'blur':
        return cv2.GaussianBlur(bgr, (0, 0), 2.)
    if kind == 'noise':
        noise = np.random.default_rng(seed).normal(0, 15, bgr.shape)
        return np.uint8(np.clip(bgr + noise, 0, 255))
    raise ValueError(f'unknown perturbation {kind}')


def box_iou(a, b):
    width = min(a[2], b[2]) - max(a[0], b[0])
    height = min(a[3], b[3]) - max(a[1], b[1])
    if width <= 0 or height <= 0:
        return 0.
    overlap = width * height
    return overlap / ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - overlap)


def compare(baseline, perturbed, min_iou=.5):
    """Agreement with the unperturbed run; baseline is not ground truth."""
    matched = sum(any(box_iou(a, b) >= min_iou for b in perturbed) for a in baseline)
    return {'baseline': len(baseline), 'perturbed': len(perturbed), 'matched': matched}
