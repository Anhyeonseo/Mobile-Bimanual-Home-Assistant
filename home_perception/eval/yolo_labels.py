"""Turn perception masks into YOLO segmentation labels."""
import cv2
import numpy as np

MIN_CONTOUR_POINTS = 3


def mask_polygon(mask, epsilon_px=1.5):
    """Largest outer contour as normalised x y pairs, or None for an empty mask."""
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = cv2.approxPolyDP(max(contours, key=cv2.contourArea), epsilon_px, True)[:, 0, :]
    if len(contour) < MIN_CONTOUR_POINTS:
        return None
    height, width = mask.shape
    return (contour / [width, height]).clip(0, 1).ravel().tolist()


def label_line(class_id, polygon):
    return ' '.join([str(class_id), *(f'{value:.6f}' for value in polygon)])
