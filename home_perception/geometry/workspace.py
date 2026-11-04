"""Tabletop workspace mask from the measured support plane."""
import cv2
import numpy as np

from home_perception.geometry.plane import backproject


def table_region(depth_m, intrinsics, plane, threshold_m=.005):
    """Convex image region of the largest connected patch on the plane."""
    valid = depth_m > 0
    heights = np.full(depth_m.shape, np.inf)
    heights[valid] = plane.height(backproject(depth_m, intrinsics, valid))
    surface = cv2.morphologyEx(np.uint8(np.abs(heights) < threshold_m), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(surface, connectivity=8)
    if count < 2:
        raise ValueError('no support surface found')
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    points = cv2.findNonZero(np.uint8(labels == largest))
    region = np.zeros(depth_m.shape, np.uint8)
    cv2.fillConvexPoly(region, cv2.convexHull(points), 1)
    return region.astype(bool)


def workspace_mask(depth_m, intrinsics, plane, object_masks=()):
    """Tall objects project past the table edge, so their masks are included."""
    region = table_region(depth_m, intrinsics, plane)
    for mask in object_masks:
        region = region | mask
    return region
