"""SAM mask + raw depth -> visible object surface, centre and OBB.

The centre describes the visible surface, not the unseen back or mass centre.
Thresholds below are provisional dev values, not validated acceptance limits.
"""
from dataclasses import dataclass

import numpy as np

from home_perception.contracts import ACCEPTED, HELD, INVALID_DEPTH, REJECTED
from home_perception.geometry.plane import backproject

MIN_MASK_QUALITY = .5
MIN_POINTS = 50
REJECT_VALID_FRACTION = .3
HOLD_VALID_FRACTION = .7
HOLD_CLUSTER_FRACTION = .8
SUPPORT_MARGIN_M = .003


@dataclass
class ObjectGeometry:
    status: str
    failure: str | None
    reason: str | None
    valid_depth_fraction: float
    point_count: int
    cluster_fraction: float | None = None
    center_xyz_m: list | None = None
    obb: dict | None = None
    support_height_m: float | None = None
    points: np.ndarray | None = None


def classify(mask_quality, valid_fraction, point_count):
    if mask_quality is not None and mask_quality < MIN_MASK_QUALITY:
        return REJECTED, None, 'low_mask_quality'
    if valid_fraction < REJECT_VALID_FRACTION or point_count < MIN_POINTS:
        return REJECTED, INVALID_DEPTH, 'insufficient_mask_depth'
    if valid_fraction < HOLD_VALID_FRACTION:
        return HELD, INVALID_DEPTH, 'partial_mask_depth'
    return ACCEPTED, None, None


def clean_cloud(points, voxel_m=.002, eps_m=.008, min_points=8):
    """Largest spatial cluster and the share of filtered points it kept."""
    import open3d as o3d
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud = cloud.voxel_down_sample(voxel_m)
    if len(cloud.points) >= 20:
        cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.)
    labels = np.asarray(cloud.cluster_dbscan(eps=eps_m, min_points=min_points))
    if not (labels >= 0).any():
        return np.empty((0, 3)), 0.
    largest = np.bincount(labels[labels >= 0]).argmax()
    keep = labels == largest
    return np.asarray(cloud.points)[keep], float(keep.mean())


def oriented_box(points):
    import open3d as o3d
    box = o3d.geometry.OrientedBoundingBox.create_from_points(
        o3d.utility.Vector3dVector(points), robust=True)
    return {'center_m': np.asarray(box.center).tolist(), 'rotation': np.asarray(box.R).tolist(),
            'extent_m': np.asarray(box.extent).tolist()}


def measure(mask, depth_m, intrinsics, plane=None, mask_quality=None):
    if mask.dtype != np.bool_ or mask.shape != depth_m.shape:
        raise ValueError('Expected boolean mask matching depth')
    pixels = int(mask.sum())
    valid = mask & (depth_m > 0)
    fraction = float(valid.sum() / pixels) if pixels else 0.
    points = backproject(depth_m, intrinsics, valid)
    height = None
    if plane is not None and len(points):
        heights = plane.height(points)
        points, heights = points[heights > SUPPORT_MARGIN_M], heights[heights > SUPPORT_MARGIN_M]
        height = float(np.percentile(heights, 95)) if len(heights) else None
    status, failure, reason = classify(mask_quality, fraction, len(points))
    cluster_fraction = None
    if status != REJECTED:
        points, cluster_fraction = clean_cloud(points)
        if len(points) < MIN_POINTS:
            status, failure, reason = REJECTED, INVALID_DEPTH, 'no_coherent_cluster'
        elif status == ACCEPTED and cluster_fraction < HOLD_CLUSTER_FRACTION:
            # A depth gap split the object, so the largest piece understates it.
            status, failure, reason = HELD, INVALID_DEPTH, 'fragmented_mask_cloud'
    if status == REJECTED:
        return ObjectGeometry(status, failure, reason, fraction, len(points), cluster_fraction)
    return ObjectGeometry(status, failure, reason, fraction, len(points), cluster_fraction,
                          points.mean(axis=0).tolist(), oriented_box(points), height, points)
