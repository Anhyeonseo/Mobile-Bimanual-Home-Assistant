"""Support-plane fit on measured depth. Missing depth is excluded, never filled."""
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt


def backproject(depth_m, intrinsics, keep):
    """Optical-frame XYZ for pixels in keep; rectified pinhole only."""
    v, u = np.nonzero(keep)
    z = depth_m[keep]
    return np.column_stack(((u - intrinsics['ppx']) * z / intrinsics['fx'],
                            (v - intrinsics['ppy']) * z / intrinsics['fy'], z))


@dataclass
class Plane:
    """normal points toward the camera, so height() is positive above the surface."""
    normal: np.ndarray
    point: np.ndarray
    rms_m: float
    inlier_fraction: float

    def height(self, points):
        return (points - self.point) @ self.normal

    def to_dict(self):
        return {'normal': self.normal.tolist(), 'point_m': self.point.tolist(),
                'rms_m': self.rms_m, 'inlier_fraction': self.inlier_fraction}


def fit_plane(points, threshold_m=.005, iterations=300, sample_size=40000, seed=0):
    if len(points) < 3:
        raise ValueError('plane fit requires at least 3 points')
    rng = np.random.default_rng(seed)
    sample = points if len(points) <= sample_size else points[rng.choice(len(points), sample_size, replace=False)]
    best, best_count = None, 0
    for _ in range(iterations):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        count = int((np.abs((sample - a) @ normal) < threshold_m).sum())
        if count > best_count:
            best, best_count = (normal, a), count
    if best is None:
        raise ValueError('degenerate points: no plane found')
    normal, origin = best
    for _ in range(3):
        inliers = np.abs((points - origin) @ normal) < threshold_m
        origin = points[inliers].mean(axis=0)
        normal = np.linalg.eigh(np.cov((points[inliers] - origin).T))[1][:, 0]
    residual = (points - origin) @ normal
    inliers = np.abs(residual) < threshold_m
    if normal[2] > 0:
        normal = -normal
    return Plane(normal, origin, float(residual[inliers].std()), float(inliers.mean()))


def target_support_plane(depth, intr, mask, global_plane, up_in_optical):
    """Prefer a measured horizontal surface next to the mask when the global plane is elsewhere."""
    target = mask & (depth > 0)
    if not target.any():
        return None
    pixels_per_metre = max(intr['fx'], intr['fy']) / np.median(depth[target])
    distance = distance_transform_edt(~mask)
    ring = (distance >= .01 * pixels_per_metre) & (distance <= .06 * pixels_per_metre)
    nearby = backproject(depth, intr, ring & (depth > 0))
    if len(nearby) < 100:
        return None
    min_up = np.cos(np.deg2rad(20))
    if global_plane.normal @ up_in_optical >= min_up and np.mean(abs(global_plane.height(nearby)) <= .005) >= .55:
        candidate = global_plane
    else:
        try:
            candidate = fit_plane(nearby)
        except (ValueError, np.linalg.LinAlgError):
            return None
        if candidate.inlier_fraction < .65 or candidate.normal @ up_in_optical < min_up:
            return None
    if not np.isfinite(candidate.rms_m) or candidate.rms_m > .005:
        return None
    surface_height = candidate.height(backproject(depth, intr, target))
    # A distant background visible past a tabletop edge is not evidence of support.
    if np.median(surface_height) <= 0 or np.percentile(surface_height, 10) > .06:
        return None
    return candidate
