"""Geometry checks and ranking on top of GraspNet candidates.

Arrays use the graspnetAPI layout: score, width, height, depth, 3x3 rotation,
translation, object id. Rotation columns are approach, closing, third axis.
These legacy filters model a generic parallel gripper, not the installed SO101.
They remain a comparison and the explicit flat-rule baseline. They must not
veto GraspNet proposals before the robot-specific CAD/IK/path checks.
"""
import numpy as np

FINGER_HEIGHT_M = .02
MIN_CONTACT_POINTS = 15
MAX_CENTER_OFFSET_M = .025
FLAT_OBJECT_HEIGHT_M = .03
FLAT_MAX_TILT_DEG = 45.
TALL_MAX_TILT_DEG = 100.
MIN_ELONGATION = 2.
TIP_CLEARANCE_M = .001
SUPPORT_MARGIN_M = .003
OBSTACLE_HEIGHT_M = .005


def parts(array):
    array = np.atleast_2d(array)
    return array[:, 0], array[:, 1], array[:, 3], array[:, 4:13].reshape(-1, 3, 3), array[:, 13:16]


def tilt_deg(rotations, normal):
    """0 means straight down onto the table, 90 means level, above 90 means upward."""
    return np.degrees(np.arccos(np.clip(-(rotations[:, :, 0] @ normal), -1, 1)))


def tip_heights(array, plane):
    """Lowest fingertip corner above the support plane."""
    _, widths, depths, rotations, translations = parts(array)
    lowest = np.full(len(widths), np.inf)
    for side in (-1, 1):
        for up in (-1, 1):
            corners = (translations + rotations[:, :, 0] * depths[:, None]
                       + rotations[:, :, 1] * (side * widths[:, None] / 2)
                       + rotations[:, :, 2] * (up * FINGER_HEIGHT_M / 2))
            lowest = np.minimum(lowest, plane.height(corners))
    return lowest


def between_fingers(array, points):
    """Target points inside each closing volume, and how centred they sit."""
    _, widths, depths, rotations, translations = parts(array)
    counts, offsets = np.zeros(len(widths), int), np.full(len(widths), np.inf)
    for index in range(len(widths)):
        local = (points - translations[index]) @ rotations[index]
        inside = ((local[:, 0] > -.01) & (local[:, 0] < depths[index] + .005)
                  & (np.abs(local[:, 1]) < widths[index] / 2)
                  & (np.abs(local[:, 2]) < FINGER_HEIGHT_M / 2))
        counts[index] = inside.sum()
        if counts[index]:
            offsets[index] = abs(local[inside, 1].mean())
    return counts, offsets


def nms(array, translation_m=.03, rotation_deg=30.):
    """Greedy duplicate removal; array must already be in rank order."""
    kept = []
    _, _, _, rotations, translations = parts(array)
    for index in range(len(array)):
        duplicate = False
        for other in kept:
            trace = np.trace(rotations[index].T @ rotations[other])
            angle = np.degrees(np.arccos(np.clip((trace - 1) / 2, -1, 1)))
            if (np.linalg.norm(translations[index] - translations[other]) < translation_m
                    and angle < rotation_deg):
                duplicate = True
                break
        if not duplicate:
            kept.append(index)
    return array[kept]


def refine(array, target_points, plane, object_height_m, max_width_m=.1, top_k=20):
    """Keep candidates that hold the object and clear the table, best first."""
    array = np.atleast_2d(array).reshape(-1, 17)
    if not len(array):
        return array, {}
    scores, widths, _, rotations, translations = parts(array)
    target = target_points[plane.height(target_points) > SUPPORT_MARGIN_M]
    counts, offsets = between_fingers(array, target)
    tilts = tilt_deg(rotations, plane.normal)
    limit = FLAT_MAX_TILT_DEG if object_height_m <= FLAT_OBJECT_HEIGHT_M else TALL_MAX_TILT_DEG
    checks = {'no_object_between_fingers': counts < MIN_CONTACT_POINTS,
              'off_centre': offsets > MAX_CENTER_OFFSET_M,
              'below_table': tip_heights(array, plane) < TIP_CLEARANCE_M,
              'approach_angle': tilts > limit,
              'too_wide': widths > max_width_m}
    rejected = {name: int(mask.sum()) for name, mask in checks.items()}
    keep = ~np.any(list(checks.values()), axis=0)
    centre = target.mean(axis=0) if len(target) else translations.mean(axis=0)
    rank = (scores + .3 * np.minimum(counts / 200, 1)
            - 2 * np.linalg.norm(translations - centre, axis=1))
    order = np.argsort(-rank[keep])
    return nms(array[keep][order])[:top_k], rejected


def flat_object_grasp(target_points, scene_points, plane, margin_m=.01, max_width_m=.1):
    """Straight-down pinch across the short side of a low flat object.

    Returns (array, reason). This depends on absolute table height, which is
    not yet checked against a ruler, and sees only surfaces facing the camera.
    """
    normal = plane.normal
    target = target_points[plane.height(target_points) > SUPPORT_MARGIN_M]
    if len(target) < MIN_CONTACT_POINTS:
        return None, 'too_few_points'
    height = float(np.percentile(plane.height(target), 95))
    if height > FLAT_OBJECT_HEIGHT_M:
        return None, 'not_flat'
    flat = target - np.outer((target - plane.point) @ normal, normal)
    middle = flat.mean(axis=0)
    values, vectors = np.linalg.eigh(np.cov((flat - middle).T))
    # values[0] is the flattened normal; the other two are the spread on the table.
    if values[2] < MIN_ELONGATION ** 2 * values[1]:
        return None, 'not_elongated'
    long_axis = vectors[:, 2]
    short_axis = np.cross(normal, long_axis)
    across = np.percentile((flat - middle) @ short_axis, [1, 99])
    along = np.percentile((flat - middle) @ long_axis, [1, 99])
    width = float(across[1] - across[0]) + 2 * margin_m
    if width > max_width_m:
        return None, 'too_wide'
    depth = .02
    centre = (middle + short_axis * across.mean() + long_axis * along.mean()
              + normal * (TIP_CLEARANCE_M * 2 + depth))
    approach = -normal
    third = np.cross(approach, short_axis)
    rotation = np.column_stack((approach, short_axis, third))
    array = np.zeros((1, 17))
    array[0, :4] = 0, width, FINGER_HEIGHT_M, depth
    array[0, 4:13] = rotation.ravel()
    array[0, 13:16] = centre
    array[0, 16] = -1
    # A concave shape can leave the middle of its bounding extent empty.
    counts, offsets = between_fingers(array, target)
    if counts[0] < MIN_CONTACT_POINTS or offsets[0] > MAX_CENTER_OFFSET_M:
        return None, 'no_object_between_fingers'
    # Fingers must not land on neighbouring objects seen by the camera.
    above = scene_points[plane.height(scene_points) > OBSTACLE_HEIGHT_M]
    local = (above - centre) @ rotation
    finger = (np.abs(local[:, 2]) < FINGER_HEIGHT_M / 2) & (local[:, 0] > -.06) & (local[:, 0] < depth)
    outside = (np.abs(local[:, 1]) > width / 2 - .002) & (np.abs(local[:, 1]) < width / 2 + .012)
    if (finger & outside).sum() > 5:
        return None, 'finger_collision'
    return array, None
