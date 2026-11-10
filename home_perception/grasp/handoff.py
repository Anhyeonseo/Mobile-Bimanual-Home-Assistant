"""Camera-frame grasp candidates for offline planning, using the explicit flat-object comparison rule."""
import math

import numpy as np

from .refine import FINGER_HEIGHT_M, MIN_CONTACT_POINTS, OBSTACLE_HEIGHT_M, SUPPORT_MARGIN_M, flat_object_grasp, refine

PRODUCER_REVISION = '8da08b05ec6509c0176b9aaf0cca8ce89a6b9c6b'
CONVENTION = 'graspnet_origin_plus_depth_to_pad_tip'


def observed_closing_section(candidate, target_points):
    """Width of the visible target in the local finger band, in the grasp camera frame.

    Keep points outside the proposed jaw opening: clipping to it would make an
    undersized candidate look like a fit. A visible top contour also contributes.
    """
    points = np.asarray(target_points, dtype=float).reshape(-1, 3)
    rotation = np.asarray(candidate['rotation_matrix'], dtype=float)
    local = (points - candidate['translation_m']) @ rotation
    depth = candidate['depth_m']
    along = local[(local[:, 0] >= -.01) & (local[:, 0] <= depth + .005)
                  & (abs(local[:, 2]) <= FINGER_HEIGHT_M / 2), 1]
    if len(along) < MIN_CONTACT_POINTS:
        return dict(status='INSUFFICIENT_SECTION', points=len(along))
    low, high = np.percentile(along, [2, 98])
    if not low < 0 < high or min(np.count_nonzero(along < 0), np.count_nonzero(along > 0)) < 3:
        return dict(status='ONE_SIDED_SECTION', points=len(along))
    return dict(status='VISIBLE_SPAN', points=len(along), low_m=float(low), high_m=float(high))


def candidate_from_row(row, source, target_points=None):
    candidate = dict(source=source, width_m=float(row[1]), depth_m=float(row[3]),
                rotation_matrix=row[4:13].reshape(3, 3).tolist(),
                translation_m=row[13:16].tolist(), score=float(row[0]))
    if target_points is not None:
        candidate['closing_section'] = observed_closing_section(candidate, target_points)
    return candidate


def observed_contact_line(candidate, target_points, radius_m=.01):
    """Observed span near the pinch line, invariant to roll about that line.

    The cylinder is a sampling region, not a gripper collision body. Its radius
    is a development observation budget; the actual hand/path still needs CAD.
    """
    points=np.asarray(target_points,dtype=float).reshape(-1,3)
    rotation=np.asarray(candidate['rotation_matrix'])
    center=np.asarray(candidate['translation_m'])+candidate['depth_m']*rotation[:,0]
    relative=points-center
    along=relative@rotation[:,1]
    distance=np.linalg.norm(relative-along[:,None]*rotation[:,1],axis=1)
    values=along[distance<=radius_m]
    evidence=dict(method='observed_contact_line',radius_m=radius_m,points=len(values))
    if len(values)<MIN_CONTACT_POINTS:
        return dict(evidence,status='INSUFFICIENT_SECTION')
    low,high=np.percentile(values,[2,98])
    if not low<0<high or min(np.count_nonzero(values<0),np.count_nonzero(values>0))<3:
        return dict(evidence,status='ONE_SIDED_SECTION')
    return dict(evidence,status='VISIBLE_SPAN',low_m=float(low),high_m=float(high))


def _finger_corridor_blocked(row, scene, plane):
    # Same observed finger corridor as flat_object_grasp, now at the proposed pose.
    local = (scene[plane.height(scene) > OBSTACLE_HEIGHT_M] - row[13:16]) @ row[4:13].reshape(3, 3)
    between_depths = (local[:, 0] > -.06) & (local[:, 0] < row[3])
    at_pads = (np.abs(local[:, 1]) > row[1]/2 - .002) & (np.abs(local[:, 1]) < row[1]/2 + .012)
    return np.count_nonzero(between_depths & at_pads & (abs(local[:, 2]) < FINGER_HEIGHT_M/2)) > 5


def _surface_variants(raw, target, scene, plane, *, longitudinal_offset_m=0.):
    """Keep the short-side pinch; aim the pad tips at the observed material on the same closing line."""
    rot = raw[4:13].reshape(3, 3)
    tip = raw[13:16] + raw[3] * rot[:, 0]
    if longitudinal_offset_m:
        # Sample the same short-side pinch at observed body sections; do not
        # move the scene, resize the target or relax any corridor checks.
        along = (target-tip) @ rot[:, 2]
        low, high = np.percentile(along, [2, 98])
        if not low+.003 <= longitudinal_offset_m <= high-.003:
            return []
        tip = tip + longitudinal_offset_m*rot[:, 2]
    local = (target - tip) @ rot
    band = (abs(local[:, 2]) <= FINGER_HEIGHT_M/2) & (abs(local[:, 1]) <= raw[1]/2)
    heights = plane.height(target[band])
    heights = heights[heights > .003]
    if not len(heights):
        return []
    tip = tip + (np.median(heights) - plane.height(tip)) * plane.normal
    results = []
    for degrees in (0, -15, 15, -30, 30):
        angle = math.radians(degrees)
        approach = math.cos(angle)*rot[:, 0] + math.sin(angle)*rot[:, 2]
        for jaw_sign in (1, -1):
            closing = jaw_sign * rot[:, 1]
            proposal = raw.copy()
            proposal[4:13] = np.column_stack((approach, closing, np.cross(approach, closing))).ravel()
            proposal[13:16] = tip - raw[3]*approach
            kept, _ = refine(proposal, target, plane, float(np.percentile(heights, 95)))
            if len(kept) and not _finger_corridor_blocked(proposal, scene, plane):
                results.append(dict(candidate_from_row(proposal, 'surface_variant', target),
                                    tilt_deg=degrees, jaw_sign=jaw_sign,
                                    longitudinal_offset_m=longitudinal_offset_m))
    return results


def flat_candidates(target_points, scene_points, plane, *, surface_variants=False):
    target_points = target_points[plane.height(target_points) > SUPPORT_MARGIN_M]
    row, reason = flat_object_grasp(target_points, scene_points, plane)
    candidates = []
    if row is not None:
        candidates.append(candidate_from_row(row[0], 'rule', target_points))
        if surface_variants:
            candidates.extend(_surface_variants(row[0], target_points, scene_points, plane))
            for offset in (-.01,.01,-.02,.02):
                candidates.extend(_surface_variants(row[0], target_points, scene_points, plane,
                                                    longitudinal_offset_m=offset))
    return dict(producer_revision=PRODUCER_REVISION, tcp_convention=CONVENTION,
                candidate_reason=reason or 'flat_rule_added', refined_grasps=candidates)
