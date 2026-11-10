"""Explicit, capture-bound contact adjustment for an operator comparison trial.

This is not a calibration or a default grasp policy. The scene and all robot
transforms stay unchanged; the resulting contact must pass normal path checks.
"""
import copy

import numpy as np

from .balance import contact_centrality, footprint_reference
from .contact_orientation import footprint_short_axis
from .handoff import observed_contact_line


def apply_trial_offset(group, points, plane, base_from_optical, distance_m, opening_range):
    if not np.isfinite(distance_m) or not 0 <= distance_m <= .03:
        raise ValueError('trial contact offset must be between 0 and 0.03 m')
    result = copy.deepcopy(group)
    if distance_m == 0:
        return result
    transform = np.asarray(base_from_optical, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1.)):
        raise ValueError('rigid base-from-optical transform required')
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    reference = footprint_reference(points, plane.normal)
    short_axis = footprint_short_axis(reference)
    if short_axis is None:
        raise ValueError('trial offset requires an observed elongated object')
    axis = np.cross(plane.normal, short_axis)
    axis /= np.linalg.norm(axis)
    forward = float(transform[0, :3] @ axis)
    if abs(forward) < 1e-6:
        raise ValueError('object length has no resolved forward/backward direction')
    if forward < 0:
        axis = -axis
    low, high = opening_range
    accepted, rejected = [], {}
    for original in group['refined_grasps']:
        candidate = copy.deepcopy(original)
        rotation = np.asarray(candidate['rotation_matrix'])
        tip = np.asarray(candidate['translation_m']) + candidate['depth_m'] * rotation[:, 0]
        # Centre along the footprint length first; preserve the original height,
        # closing-axis position and orientation. Then apply the explicit trial bias.
        centering = float((reference['center'] - tip) @ axis)
        shift = (centering + distance_m) * axis
        candidate['translation_m'] = (np.asarray(candidate['translation_m']) + shift).tolist()
        section = observed_contact_line(candidate, points)
        reason = None
        if section['status'] != 'VISIBLE_SPAN':
            reason = 'shifted_target_section_unresolved'
        else:
            span = section['high_m'] - section['low_m']
            opening = 2 * max(-section['low_m'], section['high_m'])
            if not low <= span <= high or not low <= opening <= high:
                reason = 'shifted_section_outside_robot_opening_range'
        if reason:
            rejected[reason] = rejected.get(reason, 0) + 1
            continue
        candidate.update(closing_section=section, width_m=float(opening),
                         opening_source='observed_contact_line',
                         balance_proxy=contact_centrality(tip + shift, rotation[:, 1], reference),
                         contact_position_variant='operator_trial_longitudinal_offset',
                         trial_contact_offset=dict(
                             requested_m=float(distance_m), direction='object_length_toward_base_positive_x',
                             optical_axis=axis.tolist(), base_axis=(transform[:3, :3] @ axis).tolist(),
                             original_tip_optical_m=tip.tolist(), centering_shift_m=centering,
                             total_shift_optical_m=shift.tolist(), calibration_changed=False))
        accepted.append(candidate)
    result['refined_grasps'] = accepted
    if 'counts' in result:
        result['counts']['refined'] = len(accepted)
    result['trial_contact_offset'] = dict(
        kind='operator_requested_single_trial', requested_m=float(distance_m),
        direction='object_length_toward_base_positive_x', calibration_changed=False,
        input_candidates=len(group['refined_grasps']), selected=len(accepted),
        rejected_by_evidence=rejected, requires_full_robot_path_check=True)
    return result
