"""Bounded, network-free contact proposals for offline SO101 experiments.

Visible spans are observation proxies, not force closure or reachability proof.
The fixed face is bound to the low end of each signed closing line downstream.
No robot transform, reachability map or hardware connection is assumed here.
"""
from time import perf_counter

import numpy as np

from .balance import footprint_reference, contact_centrality
from .contact_orientation import footprint_short_axis
from .handoff import CONVENTION, candidate_from_row, observed_contact_line


def candidates(target_points, plane, opening_range, *, limit=24, entry_clearance_m=.004):
    start = perf_counter()
    points = np.asarray(target_points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('finite Nx3 target points required')
    if type(limit) is not int or not 1 <= limit <= 64:
        raise ValueError('limit must be 1..64')
    low, high = opening_range
    if not 0 < low < high < .5 or not .004 <= entry_clearance_m <= .02:
        raise ValueError('valid measured opening range and 4..20 mm clearance required')
    normal = np.asarray(plane.normal, float)
    if normal.shape != (3,) or not np.isfinite(normal).all() or not np.isclose(np.linalg.norm(normal), 1.):
        raise ValueError('unit support normal required')
    if not np.isfinite(plane.point).all():
        raise ValueError('finite support point required')
    points = points[(points[:, 2] > 0) & (plane.height(points) > .003)]
    reference = footprint_reference(points, normal)
    rejected = {}
    pool = []
    evaluated = 0
    if reference is not None and len(points) >= 20:
        short = footprint_short_axis(reference)
        if short is None:
            short = np.eye(3)[np.argmin(abs(normal))]
            short -= normal * (short @ normal)
            short /= np.linalg.norm(short)
        tangent = np.cross(normal, short)
        # Include multiple directions for round footprints and non-box targets.
        for yaw in (0, -30, 30, 90):
            angle = np.deg2rad(yaw)
            axis = np.cos(angle)*short + np.sin(angle)*tangent
            lateral = np.cross(normal, axis)
            extent = np.percentile((points-reference['center']) @ lateral, [10, 90])
            for fraction in (0., -.35, .35):
                tip = reference['center'] + fraction*(extent[1]-extent[0])*lateral
                band = abs((points-tip) @ lateral) <= .01
                if np.count_nonzero(band) < 20:
                    rejected['sparse_local_band'] = rejected.get('sparse_local_band', 0)+1
                    continue
                local = points[band]
                tip += (np.median(plane.height(local))-plane.height(tip))*normal
                # Centre on the observed span, not on a symmetric robot jaw.
                ends = np.percentile((local-tip) @ axis, [2, 98])
                tip += np.mean(ends)*axis
                for tilt in (0, -25, 25):
                    a = np.deg2rad(tilt)
                    approach = -np.cos(a)*normal + np.sin(a)*lateral
                    for sign in (1, -1):
                        evaluated += 1
                        closing = sign*axis
                        rotation = np.column_stack((approach, closing, np.cross(approach, closing)))
                        row = np.zeros(17)
                        row[:4] = [1., .01, .02, .02]
                        row[4:13] = rotation.ravel()
                        row[13:16] = tip - row[3]*approach
                        candidate = candidate_from_row(row, 'so101-geometric')
                        section = observed_contact_line(candidate, points)
                        reason = None
                        if section['status'] != 'VISIBLE_SPAN':
                            reason = 'target_section_unresolved'
                        else:
                            span = section['high_m']-section['low_m']
                            if not low <= span <= high-entry_clearance_m:
                                reason = 'span_or_entry_outside_robot_range'
                        if reason:
                            rejected[reason] = rejected.get(reason, 0)+1
                            continue
                        balance = contact_centrality(tip, closing, reference)
                        score = 1.-.5*balance['normalized_offset']**2-.1*abs(tilt)/25-.1*abs(yaw)/90
                        candidate.update(width_m=float(span), score=float(score), closing_section=section,
                            entry_gap_m=float(span+entry_clearance_m), balance_proxy=balance,
                            fixed_contact_optical_m=(tip+section['low_m']*closing).tolist(),
                            moving_contact_optical_m=(tip+section['high_m']*closing).tolist(),
                            tilt_deg=tilt, jaw_sign=sign, yaw_deg=yaw,
                            robot_feasibility_checked=False, motion_authorized=False)
                        pool.append(candidate)
    pool.sort(key=lambda c: -c['score'])
    # Keep both fixed-finger sides and tilt alternatives; bounded at 72 seeds.
    kept = pool[:limit]
    return dict(tcp_convention=CONVENTION, producer_revision='so101-geometric-v1',
        candidate_reason='so101_proposals_require_robot_check' if kept else 'no_observed_so101_proposal',
        refined_grasps=kept, counts=dict(generated=evaluated, refined=len(kept)),
        proposal_selection=dict(evaluated_contact_variants=evaluated, eligible=len(pool),
            rejected_by_evidence=rejected, selected=len(kept), maximum_proposals=limit,
            reachability_checked=False, collision_checked=False, force_closure_verified=False,
            motion_authorized=False), candidate_generation_seconds=perf_counter()-start)
