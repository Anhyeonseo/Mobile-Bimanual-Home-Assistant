"""Observed grasp proposals, before any robot-specific feasibility decision.

GraspNet's parallel-jaw body is not the installed SO101 hand. This layer may
reject malformed poses, absent target evidence and an out-of-range opening;
robot geometry, approach angle, IK and whole-path collision belong downstream.
The local section is observation evidence, not a collision shape or force proof.
"""
import numpy as np

from .handoff import candidate_from_row, observed_contact_line
from .balance import footprint_reference, contact_centrality
from .contact_orientation import footprint_short_axis, orientation_variants


def contact_positions(row, reference, *, enabled):
    """Add a longitudinally centred pinch; every new section is remeasured.

    Preserve the network orientation and support-relative height. A silhouette
    centroid is not a measured COM, and a concave/empty centre may be rejected
    by the same observed-section check as the original network proposal.
    """
    if enabled and footprint_short_axis(reference) is not None:
        rotation=row[4:13].reshape(3,3)
        lateral=np.cross(reference['normal'],rotation[:,1])
        norm=np.linalg.norm(lateral)
        if norm>1e-6:
            lateral/=norm
            tip=row[13:16]+row[3]*rotation[:,0]
            shift=float((reference['center']-tip)@lateral)
            if abs(shift)>.001:
                centred=row.copy();centred[13:16]+=shift*lateral
                yield centred,'observed_longitudinal_center',shift
    yield row,'network_contact_position',0.


def contact_heights(row, points, plane):
    """Sample a measured local surface height as well as the network contact.

    Preserve planar location and closing direction. No nominal object height or
    image-specific offset is inserted; each new section is measured again.
    """
    rotation=row[4:13].reshape(3,3)
    tip=row[13:16]+row[3]*rotation[:,0]
    across=rotation[:,1]-plane.normal*(rotation[:,1]@plane.normal)
    if np.linalg.norm(across)>1e-6:
        across/=np.linalg.norm(across)
        lateral=np.cross(plane.normal,across)
        delta=points-tip;heights=plane.height(points)
        region=(abs(delta@lateral)<=.01)&(abs(delta@across)<=.06)&(heights>.003)
        region&=abs(heights-plane.height(tip))<=.02
        if np.count_nonzero(region)>=15:
            shift=float(np.median(heights[region])-plane.height(tip))
            if abs(shift)>.001:
                proposal=row.copy();proposal[13:16]+=shift*plane.normal
                yield proposal,'observed_surface_height',shift
    yield row,'network_contact',0.


def select(rows, target_points, plane, opening_range, *, limit=32, surface_heights=True,
           straight_across=True, centered_contacts=False):
    rows = np.asarray(rows, dtype=float).reshape(-1, 17)
    if type(limit) is not int or not 1 <= limit <= 64:
        raise ValueError('bounded proposal count required')
    low, high = opening_range
    if not 0 < low < high < .5:
        raise ValueError('robot opening range required')
    target_points=np.asarray(target_points,dtype=float).reshape(-1,3)
    balance_reference=footprint_reference(target_points,plane.normal)
    short_axis=footprint_short_axis(balance_reference) if straight_across else None
    rejected = {};evaluated=0
    eligible = []
    for index, row in enumerate(rows):
        row=row.copy()
        rotation = row[4:13].reshape(3, 3)
        if (not np.isfinite(row).all() or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-5)
                or not 0 < row[3] <= .05 or row[15] <= 0 or row[1] <= 0):
            rejected['invalid_pose']=rejected.get('invalid_pose',0)+1
            continue
        # Decoded float32 rotations have small orthogonality roundoff.
        # Project only already-near-rigid matrices onto SO(3), once here.
        u,_,vt=np.linalg.svd(rotation);rotation=u@vt;row[4:13]=rotation.ravel()
        variants=((p,v,h,o,c,s) for oriented,o in orientation_variants(row,short_axis)
            for positioned,c,s in contact_positions(oriented,balance_reference,enabled=centered_contacts)
            for p,v,h in (contact_heights(positioned,target_points,plane) if surface_heights else [(positioned,'network_contact',0.)]))
        for proposal,variant,height_shift,orientation,position_variant,center_shift in variants:
            rotation=proposal[4:13].reshape(3,3)
            reason=None;evaluated+=1
            tip = proposal[13:16] + proposal[3] * rotation[:, 0]
            candidate = candidate_from_row(proposal, 'graspnet')
            candidate.update(proposal_variant=variant,contact_height_shift_m=height_shift,
                orientation_variant=orientation,contact_position_variant=position_variant,
                contact_center_shift_m=center_shift)
            candidate['closing_section'] = observed_contact_line(candidate,target_points)
            section = candidate['closing_section']
            if plane.height(tip) < 0:
                # The contact point itself cannot lie below its support. No
                # imaginary 20 mm finger corners are used for this decision.
                reason = 'contact_point_below_support'
            elif section['status'] != 'VISIBLE_SPAN':
                reason = 'target_section_unresolved'
            elif not low <= section['high_m'] - section['low_m'] <= high:
                reason = 'observed_span_outside_robot_range'
            else:
                # Network width was predicted for another hand. Derive this
                # hand's opening from the same observed pinch line; entry
                # clearance and measured encoder limits remain downstream.
                opening=2*max(-section['low_m'],section['high_m'])
                if not low<=opening<=high:
                    reason='observed_opening_outside_robot_range'
                else:
                    candidate.update(network_width_m=candidate['width_m'],width_m=float(opening),
                        opening_source='observed_contact_line')
                    candidate['balance_proxy']=contact_centrality(tip,rotation[:,1],balance_reference)
                    eligible.append((index, proposal, tip, candidate))
            if reason:
                rejected[reason] = rejected.get(reason, 0) + 1
    # Keep nearby orientation alternatives for a five-DOF, asymmetric hand.
    # The former 3 cm / 30 deg NMS could remove every feasible variant before IK.
    # Prefer centred contacts before the bounded proposal pool can discard them.
    # Preserve a quarter of the pool for detector-ranked orientation alternatives.
    def priority(item):
        balance=item[3]['balance_proxy']
        penalty=.5*balance.get('normalized_offset',0.)**2
        if short_axis is not None:
            closing=item[1][4:13].reshape(3,3)[:,1]
            penalty+=.5*(1-float(short_axis@closing)**2)
        return (-item[1][0]+penalty,item[0])
    by_balance=sorted(eligible,key=priority)
    by_detector=sorted(eligible,key=lambda item:(-item[1][0],item[0]))
    preferred=max(1,limit-limit//4)
    ordered=iter(by_balance);kept=[]
    def append_distinct(item):
        rotation=item[1][4:13].reshape(3,3)
        duplicate=any(np.linalg.norm(item[2]-other[2])<.008 and
            abs((item[2]-other[2])@plane.normal)<.002 and
            np.arccos(np.clip((np.trace(rotation.T@other[1][4:13].reshape(3,3))-1)/2,-1,1))
            <np.deg2rad(10) for other in kept)
        if not duplicate:kept.append(item)
    for item in ordered:
        append_distinct(item)
        if len(kept)>=preferred:break
    for item in by_detector:
        if len(kept)>=limit:break
        append_distinct(item)
    kept.sort(key=priority)
    return [dict(item[3], proposal_row_index=item[0], robot_feasibility_checked=False) for item in kept], dict(
        policy='observed_target_proposals_v5_centered_contact', input_rows=len(rows), evaluated_contact_variants=evaluated,
        longitudinal_center_alternatives=bool(centered_contacts),
        straight_across_alternatives=short_axis is not None,
        balance_score_weight=.5,balance_preferred_slots=preferred,
        detector_alternative_slots=limit-preferred,measured_com=False,
        eligible_before_diversity=len(eligible),surface_heights_sampled=bool(surface_heights),
        selected=len(kept), maximum_proposals=limit, rejected_by_evidence=rejected,
        physical_collision_model='deferred_to_SO101_CAD_and_full_path',
        diversity_translation_m=.008,diversity_height_m=.002, diversity_rotation_deg=10, motion_authorized=False)
