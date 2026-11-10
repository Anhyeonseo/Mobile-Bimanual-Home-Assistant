"""Stationary pick closure policy, in metres at the chosen jaw section."""
import math
from numbers import Real

import numpy as np

# Provisional for rigid objects and the current bare pads. Contact uncertainty
# is independent of the amount by which the commanded gap undershoots the span.
SIDE_CLEARANCE_M = .002
RIGID_CLOSE_ALLOWANCE_M = .005
CONTACT_UNCERTAINTY_M = .002
STEP_M = .001


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def closure_plan(jaw, candidate, *, entry_clearance_m=2*SIDE_CLEARANCE_M,
                 close_allowance_m=RIGID_CLOSE_ALLOWANCE_M):
    if not _finite(entry_clearance_m) or not 2*SIDE_CLEARANCE_M <= entry_clearance_m <= .020:
        raise ValueError('entry clearance must be within 4..20 mm')
    if not _finite(close_allowance_m) or not .001<=close_allowance_m<=.005:
        raise ValueError('close allowance must be within 1..5 mm')
    opening = candidate.get('width_m')
    if not _finite(opening) or not jaw.accepts(opening):
        return dict(status='SO101_JAW_SECTION_LIMIT')
    section = candidate.get('closing_section', {})
    if not isinstance(section, dict):
        return dict(status='WIDTH_UNRESOLVED', section_status='INVALID_SECTION')
    if section.get('status') != 'VISIBLE_SPAN':
        return dict(status='WIDTH_UNRESOLVED', section_status=section.get('status', 'NO_SECTION'))
    low, high = section.get('low_m'), section.get('high_m')
    if not _finite(low) or not _finite(high) or not low < 0 < high:
        return dict(status='WIDTH_UNRESOLVED', section_status='INVALID_EXTENT')
    span = high - low
    if section.get('method') == 'observed_contact_line':
        # Carry binding places the fixed face on the measured low contact.
        # Only the moving finger opens; an off-centre network origin must not
        # inflate the required aperture as if both fingers translated equally.
        opening = span + entry_clearance_m
    else:
        opening = max(opening, 2 * max(-low, high) + entry_clearance_m)
    if not jaw.accepts(opening) or not jaw.accepts(span):
        return dict(status='SO101_JAW_SECTION_LIMIT', visible_span_m=span)

    # Choose the final target before the first close step. Encoder observations
    # may shorten the motion, but never move this target after contact.
    target = max(float(jaw.gaps[0]), span - close_allowance_m)
    return dict(status='BOUNDED_CLOSE', entry_gap_m=opening, visible_span_m=span,
                close_allowance_m=close_allowance_m,
                target_gap_m=target, min_search_gap_m=target,  # legacy replay field
                upper_contact_gap_m=min(opening, span + CONTACT_UNCERTAINTY_M),
                entry_angle_rad=float(np.interp(opening, jaw.gaps, jaw.angles)),
                min_angle_rad=float(np.interp(target, jaw.gaps, jaw.angles)),
                hardware_authorized=False)


def target_gap(plan):
    """Old captures stored the preselected target as the minimum search gap."""
    if 'target_gap_m' in plan:
        target = plan['target_gap_m']
        if 'min_search_gap_m' in plan and plan['min_search_gap_m'] != target:
            raise ValueError('inconsistent planned closing target')
    else:
        target = plan['min_search_gap_m']
    if not _finite(target) or target <= 0:
        raise ValueError('invalid planned closing target')
    return target


def next_gap(plan, measured_gap_m, effort, *, healthy=True):
    """Take one bounded step towards the fixed target, or hold for feedback."""
    if (healthy is not True or plan.get('status') != 'BOUNDED_CLOSE'
            or not _finite(measured_gap_m)
            or effort not in ('free', 'rising', 'persistent')):
        return 'STOP_UNCONFIRMED', None
    target, upper = target_gap(plan), plan['upper_contact_gap_m']
    if not target <= measured_gap_m <= plan['entry_gap_m'] + 1e-9:
        return 'STOP_OUTSIDE_PLAN', None
    if effort == 'persistent':
        return ('CONTACT_CANDIDATE' if measured_gap_m <= upper else 'UNEXPECTED_RESISTANCE'), None
    if effort == 'rising':
        return 'WAIT_AT_CURRENT_GAP', None
    if measured_gap_m <= target + 1e-9:
        return 'STOP_NO_CONTACT', None
    return 'CLOSE_STEP', max(target, measured_gap_m - STEP_M)
