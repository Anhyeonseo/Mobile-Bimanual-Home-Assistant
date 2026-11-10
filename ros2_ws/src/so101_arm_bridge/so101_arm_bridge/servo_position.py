"""Single-turn position commands, distinct from modulo encoder observations.

Keep the planner's continuous winding. Never apply shortest-angle wrapping to
an actuator goal: a position-mode servo has not been verified across its seam.
"""
from numbers import Integral

COUNTS = 4096
ZERO = 2048
TURN_URAD = 6283185


def raw_from_wire(position_urad, direction):
    """Match the MCU integer map, including rounding of negative angles."""
    if (isinstance(position_urad, bool) or not isinstance(position_urad, Integral)
            or direction not in (-1, 1)):
        raise ValueError('integer wire angle and calibrated direction required')
    numerator = int(position_urad) * COUNTS
    magnitude = (abs(numerator) + TURN_URAD // 2) // TURN_URAD
    return ZERO + direction * (magnitude if numerator >= 0 else -magnitude)


def single_turn_window(entry, reference_raw=ZERO):
    """Planner-only intersection; do not rewrite the measured physical range."""
    lower, upper = entry['minimum_unwrapped_raw'], entry['maximum_unwrapped_raw']
    if not lower <= reference_raw <= upper:
        raise ValueError('reference outside measured continuous range')
    branch = reference_raw // COUNTS
    return max(lower, branch * COUNTS), min(upper, (branch + 1) * COUNTS - 1)


def validate_wire_route(positions, directions, *, reference=None):
    """Inspect the whole route, including stage/batch boundaries, before ARM.

    A small continuous crossing (10 -> -46) and an ambiguous sparse raw jump
    (10 -> 4050) are both invalid. An explicitly sampled long in-range route
    retains its planned direction; this function never invents a shortcut.
    """
    directions = tuple(directions)
    if not directions or any(d not in (-1, 1) for d in directions):
        raise ValueError('calibrated encoder directions required')

    def convert(row):
        if len(row) != len(directions):
            raise ValueError('wire joint count differs from calibration')
        return tuple(raw_from_wire(q, d) for q, d in zip(row, directions))

    previous = None if reference is None else convert(reference)
    for index, row in enumerate(positions):
        current = convert(row)
        if previous is not None:
            for joint, (a, b) in enumerate(zip(previous, current)):
                if a // COUNTS != b // COUNTS:
                    raise ValueError(f'encoder seam in route: sample={index} joint={joint} raw={a}->{b}; replan within a single turn')
                if abs(b-a) >= COUNTS // 2:
                    raise ValueError(f'ambiguous encoder jump: sample={index} joint={joint} raw={a}->{b}')
        previous = current
