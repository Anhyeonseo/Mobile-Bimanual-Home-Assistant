"""Compare an observed target patch with measured TCP motion during a bounded test lift."""
from dataclasses import dataclass
from math import dist, hypot, isfinite
from numbers import Real

# Provisional stationary-lab bounds in metres. Physical commissioning must establish
# whether a synchronized depth patch can resolve this stroke on the particular target.
STROKE_M = .004
MAX_RISE_M = .006
MIN_VISIBLE_RISE_M = .0025
MAX_LATERAL_M = .002
FOLLOW_TOLERANCE_M = .0015


@dataclass(frozen=True)
class LiftSample:
    """TCP from FK and one visible object patch expressed in a common stationary base frame."""
    session: str
    target: str
    patch: str
    tool_time_s: float
    vision_time_s: float
    tool_xyz_m: tuple
    target_xyz_m: tuple
    position_error_bound_m: float  # combined FK/vision bound for this pair


def lift_relation(start, current, now, *, maximum_duration_s=1.):
    """Return (visual relation, reason). No command or scene attachment is evidence."""
    if not isinstance(start, LiftSample) or not isinstance(current, LiftSample):
        return 'unknown', 'MISSING_SAMPLE'
    if (isinstance(maximum_duration_s, bool) or not isinstance(maximum_duration_s, Real)
            or not isfinite(maximum_duration_s) or not .2 <= maximum_duration_s <= 15.):
        return 'unknown', 'INVALID_PROBE_DURATION'
    if not all((start.session, start.target, start.patch)) or (
        (current.session, current.target, current.patch) != (start.session, start.target, start.patch)
    ):
        return 'unknown', 'TARGET_OR_REFERENCE_CHANGED'
    for s in (start, current):
        if any(not isinstance(p, (list, tuple)) or len(p) != 3 for p in (s.tool_xyz_m, s.target_xyz_m)):
            return 'unknown', 'INVALID_MEASUREMENT'
        values = (*s.tool_xyz_m, *s.target_xyz_m, s.tool_time_s, s.vision_time_s, s.position_error_bound_m, now)
        if (any(not isinstance(v, Real) or isinstance(v, bool) or not isfinite(v) for v in values)
                or s.position_error_bound_m < 0):
            return 'unknown', 'INVALID_MEASUREMENT'
        if min(s.tool_time_s, s.vision_time_s) < 0 or abs(s.tool_time_s - s.vision_time_s) > .05:
            return 'unknown', 'UNSYNCHRONIZED_PAIR'
    if (current.tool_time_s <= start.tool_time_s or current.vision_time_s <= start.vision_time_s
            or now < max(current.tool_time_s, current.vision_time_s)
            or now - min(current.tool_time_s, current.vision_time_s) > .2
            or now - min(start.tool_time_s, start.vision_time_s) > maximum_duration_s):
        return 'unknown', 'STALE_PROBE'
    tool = tuple(b-a for a, b in zip(start.tool_xyz_m, current.tool_xyz_m))
    target = tuple(b-a for a, b in zip(start.target_xyz_m, current.target_xyz_m))
    uncertainty = start.position_error_bound_m + current.position_error_bound_m
    if uncertainty >= FOLLOW_TOLERANCE_M:
        return 'unknown', 'DISPLACEMENT_UNRESOLVED'
    if (not -uncertainty <= tool[2] <= MAX_RISE_M or hypot(*tool[:2]) > MAX_LATERAL_M):
        return 'unknown', 'OUTSIDE_PROBE_STROKE'
    if dist(tool, target) + uncertainty > FOLLOW_TOLERANCE_M:
        return 'unknown', 'TARGET_DID_NOT_FOLLOW'
    if min(tool[2], target[2]) - uncertainty < MIN_VISIBLE_RISE_M:
        return 'contact', 'INSUFFICIENT_RISE'
    return 'held', 'TARGET_FOLLOWED'
