"""Mobile-safe route assembly adapted from the frozen P&P application.

Reuses twelve-axis routing/opposite-arm hold, with quintic rest-to-rest timing. Pose
planning/collision checking stay upstream. All calibration and rate limits are
caller supplied except explicit acceleration/jerk simulation candidates; no old
workcell poses, camera split or grasp offset is copied. No new external PID loop.
"""

from __future__ import annotations
import math
from .navigation import number
from .fetch import InvalidTask


def route_for_arm(
    current,
    opposite_hold,
    arm,
    steps,
    *,
    minimum_rad,
    maximum_rad,
    max_speed_rad_s,
    interval_ms=50,
    max_acceleration_rad_s2=(1.0,) * 12,
    max_jerk_rad_s3=(8.0,) * 12,
):
    if (
        arm not in {"left", "right"}
        or type(interval_ms) is not int
        or not 5 <= interval_ms <= 100
        or interval_ms % 5
    ):
        raise InvalidTask("invalid arm or sample interval")

    def vector(values, count):
        if not isinstance(values, (list, tuple)) or len(values) != count:
            raise InvalidTask("invalid joint vector")
        return tuple(number(v, "joint coordinate") for v in values)

    start = vector(current, 12)
    hold = vector(opposite_hold, 6)
    lo, hi = vector(minimum_rad, 12), vector(maximum_rad, 12)
    speeds = vector(max_speed_rad_s, 12)
    accelerations = vector(max_acceleration_rad_s2, 12)
    jerks = vector(max_jerk_rad_s3, 12)
    if any(a >= b for a, b in zip(lo, hi)) or any(v <= 0 for v in (*speeds, *accelerations, *jerks)):
        raise InvalidTask("invalid joint limits")

    def within(values):
        if any(not a <= v <= b for a, v, b in zip(lo, values, hi)):
            raise InvalidTask("joint_limit")

    within(start)
    if not isinstance(steps, list) or not steps or len(steps) > 64:
        raise InvalidTask("bounded nonempty route required")
    index = 0 if arm == "left" else 6
    other = 6 - index
    if tuple(start[other : other + 6]) != hold:
        raise InvalidTask("opposite hold differs from measured anchor")
    # First sample preserves the measured anchor; no catch-up jump on admission.
    points = [{"offset_ms": interval_ms, "positions_rad": start}]
    for step in steps:
        target = list(start)
        if not isinstance(step, dict):
            raise InvalidTask("invalid plan step")
        if step.get("kind") == "arm" and set(step) == {"kind", "target_positions_rad"}:
            target[index : index + 5] = vector(step["target_positions_rad"], 5)
        elif step.get("kind") == "gripper" and set(step) == {
            "kind",
            "target_position_rad",
        }:
            target[index + 5] = number(step["target_position_rad"], "gripper")
        else:
            raise InvalidTask("unsupported plan step")
        target[other : other + 6] = hold
        within(target)
        # A task waypoint is a deliberate stop (e.g. grip before retreat).
        # Quintic s=10u^3-15u^4+6u^5 starts/ends with zero v and a.
        # Exact normalized peaks: v=15/8, a=10/sqrt(3), jerk=60.
        duration = max(
            max(1.875 * abs(b-a) / v,
                math.sqrt((10/math.sqrt(3)) * abs(b-a) / acc),
                (60 * abs(b-a) / jerk) ** (1/3))
            for a,b,v,acc,jerk in zip(start,target,speeds,accelerations,jerks)
        )
        if not math.isfinite(duration) or duration > 20000 * interval_ms / 1000:
            raise InvalidTask("trajectory too long")
        count = max(1, math.ceil(duration / (interval_ms / 1000)))
        if count + len(points) > 20000:
            raise InvalidTask("trajectory too long")
        offset = points[-1]["offset_ms"]
        for n in range(1, count + 1):
            u = n / count
            blend = u*u*u*(10 + u*(-15 + 6*u))
            position = (
                tuple(a + (b - a) * blend for a, b in zip(start, target))
                if n < count
                else tuple(target)
            )
            points.append(
                {"offset_ms": offset + n * interval_ms, "positions_rad": position}
            )
        start = tuple(target)
    return points
