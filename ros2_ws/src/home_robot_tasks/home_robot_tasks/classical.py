"""Mobile-safe route assembly adapted from the frozen P&P application.

Reuses twelve-axis routing/opposite-arm hold and finite interpolation. Pose
planning/collision checking stay upstream. All calibration and rate limits are
caller supplied; no old workcell poses, camera split or grasp offset is copied.
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
    if any(a >= b for a, b in zip(lo, hi)) or any(v <= 0 for v in speeds):
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
        count = max(
            1,
            math.ceil(
                max(abs(b - a) / speed for a, b, speed in zip(start, target, speeds))
                / (interval_ms / 1000)
            ),
        )
        if count + len(points) > 20000:
            raise InvalidTask("trajectory too long")
        offset = points[-1]["offset_ms"]
        for n in range(1, count + 1):
            position = (
                tuple(a + (b - a) * n / count for a, b in zip(start, target))
                if n < count
                else tuple(target)
            )
            points.append(
                {"offset_ms": offset + n * interval_ms, "positions_rad": position}
            )
        start = tuple(target)
    return points
