"""L3D -> L2: retime a checked polyline without rounding corners or wrapping axes.

All coordinates here are controller radians, not SO101 model radians. The
quintic law is the same rest-to-rest law used by classical.route_for_arm.
This is command-profile validation, not a claim about measured acceleration.
"""
from dataclasses import dataclass
import math

import numpy as np

from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint


@dataclass(frozen=True)
class MotionBounds:
    minimum: tuple
    maximum: tuple
    velocity: tuple
    acceleration: tuple
    jerk: tuple

    def __post_init__(self):
        values = np.asarray([self.minimum, self.maximum, self.velocity,
                             self.acceleration, self.jerk], dtype=float)
        if (values.shape != (5, 12) or not np.isfinite(values).all()
                or np.any(values[0] >= values[1]) or np.any(values[2:] <= 0)):
            raise ValueError('finite twelve-axis motion bounds required')


def polyline_corners(path):
    """Remove duplicates/strictly collinear samples, never cut across a bend."""
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 12 or not 1 <= len(path) <= 20000 or not np.isfinite(path).all():
        raise ValueError('bounded twelve-axis polyline required')
    if np.any(np.abs(np.diff(path, axis=0)) >= math.pi):
        raise ValueError('branch jump: do not modulo-wrap controller positions')
    result = [path[0]]
    for p in path[1:]:
        if np.max(np.abs(p-result[-1])) < 1e-12:
            continue
        while len(result) >= 2:
            a, b = result[-2:]
            delta = p-a
            fraction = float(np.dot(b-a, delta)/np.dot(delta, delta)) if np.dot(delta, delta) else -1.
            if 0 <= fraction <= 1 and np.max(np.abs(b-(a+fraction*delta))) <= 1e-10:
                result.pop()
            else:
                break
        result.append(p)
    return np.asarray(result)


def retime(path, bounds, *, interval_ms=50, lead_ms=50, dwell_s=0.):
    """Return finite points and exact continuous-law peak rates.

    At true corners the arm stops with zero speed and acceleration. Every point
    remains on a previously checked segment. No spline shortcut is introduced.
    Integer micro-radian wire rounding is checked separately by the adapter;
    finite-difference jerk of rounded encoder counts is not physical jerk.
    """
    if (not isinstance(bounds, MotionBounds) or type(interval_ms) is not int
            or not 5 <= interval_ms <= 100 or interval_ms % 5
            or type(lead_ms) is not int or not 20 <= lead_ms <= 250 or lead_ms % 5
            or not math.isfinite(dwell_s) or not 0 <= dwell_s <= 10):
        raise ValueError('invalid timing policy')
    corners = polyline_corners(path)
    if np.any(corners < bounds.minimum) or np.any(corners > bounds.maximum):
        raise ValueError('controller joint range exceeded')
    points = [TimedJointPoint(lead_ms, tuple(corners[0]))]
    peaks = np.zeros((3, 12))
    for a, b in zip(corners, corners[1:]):
        distance = np.abs(b-a)
        duration = max(float(np.max(1.875*distance/bounds.velocity)),
                       float(np.max(np.sqrt((10/math.sqrt(3))*distance/bounds.acceleration))),
                       float(np.max(np.cbrt(60*distance/bounds.jerk))))
        count = max(1, math.ceil(duration*1000/interval_ms))
        if len(points)+count > 20000:
            raise ValueError('retimed trajectory exceeds bounded sample budget')
        duration = count*interval_ms/1000
        peaks = np.maximum(peaks, [1.875*distance/duration,
                                   (10/math.sqrt(3))*distance/duration**2,
                                   60*distance/duration**3])
        offset = points[-1].offset_ms
        for n in range(1, count+1):
            u = n/count
            s = u**3*(10+u*(-15+6*u))
            q = b if n == count else a+(b-a)*s
            points.append(TimedJointPoint(offset+n*interval_ms, tuple(q)))
    for _ in range(max(1 if len(points) == 1 else 0, math.ceil(dwell_s*1000/interval_ms))):
        points.append(TimedJointPoint(points[-1].offset_ms+interval_ms, points[-1].positions_rad))
    if len(points) > 20000:
        raise ValueError('retimed trajectory exceeds bounded sample budget')
    return tuple(points), dict(duration_s=points[-1].offset_ms/1000, samples=len(points),
                              corners=len(corners), velocity=peaks[0].tolist(),
                              acceleration=peaks[1].tolist(), jerk=peaks[2].tolist())
