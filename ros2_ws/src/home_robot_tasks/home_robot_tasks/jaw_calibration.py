"""Measured installed-pad gap in millimetres and the corresponding control angle."""
from .fetch import InvalidTask
from .navigation import number


class JawCalibration:
    """Measured monotonic angle/gap samples, with no extrapolation."""
    def __init__(self, samples, *, open_rad, close_rad):
        if not isinstance(samples, (tuple, list)) or not 2 <= len(samples) <= 64:
            raise InvalidTask("bounded measured jaw calibration required")
        self.samples = tuple((number(a, "jaw angle"), number(g, "jaw gap")) for a, g in samples)
        angles, gaps = zip(*self.samples)
        if (any(a >= b for a, b in zip(angles, angles[1:])) or min(gaps) < 0
                or not (all(a < b for a, b in zip(gaps, gaps[1:]))
                        or all(a > b for a, b in zip(gaps, gaps[1:])))):
            raise InvalidTask("strictly monotonic jaw angle/gap samples required")
        self.open = number(open_rad, "open angle")
        self.close = number(close_rad, "close angle")
        if self.gap(self.open) <= self.gap(self.close):
            raise InvalidTask("open jaw must have the larger measured gap")

    def gap(self, angle):
        angle = number(angle, "measured jaw angle")
        if not self.samples[0][0] <= angle <= self.samples[-1][0]:
            raise InvalidTask("jaw angle outside calibration")
        for (a, x), (b, y) in zip(self.samples, self.samples[1:]):
            if a <= angle <= b:
                return x + (y-x) * (angle-a) / (b-a)
        raise InvalidTask("jaw interpolation unavailable")

    def angle_for_gap(self, gap_mm):
        """Invert the measured curve, including jaws whose control angle decreases on opening."""
        gap = number(gap_mm, "measured jaw gap")
        for (a, x), (b, y) in zip(self.samples, self.samples[1:]):
            if min(x, y) <= gap <= max(x, y):
                return a + (b-a) * (gap-x) / (y-x)
        raise InvalidTask("jaw gap outside calibration")
