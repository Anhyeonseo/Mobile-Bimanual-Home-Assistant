"""Stationary scan aggregation; never smooth collision rays or moving poses."""
import math
import warnings
import numpy as np


class StationaryScanWindow:
    # Repeatability policies, not a claim about surveyed map accuracy.
    window_s = 2.0
    minimum_span_s = 1.2
    minimum_samples = 8

    def __init__(self):
        self.reset()

    def reset(self):
        self.scans = []
        self.poses = []
        self.anchor = None
        self.report = {'ready': False, 'reason': 'stationary evidence unavailable'}

    def prepare(self, scan, seed, wheel, *, execute, now):
        stamp = scan['observed_s']
        quiet = (not execute and wheel is not None
                 and 0 <= now-wheel['observed_s'] <= .35
                 and all(abs(v) < .001 for v in wheel['twist'])
                 and 0 <= now-stamp <= .35)
        if not quiet:
            self.reset()
            return scan, seed
        if self.scans and (stamp <= self.scans[-1][0] or stamp-self.scans[-1][0] > .5):
            self.reset()
        if self.anchor is None:
            self.anchor = list(seed)
        self.scans.append((stamp, np.asarray(scan['ranges'], dtype=float)))
        self.scans = [(t, r) for t, r in self.scans if stamp-t <= self.window_s]
        # Invalid/no-return rays cannot become repeated old obstacle readings.
        values = np.stack([r for _, r in self.scans])
        values[~np.isfinite(values) | (values < .25) | (values > 12)] = np.nan
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            ranges = np.nanmedian(values, axis=0)
        ranges[np.sum(np.isfinite(values), axis=0) < math.ceil(.6*len(values))] = np.nan
        return dict(scan, ranges=ranges.tolist()), self.anchor

    def observe(self, candidate, stamp, *, revision=None):
        if self.anchor is None:
            return list(candidate)
        if self.poses and self.poses[-1][2] != revision:
            self.reset()
            return list(candidate)
        self.poses.append((stamp, list(candidate), revision))
        self.poses = [(t, p, r) for t, p, r in self.poses if stamp-t <= self.window_s]
        values = np.array([p for _, p, _ in self.poses])
        values[:, 2] = np.unwrap(values[:, 2])
        center = np.median(values, axis=0)
        errors = np.linalg.norm(values[:, :2]-center[:2], axis=1)
        yaw_errors = np.abs(values[:, 2]-center[2])
        half = max(1, len(values)//2)
        drift = np.median(values[half:], axis=0)-np.median(values[:half], axis=0) if len(values)>1 else np.zeros(3)
        span = stamp-self.poses[0][0]
        ready = (len(self.poses) >= self.minimum_samples and span >= self.minimum_span_s
                 and np.quantile(errors, .9) <= .015
                 and np.quantile(yaw_errors, .9) <= math.radians(.7)
                 and np.linalg.norm(drift[:2]) <= .008 and abs(drift[2]) <= math.radians(.5))
        self.report = dict(ready=bool(ready), samples=len(self.poses), span_s=span,
            observed_s=stamp, map_revision=revision, pose=center.tolist(),
            p90_xy_m=float(np.quantile(errors, .9)), p90_yaw_rad=float(np.quantile(yaw_errors, .9)),
            half_window_drift_xy_m=float(np.linalg.norm(drift[:2])),
            half_window_drift_yaw_rad=float(abs(drift[2])),
            reason='stationary window ready' if ready else 'collecting consistent stationary scans',
            surveyed_accuracy_verified=False)
        # Do not feed the moving window back as the next stationary ICP seed.
        center[2] = math.atan2(math.sin(center[2]), math.cos(center[2]))
        self.report['pose'] = center.tolist()
        return center.tolist()


def fresh_stationary(report, now):
    return bool(report and report.get('ready') is True
                and 0 <= now-report.get('observed_s', -math.inf) <= .5)
