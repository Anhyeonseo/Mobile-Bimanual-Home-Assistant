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
                 and (wheel.get('stationary_inventory') is True or all(abs(v) < .001 for v in wheel['twist']))
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


def fresh_stationary(report, now, *, revision=None):
    return bool(report and report.get('ready') is True
                and 0 <= now-report.get('observed_s', -math.inf) <= .5
                and (revision is None or report.get('map_revision') == revision))


class StationaryWheelInventory:
    """Torque-OFF inspection only; reuse the 400ms/two-count parked window.

    Keep raw speed/movement reports in the caller's log. This is evidence for
    scan aggregation, never authority to enable motors or confirm runtime STOP.
    """
    def __init__(self):
        self.samples=[]

    def observe(self,rows,started,received,geometry):
        valid=(0<=received-started<=.25 and len(rows)==3
            and all(row.get('complete') and row['bus']=='left' and row['requested_id']==mid
                and row['offset_and_mode']['operating_mode']==1
                and row['command']['torque_enabled_raw']==0
                and row['command']['goal_speed_raw']==0
                and 0<=row['feedback']['position_raw']<4096
                and (row['feedback']['speed_raw']&32767)<=50
                and row['feedback']['moving_raw'] in (0,1)
                for row,mid in zip(rows,(7,8,9))))
        if not valid:
            self.samples=[];return None
        ticks=[r['sample_started_tick_ms'] for r in rows]
        positions=[r['feedback']['position_raw'] for r in rows]
        if self.samples and (not 0<started-self.samples[-1][0]<=.25 or
                any(not 0<(n-o)%2**32<=250 for n,o in zip(ticks,self.samples[-1][2]))):
            self.samples=[]
        self.samples.append((started,positions,ticks))
        self.samples=[r for r in self.samples if started-r[0]<=.6]
        span=started-self.samples[0][0]
        values=np.array([[(p-a+2048)%4096-2048 for p,a in zip(r[1],self.samples[0][1])]
                         for r in self.samples])
        if len(self.samples)<3 or span<.4 or np.any(np.ptp(values,axis=0)>2):return None
        twist=geometry.body_twist(tuple(float(v) for v in values[-1]*math.tau/4096/span))
        return dict(observed_s=started,twist=twist,stationary_inventory=True,
            source='torque_off_encoder_window',span_s=span,position_span_counts=np.ptp(values,axis=0).tolist())
