"""Independent visible-patch evidence for probe and changing carry orientation.

Input TCP poses must come from measured joints, and patch positions from RGB-D.
This module never receives a planned pose or substitutes a command for a sample.
An invisible patch remains unknown. It is not a force or mass measurement.
"""
import math
import numpy as np

from .probe_evidence import LiftSample, lift_relation
from .s2_geometry import rigid


class CarryVisualMonitor:
    def __init__(self, *, probe_duration_s, retention_tolerance_m=.01):
        if (not math.isfinite(probe_duration_s) or not .2 <= probe_duration_s <= 14.
                or not math.isfinite(retention_tolerance_m) or not .002 <= retention_tolerance_m <= .02):
            raise ValueError('bounded probe and explicit tracking tolerance required')
        self.probe_duration = probe_duration_s+1.
        self.tolerance = retention_tolerance_m
        self.reference = None
        self.patch_in_tcp = None
        self.patch_error = None
        self.last_sample = None
        self.failed = False

    def begin_probe(self, sample, base_from_measured_tcp, now):
        if self.reference is not None:
            raise ValueError('probe reference is single-use')
        self._validate(sample, base_from_measured_tcp, now)
        self.reference = sample

    def _validate(self, sample, pose, now):
        pose = rigid(np.asarray(pose).tolist())
        if not isinstance(sample, LiftSample):
            raise ValueError('measured patch/FK pair required')
        data = (*sample.target_xyz_m, *sample.tool_xyz_m, sample.vision_time_s,
                sample.tool_time_s, sample.position_error_bound_m, now)
        if (len(sample.target_xyz_m) != 3 or len(sample.tool_xyz_m) != 3 or not np.isfinite(data).all()
                or not 0 <= now-min(sample.tool_time_s,sample.vision_time_s) <= .2
                or max(sample.tool_time_s,sample.vision_time_s) > now
                or abs(sample.tool_time_s-sample.vision_time_s) > .05
                or sample.position_error_bound_m < 0
                or not all((sample.session,sample.target,sample.patch))
                or np.linalg.norm(pose[:3,3]-sample.tool_xyz_m) > 1e-6):
            raise ValueError('stale, inconsistent or invalid measured patch pair')
        if self.reference and (sample.session,sample.target,sample.patch) != (
                self.reference.session,self.reference.target,self.reference.patch):
            raise ValueError('visible patch identity changed')
        if self.last_sample and (sample.tool_time_s <= self.last_sample.tool_time_s
                                 or sample.vision_time_s <= self.last_sample.vision_time_s):
            raise ValueError('repeated patch/joint pair')
        return pose

    def observe(self, sample, base_from_measured_tcp, now):
        if self.reference is None or self.failed:
            return 'unknown', 'PROBE_REFERENCE_REQUIRED' if not self.failed else 'TRACKING_LOST'
        try:
            pose = self._validate(sample, base_from_measured_tcp, now)
            self.last_sample = sample
            if self.patch_in_tcp is None:
                relation, reason = lift_relation(self.reference, sample, now,
                                                maximum_duration_s=self.probe_duration)
                if relation == 'held':
                    self.patch_in_tcp = np.linalg.inv(pose)@np.r_[sample.target_xyz_m,1.]
                    self.patch_error = sample.position_error_bound_m
                return relation, reason
            expected = (pose@self.patch_in_tcp)[:3]
            error = np.linalg.norm(expected-sample.target_xyz_m)
            uncertainty = self.patch_error+sample.position_error_bound_m
            if error+uncertainty > self.tolerance:
                self.failed = True
                return 'unknown', 'PAYLOAD_RELATION_CHANGED'
            return 'held', 'MEASURED_PATCH_RETAINED'
        except (ValueError, TypeError):
            self.failed = True
            return 'unknown', 'INVALID_OR_STALE_PATCH'
