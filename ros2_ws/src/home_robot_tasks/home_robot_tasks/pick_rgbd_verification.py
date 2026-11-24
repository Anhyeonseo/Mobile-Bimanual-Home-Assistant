"""Discrete post-pick verification from measured depth and object masks.

This is not continuous tracking. Inputs must be stationary captures from the
same controller owner; the caller checks unchanged hold before/after inference.
Missing objects, depth, ambiguous instances, or a view with insufficient surface
coverage remain UNKNOWN. A missing detection alone never establishes success.
"""
import numpy as np
from scipy.spatial import cKDTree


def compare_clouds(reference, observed, reference_tcp, current_tcp, *, contact,
                   empty=False, tolerance_m=.008):
    from .s2_geometry import rigid
    unknown = lambda reason: dict(state='UNKNOWN',reason=reason)
    a, b = np.asarray(reference,float), np.asarray(observed,float)
    if (a.ndim!=2 or b.ndim!=2 or a.shape[1:]!=(3,) or b.shape[1:]!=(3,)
            or min(len(a),len(b))<50 or not np.isfinite(a).all() or not np.isfinite(b).all()):
        return unknown('INSUFFICIENT_MEASURED_TARGET_DEPTH')
    movement = rigid(current_tcp) @ np.linalg.inv(rigid(reference_tcp))
    held = (movement @ np.c_[a,np.ones(len(a))].T).T[:,:3]
    if np.linalg.norm(np.median(held,axis=0)-np.median(a,axis=0)) < .02:
        return unknown('HYPOTHESES_NOT_SEPARABLE')
    # Bidirectional coverage rejects a tiny surviving patch, broad background
    # segment, or an unrelated remote elsewhere. Deterministically bound work.
    sample = lambda p: p[np.linspace(0,len(p)-1,min(2500,len(p)),dtype=int)]
    a,b,held = map(sample,(a,b,held))
    def overlap(expected):
        return min(float(np.mean(cKDTree(expected).query(b)[0] <= tolerance_m)),
                   float(np.mean(cKDTree(b).query(expected)[0] <= tolerance_m)))
    stayed, followed = overlap(a), overlap(held)
    result = dict(source_overlap=stayed,held_overlap=followed)
    if followed >= .65 and stayed <= .15 and contact is True:
        return dict(result,state='SUCCESS',reason='MEASURED_TARGET_FOLLOWED_TCP')
    if stayed >= .65 and followed <= .15 and empty is True:
        return dict(result,state='FAILURE',reason='MEASURED_TARGET_REMAINED_AND_JAW_EMPTY')
    return dict(result,state='UNKNOWN',reason='CONTRADICTORY_OR_INSUFFICIENT_DEPTH')


class StableVerification:
    def __init__(self):
        self.samples = []

    def add(self, evidence):
        if self.samples and (evidence['capture_id'] in {s['capture_id'] for s in self.samples} or
                             evidence['observed_s']<=self.samples[-1]['observed_s']):
            raise ValueError('verification must use distinct increasing exposures')
        self.samples.append(evidence)
        recent = self.samples[-3:]
        if (len(recent)==3 and recent[-1]['observed_s']-recent[0]['observed_s']>=.3
                and len({s['state'] for s in recent})==1 and recent[0]['state'] in ('SUCCESS','FAILURE')):
            return recent[-1]['state']
        return 'UNKNOWN'
