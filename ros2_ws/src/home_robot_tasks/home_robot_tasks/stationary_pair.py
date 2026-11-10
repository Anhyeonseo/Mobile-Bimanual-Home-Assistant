"""Geometric screen for two nominal 5-DoF arms, with a common fixed lift. Simulation only."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

from .stationary_pick_sim import LINK_RADIUS, MAX_STEP


def _bounds(points, padding=0.):
    return points.min(axis=0)-padding, points.max(axis=0)+padding


def _overlap(a, b):
    return bool(np.all(a[1] >= b[0]) and np.all(b[1] >= a[0]))


def _hand(arm, q, width, depth):
    pose, links = arm.fk(q)
    boxes = [_bounds(links[i:i+2], LINK_RADIUS) for i in range(len(links)-1)]
    corners = np.array([pose[:3,3] + s*pose[:3,2] - w*pose[:3,0] for s in (-depth, 0.)
                        for w in (arm.jaw.fixed_face_m, arm.jaw.fixed_face_m+width)])
    boxes.append(_bounds(corners, .003))
    return pose, boxes


def _payload_bounds(pose, payload):
    center = pose[:3,3]
    span = np.abs(pose[:3,:3]) @ payload.half_extents
    return center-span, center+span


def paired_path(arms, paths, payloads, *, shared, schedule='together', lag_fraction=0.,
                contacts_verified=(True, True)):
    """Check sampled link/jaw/load envelopes; a shared body also needs near-rigid relative motion.

    Each path contains three approach/contact/retreat nodes and its opening/depth.
    lag_fraction delays one planned motion within a phase; it is not measured controller latency.
    """
    if (len(arms) != 2 or [a.side for a in arms] != ['left','right'] or arms[0].lift_m != arms[1].lift_m
            or len(paths) != 2 or len(payloads) != 2 or len(contacts_verified) != 2
            or not all(type(ok) is bool for ok in contacts_verified)
            or schedule not in ('together','left_then_right') or not -1 < lag_fraction < 1):
        raise ValueError('two arms at one lift and a bounded phase schedule required')
    if shared and (schedule != 'together' or payloads[0] is not payloads[1]):
        raise ValueError('shared load requires one body and a common lifting phase')
    if not shared and payloads[0] is payloads[1]:
        raise ValueError('independent picks require distinct bodies')
    chains = [[a.anchor, *[np.asarray(s['model_rad']) for s in p['stages']]] for a,p in zip(arms, paths)]
    phases = ([(1,1),(2,2),(3,3)] if schedule == 'together' else
              [(1,0),(2,0),(3,0),(3,1),(3,2),(3,3)])
    previous = (0,0)
    peaks = dict(relative_drift_m=0., relative_drift_deg=0.)
    result = dict(mode='nominal_pair_screen', motion_authorized=False, hardware_commands=0,
                  dynamics_checked=False, full_robot_collision_verified=False, measured_synchronization=False,
                  schedule=schedule, lag_fraction=lag_fraction, **peaks)
    attachments = [np.linalg.inv(a.fk(c[2])[0]) @ obj.base_pose for a,c,obj in zip(arms, chains, payloads)]
    for phase in phases:
        start = [c[i] for c,i in zip(chains, previous)]
        end = [c[i] for c,i in zip(chains, phase)]
        if any(i == 3 and not ok for i,ok in zip(phase, contacts_verified)):
            return dict(result, status='STOP_BEFORE_LIFT', phase=phase, **peaks)
        motion = max(float(np.max(abs(q1-q0))) for q0,q1 in zip(start, end))
        steps = max(1, math.ceil(motion / (MAX_STEP * (1-abs(lag_fraction)))))
        for t in np.linspace(0., 1., steps+1):
            hands = []
            for i, (arm, path, q0, q1) in enumerate(zip(arms, paths, start, end)):
                delay = max(0., lag_fraction if i == 1 else -lag_fraction)
                f = np.clip((t-delay)/(1-delay), 0., 1.)
                hands.append(_hand(arm, q0 + f*(q1-q0), path['width'], path['depth']))
            if any(_overlap(a,b) for a in hands[0][1] for b in hands[1][1]):
                return dict(result, status='INTER_ARM_ENVELOPE', phase=phase, **peaks)
            poses = [h[0] @ att if i == 3 else obj.base_pose
                     for h,att,i,obj in zip(hands, attachments, phase, payloads)]
            if shared and phase == (3,3):
                error = np.linalg.inv(poses[0]) @ poses[1]
                peaks['relative_drift_m'] = max(peaks['relative_drift_m'], float(np.linalg.norm(error[:3,3])))
                peaks['relative_drift_deg'] = max(peaks['relative_drift_deg'], float(
                    np.degrees(Rotation.from_matrix(error[:3,:3]).magnitude())))
                if peaks['relative_drift_m'] > .003 or peaks['relative_drift_deg'] > 3.:
                    return dict(result, status='SHARED_BODY_DRIFT', phase=phase, **peaks)
            if not shared:
                loads = [_payload_bounds(q,p) for q,p in zip(poses,payloads)]
                if (_overlap(*loads) or any(_overlap(loads[i], other) for i in range(2)
                                            for other in hands[1-i][1])):
                    return dict(result, status='OTHER_PAYLOAD_ENVELOPE', phase=phase, **peaks)
        previous = phase
    return dict(result, status='PAIR_SCREEN_PASSED', **peaks)
