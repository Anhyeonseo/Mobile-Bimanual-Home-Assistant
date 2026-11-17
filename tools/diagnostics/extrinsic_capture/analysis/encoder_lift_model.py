"""Shared pieces of the encoder-lift calibration (calib_encoder_lift.py, make_candidate_profile.py).

Lift height of a shot = (bottom_raw - raw) * |m/count| + 0.084 m * turns, turns chosen nearest to the operator
height; the model places the arms at h * (1 + lscale) + loff along the (optionally tilted) lift axis.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from calib_model import Model, ZJ, CJ

MPC = 2.05078125e-05; TURN = 4096 * MPC


def enc_height(raw, op, bottom_raw):
    base = ((bottom_raw - raw) % 4096) * MPC
    return base + TURN * round((op - base) / TURN)


def add_encoder_heights(ctx, bottom_raw):
    for s in ctx.shots:
        s['h_enc'] = enc_height(s['lift'], s['lift_op'], bottom_raw)

class EncModel(Model):
    def __init__(self, ctx, free, wplane=1.0):
        super().__init__(ctx, set(free) - {'lscale', 'loff'}, wplane)
        self.free = set(free)
        for n in ('lscale', 'loff'):
            self.spec.append((n, 1)); self.slices[n] = slice(self.size, self.size + 1); self.size += 1
            self.labels.append(n); self.units.append('%' if n == 'lscale' else 'mm')
        self.mask = np.zeros(self.size, bool)
        for n in self.free: self.mask[self.slices[n]] = True

    def nominal(self):
        p = np.zeros(self.size); p[:self.size - 2] = super().nominal()[:self.size - 2]; return p

    def to_human(self, p):
        h = super().to_human(p)
        if 'lscale' in self.free: h['lscale'] = 100 * p[self.slices['lscale']][0]
        return h

    def tool_in_base(self, arm, ang, lvl, dc, d, shot=None):
        c = self.ctx
        h = shot['h_enc'] * (1 + d['lscale']) + d['loff']
        axis = Rot.from_rotvec([d['lt_x'], d['lt_y'], 0.0]).apply(c.AXIS)
        L = np.eye(4); L[:3, 3] = axis * h
        m = c.MOUNT[arm] @ d['rm'] if arm == 'right' else c.MOUNT[arm]
        zero = {j: d[f'z_{arm}_{j}'] for j in ZJ}
        sc = {'elbow_flex': d[f's_{arm}_upper'], 'wrist_flex': d[f's_{arm}_fore']}
        comp = {j: d[f'c_{arm}_{j}'] for j in CJ if d[f'c_{arm}_{j}'] != 0.0}
        return L @ m @ c.fk(ang, zero, sc, comp)

    def marker_in_cam(self, s, d):
        return np.linalg.inv(d['cam']) @ self.tool_in_base(s['arm'], s['ang'], s['lvl'], 0.0, d, s) @ d['gm_' + s['att']]


