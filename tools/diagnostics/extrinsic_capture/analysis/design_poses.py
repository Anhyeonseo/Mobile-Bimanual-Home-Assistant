"""Design the next arm-marker capture so the joint calibration can separate what it cannot today.

Starting from the information the 2026-11-13 data already holds (model C of joint_calib.py:
camera + joint zeros + gravity sag + right mount), candidate arm poses are sampled inside the
joint ranges, kept only when the marker would be seen well (and, if a filter is given, only when
the pose is safe to reach), and added greedily: each step takes the pose that most reduces the
summed variance of the target parameters (A-optimal design). A random selection of the same
number of usable poses is the baseline.

usage: python3 design_poses.py REPO DATA JOINT_CALIB.json OUT.json [--n 30] [--cand 3000]
Importable: plan_tour.py uses Designer with a safety filter.
"""
import sys, json, argparse
import numpy as np
sys.path.insert(0, __import__('os').path.dirname(__file__))
from calib_model import Context, Model, ATTS, LEVELS, ARMS, ZJ, CJ, OBJ, OP

W, H = 640, 480
MOVING = ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll')


class Designer:
    def __init__(self, ctx, calib, model_name='C zeros + gravity sag'):
        self.ctx = ctx
        base = {'cam', 'rm'} | {'gm_' + a for a in ctx.atts} | {'dh_' + l for l in ctx.levels}
        zeros = {f'z_{a}_{j}' for a in ARMS for j in ZJ} - {'z_left_base', 'z_right_base'}
        sag = {f'c_{a}_{j}' for a in ARMS for j in CJ}
        self.model = Model(ctx, base | zeros | sag)
        self.p = self.model.expand(calib[model_name]['p'])
        self.sigma = calib[model_name]['sigma_px']
        self.free = np.flatnonzero(self.model.mask)
        self.lab = [self.model.labels[i] for i in self.free]
        self.D = self.model.human_scale()[self.free]
        self.TARGET = [i for i, l in enumerate(self.lab)
                       if l.startswith(('z_', 'c_', 'rm.')) or l in ('cam.tx', 'cam.ty', 'cam.rz')]

    # ---------- one pose ----------
    def corners(self, arm, raw, lvl, pv=None):
        """Marker corner pixels and the marker pose in the camera (pixels None if behind the camera)."""
        dd = self.model.unpack(self.p if pv is None else pv)
        ang = self.ctx.s2.model_angles(raw, self.ctx.P['arms'][arm]['alignment'])
        att = 'L' if arm == 'left' else 'R3'
        cm = np.linalg.inv(dd['cam']) @ self.model.tool_in_base(arm, ang, lvl, 0.0, dd) @ dd['gm_' + att]
        pts = (cm[:3, :3] @ OBJ.T).T + cm[:3, 3]
        if (pts[:, 2] < 0.2).any(): return None, cm
        uv = (self.ctx.K @ pts.T).T
        return uv[:, :2] / uv[:, 2:], cm

    def usable(self, arm, raw, lvl):
        uv, cm = self.corners(arm, raw, lvl)
        if uv is None: return False
        if (uv < 20).any() or (uv[:, 0] > W - 20).any() or (uv[:, 1] > H - 20).any(): return False
        t = cm[:3, 3]
        if not 0.25 < t[2] < 0.95: return False
        tilt = np.degrees(np.arccos(abs(cm[:3, 2] @ (-t / np.linalg.norm(t)))))
        if tilt > 50 or np.linalg.norm(uv - np.roll(uv, -1, 0), axis=1).min() < 35: return False
        tool = (self.model.unpack(self.p)['cam'] @ cm)[:3, 3]
        return tool[2] > self.ctx.GROUND + 0.30 and tool[0] > 0.12

    def rows(self, arm, raw, lvl):
        """Scaled Jacobian (8 x free) of the corner pixels for one pose."""
        J = np.zeros((8, len(self.free)))
        base = self.corners(arm, raw, lvl)[0].ravel()
        own_gm = 'gm_' + ('L' if arm == 'left' else 'R3')
        for k, i in enumerate(self.free):
            l = self.lab[k]
            if l.startswith(('z_', 'c_')) and f'_{arm}_' not in l: continue
            if l.startswith('rm.') and arm == 'left': continue
            if l.startswith('gm_') and not l.startswith(own_gm): continue
            if l.startswith('dh_') and l != 'dh_' + lvl: continue
            q = self.p.copy(); q[i] += self.D[k] * 0.1
            J[:, k] = (self.corners(arm, raw, lvl, q)[0].ravel() - base) / 0.1
        return J

    # ---------- information ----------
    def current_info(self):
        """Information already in the 2026-11-13 data (soft_l1-weighted Gauss-Newton), scaled units."""
        m, ctx = self.model, self.ctx
        def f(pv): return m.residuals(pv, ctx.shots, ctx.planes)
        r0 = f(self.p); Jc = np.zeros((len(r0), len(self.free)))
        for k, i in enumerate(self.free):
            q = self.p.copy(); q[i] += self.D[k] * 0.1; Jc[:, k] = (f(q) - r0) / 0.1
        w = 1 / np.sqrt(1 + (r0 / 2.0) ** 2)
        return (Jc * w[:, None]).T @ (Jc * w[:, None]) + 1e-6 * np.eye(len(self.free))

    def target_std(self, Minv):
        return np.sqrt(self.sigma ** 2 * np.diag(Minv)[self.TARGET])

    def select(self, Js, M, n):
        """Greedy A-optimal choice of n rows-blocks; returns chosen indices and the updated information."""
        Minv = np.linalg.inv(M); chosen = []
        for _ in range(min(n, len(Js))):
            best, bestv, bestM = None, None, None
            for c, J in enumerate(Js):
                if c in chosen: continue
                S = np.eye(len(J)) + J @ Minv @ J.T
                Mn = Minv - Minv @ J.T @ np.linalg.solve(S, J @ Minv)
                v = np.sum(np.diag(Mn)[self.TARGET])
                if bestv is None or v < bestv: best, bestv, bestM = c, v, Mn
            chosen.append(best); Minv = bestM
        return chosen, np.linalg.inv(Minv)

    def sample(self, arm, ranges, rng, count, max_tries, accept=None, levels=('L1', 'L2', 'H50')):
        """Random raw poses inside the joint ranges that pass `usable` (and `accept`, if given)."""
        cands, tries = [], 0
        rest_gripper = 2048 if arm == 'left' else 2235
        while len(cands) < count and tries < max_tries:
            tries += 1
            raw = {j: int(rng.integers(ranges[j]['minimum_unwrapped_raw'] + 20, ranges[j]['maximum_unwrapped_raw'] - 20))
                   for j in MOVING}
            raw['gripper'] = rest_gripper
            lvl = levels[rng.integers(0, len(levels))]
            if self.usable(arm, raw, lvl) and (accept is None or accept(arm, raw, lvl)):
                cands.append((raw, lvl))
        return cands, tries


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('repo'); ap.add_argument('data'); ap.add_argument('calib'); ap.add_argument('out')
    ap.add_argument('--n', type=int, default=30, help='poses per arm'); ap.add_argument('--cand', type=int, default=3000)
    ap.add_argument('--model', default='C zeros + gravity sag')
    args = ap.parse_args()
    ctx = Context(args.repo, args.data)
    R = json.load(open(args.repo + '/config/home_assistant/arm_ranges.development.json'))['arms']
    d = Designer(ctx, json.load(open(args.calib)), args.model)
    rng = np.random.default_rng(7)
    M0 = d.current_info()
    print('target parameters now (1 sigma):', {d.lab[t]: round(float(s), 2) for t, s in zip(d.TARGET, d.target_std(np.linalg.inv(M0)))})
    result = {'arms': {}}; Md, Mr = M0.copy(), M0.copy()
    for arm in ARMS:
        cands, tries = d.sample(arm, R[arm], rng, args.cand, args.cand * 60)
        print(f'{arm}: {len(cands)} usable candidates from {tries} samples', flush=True)
        Js = [d.rows(arm, raw, lvl) for raw, lvl in cands]
        chosen, Md = d.select(Js, Md, args.n)
        for c in rng.choice(len(Js), min(args.n, len(Js)), replace=False): Mr = Mr + Js[c].T @ Js[c]
        result['arms'][arm] = [dict(lift_m=OP[cands[c][1]], level=cands[c][1], raw=cands[c][0]) for c in chosen]
    now, rnd, des = (d.target_std(np.linalg.inv(M)) for M in (M0, Mr, Md))
    print(f'\n{"parameter":22s} {"now":>8s} {"+random":>8s} {"+designed":>9s}   ({args.n} poses per arm)')
    for t, a, b, c in zip(d.TARGET, now, rnd, des):
        print(f'{d.lab[t]:22s} {a:8.2f} {b:8.2f} {c:9.2f}  {d.model.units[d.free[t]]}')
    result.update(targets_now=dict(zip([d.lab[t] for t in d.TARGET], map(float, now))),
                  targets_random=dict(zip([d.lab[t] for t in d.TARGET], map(float, rnd))),
                  targets_designed=dict(zip([d.lab[t] for t in d.TARGET], map(float, des))))
    json.dump(result, open(args.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
