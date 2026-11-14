"""Shared model for the joint camera + arm calibration of the fixed D415 (eye-to-hand).

One parameter vector holds everything the arm-marker shots and the floor/table planes can inform:

  cam            base_from_optical (rotvec + t)                          6
  gm_<att>       marker pose on the gripper, per tape attachment         6 each
  dh_<level>     lift height offset per lift level (L1 = reference)      1 each
  rm             right-arm mount correction (right of the CAD mount)     6
  z_<arm>_<j>    joint zero offset, j in base/shoulder/elbow/wrist_flex  1 each
  s_<arm>_<l>    link length scale, l in upper (elbow_flex origin),
                 fore (wrist_flex origin)                                1 each

Gauge: base_link is defined by the wheels, which no image sees. The left-arm CAD mount and the
left base (shoulder_pan) zero are held fixed, so base_link's horizontal position and heading are
"where the CAD says the left arm is"; the floor fixes height, pitch and roll. wrist_roll zero is
not a parameter: the per-attachment marker pose absorbs it exactly.

Imported by joint_calib.py and design_poses.py.
"""
import sys, json, glob, pathlib
import numpy as np, cv2
from scipy.spatial.transform import Rotation as Rot

SIDE = 0.070                       # ArUco id 4 black square, measured
OBJ = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float) * SIDE / 2
PX_PER_MM = 1.5                    # ~617 px focal / 0.4 m: plane mm residuals on the marker pixel scale
OP = {'L1': .20, 'L2': .35, 'H50': .50, 'RCD': .35}   # operator-entered lift heights (m)
ATTS = ('L', 'R3', 'RD')
LEVELS = ('L2', 'H50', 'RCD')
ZJ = ('base', 'shoulder', 'elbow', 'wrist_flex')
CJ = ('shoulder', 'elbow', 'wrist_flex')        # joints that carry a gravity moment
ARMS = ('left', 'right')


def T(v):
    M = np.eye(4); M[:3, :3] = Rot.from_rotvec(v[:3]).as_matrix(); M[:3, 3] = v[3:]; return M


def V(M):
    return np.r_[Rot.from_matrix(M[:3, :3]).as_rotvec(), M[:3, 3]]


def group_of(name, joints=None, session=''):
    """(arm, marker attachment, lift level) of one shot, or None to skip it.

    Shots from sweep_log.py / calib_tour.py carry a calibration_group; their level is
    '<session>@<lift>' because the lift height is re-established in every session (the session is
    the capture day folder, written by the tools; folders without it count as their own session).
    2026-11-13 shots are grouped by their folder names."""
    g = (joints or {}).get('calibration_group')
    if g: return g['arm'], g['attachment'], f'{g.get("session", session)}@{g["level"]}'
    if name.startswith('L'): return 'left', 'L', {'1': 'L1', '2': 'L2', '3': 'H50'}[name[1]]
    if name.startswith('R3'): return 'right', 'R3', 'H50'     # same lift encoder reading as L3
    if name.startswith('RD'): return 'right', 'RD', 'RCD'
    return None                                               # RB2 / RC / C-01: marker came loose


class Context:
    """Profile, geometry, arm-marker shots and floor/table planes for one or more capture folders.

    data: a folder or a list of folders. Attachments and lift levels are collected from the shots
    (ctx.atts, ctx.levels, ctx.op); the reference lift level (offset fixed at 0) is 'L1' when the
    2026-11-13 capture is included, otherwise the first level found."""

    def __init__(self, repo, data, seed=0):
        sys.path.insert(0, repo + '/ros2_ws/src/home_robot_tasks')
        from home_robot_tasks import s2_geometry as s2
        self.s2 = s2
        self.P = json.load(open(repo + '/config/home_assistant/s2_coordinates.development.json'))
        self.G = s2.load_geometry(pathlib.Path(repo + '/config/home_assistant/official_geometry.json'))
        self.J = {j['name']: j for j in self.G['arm_joints']}
        self.AXIS = np.array(self.P['lift']['axis_base']); self.MPC = self.P['lift']['meters_per_count']
        self.X0 = np.array(self.P['base_from_optical']); self.GROUND = self.P['ground_z_m']
        self.MOUNT = {a: np.array(self.P['arms'][a]['base_from_arm_at_lift_zero']) for a in ARMS}
        self.TCP = s2.rigid(self.P['arms']['left']['reference_from_tcp'])
        self.CHAIN = list(zip(s2.JOINTS, s2.UPSTREAM))
        self.rng = np.random.default_rng(seed)
        self.datas = [data] if isinstance(data, str) else list(data)
        self.data = self.datas[0]
        self.op = dict(OP)
        self._load_shots(); self._load_planes()
        seen = lambda key: list(dict.fromkeys(s[key] for s in self.shots))
        self.atts = tuple(seen('att'))
        levels = seen('lvl')
        self.ref_level = 'L1' if 'L1' in levels else levels[0]
        self.levels = tuple(l for l in levels if l != self.ref_level)
        self.test = self.shots[2::5]; self.train = [s for s in self.shots if s not in self.test]
        self.ptest = [p for p in self.planes if p['name'].endswith('-3')]
        self.ptrain = [p for p in self.planes if p not in self.ptest]

    # ---------- data ----------
    def _load_shots(self):
        det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                      cv2.aruco.DetectorParameters())
        shots = []
        files = [(d, f) for d in self.datas for f in sorted(glob.glob(d + '/*/color.png'))]
        for d, f in files:
            name = f.split('/')[-2]
            try: j = json.load(open(f.replace('color.png', 'joints.json')))
            except FileNotFoundError: continue                        # camera-only shot
            g = group_of(name, j, pathlib.Path(d).name)
            if g is None: continue
            arm, att, lvl = g
            if 'calibration_group' in j: self.op[lvl] = float(j['calibration_group']['lift_m'])
            raw, before = j['raw_after'], j['raw_before']
            k = json.load(open(f.replace('color.png', 'metadata.json')))['color_intrinsics']
            K = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1]])
            c, ids, _ = det.detectMarkers(cv2.imread(f, 0))
            if ids is None or 4 not in ids.ravel(): continue
            c = c[list(ids.ravel()).index(4)].reshape(4, 2)
            _, r, t = cv2.solvePnP(OBJ, c, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            R = cv2.Rodrigues(r)[0]; t = t.ravel()
            tilt = np.degrees(np.arccos(abs(R[:, 2] @ (-t / np.linalg.norm(t)))))
            side = np.linalg.norm(c - np.roll(c, -1, 0), axis=1).min()
            if tilt > 62 or side < 35 or max(abs(raw[q] - before[q]) for q in raw if q.startswith(arm)) > 3: continue
            cm = np.eye(4); cm[:3, :3] = R; cm[:3, 3] = t
            ang = self.s2.model_angles({q: raw[arm + '/' + q] for q in self.s2.JOINTS}, self.P['arms'][arm]['alignment'])
            shots.append(dict(name=name, arm=arm, att=att, lvl=lvl, c=c, K=K, ang=ang, lift=raw['lift/id10'], cm=cm, tilt=tilt,
                              session=(j.get('calibration_group') or {}).get('session', pathlib.Path(d).name), lift_op=float(j.get('lift_m_operator', self.op[lvl]))))
        ref = {}
        for s in shots: ref.setdefault(s['lvl'], s['lift'])
        self.K = shots[0]['K']
        # lift drift within one level (a few mm), relative to that level's first shot
        for s in shots: s['dc'] = (s['lift'] - ref[s['lvl']] + 2048) % 4096 - 2048
        # continuous lift: encoder counts since the session's first shot, unwrapped over whole turns
        # with the operator-entered heights (84 mm per turn; operator heights are good to a few cm)
        self.sessions = list(dict.fromkeys(s['session'] for s in shots)); self.lift_ref = {}; self.lift_ref_raw = {}
        for sess in self.sessions:
            r = next(s for s in shots if s['session'] == sess); self.lift_ref[sess] = r['lift_op']; self.lift_ref_raw[sess] = r['lift']
            for s in shots:
                if s['session'] != sess: continue
                w = (s['lift'] - r['lift'] + 2048) % 4096 - 2048
                s['counts'] = w + 4096 * round(((s['lift_op'] - r['lift_op']) / self.MPC - w) / 4096)
        self.shots = shots

    def _load_planes(self):
        def cloud(folder):
            m = json.load(open(folder + '/metadata.json')); k = m['color_intrinsics']
            z = cv2.imread(folder + '/depth_aligned.png', cv2.IMREAD_UNCHANGED).astype(float) * m['depth_scale_m']
            v, u = np.nonzero((z > 0.2) & (z < 2.5)); z = z[v, u]
            return np.stack([(u - k['ppx']) / k['fx'] * z, (v - k['ppy']) / k['fy'] * z, z], 1)

        def ransac(Q, iters=200, tol=0.006):
            best = None
            for _ in range(iters):
                s = Q[self.rng.choice(len(Q), 3, replace=False)]
                n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
                if nn < 1e-9: continue
                n /= nn; inl = abs((Q - s[0]) @ n) < tol
                if best is None or inl.sum() > best.sum(): best = inl
            return best
        planes = []
        folders = [f'{d}/{name}/cam' for d in self.datas
                   for name in ['F-1', 'F-2', 'F-3', 'TB-1', 'TB-2', 'TB-3', 'TF-1', 'TF-2', 'TF-3']]
        for folder in folders:
            if not pathlib.Path(folder, 'depth_aligned.png').exists(): continue
            name = pathlib.Path(folder).parent.name
            Q = cloud(folder); Q = Q[self.rng.choice(len(Q), min(len(Q), 15000), replace=False)]
            target = self.GROUND + (0.0 if name.startswith('F') else 0.800)
            zb = (self.X0[:3, :3] @ Q.T).T[:, 2] + self.X0[2, 3]
            near = Q[abs(zb - target) < 0.05]
            pts = near[ransac(near)]
            planes.append(dict(name=name, target=target, pts=pts[self.rng.choice(len(pts), min(len(pts), 150), replace=False)]))
        self.planes = planes

    # ---------- kinematics ----------
    def _chain(self, ang, zero, scale):
        s2, J = self.s2, self.J
        tr = {'base_link': np.eye(4)}; joints = {}
        for key, name in self.CHAIN:
            j = J[name]; o = s2.origin(j); o[:3, 3] *= scale.get(name, 1.0)
            axis = np.array([float(x) for x in j['axis']['xyz'].split()])
            pre = tr[j['parent']['link']] @ o
            joints[key] = (pre[:3, 3].copy(), pre[:3, :3] @ axis)
            tr[j['child']['link']] = pre @ s2.rotation(axis, ang[key] + zero.get(key, 0.0))
        g = J['gripper_frame_joint']
        return tr[g['parent']['link']] @ s2.origin(g) @ self.TCP, joints

    def fk(self, ang, zero, scale, comp=None):
        """Tool pose in the arm frame with joint-zero offsets (rad), per-joint origin scales and
        gravity sag: joint j turns by comp[j] * (gravity moment arm of the links beyond it, m).
        Distal masses are lumped at the elbow, the wrist and the tool point; the arm frame's z is up."""
        tool, joints = self._chain(ang, zero, scale)
        if not comp: return tool
        pts = [joints['elbow'][0], joints['wrist_flex'][0], tool[:3, 3]]
        down = np.array([0.0, 0.0, -1.0]); extra = dict(zero)
        for key, c in comp.items():
            p, a = joints[key]; first = {'shoulder': 0, 'elbow': 1, 'wrist_flex': 2}[key]
            arm = sum(a @ np.cross(q - p, down) for q in pts[first:])
            extra[key] = extra.get(key, 0.0) + c * arm
        return self._chain(ang, extra, scale)[0]


class Model:
    """Parameter layout + residuals. `free` = set of parameter names to estimate; the rest stay nominal."""

    def __init__(self, ctx, free, wplane=1.0):
        self.ctx, self.wplane = ctx, wplane
        spec = [('cam', 6)] + [('gm_' + a, 6) for a in ctx.atts] + [('dh_' + l, 1) for l in ctx.levels] + [('rm', 6)]
        spec += [(f'z_{a}_{j}', 1) for a in ARMS for j in ZJ] + [(f's_{a}_{l}', 1) for a in ARMS for l in ('upper', 'fore')]
        spec += [(f'c_{a}_{j}', 1) for a in ARMS for j in CJ]
        # continuous lift (used when 'lk' is free): height = session reference + lo_<session>
        #   + lk * metres_per_count * counts, along the CAD lift axis tilted by (lt_x, lt_y)
        spec += [('lk', 1), ('lt_x', 1), ('lt_y', 1)] + [('lo_' + s, 1) for s in ctx.sessions[1:]]
        self.spec = spec
        self.lift_linear = 'lk' in free
        self.slices, i = {}, 0
        for n, w in spec: self.slices[n] = slice(i, i + w); i += w
        self.size = i
        self.free = set(free)
        self.mask = np.zeros(self.size, bool)
        for n in self.free: self.mask[self.slices[n]] = True
        # one label per scalar, in human units for reports
        self.labels, self.units = [], []
        for n, w in spec:
            if w == 6:
                self.labels += [f'{n}.r{c}' for c in 'xyz'] + [f'{n}.t{c}' for c in 'xyz']
                self.units += ['deg'] * 3 + ['mm'] * 3
            else:
                self.labels.append(n)
                self.units.append('%' if n.startswith('s_') or n == 'lk' else 'deg' if n.startswith(('z_', 'lt_')) else
                                  'deg/10cm' if n.startswith('c_') else 'mm')

    def nominal(self):
        c = self.ctx; p = np.zeros(self.size)
        p[self.slices['cam']] = V(c.X0)
        for a in c.atts:
            s = next(s for s in c.shots if s['att'] == a)
            L = np.eye(4); L[:3, 3] = c.AXIS * c.op[s['lvl']]
            p[self.slices['gm_' + a]] = V(np.linalg.inv(L @ c.MOUNT[s['arm']] @ c.fk(s['ang'], {}, {})) @ c.X0 @ s['cm'])
        for n, w in self.spec:
            if n.startswith('s_') or n == 'lk': p[self.slices[n]] = 1.0
        return p

    def expand(self, p):
        """A parameter vector saved before later parameters were appended (e.g. the continuous lift):
        keep its values and fill the missing tail with nominal values."""
        p = np.asarray(p, float)
        if len(p) == self.size: return p
        if len(p) > self.size: raise ValueError('parameter vector longer than this layout: different data folders?')
        q = self.nominal(); q[:len(p)] = p
        return q

    def unpack(self, p):
        d = {}
        for n, w in self.spec:
            v = p[self.slices[n]]
            d[n] = T(v) if w == 6 else float(v[0])
        return d

    def tool_in_base(self, arm, ang, lvl, dc, d, shot=None):
        c = self.ctx
        if self.lift_linear and shot is not None:
            sess = shot['session']
            h = c.lift_ref[sess] + d.get('lo_' + sess, 0.0) + d['lk'] * c.MPC * shot['counts']
            axis = Rot.from_rotvec([d['lt_x'], d['lt_y'], 0.0]).apply(c.AXIS)
        else:
            h = c.op[lvl] + d.get('dh_' + lvl, 0.0) + dc * c.MPC; axis = c.AXIS
        L = np.eye(4); L[:3, 3] = axis * h
        m = c.MOUNT[arm] @ d['rm'] if arm == 'right' else c.MOUNT[arm]
        zero = {j: d[f'z_{arm}_{j}'] for j in ZJ}
        sc = {'elbow_flex': d[f's_{arm}_upper'], 'wrist_flex': d[f's_{arm}_fore']}
        comp = {j: d[f'c_{arm}_{j}'] for j in CJ if d[f'c_{arm}_{j}'] != 0.0}
        return L @ m @ c.fk(ang, zero, sc, comp)

    def marker_in_cam(self, s, d):
        return np.linalg.inv(d['cam']) @ self.tool_in_base(s['arm'], s['ang'], s['lvl'], s['dc'], d, s) @ d['gm_' + s['att']]

    def residuals(self, p, shots, planes):
        d = self.unpack(p); out = []
        for s in shots:
            cm = self.marker_in_cam(s, d); pts = (cm[:3, :3] @ OBJ.T).T + cm[:3, 3]
            uv = (s['K'] @ pts.T).T; out.append((uv[:, :2] / uv[:, 2:] - s['c']).ravel())
        X = d['cam']
        for q in planes:
            zb = (X[:3, :3] @ q['pts'].T).T[:, 2] + X[2, 3]
            out.append(self.wplane * PX_PER_MM * 1000 * (zb - q['target']) / np.sqrt(len(q['pts']) / 8))
        return np.concatenate(out) if out else np.zeros(0)

    def arm_errors(self, p, shots):
        d = self.unpack(p)
        return np.array([1000 * np.linalg.norm(self.marker_in_cam(s, d)[:3, 3] - s['cm'][:3, 3]) for s in shots])

    def plane_errors(self, p, planes):
        X = self.unpack(p)['cam']
        return {q['name']: 1000 * float(np.median((X[:3, :3] @ q['pts'].T).T[:, 2] + X[2, 3] - q['target'])) for q in planes}

    def to_human(self, p):
        """Free-parameter values in deg / mm / % (rotvec components as deg, link scales as % change)."""
        out = {}
        for i, (lab, u) in enumerate(zip(self.labels, self.units)):
            if not self.mask[i]: continue
            v = p[i]
            out[lab] = (np.degrees(v) if u == 'deg' else (v - 1) * 100 if u == '%' else
                        np.degrees(v) * 0.1 if u == 'deg/10cm' else 1000 * v)
        return out

    def human_scale(self):
        """Column scales: the size of a 'meaningful' change per parameter (1 deg, 1 mm, 1 %, 1 deg per 10 cm)."""
        return np.array([np.radians(1) if u == 'deg' else 0.01 if u == '%' else np.radians(1) / 0.1 if u == 'deg/10cm'
                         else 0.001 for u in self.units])


def camera_summary(X):
    fwd = X[:3, 2]
    return dict(pitch_deg=float(np.degrees(np.arcsin(-fwd[2]))), roll_deg=float(np.degrees(np.arcsin(X[2, 0]))),
                yaw_deg=float(np.degrees(np.arctan2(fwd[1], fwd[0]))), xyz_mm=(1000 * X[:3, 3]).round(1).tolist())
