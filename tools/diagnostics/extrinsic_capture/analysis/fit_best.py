"""Best base_from_optical from all 2026-11-13 data: arm markers (L, R3, RD) + floor/table planes.

Arm shots constrain the camera relative to the arm chain; floor (F-*) and table (TB-*, TF-*) depth
planes constrain it relative to the floor (base_link z = ground_z_m, table = ground + 0.800 m tape).
A ladder of arm-side models is fitted jointly with the camera; the model is chosen on held-out data
(every 5th arm shot, plus F-3 / TB-3 / TF-3). Marker side fixed at the measured 70.0 mm.

usage: python3 fit_best.py REPO DATA OUT.json [plane_weight]
"""
import sys, json, glob, pathlib
import numpy as np, cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
MAIN, DATA, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
WPLANE = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
sys.path.insert(0, MAIN + '/ros2_ws/src/home_robot_tasks')
from home_robot_tasks import s2_geometry as s2
P = json.load(open(MAIN + '/config/home_assistant/s2_coordinates.development.json'))
G = s2.load_geometry(pathlib.Path(MAIN + '/config/home_assistant/official_geometry.json'))
AXIS = np.array(P['lift']['axis_base']); MPC = P['lift']['meters_per_count']
X0 = np.array(P['base_from_optical']); GROUND = P['ground_z_m']; TABLE = GROUND + 0.800
SIDE = 0.070
OBJ = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float) * SIDE / 2
PX_PER_MM = 1.5          # ~617 px focal / 0.4 m: puts plane mm residuals on the marker pixel scale
J = {j['name']: j for j in G['arm_joints']}
rng = np.random.default_rng(0)

# ---------------- arm marker shots ----------------
def group_of(name):
    if name.startswith('L'): return 'left', 'L', {'1': 'L1', '2': 'L2', '3': 'H50'}[name[1]]
    if name.startswith('R3'): return 'right', 'R3', 'H50'
    if name.startswith('RD'): return 'right', 'RD', 'RCD'
OP = {'L1': .20, 'L2': .35, 'H50': .50, 'RCD': .35}
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
shots = []
for f in sorted(glob.glob(DATA + '/*/color.png')):
    name = f.split('/')[-2]; g_ = group_of(name)
    if g_ is None: continue
    arm, att, lvl = g_
    j = json.load(open(f.replace('color.png', 'joints.json'))); raw, before = j['raw_after'], j['raw_before']
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
    ang = s2.model_angles({q: raw[arm + '/' + q] for q in s2.JOINTS}, P['arms'][arm]['alignment'])
    shots.append(dict(name=name, arm=arm, att=att, lvl=lvl, c=c, K=K, ang=ang, lift=raw['lift/id10'], cm=cm))
REF = {}
for s in shots: REF.setdefault(s['lvl'], s['lift'])
def unwrap(raw, op):
    base = raw - REF['L1']; guess = (op - OP['L1']) / MPC
    return base + 4096 * np.round((guess - base) / 4096)
for s in shots: s['dc'] = unwrap(s['lift'], OP[s['lvl']]) - unwrap(REF[s['lvl']], OP[s['lvl']])
test = shots[2::5]; train = [s for s in shots if s not in test]

# ---------------- floor / table planes ----------------
def cloud(folder):
    m = json.load(open(folder + '/metadata.json')); k = m['color_intrinsics']
    z = cv2.imread(folder + '/depth_aligned.png', cv2.IMREAD_UNCHANGED).astype(float) * m['depth_scale_m']
    v, u = np.nonzero((z > 0.2) & (z < 2.5)); z = z[v, u]
    return np.stack([(u - k['ppx']) / k['fx'] * z, (v - k['ppy']) / k['fy'] * z, z], 1)
def ransac(Q, iters=200, tol=0.006):
    best = None
    for _ in range(iters):
        s = Q[rng.choice(len(Q), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
        if nn < 1e-9: continue
        n /= nn; inl = abs((Q - s[0]) @ n) < tol
        if best is None or inl.sum() > best.sum(): best = inl
    return best
planes = []
for name in ['F-1', 'F-2', 'F-3', 'TB-1', 'TB-2', 'TB-3', 'TF-1', 'TF-2', 'TF-3']:
    Q = cloud(f'{DATA}/{name}/cam'); Q = Q[rng.choice(len(Q), min(len(Q), 15000), replace=False)]
    target = GROUND if name.startswith('F') else TABLE
    zb = (X0[:3, :3] @ Q.T).T[:, 2] + X0[2, 3]
    near = Q[abs(zb - target) < 0.05]          # candidate points near the expected surface (nominal frame)
    inl = ransac(near); pts = near[inl]
    pts = pts[rng.choice(len(pts), min(len(pts), 150), replace=False)]
    planes.append(dict(name=name, target=target, pts=pts, n_inl=int(inl.sum())))
ptest = [p for p in planes if p['name'].endswith('-3')]; ptrain = [p for p in planes if p not in ptest]
print('arm shots', len(shots), {a: sum(s['att'] == a for s in shots) for a in ('L', 'R3', 'RD')},
      '| planes', {p['name']: p['n_inl'] for p in planes}, '| plane weight', WPLANE)

# ---------------- model ----------------
def T(v): M = np.eye(4); M[:3, :3] = Rot.from_rotvec(v[:3]).as_matrix(); M[:3, 3] = v[3:]; return M
def V(M): return np.r_[Rot.from_matrix(M[:3, :3]).as_rotvec(), M[:3, 3]]
CHAIN = list(zip(s2.JOINTS, s2.UPSTREAM)); TCP = s2.rigid(P['arms']['left']['reference_from_tcp'])
ALL_LINKS = [n for _, n in CHAIN] + ['gripper_frame_joint']
ZJ = ('shoulder', 'elbow', 'wrist_flex')
def fk(ang, zero, sc):
    tr = {'base_link': np.eye(4)}
    for key, name in CHAIN:
        j = J[name]; o = s2.origin(j); o[:3, 3] *= sc
        tr[j['child']['link']] = tr[j['parent']['link']] @ o @ s2.rotation([float(x) for x in j['axis']['xyz'].split()], ang[key] + zero.get(key, 0.0))
    o = s2.origin(J['gripper_frame_joint']); o[:3, 3] *= sc
    return tr[J['gripper_frame_joint']['parent']['link']] @ o @ TCP
MOUNT = {a: np.array(P['arms'][a]['base_from_arm_at_lift_zero']) for a in ('left', 'right')}
PIVOT = (MOUNT['left'][:3, 3] + MOUNT['right'][:3, 3]) / 2 + AXIS * 0.35

class Model:
    def __init__(self, feats):
        self.f = feats; n = ['cam'] * 6
        for a in ('L', 'R3', 'RD'): n += ['gm_' + a] * 6
        n += ['dh_L2', 'dh_H50', 'dh_RCD']
        if 'rmount' in feats: n += ['rm'] * 6
        if 'scale' in feats: n += ['scale']
        if 'zeros' in feats: n += ['z_left_' + q for q in ZJ] + ['z_right_' + q for q in ZJ]
        if 'zeros_shared' in feats: n += ['z_' + q for q in ZJ]
        if 'body_tilt' in feats: n += ['tilt'] * 2
        if 'ground_free' in feats: n += ['ground']
        self.names = n
    def p0(self):
        p = list(V(X0))
        for a in ('L', 'R3', 'RD'):
            s = next(s for s in shots if s['att'] == a)
            L = np.eye(4); L[:3, 3] = AXIS * OP[s['lvl']]
            p += list(V(np.linalg.inv(L @ MOUNT[s['arm']] @ fk(s['ang'], {}, 1.0)) @ X0 @ s['cm']))
        rest = self.names[6 + 18:]
        return np.r_[p, [1.0 if n == 'scale' else 0.0 for n in rest]]
    def unpack(self, p):
        d = {}; i = 0
        while i < len(p):
            n = self.names[i]
            w = 6 if n in ('cam', 'rm') or n.startswith('gm_') else 2 if n == 'tilt' else 1
            d[n] = T(p[i:i + 6]) if w == 6 else p[i:i + w] if w == 2 else p[i]; i += w
        return d
    def predict(self, s, d):
        h = OP[s['lvl']] + d.get('dh_' + s['lvl'], 0.0) + s['dc'] * MPC
        L = np.eye(4); L[:3, 3] = AXIS * h
        m = MOUNT[s['arm']]
        if s['arm'] == 'right' and 'rm' in d: m = m @ d['rm']
        if 'tilt' in d:
            Rt = np.eye(4); Rt[:3, :3] = Rot.from_rotvec([d['tilt'][0], d['tilt'][1], 0]).as_matrix()
            Pv = np.eye(4); Pv[:3, 3] = PIVOT; Pi = np.eye(4); Pi[:3, 3] = -PIVOT
            L = Pv @ Rt @ Pi @ L
        if 'z_shoulder' in d: z = {q: d['z_' + q] for q in ZJ}
        elif 'z_left_shoulder' in d: z = {q: d[f'z_{s["arm"]}_{q}'] for q in ZJ}
        else: z = {}
        return np.linalg.inv(d['cam']) @ L @ m @ fk(s['ang'], z, d.get('scale', 1.0)) @ d['gm_' + s['att']]
    def resid(self, p, sh, pl):
        d = self.unpack(p); out = []
        for s in sh:
            cm = self.predict(s, d); pts = (cm[:3, :3] @ OBJ.T).T + cm[:3, 3]
            uv = (s['K'] @ pts.T).T; out.append((uv[:, :2] / uv[:, 2:] - s['c']).ravel())
        g = d.get('ground', 0.0); X = d['cam']
        for q in pl:
            zb = (X[:3, :3] @ q['pts'].T).T[:, 2] + X[2, 3]
            out.append(WPLANE * PX_PER_MM * 1000 * (zb - (q['target'] + g)) / np.sqrt(len(q['pts']) / 8))
        return np.concatenate(out)

def arm_err(m, p, sh):
    d = m.unpack(p); return np.array([1000 * np.linalg.norm(m.predict(s, d)[:3, 3] - s['cm'][:3, 3]) for s in sh])
def plane_err(X, pl, g=0.0):
    """per frame: median height error (mm) and far-minus-near height difference (mm, tilt signature)."""
    rows = []
    for q in pl:
        zb = (X[:3, :3] @ q['pts'].T).T[:, 2] + X[2, 3] - (q['target'] + g); depth = q['pts'][:, 2]
        far, near = zb[depth > np.median(depth)], zb[depth <= np.median(depth)]
        rows.append((q['name'], 1000 * np.median(zb), 1000 * (np.median(far) - np.median(near))))
    return rows
def angles(X):
    return np.degrees(np.arcsin(-X[2, 2])), np.degrees(np.arcsin(X[2, 0]))
def summary(label, m, p):
    d = m.unpack(p) if m else {'cam': X0}; X = d['cam']
    if m: el, er = arm_err(m, p, [s for s in test if s['arm'] == 'left']), arm_err(m, p, [s for s in test if s['arm'] == 'right'])
    pe = plane_err(X, ptest + ptrain)
    pt = [r for r in pe if r[0].endswith('-3')]
    fl = np.mean([abs(r[1]) for r in pe if r[0].startswith('F')]); tb = np.mean([abs(r[1]) for r in pe if r[0].startswith('T')])
    pitch, roll = angles(X)
    arm_txt = f'L {np.median(el):4.1f} / R {np.median(er):4.1f} mm (95% {np.percentile(np.r_[el, er], 95):4.1f})' if m else ' ' * 34
    print(f'{label:34s} {arm_txt} | held-out planes {" ".join(f"{n} {h:+5.1f}" for n, h, _ in pt)} | floor {fl:4.1f} table {tb:4.1f} mm | pitch {pitch:.2f} roll {roll:+.2f}')
    return dict(arm_left_heldout_median_mm=float(np.median(el)) if m else None, arm_right_heldout_median_mm=float(np.median(er)) if m else None,
                planes={n: dict(height_err_mm=float(h), far_minus_near_mm=float(fn)) for n, h, fn in pe},
                camera_pitch_deg=float(pitch), camera_roll_deg=float(roll), base_from_optical=X.tolist())

def solve(feats, use_planes=True):
    m = Model(feats); p0 = m.p0()
    r = least_squares(lambda p: m.resid(p, train, ptrain if use_planes else []), p0, loss='soft_l1', f_scale=2.0, x_scale='jac')
    return m, r.x

LADDER = [
    ('arm only: +right mount', ['rmount'], False),
    ('1 +right mount', ['rmount'], True),
    ('2 +right mount +scale', ['rmount', 'scale'], True),
    ('3 +right mount +zeros(shared)', ['rmount', 'zeros_shared'], True),
    ('4 +right mount +zeros(per arm)', ['rmount', 'zeros'], True),
    ('5 +right mount +body tilt', ['rmount', 'body_tilt'], True),
    ('6 +rm +scale +zeros(shared)', ['rmount', 'scale', 'zeros_shared'], True),
    ('7 +rm +scale +zeros +tilt', ['rmount', 'scale', 'zeros_shared', 'body_tilt'], True),
    ('8 = 6 with ground height free', ['rmount', 'scale', 'zeros_shared', 'ground_free'], True),
]
results = {'current profile (CAD + 9.1mm)': summary('current profile (CAD + 9.1mm)', None, None)}
fits = {}
for label, feats, up in LADDER:
    m, p = solve(feats, up); fits[label] = (m, p)
    res = summary(label, m, p)
    d = m.unpack(p)
    res['params'] = {n: (float(v) if np.ndim(v) == 0 else np.asarray(v).tolist()) for n, v in d.items() if n in ('scale', 'ground', 'tilt', 'dh_L2', 'dh_H50', 'dh_RCD') or n.startswith('z_')}
    if 'rm' in d: res['params']['right_mount_mm_deg'] = [float(1000 * np.linalg.norm(d['rm'][:3, 3])), float(np.degrees(np.linalg.norm(Rot.from_matrix(d['rm'][:3, :3]).as_rotvec())))]
    res['position_change_vs_profile_mm'] = (1000 * (d['cam'][:3, 3] - X0[:3, 3])).round(1).tolist()
    results[label] = res
    pr = {k: (round(v * 1000, 1) if k.startswith(('dh', 'ground')) else round(np.degrees(v), 2) if k.startswith('z_') else np.round(np.degrees(v), 2).tolist() if k == 'tilt' else round(v, 4) if np.ndim(v) == 0 else v) for k, v in res['params'].items()}
    print(' ' * 36, 'params', pr, '| cam moved (mm)', res['position_change_vs_profile_mm'])
json.dump(results, open(OUT, 'w'), indent=1)

# ---------------- what does the camera alone buy? (arm model left uncorrected except right mount) ----------------
def solve_fixed_cam(feats, X):
    m = Model(feats); p0 = m.p0(); p0[:6] = V(X); mask = np.ones(len(p0), bool); mask[:6] = False
    def f(q):
        p = p0.copy(); p[mask] = q; return m.resid(p, train, [])
    r = least_squares(f, p0[mask], loss='soft_l1', f_scale=2.0, x_scale='jac'); p = p0.copy(); p[mask] = r.x
    return m, p
best_label = '3 +right mount +zeros(shared)'
Xb = fits[best_label][0].unpack(fits[best_label][1])['cam']
print('\ncamera fixed, arm without joint-zero correction:')
for label, X in (('CAD camera', X0), ('model-3 camera', Xb)):
    for feats in (['rmount'], ['rmount', 'zeros_shared']):
        m, p = solve_fixed_cam(feats, X)
        el, er = arm_err(m, p, [s for s in test if s['arm'] == 'left']), arm_err(m, p, [s for s in test if s['arm'] == 'right'])
        print(f'  {label:16s} arm {"+".join(feats):22s} L {np.median(el):4.1f} / R {np.median(er):4.1f} mm (95% {np.percentile(np.r_[el, er], 95):4.1f})')
