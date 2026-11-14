"""Find the source of the ~2.7% length mismatch with the marker fixed at its measured 70.0 mm.

Each model adds one candidate cause on top of the base model (camera + per-attachment marker pose
+ per-level lift offset) and is fitted on the left arm only. The right arm (R3, RD) is then checked
with the left-arm camera/arm parameters held fixed; only its marker attachments (and optionally the
right-arm mount correction) are fitted.

usage: python3 fit_scale.py REPO DATA OUT.json
"""
import sys, json, glob, pathlib
import numpy as np, cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
MAIN, DATA, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
sys.path.insert(0, MAIN + '/ros2_ws/src/home_robot_tasks')
from home_robot_tasks import s2_geometry as s2
P = json.load(open(MAIN + '/config/home_assistant/s2_coordinates.development.json'))
G = s2.load_geometry(pathlib.Path(MAIN + '/config/home_assistant/official_geometry.json'))
AXIS = np.array(P['lift']['axis_base']); MPC = P['lift']['meters_per_count']
X0 = np.array(P['base_from_optical'])
SIDE = 0.070
OBJ = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float) * SIDE / 2
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
J = {j['name']: j for j in G['arm_joints']}

def group_of(name):
    if name.startswith('L'): return 'left', 'L', {'1': 'L1', '2': 'L2', '3': 'H50'}[name[1]]
    if name.startswith('R3'): return 'right', 'R3', 'H50'
    if name.startswith('RD'): return 'right', 'RD', 'RCD'
    return None
OP = {'L1': .20, 'L2': .35, 'H50': .50, 'RCD': .35}

shots = []
for f in sorted(glob.glob(DATA + '/*/color.png')):
    name = f.split('/')[-2]
    g_ = group_of(name)
    if g_ is None: continue
    arm, att, lvl = g_
    j = json.load(open(f.replace('color.png', 'joints.json')))
    raw, before = j['raw_after'], j['raw_before']
    g = cv2.imread(f, 0); k = json.load(open(f.replace('color.png', 'metadata.json')))['color_intrinsics']
    K = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1]])
    c, ids, _ = det.detectMarkers(g)
    if ids is None or 4 not in ids.ravel(): continue
    c = c[list(ids.ravel()).index(4)].reshape(4, 2)
    _, r, t = cv2.solvePnP(OBJ, c, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    R = cv2.Rodrigues(r)[0]; t = t.ravel()
    tilt = np.degrees(np.arccos(abs(R[:, 2] @ (-t / np.linalg.norm(t)))))
    side = np.linalg.norm(c - np.roll(c, -1, 0), axis=1).min()
    if tilt > 62 or side < 35 or max(abs(raw[q] - before[q]) for q in raw if q.startswith(arm)) > 3: continue
    cm = np.eye(4); cm[:3, :3] = R; cm[:3, 3] = t
    angles = s2.model_angles({q: raw[arm + '/' + q] for q in s2.JOINTS}, P['arms'][arm]['alignment'])
    shots.append(dict(name=name, arm=arm, att=att, lvl=lvl, c=c, K=K, ang=angles, lift=raw['lift/id10'], cm=cm))

# lift: unwrap encoder counts relative to L1 using the operator heights to pick the turn count
REF = {}
for s in shots: REF.setdefault(s['lvl'], s['lift'])
ref0 = REF['L1']
def unwrap(raw, op):
    guess = (op - OP['L1']) / MPC
    base = raw - ref0
    return base + 4096 * np.round((guess - base) / 4096)
for s in shots: s['lift_counts'] = unwrap(s['lift'], OP[s['lvl']])
print('usable shots:', len(shots), {a: sum(s['att'] == a for s in shots) for a in ('L', 'R3', 'RD')})
print('lift counts from L1 per level:', {l: int(unwrap(REF[l], OP[l])) for l in REF},
      ' nominal-scale heights (m):', {l: round(OP['L1'] + unwrap(REF[l], OP[l]) * MPC, 4) for l in REF})

def T(v): M = np.eye(4); M[:3, :3] = Rot.from_rotvec(v[:3]).as_matrix(); M[:3, 3] = v[3:]; return M
def V(M): return np.r_[Rot.from_matrix(M[:3, :3]).as_rotvec(), M[:3, 3]]

CHAIN = list(zip(s2.JOINTS, s2.UPSTREAM))
def fk(ang, zero, scale):
    """FK with joint-zero offsets (rad) and per-joint origin translation scales."""
    tr = {'base_link': np.eye(4)}
    for key, name in CHAIN:
        j = J[name]; o = s2.origin(j); o[:3, 3] *= scale.get(name, 1.0)
        axis = [float(x) for x in j['axis']['xyz'].split()]
        tr[j['child']['link']] = tr[j['parent']['link']] @ o @ s2.rotation(axis, ang[key] + zero.get(key, 0.0))
    tcp = J['gripper_frame_joint']; o = s2.origin(tcp); o[:3, 3] *= scale.get('gripper_frame_joint', 1.0)
    return tr[tcp['parent']['link']] @ o @ s2.rigid(P['arms']['left']['reference_from_tcp'])

ALL_LINKS = [n for _, n in CHAIN] + ['gripper_frame_joint']

class Model:
    """Parameter layout built from a list of feature names."""
    def __init__(self, feats, atts, levels):
        self.feats, self.atts, self.levels = feats, atts, levels
        self.names = ['cam'] * 6
        for a in atts: self.names += ['gm_' + a] * 6
        if 'lift_scale' in feats: self.names += ['lift_k']
        else: self.names += ['dh_' + l for l in levels if l != 'L1']
        if 'link_scale' in feats: self.names += ['s_all']
        if 'arm_scale2' in feats: self.names += ['s_upper', 's_fore']
        for q in ('shoulder', 'elbow', 'wrist_flex'):
            if 'zero_' + q in feats or ('zeros' in feats): self.names += ['z_' + q]
        if 'right_mount' in feats: self.names += ['rm'] * 6
        if 'marker_free' in feats: self.names += ['side']
    def p0(self, X):
        p = list(V(X))
        for a in self.atts:
            s = next(s for s in shots if s['att'] == a)
            L = np.eye(4); L[:3, 3] = AXIS * OP[s['lvl']]
            mount = np.array(P['arms'][s['arm']]['base_from_arm_at_lift_zero'])
            p += list(V(np.linalg.inv(L @ mount @ fk(s['ang'], {}, {})) @ X @ s['cm']))
        for n in self.names[6 + 6 * len(self.atts):]:
            p.append(1.0 if n in ('lift_k', 's_all', 's_upper', 's_fore', 'side') else 0.0)
        return np.array(p, float)
    def unpack(self, p):
        d = {}; i = 0
        while i < len(p):
            n = self.names[i]
            if n == 'cam' or n.startswith('gm_') or n == 'rm':
                d[n] = T(p[i:i + 6]); i += 6
            else:
                d[n] = p[i]; i += 1
        return d
    def predict(self, s, d):
        if 'lift_k' in d: h = OP['L1'] + d['lift_k'] * MPC * s['lift_counts']
        else: h = OP[s['lvl']] + d.get('dh_' + s['lvl'], 0.0) + (s['lift_counts'] - unwrap(REF[s['lvl']], OP[s['lvl']])) * MPC
        L = np.eye(4); L[:3, 3] = AXIS * h
        mount = np.array(P['arms'][s['arm']]['base_from_arm_at_lift_zero'])
        if s['arm'] == 'right' and 'rm' in d: mount = mount @ d['rm']
        sc = {}
        if 's_all' in d: sc = {n: d['s_all'] for n in ALL_LINKS}
        if 's_upper' in d: sc['elbow_flex'] = d['s_upper']; sc['wrist_flex'] = d['s_fore']
        zero = {q: d['z_' + q] for q in ('shoulder', 'elbow', 'wrist_flex') if 'z_' + q in d}
        return np.linalg.inv(d['cam']) @ L @ mount @ fk(s['ang'], zero, sc) @ d['gm_' + s['att']]
    def resid(self, p, data):
        d = self.unpack(p); out = []; side = d.get('side', 1.0)
        for s in data:
            cm = self.predict(s, d)
            pts = (cm[:3, :3] @ (OBJ * side).T).T + cm[:3, 3]
            uv = (s['K'] @ pts.T).T; out.append((uv[:, :2] / uv[:, 2:] - s['c']).ravel())
        return np.concatenate(out)
    def errs(self, p, data):
        d = self.unpack(p); side = d.get('side', 1.0)
        return np.array([1000 * np.linalg.norm(self.predict(s, d)[:3, 3] - s['cm'][:3, 3] * side) for s in data])

def fit(m, p0, data, free):
    mask = np.array([free(n) for n in m.names])
    def f(q):
        p = p0.copy(); p[mask] = q; return m.resid(p, data)
    r = least_squares(f, p0[mask], loss='soft_l1', f_scale=2.0)
    p = p0.copy(); p[mask] = r.x; return p

left = [s for s in shots if s['arm'] == 'left']; right = [s for s in shots if s['arm'] == 'right']
lt = left[2::5]; ltr = [s for s in left if s not in lt]
rt = right[2::5]; rtr = [s for s in right if s not in rt]

MODELS = [
    ('A. base (marker 70mm)', []),
    ('B. + arm link scale (all links)', ['link_scale']),
    ('C. + upper/forearm scale', ['arm_scale2']),
    ('D. + shoulder zero', ['zero_shoulder']),
    ('E. + elbow zero', ['zero_elbow']),
    ('F. + wrist_flex zero', ['zero_wrist_flex']),
    ('G. + shoulder/elbow/wrist zeros', ['zeros']),
    ('H. lift linear scale (no per-level offset)', ['lift_scale']),
    ('I. zeros + link scale', ['zeros', 'link_scale']),
    ('Z. ref: marker size free', ['marker_free']),
]
results = {}
print('\n%-44s %18s %18s %18s   params' % ('model (left-only fit)', 'left fit/held', 'right held (rigid)', 'right held (+mount)'))
for label, feats in MODELS:
    m = Model(feats, ['L'], ['L1', 'L2', 'H50'])
    p = fit(m, m.p0(X0), ltr, lambda n: True)
    d = m.unpack(p)
    tr_e, te_e = m.errs(p, ltr), m.errs(p, lt)
    # right-arm check: same camera/arm params; fit only right attachments (+ RCD level offset), optionally mount
    out = []
    for rmnt in (False, True):
        mr = Model(feats + (['right_mount'] if rmnt else []), ['L', 'R3', 'RD'], ['L1', 'L2', 'H50', 'RCD'])
        pr = mr.p0(d['cam'])
        dl = dict(zip(m.names, p))
        for i, n in enumerate(mr.names):   # copy left-arm solution in
            if n in m.names and n not in ('cam',) and not n.startswith('gm_'):
                pr[i] = p[m.names.index(n)]
        pr[:6] = p[:6]; pr[6:12] = p[6:12]
        pr = fit(mr, pr, rtr, lambda n: n in ('gm_R3', 'gm_RD', 'dh_RCD', 'rm'))
        out.append(mr.errs(pr, rt))
    extra = {n: round(float(v), 4) for n, v in d.items() if np.ndim(v) == 0}
    for n in ('z_shoulder', 'z_elbow', 'z_wrist_flex'):
        if n in extra: extra[n] = '%.2fdeg' % np.degrees(extra[n])
    for n in list(extra):
        if n.startswith('dh_'): extra[n] = '%.1fmm' % (1000 * float(extra[n]))
    X = d['cam']; fwd, rgt = X[:3, 2], X[:3, 0]
    pitch = np.degrees(np.arcsin(-fwd[2])); roll = np.degrees(np.arcsin(rgt[2]))
    print('%-44s %7.1f / %5.1f mm   %7.1f mm         %7.1f mm         pitch %.2f roll %.2f %s' % (
        label, np.median(tr_e), np.median(te_e), np.median(out[0]), np.median(out[1]), pitch, roll, extra))
    results[label] = dict(left_fit_median_mm=float(np.median(tr_e)), left_heldout_median_mm=float(np.median(te_e)),
                          left_heldout_max_mm=float(te_e.max()), right_heldout_rigid_median_mm=float(np.median(out[0])),
                          right_heldout_mount_median_mm=float(np.median(out[1])), camera_pitch_deg=float(pitch),
                          camera_roll_deg=float(roll), base_from_optical=X.tolist(),
                          params={k: (v if isinstance(v, str) else float(v)) for k, v in extra.items()})
json.dump(results, open(OUT, 'w'), indent=1)
