"""Joint eye-to-hand fit: fixed D415, both arms, lift levels, per-attachment marker poses."""
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

def group_of(name):   # (arm, marker attachment, lift level, operator lift)
    if name.startswith('L'): return 'left', 'L', {'1': 'L1', '2': 'L2', '3': 'H50'}[name[1]]
    if name.startswith('R3'): return 'right', 'R3', 'H50'          # same encoder reading as L3
    if name.startswith('RB2'): return 'right', 'RB', 'RB'
    if name.startswith(('RC', 'C-')): return 'right', 'RC', 'RCD'
    if name.startswith('RD'): return 'right', 'RD', 'RCD'           # same encoder reading as RC-06/07
OP = {'L1': .20, 'L2': .35, 'H50': .50, 'RB': .20, 'RCD': .35}

shots = []
for f in sorted(glob.glob(DATA + '/*/color.png')):
    name = f.split('/')[-2]
    if not name.startswith(('L', 'R', 'C-')): continue
    j = json.load(open(f.replace('color.png', 'joints.json')))
    raw, before = j['raw_after'], j['raw_before']
    arm, att, lvl = group_of(name)
    g = cv2.imread(f, 0); k = json.load(open(f.replace('color.png', 'metadata.json')))['color_intrinsics']
    K = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1]])
    c, ids, _ = det.detectMarkers(g)
    if ids is None or 4 not in ids.ravel(): continue
    c = c[list(ids.ravel()).index(4)].reshape(4, 2)
    _, r, t = cv2.solvePnP(OBJ, c, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    R = cv2.Rodrigues(r)[0]; t = t.ravel()
    tilt = np.degrees(np.arccos(abs(R[:, 2] @ (-t / np.linalg.norm(t)))))
    side = np.linalg.norm(c - np.roll(c, -1, 0), axis=1).min()
    if att in ('RB', 'RC'): continue
    if tilt > 62 or side < 35 or max(abs(raw[q] - before[q]) for q in raw if q.startswith(arm)) > 3: continue
    cm = np.eye(4); cm[:3, :3] = R; cm[:3, 3] = t
    tool = s2.tool_pose(G, P['arms'][arm], {q: raw[arm + '/' + q] for q in s2.JOINTS})
    shots.append(dict(name=name, arm=arm, att=att, lvl=lvl, c=c, K=K, tool=tool, lift=raw['lift/id10'], cm=cm))
REF = {}
for s in shots: REF.setdefault(s['lvl'], s['lift'])
LV = ['L2', 'H50', 'RCD']; ATT = ['L', 'R3', 'RD']
print('usable shots:', len(shots), {a: sum(s['att'] == a for s in shots) for a in ATT})

def T(v): M = np.eye(4); M[:3, :3] = Rot.from_rotvec(v[:3]).as_matrix(); M[:3, 3] = v[3:]; return M
def V(M): return np.r_[Rot.from_matrix(M[:3, :3]).as_rotvec(), M[:3, 3]]
def wrap(d): return (d + 2048) % 4096 - 2048

def unpack(p, right_mount):
    X = T(p[:6]); i = 6
    gm = {a: T(p[i + 6 * n: i + 6 * n + 6]) for n, a in enumerate(ATT)}; i += 6 * len(ATT)
    dh = {'L1': 0.0, **{l: p[i + n] for n, l in enumerate(LV)}}; i += len(LV)
    sc = 1.0
    rm = T(p[i:i + 6]) if right_mount else np.eye(4); i += 6 if right_mount else 0
    global AX
    AX = np.array([p[i], p[i + 1], 1.0]) / np.linalg.norm([p[i], p[i + 1], 1.0]) if TILT else AXIS
    return X, gm, dh, sc, rm

def predict(s, X, gm, dh, sc, rm):
    h = OP[s['lvl']] + dh[s['lvl']] + wrap(s['lift'] - REF[s['lvl']]) * MPC
    L = np.eye(4); L[:3, 3] = AX * h
    mount = np.array(P['arms'][s['arm']]['base_from_arm_at_lift_zero'])
    if s['arm'] == 'right': mount = mount @ rm
    return np.linalg.inv(X) @ L @ mount @ s['tool'] @ gm[s['att']]

def resid(p, data, right_mount):
    X, gm, dh, sc, rm = unpack(p, right_mount); out = []
    for s in data:
        cm = predict(s, X, gm, dh, sc, rm)
        pts = (cm[:3, :3] @ (OBJ * sc).T).T + cm[:3, 3]
        uv = (s['K'] @ pts.T).T; out.append((uv[:, :2] / uv[:, 2:] - s['c']).ravel())
    return np.concatenate(out)

def init(X):
    p = list(V(X))
    for a in ATT:
        s = next(s for s in shots if s['att'] == a)
        h = OP[s['lvl']]; L = np.eye(4); L[:3, 3] = AXIS * h
        p += list(V(np.linalg.inv(L @ np.array(P['arms'][s['arm']]['base_from_arm_at_lift_zero']) @ s['tool']) @ X @ s['cm']))
    return np.r_[p, np.zeros(len(LV))]

def errs(p, data, right_mount):
    X, gm, dh, sc, rm = unpack(p, right_mount)
    return np.array([1000 * np.linalg.norm(predict(s, X, gm, dh, sc, rm)[:3, 3] - s['cm'][:3, 3] * sc) for s in data])

test = shots[2::5]; train = [s for s in shots if s not in test]
def solve(free_cam, right_mount):
    p0 = np.r_[init(X0), np.zeros(6) if right_mount else [], [0.0, 0.0] if TILT else []]
    mask = np.ones(len(p0), bool)
    if not free_cam: mask[:6] = False
    def f(q):
        p = p0.copy(); p[mask] = q; return resid(p, train, right_mount)
    r = least_squares(f, p0[mask], loss='soft_l1', f_scale=2.0)
    p = p0.copy(); p[mask] = r.x; return p
TILT = False
rows = {}
for label, fc, rmnt in (('0. current profile (camera fixed)', False, False), ('1. camera fitted', True, False), ('2. camera + right-arm mount fitted', True, True)):
    p = solve(fc, rmnt); rows[label] = p
    for arm in ('left', 'right'):
        te = errs(p, [s for s in test if s['arm'] == arm], rmnt); tr = errs(p, [s for s in train if s['arm'] == arm], rmnt)
        print(f'{label:36s} {arm:5s} fit median {np.median(tr):5.1f} | held-out median {np.median(te):5.1f}  95% {np.percentile(te, 95):5.1f}  max {te.max():5.1f} mm  (n={len(te)})')
TILT = True
p = solve(True, True)
for arm in ('left', 'right'):
    te = errs(p, [s for s in test if s['arm'] == arm], True); tr = errs(p, [s for s in train if s['arm'] == arm], True)
    print(f'{"3. + lift axis tilt fitted":36s} {arm:5s} fit median {np.median(tr):5.1f} | held-out median {np.median(te):5.1f}  95% {np.percentile(te, 95):5.1f}  max {te.max():5.1f} mm  (n={len(te)})')
X, gm, dh, sc, rm = unpack(p, True)
print('lift axis tilt from vertical: %.2f deg; forward(+x) %.2f deg, left(+y) %.2f deg' % (np.degrees(np.arccos(AX[2])), np.degrees(np.arctan2(AX[0], AX[2])), np.degrees(np.arctan2(AX[1], AX[2]))))
fwd, right = X[:3, 2], X[:3, 0]
print('camera pitch %.2f deg (profile 45.00), roll %.2f deg, position change vs profile (mm) %s' % (np.degrees(np.arcsin(-fwd[2])), np.degrees(np.arcsin(right[2])), np.round(1000 * (X[:3, 3] - X0[:3, 3]), 1)))
print('lift level offsets vs operator (mm):', {k: round(1000 * v, 1) for k, v in dh.items()})
print('right mount correction: %.1f mm, %.2f deg' % (1000 * np.linalg.norm(rm[:3, 3]), np.degrees(np.linalg.norm(Rot.from_matrix(rm[:3, :3]).as_rotvec()))))
json.dump(dict(base_from_optical=X.tolist(), marker_side_m=SIDE * sc, lift_offsets_m=dh, right_mount_correction=rm.tolist(),
               used=[s['name'] for s in shots], held_out=[s['name'] for s in test]), open(OUT, 'w'), indent=1)
