"""Eye-to-hand fit for the fixed D415 against the left arm marker (stage A/B)."""
import sys, json, glob
import numpy as np, cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
MAIN, DATA = sys.argv[1], sys.argv[2]
sys.path.insert(0, MAIN + '/ros2_ws/src/home_robot_tasks')
from home_robot_tasks import s2_geometry as s2
P = json.load(open(MAIN + '/config/home_assistant/s2_coordinates.development.json'))
G = s2.load_geometry(s2.Path(MAIN + '/config/home_assistant/official_geometry.json')) if hasattr(s2, 'Path') else s2.load_geometry(__import__('pathlib').Path(MAIN + '/config/home_assistant/official_geometry.json'))
ARM = P['arms']['left']; AXIS = np.array(P['lift']['axis_base']); MPC = P['lift']['meters_per_count']
X0 = np.array(P['base_from_optical']); X0_cad = np.array(P['camera_nominal_base_from_optical'])
SIDE = 0.06985
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
OBJ = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float) * SIDE / 2

shots = []
for f in sorted(glob.glob(DATA + '/L[123]-*/color.png')):
    name = f.split('/')[-2]; level = int(name[1])
    g = cv2.imread(f, 0); m = json.load(open(f.replace('color.png', 'metadata.json'))); k = m['color_intrinsics']
    K = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1]])
    c, ids, _ = det.detectMarkers(g)
    if ids is None or 4 not in ids.flatten(): continue
    c = c[list(ids.flatten()).index(4)].reshape(4, 2)
    _, r, t = cv2.solvePnP(OBJ, c, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    R = cv2.Rodrigues(r)[0]; t = t.ravel()
    tilt = np.degrees(np.arccos(abs(R[:, 2] @ (-t / np.linalg.norm(t)))))
    j = json.load(open(f.replace('color.png', 'joints.json'))); raw = j['raw_after']
    if max(abs(raw[k2] - j['raw_before'][k2]) for k2 in raw) > 3: continue
    tool = s2.tool_pose(G, ARM, {jn: raw['left/' + jn] for jn in s2.JOINTS})
    cm = np.eye(4); cm[:3, :3] = R; cm[:3, 3] = t
    shots.append(dict(name=name, level=level, corners=c, K=K, tool=tool, lift_raw=raw['lift/id10'], lift_op=j['lift_m_operator'], tilt=tilt, cam_marker=cm))
good = [s for s in shots if s['tilt'] < 62 and np.linalg.norm(s['corners'] - np.roll(s['corners'], -1, 0), axis=1).min() >= 35]
print(f'shots with marker: {len(shots)}, used (tilt<62, side>=35px): {len(good)}')
levels = sorted({s['level'] for s in good})
ref = {L: [s['lift_raw'] for s in good if s['level'] == L][0] for L in levels}
op = {L: [s['lift_op'] for s in good if s['level'] == L][0] for L in levels}

def T(v):  # 6-vector -> 4x4
    M = np.eye(4); M[:3, :3] = Rot.from_rotvec(v[:3]).as_matrix(); M[:3, 3] = v[3:6]; return M
def V(M): return np.r_[Rot.from_matrix(M[:3, :3]).as_rotvec(), M[:3, 3]]

def base_from_tool(s, dh):
    h = op[s['level']] + dh.get(s['level'], 0.0) + (s['lift_raw'] - ref[s['level']]) * MPC
    L = np.eye(4); L[:3, 3] = AXIS * h
    return L @ np.array(ARM['base_from_arm_at_lift_zero']) @ s['tool']

def unpack(p, free_cam, free_lift):
    i = 0
    if free_cam: X = T(p[:6]); i = 6
    else: X = FIXED_X
    gm = T(p[i:i + 6]); i += 6
    dh = {}
    if free_lift:
        for L in levels[1:]: dh[L] = p[i]; i += 1
    return X, gm, dh

def residual(p, data, free_cam, free_lift):
    X, gm, dh = unpack(p, free_cam, free_lift)
    out = []
    for s in data:
        cam_marker = np.linalg.inv(X) @ base_from_tool(s, dh) @ gm
        pts = (cam_marker[:3, :3] @ OBJ.T).T + cam_marker[:3, 3]
        uv = (s['K'] @ pts.T).T; uv = uv[:, :2] / uv[:, 2:]
        out.append((uv - s['corners']).ravel())
    return np.concatenate(out)

def centre_err_mm(X, gm, dh, data):
    e = []
    for s in data:
        pred = np.linalg.inv(X) @ base_from_tool(s, dh) @ gm
        e.append(1000 * np.linalg.norm(pred[:3, 3] - s['cam_marker'][:3, 3]))
    return np.array(e)

def init_gm(X):
    s = good[0]; return V(np.linalg.inv(base_from_tool(s, {})) @ X @ s['cam_marker'])

def fit(data, free_cam, free_lift, X_init):
    global FIXED_X; FIXED_X = X_init
    p0 = np.r_[V(X_init) if free_cam else [], init_gm(X_init), [0.0] * (len(levels) - 1 if free_lift else 0)]
    r = least_squares(residual, p0, args=(data, free_cam, free_lift), loss='soft_l1', f_scale=2.0)
    return unpack(r.x, free_cam, free_lift)

def report(label, X, gm, dh, train, test):
    tr, te = centre_err_mm(X, gm, dh, train), centre_err_mm(X, gm, dh, test)
    rp = np.abs(residual(np.r_[V(X), V(gm), [dh.get(L, 0) for L in levels[1:]]], test, True, True)).reshape(-1, 2)
    print(f'{label:46s} fit {np.median(tr):5.1f} mm | held-out median {np.median(te):5.1f}  95% {np.percentile(te, 95):5.1f}  max {te.max():5.1f} mm')
    return te

# hold out every 5th usable shot
test = good[2::5]; train = [s for s in good if s not in test]
print(f'train {len(train)}  held-out {len(test)}  levels {levels}  operator lift {op}')
print('--- marker-centre error (mm): camera prediction vs arm prediction ---')
for label, Xi in (('current profile (CAD + 9.1mm Z)', X0), ('CAD nominal (no Z fix)', X0_cad)):
    X, gm, dh = fit(train, False, False, Xi); report(label + ', marker offset fitted', X, gm, dh, train, test)
X, gm, dh = fit(train, True, False, X0); report('A: camera fitted, lift = operator values', X, gm, dh, train, test)
X, gm, dh = fit(train, True, True, X0); report('B: camera + lift offsets fitted', X, gm, dh, train, test)
d = np.linalg.inv(X0) @ X
print('--- result B vs current profile ---')
print('camera position change (mm, base x/y/z):', np.round(1000 * (X[:3, 3] - X0[:3, 3]), 1))
print('camera rotation change (deg):', round(float(np.degrees(np.linalg.norm(Rot.from_matrix(d[:3, :3]).as_rotvec()))), 2))
fwd = X[:3, 2]; print('optical axis pitch below horizontal: fitted %.2f deg, profile %.2f deg' % (np.degrees(np.arcsin(-fwd[2])), np.degrees(np.arcsin(-X0[2, 2]))))
right = X[:3, 0]; print('camera roll (image x axis vs horizontal): fitted %.2f deg, profile %.2f deg' % (np.degrees(np.arcsin(right[2])), np.degrees(np.arcsin(X0[2, 0]))))
print('camera height above base_link: fitted %.1f mm, profile %.1f mm' % (1000 * X[2, 3], 1000 * X0[2, 3]))
print('lift offsets vs operator values (mm):', {L: round(1000 * v, 1) for L, v in dh.items()})
print('marker in gripper frame (mm):', np.round(1000 * gm[:3, 3], 1))
json.dump(dict(base_from_optical=X.tolist(), gripper_from_marker=gm.tolist(), lift_offsets_m=dh, used=[s['name'] for s in good], held_out=[s['name'] for s in test]), open(sys.argv[3], 'w'), indent=1)
