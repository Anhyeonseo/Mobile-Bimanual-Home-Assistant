"""Gripper-tip (TCP) pivot calibration and an end-to-end camera -> arm check, without a ruler.

Capture (torque off, hand-guided, raw_shot.py / `rawshot`): print docs/tcp_touch_target_A4.pdf,
tape it flat, close the gripper and touch the cross-hair with the gripper tip from >= 6 clearly
different wrist/arm orientations. Name the shots TCP-<L|R>-<target letter><nn>, e.g. TCP-L-A01,
and move the sheet to a new spot for each new target letter.

1. Pivot: every touch puts the same unknown tool-frame tip point t on the same unknown base point
   p_target. With the calibrated arm model (joint zeros, sag, lift, right mount) the tool poses T_i
   are known, so min sum |R_i t + o_i - p_target|^2 is linear in (t, p): solved per arm.
   t is compared with the CAD TCP (gripper_frame_link, t = 0).
2. Camera: markers id 1 / id 2 beside the cross-hair give the same point through PnP and the
   calibrated base_from_optical. Camera point minus pivot point = end-to-end camera->arm error.

usage: python3 tcp_touch.py REPO DATA[,DATA...] JOINT_CALIB.json "MODEL LABEL" TOUCH_DIR OUT.json
       DATA must be the folders the calibration was run with (the parameter layout depends on them).
"""
import sys, os, json, glob, re
import numpy as np, cv2
sys.path.insert(0, os.path.dirname(__file__))
from calib_model import Context, Model, OBJ

repo, data, calib_path, label, touch_dir, out = sys.argv[1:7]
OFFSET = {1: np.array([0.080, 0, 0]), 2: np.array([-0.080, 0, 0])}       # marker centre -> cross-hair (make_tcp_target.py)
ctx = Context(repo, data.split(','))
calib = json.load(open(calib_path))[label]
model = Model(ctx, {'lk'} if 'lk' in calib['params'] else set())
try: p = model.expand(calib['p'])
except ValueError as e: sys.exit(str(e))
d = model.unpack(p)
X = d['cam']
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
session = os.path.basename(os.path.abspath(touch_dir))

rows = []
for f in sorted(glob.glob(touch_dir + '/TCP-*/joints.json')):
    name = os.path.basename(os.path.dirname(f)); m = re.match(r'TCP-([LR])-([A-Z])\d+', name)
    if not m: continue
    arm = 'left' if m.group(1) == 'L' else 'right'; target = m.group(1) + '-' + m.group(2)
    j = json.load(open(f)); raw = j['raw_after']
    if max(abs(raw[k] - j['raw_before'][k]) for k in raw if k.startswith(arm)) > 3: continue
    ang = ctx.s2.model_angles({q: raw[f'{arm}/{q}'] for q in ctx.s2.JOINTS}, ctx.P['arms'][arm]['alignment'])
    lift_op = float(j['lift_m_operator'])
    shot = dict(session=session, counts=0)
    if model.lift_linear and session in ctx.lift_ref:
        w = (raw['lift/id10'] - ctx.lift_ref_raw[session] + 2048) % 4096 - 2048
        shot['counts'] = w + 4096 * round(((lift_op - ctx.lift_ref[session]) / ctx.MPC - w) / 4096)
        T = model.tool_in_base(arm, ang, None, 0.0, d, shot)
    else:
        if model.lift_linear: print(f'{name}: session {session} has no calibrated lift; using the operator height')
        ctx.op['_touch'] = lift_op
        T = Model(ctx, set()).tool_in_base(arm, ang, '_touch', 0.0, {**d})
    cam_pts = []
    img = cv2.imread(f.replace('joints.json', 'color.png'), 0)
    k = json.load(open(f.replace('joints.json', 'metadata.json')))['color_intrinsics']
    K = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1]])
    c, ids, _ = det.detectMarkers(img)
    for corners, i in zip(c if ids is not None else [], ids.ravel() if ids is not None else []):
        if int(i) not in OFFSET: continue
        _, r, t = cv2.solvePnP(OBJ, corners.reshape(4, 2), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        q = cv2.Rodrigues(r)[0] @ OFFSET[int(i)] + t.ravel()
        cam_pts.append((X @ np.r_[q, 1])[:3])
    rows.append(dict(name=name, arm=arm, target=target, T=T, cam=cam_pts))
if not rows: sys.exit('no TCP-* touch shots found in ' + touch_dir)

report = {}
for arm in ('left', 'right'):
    rs = [r for r in rows if r['arm'] == arm]
    if not rs: continue
    targets = sorted({r['target'] for r in rs})
    A = np.zeros((3 * len(rs), 3 + 3 * len(targets))); b = np.zeros(3 * len(rs))
    for i, r in enumerate(rs):
        A[3 * i:3 * i + 3, :3] = r['T'][:3, :3]
        k = targets.index(r['target']); A[3 * i:3 * i + 3, 3 + 3 * k:6 + 3 * k] = -np.eye(3)
        b[3 * i:3 * i + 3] = -r['T'][:3, 3]
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    tip = sol[:3]; pts = sol[3:].reshape(-1, 3)
    res = (A @ sol - b).reshape(-1, 3); per = 1000 * np.linalg.norm(res, axis=1)
    cond = np.linalg.cond(A)
    tg = {}
    for k, name in enumerate(targets):
        cams = [q for r in rs if r['target'] == name for q in r['cam']]
        entry = dict(touches=sum(r['target'] == name for r in rs), point_base_mm=(1000 * pts[k]).round(1).tolist())
        if cams:
            diff = 1000 * (np.median(cams, axis=0) - pts[k])
            entry.update(camera_minus_arm_mm=diff.round(1).tolist(), camera_minus_arm_norm_mm=round(float(np.linalg.norm(diff)), 1),
                         camera_views=len(cams))
        tg[name] = entry
    report[arm] = dict(tip_in_tool_frame_mm=(1000 * tip).round(1).tolist(), tip_offset_from_cad_mm=round(float(1000 * np.linalg.norm(tip)), 1),
                       pivot_rms_mm=round(float(np.sqrt(np.mean(per ** 2))), 2), pivot_max_mm=round(float(per.max()), 2),
                       touches=len(rs), condition=round(float(cond), 1), targets=tg,
                       per_touch_mm={r['name']: round(float(e), 2) for r, e in zip(rs, per)})
    print(f'{arm}: tip in tool frame {np.round(1000 * tip, 1)} mm (|offset from CAD TCP| {1000 * np.linalg.norm(tip):.1f} mm), '
          f'pivot RMS {report[arm]["pivot_rms_mm"]} mm over {len(rs)} touches')
    for name, e in tg.items():
        print(f'   target {name}: {e["touches"]} touches' + (f', camera - arm {e["camera_minus_arm_mm"]} mm '
              f'(|{e["camera_minus_arm_norm_mm"]}| mm, {e["camera_views"]} marker views)' if 'camera_views' in e else ', no marker seen'))
json.dump(dict(model=label, calibration=os.path.abspath(calib_path), touch_dir=os.path.abspath(touch_dir), arms=report),
          open(out, 'w'), indent=1)
