"""Joint camera + arm calibration with the lift height taken from the lift encoder (2026-11-17 data).

calib_model.py places the arms at the operator-entered lift height. On 2026-11-17 those heights were
hand-set guesses; the lift encoder relative to the measured bottom is better. Here every shot's height is
    h = h_enc * (1 + lscale) + loff,   h_enc = (bottom_raw - raw) * |m/count| + 0.084 m * turns
with `turns` chosen so h_enc is closest to the operator height. lscale (lift travel scale) and loff (offset
of the lift zero) are fitted; the floor/table planes fix the camera height, so loff is observable.

Then: K-fold cross-validation per arm and lift level (total / lateral / along-ray error), and the
gripper-tip touch check (pivot + camera point of the touch target) with the selected model.

usage: python3 calib_encoder_lift.py REPO CAPTURE_DIR TOUR_DIR[,TOUR_DIR...] OUT.json [--bottom-raw 3900] [--cv 5]
"""
import sys, os, json, glob, re, argparse
import numpy as np, cv2
from scipy.optimize import least_squares
sys.path.insert(0, os.path.dirname(__file__))
from calib_model import Context, Model, ARMS, ZJ, CJ, OBJ, camera_summary

ap = argparse.ArgumentParser()
ap.add_argument('repo'); ap.add_argument('capture'); ap.add_argument('tours'); ap.add_argument('out')
ap.add_argument('--bottom-raw', type=int, default=3900); ap.add_argument('--cv', type=int, default=5)
args = ap.parse_args()

from encoder_lift_model import EncModel, enc_height as _enc_height, add_encoder_heights, MPC, TURN

def enc_height(raw, op):
    return _enc_height(raw, op, args.bottom_raw)

ctx = Context(args.repo, [args.capture] + args.tours.split(','))
add_encoder_heights(ctx, args.bottom_raw)
print('shots', len(ctx.shots), {(a, l.split('@')[-1]): sum(s['arm'] == a and s['lvl'] == l for s in ctx.shots)
                                 for a in ARMS for l in [ctx.ref_level, *ctx.levels]})


BASE = {'cam', 'rm'} | {'gm_' + a for a in ctx.atts}
Z = {f'z_{a}_{j}' for a in ARMS for j in ZJ} - {'z_left_base', 'z_right_base'}
S = {f'c_{a}_{j}' for a in ARMS for j in CJ}
L = {'lscale', 'loff'}; T = {'lt_x', 'lt_y'}
MODELS = {'A zeros': BASE | Z, 'AL zeros + lift': BASE | Z | L, 'CL zeros + sag + lift': BASE | Z | S | L,
          'CLT zeros + sag + lift + lift tilt': BASE | Z | S | L | T}


def fit(free, shots, planes):
    m = EncModel(ctx, free); p0 = m.nominal(); k = m.mask
    def put(q):
        p = p0.copy(); p[k] = q; return p
    r = least_squares(lambda q: m.residuals(put(q), shots, planes), p0[k], loss='soft_l1', f_scale=2.0, x_scale='jac')
    return m, put(r.x)


def errors(m, p, shots):
    d = m.unpack(p); out = []
    for s in shots:
        pred = m.marker_in_cam(s, d)[:3, 3]; obs = s['cm'][:3, 3]; u = obs / np.linalg.norm(obs); e = (pred - obs) * 1000
        a = e @ u; out.append((s, np.linalg.norm(e), np.linalg.norm(e - a * u), a))
    return out


report = {}
groups = [(a, l) for a in ARMS for l in [ctx.ref_level, *ctx.levels]]
for label, free in MODELS.items():
    err = []
    for k in range(args.cv):
        hold = ctx.shots[k::args.cv]; hid = {id(s) for s in hold}
        m, p = fit(free, [s for s in ctx.shots if id(s) not in hid], ctx.planes)
        err += errors(m, p, hold)
    row = {}
    for a, l in groups:
        e = [x for x in err if x[0]['arm'] == a and x[0]['lvl'] == l]
        if e: row[f'{a}@{l.split("@")[-1]}'] = dict(total=float(np.median([x[1] for x in e])), lateral=float(np.median([x[2] for x in e])),
                                                     ray=float(np.median([x[3] for x in e])), n=len(e))
    allt = [x[1] for x in err]
    m, p = fit(free, ctx.shots, ctx.planes)
    h = m.to_human(p)
    report[label] = dict(cv=row, cv_all_median=float(np.median(allt)), cv_p95=float(np.percentile(allt, 95)),
                         params={k: round(float(v), 3) for k, v in h.items() if not k.startswith('gm_')},
                         camera=camera_summary(m.unpack(p)['cam']), planes=m.plane_errors(p, ctx.planes), p=p.tolist())
    print(f'\n=== {label}: {args.cv}-fold all {np.median(allt):.1f} mm (95% {np.percentile(allt, 95):.1f})')
    for g, v in row.items(): print(f'   {g:12s} total {v["total"]:5.1f}  lateral {v["lateral"]:4.1f}  ray {v["ray"]:+6.1f}  (n={v["n"]})')
    print('   ' + '  '.join(f'{k}={v:+.2f}' for k, v in h.items() if k in ('lscale', 'loff', 'lt_x', 'lt_y') or k.startswith(('z_', 'rm.t'))))

best = min(report, key=lambda k: report[k]['cv_all_median'])
print('\nselected:', best)

# ---------- gripper-tip touch check ----------
m = EncModel(ctx, MODELS[best]); p = np.array(report[best]['p']); d = m.unpack(p); X = d['cam']
meas = {}
mp = os.path.join(args.capture, 'measurements.json')
if os.path.exists(mp): meas = json.load(open(mp)).get('tcp_target', {})
side = float(meas.get('marker_side_mm', 70.0)) / 1000; off = float(meas.get('offset_mm', 80.0)) / 1000
OBJT = OBJ * side / 0.070; OFFSET = {1: np.array([off, 0, 0]), 2: np.array([-off, 0, 0])}
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
rows = []
for f in sorted(glob.glob(os.path.join(args.capture, 'TCP-*/joints.json'))):
    name = os.path.basename(os.path.dirname(f)); mt = re.match(r'TCP-([LR])-([A-Z])\d+', name)
    if not mt: continue
    arm = 'left' if mt.group(1) == 'L' else 'right'
    j = json.load(open(f)); raw = j['raw_after']
    if max(abs(raw[k] - j['raw_before'][k]) for k in raw if k.startswith(arm)) > 3: continue
    ang = ctx.s2.model_angles({q: raw[f'{arm}/{q}'] for q in ctx.s2.JOINTS}, ctx.P['arms'][arm]['alignment'])
    shot = dict(h_enc=enc_height(raw['lift/id10'], float(j['lift_m_operator'])))
    Tt = m.tool_in_base(arm, ang, None, 0.0, d, shot)
    km = json.load(open(f.replace('joints.json', 'metadata.json')))['color_intrinsics']
    K = np.array([[km['fx'], 0, km['ppx']], [0, km['fy'], km['ppy']], [0, 0, 1]])
    c, ids, _ = det.detectMarkers(cv2.imread(f.replace('joints.json', 'color.png'), 0)); cams = []
    for corners, i in zip(c if ids is not None else [], ids.ravel() if ids is not None else []):
        if int(i) not in OFFSET: continue
        _, r, t = cv2.solvePnP(OBJT, corners.reshape(4, 2), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        cams.append((X @ np.r_[cv2.Rodrigues(r)[0] @ OFFSET[int(i)] + t.ravel(), 1])[:3])
    rows.append(dict(name=name, arm=arm, target=mt.group(1) + '-' + mt.group(2), T=Tt, cam=cams))
touch = {}
for arm in ARMS:
    rs = [r for r in rows if r['arm'] == arm]
    targets = sorted({r['target'] for r in rs})
    if len(rs) < 4: continue
    # unknowns: tip offset t (3, tool frame) + one base point per target (3 each)
    A = np.zeros((3 * len(rs), 3 + 3 * len(targets))); b = np.zeros(3 * len(rs))
    for i, r in enumerate(rs):
        A[3 * i:3 * i + 3, :3] = r['T'][:3, :3]; ti = targets.index(r['target'])
        A[3 * i:3 * i + 3, 3 + 3 * ti:6 + 3 * ti] = -np.eye(3); b[3 * i:3 * i + 3] = -r['T'][:3, 3]
    sol = np.linalg.lstsq(A, b, rcond=None)[0]; tip = sol[:3]
    res = (A @ sol - b).reshape(-1, 3); spread = np.linalg.norm(res, axis=1) * 1000
    out = dict(tip_offset_mm=(tip * 1000).round(1).tolist(), pivot_residual_median_mm=float(np.median(spread)), targets={})
    for ti, tg in enumerate(targets):
        pin = sol[3 + 3 * ti:6 + 3 * ti]
        cams = [c for r in rs if r['target'] == tg for c in r['cam']]
        # arm tip with the CAD tip (t = 0): the point the arm model thinks it touched
        arm_cad = np.mean([r['T'][:3, 3] for r in rs if r['target'] == tg], axis=0)
        entry = dict(shots=sum(r['target'] == tg for r in rs), pivot_point_mm=(pin * 1000).round(1).tolist())
        if cams:
            cam = np.mean(cams, axis=0)
            entry['camera_minus_arm_mm'] = ((cam - pin) * 1000).round(1).tolist()
            entry['camera_minus_arm_cadtip_mm'] = ((cam - arm_cad) * 1000).round(1).tolist()
        out['targets'][tg] = entry
    touch[arm] = out
    print(f'\ntouch {arm}: tip offset vs CAD {out["tip_offset_mm"]} mm, pivot residual median {out["pivot_residual_median_mm"]:.1f} mm')
    for tg, e in out['targets'].items():
        print(f'   {tg}: {e["shots"]} shots  camera - arm (fitted tip) {e.get("camera_minus_arm_mm")}  (CAD tip) {e.get("camera_minus_arm_cadtip_mm")}  [base x fwd, y left, z up, mm]')
report['selected'] = best; report['touch'] = touch
json.dump(report, open(args.out, 'w'), indent=1, default=float)
