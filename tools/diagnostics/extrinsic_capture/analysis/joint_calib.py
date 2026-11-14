"""Joint camera + arm calibration with identifiability and uncertainty.

Fits base_from_optical together with the arm kinematic corrections (joint zeros, link lengths,
right mount, lift level offsets, marker attachments) from arm-marker shots + floor/table planes,
then reports:
  - held-out marker and plane errors, compared with the current profile
  - which parameter combinations the data cannot separate (SVD of the scaled Jacobian)
  - one-sigma uncertainty of each parameter and the strongly correlated pairs
  - optional bootstrap over shots (--boot N)

usage: python3 joint_calib.py REPO DATA OUT.json [--boot N] [--wplane W]
"""
import sys, json, argparse
import numpy as np
from scipy.optimize import least_squares
sys.path.insert(0, __import__('os').path.dirname(__file__))
from calib_model import Context, Model, ARMS, ZJ, CJ, camera_summary

ap = argparse.ArgumentParser()
ap.add_argument('repo'); ap.add_argument('data'); ap.add_argument('out')
ap.add_argument('--boot', type=int, default=0); ap.add_argument('--wplane', type=float, default=1.0)
ap.add_argument('--cv', type=int, default=0, help='K-fold cross-validation over all arm shots (model choice)')
args = ap.parse_args()
ctx = Context(args.repo, args.data.split(','))   # several capture folders: comma-separated
print(f'arm shots {len(ctx.shots)} (train {len(ctx.train)}, held-out {len(ctx.test)}), planes {len(ctx.planes)}')

BASE = {'cam', 'rm'} | {'gm_' + a for a in ctx.atts} | {'dh_' + l for l in ctx.levels}
# gauge: left base zero fixed (defines heading); right base zero = right mount yaw (rm.rz), so fixed too
ZEROS = {f'z_{a}_{j}' for a in ARMS for j in ZJ} - {'z_left_base', 'z_right_base'}
SCALES = {f's_{a}_{l}' for a in ARMS for l in ('upper', 'fore')}
SAG = {f'c_{a}_{j}' for a in ARMS for j in CJ}
LEVEL_OFFSETS = {'dh_' + l for l in ctx.levels}
LIFT = {'lk'} | {'lo_' + x for x in ctx.sessions[1:]}
MODELS = {
    'A zeros': BASE | ZEROS,
    'B zeros + link lengths': BASE | ZEROS | SCALES,
    'C zeros + gravity sag': BASE | ZEROS | SAG,
    'D zeros + link lengths + gravity sag': BASE | ZEROS | SCALES | SAG,
    # one continuous lift model instead of a free offset per lift level
    'E zeros + gravity sag + linear lift': (BASE - LEVEL_OFFSETS) | ZEROS | SAG | LIFT,
    'F zeros + gravity sag + linear lift + lift tilt': (BASE - LEVEL_OFFSETS) | ZEROS | SAG | LIFT | {'lt_x', 'lt_y'},
}
FULL = 'D zeros + link lengths + gravity sag'


def fit(model, shots, planes, p0=None):
    p0 = model.nominal() if p0 is None else p0.copy()
    m = model.mask
    def f(q):
        p = p0.copy(); p[m] = q; return model.residuals(p, shots, planes)
    r = least_squares(f, p0[m], loss='soft_l1', f_scale=2.0, x_scale='jac')
    p = p0.copy(); p[m] = r.x
    return p, r


def evaluate(model, p):
    el = model.arm_errors(p, [s for s in ctx.test if s['arm'] == 'left'])
    er = model.arm_errors(p, [s for s in ctx.test if s['arm'] == 'right'])
    pe = model.plane_errors(p, ctx.planes)
    return dict(left_median_mm=float(np.median(el)), right_median_mm=float(np.median(er)),
                p95_mm=float(np.percentile(np.r_[el, er], 95)),
                floor_mm=float(np.mean([abs(v) for k, v in pe.items() if k.startswith('F')])),
                table_mm=float(np.mean([abs(v) for k, v in pe.items() if k.startswith('T')])), planes=pe)


report = {}
# current profile: camera and arms at CAD, only marker attachments + lift offsets + right mount fitted
cur = Model(ctx, {'gm_' + a for a in ctx.atts} | {'dh_' + l for l in ctx.levels} | {'rm'}, args.wplane)
pc, _ = fit(cur, ctx.train, [])
report['current profile'] = dict(eval=evaluate(cur, pc), camera=camera_summary(cur.unpack(pc)['cam']))

for label, free in MODELS.items():
    model = Model(ctx, free, args.wplane)
    p, r = fit(model, ctx.train, ctx.ptrain)
    ev = evaluate(model, p)
    # ---- identifiability: SVD of the robust-weighted Jacobian, columns scaled to 1 deg / 1 mm / 1 % ----
    Jf = r.jac
    lab = [l for l, k in zip(model.labels, model.mask) if k]; unit = [u for u, k in zip(model.units, model.mask) if k]
    D = model.human_scale()[model.mask]
    Js = Jf * D
    U, S, Vt = np.linalg.svd(Js, full_matrices=False)
    arm_res = model.residuals(p, ctx.train, [])
    sigma = 1.4826 * np.median(np.abs(arm_res - np.median(arm_res)))      # px, robust
    cov = sigma ** 2 * np.linalg.pinv(Js.T @ Js)                          # in human units (deg / mm / %)
    std = np.sqrt(np.diag(cov)); corr = cov / np.outer(std, std)
    weak = []
    for k in np.argsort(S)[:6]:
        v = Vt[k]; top = np.argsort(-abs(v))[:4]
        weak.append(dict(singular=float(S[k]), combo=[(lab[i], round(float(v[i]), 2)) for i in top]))
    pairs = sorted(((abs(corr[i, j]), lab[i], lab[j], corr[i, j]) for i in range(len(lab)) for j in range(i + 1, len(lab))
                    if abs(corr[i, j]) > 0.85 and not (lab[i].split('.')[0] == lab[j].split('.')[0] and lab[i].startswith('gm_'))),
                   reverse=True)[:12]
    human = model.to_human(p)
    key = [l for l in lab if not l.startswith('gm_')]
    report[label] = dict(
        eval=ev, camera=camera_summary(model.unpack(p)['cam']), base_from_optical=model.unpack(p)['cam'].tolist(),
        sigma_px=float(sigma), condition=float(S.max() / S.min()),
        params={l: dict(value=round(float(human[l]), 3), std=round(float(std[lab.index(l)]), 3), unit=unit[lab.index(l)]) for l in key},
        weakest_directions=weak, correlated_pairs=[dict(a=a, b=b, r=round(float(c), 3)) for _, a, b, c in pairs], p=p.tolist())

    print(f'\n=== {label} ===  free {model.mask.sum()}  sigma {sigma:.2f} px  condition {S.max() / S.min():.0f}')
    print(f'held-out: left {ev["left_median_mm"]:.1f} / right {ev["right_median_mm"]:.1f} mm (95% {ev["p95_mm"]:.1f}) | floor {ev["floor_mm"]:.1f} table {ev["table_mm"]:.1f} mm')
    cs = camera_summary(model.unpack(p)['cam'])
    print(f'camera pitch {cs["pitch_deg"]:.2f} roll {cs["roll_deg"]:+.2f} yaw {cs["yaw_deg"]:+.2f} xyz {cs["xyz_mm"]}')
    print('parameters (value +- 1 sigma):')
    for l in key:
        i = lab.index(l); print(f'   {l:22s} {human[l]:+8.2f} +- {std[i]:6.2f} {unit[i]}')
    print('weakest directions (small singular value = data barely sees this combination):')
    for w in weak: print(f'   s={w["singular"]:8.2f}  {w["combo"]}')
    print('strongly correlated pairs:')
    for _, a, b, c in pairs: print(f'   r={c:+.2f}  {a}  <->  {b}')

c0 = report['current profile']['eval']
print(f'\ncurrent profile held-out: left {c0["left_median_mm"]:.1f} / right {c0["right_median_mm"]:.1f} mm (95% {c0["p95_mm"]:.1f}) | floor {c0["floor_mm"]:.1f} table {c0["table_mm"]:.1f} mm')

if args.cv:
    # every shot is held out exactly once; planes are all used (they do not inform the arm)
    print(f'\n{args.cv}-fold cross-validation (all {len(ctx.shots)} shots held out once):')
    cv = {}
    for label, free in {'current profile': {'gm_' + a for a in ctx.atts} | {'dh_' + l for l in ctx.levels} | {'rm'}, **MODELS}.items():
        model = Model(ctx, free, args.wplane); err = {}
        for k in range(args.cv):
            hold = [s for i, s in enumerate(ctx.shots) if i % args.cv == k]
            fitset = [s for i, s in enumerate(ctx.shots) if i % args.cv != k]
            p, _ = fit(model, fitset, [] if label == 'current profile' else ctx.planes)
            for s, e in zip(hold, model.arm_errors(p, hold)): err[s['name']] = (s['arm'], e)
        el = np.array([e for a, e in err.values() if a == 'left']); er = np.array([e for a, e in err.values() if a == 'right'])
        cv[label] = dict(left_median_mm=float(np.median(el)), right_median_mm=float(np.median(er)),
                         all_median_mm=float(np.median(np.r_[el, er])), p95_mm=float(np.percentile(np.r_[el, er], 95)),
                         per_shot={n: float(e) for n, (a, e) in err.items()})
        print(f'   {label:40s} left {np.median(el):5.1f} (n={len(el)})  right {np.median(er):5.1f} (n={len(er)})  all {np.median(np.r_[el, er]):5.1f}  95% {np.percentile(np.r_[el, er], 95):5.1f} mm', flush=True)
    report['cross_validation'] = cv

if args.boot:
    model = Model(ctx, MODELS[FULL], args.wplane)
    p_full = np.array(report[FULL]['p'])
    rng = np.random.default_rng(1); draws = []
    for b in range(args.boot):
        sh = [ctx.train[i] for i in rng.integers(0, len(ctx.train), len(ctx.train))]
        pb, _ = fit(model, sh, ctx.ptrain, p_full)
        h = model.to_human(pb); cs = camera_summary(model.unpack(pb)['cam'])
        draws.append({**{k: v for k, v in h.items() if not k.startswith('gm_')}, 'cam_pitch': cs['pitch_deg'], 'cam_yaw': cs['yaw_deg'],
                      'cam_x': cs['xyz_mm'][0], 'cam_y': cs['xyz_mm'][1], 'cam_z': cs['xyz_mm'][2]})
        print(f'bootstrap {b + 1}/{args.boot}', flush=True)
    keys = draws[0].keys()
    report['bootstrap'] = {k: dict(mean=float(np.mean([d[k] for d in draws])), std=float(np.std([d[k] for d in draws]))) for k in keys}
    print('bootstrap std:', {k: round(v['std'], 2) for k, v in report['bootstrap'].items()})

json.dump(report, open(args.out, 'w'), indent=1)
