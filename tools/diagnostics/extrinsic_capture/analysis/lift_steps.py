"""Lift travel per encoder count from camera shots of a marker on the lift plate (block D). No ruler.

The camera is fixed to the base and the lift carries the arm plate, so a marker on the plate seen by the
camera measures the real plate height at every lift step. Against the encoder counts this gives
  - metres per count (and mm per motor turn), to compare with the official 84 mm/turn, the 11/01 top-bottom
    estimate (85.34) and the current profile (11/17 calibration),
  - linearity (quadratic term, residuals), up/down hysteresis, sideways drift / tilt of the travel,
  - the real total travel from the bottom step to the top step (profile travel_m is the CAD 0.6438 m).

Capture: start at the lift bottom; raise in steps SMALLER THAN HALF A TURN (< 42 mm, use 35-40 mm) so the
single-turn lift raw unwraps without ambiguity; at each stop wait for the raw to settle, write it down and
`camshot LIFT-U00`, `LIFT-U01`, ...; optionally come back down with `LIFT-D..`. STEPS.csv lists the shots
in capture order:  name,lift_raw   (header optional; a third column up/down is optional, else from the
name: LIFT-U.. = up, LIFT-D.. = down). If a shot folder has joints.json (rawshot), its lift raw is used.

usage: python3 lift_steps.py REPO SESSION_DIR STEPS.csv [--board 9x6 --square-mm 25.0 | --marker-id ID --side-mm 66.2]
                             [--out out.json] [--plot out.png]
Target on the plate: prefer the checkerboard (docs/calibration_checkerboard_A4.pdf, 9 x 6 inner corners,
25.0 mm squares, printed at 100 %) glued flat on a rigid board: 54 sub-pixel corners give the range to
~0.1 %. A single 66 mm ArUco marker (45-60 px) is only good to ~2-4 % in range (synthetic test,
2026-11-18), too coarse to separate the +1.6 % and +3.9 % scale estimates.
Marker side defaults to SESSION_DIR/measurements.json "lift_marker_side_mm", else 70 mm. Measure the
square or marker with calipers: the metric scale of every height here is proportional to it.
"""
import argparse, csv, json, os
import numpy as np, cv2

ap = argparse.ArgumentParser()
ap.add_argument('repo'); ap.add_argument('session'); ap.add_argument('steps')
ap.add_argument('--marker-id', type=int, help='ArUco id on the lift plate (default: the only id seen in the first shot)')
ap.add_argument('--side-mm', type=float); ap.add_argument('--out'); ap.add_argument('--plot')
ap.add_argument('--board', help='checkerboard inner corners COLSxROWS, e.g. 9x6 (overrides the ArUco marker)')
ap.add_argument('--square-mm', type=float, default=25.0)
args = ap.parse_args()

prof = json.load(open(os.path.join(args.repo, 'config/home_assistant/s2_coordinates.development.json')))
B = np.array(prof['base_from_optical'], float)
AXIS = np.array(prof['lift']['axis_base'], float); AXIS /= np.linalg.norm(AXIS)
MPC_PROFILE = float(prof['lift']['meters_per_count'])          # signed: + height per + count
SIGN = np.sign(MPC_PROFILE)
REF = {'official 84 mm/turn': 0.084 / 4096, '11/01 top-bottom 85.34 mm/turn': 0.085342 / 4096,
       'profile (11/17 calibration)': abs(MPC_PROFILE)}
meas = os.path.join(args.session, 'measurements.json')
side_mm = args.side_mm or float((json.load(open(meas)) if os.path.exists(meas) else {}).get('lift_marker_side_mm', 70.0))
OBJ = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float) * side_mm / 2000
BOARD = tuple(int(v) for v in args.board.lower().split('x')) if args.board else None
BOARD_OBJ = (np.array([[i - (BOARD[0] - 1) / 2, j - (BOARD[1] - 1) / 2, 0] for j in range(BOARD[1]) for i in range(BOARD[0])], float)
             * args.square_mm / 1000) if BOARD else None
_PAR = cv2.aruco.DetectorParameters()
_PAR.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # default NONE gives integer corners: ~1 px = 2 % range at 45 px
DET = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), _PAR)


def folder_of(name):
    d = os.path.join(args.session, name)
    return os.path.join(d, 'cam') if os.path.exists(os.path.join(d, 'cam', 'color.png')) else d


def read_steps(path):
    rows = []
    for rec in csv.reader(open(path)):
        if not rec or rec[0].strip().startswith('#') or rec[0].strip().lower() == 'name': continue
        name, raw = rec[0].strip(), int(float(rec[1]))
        d = rec[2].strip().lower() if len(rec) > 2 and rec[2].strip() else ('down' if name.upper().startswith('LIFT-D') else 'up')
        j = os.path.join(args.session, name, 'joints.json')
        if os.path.exists(j): raw = int(json.load(open(j))['raw_after']['lift/id10'])
        rows.append(dict(name=name, raw=raw, dir=d))
    return rows


def board(folder):
    """Checkerboard centre and normal in camera coordinates (centre is invariant to the corner order)."""
    m = json.load(open(os.path.join(folder, 'metadata.json'))); k = m['color_intrinsics']
    K = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1.]])
    gray = cv2.imread(os.path.join(folder, 'color.png'), 0)
    ok, c = cv2.findChessboardCornersSB(gray, BOARD, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok: return 'board', None
    c = c.reshape(-1, 2)
    _, r, t = cv2.solvePnP(BOARD_OBJ, c, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    R = cv2.Rodrigues(r)[0]; t = t.ravel()
    reproj = cv2.projectPoints(BOARD_OBJ, r, t, K, None)[0].reshape(-1, 2)
    z = cv2.imread(os.path.join(folder, 'depth_aligned.png'), cv2.IMREAD_UNCHANGED).astype(float) * m['depth_scale_m']
    mask = np.zeros(z.shape, np.uint8); cv2.fillConvexPoly(mask, cv2.convexHull(c.astype(np.int32)), 1)
    zin = z[(mask > 0) & (z > 0)]
    return 'board', dict(p_cam=t, n_cam=R[:, 2], side_px=float(np.linalg.norm(c[1] - c[0])),
                         reproj_px=float(np.sqrt(np.mean(np.sum((reproj - c) ** 2, 1)))),
                         depth_minus_pnp_mm=None if zin.size < 50 else float(1000 * (np.median(zin) - np.median(
                             [(R @ q + t)[2] for q in BOARD_OBJ]))))


def marker(folder, want):
    if BOARD: return board(folder)
    m = json.load(open(os.path.join(folder, 'metadata.json'))); k = m['color_intrinsics']
    K = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1.]])
    gray = cv2.imread(os.path.join(folder, 'color.png'), 0)
    c, ids, _ = DET.detectMarkers(gray)
    ids = [] if ids is None else [int(i) for i in ids.ravel()]
    if want is None:
        if len(ids) != 1: raise SystemExit(f'{folder}: {len(ids)} markers seen {ids}; pass --marker-id')
        want = ids[0]
    if want not in ids: return want, None
    q = c[ids.index(want)].reshape(4, 2)
    _, r, t = cv2.solvePnP(OBJ, q, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    R = cv2.Rodrigues(r)[0]; t = t.ravel()
    # depth cross-check: median aligned depth inside the marker vs the PnP depth of the same pixels
    z = cv2.imread(os.path.join(folder, 'depth_aligned.png'), cv2.IMREAD_UNCHANGED).astype(float) * m['depth_scale_m']
    mask = np.zeros(z.shape, np.uint8); cv2.fillConvexPoly(mask, q.astype(np.int32), 1)
    zin = z[(mask > 0) & (z > 0)]
    side_px = float(np.linalg.norm(q - np.roll(q, -1, 0), axis=1).min())
    return want, dict(p_cam=t, n_cam=R[:, 2], side_px=side_px,
                      depth_minus_pnp_mm=float(1000 * (np.median(zin) - t[2])) if zin.size > 20 else None)


rows = read_steps(args.steps)
if len(rows) < 3: raise SystemExit('need at least 3 steps')
# unwrap the single-turn raw: consecutive steps must be < half a turn apart
counts, warn = [0], []
for a, b in zip(rows, rows[1:]):
    d = (b['raw'] - a['raw'] + 2048) % 4096 - 2048
    if abs(d) > 1900: warn.append(f"{a['name']} -> {b['name']}: {d} counts ({abs(d) * 0.021:.0f} mm) is close to half a turn; unwrap may be wrong")
    counts.append(counts[-1] + d)
mid = args.marker_id; data = []
for r, c in zip(rows, counts):
    mid, mk = marker(folder_of(r['name']), mid)
    if mk is None: warn.append(f"{r['name']}: marker {mid} not found, skipped"); continue
    p = B[:3, :3] @ mk['p_cam'] + B[:3, 3]; n = B[:3, :3] @ mk['n_cam']
    data.append(dict(r, counts=int(c), nominal_m=float(SIGN * c * abs(MPC_PROFILE)), s_m=float(AXIS @ p),
                     lateral_m=(p - (AXIS @ p) * AXIS).tolist(), normal=n.tolist(), side_px=mk['side_px'],
                     depth_minus_pnp_mm=mk['depth_minus_pnp_mm']))
if len(data) < 3: raise SystemExit('fewer than 3 usable steps:\n' + '\n'.join(warn))

x = np.array([SIGN * d['counts'] for d in data], float)        # + = up
s = np.array([d['s_m'] for d in data])
up = np.array([d['dir'] == 'up' for d in data])
A1 = np.stack([np.ones_like(x), x], 1); c1, *_ = np.linalg.lstsq(A1, s, rcond=None); res1 = s - A1 @ c1
A2 = np.stack([np.ones_like(x), x, x ** 2], 1); c2, *_ = np.linalg.lstsq(A2, s, rcond=None); res2 = s - A2 @ c2
mpc = float(c1[1]); span = x.max() - x.min()
hyst = None
if up.sum() >= 2 and (~up).sum() >= 2:                     # common slope, separate offsets for up / down
    A = np.stack([up.astype(float), (~up).astype(float), x], 1); ch, *_ = np.linalg.lstsq(A, s, rcond=None)
    hyst = float(1000 * (ch[1] - ch[0]))
lat = np.array([d['lateral_m'] for d in data]); tilt_mm_per_100 = (100 * np.polyfit(s, lat, 1)[0]).tolist()
nrm = np.array([d['normal'] for d in data]); plate_rot_deg = float(np.degrees(np.arccos(np.clip(abs(nrm @ nrm[0]), -1, 1))).max())
top = float(mpc * (x.max() - x[0]))

print(f'lift target {"checkerboard %dx%d, %.2f mm squares" % (*BOARD, args.square_mm) if BOARD else "marker id %s, side %s mm" % (mid, side_mm)}, {len(data)} steps, {span:.0f} counts ({span / 4096:.2f} turns); profile {prof["calibration_id"]}')
print(f'{"step":10s} {"dir":4s} {"raw":>5s} {"counts":>7s} {"camera mm":>9s} {"nominal mm":>10s} {"resid mm":>8s} {"side px":>7s} {"depth-pnp":>9s}')
for d, r in zip(data, res1):
    dp = d['depth_minus_pnp_mm']
    print(f"{d['name']:10s} {d['dir']:4s} {d['raw']:5d} {d['counts']:7d} {1000 * (d['s_m'] - data[0]['s_m']):9.1f} "
          f"{1000 * (d['nominal_m'] - data[0]['nominal_m']):10.1f} {1000 * r:8.2f} {d['side_px']:7.0f} {'   -' if dp is None else f'{dp:9.1f}'}")
print(f'\nfit: {1e6 * mpc:.4f} um/count = {mpc * 4096 * 1000:.2f} mm/turn   residual rms {1000 * np.sqrt(np.mean(res1 ** 2)):.2f} mm, max {1000 * abs(res1).max():.2f} mm')
for k, v in REF.items(): print(f'  vs {k:32s} {100 * (mpc / v - 1):+6.2f} %')
print(f'linearity: quadratic term moves the ends by {1000 * abs(c2[2]) * (span / 2) ** 2:.2f} mm; residual rms {1000 * np.sqrt(np.mean(res2 ** 2)):.2f} mm')
if hyst is not None: print(f'hysteresis: down minus up at the same counts {hyst:+.2f} mm')
print(f'sideways drift of the travel per 100 mm of height (base x, y, z): {np.round(tilt_mm_per_100, 2).tolist()} mm; plate rotation up to {plate_rot_deg:.2f} deg')
print(f'travel first -> highest step: {1000 * top:.1f} mm by camera  vs  {1000 * abs(MPC_PROFILE) * (x.max() - x[0]):.1f} mm by the profile scale'
      f'  (CAD travel_m {1000 * prof["lift"]["travel_m"]:.1f} mm is bottom -> top stop)')
for w in warn: print('WARNING', w)
if args.out:
    json.dump(dict(marker_id=mid, side_mm=side_mm, calibration_id=prof['calibration_id'], steps=data,
                   meters_per_count=mpc, mm_per_turn=mpc * 4096 * 1000, residual_rms_mm=float(1000 * np.sqrt(np.mean(res1 ** 2))),
                   vs_reference_percent={k: float(100 * (mpc / v - 1)) for k, v in REF.items()},
                   quadratic_end_mm=float(1000 * abs(c2[2]) * (span / 2) ** 2), hysteresis_down_minus_up_mm=hyst,
                   drift_mm_per_100mm=tilt_mm_per_100, plate_rotation_deg=plate_rot_deg, travel_first_to_top_mm=1000 * top,
                   warnings=warn), open(args.out, 'w'), indent=1, default=float)
    print('wrote', args.out)
if args.plot:
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].plot(x, 1000 * (s - s[0]), 'o', label='camera'); ax[0].plot(x, 1000 * (A1 @ c1 - s[0]), '-', label=f'fit {mpc * 4096 * 1000:.2f} mm/turn')
    ax[0].plot(x, 1000 * abs(MPC_PROFILE) * (x - x[0]), '--', label='profile scale'); ax[0].set_xlabel('counts (up +)'); ax[0].set_ylabel('height (mm)'); ax[0].legend()
    ax[1].plot(x[up], 1000 * res1[up], 'o', label='up'); ax[1].plot(x[~up], 1000 * res1[~up], 's', label='down')
    ax[1].axhline(0, color='k', lw=.5); ax[1].set_xlabel('counts (up +)'); ax[1].set_ylabel('residual of the linear fit (mm)'); ax[1].legend()
    fig.tight_layout(); fig.savefig(args.plot, dpi=120); print('wrote', args.plot)
