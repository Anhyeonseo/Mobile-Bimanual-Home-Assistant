"""Single-point depth vs surface plane vs ArUco PnP, for target points on a flat surface (block B).

Question: is a target point's raw depth (e.g. on black tape) biased against the surface it lies on?
For every target pixel in a camera shot this reports three estimates of the same 3-D point:
  raw    median aligned depth in a small window at the pixel (what a single-point target uses)
  plane  the pixel's viewing ray intersected with the surface plane, fitted by RANSAC on the depth
         in an annulus around the pixel (target areas and markers masked out)
  pnp    for the touch-target cross-hair only: markers id 1 / id 2 of tcp_touch_target_A4.pdf by PnP
         (independent of the depth sensor)
and their differences along the ray and in base height (profile base_from_optical, ground_z_m).

Targets: the cross-hair of tcp_touch_target_A4.pdf is found automatically ("cross"). Other targets
(black tape, a printed white cross, ...) are pixels from POINTS.json {"SHOT": {"label": [u, v]}}, or
clicked with --click LABEL[,LABEL...] (OpenCV window, one click per label in order; saved to POINTS.json).

usage: python3 depth_targets.py REPO SESSION_DIR [--shots 'DEPTH-*'] [--points points.json]
                                [--click tape,white] [--side-mm 66.2 --offset-mm 76.15] [--out out.json]
Shots are SESSION_DIR/<name>/cam (camshot) or SESSION_DIR/<name> (rawshot). Marker size and offset
default to SESSION_DIR/measurements.json "tcp_target", else the nominal 70 / 80 mm.
"""
import argparse, glob, json, os
import numpy as np, cv2

ap = argparse.ArgumentParser()
ap.add_argument('repo'); ap.add_argument('session')
ap.add_argument('--shots', default='DEPTH-*', help="glob of shot names inside SESSION_DIR")
ap.add_argument('--points', help='POINTS.json with pixel targets per shot (written by --click)')
ap.add_argument('--click', help='comma-separated labels to click per shot, e.g. tape,white')
ap.add_argument('--side-mm', type=float); ap.add_argument('--offset-mm', type=float)
ap.add_argument('--window', type=int, default=5, help='raw-depth median window (px, odd)')
ap.add_argument('--r-in', type=int, default=30, help='annulus inner radius (px) excluded around each target')
ap.add_argument('--r-out', type=int, default=120, help='annulus outer radius (px) for the plane fit')
ap.add_argument('--out')
args = ap.parse_args()

prof = json.load(open(os.path.join(args.repo, 'config/home_assistant/s2_coordinates.development.json')))
B = np.array(prof['base_from_optical'], float); GROUND = float(prof['ground_z_m'])
meas = os.path.join(args.session, 'measurements.json')
tgt = (json.load(open(meas)).get('tcp_target', {}) if os.path.exists(meas) else {})
side_mm = args.side_mm or float(tgt.get('marker_side_mm', 70.0))
offset_mm = args.offset_mm or float(tgt.get('offset_mm', 80.0 * side_mm / 70.0))
OBJ = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float) * side_mm / 2000
OFFSET = {1: np.array([offset_mm / 1000, 0, 0]), 2: np.array([-offset_mm / 1000, 0, 0])}
_PAR = cv2.aruco.DetectorParameters()
_PAR.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # default NONE gives integer corners: ~1 px = 2 % range at 45 px
DET = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), _PAR)
rng = np.random.default_rng(0)


def shot_folder(name):
    d = os.path.join(args.session, name)
    return os.path.join(d, 'cam') if os.path.exists(os.path.join(d, 'cam', 'color.png')) else d


def load(folder):
    m = json.load(open(os.path.join(folder, 'metadata.json'))); k = m['color_intrinsics']
    K = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1.]])
    z = cv2.imread(os.path.join(folder, 'depth_aligned.png'), cv2.IMREAD_UNCHANGED).astype(float) * m['depth_scale_m']
    exposure = None
    try: exposure = json.load(open(os.path.join(folder, 'record.json'))).get('capture_settings')
    except (FileNotFoundError, json.JSONDecodeError): pass
    return cv2.imread(os.path.join(folder, 'color.png')), z, K, exposure


def ray(K, uv):
    v = np.array([(uv[0] - K[0, 2]) / K[0, 0], (uv[1] - K[1, 2]) / K[1, 1], 1.0])
    return v / np.linalg.norm(v)


def backproject(K, uv, depth):
    return np.array([(uv[0] - K[0, 2]) / K[0, 0] * depth, (uv[1] - K[1, 2]) / K[1, 1] * depth, depth])


def cross_hair(gray, K):
    """Cross-hair in camera coordinates from every visible target marker (mean), its pixel and the marker quads."""
    c, ids, _ = DET.detectMarkers(gray)
    pts, quads = [], []
    for corners, i in zip(c if ids is not None else [], ids.ravel() if ids is not None else []):
        quads.append(corners.reshape(4, 2))
        if int(i) not in OFFSET: continue
        _, r, t = cv2.solvePnP(OBJ, corners.reshape(4, 2), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        pts.append(cv2.Rodrigues(r)[0] @ OFFSET[int(i)] + t.ravel())
    if not pts: return None, None, quads
    p = np.mean(pts, 0); uv = (K @ p)[:2] / p[2]
    return p, dict(uv=uv.tolist(), markers=len(pts), spread_mm=float(1000 * np.ptp(np.array(pts), 0).max()) if len(pts) > 1 else None), quads


def fit_plane(z, K, uv, exclude):
    h, w = z.shape; vv, uu = np.mgrid[0:h, 0:w]
    r = np.hypot(uu - uv[0], vv - uv[1])
    sel = (r >= args.r_in) & (r <= args.r_out) & (z > 0.15) & (z < 3.0) & ~exclude
    v, u = np.nonzero(sel)
    if len(u) < 200: return None
    Q = np.stack([(u - K[0, 2]) / K[0, 0] * z[v, u], (v - K[1, 2]) / K[1, 1] * z[v, u], z[v, u]], 1)
    best = None
    for _ in range(300):
        s = Q[rng.choice(len(Q), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
        if nn < 1e-9: continue
        n /= nn; inl = abs((Q - s[0]) @ n) < 0.004
        if best is None or inl.sum() > best.sum(): best = inl
    P = Q[best]; c = P.mean(0); n = np.linalg.svd(P - c)[2][2]
    return dict(n=n, c=c, inliers=int(best.sum()), used=len(Q), rms_mm=float(1000 * np.sqrt(np.mean(((P - c) @ n) ** 2))))


def base_height_cm(p):
    return float(100 * ((B[:3, :3] @ p + B[:3, 3])[2] - GROUND))


points = json.load(open(args.points)) if args.points and os.path.exists(args.points) else {}
names = sorted(os.path.basename(d) for d in glob.glob(os.path.join(args.session, args.shots)) if os.path.isdir(d))
if not names: raise SystemExit(f'no shots matching {args.shots} in {args.session}')
labels = args.click.split(',') if args.click else []
rows = []
for name in names:
    folder = shot_folder(name)
    img, z, K, exposure = load(folder)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    p_pnp, info, quads = cross_hair(gray, K)
    if labels:
        clicks = []
        win = f'{name}: click {",".join(labels)} in order (any key to skip)'
        cv2.namedWindow(win); cv2.setMouseCallback(win, lambda e, x, y, *_: clicks.append([x, y]) if e == cv2.EVENT_LBUTTONDOWN else None)
        while len(clicks) < len(labels):
            view = img.copy()
            for (x, y), lab in zip(clicks, labels): cv2.circle(view, (x, y), 4, (0, 0, 255), -1); cv2.putText(view, lab, (x + 6, y), 0, .5, (0, 0, 255), 1)
            cv2.imshow(win, view)
            if cv2.waitKey(30) != -1: break
        cv2.destroyWindow(win)
        if len(clicks) == len(labels): points[name] = dict(zip(labels, clicks))
    targets = dict(points.get(name, {}))
    if info: targets['cross'] = info['uv']
    exclude = np.zeros(z.shape, bool)
    for q in quads: cv2.fillConvexPoly(exclude.view(np.uint8), cv2.convexHull(q.astype(np.int32)), 1)
    exclude = cv2.dilate(exclude.astype(np.uint8), np.ones((17, 17), np.uint8)).astype(bool)
    h, w = z.shape; vv, uu = np.mgrid[0:h, 0:w]
    for uv in targets.values(): exclude |= np.hypot(uu - uv[0], vv - uv[1]) < args.r_in
    for label, uv in targets.items():
        u, v = int(round(uv[0])), int(round(uv[1])); k = args.window // 2
        win = z[max(v - k, 0):v + k + 1, max(u - k, 0):u + k + 1]; ok = win[win > 0]
        row = dict(shot=name, target=label, uv=[float(uv[0]), float(uv[1])], valid_fraction=float(ok.size / win.size),
                   capture_settings=exposure)
        pl = fit_plane(z, K, uv, exclude)
        d = ray(K, uv)
        if pl is not None:
            t = (pl['c'] @ pl['n']) / (d @ pl['n']); p_plane = t * d
            row.update(plane_rms_mm=pl['rms_mm'], plane_inliers=pl['inliers'], plane_h_cm=base_height_cm(p_plane))
        if ok.size:
            p_raw = backproject(K, uv, float(np.median(ok)))
            row.update(raw_h_cm=base_height_cm(p_raw))
            if pl is not None:
                row.update(raw_minus_plane_ray_mm=float(1000 * (np.linalg.norm(p_raw) - np.linalg.norm(p_plane))),
                           raw_minus_plane_h_mm=float(10 * (row['raw_h_cm'] - row['plane_h_cm'])))
        if label == 'cross' and p_pnp is not None:
            row.update(pnp_h_cm=base_height_cm(p_pnp), pnp_markers=info['markers'], pnp_spread_mm=info['spread_mm'])
            if pl is not None:
                row.update(pnp_minus_plane_ray_mm=float(1000 * (np.linalg.norm(p_pnp) - np.linalg.norm(p_plane))),
                           pnp_minus_plane_h_mm=float(10 * (row['pnp_h_cm'] - row['plane_h_cm'])))
        rows.append(row)

if args.click and args.points:
    json.dump(points, open(args.points, 'w'), indent=1)
    print('saved clicks to', args.points)

f = lambda x, n=1: '   -  ' if x is None else f'{x:6.{n}f}'
print(f'marker side {side_mm} mm, cross offset {offset_mm} mm; base from profile {prof["calibration_id"]}')
print(f'{"shot":14s} {"target":8s} {"valid":>5s} {"plane cm":>8s} {"raw-plane":>9s} {"(height)":>8s} {"pnp-plane":>9s} {"(height)":>8s} {"plane rms":>9s}')
for r in rows:
    print(f'{r["shot"]:14s} {r["target"]:8s} {r["valid_fraction"]:5.2f} {f(r.get("plane_h_cm"), 2):>8s} '
          f'{f(r.get("raw_minus_plane_ray_mm")):>9s} {f(r.get("raw_minus_plane_h_mm")):>8s} '
          f'{f(r.get("pnp_minus_plane_ray_mm")):>9s} {f(r.get("pnp_minus_plane_h_mm")):>8s} {f(r.get("plane_rms_mm")):>9s}')
print('mm; ray = along the viewing ray (+ = farther than the plane), height = base height (+ = above the plane)')
summary = {}
for r in rows:
    if 'raw_minus_plane_h_mm' in r: summary.setdefault(r['target'], []).append(r['raw_minus_plane_h_mm'])
for label, v in summary.items():
    v = np.array(v); print(f'{label:8s} raw-plane height: mean {v.mean():+.1f} mm  sd {v.std():.1f}  n={len(v)}')
if args.out:
    json.dump(dict(side_mm=side_mm, offset_mm=offset_mm, calibration_id=prof['calibration_id'], rows=rows,
                   summary={k: dict(mean_mm=float(np.mean(v)), sd_mm=float(np.std(v)), n=len(v)) for k, v in summary.items()}),
              open(args.out, 'w'), indent=1)
    print('wrote', args.out)
