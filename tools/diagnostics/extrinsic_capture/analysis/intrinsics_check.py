"""Check the D415 colour intrinsics (recorded with distortion 0) against checkerboard photos.

Compares, on photos held out one at a time (leave-one-out):
  A  factory fx, fy, cx, cy and zero distortion (what every tool uses today)
  B  factory fx, fy, cx, cy + estimated distortion k1, k2, p1, p2, k3
  C  everything estimated
and reports how far B / C move points at the image corners versus A, in pixels and in mm at 0.5 m.
Distortion only shows up near the image border: photos must put the board into all four corners.

Note: the RealSense SDK aligns depth to colour with its own (factory) colour model. A corrected
colour model changes where ArUco/checkerboard corners are measured; the depth image is unchanged.

usage: python3 intrinsics_check.py DIR [DIR ...] --square-mm 25.0 [--out OUT.json]
       photos: <DIR>/CB-*/cam/color.png, <DIR>/IN-*/cam/color.png (camshot names)
"""
import argparse, glob, json, os
import numpy as np, cv2

ap = argparse.ArgumentParser()
ap.add_argument('dirs', nargs='+'); ap.add_argument('--square-mm', type=float, required=True, help='measured square size')
ap.add_argument('--out')
args = ap.parse_args()

files = sorted(f for d in args.dirs for pat in ('CB-*', 'IN-*') for f in glob.glob(f'{d}/{pat}/**/color.png', recursive=True))
pattern, views = None, []
for f in files:
    g = cv2.imread(f, 0)
    for pat in ([pattern] if pattern else [(8, 5), (5, 8), (9, 6), (6, 9), (7, 4)]):
        ok, c = cv2.findChessboardCornersSB(g, pat, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            pattern = pat; views.append((f, c.reshape(-1, 1, 2).astype(np.float32))); break
if len(views) < 6: raise SystemExit(f'need >= 6 checkerboard photos, found {len(views)} of {len(files)}')
meta = json.load(open(views[0][0].replace('color.png', 'metadata.json')))['color_intrinsics']
size = (meta['width'], meta['height'])
K0 = np.array([[meta['fx'], 0, meta['ppx']], [0, meta['fy'], meta['ppy']], [0, 0, 1]])
grid = np.zeros((pattern[0] * pattern[1], 3), np.float32)
grid[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * args.square_mm / 1000
obj = [grid] * len(views); img = [c for _, c in views]

cells = np.zeros((6, 8), bool)
for c in img:
    for u, v in c.reshape(-1, 2): cells[min(int(v / size[1] * 6), 5), min(int(u / size[0] * 8), 7)] = True
corner_cells = [cells[0, 0], cells[0, -1], cells[-1, 0], cells[-1, -1]]
print(f'{len(views)} boards ({pattern[0]}x{pattern[1]} inner corners), image coverage {cells.mean() * 100:.0f} % of 8x6 cells, '
      f'corner cells covered {sum(corner_cells)}/4')

FLAGS = {'B': cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_FOCAL_LENGTH | cv2.CALIB_FIX_PRINCIPAL_POINT,
         'C': cv2.CALIB_USE_INTRINSIC_GUESS}


def fit(kind, idx):
    if kind == 'A': return K0.copy(), np.zeros(5)
    _, K, dist, _, _ = cv2.calibrateCamera([obj[i] for i in idx], [img[i] for i in idx], size, K0.copy(), np.zeros(5), flags=FLAGS[kind])
    return K, dist.ravel()


def reproj(K, dist, i):
    ok, r, t = cv2.solvePnP(obj[i], img[i], K, dist)
    proj, _ = cv2.projectPoints(obj[i], r, t, K, dist)
    return np.sqrt(np.mean(np.sum((proj - img[i]) ** 2, axis=2)))


res = {}
for kind in 'ABC':
    loo = [reproj(*fit(kind, [j for j in range(len(views)) if j != i]), i) for i in range(len(views))]
    K, dist = fit(kind, range(len(views)))
    res[kind] = dict(loo_rms_px=float(np.sqrt(np.mean(np.square(loo)))), K=K.tolist(), dist=dist.tolist())
corners = np.array([[0, 0], [size[0] - 1, 0], [0, size[1] - 1], [size[0] - 1, size[1] - 1]], np.float32).reshape(-1, 1, 2)
for kind in 'BC':
    K, dist = np.array(res[kind]['K']), np.array(res[kind]['dist'])
    und = cv2.undistortPoints(corners, K, dist, P=K0).reshape(-1, 2)            # where A would put the same rays
    shift = np.linalg.norm(und - corners.reshape(-1, 2), axis=1).max()
    res[kind].update(max_corner_shift_px=float(shift), mm_at_0p5m=float(shift / K0[0, 0] * 500))
for kind, name in zip('ABC', ('factory, no distortion', 'factory + distortion', 'all estimated')):
    r = res[kind]
    extra = f'  corner shift {r["max_corner_shift_px"]:.1f} px = {r["mm_at_0p5m"]:.1f} mm at 0.5 m' if kind != 'A' else ''
    print(f'{kind} {name:24s} held-out reprojection RMS {r["loo_rms_px"]:.3f} px{extra}')
print('distortion (B):', np.round(res['B']['dist'], 4).tolist())
gain = 1 - res['B']['loo_rms_px'] / res['A']['loo_rms_px']
verdict = ('use B: distortion is real and measurable' if gain > 0.2 and res['B']['max_corner_shift_px'] > 0.5 and sum(corner_cells) >= 3
           else 'keep A: no measurable improvement' if sum(corner_cells) >= 3 else 'inconclusive: put the board into the image corners')
print('verdict:', verdict)
if args.out:
    json.dump(dict(pattern=pattern, square_mm=args.square_mm, photos=[f for f, _ in views], coverage=float(cells.mean()),
                   corner_cells=int(sum(corner_cells)), factory=dict(K=K0.tolist()), results=res, verdict=verdict), open(args.out, 'w'), indent=1)
