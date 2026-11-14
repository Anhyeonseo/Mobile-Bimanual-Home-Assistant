import sys, json, glob
import cv2, numpy as np
rng = np.random.default_rng(0)
root = sys.argv[1]
T = np.array([[-0.011929717012670982, -0.707056462332393, 0.7070564623323932, 0.1007488922318314], [-0.9999288383940117, 0.008435583797296172, -0.008435583797296173, 0.033217110981069504], [0.0, -0.7071067811865476, -0.7071067811865475, 1.0070999249838757], [0, 0, 0, 1]])
GROUND = -0.0702

def cloud(folder):
    d = cv2.imread(folder + '/depth_aligned.png', cv2.IMREAD_UNCHANGED).astype(float)
    m = json.load(open(folder + '/metadata.json')); k = m['color_intrinsics']
    z = d * m['depth_scale_m']
    v, u = np.nonzero((z > 0.2) & (z < 2.5))
    z = z[v, u]
    P = np.stack([(u - k['ppx']) / k['fx'] * z, (v - k['ppy']) / k['fy'] * z, z], 1)
    return P, u, v

def ransac(P, iters=150, tol=0.006):
    best = None
    for _ in range(iters):
        s = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
        if nn < 1e-9: continue
        n /= nn; dist = abs((P - s[0]) @ n); inl = dist < tol
        if best is None or inl.sum() > best.sum(): best = inl
    Q = P[best]; c = Q.mean(0); n = np.linalg.svd(Q - c)[2][2]
    if n[1] > 0: n = -n            # normal points "up" = toward camera -y side
    return n, c, best

def describe(n, c):
    # plane: n.x = n.c ; distance from camera origin = |n.c|
    h = abs(n @ c)
    tilt = np.degrees(np.arccos(abs(n[2])))       # angle between optical axis and plane normal
    pitch = 90 - tilt                             # camera depression below the plane
    roll = np.degrees(np.arctan2(n[0], -n[1]))    # sideways lean of the plane in the image
    return h, pitch, roll

res = {}
for name in ['F-1', 'F-2', 'F-3', 'TB-1', 'TB-2', 'TB-3', 'TF-1', 'TF-2', 'TF-3']:
    P, u, v = cloud(glob.glob(f'{root}/{name}/cam')[0])
    P = P[rng.choice(len(P), min(len(P), 12000), replace=False)]
    planes = []
    rest = P
    for _ in range(3):
        if len(rest) < 1000: break
        n, c, inl = ransac(rest)
        if inl.sum() < 600: break
        planes.append((inl.sum(), n, c, rest[inl])); rest = rest[~inl]
    # horizontal planes in the nominal base frame
    out = []
    for cnt, n, c, Q in planes:
        nb = T[:3, :3] @ n
        if abs(nb[2]) < 0.97: continue
        zb = (T[:3, :3] @ c + T[:3, 3])[2]
        h, pitch, roll = describe(n, c)
        out.append((zb, cnt, h, pitch, roll))
    out.sort()
    res[name] = out
    print(name, ' | '.join(f'z_base(nominal)={zb:+.3f} m  pts={cnt}  cam->plane {h:.3f} m  pitch {p:.2f} deg  roll {r:+.2f} deg' for zb, cnt, h, p, r in out))
fl = [o[0] for k in ['F-1', 'F-2', 'F-3'] for o in res[k][:1]]
print('floor cam distance:', [round(o[2], 4) for k in ['F-1','F-2','F-3'] for o in res[k][:1]])
