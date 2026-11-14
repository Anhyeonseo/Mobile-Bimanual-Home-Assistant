import sys, json, glob
import cv2, numpy as np
SQ = 0.025
for f in sorted(glob.glob(sys.argv[1] + '/CB-*/cam/color.png')):
    name = f.split('/')[-3]
    img = cv2.imread(f, 0); d = cv2.imread(f.replace('color', 'depth_aligned'), cv2.IMREAD_UNCHANGED).astype(float)
    m = json.load(open(f.replace('color.png', 'metadata.json')))
    k = m['color_intrinsics']; d *= m['depth_scale_m']
    K = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1]]); dist = np.array(k.get('coeffs', [0]*5), float)
    ok, c = cv2.findChessboardCornersSB(img, (8, 5), flags=cv2.CALIB_CB_EXHAUSTIVE)
    obj = np.array([[x*SQ, y*SQ, 0] for y in range(5) for x in range(8)], np.float32)
    _, r, t = cv2.solvePnP(obj, c, K, dist)
    R = cv2.Rodrigues(r)[0]
    P = (R @ obj.T).T + t.ravel()                     # corners in camera frame from the printed size
    u, v = c.reshape(-1, 2).round().astype(int).T
    z_depth = np.array([np.median(d[max(0,y-2):y+3, max(0,x-2):x+3][d[max(0,y-2):y+3, max(0,x-2):x+3] > 0]) for x, y in zip(u, v)])
    diff = z_depth - P[:, 2]
    # depth-derived square size: back-project corners with depth, measure neighbour spacing
    X = np.stack([(u - k['ppx']) / k['fx'] * z_depth, (v - k['ppy']) / k['fy'] * z_depth, z_depth], 1).reshape(5, 8, 3)
    sq_depth = np.median(np.linalg.norm(np.diff(X, axis=1), axis=2))
    tilt = np.degrees(np.arccos(abs(R[:, 2] @ np.array([0, 0, 1]))))
    print(f'{name}  Z(checker)={P[:,2].mean():.3f} m  depth-checker = {1000*np.median(diff):+5.1f} mm (spread {1000*np.std(diff):4.1f})  square from depth {1000*sq_depth:5.2f} mm  view tilt {tilt:4.1f} deg')
print('intrinsics', {k2: round(v2, 2) if isinstance(v2, float) else v2 for k2, v2 in k.items()})
