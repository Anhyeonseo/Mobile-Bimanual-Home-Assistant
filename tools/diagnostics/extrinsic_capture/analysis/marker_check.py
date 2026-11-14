import sys, json, glob, os
import cv2, numpy as np
root, out = sys.argv[1], sys.argv[2]; os.makedirs(out, exist_ok=True)
SIDE = 0.06985
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
det = cv2.aruco.ArucoDetector(D, cv2.aruco.DetectorParameters())
J = ['base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll', 'gripper']
rows, tiles, centres = [], [], []
for f in sorted(glob.glob(root + '/' + (sys.argv[3] if len(sys.argv) > 3 else 'L1') + '-*/color.png')):
    name = f.split('/')[-2]; img = cv2.imread(f); g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    m = json.load(open(f.replace('color.png', 'metadata.json'))); k = m['color_intrinsics']
    K = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1]])
    jr = json.load(open(f.replace('color.png', 'joints.json')))['raw_after']
    corners, ids, _ = det.detectMarkers(g)
    vis = img.copy(); line = f'{name}'
    if ids is None:
        line += '  NOT FOUND'
    else:
        ids = ids.flatten().tolist(); i = ids.index(4) if 4 in ids else 0
        c = corners[i].reshape(4, 2)
        obj = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], np.float32) * SIDE / 2
        _, r, t = cv2.solvePnP(obj, c, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        R = cv2.Rodrigues(r)[0]; t = t.ravel()
        view = np.degrees(np.arccos(abs(R[:, 2] @ (-t / np.linalg.norm(t)))))
        side = np.linalg.norm(c - np.roll(c, -1, 0), axis=1).min()
        dimg = cv2.imread(f.replace('color', 'depth_aligned'), cv2.IMREAD_UNCHANGED).astype(float) * m['depth_scale_m']
        mask = np.zeros(g.shape, np.uint8); cv2.fillConvexPoly(mask, c.astype(np.int32), 1)
        dz = dimg[mask > 0]; dz = dz[dz > 0]
        cx, cy = c.mean(0); centres.append((cx, cy, t[2]))
        line += f'  id{ids}  centre ({cx:3.0f},{cy:3.0f})  side {side:3.0f}px  dist {t[2]*100:5.1f}cm  marker tilt {view:4.1f}deg  depth-PnP {1000*(np.median(dz)-t[2]) if len(dz) else float("nan"):+5.1f}mm'
        cv2.aruco.drawDetectedMarkers(vis, corners, np.asarray(ids).reshape(-1, 1)); cv2.drawFrameAxes(vis, K, None, r, t, 0.04)
        rows.append((name, view, [jr['left/' + j] for j in J], jr['lift/id10']))
    print(line)
    cv2.putText(vis, name, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    tiles.append(cv2.resize(vis, (256, 192)))
while len(tiles) % 5: tiles.append(np.zeros((192, 256, 3), np.uint8))
cv2.imwrite(out + '/contact.jpg', np.vstack([np.hstack(tiles[i:i+5]) for i in range(0, len(tiles), 5)]))
cm = np.zeros((480, 640, 3), np.uint8) + 40
for x, y, z in centres: cv2.circle(cm, (int(x), int(y)), 9, (0, 220, 0), -1)
cv2.rectangle(cm, (0, 0), (639, 479), (200, 200, 200), 2); cv2.imwrite(out + '/coverage.png', cm)
J6 = np.array([r[2] for r in rows]); print('joint raw ranges (deg):', {j: round(float(np.ptp(J6[:, i]) * 360 / 4096), 1) for i, j in enumerate(J)})
print('lift raw values:', sorted(set(r[3] for r in rows)))
print('marker tilt range:', round(min(r[1] for r in rows), 1), '-', round(max(r[1] for r in rows), 1))
c = np.array(centres); print('image x range', c[:, 0].min().round(), c[:, 0].max().round(), ' y range', c[:, 1].min().round(), c[:, 1].max().round(), ' dist cm', (c[:, 2].min()*100).round(1), (c[:, 2].max()*100).round(1))
