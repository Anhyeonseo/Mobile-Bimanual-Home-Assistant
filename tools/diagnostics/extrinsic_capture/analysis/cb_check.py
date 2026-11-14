import sys, json, glob, os
import cv2, numpy as np
root = sys.argv[1]; out = sys.argv[2]
os.makedirs(out, exist_ok=True)
tiles = []
for f in sorted(glob.glob(root + '/CB-*/**/color.png', recursive=True)):
    name = f.split('/')[-3] if f.endswith('cam/color.png') else f.split('/')[-2]
    img = cv2.imread(f); gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found = None
    for pattern in [(8, 5), (5, 8), (9, 6), (6, 9), (7, 4)]:
        ok, c = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE)
        if ok:
            found = (pattern, c); break
    d = cv2.imread(f.replace('color.png', 'depth_aligned.png'), cv2.IMREAD_UNCHANGED)
    meta = json.load(open(f.replace('color.png', 'metadata.json')))
    scale = meta.get('depth_scale', meta.get('depth_scale_m', 0.001))
    line = f'{name:6s}'
    vis = img.copy()
    if found:
        pattern, c = found
        cv2.drawChessboardCorners(vis, pattern, c, True)
        pts = c.reshape(-1, 2)
        x0, y0 = pts.min(0); x1, y1 = pts.max(0)
        # board region depth
        hull = cv2.convexHull(pts.astype(np.float32)).astype(np.int32)
        mask = np.zeros(gray.shape, np.uint8); cv2.fillConvexPoly(mask, hull, 1)
        dz = d[mask > 0].astype(float) * scale
        valid = dz[dz > 0]
        # square size in pixels
        grid = pts.reshape(pattern[1], pattern[0], 2)
        sq = np.median(np.linalg.norm(np.diff(grid, axis=1), axis=2))
        margin = min(x0, y0, img.shape[1] - x1, img.shape[0] - y1)
        line += f' FOUND {pattern[0]}x{pattern[1]} corners | centre x={pts[:,0].mean():.0f} y={pts[:,1].mean():.0f} | square {sq:.0f}px | edge margin {margin:.0f}px | depth valid {100*len(valid)/max(1,len(dz)):.0f}% median {np.median(valid) if len(valid) else 0:.3f} m'
    else:
        line += ' NOT FOUND'
    print(line)
    cv2.putText(vis, name + (' OK' if found else ' NOT FOUND'), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    tiles.append(cv2.resize(vis, (320, 240)))
while len(tiles) % 3: tiles.append(np.zeros((240, 320, 3), np.uint8))
rows = [np.hstack(tiles[i:i+3]) for i in range(0, len(tiles), 3)]
cv2.imwrite(out + '/contact.jpg', np.vstack(rows))
print('meta keys:', list(meta.keys())[:20])
