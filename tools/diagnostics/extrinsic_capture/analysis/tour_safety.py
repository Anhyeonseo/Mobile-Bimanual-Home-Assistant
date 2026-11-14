"""Proven-free-space roadmap for an automatic calibration tour (PC planning only).

There is no full-robot collision model in this repository (the stationary planner reports
full_robot_collision_verified=False; the AlohaMini body meshes are not in the repo; and
arm_ranges.development.json is not a clearance certificate). Motion is therefore admitted only
inside space an arm has already traversed in a hand-guided, torque-off sweep (sweep_log.py):

- every logged configuration is a roadmap node candidate; nodes are kept every 2 deg of travel
- consecutive logged nodes are joined by edges: a person moved the arm along them
- two nodes closer than `link_deg` may also be joined, but only if every intermediate pose keeps
  every point of the arm skeleton within `tol` of the swept skeleton cloud, `arm_gap` away from
  the resting other arm and `floor_margin` above the floor
- planned paths follow roadmap edges; shortcuts are taken only when they pass the same check

The tour must run with the same lift height and with the reachable area as clear as in the sweep.
"""
import heapq, json
import numpy as np
from scipy.spatial import cKDTree

JOINTS6 = ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll', 'gripper')
MOVING = JOINTS6[:5]
SPACING = 0.01                  # skeleton densification (m)
PATH_STEP_DEG = 2.0
DEG = 360 / 4096                # degrees per raw count


def load_sweep(path):
    """Header + samples of one sweep.jsonl; raw values keyed 'left/base' ..."""
    rows = [json.loads(l) for l in open(path)]
    head, samples = rows[0], rows[1:]
    if head.get('kind') != 'header': raise ValueError('sweep log without header: ' + str(path))
    return head, samples


def skeleton(ctx, arm, raw, lift_m):
    """Base-frame points every 1 cm along pan axis -> shoulder -> elbow -> wrist -> roll -> tool."""
    ang = ctx.s2.model_angles({k: raw[k] for k in JOINTS6}, ctx.P['arms'][arm]['alignment'])
    tool, joints = ctx._chain(ang, {}, {})
    L = np.eye(4); L[:3, 3] = ctx.AXIS * lift_m
    M = L @ ctx.MOUNT[arm]
    nodes = [joints[k][0] for k in ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll')] + [tool[:3, 3]]
    nodes = [(M @ np.r_[n, 1])[:3] for n in nodes]
    pts = []
    for a, b in zip(nodes, nodes[1:]):
        k = max(1, int(np.ceil(np.linalg.norm(b - a) / SPACING)))
        pts.append(a + (b - a) * np.linspace(0, 1, k + 1)[:, None])
    return np.vstack(pts)


def vec(raw): return np.array([raw[k] for k in MOVING], float)


class Roadmap:
    """Free space and roadmap of one arm at one lift height, built from that arm's sweep logs."""

    def __init__(self, ctx, arm, lift_m, sweeps, other_rest, tol=0.015, arm_gap=0.06, floor_margin=0.05,
                 node_deg=2.0, link_deg=8.0, admissible=None):
        """admissible(raw) -> bool: configurations failing it (e.g. too close to a joint range limit)
        still count as swept space but never become nodes, and they break the traversed chain."""
        self.ctx, self.arm, self.lift, self.other_rest = ctx, arm, lift_m, other_rest
        self.other = 'right' if arm == 'left' else 'left'
        self.tol, self.arm_gap, self.floor_margin = tol, arm_gap, floor_margin
        self.fixed = None            # gripper raw, constant during the tour
        self.nodes, edges, cloud = [], set(), []
        for samples in sweeps:
            prev = None
            last_cloud = None
            for s in samples:
                raw = {k: s['raw'][f'{arm}/{k}'] for k in JOINTS6}
                if admissible is not None and not admissible(raw):
                    if last_cloud is None or np.abs(vec(raw) - last_cloud).max() * DEG >= node_deg:
                        cloud.append(skeleton(ctx, arm, raw, lift_m)); last_cloud = vec(raw)
                    prev = None
                    continue
                if prev is not None and np.abs(vec(raw) - vec(self.nodes[prev])).max() * DEG < node_deg:
                    continue
                cloud.append(skeleton(ctx, arm, raw, lift_m)); last_cloud = vec(raw)
                self.nodes.append(raw)
                if prev is not None: edges.add((prev, len(self.nodes) - 1))
                prev = len(self.nodes) - 1
        if not self.nodes: raise ValueError(f'no sweep samples for {arm} at {lift_m}')
        self.tree = cKDTree(np.vstack(cloud))
        self.other_tree = cKDTree(skeleton(ctx, self.other, other_rest, lift_m))
        self.traversed = len(edges)
        X = np.array([vec(n) for n in self.nodes]) * DEG
        d, nb = cKDTree(X).query(X, k=7, distance_upper_bound=link_deg, p=np.inf)   # up to 6 neighbours each
        for i in range(len(X)):
            for j in nb[i][1:]:
                if j >= len(X) or (i, j) in edges or (j, i) in edges: continue
                if self.path_check(self.nodes[i], self.nodes[j])[0]: edges.add((min(i, j), max(i, j)))
        self.adj = {i: [] for i in range(len(self.nodes))}
        for i, j in edges:
            w = float(np.abs(X[i] - X[j]).max())
            self.adj[i].append((j, w)); self.adj[j].append((i, w))
        self.X = X

    def pose_check(self, raw):
        pts = skeleton(self.ctx, self.arm, raw, self.lift)
        d = self.tree.query(pts)[0].max()
        if d > self.tol: return False, 'OUTSIDE_SWEPT_SPACE', float(d)
        if pts[:, 2].min() < self.ctx.GROUND + self.floor_margin: return False, 'NEAR_FLOOR', float(d)
        if self.other_tree.query(pts)[0].min() < self.arm_gap: return False, 'NEAR_OTHER_ARM', float(d)
        return True, '', float(d)

    def path_check(self, a, b):
        va, vb = vec(a), vec(b)
        n = max(1, int(np.ceil(np.abs(vb - va).max() * DEG / PATH_STEP_DEG)))
        for s in np.linspace(0, 1, n + 1):
            r = dict(a); r.update({k: float(v) for k, v in zip(MOVING, va + (vb - va) * s)})
            ok, why, _ = self.pose_check(r)
            if not ok: return False, why
        return True, ''

    def nearest(self, raw):
        d = np.abs(self.X - vec(raw) * DEG).max(axis=1)
        i = int(np.argmin(d)); return i, float(d[i])

    def shortest(self, i, j):
        """Dijkstra over roadmap edges (weight = largest joint change, deg). Returns node list or None."""
        dist, prev, pq = {i: 0.0}, {}, [(0.0, i)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == j: break
            if d > dist[u]: continue
            for v, w in self.adj[u]:
                if d + w < dist.get(v, 1e18):
                    dist[v] = d + w; prev[v] = u; heapq.heappush(pq, (d + w, v))
        if j not in dist: return None, None
        path = [j]
        while path[-1] != i: path.append(prev[path[-1]])
        return path[::-1], dist[j]

    def shortcut(self, path):
        """Drop intermediate nodes where a direct checked segment exists (fewer corners)."""
        out, k = [path[0]], 0
        while k < len(path) - 1:
            far = k + 1
            for m in range(min(len(path) - 1, k + 20), k + 1, -1):
                if self.path_check(self.nodes[path[k]], self.nodes[path[m]])[0]:
                    far = m; break
            out.append(path[far]); k = far
        return out
