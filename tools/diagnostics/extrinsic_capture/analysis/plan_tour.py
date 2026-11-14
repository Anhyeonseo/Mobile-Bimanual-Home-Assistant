"""Plan an automatic arm-marker calibration tour from hand-guided sweeps (PC, offline; no device access).

Inputs: sweep folders written by ../sweep_log.py (one arm, one lift height each, torque off).

1. Rest pose of each arm = where it sat while the other arm was swept (median).
2. A roadmap per (arm, lift) from that arm's sweeps (tour_safety.Roadmap): only swept space.
3. Candidate targets = roadmap nodes where the marker is well visible and that connect to rest.
   Targets are chosen A-optimally for the joint calibration (design_poses.Designer, model C).
4. A few targets are visited three times from different neighbours (repeatability / backlash).
5. Per lift and arm: rest -> targets -> rest along roadmap shortest paths (shortcut when checked).
6. Each move becomes a 12-joint finite route (minimum-jerk along the path, capped joint speed, the
   other arm and both grippers at rest) and is validated with the firmware adapter's own
   finite-plan checks (approved ranges, per-tick step, single-turn encoder route).

The Jetson runner (../calib_tour.py) executes the result one lift height at a time.

usage: python3 plan_tour.py REPO DATA JOINT_CALIB.json OUT_TOUR.json --sweeps DIR [DIR ...]
       DATA = the 2026-11-13 capture (information already held); JOINT_CALIB = joint_calib.py output
"""
import sys, os, json, glob, hashlib, argparse, datetime
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from calib_model import Context, OP, ARMS
from design_poses import Designer
from tour_safety import Roadmap, load_sweep, JOINTS6, MOVING, DEG, vec

ap = argparse.ArgumentParser()
ap.add_argument('repo'); ap.add_argument('data'); ap.add_argument('calib'); ap.add_argument('out')
ap.add_argument('--sweeps', nargs='+', required=True)
ap.add_argument('--n', type=int, default=24, help='designed poses per arm (all lift heights)')
ap.add_argument('--repeats', type=int, default=2, help='targets per arm and lift visited 3 times')
ap.add_argument('--cand', type=int, default=400, help='candidate nodes per arm and lift (random subset)')
ap.add_argument('--speed', type=float, default=20.0, help='peak joint speed, deg/s')
ap.add_argument('--tol', type=float, default=0.015, help='allowed distance outside swept space, m')
ap.add_argument('--seed', type=int, default=11)
args = ap.parse_args()
if os.path.exists(args.out): sys.exit('output exists: ' + args.out)

sys.path[:0] = [args.repo + '/ros2_ws/src/so101_arm_bridge']
try:
    import serial  # noqa: F401  (the adapter module imports it; planning never opens a port)
except ModuleNotFoundError:
    import types
    sys.modules['serial'] = types.SimpleNamespace(SerialTimeoutException=type('SerialTimeoutException', (Exception,), {}))
from so101_arm_bridge.stationary_pick_adapter import StationaryPickAdapter
from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint, CANONICAL_JOINT_NAMES

RANGES_PATH = args.repo + '/config/home_assistant/arm_ranges.development.json'
RANGES = json.load(open(RANGES_PATH))['arms']
DIR = {a: {j: RANGES[a][j]['positive_raw_direction'] for j in JOINTS6} for a in ARMS}
LEVEL_OF = {round(v, 2): k for k, v in OP.items() if k != 'RCD'}           # 0.20 L1, 0.35 L2, 0.50 H50
POINT_MS = 40
rng = np.random.default_rng(args.seed)
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()

# ---------- sweeps ----------
sweeps = {}                      # (arm, lift) -> [samples, ...]
sources = []
for d in args.sweeps:
    head, samples = load_sweep(os.path.join(d, 'sweep.jsonl'))
    lift = round(float(head['lift_m_operator']), 2)
    if lift not in LEVEL_OF: sys.exit(f'{d}: lift {lift} m is not one of {sorted(LEVEL_OF)}')
    sweeps.setdefault((head['arm'], lift), []).append(samples)
    sources.append(dict(folder=os.path.abspath(d), arm=head['arm'], lift_m=lift, samples=len(samples),
                        sha256=sha(os.path.join(d, 'sweep.jsonl'))))
print('sweeps:', {f'{a}@{l}': sum(len(s) for s in v) for (a, l), v in sweeps.items()})


def inside_ranges(arm, raw, margin=25):
    return all(RANGES[arm][j]['minimum_unwrapped_raw'] + margin <= raw[j] <= RANGES[arm][j]['maximum_unwrapped_raw'] - margin
               for j in JOINTS6)


REST = {}
for arm in ARMS:
    other = 'right' if arm == 'left' else 'left'
    resting = [s['raw'] for (a, _), v in sweeps.items() if a == other for samples in v for s in samples]
    if not resting: resting = [v[0][0]['raw'] for (a, _), v in sweeps.items() if a == arm]
    if not resting: sys.exit(f'no sweep shows where the {arm} arm rests')
    REST[arm] = {j: int(round(np.median([r[f'{arm}/{j}'] for r in resting]))) for j in JOINTS6}
    if not inside_ranges(arm, REST[arm]): sys.exit(f'{arm} rest pose is within 25 counts of a range limit: {REST[arm]}')
print('rest poses:', REST)

ctx = Context(args.repo, args.data)
designer = Designer(ctx, json.load(open(args.calib)))
maps = {}
for (arm, lift), v in sweeps.items():
    other = 'right' if arm == 'left' else 'left'
    rm = Roadmap(ctx, arm, lift, v, REST[other], tol=args.tol,
                 admissible=lambda raw, arm=arm: inside_ranges(arm, raw, margin=15))
    i, d = rm.nearest(REST[arm])
    if d > 3.0: sys.exit(f'{arm}@{lift}: the sweep never passes within 3 deg of the rest pose ({d:.1f} deg)')
    rm.rest_node = i
    maps[(arm, lift)] = rm
    print(f'{arm}@{lift}: {len(rm.nodes)} nodes, {rm.traversed} traversed + '
          f'{sum(len(a) for a in rm.adj.values()) // 2 - rm.traversed} checked shortcut edges', flush=True)

# ---------- candidates + A-optimal choice ----------
M0 = designer.current_info(); M = M0.copy(); Mr = M0.copy(); chosen = {a: [] for a in ARMS}
for arm in ARMS:
    cands = []
    for (a, lift), rm in maps.items():
        if a != arm: continue
        lvl = LEVEL_OF[lift]
        idx = [i for i in rng.permutation(len(rm.nodes))
               if designer.usable(arm, {**rm.nodes[i], 'gripper': REST[arm]['gripper']}, lvl)][:args.cand]
        reach = [i for i in idx if rm.shortest(rm.rest_node, i)[0] is not None]
        cands += [(lift, i) for i in reach]
        print(f'{arm}@{lift}: {len(idx)} visible candidates, {len(reach)} reachable from rest', flush=True)
    if not cands: continue
    Js = [designer.rows(arm, {**maps[(arm, l)].nodes[i], 'gripper': REST[arm]['gripper']}, LEVEL_OF[l]) for l, i in cands]
    pick, M = designer.select(Js, M, args.n)
    for c in rng.choice(len(Js), min(args.n, len(Js)), replace=False): Mr = Mr + Js[c].T @ Js[c]
    chosen[arm] = [cands[c] for c in pick]
std = {k: designer.target_std(np.linalg.inv(m)) for k, m in (('now', M0), ('random_swept', Mr), ('designed_swept', M))}
print(f'\n{"parameter":22s} {"now":>8s} {"+random":>8s} {"+designed":>9s}  (swept poses only)')
for k, t in enumerate(designer.TARGET):
    print(f'{designer.lab[t]:22s} {std["now"][k]:8.2f} {std["random_swept"][k]:8.2f} {std["designed_swept"][k]:9.2f}')


# ---------- order, repeats, routes ----------
def order_targets(rm, nodes, tag):
    named = [(f'{tag}-{k + 1:02d}', i) for k, i in enumerate(nodes)]
    order, cur, left = [], rm.rest_node, list(named)
    while left:
        costs = [rm.shortest(cur, i)[1] for _, i in left]
        k = int(np.argmin(costs)); order.append(left.pop(k)); cur = order[-1][1]

    def next_to(pos, name):
        before = order[pos - 1][0] if pos > 0 else ''
        after = order[pos][0] if pos < len(order) else ''
        return before.startswith(name) or after.startswith(name)
    for name, i in list(order[:args.repeats]):
        for rep in (1, 2):
            spots = [p for p in range(1, len(order) + 1) if not next_to(p, name)]
            if spots: order.insert(int(rng.choice(spots)), (f'{name}-r{rep}', i))
    return order


def wire(arm_moving, raw_moving):
    q = []
    for arm in ARMS:
        for j in JOINTS6:
            r = REST[arm][j] if (arm != arm_moving or j == 'gripper') else raw_moving[j]
            q.append(DIR[arm][j] * (r - 2048) * 2 * np.pi / 4096)
    return np.array(q)


def route_points(arm, raws):
    """Minimum-jerk timing along the joint-space polyline through raws; 40 ms samples."""
    Q = np.array([wire(arm, r) for r in raws])
    seg = np.abs(np.diff(Q, axis=0)).max(axis=1)
    keep = np.r_[True, seg > 1e-9]; Q = Q[keep]; seg = seg[seg > 1e-9]
    if len(Q) == 1: Q = np.vstack([Q, Q]); seg = np.array([0.0])
    S = np.r_[0, np.cumsum(seg)]; total = S[-1]
    T = max(1.5, 1.875 * total / np.radians(args.speed))
    n = int(np.ceil(T * 1000 / POINT_MS)); pts = []
    for i in range(n + 1):
        u = i / n; s = total * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)
        k = min(int(np.searchsorted(S, s, side='right')) - 1, len(seg) - 1)
        f = 0.0 if seg[k] == 0 else (s - S[k]) / seg[k]
        pts.append((POINT_MS + i * POINT_MS, (Q[k] + (Q[k + 1] - Q[k]) * min(max(f, 0.0), 1.0)).tolist()))
    return pts


checker = StationaryPickAdapter(object(), __import__("pathlib").Path(RANGES_PATH))
levels_out = []
for lift in sorted({l for _, l in maps}):
    steps = []
    for arm in ARMS:
        if (arm, lift) not in maps: continue
        rm = maps[(arm, lift)]
        targets = [i for l, i in chosen[arm] if l == lift]
        if not targets: continue
        order = [('REST', rm.rest_node)] + order_targets(rm, targets, f'T{round(lift * 100):02d}-{arm[0].upper()}') + [('REST', rm.rest_node)]
        for (na, a), (nb, b) in zip(order, order[1:]):
            path, cost = rm.shortest(a, b)
            path = rm.shortcut(path)
            raws = ([REST[arm]] if na == 'REST' else []) + [rm.nodes[i] for i in path] + ([REST[arm]] if nb == 'REST' else [])
            pts = route_points(arm, raws)
            checker._validate_finite_plan_points(tuple(TimedJointPoint(t, tuple(q)) for t, q in pts))
            steps.append(dict(kind='move', arm=arm, from_name=na, to_name=nb, waypoints=len(raws),
                              duration_s=pts[-1][0] / 1000, points=[[t, [round(x, 7) for x in q]] for t, q in pts]))
            if nb != 'REST':
                # a checked way home from this target, used when the operator ends the tour early
                back, _ = rm.shortest(b, rm.rest_node)
                back_pts = route_points(arm, [rm.nodes[i] for i in rm.shortcut(back)] + [REST[arm]])
                checker._validate_finite_plan_points(tuple(TimedJointPoint(t, tuple(q)) for t, q in back_pts))
                steps.append(dict(kind='shot', arm=arm, name=nb, base_name=nb.split('-r')[0], approach_from=na,
                                  target_raw={**{j: int(round(rm.nodes[b][j])) for j in MOVING}, 'gripper': REST[arm]['gripper']},
                                  return_points=[[t, [round(x, 7) for x in q]] for t, q in back_pts]))
    moves = [s for s in steps if s['kind'] == 'move']
    if moves and not np.allclose(moves[0]['points'][0][1], wire('left', REST['left']), atol=1e-6):
        sys.exit('tour level does not start at rest')
    for m1, m2 in zip(moves, moves[1:]):
        if not np.allclose(m1['points'][-1][1], m2['points'][0][1], atol=1e-6):
            sys.exit(f'discontinuous route at {m1["to_name"]} -> {m2["from_name"]}')
    shots = sum(s['kind'] == 'shot' for s in steps); dur = sum(s['duration_s'] for s in moves)
    levels_out.append(dict(lift_m=lift, level=LEVEL_OF[lift], shots=shots, motion_s=round(dur, 1), steps=steps))
    print(f'lift {lift:.2f} m: {shots} shots, {len(moves)} moves, {dur / 60:.1f} min of motion')

tour = dict(kind='extrinsic_calibration_tour', schema_version=1,
            created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
            arm_ranges_sha256=sha(RANGES_PATH),
            profile_sha256=sha(args.repo + '/config/home_assistant/s2_coordinates.development.json'),
            joint_names=list(CANONICAL_JOINT_NAMES), sweeps=sources, rest_raw=REST,
            rest_wire_rad=wire('left', REST['left']).tolist(), point_interval_ms=POINT_MS,
            safety=dict(method='hand-guided swept-space roadmap', tolerance_m=args.tol,
                        arm_gap_m=next(iter(maps.values())).arm_gap, floor_margin_m=next(iter(maps.values())).floor_margin,
                        peak_joint_speed_deg_s=args.speed, other_arm='held at rest', grippers='held at rest',
                        full_robot_collision_verified=False, area_must_match_sweep=True),
            design=dict(model='C zeros + gravity sag', per_arm=args.n, repeats_per_arm_lift=args.repeats,
                        target_std={k: dict(zip([designer.lab[t] for t in designer.TARGET], map(float, v))) for k, v in std.items()}),
            levels=levels_out)
with open(args.out, 'x') as f: json.dump(tour, f)
print('wrote', args.out, 'sha256', sha(args.out))
