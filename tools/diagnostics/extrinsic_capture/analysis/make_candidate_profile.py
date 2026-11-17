"""Write candidate profile files from one model of calib_encoder_lift.py. Writes NEW files only.

The runtime places an arm at  base_from_arm_at_lift_zero (translated by axis_base * h) @ FK(home_model_rad + ...)
with h = (lift_raw - origin) * meters_per_count, and maps camera points with base_from_optical. A model
without gravity sag maps onto those fields exactly:

  base_from_optical               <- fitted camera
  arms.*.alignment.*.home_model_rad  += fitted joint zero   (reference_model_rad moves with it: the profile
                                       validator requires model_angles(reference_raw) == reference_model_rad)
  arms.right.base_from_arm_at_lift_zero <- old @ fitted right-mount correction
  arms.*.base_from_arm_at_lift_zero translation += axis_base * loff   (lift zero offset)
  lift.meters_per_count           *= (1 + lscale)
  camera_height_adjustment        removed (the fitted camera already holds the height; validator checks it)

calibration_id / wide base_profile_id / height model_id are recomputed with the same digests the loaders use,
and the base candidate is checked with s2_development.validate().

usage: python3 make_candidate_profile.py REPO CAPTURE_DIR TOUR_DIRS RESULT.json "MODEL LABEL" OUT_DIR [--bottom-raw 3900]
       CAPTURE_DIR/TOUR_DIRS must be the folders RESULT.json was computed from (parameter layout depends on them).
"""
import sys, os, json, copy, argparse, hashlib
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from calib_model import Context, ARMS, ZJ, CJ, camera_summary
from encoder_lift_model import EncModel, add_encoder_heights

ap = argparse.ArgumentParser()
ap.add_argument('repo'); ap.add_argument('capture'); ap.add_argument('tours'); ap.add_argument('result'); ap.add_argument('label')
ap.add_argument('out'); ap.add_argument('--bottom-raw', type=int, default=3900); ap.add_argument('--tag', default='candidate-20261117')
args = ap.parse_args()

R = json.load(open(args.result))[args.label]
ctx = Context(args.repo, [args.capture] + args.tours.split(',')); add_encoder_heights(ctx, args.bottom_raw)
free = {n for n in R['params']} | {'cam', 'rm'} | {'gm_' + a for a in ctx.atts}
free = {n.split('.')[0] for n in free}
m = EncModel(ctx, free); p = np.array(R['p'])
if len(p) != m.size: sys.exit('parameter layout mismatch: run with the same folders as the result')
d = m.unpack(p)
sag = {k: v for k, v in d.items() if k.startswith('c_') and abs(v) > 0}
if sag: print(f'WARNING: {args.label} has gravity-sag terms the runtime cannot apply: {sorted(sag)}. Prefer a model without sag.')
if any(abs(d.get(k, 0.0)) > 0 for k in ('lt_x', 'lt_y')): print('WARNING: lift tilt is not applied (no profile field).')

cfg = os.path.join(args.repo, 'config/home_assistant')
base = json.load(open(os.path.join(cfg, 's2_coordinates.development.json')))
wide = json.load(open(os.path.join(cfg, 's2_coordinates.wide.development.json')))
height = json.load(open(os.path.join(cfg, 'camera_height.calibrated.json')))
digest = lambda v: hashlib.sha256(json.dumps(v, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def apply(profile):
    q = copy.deepcopy(profile)
    q['base_from_optical'] = d['cam'].tolist()
    q.pop('camera_height_adjustment', None)
    q['provenance'] = 'measured_joint_camera_arm_lift_calibration_' + args.tag
    axis = np.array(q['lift']['axis_base'])
    for arm in ARMS:
        a = q['arms'][arm]
        M = np.array(a['base_from_arm_at_lift_zero'])
        if arm == 'right': M = M @ d['rm']
        M[:3, 3] += axis * d['loff']
        a['base_from_arm_at_lift_zero'] = M.tolist()
        for j in ZJ:
            z = float(d[f'z_{arm}_{j}'])
            a['alignment'][j]['home_model_rad'] = a['alignment'][j]['home_model_rad'] + z
            a['reference_model_rad'][j] = a['reference_model_rad'][j] + z
    q['lift']['meters_per_count'] = q['lift']['meters_per_count'] * (1 + d['lscale'])
    q['measured_calibration'] = dict(
        source='docs/EXTRINSIC_CALIBRATION_HANDOFF_20261117.md', model=args.label,
        result_sha256=hashlib.sha256(open(args.result, 'rb').read()).hexdigest(),
        lift_scale=float(d['lscale']), lift_zero_offset_m=float(d['loff']),
        cv_median_mm=R['cv_all_median'], cv_p95_mm=R['cv_p95'], applied_to_robot=False)
    return q


cand = apply(base); cand.pop('calibration_id'); cand['calibration_id'] = 's2-dev-' + digest(cand)[:24]
cw = apply(wide); cw.pop('calibration_id'); cw['base_profile_id'] = cand['calibration_id']
cw['calibration_id'] = 's2-dev-' + digest(cw)[:24]
ch = copy.deepcopy(height); ch.pop('model_id')
X = d['cam']; ch['base_z_from_optical'] = [float(X[2, 0]), float(X[2, 1]), float(X[2, 2]), float(X[2, 3])]
ch['profile_calibration_id'] = cand['calibration_id']
ch['source'] = dict(branch='yeoeun/extrinsic-calibration', path='docs/EXTRINSIC_CALIBRATION_HANDOFF_20261117.md', model=args.label)
ch['status'] = 'candidate_from_joint_calibration_not_applied'
ch['model_id'] = 'height-' + digest(ch)[:24]

# validate with the project's own checks
sys.path.insert(0, os.path.join(args.repo, 'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks import s2_development, s2_geometry as s2
import pathlib
geometry = s2.load_geometry(pathlib.Path(cfg) / 'official_geometry.json')
s2_development.validate(cand, geometry)
assert ch['profile_calibration_id'] == cand['calibration_id'] and ch['ground_z_m'] == cand['ground_z_m']

os.makedirs(args.out, exist_ok=True)
files = {f's2_coordinates.development.{args.tag}.json': cand, f's2_coordinates.wide.development.{args.tag}.json': cw,
         f'camera_height.calibrated.{args.tag}.json': ch}
for n, v in files.items():
    path = os.path.join(args.out, n)
    if os.path.exists(path): sys.exit('exists, not overwritten: ' + path)
    json.dump(v, open(path, 'w'), indent=2); print('wrote', path)

# human-readable change summary
old, new = camera_summary(np.array(base['base_from_optical'])), camera_summary(d['cam'])
print('\ncamera pitch/roll/yaw deg: %.2f/%.2f/%.2f -> %.2f/%.2f/%.2f' % (old['pitch_deg'], old['roll_deg'], old['yaw_deg'],
      new['pitch_deg'], new['roll_deg'], new['yaw_deg']))
print('camera xyz mm:', old['xyz_mm'], '->', new['xyz_mm'])
for arm in ARMS:
    print(f'{arm} joint zeros deg:', {j: round(float(np.degrees(d[f"z_{arm}_{j}"])), 2) for j in ZJ})
print('right mount correction mm:', (d['rm'][:3, 3] * 1000).round(1).tolist())
print('lift meters_per_count: %.6e -> %.6e (scale %+.2f %%), lift zero offset %+.1f mm' % (
      base['lift']['meters_per_count'], cand['lift']['meters_per_count'], 100 * d['lscale'], 1000 * d['loff']))
print('calibration_id:', base['calibration_id'], '->', cand['calibration_id'])
