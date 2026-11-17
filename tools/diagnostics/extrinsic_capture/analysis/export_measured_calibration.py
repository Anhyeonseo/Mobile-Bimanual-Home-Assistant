"""Export one sag-free model of calib_encoder_lift.py as config/home_assistant/measured_calibration.development.json.

The record holds only what s2_development.apply_measured_calibration can apply (camera, shoulder/elbow/wrist_flex
zero offsets, right-mount correction, lift zero offset and travel scale) plus the nominal values it was fitted
against, which the build compares numerically before applying.

usage: python3 export_measured_calibration.py REPO CAPTURE_DIR TOUR_DIRS RESULT.json "MODEL LABEL" NOMINAL_PROFILE.json OUT.json
       [--bottom-raw 3900] [--date 2026-11-17]
"""
import sys, os, json, argparse, hashlib
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from calib_model import Context, ARMS, ZJ
from encoder_lift_model import EncModel, add_encoder_heights

ap = argparse.ArgumentParser()
ap.add_argument('repo'); ap.add_argument('capture'); ap.add_argument('tours'); ap.add_argument('result'); ap.add_argument('label')
ap.add_argument('nominal'); ap.add_argument('out'); ap.add_argument('--bottom-raw', type=int, default=3900)
ap.add_argument('--date', default='2026-11-17')
args = ap.parse_args()

R = json.load(open(args.result))[args.label]
ctx = Context(args.repo, [args.capture] + args.tours.split(',')); add_encoder_heights(ctx, args.bottom_raw)
free = {n.split('.')[0] for n in R['params']} | {'cam', 'rm'} | {'gm_' + a for a in ctx.atts}
m = EncModel(ctx, free); p = np.array(R['p'])
if len(p) != m.size: sys.exit('parameter layout mismatch: run with the same folders as the result')
d = m.unpack(p)
if any(k.startswith('c_') and abs(v) > 0 for k, v in d.items() if not isinstance(v, np.ndarray)):
    sys.exit('this model has gravity-sag terms; the profile cannot represent them')
if any(abs(d.get(k, 0.0)) > 0 for k in ('lt_x', 'lt_y')): sys.exit('lift tilt is not representable in the profile')
nominal = json.load(open(args.nominal))
if nominal.get('provenance', '').startswith('measured'): sys.exit('nominal profile required (this one is already measured)')
record = dict(
    schema_version=1, record_kind='measured_joint_camera_arm_lift_calibration',
    calibration_date=args.date, model=args.label,
    result_sha256=hashlib.sha256(open(args.result, 'rb').read()).hexdigest(),
    cross_validation=dict(median_mm=R['cv_all_median'], p95_mm=R['cv_p95'],
                          per_arm_level={k: {kk: round(vv, 2) for kk, vv in v.items()} for k, v in R['cv'].items()},
                          metric='5-fold held-out arm-marker position error, torque-on tour shots'),
    gauge='left arm CAD mount and left base zero; right base zero folded into right_mount_correction',
    geometry_sha256=nominal['geometry_sha256'],
    fitted_against=dict(
        calibration_id=nominal['calibration_id'], base_from_optical=nominal['base_from_optical'],
        meters_per_count=nominal['lift']['meters_per_count'], axis_base=nominal['lift']['axis_base'],
        base_from_arm_at_lift_zero={a: nominal['arms'][a]['base_from_arm_at_lift_zero'] for a in ARMS},
        home_model_rad={a: {j: v['home_model_rad'] for j, v in nominal['arms'][a]['alignment'].items()} for a in ARMS}),
    base_from_optical=d['cam'].tolist(),
    joint_zero_offset_rad={a: {j: float(d[f'z_{a}_{j}']) for j in ZJ if j != 'base'} for a in ARMS},
    right_mount_correction=d['rm'].tolist(),
    lift_zero_offset_m=float(d['loff']), lift_travel_scale=float(d['lscale']),
    lift_bottom_raw=args.bottom_raw,
    gravity_sag_applied=False, motion_authorized=False,
    source='docs/EXTRINSIC_CALIBRATION_HANDOFF_20261117.md; data ~/cal_20261116 (not in the repository)')
if os.path.exists(args.out): sys.exit('exists, not overwritten: ' + args.out)
json.dump(record, open(args.out, 'w'), indent=2); print('wrote', args.out)
