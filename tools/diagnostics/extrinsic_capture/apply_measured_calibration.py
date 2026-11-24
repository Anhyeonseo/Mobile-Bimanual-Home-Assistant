"""Apply config/home_assistant/measured_calibration.development.json to the released profiles. No device access.

1. Archive the current released files to config/home_assistant/historical/pre-extrinsic-<date>/ (once).
2. s2_coordinates.development.json      = apply_measured_calibration(released nominal profile)
   (the released bytes, not a rebuild: a rebuild may differ by ~1e-16 and change calibration_id)
3. s2_coordinates.wide.development.json = camera_evidence.wide_color_profile(new profile, ...)
4. camera_height.calibrated.json       = z row of the new camera, re-bound to the new profile id
5. left_pad_gap.observed.json          = re-bound to the new profile id (gripper mapping unchanged; no new measurement)
6. Check with the loaders the runtime uses.

usage: python3 tools/diagnostics/extrinsic_capture/apply_measured_calibration.py [--date 20261117] [--check-only]
"""
import sys, json, hashlib, argparse, shutil
from copy import deepcopy
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'ros2_ws/src/home_robot_tasks'), str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks import s2_development as dev, s2_geometry as s2, camera_evidence as ce
from home_robot_tasks.stationary_rgbd import load_surface_height_model
from home_robot_tasks.installed_jaw import InstalledLeftJaw

ap = argparse.ArgumentParser(); ap.add_argument('--date', default='20261117'); ap.add_argument('--check-only', action='store_true')
args = ap.parse_args()
C = ROOT/'config/home_assistant'
NAMES = ('s2_coordinates.development.json', 's2_coordinates.wide.development.json',
         'camera_height.calibrated.json', 'left_pad_gap.observed.json')
read = lambda p: json.loads(Path(p).read_text())
digest = lambda v: hashlib.sha256(json.dumps(v, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def check():
    geometry = s2.load_geometry(C/'official_geometry.json')
    p = dev.released_profile(ROOT, geometry)
    wide, _ = ce.load_capture_profile(ROOT, 'wide')
    height = load_surface_height_model(ROOT)
    jaw = InstalledLeftJaw(read(C/'left_pad_gap.observed.json'), p)
    assert wide['base_profile_id'] == p['calibration_id'] and wide['base_from_optical'] == p['base_from_optical']
    print('check passed:', p['calibration_id'], p['provenance'], '| wide', wide['calibration_id'],
          '| height', height.model_id, '| jaw check residual %.2f mm' % jaw.check_residual_mm)


if args.check_only:
    check(); sys.exit()

archive = C/'historical'/f'pre-extrinsic-{args.date}'
if not archive.exists():
    archive.mkdir(parents=True)
    for n in NAMES: shutil.copy2(C/n, archive/n)
    print('archived to', archive.relative_to(ROOT))
nominal = read(archive/'s2_coordinates.development.json')
if nominal['provenance'].startswith('measured'): sys.exit('archived profile is not nominal')
geometry = s2.load_geometry(C/'official_geometry.json')
raw = (ROOT/dev.MEASURED).read_bytes()
profile = dev.apply_measured_calibration(nominal, json.loads(raw), hashlib.sha256(raw).hexdigest(), geometry)
wide = ce.wide_color_profile(profile, read(C/'camera_device.observed.json'), read(C/'camera_device.wide.observed.json'))

height = read(archive/'camera_height.calibrated.json'); height.pop('model_id')
X = profile['base_from_optical']
height.update(base_z_from_optical=[float(X[2][0]), float(X[2][1]), float(X[2][2]), float(X[2][3])],
              profile_calibration_id=profile['calibration_id'],
              source=dict(record=dev.MEASURED, sha256=hashlib.sha256(raw).hexdigest(),
                          note='z row of the measured base_from_optical (same camera as the profile)'),
              adoption_scope='Surface/object height from raw optical XYZ with the measured camera of the profile',
              status='measured_joint_calibration_pending_physical_grasp_check')
height['model_id'] = 'height-' + digest(height)[:24]

jaw = read(archive/'left_pad_gap.observed.json')
old_g, new_g = nominal['arms']['left']['alignment']['gripper'], profile['arms']['left']['alignment']['gripper']
if old_g != new_g: sys.exit('gripper mapping changed: the jaw record needs a real rebase, not a re-bind')
jaw['measured_calibration_profile_rebind'] = dict(
    previous_profile_id=jaw['calibration_id'], current_profile_id=profile['calibration_id'],
    gripper_mapping_changed=False, new_measurements=False,
    reason='measured camera/arm/lift calibration; left gripper alignment and installed pads unchanged')
jaw['calibration_id'] = profile['calibration_id']

for name, value in zip(NAMES, (profile, wide, height, jaw)):
    (C/name).write_text(json.dumps(value, indent=2) + '\n'); print('wrote', (C/name).relative_to(ROOT))
check()
