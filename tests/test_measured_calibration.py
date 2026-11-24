"""The 2026-11-17 measured camera/arm/lift calibration as released, and its binding rules. No device access."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from home_robot_tasks import s2_development as dev, s2_geometry as geo, camera_evidence as ce
from home_robot_tasks.stationary_rgbd import load_surface_height_model
from home_robot_tasks.installed_jaw import InstalledLeftJaw

ROOT = Path(__file__).resolve().parents[1]
C = ROOT/'config/home_assistant'
ARCHIVE = C/'historical/pre-extrinsic-20261117'
read = lambda p: json.loads(Path(p).read_text())


@pytest.fixture(scope='module')
def ctx():
    g = geo.load_geometry(C/'official_geometry.json')
    raw = (ROOT/dev.MEASURED).read_bytes()
    return g, read(ARCHIVE/'s2_coordinates.development.json'), json.loads(raw), hashlib.sha256(raw).hexdigest()


def test_released_profile_is_the_measured_calibration_applied_to_the_archived_nominal(ctx):
    g, nominal, measured, sha = ctx
    released = dev.released_profile(ROOT, g)
    rebuilt = dev.apply_measured_calibration(nominal, measured, sha, g)
    # BLAS changes the right-mount product by ~1e-16 across hosts. Preserve the
    # upstream released bytes/ID; do not regenerate deployment files for that.
    for side in geo.ARMS:
        np.testing.assert_allclose(rebuilt['arms'][side]['base_from_arm_at_lift_zero'],
                                   released['arms'][side]['base_from_arm_at_lift_zero'], rtol=0, atol=1e-15)
        rebuilt['arms'][side]['base_from_arm_at_lift_zero'] = released['arms'][side]['base_from_arm_at_lift_zero']
    rebuilt['calibration_id'] = released['calibration_id']
    assert released == rebuilt
    assert released['provenance'] == 'measured_joint_camera_arm_lift_calibration'
    assert released['nominal_calibration_id'] == nominal['calibration_id']
    assert 'camera_height_adjustment' not in released
    assert released['motion_authorized'] is False and released['coordinate_use_allowed'] is False


def test_each_measured_term_lands_in_its_runtime_field(ctx):
    g, nominal, m, sha = ctx
    p = dev.apply_measured_calibration(nominal, m, sha, g)
    np.testing.assert_allclose(p['base_from_optical'], m['base_from_optical'], atol=0)
    assert p['lift']['meters_per_count'] == pytest.approx(nominal['lift']['meters_per_count'] * (1 + m['lift_travel_scale']))
    axis = np.array(nominal['lift']['axis_base'])
    for side in ('left', 'right'):
        old = np.array(nominal['arms'][side]['base_from_arm_at_lift_zero'])
        if side == 'right': old = old @ np.array(m['right_mount_correction'])
        old[:3, 3] += axis * m['lift_zero_offset_m']
        np.testing.assert_allclose(p['arms'][side]['base_from_arm_at_lift_zero'], old, atol=1e-15)
        a = p['arms'][side]
        expected = {j: m['joint_zero_offset_rad'][side].get(j, 0.) for j in geo.JOINTS}
        assert geo.model_angles(a['reference_raw'], a['alignment']) == pytest.approx(expected)
        assert a['reference_model_rad'] == pytest.approx(expected)
        assert a['alignment']['base']['home_model_rad'] == nominal['arms'][side]['alignment']['base']['home_model_rad']
        assert a['alignment']['gripper'] == nominal['arms'][side]['alignment']['gripper']


def test_build_applies_the_calibration_to_a_rebuilt_nominal(ctx):
    g, nominal, m, sha = ctx
    built, released = dev.build(ROOT), dev.released_profile(ROOT, g)
    for key in ('base_from_optical',):
        np.testing.assert_allclose(built[key], released[key], atol=1e-12)
    for side in ('left', 'right'):
        np.testing.assert_allclose(built['arms'][side]['base_from_arm_at_lift_zero'],
                                   released['arms'][side]['base_from_arm_at_lift_zero'], atol=1e-12)
    assert built['lift']['meters_per_count'] == pytest.approx(released['lift']['meters_per_count'], rel=1e-12)
    assert dev.build(ROOT, measured=False)['provenance'] == 'nominal_with_operator_height_adjustment'


@pytest.mark.parametrize('fault', ['camera', 'mount', 'home', 'lift', 'geometry', 'sag', 'base_zero', 'kind'])
def test_calibration_fitted_against_another_nominal_is_refused(ctx, fault):
    g, nominal, m, sha = ctx
    nominal, m = deepcopy(nominal), deepcopy(m)
    if fault == 'camera': nominal['base_from_optical'][0][3] += 1e-6
    elif fault == 'mount': nominal['arms']['right']['base_from_arm_at_lift_zero'][2][3] += 1e-6
    elif fault == 'home': nominal['arms']['left']['alignment']['shoulder']['home_model_rad'] += 1e-6
    elif fault == 'lift': nominal['lift']['meters_per_count'] *= 1.001
    elif fault == 'geometry': m['geometry_sha256'] = '0' * 64
    elif fault == 'sag': m['gravity_sag_applied'] = True
    elif fault == 'base_zero': m['joint_zero_offset_rad']['left']['base'] = .01
    else: m['record_kind'] = 'other'
    with pytest.raises(ValueError):
        dev.apply_measured_calibration(nominal, m, sha, g)


def test_height_model_and_wide_profile_share_the_measured_camera(ctx):
    g, *_ = ctx
    p = dev.released_profile(ROOT, g)
    height = load_surface_height_model(ROOT)
    assert list(height.base_z_from_optical) == pytest.approx(p['base_from_optical'][2])
    wide, _ = ce.load_capture_profile(ROOT, 'wide')
    assert wide['base_profile_id'] == p['calibration_id'] and wide['base_from_optical'] == p['base_from_optical']
    assert wide == ce.wide_color_profile(p, read(C/'camera_device.observed.json'), read(C/'camera_device.wide.observed.json'))


def test_jaw_record_rebind_keeps_every_measured_aperture(ctx):
    g, nominal, *_ = ctx
    p = dev.released_profile(ROOT, g)
    before, after = read(ARCHIVE/'left_pad_gap.observed.json'), read(C/'left_pad_gap.observed.json')
    assert before['samples'] == after['samples'] and before['check'] == after['check']
    assert after['measured_calibration_profile_rebind']['gripper_mapping_changed'] is False
    old, new = InstalledLeftJaw(before, nominal), InstalledLeftJaw(after, p)
    for raw in (2000, 2100, 2200, 2300):
        assert old.gap_mm(raw) == new.gap_mm(raw)
    with pytest.raises(ValueError):
        InstalledLeftJaw(before, p)


@pytest.mark.parametrize('controller_um', [0, 8000, 45900, 299500])
def test_integrated_controller_height_uses_scale_once_without_reapplying_mount_offset(ctx, controller_um):
    from home_robot_tasks.unified_carry_inputs import controller_lift_height_m, UnifiedPlatformGuard
    from test_unified_carry_inputs import samples
    from dataclasses import replace
    g, nominal, m, _ = ctx
    profile = dev.released_profile(ROOT, g)
    expected = controller_um / 1e6 * (1 + m['lift_travel_scale'])
    assert controller_lift_height_m(controller_um, nominal) == controller_um / 1e6
    assert controller_lift_height_m(controller_um, profile) == pytest.approx(expected, abs=1e-15)
    mobile, lift = samples()
    lift = replace(lift, height_um=controller_um, target_um=controller_um)
    guard = UnifiedPlatformGuard(4, expected, profile=profile)
    guard.update(mobile, lift, 20., 20.01)
    assert guard.current_height == pytest.approx(expected, abs=1e-15)
    assert guard.last_observation['lift']['height_um'] == controller_um
    with pytest.raises(ValueError, match='stop proof lost'):
        guard.update(mobile, replace(lift, height_um=controller_um + 3001), 20., 20.01)


def test_measured_capture_rejects_old_profile_and_mixed_unscaled_height(ctx, tmp_path):
    from home_robot_tasks.pick_capture import make_evidence, save_capture, review_capture
    from test_pick_pipeline import capture_fixture
    profile, platform, history, camera, meta = capture_fixture()
    evidence = make_evidence(camera, history, platform, profile, now=20.02,
                             requested_s=19.9, session_id='calibration-test', anchor=(0.,)*12)
    folder = tmp_path / camera['capture_id']
    saved = save_capture(folder, (np.zeros((8,8,3),np.uint8), np.ones((8,8),np.uint16), meta), evidence)
    review_capture(folder, saved, profile)
    old = read(ARCHIVE/'s2_coordinates.wide.development.json')
    with pytest.raises(ValueError, match='schema/profile mismatch'):
        review_capture(folder, saved, old)
    saved['fixed_lift_m'] = platform['lift']['height_um'] / 1e6
    with pytest.raises(ValueError, match='evidence changed: fixed_lift_m'):
        review_capture(folder, saved, profile)
