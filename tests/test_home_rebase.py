"""Replay real calibration evidence without hardware; reject mixed coordinates."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from so101_arm_bridge.development_ranges import validate_development_ranges, offset_shift
from so101_arm_bridge.home_rebase import rebase_verified_ranges
from home_robot_tasks import s2_development as dev, s2_geometry as geo
from home_robot_tasks.installed_jaw import InstalledLeftJaw

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'config/home_assistant'
OLD = CONFIG/'historical/pre-home-20261110'
EVIDENCE = CONFIG/'home_calibration/20261110'


def inputs():
    return (OLD/'arm_ranges.development.json').read_bytes(), {
        arm: (next(EVIDENCE.glob(arm+'-*-centered-*.json')).read_bytes(),
              next(EVIDENCE.glob(arm+'-*-persistence-*.json')).read_bytes())
        for arm in ('left', 'right')}


def test_actual_eeprom_translation_preserves_travel_and_copy_provenance():
    source, records = inputs()
    profile = rebase_verified_ranges(source, records)
    assert profile == json.loads((CONFIG/'arm_ranges.development.json').read_bytes())
    old = json.loads(source)
    for arm in ('left', 'right'):
        for name in geo.JOINTS:
            a = old['arms'][arm][name]; b = profile['arms'][arm][name]
            assert b['maximum_unwrapped_raw']-b['minimum_unwrapped_raw'] == a['maximum_unwrapped_raw']-a['minimum_unwrapped_raw']
            assert 0 <= b['minimum_unwrapped_raw'] <= 2048 <= b['maximum_unwrapped_raw'] <= 4095
            assert b['range_source'] == a['range_source']
            assert b['measured_on_this_arm'] == (arm == 'left')
    # Actual EEPROM rounding differs from nominal 2048 minus pre-center raw.
    assert profile['arms']['left']['shoulder']['minimum_unwrapped_raw'] == 833
    assert profile['arms']['right']['shoulder']['minimum_unwrapped_raw'] == 854
    assert offset_shift(1570, 3532) == -1042


@pytest.mark.parametrize('fault', ['wrong_arm', 'persistence_bytes', 'unverified', 'wrong_center_hash', 'source_offset', 'register_change'])
def test_rebase_rejects_mixed_or_unverified_records(fault):
    source, records = inputs()
    center = json.loads(records['right'][0]); persistence = json.loads(records['right'][1])
    if fault == 'wrong_arm': center['arm'] = 'left'
    elif fault == 'persistence_bytes': persistence['before'][0]['eeprom_hex'] = '00'*40
    elif fault == 'unverified': persistence['status'] = 'backup_saved'
    elif fault == 'wrong_center_hash': persistence['compared_record_sha256'] = '0'*64
    elif fault == 'source_offset':
        d = json.loads(source); d['calibration']['right']['offsets_raw'][0] += 1
        source = json.dumps(d).encode()
    elif fault == 'register_change':
        row = center['after'][0]; raw = bytearray.fromhex(row['eeprom_hex']); raw[10] ^= 1
        row['eeprom_hex'] = raw.hex(); persistence['before'][0]['eeprom_hex'] = raw.hex()
    cb = json.dumps(center).encode()
    if fault != 'wrong_center_hash': persistence['compared_record_sha256'] = hashlib.sha256(cb).hexdigest()
    records['right'] = cb, json.dumps(persistence).encode()
    with pytest.raises(ValueError): rebase_verified_ranges(source, records)


@pytest.mark.parametrize('fault', ['copy_left', 'source', 'direction', 'measured', 'width', 'offset'])
def test_validator_rejects_silent_range_or_provenance_changes(fault):
    d = rebase_verified_ranges(*inputs())
    e = d['arms']['right']['shoulder']
    if fault == 'copy_left': d['arms']['right']['shoulder'] = deepcopy(d['arms']['left']['shoulder'])
    elif fault == 'source': d['rebase']['source_profile']['arms']['left']['shoulder']['maximum_unwrapped_raw'] += 1
    elif fault == 'direction': e['positive_raw_direction'] = -1
    elif fault == 'measured': e['measured_on_this_arm'] = True
    elif fault == 'width': e['maximum_unwrapped_raw'] -= 1
    elif fault == 'offset': d['calibration']['right']['offsets_raw'][1] += 1
    with pytest.raises(ValueError): validate_development_ranges(d)


def test_persistence_position_does_not_redefine_reference_and_old_pad_fit_is_rejected():
    geometry = geo.load_geometry(CONFIG/'official_geometry.json')
    p = dev.released_profile(ROOT, geometry)
    for side in ('left', 'right'):
        a = p['arms'][side]
        assert geo.model_angles(a['reference_raw'], a['alignment']) == pytest.approx(dict.fromkeys(geo.JOINTS, 0.))
        actual = geo.tool_pose(geometry, a, a['reference_raw'])
        official = geo.arm_transforms(geometry, dict.fromkeys(geo.JOINTS, 0.))['gripper_frame_link']
        np.testing.assert_allclose(actual, official, atol=1e-12)
    assert p['arms']['left']['reference_raw']['shoulder'] == 2048  # latest torque-off scan was 3162
    assert p['arms']['left']['reference_raw']['gripper'] == 2047
    assert p['arms']['right']['reference_raw']['wrist_roll'] == 2047
    with pytest.raises(ValueError, match='calibration'):
        InstalledLeftJaw(json.loads((CONFIG/'historical/pre-home-20261110/left_pad_gap.observed.json').read_text()), p)
    rebased = json.loads((CONFIG/'left_pad_gap.observed.json').read_text())
    assert rebased['physical_gap_remeasurement'] is False
    assert InstalledLeftJaw(rebased, p).check_residual_mm == pytest.approx(.4)
