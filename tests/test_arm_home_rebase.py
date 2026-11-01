import copy
import importlib.util
import json
from pathlib import Path

import pytest
from so101_arm_bridge.limit_manifest import validate_limit_manifest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('home_rebase', ROOT / 'tools/setup/firmware/rebase_arm_home.py')
rebase = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rebase)
LIMITS = ROOT / 'config/bimanual_operational_limits.json'


def sample(arm='right', old_offset=3683, new_offset=1432):
    def row(i, offset, position):
        eeprom = bytearray(40)
        eeprom[5] = i
        eeprom[31:33] = offset.to_bytes(2, 'little')
        return dict(id=i, ok=True, torque=0, position=position, eeprom_hex=eeprom.hex())
    shift = rebase.modulo_delta(rebase.signed_offset(old_offset) - rebase.signed_offset(new_offset))
    after = [row(i, new_offset, 2048) for i in range(1, 7)]
    centered = dict(arm=arm, firmware=dict(arm=arm, firmware='arm-calibration-v1', motion_enabled=False),
        status='centered_verified', motion_commands_sent=False,
        before=[row(i, old_offset, (2048-shift) % 4096) for i in range(1, 7)], after=after,
        results=[dict(id=i, center_ok=True, torque_off_verified=True) for i in range(1, 7)])
    persisted = copy.deepcopy(centered)
    persisted.update(status='persistence_verified', calibration_registers_match=True,
        compared_record_sha256='hash', before=copy.deepcopy(after))
    return centered, persisted


def audit(c, p):
    return rebase.audit_arm('right', json.loads(LIMITS.read_text())['arms']['right'], c, p, 'hash')


def test_sign_magnitude_wrap_does_not_create_reverse_full_turn():
    c, p = sample()
    r = audit(c, p)['base']
    assert r['raw_shift'] == 1029  # signed EEPROM subtraction is -3067
    assert (r['conditional_minimum_unwrapped_raw'], r['conditional_maximum_unwrapped_raw']) == (2137, 4025)
    assert not r['home_in_conditional_range']
    assert r['issues'] == ['new_home_outside_conditional_range']


def test_preserves_interval_width_and_wrap_and_negative_direction():
    c, p = sample(old_offset=2034, new_offset=2042)
    r = audit(c, p)
    assert r['shoulder']['crosses_modulo_boundary']
    assert r['shoulder']['conditional_maximum_unwrapped_raw'] == 4180
    for joint, entry in r.items():
        old = json.loads(LIMITS.read_text())['arms']['right'][joint]
        assert entry['conditional_maximum_unwrapped_raw'] - entry['conditional_minimum_unwrapped_raw'] == old['maximum_unwrapped_raw'] - old['minimum_unwrapped_raw']
        assert entry['conditional_minimum_urad'] < entry['conditional_maximum_urad']
    assert r['elbow']['positive_raw_direction'] == -1


@pytest.mark.parametrize('mutate', [
    lambda c,p: p.update(compared_record_sha256='other calibration'),
    lambda c,p: p.update(status='backup_saved'),
    lambda c,p: c.update(arm='left'),
    lambda c,p: c['results'].pop(),
    lambda c,p: p['before'][0].update(eeprom_hex='00'*40),
    lambda c,p: c['before'][0].update(position=2000),
    lambda c,p: c['after'][0].update(torque=1),
    lambda c,p: c['before'].reverse(),
])
def test_rejects_missing_stale_wrong_bus_or_inconsistent_evidence(mutate):
    c, p = sample()
    mutate(c, p)
    with pytest.raises(ValueError):
        audit(c, p)


def test_report_cannot_be_loaded_as_authorized_runtime_limits(tmp_path):
    paths = {}
    for arm in ('left', 'right'):
        c, p = sample(arm=arm)
        cp, pp = tmp_path / (arm+'-c.json'), tmp_path / (arm+'-p.json')
        cp.write_text(json.dumps(c))
        p['compared_record_sha256'] = rebase.sha(cp)
        pp.write_text(json.dumps(p))
        paths[arm] = cp, pp
    original = LIMITS.read_bytes()
    report = rebase.build_report(LIMITS, paths)
    assert LIMITS.read_bytes() == original
    assert not report['firmware_limit_authorized']
    assert not report['historical_frame_continuity_verified']
    with pytest.raises(ValueError):
        validate_limit_manifest(report)


def test_half_turn_and_invalid_offsets_rejected():
    with pytest.raises(ValueError):
        rebase.modulo_delta(2048)
    for invalid in (True, -1, 4096, 1.0):
        with pytest.raises(ValueError):
            rebase.signed_offset(invalid)
