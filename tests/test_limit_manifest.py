"""Malformed or drifted joint calibration must fail before command admission."""
import copy
import json
import subprocess
import sys
from pathlib import Path

import pytest
from so101_arm_bridge.bimanual_stream_adapter import BimanualStreamContractError, load_operational_limits
from so101_arm_bridge.limit_manifest import validate_limit_manifest

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'config/bimanual_operational_limits.json'


def document():
    return json.loads(MANIFEST.read_text())


@pytest.mark.parametrize('value', [True, 1.5, '1523243', None, 2**31, float('inf')])
def test_rejects_non_integer_limits_without_coercion(tmp_path, value):
    data = document()
    data['arms']['left']['base']['maximum_urad'] = value
    path = tmp_path / 'limits.json'
    path.write_text(json.dumps(data))
    with pytest.raises(BimanualStreamContractError):
        load_operational_limits(path)


@pytest.mark.parametrize('mutation', [
    lambda d: d.update(schema_version=True),
    lambda d: d.update(raw_units_per_turn=4096.0),
    lambda d: d.update(operator_approved=False),
    lambda d: d.update(joint_order=d['joint_order'][::-1]),
    lambda d: d['arms']['left']['base'].update(maximum_urad=1600000),
    lambda d: d['arms']['left']['shoulder'].update(maximum_unwrapped_raw=6000),
    lambda d: d['arms']['left']['shoulder'].update(coordinate='raw'),
    lambda d: d['arms']['left'].pop('gripper'),
    lambda d: d['arms'].update(extra={}),
])
def test_rejects_inconsistent_calibration(mutation):
    data = document()
    mutation(data)
    with pytest.raises(ValueError):
        validate_limit_manifest(data)


def test_duplicate_json_keys_rejected(tmp_path):
    path = tmp_path / 'limits.json'
    path.write_text(MANIFEST.read_text().replace('"schema_version": 1,', '"schema_version": 1, "schema_version": 1,'))
    with pytest.raises(BimanualStreamContractError, match='duplicate'):
        load_operational_limits(path)


def test_rejects_non_object_document(tmp_path):
    path = tmp_path / 'limits.json'
    path.write_text('[]')
    with pytest.raises(BimanualStreamContractError):
        load_operational_limits(path)


def test_generated_table_and_package_copy_are_current():
    subprocess.run([sys.executable, str(ROOT / 'tools/setup/firmware/generate_joint_limits.py'), '--check'], check=True)


def test_validation_does_not_modify_or_expand_calibration():
    data = document()
    before = copy.deepcopy(data)
    assert len(validate_limit_manifest(data)) == 12
    assert data == before
