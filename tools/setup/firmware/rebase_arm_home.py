#!/usr/bin/env python3
"""Audit old joint bounds against persisted Home offsets, without motor I/O.

The output is a review record, never an authorized operational-limit manifest.
The historical limits have no EEPROM fingerprint, so the computed bounds are
conditional on their coordinate frame matching the pre-centering frame.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/so101_arm_bridge'))
from so101_arm_bridge.limit_manifest import DIRECTIONS, JOINTS, read_limit_manifest


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def signed_offset(value):
    # STS correction: 11-bit magnitude and bit 11 sign (not two's complement).
    if type(value) is not int or not 0 <= value < 4096:
        raise ValueError('invalid STS offset')
    return -(value & 2047) if value & 2048 else value


def modulo_delta(value):
    result = (value + 2048) % 4096 - 2048
    if result == -2048:
        raise ValueError('half-turn branch is ambiguous')
    return result


def rows(record, field):
    result = record[field]
    if not isinstance(result, list) or [r['id'] for r in result] != list(range(1, 7)):
        raise ValueError('expected ordered IDs 1..6')
    for row in result:
        raw = bytes.fromhex(row['eeprom_hex'])
        if (row['ok'] is not True or row['torque'] != 0 or len(raw) != 40
                or raw[5] != row['id'] or raw[33] != 0
                or type(row['position']) is not int or not 0 <= row['position'] < 4096):
            raise ValueError('invalid motor evidence')
    return result


def audit_arm(arm, limits, centered, persisted, centered_sha):
    for record in (centered, persisted):
        info = record['firmware']
        if (record.get('arm') != arm or info.get('arm') != arm
                or info.get('firmware') != 'arm-calibration-v1'
                or info.get('motion_enabled') is not False
                or record.get('motion_commands_sent') is not False):
            raise ValueError('wrong arm or firmware evidence')
    if (centered.get('status') != 'centered_verified'
            or persisted.get('status') != 'persistence_verified'
            or persisted.get('calibration_registers_match') is not True
            or persisted.get('compared_record_sha256') != centered_sha):
        raise ValueError('missing matching power-cycle verification')
    before, after, reboot = rows(centered, 'before'), rows(centered, 'after'), rows(persisted, 'before')
    results = centered['results']
    if (len(results) != 6 or sorted(r['id'] for r in results) != list(range(1, 7))
            or any(r.get('center_ok') is not True or r.get('torque_off_verified') is not True for r in results)):
        raise ValueError('incomplete center results')
    output = {}
    for name, direction, old, new, power in zip(JOINTS, DIRECTIONS, before, after, reboot, strict=True):
        a, b = bytes.fromhex(old['eeprom_hex']), bytes.fromhex(new['eeprom_hex'])
        if (new['eeprom_hex'] != power['eeprom_hex']
                or any(a[i] != b[i] for i in range(40) if i not in (31, 32))
                or abs(new['position'] - 2048) > 8):
            raise ValueError('EEPROM/centering evidence mismatch')
        old_offset = int.from_bytes(a[31:33], 'little')
        new_offset = int.from_bytes(b[31:33], 'little')
        shift = modulo_delta(signed_offset(old_offset) - signed_offset(new_offset))
        residual = modulo_delta(new['position'] - old['position'] - shift)
        if abs(residual) > 8:
            raise ValueError('offset change disagrees with measured encoder change')
        prior = limits[name]
        lo, hi = (prior[key] + shift for key in ('minimum_unwrapped_raw', 'maximum_unwrapped_raw'))
        issues = []
        if not lo <= 2048 <= hi:
            issues.append('new_home_outside_conditional_range')
        if not -4096 <= lo < hi <= 8191 or hi - lo >= 4096:
            issues.append('unsupported_unwrap_range')
        angles = sorted(round(direction * (raw - 2048) * 6283185 / 4096) for raw in (lo, hi))
        output[name] = dict(
            motor_id=old['id'], zero_raw=2048, positive_raw_direction=direction,
            direction_basis='historical convention; no new direction motion test',
            offset_before_raw=old_offset, offset_after_raw=new_offset,
            raw_shift=shift, measured_shift_residual_raw=residual,
            conditional_minimum_unwrapped_raw=lo, conditional_maximum_unwrapped_raw=hi,
            conditional_minimum_urad=angles[0], conditional_maximum_urad=angles[1],
            home_in_conditional_range=lo <= 2048 <= hi,
            crosses_modulo_boundary=lo // 4096 != hi // 4096,
            issues=issues,
        )
    return output


def build_report(manifest, records):
    document, _ = read_limit_manifest(manifest)
    report = dict(
        schema_version=1, record_kind='arm_home_rebase_review',
        status='REVIEW_ONLY_NOT_RUNTIME_LIMITS', firmware_limit_authorized=False,
        hardware_enabled=False, motion_commands_sent=False,
        home_offsets_persistence_verified=True, historical_frame_continuity_verified=False,
        source_limits_sha256=sha(manifest), arms={}, evidence={},
        unresolved=[
            'Historical envelope does not record motor EEPROM offsets; its equivalence to the before frame is unverified.',
            'Reassembly may change physical link alignment; offset arithmetic does not verify mechanical clearance.',
            'New physical Home has not been aligned to URDF joint zeros.',
        ],
    )
    for arm in ('left', 'right'):
        center, persistence = records[arm]
        report['arms'][arm] = audit_arm(arm, document['arms'][arm],
            json.loads(center.read_text()), json.loads(persistence.read_text()), sha(center))
        report['evidence'][arm] = dict(centered_sha256=sha(center), persistence_sha256=sha(persistence))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--limits', type=Path, default=ROOT / 'config/bimanual_operational_limits.json')
    for arm in ('left', 'right'):
        parser.add_argument(f'--{arm}-centered', type=Path, required=True)
        parser.add_argument(f'--{arm}-persistence', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output already exists; preserve the prior review')
    report = build_report(args.limits, {arm: (getattr(args, arm+'_centered'), getattr(args, arm+'_persistence'))
                                        for arm in ('left', 'right')})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print('12 offsets audited; review only, runtime limits unchanged')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
