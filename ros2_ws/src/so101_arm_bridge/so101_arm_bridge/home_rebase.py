"""Read-only feasibility of assigning a supported physical Home to raw 2048."""
from copy import deepcopy
import hashlib
import json

from .development_ranges import unwrap_in_range, validate_development_ranges, offset_shift, translated_entry
from .limit_manifest import JOINTS


def rebase_verified_ranges(source_bytes, records):
    """Build a non-executable profile from actual CENTER and persistence evidence.

    records maps each arm to (center_bytes, persistence_bytes). Current positions
    in the persistence scan need not still be Home and never redefine the zero.
    """
    source = json.loads(source_bytes)
    validate_development_ranges(source)
    if source['schema_version'] != 1 or set(records) != {'left', 'right'}:
        raise ValueError('original profile and both arms required')
    result = deepcopy(source)
    result.update(schema_version=2, status='HOME_REBASED_LEFT_CAPTURED_RIGHT_COPIED',
                  right_policy='Original left copy translated by right EEPROM change; no right range measurement',
                  home_branch_alignment_verified=False)
    result['rebase'] = dict(source_profile=source,
        source_file_sha256=hashlib.sha256(source_bytes).hexdigest(),
        source_document_sha256=hashlib.sha256(json.dumps(source, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
        method='signed EEPROM delta; preserve full continuous range and original measurement provenance')
    from .calibration_maintenance import validate
    for arm, (center_bytes, persistence_bytes) in records.items():
        center = json.loads(center_bytes); persistence = json.loads(persistence_bytes)
        digest = hashlib.sha256(center_bytes).hexdigest()
        if (center.get('status') != 'centered_verified' or center.get('arm') != arm
                or center.get('motion_commands_sent') is not False
                or persistence.get('status') != 'persistence_verified' or persistence.get('arm') != arm
                or persistence.get('motion_commands_sent') is not False
                or persistence.get('calibration_registers_match') is not True
                or persistence.get('compared_record_sha256') != digest):
            raise ValueError('verified same-arm CENTER and persistence evidence required')
        before = [center['center_reference_rows'][str(i)] for i in range(1, 7)]
        after = center['after']; persisted = persistence['before']
        for rows in (before, after, persisted): validate(rows)
        offsets = []
        for i, name in enumerate(JOINTS):
            old = bytes.fromhex(before[i]['eeprom_hex']); new = bytes.fromhex(after[i]['eeprom_hex'])
            old_offset = int.from_bytes(old[31:33], 'little'); new_offset = int.from_bytes(new[31:33], 'little')
            if (old_offset != source['calibration'][arm]['offsets_raw'][i]
                    or new != bytes.fromhex(persisted[i]['eeprom_hex'])
                    or any(old[k] != new[k] for k in range(40) if k not in (31, 32))
                    or abs(after[i]['position']-2048) > 8):
                raise ValueError('calibration bytes or Home readback mismatch')
            entry = source['arms'][arm][name]
            raw = unwrap_in_range(entry, before[i]['position'])
            shift = offset_shift(old_offset, new_offset)
            if abs(raw + shift - after[i]['position']) > 8:
                raise ValueError('EEPROM shift differs from observed Home')
            result['arms'][arm][name] = translated_entry(entry, shift)
            offsets.append(new_offset)
        result['calibration'][arm] = dict(centered_sha256=digest,
            persistence_sha256=hashlib.sha256(persistence_bytes).hexdigest(), offsets_raw=offsets)
    validate_development_ranges(result)
    return result


def centering_preflight(document, arm, rows):
    """Check all six axes before the first EEPROM write, without shrinking travel.

    Rebased intervals are a proposal, not an installed range profile. The old
    measured/copied provenance survives; mechanical reassembly needs separate
    verification. An offset mismatch invalidates the old coordinate evidence.
    """
    validate_development_ranges(document)
    if arm not in ('left', 'right') or [r['id'] for r in rows] != list(range(1, 7)):
        raise ValueError('selected arm and six ordered encoder readings required')
    joints = []
    for name, row, expected in zip(JOINTS, rows, document['calibration'][arm]['offsets_raw']):
        entry = document['arms'][arm][name]
        raw = bytes.fromhex(row['eeprom_hex'])
        if len(raw) != 40 or int.from_bytes(raw[31:33], 'little') != expected:
            raise ValueError('stored range calibration differs from live EEPROM: '+name)
        reference = unwrap_in_range(entry, row['position'])
        lo, hi = entry['minimum_unwrapped_raw'], entry['maximum_unwrapped_raw']
        shift = 2048-reference
        new_lo, new_hi = lo+shift, hi+shift
        feasible = [max(0, reference-lo), min(4095, 4095-(hi-reference))]
        joints.append(dict(joint=name, reference_unwrapped_raw=reference,
                           proposed_minimum_raw=new_lo, proposed_maximum_raw=new_hi,
                           home_2048_fits=0 <= new_lo <= new_hi <= 4095,
                           feasible_home_raw_interval=feasible,
                           positive_raw_direction=entry['positive_raw_direction'],
                           range_source=entry['range_source']))
    return dict(kind='home_2048_preflight', arm=arm, joints=joints,
                safe_to_center=all(j['home_2048_fits'] for j in joints),
                invalidated_artifacts=['arm_ranges_and_firmware_hash', 'model_alignment_and_TF',
                                       'jaw_gap_calibration', 'saved_raw_poses_and_paths'],
                motion_authorized=False)
