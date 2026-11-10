#!/usr/bin/env python3
"""Rebuild local Home profiles from archived ranges and verified EEPROM records.

No hardware I/O. Firmware headers are generated separately after this step.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'ros2_ws/src/so101_arm_bridge'), str(ROOT/'ros2_ws/src/home_robot_tasks')]
from so101_arm_bridge.home_rebase import rebase_verified_ranges
from so101_arm_bridge.development_ranges import offset_shift
from home_robot_tasks.installed_jaw import InstalledLeftJaw
from home_robot_tasks import s2_geometry as geo


def encoded(document):
    return (json.dumps(document, indent=2, ensure_ascii=False, allow_nan=False)+'\n').encode()


def build(root, source_dir, record_paths):
    records = {arm: tuple(path.read_bytes() for path in paths) for arm, paths in record_paths.items()}
    ranges = rebase_verified_ranges((source_dir/'arm_ranges.development.json').read_bytes(), records)
    range_bytes = encoded(ranges)
    alignment = json.loads((source_dir/'arm_model_alignment.development.json').read_bytes())
    pose = json.loads((source_dir/'arm_home_pose.development.json').read_bytes())
    joints = geo.JOINTS
    geometry = geo.load_geometry(root/'config/home_assistant/official_geometry.json')
    zero = dict.fromkeys(joints, 0.0)
    tcp = geo.arm_transforms(geometry, zero)['gripper_frame_link'][:3, 3].tolist()
    alignment.update(schema_version=2, status='official_stl_zero_centered_and_persistence_verified',
                     direction_source_sha256=hashlib.sha256(range_bytes).hexdigest(),
                     encoder_home_preserved=False, home_recalibrated=True,
                     assembly_revision='mini1-fixed-d415-official-home-20261110')
    for arm in ('left', 'right'):
        center = json.loads(records[arm][0])
        reference = {name: row['position'] for name, row in zip(joints, center['after'], strict=True)}
        a = alignment['arms'][arm]
        a.update(reference_raw=reference, reference_model_rad=deepcopy(zero),
                 reference_report=str(record_paths[arm][0].relative_to(root)),
                 reference_report_sha256=ranges['calibration'][arm]['centered_sha256'],
                 persistence_report=str(record_paths[arm][1].relative_to(root)),
                 persistence_report_sha256=ranges['calibration'][arm]['persistence_sha256'],
                 captured_utc=center['created_utc'], torque_off_verified=True,
                 eeprom_calibration_persistence_verified=True,
                 sample_stability_check='two pre-center scans, per-axis drift checks and final Home readback; original scans retained',
                 model_tcp_in_arm_base_at_reference_m=tcp,
                 reference_note_precedence='operator-confirmed official STL zero; no pad-gap or independent angle measurement')
        a.pop('eeprom_matches_pre_reference_backup', None)
        # This legacy intermediate stores controller directions; the coordinate
        # builder derives official model-axis signs separately from both URDFs.
        a['alignment'] = {name: dict(sign=ranges['arms'][arm][name]['positive_raw_direction'],
            home_model_rad=-ranges['arms'][arm][name]['positive_raw_direction']*(reference[name]-2048)*math.tau/4096)
            for name in joints}
        pose['encoder_reference_raw'][arm] = reference
    pose.update(status='official_stl_zero_centered_and_persistence_verified',
                operator_statement='공식 Home 자세로 한 팔씩 정렬; 집게는 패드 실측 대신 STL 기준',
                operator_physical_pose_statement='left and right separately confirmed by operator',
                model_angles_deg=dict.fromkeys(joints, 0), model_angles_rad=deepcopy(zero),
                existing_eeprom_home_preserved=False,
                moving_jaw_orientation='official STL/URDF at all six model angles zero',
                visualization='output/home-reference-20261109/official-home-reference.png')
    return {'arm_ranges.development.json': range_bytes,
            'arm_model_alignment.development.json': encoded(alignment),
            'arm_home_pose.development.json': encoded(pose)}


def rebase_jaw_record(record, old_profile, new_profile, old_range_bytes, new_ranges, records):
    """Require the same persisted calibration evidence used by the canonical ranges."""
    old_ranges = json.loads(old_range_bytes)
    if rebase_verified_ranges(old_range_bytes, records) != new_ranges:
        raise ValueError('current ranges differ from verified EEPROM rebase')
    key = 'config/home_assistant/arm_ranges.development.json'
    if (old_profile['source_files_sha256'].get(key) != hashlib.sha256(old_range_bytes).hexdigest()
            or new_profile['source_files_sha256'].get(key) != hashlib.sha256(
                (json.dumps(new_ranges, indent=2, ensure_ascii=False, allow_nan=False)+'\n').encode()).hexdigest()):
        raise ValueError('coordinate profiles are not bound to the supplied ranges')
    for field in ('base_from_optical', 'lift', 'camera_device'):
        if old_profile[field] != new_profile[field]:
            raise ValueError('hardware geometry changed during coordinate-only rebase')
    old_jaw = InstalledLeftJaw(record, old_profile)
    shift = offset_shift(old_ranges['calibration']['left']['offsets_raw'][5],
                         new_ranges['calibration']['left']['offsets_raw'][5])
    result = deepcopy(record)
    bounds = new_ranges['arms']['left']['gripper']
    for point in result['samples']+[result['check']['measurement']]:
        for field in ('raw', 'raw_min', 'raw_max'):
            point[field] += shift
            if not bounds['minimum_unwrapped_raw'] <= point[field] <= bounds['maximum_unwrapped_raw']:
                raise ValueError('historical measured point outside translated gripper range')
    result.update(calibration_id=new_profile['calibration_id'], assembly_revision=new_profile['assembly_revision'],
                  calibration_basis='historical_measured_gaps_rebased_by_verified_eeprom_delta',
                  physical_gap_remeasurement=False, motion_authorized=False)
    result['coordinate_rebase'] = dict(
        method='signed_eeprom_offset_delta_without_wrap', raw_shift=shift,
        physical_assumption='same installed pads and geartrain; only Home offset changed',
        source_record=deepcopy(record), source_record_sha256=geo.digest(record),
        source_coordinate_sha256=geo.digest(old_profile), target_coordinate_sha256=geo.digest(new_profile),
        center_sha256=new_ranges['calibration']['left']['centered_sha256'],
        persistence_sha256=new_ranges['calibration']['left']['persistence_sha256'])
    new_jaw = InstalledLeftJaw(result, new_profile)
    # Preserve interpolation and the independent check, not just the saved points.
    low = min(p['raw'] for p in record['samples']); high = max(p['raw'] for p in record['samples'])
    for i in range(101):
        raw = low+(high-low)*i/100
        if abs(old_jaw.gap_mm(raw)-new_jaw.gap_mm(raw+shift)) > 1e-9:
            raise ValueError('aperture conversion changed beyond the coordinate shift')
    if abs(old_jaw.check_residual_mm-new_jaw.check_residual_mm) > 1e-9:
        raise ValueError('independent historical check changed')
    result['check']['predicted_gap_mm'] = new_jaw.gap_mm(result['check']['measurement']['raw'])
    result['check']['absolute_error_mm'] = new_jaw.check_residual_mm
    return result


def build_jaw_profile(root=ROOT, source_dir=None):
    config = root/'config/home_assistant'
    archive = source_dir or config/'historical/pre-home-20261110'
    read = lambda p: json.loads(p.read_bytes())
    alignment = read(config/'arm_model_alignment.development.json')
    records = {side: tuple((root/alignment['arms'][side][key]).read_bytes()
               for key in ('reference_report', 'persistence_report')) for side in ('left', 'right')}
    return rebase_jaw_record(read(archive/'left_pad_gap.observed.json'), read(archive/'s2_coordinates.development.json'),
                     read(config/'s2_coordinates.development.json'), (archive/'arm_ranges.development.json').read_bytes(),
                     read(config/'arm_ranges.development.json'), records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path)
    parser.add_argument('--jaw-only', action='store_true', help='verify or write historical jaw measurements translated by persisted Home offsets')
    for arm in ('left', 'right'):
        parser.add_argument('--'+arm+'-center', type=Path)
        parser.add_argument('--'+arm+'-persistence', type=Path)
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    if args.jaw_only:
        content = encoded(build_jaw_profile(ROOT, args.source_dir.resolve() if args.source_dir else None))
        path = ROOT/'config/home_assistant/left_pad_gap.observed.json'
        if args.write:
            path.write_bytes(content)
        elif not path.is_file() or path.read_bytes() != content:
            raise SystemExit('jaw profile differs: '+str(path))
        print('Historical jaw coordinate rebase verified; no physical remeasurement or device I/O.')
        return
    if args.source_dir is None or any(getattr(args, arm+'_'+kind) is None
            for arm in ('left', 'right') for kind in ('center', 'persistence')):
        parser.error('Home profiles require source directory and both CENTER/persistence records')
    paths = {arm: (getattr(args, arm+'_center').resolve(), getattr(args, arm+'_persistence').resolve())
             for arm in ('left', 'right')}
    for name, content in build(ROOT, args.source_dir.resolve(), paths).items():
        path = ROOT/'config/home_assistant'/name
        if args.write:
            path.write_bytes(content)
        elif not path.exists() or path.read_bytes() != content:
            raise SystemExit('profile differs: '+str(path))
    print('Home ranges and references verified; no device I/O or motion authorization.')


if __name__ == '__main__':
    main()
