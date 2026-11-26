#!/usr/bin/env python3
"""Read installed calibration/code bytes, or compare a saved snapshot. No device I/O."""
import argparse
import base64
from datetime import datetime, timezone
import difflib
import hashlib
import json
from pathlib import Path

FILES = tuple('config/home_assistant/'+name for name in (
    's2_coordinates.development.json', 's2_coordinates.wide.development.json',
    'camera_height.calibrated.json', 'left_pad_gap.observed.json',
    'measured_calibration.development.json')) + tuple(
    'ros2_ws/src/home_robot_tasks/home_robot_tasks/'+name for name in (
    's2_development.py', 'unified_carry_inputs.py', 'pick_capture.py', 'pick_hardware.py',
    'carry_live_inputs.py', 'camera_evidence.py', 'platform_preparation.py')) + (
    'tools/run/unified_pick.py', 'tools/run/prepare_unified_controller.py',
    'ros2_ws/src/so101_arm_bridge/so101_arm_bridge/encoder_units.py',
    'tools/diagnostics/extrinsic_capture/apply_measured_calibration.py')


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def collect(source):
    source = Path(source).resolve()
    if not source.is_dir():
        raise ValueError('source directory not found: '+str(source))
    records = {}
    for name in FILES:
        path = source/name
        if not path.exists():
            records[name] = dict(status='MISSING')
            continue
        if not path.resolve().is_relative_to(source) or not path.is_file():
            raise ValueError('expected a regular file inside the installation: '+name)
        if path.stat().st_size > 2*1024*1024:
            raise ValueError('unexpectedly large source file: '+name)
        raw = path.read_bytes()
        records[name] = dict(status='PRESENT', sha256=sha(raw),
                             base64=base64.b64encode(raw).decode('ascii'))
    return dict(source=str(source), captured_at_utc=datetime.now(timezone.utc).isoformat(),
                files=records, hardware_commands=0, source_files_modified=False)


def compare(snapshot, package, output):
    manifest = json.loads((package/'overlay.json').read_text())
    report = json.loads(snapshot.read_text())
    output.mkdir(parents=True, exist_ok=False)
    summary, diffs = [], []
    for name in FILES:
        record = report['files'][name]
        raw = None
        if record['status'] == 'PRESENT':
            raw = base64.b64decode(record['base64'], validate=True)
            if sha(raw) != record['sha256']:
                raise ValueError('snapshot checksum mismatch: '+name)
            target = output/'files'/name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(raw)
        if name not in manifest['files']:
            continue
        spec = manifest['files'][name]
        wanted = (package/'payload'/name).read_bytes()
        if sha(wanted) != spec['sha256']:
            raise ValueError('packaged payload changed: '+name)
        actual = sha(raw) if raw is not None else None
        if actual == spec['sha256']:
            state = 'ALREADY_UPDATED'
        elif actual == spec['before_sha256']:
            state = 'EXPECTED_ORIGINAL' if raw is not None else 'NEW_FILE'
        elif raw is None and spec.get('optional_existing'):
            state = 'OPTIONAL_NOT_INSTALLED'
        else:
            state = 'DIFFERENT'
        summary.append(dict(path=name, status=state, actual_sha256=actual,
                            expected_before_sha256=spec['before_sha256']))
        if state == 'DIFFERENT' and raw is not None:
            diffs.extend(difflib.unified_diff(raw.decode('utf-8').splitlines(True),
                wanted.decode('utf-8').splitlines(True), fromfile='jetson/'+name, tofile='proposed/'+name))
    result = dict(source=report['source'], captured_at_utc=report['captured_at_utc'],
                  files=summary, hardware_commands=0, source_files_modified=False)
    (output/'comparison.json').write_text(json.dumps(result, indent=2)+'\n')
    (output/'review.diff').write_text(''.join(diffs))  # Review only; never applied automatically.
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('/home/hyper/robot/releases/s4-unified-pick-20261116'))
    parser.add_argument('--compare', type=Path)
    parser.add_argument('--package', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.compare:
        if args.package is None or args.output is None:
            parser.error('--compare requires --package and --output')
        result = compare(args.compare, args.package, args.output)
        for row in result['files']:
            print(row['status']+': '+row['path'])
        print('INSPECTION_SAVED: '+str(args.output.resolve()))
    else:
        print(json.dumps(collect(args.source)))
