#!/usr/bin/env python3
"""Selected-arm physical 2048 calibration; default only reads a durable backup.

Requires arm-calibration-v1 on STM32. This is not a runtime motion tool.
Use --center only after the selected arm and gripper are supported at the
operator-confirmed reference pose. Record the gripper reference explicitly;
2048 does not imply a particular opening width. Torque must already be disabled.
"""
import argparse
import hashlib
import json
from pathlib import Path

import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/so101_arm_bridge'))
from so101_arm_bridge.calibration_maintenance import Link, run, save, validate
from so101_arm_bridge.development_ranges import read_development_ranges
from so101_arm_bridge.home_rebase import centering_preflight

RANGES = ROOT/'config/home_assistant/arm_ranges.development.json'


class SelectedArmLink:
    def __init__(self, link, arm):
        if arm not in ('left', 'right'): raise ValueError('explicit left or right arm required')
        self.link, self.arm = link, arm

    def exchange(self, command, *, scan=False):
        return self.link.exchange(self.arm.upper()+' '+command, scan=scan)


def capture(link, path, arm, *, center=False, reference_note='', verify_record=None, range_path=RANGES,check_home=False):
    if center and not reference_note.strip():
        raise ValueError('record the operator-confirmed reference pose before centering')
    if verify_record is not None and center:
        raise ValueError('verification is read-only')
    preflight = None
    if center or check_home:
        document, _, checksum = read_development_ranges(range_path)
        def preflight(rows):
            return dict(centering_preflight(document,arm,rows), source_ranges_sha256=checksum)
    previous = None
    if verify_record is not None:
        previous_bytes = verify_record.read_bytes()
        previous = json.loads(previous_bytes)
        if previous.get('status') != 'centered_verified' or previous.get('arm') != arm:
            raise ValueError('verification requires a successful record for this arm')
        validate(previous['after'])
    result = run(SelectedArmLink(link, arm), path, center=center,
                 expected_firmware='arm-calibration-v1', arm=arm, verify_torque_only=True,
                 reference_note=reference_note,before_center=preflight)
    if check_home and not center:
        result['centering_preflight']=preflight(result['before'])
        save(path,result)
    if previous is not None:
        matches = all(old['eeprom_hex'] == new['eeprom_hex']
                      for old, new in zip(previous['after'], result['before'], strict=True))
        result.update(status='persistence_verified' if matches else 'persistence_mismatch',
                      calibration_registers_match=matches,
                      compared_record_sha256=hashlib.sha256(previous_bytes).hexdigest())
        save(path, result)
        if not matches: raise RuntimeError('calibration registers changed after power cycle')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=('left', 'right'), required=True)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference-note', default='')
    parser.add_argument('--ranges',type=Path,default=RANGES,
                        help='current measured ranges/EEPROM binding; checked before any CENTER')
    group=parser.add_mutually_exclusive_group()
    group.add_argument('--center', action='store_true')
    group.add_argument('--preflight',action='store_true',help='read Home feasibility without changing calibration')
    group.add_argument('--verify-record', type=Path)
    args=parser.parse_args(argv)
    import serial
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.serial_port import open_exclusive_serial
    with acquire_backend_lease('stm32', 0), open_exclusive_serial(serial, args.port, 921600, .1) as port:
        port.reset_input_buffer()
        report=capture(Link(port),args.output,args.arm,center=args.center,
                       reference_note=args.reference_note,verify_record=args.verify_record,range_path=args.ranges,check_home=args.preflight)
    print(json.dumps(report,indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
