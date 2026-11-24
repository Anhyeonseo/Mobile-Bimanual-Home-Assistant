#!/usr/bin/env python3
"""Preview, or explicitly read one arm servo's gains and runtime settings.

No prepare, torque, movement, STOP, reset, heartbeat or configuration writes.
Firmware must support inventory schema 2 and permit maintenance reads. A busy
owner, STOP admission failure or older firmware is reported without recovery.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/so101_arm_bridge'))

JOINTS = ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll', 'gripper')
DEVICE = '/dev/ttyACM0'


def joint_address(name):
    for bus, side in enumerate(('left', 'right')):
        if name.startswith(side+'_') and name[len(side)+1:] in JOINTS:
            return bus, JOINTS.index(name[len(side)+1:])+1
    raise ValueError('unknown arm joint')


def inspect(transport, joint, expected_firmware=0x24930):
    from so101_arm_bridge.servo_tuning import TuningQueryUnsupported
    bus, motor_id = joint_address(joint)
    result = dict(joint=joint, checked_at_utc=datetime.now(timezone.utc).isoformat(),
                  status='FAILED', motor_write_commands=0, motion_commands=0)
    try:
        hello = transport.enter_binary_mode()
        result['hello'] = asdict(hello)
        if (hello.firmware_version != expected_firmware or hello.protocol_version != 2 or hello.joint_count != 12):
            raise ValueError('unexpected controller identity; tuning query was not sent')
        result['state_before'] = asdict(transport.get_state())
        if result['state_before']['status_code']:
            raise ValueError('controller state query failed; tuning query was not sent')
        try:
            result['snapshot'] = transport.read_servo_tuning(bus, motor_id)
            result['status'] = ('READ_COMPLETE' if result['snapshot']['complete'] else
                                result['snapshot']['status'].upper())
            if result['snapshot']['status'] == 'complete' and not result['snapshot']['complete']:
                result['status'] = 'IDENTITY_MISMATCH'
        except TuningQueryUnsupported as error:
            result.update(status='FIRMWARE_UPDATE_REQUIRED', error=str(error))
        finally:
            result['state_after'] = asdict(transport.get_state())
        unchanged = ('stop_latched', 'protocol_version', 'joint_count',
                     'left_calibration_hash', 'right_calibration_hash')
        if (result['state_after']['status_code'] or
                any(result['state_before'][k] != result['state_after'][k] for k in unchanged)):
            result.update(status='STATE_CHANGED', error='controller state changed during read; no recovery attempted')
    except (OSError, RuntimeError, ValueError) as error:
        result.update(status='FAILED', error=str(error))
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--joint', choices=[side+'_'+j for side in ('left', 'right') for j in JOINTS],
                   default='left_shoulder')
    p.add_argument('--read-motors', action='store_true', help='explicitly connect and query; default is offline preview')
    p.add_argument('--port', default=DEVICE)
    p.add_argument('--expected-firmware', type=lambda s: int(s, 0), default=0x24930)
    p.add_argument('--output', type=Path)
    args = p.parse_args(argv)
    if not args.read_motors:
        report = dict(status='OFFLINE_PREVIEW', joint=args.joint,
                      request_schema=2, registers=[[0, 7], [16, 14], [40, 10]],
                      expected_firmware=hex(args.expected_firmware), devices_opened=0,
                      motor_write_commands=0, motion_commands=0,
                      note='P/D/I, dead zones, startup force, acceleration, goal time/speed; firmware support required')
    else:
        try:
            import serial
            from so101_arm_bridge.backend_lease import acquire_backend_lease
            from so101_arm_bridge.serial_port import open_exclusive_serial
            from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
            with acquire_backend_lease('stm32', 189):
                with open_exclusive_serial(serial, args.port, 921600, .02) as port:
                    report = inspect(StreamValidationTransportV2(port, response_timeout_s=1),
                                     args.joint, args.expected_firmware)
        except (ImportError, OSError, RuntimeError, ValueError) as error:
            report = dict(status='FAILED', error=str(error), motor_write_commands=0, motion_commands=0)
    encoded = json.dumps(report, ensure_ascii=False, indent=2)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded)
    print(encoded, end='')
    return 0 if report['status'] in ('OFFLINE_PREVIEW', 'READ_COMPLETE') else 2


if __name__ == '__main__':
    raise SystemExit(main())
