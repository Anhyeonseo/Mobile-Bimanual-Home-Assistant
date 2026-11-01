#!/usr/bin/env python3
"""Inspect the S1 inventory firmware; motor READs require --read-motors.

Default: HELLO and host state only, without servo bus access.
Never enables torque, writes settings, anchors a trajectory or sends STOP.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/so101_arm_bridge'))
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.serial_port import open_exclusive_serial
from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
from so101_arm_bridge.device_inventory import FIRMWARE_VERSION, collect_inventory


def inspect(transport, read_motors=False):
    hello = transport.enter_binary_mode()
    report = dict(checked_at_utc=datetime.now(timezone.utc).isoformat(), hello=asdict(hello),
                  motor_write_commands=0, motion_authorized=False, physical_calibration_verified=False)
    if hello.firmware_version != FIRMWARE_VERSION or hello.joint_count != 12 or hello.protocol_version != 2:
        raise RuntimeError('S1 inventory firmware 0x00024909 / 12 joints required; no servo reads sent')
    state = transport.get_state()
    report['state_before'] = asdict(state)
    if hello.stop_latched or state.stop_latched or state.status_code:
        report.update(passed=False, motor_reads_requested=read_motors, motor_reads_sent=0,
                      error='controller not ready for maintenance; no servo reads sent')
        return report
    if read_motors:
        report.update(collect_inventory(transport))
    report['motor_reads_requested'] = read_motors
    try:
        report['state_after'] = asdict(transport.get_state())
    except (OSError, RuntimeError, ValueError) as error:
        report.update(passed=False, final_state_error=str(error))
        return report
    report['passed'] = (not report['state_after']['stop_latched'] and
                        report['state_after']['status_code'] == 0 and
                        all(report['state_after'][key] == report['state_before'][key]
                            for key in ('protocol_version', 'joint_count',
                                        'left_calibration_hash', 'right_calibration_hash')) and
                        (not read_motors or report['all_snapshots_complete']))
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--read-motors', action='store_true')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(argv)
    try:
        import serial
        with acquire_backend_lease('stm32', 0):
            with open_exclusive_serial(serial, args.port, 921600, .2) as port:
                report = inspect(StreamValidationTransportV2(port, response_timeout_s=1), args.read_motors)
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        report = dict(passed=False, error=str(error), motor_write_commands=0, motion_authorized=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(report, ensure_ascii=False, indent=2)+'\n'
    args.output.write_text(encoded); print(encoded, end='')
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
