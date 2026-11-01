#!/usr/bin/env python3
"""Check a known legacy teleop link using INFO only; never auto-detect protocols.

This is a maintenance tool, not the runtime controller. It takes the same
exclusive backend lease as the controller and closes each port after querying.
No STOP, ARM, binary-mode switch, reset or motor command is sent.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/so101_arm_bridge'))
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.serial_port import open_exclusive_serial

IDENTITIES = ('mobile-base-teleop-v4', 'mobile-base-lift-teleop-v3.1')
BAUD = 921600
MAX_RESPONSE = 1024


def read_info(port, expected_firmware, *, clock=time.monotonic, timeout_s=2.0):
    if expected_firmware not in IDENTITIES:
        raise ValueError('select a previously identified legacy firmware')
    if not 0 < timeout_s <= 2:
        raise ValueError('timeout must be in (0, 2] seconds')
    port.reset_input_buffer()
    if port.write(b'INFO\n') != 5:
        raise RuntimeError('partial INFO write; no automatic resend')
    start = clock()
    data = bytearray()
    while clock() - start < timeout_s:
        part = port.read(1)
        if part == b'\n':
            info = json.loads(data.decode('ascii'))
            expected = dict(firmware=expected_firmware, host_baud=BAUD,
                            servo_baud=1000000, ids=[7, 8, 9, 10],
                            max_raw=200, watchdog_ms=400)
            if expected_firmware == 'mobile-base-lift-teleop-v3.1':
                expected['lift_window_ticks'] = 640
            # Compare canonical JSON too: True must not pass as numeric 1.
            if json.dumps(info, sort_keys=True) != json.dumps(expected, sort_keys=True):
                raise ValueError('unexpected INFO identity or contract; no fallback protocol sent')
            return info
        data.extend(part)
        if len(data) > MAX_RESPONSE:
            raise ValueError('oversized INFO response')
    raise TimeoutError(f'INFO timeout ({len(data)} response bytes); cause undetermined')


def check_link(port_factory, expected_firmware, attempts, *, clock=time.monotonic):
    if expected_firmware not in IDENTITIES:
        raise ValueError('unsupported legacy firmware')
    if type(attempts) is not int or not 1 <= attempts <= 10:
        raise ValueError('attempts must be an integer in 1..10')
    report = dict(schema_version=1, checked_at_utc=datetime.now(timezone.utc).isoformat(),
                  protocol='legacy-teleop-info', expected_firmware=expected_firmware,
                  host_baud=BAUD, requested_attempts=attempts, results=[],
                  motor_commands_sent=0, mode_changed=False, reset_requested=False,
                  motion_authorized=False, motor_ids_verified=False,
                  root_cause_of_previous_failure='undetermined')
    for number in range(1, attempts + 1):
        start = clock()
        try:
            with port_factory() as port:
                info = read_info(port, expected_firmware, clock=clock)
            result = dict(attempt=number, success=True, info=info)
        except (OSError, RuntimeError, ValueError, UnicodeError) as error:
            result = dict(attempt=number, success=False, error=str(error))
        result['elapsed_ms'] = round((clock() - start) * 1000, 3)
        report['results'].append(result)
        if not result['success']:
            break  # Keep the first failure; don't hide it behind successful retries.
    report['link_verified'] = (len(report['results']) == attempts
                               and all(item['success'] for item in report['results']))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='known STM32 ST-LINK serial port')
    parser.add_argument('--expect-firmware', required=True, choices=IDENTITIES)
    parser.add_argument('--attempts', type=int, choices=range(1, 11), default=5)
    parser.add_argument('--output', type=Path, help='private local result file, under output/')
    args = parser.parse_args(argv)
    try:
        import serial
        with acquire_backend_lease('stm32', 0):
            report = check_link(lambda: open_exclusive_serial(serial, args.port, BAUD, .05),
                                args.expect_firmware, args.attempts)
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        report = dict(link_verified=False, error=str(error), motor_commands_sent=0,
                      motion_authorized=False, mode_changed=False, reset_requested=False)
    encoded = json.dumps(report, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded)
    print(encoded, end='')
    return 0 if report['link_verified'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
