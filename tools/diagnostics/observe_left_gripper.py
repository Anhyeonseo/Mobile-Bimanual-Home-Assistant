#!/usr/bin/env python3
"""Record raw left ID6 telemetry for commissioning; no motor writes or inferred grip success."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/so101_arm_bridge'))
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.observer_protocol import Reader
from so101_arm_bridge.serial_port import open_exclusive_serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seconds', type=float, default=2.)
    parser.add_argument('--label', choices=('idle', 'empty_close', 'target_contact', 'other_contact'), default='idle')
    args = parser.parse_args()
    if not 0 < args.seconds <= 10:
        parser.error('expected a duration in (0, 10] seconds')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(schema_version=1, operator_label=args.label, motor_write_commands=0,
                  contact_verified=False, records=[], status='failed')
    with args.output.open('x') as dest:
        try:
            import serial
            with acquire_backend_lease('stm32', 0), open_exclusive_serial(serial, args.port, 921600, .05) as port:
                port.reset_input_buffer()
                reader = Reader(port)
                report['binding'] = reader.bind()
                deadline = time.monotonic() + args.seconds
                while not report['records'] or time.monotonic() < deadline:
                    report['records'].append(reader.sample(left_gripper_only=True))
                    time.sleep(.02)
                report['status'] = 'telemetry_recorded'
        except (ImportError, OSError, ValueError, RuntimeError) as exc:
            report['error'] = str(exc)
        finally:
            json.dump(report, dest, indent=2, allow_nan=False)
            dest.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'records'}, ensure_ascii=False))
    return 0 if report['status'] == 'telemetry_recorded' else 2


if __name__ == '__main__':
    raise SystemExit(main())
