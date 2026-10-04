#!/usr/bin/env python3
"""Inspect an explicitly selected STM32 host port; never move or configure motors.

Run on the Pi. Requires protocol-v2 firmware, matching host baud and pyserial.
Enters binary host protocol mode; reset the MCU to return to its text console.
Does not claim that an available endpoint means the hardware is commissioned.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/so101_arm_bridge"))
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.mobile_inspection import inspect_mobile
from so101_arm_bridge.serial_port import open_exclusive_serial
from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", required=True,
        help="STM32 host port, NOT a direct Waveshare servo port",
    )
    parser.add_argument("--baud", required=True, type=int, choices=(115200, 921600))
    args = parser.parse_args()
    try:
        import serial
        with acquire_backend_lease("stm32", 0):
            with open_exclusive_serial(serial, args.port, args.baud, .2) as port:
                report = inspect_mobile(
                    StreamValidationTransportV2(port, response_timeout_s=.8)
                )
        print(json.dumps(report, indent=2))
        return 0 if report["mobile_available"] else 2
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        print(json.dumps(dict(
            error=str(error), motion_authorized=False, motor_commands_sent=0,
        )), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
