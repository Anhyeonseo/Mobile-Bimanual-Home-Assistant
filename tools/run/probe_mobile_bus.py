#!/usr/bin/env python3
"""Read-only INFO/SCAN for mobile-bus-inspection-v2 or mobile-wheel-setup-v1.

INFO is the default; --scan explicitly reads reference ID 1 and mobile IDs 7..10. No motor writes.
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/so101_arm_bridge"))
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.serial_port import open_exclusive_serial


def receive(port, deadline):
    while time.monotonic() < deadline:
        line = port.readline()
        if line:
            result = json.loads(line.decode("ascii"))
            if not isinstance(result, dict):
                raise ValueError("invalid inspection response")
            return result
    raise RuntimeError("inspection response timeout")


def probe(port, scan=False):
    port.reset_input_buffer()
    if port.write(b"INFO\n") != 5:
        raise RuntimeError("partial INFO transmission")
    info = receive(port, time.monotonic() + 2)
    if (info.get("firmware") not in ("mobile-bus-inspection-v2", "mobile-wheel-setup-v1") or
            info.get("motion_enabled") is not False or
            info.get("host_baud") != 921600 or info.get("servo_baud") != 1000000 or
            info.get("ids") != [7, 8, 9, 10] or info.get("reference_id") != 1):
        raise RuntimeError("unexpected firmware/configuration; SCAN not sent")
    report = {"info": info, "scanned": False, "motor_write_commands": 0}
    if not scan:
        return report
    if port.write(b"SCAN\n") != 5:
        raise RuntimeError("partial SCAN transmission")
    deadline = time.monotonic() + 4
    if receive(port, deadline) != {"scan": "begin"}:
        raise RuntimeError("missing scan begin")
    reference = receive(port, deadline)
    if reference.get("id") != 1:
        raise RuntimeError("reference ID mismatch")
    motors = [receive(port, deadline) for _ in range(4)]
    if [m.get("id") for m in motors] != info["ids"]:
        raise RuntimeError("scan ID sequence mismatch")
    if receive(port, deadline) != {"scan": "end", "motor_write_commands": 0}:
        raise RuntimeError("incomplete scan")
    reference_ok = reference.get("identity_ok") is True and reference.get("reported_id") == 1
    report.update(scanned=True, reference=reference, reference_read_ok=reference_ok,
                  motors=motors, all_reads_ok=reference_ok and all(
        m.get("identity_ok") is True and m.get("reported_id") == m["id"] and
        all(type(m.get(k)) is int and m[k] >= 0 for k in
            ("model_raw", "baud_code", "mode_raw", "torque_raw", "position_raw"))
        for m in motors
    ))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--scan", action="store_true", help="Explicit servo READs; support lift before motor power-on")
    args = parser.parse_args()
    try:
        import serial
        with acquire_backend_lease("stm32", 0):
            with open_exclusive_serial(serial, args.port, 921600, .2) as port:
                result = probe(port, args.scan)
        print(json.dumps(result, indent=2))
        return 0 if result.get("all_reads_ok", True) else 2
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        print(json.dumps({"error": str(error), "motor_write_commands": 0}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
