#!/usr/bin/env python3
"""Raised-wheel ID/direction test: fixed raw speed 200 for 3000 ms, once per MCU boot.

Default is INFO only. Keep ALL wheels raised, lift supported, and 12 V cutoff
accessible. MCU reset, bus failure or MCU power loss can prevent firmware stop.
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


def receive(port, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = port.readline()
        if raw:
            value = json.loads(raw.decode("ascii"))
            if not isinstance(value, dict):
                raise ValueError("invalid response")
            return value
    raise RuntimeError("response timeout; never automatically retry the pulse")


def pulse(port, wheel=None, direction="positive"):
    if wheel is not None and (type(wheel) is not int or wheel not in (7, 8, 9)):
        raise ValueError("wheel must be 7, 8 or 9")
    if direction not in ("positive", "negative"):
        raise ValueError("invalid direction")
    port.reset_input_buffer()
    if port.write(b"INFO\n") != 5:
        raise RuntimeError("partial INFO write")
    info = receive(port, 2)
    if (info.get("firmware") != "mobile-wheel-pulse-v4" or
            info.get("ids") != [7, 8, 9, 10] or
            info.get("pulse_supported") is not True or
            info.get("motion_enabled") is not False or
            type(info.get("pulse_consumed")) is not bool or
            info.get("host_baud") != 921600 or info.get("servo_baud") != 1000000 or
            info.get("speed_raw") != 200 or info.get("duration_ms") != 3000):
        raise RuntimeError("unexpected firmware; no pulse command sent")
    report = {"info": info, "pulse_requested": False}
    if wheel is None:
        return report
    if info["pulse_consumed"]:
        raise RuntimeError("pulse already consumed; cut motor power before MCU reset")
    command = f"PULSE {wheel} {'+' if direction == 'positive' else '-'}\n".encode("ascii")
    if port.write(command) != len(command):
        raise RuntimeError("partial pulse write; do not retry")
    result = receive(port, 8)
    if (type(result.get("pulse_ok")) is not bool or result.get("id") != wheel or
            type(result.get("stop_confirmed")) is not bool):
        raise RuntimeError("invalid pulse response")
    if result["pulse_ok"] and not (
            result["stop_confirmed"] is True and result.get("motion_attempted") is True and
            result.get("cleanup_attempted") is True and
            result.get("stage") == "PULSE_COMPLETE_TORQUE_OFF" and
            result.get("speed_raw") == (200 if direction == "positive" else -200) and
            result.get("goal_after") == result.get("torque_after") == result.get("speed_after") == 0 and
            type(result.get("position_before")) is int and result["position_before"] >= 0 and
            type(result.get("position_after")) is int and result["position_after"] >= 0):
        raise RuntimeError("inconsistent pulse/stop readback")
    report.update(pulse_requested=True, result=result)
    if not result["stop_confirmed"]:
        report["next_action"] = "Cut motor 12 V power. Do not retry or reset with motor power on."
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--pulse", action="store_true", help="Perform exactly one raised-wheel pulse")
    parser.add_argument("--wheel", type=int, choices=(7, 8, 9))
    parser.add_argument("--direction", choices=("positive", "negative"), default="positive")
    args = parser.parse_args()
    if args.pulse != (args.wheel is not None):
        parser.error("use --pulse and --wheel together; omitting both is read-only INFO")
    try:
        import serial
        with acquire_backend_lease("stm32", 0):
            with open_exclusive_serial(serial, args.port, 921600, .2) as port:
                report = pulse(port, args.wheel, args.direction)
        print(json.dumps(report, indent=2))
        return 0 if report.get("result", {}).get("pulse_ok", True) else 2
    except (ImportError, OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        print(json.dumps({"error": str(error) or "interrupted", "stop_confirmed": False,
            "next_action": "If a pulse was requested, cut motor 12 V power; do not retry."}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
