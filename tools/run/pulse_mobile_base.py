#!/usr/bin/env python3
"""Raised-wheel base combinations, fixed 3-second pulse, once per MCU boot.

Nominal 120-degree layout: 7 left, 8 right, 9 front. All +raw turn clockwise
viewed from outside. No ground driving, calibration, EEPROM or lift writes.
"""
import argparse
import json
import sys
from pulse_mobile_wheel import pulse, receive, acquire_backend_lease, open_exclusive_serial

# Wheel order 7/8/9. Patterns are candidate ratios, not measured linear speeds.
PATTERNS = {
    "forward": ("F", [-200, 200, 0]),
    "backward": ("B", [200, -200, 0]),
    "left": ("L", [-100, -100, 200]),
    "right": ("R", [100, 100, -200]),
    "rotate-left": ("A", [200, 200, 200]),
    "rotate-right": ("D", [-200, -200, -200]),
}


def run(port, motion=None):
    if motion is not None and motion not in PATTERNS:
        raise ValueError("unknown motion")
    report = pulse(port)  # INFO only, including strict image/baud/ID checks
    info = report["info"]
    if info.get("base_pulse_supported") is not True:
        raise RuntimeError("base pulse unsupported; no command sent")
    if motion is None:
        return report
    if info["pulse_consumed"]:
        raise RuntimeError("pulse already consumed; cut motor power before MCU reset")
    letter, expected = PATTERNS[motion]
    command = f"BASE {letter}\n".encode("ascii")
    if port.write(command) != len(command):
        raise RuntimeError("partial base command; do not retry")
    result = receive(port, 8)
    if (result.get("base_result") != "begin" or result.get("motion") != letter or
            type(result.get("pulse_ok")) is not bool or
            type(result.get("stop_confirmed")) is not bool):
        raise RuntimeError("invalid base response")
    wheels = [receive(port, 2) for _ in range(3)]
    if receive(port, 2) != {"base_result": "end"}:
        raise RuntimeError("missing base result end")
    for i, wheel in enumerate(wheels):
        if (wheel.get("id") != i + 7 or wheel.get("command_raw") != expected[i] or
                type(wheel.get("stop_confirmed")) is not bool):
            raise RuntimeError("invalid wheel mapping/readback")
        if wheel["stop_confirmed"] and not all(
                type(wheel.get(k)) is int and wheel[k] == 0
                for k in ("goal_after", "torque_after", "speed_after")):
            raise RuntimeError("inconsistent wheel stop evidence")
    if result["stop_confirmed"] != all(w["stop_confirmed"] for w in wheels):
        raise RuntimeError("inconsistent group stop evidence")
    if result["pulse_ok"] and not (
            result["stop_confirmed"] and result.get("cleanup_attempted") is True and
            result.get("stage") == "BASE_COMPLETE_TORQUE_OFF" and
            all(type(w.get(k)) is int and w[k] >= 0 for w in wheels
                for k in ("position_before", "position_after"))):
        raise RuntimeError("inconsistent base success")
    report.update(pulse_requested=True, motion=motion, result=result, wheels=wheels)
    if not result["stop_confirmed"]:
        report["next_action"] = "Cut motor 12 V power; do not retry or reset with motor power on."
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--motion", choices=tuple(PATTERNS), help="Explicitly run one raised-wheel combination")
    args = parser.parse_args()
    try:
        import serial
        with acquire_backend_lease("stm32", 0):
            with open_exclusive_serial(serial, args.port, 921600, .2) as port:
                report = run(port, args.motion)
        print(json.dumps(report, indent=2))
        return 0 if report.get("result", {}).get("pulse_ok", True) else 2
    except (ImportError, OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        print(json.dumps({"error": str(error) or "interrupted", "stop_confirmed": False,
            "next_action": "If motion was requested, cut motor 12 V power; do not retry."}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
