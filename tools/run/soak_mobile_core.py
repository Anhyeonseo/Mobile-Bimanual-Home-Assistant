#!/usr/bin/env python3
"""Accelerated, ideal-feedback C/Python soak. No device or real-time claim."""
import argparse
import json
from pathlib import Path
import time
from so101_arm_bridge.mobile_client import MobileClient
from so101_arm_bridge.mobile_simulator import NativeMobileSimulator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--duration-s", type=int, default=3600)
    args = parser.parse_args()
    if not 1 <= args.duration_s <= 86400:
        parser.error("duration must be 1..86400 simulated seconds")
    clock = [0xFFFFF000]
    started = time.monotonic()
    port = NativeMobileSimulator(args.library, lambda: clock[0], boot_id=42)
    client = MobileClient(port, lambda: clock[0])
    client.synchronize()
    client.arm(1)
    initial_tick = clock[0]
    for i in range(args.duration_s * 100):
        clock[0] = initial_tick + (i + 1) * 10
        values = ((i % 200) - 100, 50, -50, 0)
        client.velocity(values)
        if i % 100 == 0:
            # A query at the command's sample timestamp cannot turn a command
            # echo into new feedback. Advance to a distinct plant sample.
            clock[0] += 1
            status = client.status()
            assert status.velocity_raw == values and status.state == 2
    clock[0] += 10
    client.stop()
    clock[0] += 10
    assert client.status().flags & 8
    print(
        json.dumps(
            {
                "mode": "simulation",
                "simulated_duration_s": args.duration_s,
                "wall_duration_s": round(time.monotonic() - started, 3),
                "velocity_commands": args.duration_s * 100,
                "simulated_bus_transmissions": port.library.pc_mobile_transmissions(),
                "hardware_commands": 0,
                "physical_stop_proven": False,
                "passed": True,
            }
        )
    )


if __name__ == "__main__":
    main()
