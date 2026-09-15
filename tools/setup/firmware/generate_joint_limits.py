#!/usr/bin/env python3
"""Generate firmware table and ROS package copy from one calibrated manifest.

Defaults to verification. --write updates generated regions, never calibration
or authorization metadata. URDF geometric joint zeros need separate calibration.
"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "ros2_ws/src/so101_arm_bridge"))
from so101_arm_bridge.limit_manifest import DIRECTIONS, read_limit_manifest  # noqa: E402

BEGIN = "/* BEGIN GENERATED OPERATIONAL LIMITS */"
END = "/* END GENERATED OPERATIONAL LIMITS */"


def render_table(entries: tuple[dict, ...], sha256: str) -> str:
    lines = [BEGIN, f"/* config/bimanual_operational_limits.json SHA256: {sha256} */",
             "static const BimanualOperationalLimit operational_limits",
             "    [BIMANUAL_ARM_COUNT][BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT / 2U] = {"]
    for arm_index, arm in enumerate(("LEFT", "RIGHT")):
        lines.append(f"    [BIMANUAL_ARM_{arm}] = {{")
        for index, direction in enumerate(DIRECTIONS):
            e = entries[arm_index * 6 + index]
            lines.append("        {2048U, %2d, {%4d, %4d}, {%8d, %7d}}," % (
                direction, e["minimum_unwrapped_raw"], e["maximum_unwrapped_raw"],
                e["minimum_urad"], e["maximum_urad"]))
        lines.append("    },")
    lines.extend(["};", END])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--write", action="store_true")
    args = parser.parse_args()
    manifest = ROOT / "config/bimanual_operational_limits.json"
    try:
        _, entries = read_limit_manifest(manifest)
        canonical = manifest.read_bytes()
        source = ROOT / "firmware/stm32_g474_single_arm/Core/Src/bimanual_operational_limits.c"
        original = source.read_text()
        if original.count(BEGIN) != 1 or original.count(END) != 1:
            raise ValueError("expected one generated table region")
        start, end = original.index(BEGIN), original.index(END) + len(END)
        if start >= end:
            raise ValueError("invalid generated region")
        generated = original[:start] + render_table(entries, hashlib.sha256(canonical).hexdigest()) + original[end:]
        installed = ROOT / "ros2_ws/src/so101_arm_bridge/config/bimanual_operational_limits.json"
        outputs = ((source, generated.encode()), (installed, canonical))
        changed = [path for path, expected in outputs if not path.exists() or path.read_bytes() != expected]
        if args.write:
            for path, expected in outputs:
                if path in changed:
                    path.write_bytes(expected)
        elif changed:
            for path in changed:
                print(f"out of date: {path.relative_to(ROOT)}", file=sys.stderr)
            return 1
        print("Joint limits: 12 calibrated ranges; firmware and ROS copy match" + (" (updated)" if args.write else ""))
        return 0
    except (OSError, ValueError) as error:
        print(f"Joint limits rejected: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
