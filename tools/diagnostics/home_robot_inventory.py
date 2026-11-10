#!/usr/bin/env python3
"""Collect passive Linux host metadata for S0; no device ports or drivers opened.

Uses only the standard library. May be streamed over SSH to ``python3 -``.
Detected USB devices do not prove powered motors, firmware, wiring or calibration.
"""

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def read_metadata(path):
    try:
        return {"status": "read", "value": path.read_text().replace("\x00", "").strip()}
    except (OSError, UnicodeError) as error:
        return {"status": "unavailable", "reason": type(error).__name__}


def entries(path):
    try:
        return sorted(path.iterdir()), None
    except OSError as error:
        return [], type(error).__name__


def command(argv):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=5)
        return {"returncode": result.returncode, "stdout": result.stdout.strip(),
                "stderr": result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"status": "unavailable", "reason": type(error).__name__}


def usb_inventory(directory):
    usb_paths, usb_error = entries(directory)
    usb = []
    for path in usb_paths:
        if not (path / "idVendor").is_file():
            continue
        usb.append({"sysfs_name": path.name, **{
            field: read_metadata(path / field) for field in
            ("idVendor", "idProduct", "manufacturer", "product", "serial", "speed")
        }})
    return {"entries": usb, "error": usb_error}


def serial_inventory(root):
    serial_links = {}
    for directory in ("/dev/serial/by-id", "/dev/serial/by-path"):
        paths, error = entries(root / directory.lstrip("/"))
        links = []
        for path in paths:
            try:
                links.append({"name": path.name, "target": os.readlink(path)})
            except OSError as link_error:
                links.append({"name": path.name, "error": type(link_error).__name__})
        serial_links[directory] = {"entries": links, "error": error}
    return serial_links


def package_versions():
    packages = {}
    for name in ("numpy", "torch", "torchvision", "transformers", "pyrealsense2",
                 "opencv-python", "opencv-python-headless", "pyserial"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None

    return packages


def collect(root=Path("/")):
    def at(path):
        return root / path.lstrip("/")

    tty_paths, tty_error = entries(at("/sys/class/tty"))
    tty_names = [p.name for p in tty_paths if p.name.startswith(("ttyUSB", "ttyACM", "ttyTHS"))]
    ros_paths, ros_error = entries(at("/opt/ros"))
    return {
        "schema_version": 1,
        "collected_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "passive_host_metadata_only",
        "hostname": platform.node(), "architecture": platform.machine(),
        "kernel": platform.release(), "python_executable": sys.executable,
        "python_version": platform.python_version(),
        "system": {path: read_metadata(at(path)) for path in (
            "/etc/os-release", "/etc/nv_tegra_release", "/proc/device-tree/model",
            "/proc/meminfo", "/proc/uptime")},
        "ros": {"installation_names": [p.name for p in ros_paths if p.is_dir()],
                "directory_error": ros_error,
                "environment": {key: os.environ.get(key) for key in (
                    "ROS_DISTRO", "ROS_VERSION", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION")}},
        "python_packages_in_this_interpreter": package_versions(),
        "usb": usb_inventory(at("/sys/bus/usb/devices")),
        "serial_links": serial_inventory(root),
        "serial_node_names": {"entries": tty_names, "error": tty_error},
        "verification": {
            "device_ports_opened": False, "camera_stream_started": False,
            "motor_commands_sent": False, "firmware_written": False,
            "installed_motor_ids_verified": False, "active_stm32_firmware": None,
            "physical_configuration_verified": False,
        },
        "limitations": [
            "USB names are observations, not verified device roles or wiring.",
            "Missing metadata is unknown, not proof that hardware is absent.",
            "Package versions describe this interpreter, not other virtual environments.",
            "No driver/model imports, serial protocol probes, sudo or device SDK calls.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = collect()
    report["jetson_packages"] = command([
        "dpkg-query", "-W", "nvidia-l4t-core", "nvidia-jetpack"])
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
