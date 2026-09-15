#!/usr/bin/env python3
"""One-command PC release checks. Never opens a device or flashes firmware.

Core checks require Python host requirements, GCC and CMake. --stm32 also needs
the ARM toolchain; --ros needs a sourced ROS Jazzy workspace and nav2_msgs.
Records source/config hashes and installed dependency versions, including dirty
and untracked sources. A passing report is a software candidate, not hardware
qualification. Only explicitly requested build directories are generated.
"""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "output/offline-verification"
    )
    parser.add_argument("--stm32", action="store_true")
    parser.add_argument("--ros", action="store_true")
    parser.add_argument("--bundle", action="store_true")
    parser.add_argument(
        "--moveit",
        action="store_true",
        help="Real MoveIt planning in an isolated local ROS namespace",
    )
    parser.add_argument(
        "--nav2",
        action="store_true",
        help="Actual Nav2/AMCL with the native C synthetic wheel plant",
    )
    parser.add_argument("--soak-seconds", type=int, default=3600)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    # Host tests use pytest's built-ins only. Sourced ROS environments expose
    # unrelated launch_testing plugins whose dependencies are not host inputs.
    env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    env["PYTHONPATH"] = os.pathsep.join(
        [
            str(ROOT / "ros2_ws/src/home_robot_tasks"),
            str(ROOT / "ros2_ws/src/so101_arm_bridge"),
            env.get("PYTHONPATH", ""),
        ]
    )
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env.setdefault("ROS_DOMAIN_ID", "173")
    files = (
        subprocess.check_output(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
            cwd=ROOT,
        )
        .decode()
        .split("\0")
    )
    hashes = {
        name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
        for name in sorted(set(files))
        if name and (ROOT / name).is_file()
    }
    report = {
        "mode": "simulation",
        "hardware_commands": 0,
        "physical_task_completed": False,
        "platform": platform.platform(),
        "python": sys.version,
        "git_head": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "source_sha256": hashes,
        "dependencies": {},
        "checks": [],
        "passed": False,
    }
    for name in ("pytest", "PyYAML", "pyserial", "numpy", "urdf-parser-py"):
        try:
            report["dependencies"][name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            report["dependencies"][name] = None

    def run(name, command, timeout=600):
        started = time.monotonic()
        log = out / f"{name}.log"
        print(f"Checking {name}", flush=True)
        with log.open("w") as stream:
            result = subprocess.run(
                command,
                cwd=ROOT,
                env=env,
                stdout=stream,
                stderr=subprocess.STDOUT,
                timeout=timeout,
            )
        report["checks"].append(
            {
                "name": name,
                "command": command,
                "exit_code": result.returncode,
                "wall_s": round(time.monotonic() - started, 3),
                "log": log.name,
            }
        )
        if result.returncode:
            raise RuntimeError(f"{name} failed: {log}")

    try:
        run(
            "python",
            [
                sys.executable,
                "-m",
                "pytest",
                "-c",
                "config/pytest.ini",
                "--rootdir=.",
                "-q",
            ],
        )
        for name, script in [
            ("protocol", "tools/run/validate_protocol_manifest.py"),
            ("protocol_header", "tools/setup/firmware/generate_protocol_header.py"),
            ("joint_limits", "tools/setup/firmware/generate_joint_limits.py"),
        ]:
            run(
                name,
                [sys.executable, script] + ([] if name == "protocol" else ["--check"]),
            )
        build = ROOT / "build/offline-core"
        run(
            "core_configure",
            ["cmake", "-S", "firmware/stm32_actuator", "-B", str(build)],
        )
        run("core_build", ["cmake", "--build", str(build), "--parallel", "2"])
        run("core_tests", ["ctest", "--test-dir", str(build), "--output-on-failure"])
        run(
            "mobile_soak",
            [
                sys.executable,
                "tools/run/soak_mobile_core.py",
                "--library",
                str(build / "libactuator_mobile_simulator.so"),
                "--duration-s",
                str(args.soak_seconds),
            ],
        )
        for kind, request in [
            ("fetch", "fetch_remote.example.json"),
            ("navigation", "navigate_to.simulation.json"),
        ]:
            run(
                kind,
                [
                    sys.executable,
                    "-m",
                    "home_robot_tasks.simulate_cli",
                    "--world",
                    "config/home.example.json",
                    "--map",
                    "config/navigation_map.simulation.json",
                    "--request",
                    f"config/{request}",
                ],
            )
            result = json.loads((out / f"{kind}.log").read_text())
            assert (
                result["status"] == "SUCCEEDED"
                and result["hardware_commands"] == 0
                and not result["physical_task_completed"]
            )
        for scenario, expected in [
            ("alternate_found", "FOUND"),
            ("hidden", "NOT_FOUND_UNCONFIRMED"),
            ("invalid_depth", "NOT_FOUND_UNCONFIRMED"),
            ("stale_transform", "NOT_FOUND_UNCONFIRMED"),
            ("cancel_pending", "CANCELLED"),
        ]:
            name = "search_" + scenario
            run(
                name,
                [
                    sys.executable,
                    "-m",
                    "home_robot_tasks.search_simulator",
                    "--config",
                    "config/search_sofa.simulation.json",
                    "--scenario",
                    scenario,
                ],
            )
            result = json.loads((out / f"{name}.log").read_text())
            assert result["state"] == expected and result["hardware_commands"] == 0
            assert (
                not result["absence_proven"] and not result["physical_task_completed"]
            )
        if args.stm32:
            for profile in ("legacy", "resident"):
                build = ROOT / f"build/offline-{profile}"
                run(
                    f"{profile}_configure",
                    [
                        "cmake",
                        "-S",
                        "firmware/stm32_g474_single_arm",
                        "-B",
                        str(build),
                        "-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake",
                        "-DCMAKE_BUILD_TYPE=Release",
                        "-DBIMANUAL_GRIPPER_TERMINAL_SETTLE_CANDIDATE="
                        + ("ON" if profile == "resident" else "OFF"),
                    ],
                )
                run(
                    f"{profile}_build",
                    ["cmake", "--build", str(build), "--parallel", "2"],
                )
        if args.ros:
            run("mobile_model", [sys.executable, "tools/run/check_mobile_model.py"])
            run(
                "nav2_action_port",
                [sys.executable, "tools/run/check_nav2_action_port.py"],
            )
        if args.moveit:
            run(
                "moveit_planning",
                [
                    sys.executable,
                    "tools/run/check_moveit_planning.py",
                    "--output",
                    str(out / "moveit"),
                    "--retain-moveit-plugin",
                ],
            )
        if args.nav2:
            for fault in ("none", "scan_loss", "tf_loss"):
                run(
                    "nav2_" + fault,
                    [
                        sys.executable,
                        "tools/run/check_nav2_closed_loop.py",
                        "--library",
                        str(ROOT / "build/offline-core/libactuator_host_simulator.so"),
                        "--fault",
                        fault,
                        "--output",
                        str(out / ("nav2_" + fault)),
                    ],
                )
        # Fail if a concurrent source edit invalidated the checked candidate.
        current_files = (
            subprocess.check_output(
                ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
                cwd=ROOT,
            )
            .decode()
            .split("\0")
        )
        assert {
            name for name in current_files if name and (ROOT / name).is_file()
        } == set(hashes), "source file list changed during verification"
        assert all(
            (ROOT / name).is_file()
            and hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest
            for name, digest in hashes.items()
        ), "source changed during verification"
        report["passed"] = True
        if args.bundle:
            with zipfile.ZipFile(
                out / "offline-source-candidate.zip", "w", zipfile.ZIP_DEFLATED
            ) as archive:
                for name in hashes:
                    archive.write(ROOT / name, name)
                archive.writestr("OFFLINE_CANDIDATE.json", json.dumps(report, indent=2))
    except Exception as error:
        report["error"] = str(error)
    (out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"passed": report["passed"], "report": str(out / "report.json")}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
