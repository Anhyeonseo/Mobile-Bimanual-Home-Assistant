"""Run the real periodic board writer/router/gate against deterministic fake HAL."""

from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def writer(tmp_path_factory):
    binary = tmp_path_factory.mktemp("mobile-writer") / "writer"
    fake = ROOT / "tests/fixtures/servo_hal"
    board = ROOT / "firmware/stm32_g474_single_arm/Core"
    core = ROOT / "firmware/stm32_actuator"
    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(fake),
            "-I",
            str(board / "Inc"),
            "-I",
            str(core / "include"),
            str(fake / "mobile_output_harness.c"),
            str(board / "Src/mobile_servo_output.c"),
            str(board / "Src/servo_transport.c"),
            *(
                str(core / "src" / (name + ".c"))
                for name in (
                    "mobile_output",
                    "lift_endpoint",
                    "lift_controller",
                    "crc32c",
                    "mobile_supervisor",
                    "bus_schedule",
                    "bus_router",
                    "shared_bus",
                    "motor_groups",
                    "sts3215_packet",
                )
            ),
            "-o",
            str(binary),
        ],
        check=True,
    )
    return binary


@pytest.mark.parametrize("scenario", range(14))
def test_mobile_board_writer(writer, scenario):
    subprocess.run([str(writer), str(scenario)], check=True)
