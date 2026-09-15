"""Compile actual board transport/dispatcher against a deterministic fake HAL."""

from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT / "firmware/stm32_g474_single_arm/Core"
CORE = ROOT / "firmware/stm32_actuator"


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    binary = tmp_path_factory.mktemp("servo-transport") / "harness"
    fake = ROOT / "tests/fixtures/servo_hal"
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
            str(BOARD / "Inc"),
            "-I",
            str(CORE / "include"),
            str(fake / "transport_harness.c"),
            str(BOARD / "Src/servo_transport.c"),
            str(BOARD / "Src/bimanual_servo_dispatch.c"),
            *(
                str(CORE / "src" / f"{name}.c")
                for name in (
                    "bimanual_dispatch",
                    "motor_groups",
                    "sts3215_packet",
                    "shared_bus",
                    "bus_router",
                )
            ),
            "-o",
            str(binary),
        ],
        check=True,
    )
    return binary


@pytest.mark.parametrize("scenario", range(15))
def test_board_transport_and_dispatch_fault_scenarios(harness, scenario):
    subprocess.run([str(harness), str(scenario)], check=True)


def test_no_direct_servo_writer_bypasses_the_board_gate():
    for name in (
        "servo_bus.c",
        "right_servo_bus.c",
        "bimanual_servo_dispatch.c",
        "mobile_servo_output.c",
        "mobile_servo_feedback.c",
    ):
        text = (BOARD / "Src" / name).read_text()
        for call in (
            "HAL_UART_Transmit(",
            "HAL_UART_Transmit_IT(",
            "HAL_UART_Transmit_DMA(",
        ):
            assert call not in text, (name, call)
        assert "ServoTransport_" in text
