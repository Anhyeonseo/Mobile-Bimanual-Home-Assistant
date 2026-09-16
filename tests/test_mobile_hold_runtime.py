from pathlib import Path
import subprocess
import pytest
ROOT = Path(__file__).resolve().parents[1]

@pytest.fixture(scope="module")
def writer(tmp_path_factory):
    binary = tmp_path_factory.mktemp("hold-writer") / "writer"
    fake = ROOT / "tests/fixtures/servo_hal"
    board = ROOT / "firmware/stm32_g474_single_arm/Core"
    core = ROOT / "firmware/stm32_actuator"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-I", str(fake), "-I", str(board / "Inc"), "-I", str(core / "include"),
        str(fake / "mobile_hold_harness.c"), str(board / "Src/mobile_hold_output.c"),
        str(board / "Src/servo_transport.c"),
        *(str(core / "src" / f"{name}.c") for name in ("bus_router", "shared_bus", "motor_groups", "sts3215_packet")),
        "-o", str(binary)], check=True)
    return binary

@pytest.mark.parametrize("scenario", range(7))
def test_stop_only_writer(writer, scenario):
    subprocess.run([str(writer), str(scenario)], check=True)
