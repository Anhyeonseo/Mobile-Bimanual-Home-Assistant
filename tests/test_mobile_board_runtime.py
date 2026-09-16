"""Actual board initialization, stop writers and measured hold integration."""
from pathlib import Path
import subprocess
import pytest
from so101_arm_bridge.system_stop import SystemStopStatus
ROOT = Path(__file__).resolve().parents[1]

@pytest.fixture(scope="module")
def board(tmp_path_factory):
    binary = tmp_path_factory.mktemp("mobile-board") / "board"
    fake = ROOT / "tests/fixtures/servo_hal"
    board = ROOT / "firmware/stm32_g474_single_arm/Core"
    core = ROOT / "firmware/stm32_actuator"
    subprocess.run([
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-DHOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1U", "-DHOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1U",
        "-DHOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD=1U", "-DHOST_BIMANUAL_DMA_DISPATCH_BUILD=1U",
        "-I", str(fake), "-I", str(board / "Inc"), "-I", str(core / "include"),
        str(fake / "mobile_board_harness.c"),
        *(str(board / "Src" / f"{name}.c") for name in (
            "mobile_board", "mobile_hold_output", "mobile_servo_output", "servo_transport",
            "mobile_arm_observer", "bimanual_feedback_snapshot", "bimanual_operational_limits")),
        *(str(core / "src" / f"{name}.c") for name in (
            "mobile_output", "mobile_supervisor", "mobile_feedback", "sts3215_response",
            "mobile_endpoint", "mobile_wire", "device_startup", "lift_endpoint", "lift_controller",
            "system_stop", "robot_evidence", "arm_hold_monitor", "crc32c", "bus_schedule", "bus_router", "shared_bus",
            "motor_groups", "sts3215_packet", "joint_unwrap", "bimanual_goal_map")), "-o", str(binary)], check=True)
    return binary

@pytest.mark.parametrize("scenario", range(11))
def test_board_stop_and_startup(board, scenario):
    result = subprocess.run([str(board), str(scenario)], check=True, text=True, capture_output=True)
    if scenario == 0:
        status = SystemStopStatus.decode(bytes.fromhex(result.stdout.strip()))
        assert status.confirmed and status.boot_id == 42 and status.sequence == 7
