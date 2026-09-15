"""Real reader + writer + transport/core; only HAL and circular DMA source are fake."""

from pathlib import Path
import subprocess
import pytest
from so101_arm_bridge.mobile_wire import MobileStatus

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def feedback_runtime(tmp_path_factory):
    binary = tmp_path_factory.mktemp("mobile-feedback") / "reader"
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
            str(fake / "mobile_feedback_harness.c"),
            *(
                str(board / "Src" / (n + ".c"))
                for n in (
                    "mobile_servo_feedback",
                    "mobile_servo_output",
                    "servo_transport",
                )
            ),
            *(
                str(core / "src" / (n + ".c"))
                for n in (
                    "mobile_feedback",
                    "sts3215_response",
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
                    "mobile_endpoint",
                    "mobile_wire",
                    
                )
            ),
            "-o",
            str(binary),
        ],
        check=True,
    )
    return binary


@pytest.mark.parametrize("scenario", [*range(19), *range(20, 25)])
def test_mobile_feedback_runtime(feedback_runtime, scenario):
    subprocess.run([str(feedback_runtime), str(scenario)], check=True)


def test_device_replies_reach_ros_mobile_status(feedback_runtime):
    raw = subprocess.check_output([str(feedback_runtime), "19"], text=True)
    status = MobileStatus.decode(bytes.fromhex(raw.strip()))
    assert status.boot_id == 77 and status.kind == 5
    assert status.velocity_raw == (0, 0, 0, 0)
    assert status.lift_position_um == 12000
    assert status.flags == 31 and 0 < status.feedback_tick_ms < status.mcu_tick_ms
    assert status.feedback_is_fresh()
