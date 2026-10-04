"""Bounded raised-wheel pulse, ambiguous writes, stop evidence, and TX whitelist."""
import importlib.util
import json
from pathlib import Path
import subprocess
import pytest
ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT / "firmware/stm32_g474_single_arm/Core"
CORE = ROOT / "firmware/stm32_actuator"


@pytest.fixture(scope="module")
def binaries(tmp_path_factory):
    out = tmp_path_factory.mktemp("wheel-pulse")
    decl = out / "hal_extra.h"
    decl.write_text('#include "stm32g4xx_hal.h"\nvoid HAL_Delay(uint32_t);\n'
        'HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);\n')
    common = ["gcc", "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-DACTUATOR_MOBILE_FIRST_ID=7", "-DHOST_MOBILE_WHEEL_PULSE_ONLY=1",
        "-include", str(decl), "-I", str(ROOT / "tests/fixtures/servo_hal"),
        "-I", str(BOARD / "Inc"), "-I", str(CORE / "include")]
    for name, sources in {
        "base": [ROOT / "tests/fixtures/base_pulse_harness.c", BOARD / "Src/mobile_wheel_pulse.c"],
        "pulse": [ROOT / "tests/fixtures/wheel_pulse_harness.c", BOARD / "Src/mobile_wheel_pulse.c"],
        "gate": [ROOT / "tests/fixtures/wheel_pulse_gate.c", BOARD / "Src/servo_transport.c",
                 CORE / "src/bus_router.c", CORE / "src/shared_bus.c"],
        "app": [ROOT / "tests/fixtures/wheel_pulse_app.c", BOARD / "Src/mobile_bus_inspection_app.c"],
    }.items():
        subprocess.run([*common, *map(str, sources), "-o", str(out / name)], check=True)
    return out


@pytest.mark.parametrize("scenario", range(14))
def test_firmware_pulse_and_fault_cleanup(binaries, scenario):
    subprocess.run([str(binaries / "pulse"), str(scenario)], check=True)


@pytest.mark.parametrize("binary", ["gate", "app"])
def test_transport_whitelist_and_command_admission(binaries, binary):
    subprocess.run([str(binaries / binary)], check=True)


spec = importlib.util.spec_from_file_location("pulse_wheel", ROOT / "tools/run/pulse_mobile_wheel.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
INFO = dict(firmware="mobile-wheel-pulse-v4", pulse_supported=True, pulse_consumed=False,
    motion_enabled=False, ids=[7, 8, 9, 10], host_baud=921600, servo_baud=1000000,
    speed_raw=200, duration_ms=3000)
RESULT = dict(pulse_ok=True, id=7, stop_confirmed=True, motion_attempted=True,
    cleanup_attempted=True, stage="PULSE_COMPLETE_TORQUE_OFF", speed_raw=200,
    goal_after=0, torque_after=0, speed_after=0, position_before=1000, position_after=1100)


class Port:
    def __init__(self, replies):
        self.replies = list(replies)
        self.writes = []
    def reset_input_buffer(self): pass
    def write(self, data):
        self.writes.append(data)
        return len(data)
    def readline(self):
        return json.dumps(self.replies.pop(0)).encode() + b"\n"


def test_info_is_default():
    port = Port([INFO])
    assert not module.pulse(port)["pulse_requested"]
    assert port.writes == [b"INFO\n"]


@pytest.mark.parametrize("change", [dict(firmware="mobile-wheel-setup-v1"),
    dict(firmware="mobile-wheel-pulse-v1"), dict(firmware="mobile-wheel-pulse-v2"), dict(firmware="mobile-wheel-pulse-v3"), dict(pulse_consumed=True), dict(ids=[8, 9, 10, 11]), dict(duration_ms=1000)])
def test_wrong_or_consumed_firmware_never_moves(change):
    port = Port([INFO | change])
    with pytest.raises(RuntimeError): module.pulse(port, 7)
    assert port.writes == [b"INFO\n"]


@pytest.mark.parametrize("direction", ["positive", "negative"])
def test_single_request_and_stop_evidence(direction):
    sign = 1 if direction == "positive" else -1
    port = Port([INFO, RESULT | dict(speed_raw=200 * sign)])
    assert module.pulse(port, 7, direction)["result"]["pulse_ok"]
    assert port.writes == [b"INFO\n", f"PULSE 7 {'+' if sign > 0 else '-'}\n".encode()]


@pytest.mark.parametrize("change", [dict(torque_after=1), dict(speed_after=50),
    dict(stop_confirmed=False), dict(position_after=-1)])
def test_inconsistent_success_is_rejected_without_retry(change):
    port = Port([INFO, RESULT | change])
    with pytest.raises(RuntimeError): module.pulse(port, 7)
    assert len(port.writes) == 2


def test_stop_failure_is_explicit():
    port = Port([INFO, RESULT | dict(pulse_ok=False, stop_confirmed=False)])
    assert "Cut motor" in module.pulse(port, 7)["next_action"]
    assert len(port.writes) == 2


@pytest.mark.parametrize("scenario", range(9))
@pytest.mark.parametrize("motion", ["F", "B", "L", "R", "A", "D"])
def test_base_group_cleanup_and_shared_latch(binaries, scenario, motion):
    completed = subprocess.run([str(binaries / "base"), str(scenario), motion], check=True, capture_output=True, text=True)
    if scenario in (0, 8):
        expected = {"F": [-200, 200, 0], "B": [200, -200, 0], "L": [-100, -100, 200],
                    "R": [100, 100, -200], "A": [200, 200, 200], "D": [-200, -200, -200]}
        assert list(map(int, completed.stdout.split())) == expected[motion]
