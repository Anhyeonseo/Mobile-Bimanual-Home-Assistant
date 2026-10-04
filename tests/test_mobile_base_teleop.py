"""Manual teleop watchdog, terminal leases, CRC/session and cleanup regression."""
import importlib.util
import json
from pathlib import Path
import subprocess
import pytest
ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT / "firmware/stm32_g474_single_arm/Core"
CORE = ROOT / "firmware/stm32_actuator"
spec = importlib.util.spec_from_file_location("teleop_base", ROOT / "tools/run/teleop_mobile_base.py")
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


@pytest.fixture(scope="module")
def binaries(tmp_path_factory):
    out = tmp_path_factory.mktemp("base-teleop")
    decl = out / "hal.h"
    decl.write_text('#include "stm32g4xx_hal.h"\nvoid HAL_Delay(uint32_t);\n'
        'HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);\n')
    common = ["gcc", "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-DACTUATOR_MOBILE_FIRST_ID=7", "-DHOST_MOBILE_BASE_TELEOP_ONLY=1",
        "-include", str(decl), "-I", str(ROOT / "tests/fixtures/servo_hal"),
        "-I", str(BOARD / "Inc"), "-I", str(CORE / "include")]
    subprocess.run([*common, str(ROOT / "tests/fixtures/base_teleop_harness.c"),
        str(BOARD / "Src/mobile_base_teleop_app.c"), "-o", str(out / "app")], check=True)
    subprocess.run([*common, str(ROOT / "tests/fixtures/wheel_pulse_gate.c"),
        str(BOARD / "Src/servo_transport.c"), str(CORE / "src/bus_router.c"),
        str(CORE / "src/shared_bus.c"), "-o", str(out / "gate")], check=True)
    return out


@pytest.mark.parametrize("scenario", range(21))
def test_firmware_watchdog_session_cleanup(binaries, scenario):
    subprocess.run([str(binaries / "app"), str(scenario)], check=True)


def test_teleop_transport_blocks_arm_lift_arbitrary_writes(binaries):
    subprocess.run([str(binaries / "gate")], check=True)


def test_crc_known_vector():
    assert module.frame("123456789") == b"123456789 29B1\n"


@pytest.mark.parametrize("key,expected", module.KEYS.items())
def test_keyboard_map_and_release_timeout(key, expected):
    current, stamp, done = module.command_for(key, 1, "Z", 0)
    assert (current, stamp, done) == (expected, 1, False)
    assert module.command_for("", 1.31, current, stamp)[0] == "Z"
    assert module.command_for(key, 1.2, current, stamp)[0] == expected


def test_space_exit_and_paste_burst():
    assert module.command_for(" ", 2, "F", 1)[0] == "Z"
    for key in ("x", "\x03", "\x1b"):
        assert module.command_for(key, 2, "F", 1)[2]
    with pytest.raises(RuntimeError): module.command_for("w" * 100, 2, "F", 1)


class Port:
    def __init__(self, replies): self.data = bytearray(b"".join(json.dumps(x).encode() + b"\n" for x in replies)); self.writes = []
    def reset_input_buffer(self): pass
    def read(self, n): result = bytes(self.data[:n]); del self.data[:n]; return result
    def write(self, data): self.writes.append(data); return len(data)


def test_info_only_and_wrong_image():
    info = dict(firmware="mobile-base-teleop-v4", host_baud=921600, servo_baud=1000000,
        ids=[7, 8, 9, 10], max_raw=200, watchdog_ms=400)
    port = Port([info]); assert module.inspect(port) == info; assert port.writes == [b"INFO\n"]
    port = Port([info | dict(firmware="mobile-wheel-pulse-v4")])
    with pytest.raises(RuntimeError): module.inspect(port)
    assert port.writes == [b"INFO\n"]


@pytest.mark.parametrize("change", [dict(session=2), dict(sequence=3), dict(armed=False), dict(fault=True)])
def test_invalid_ack(change):
    with pytest.raises(RuntimeError): module.validate(dict(type="ack", session=1, sequence=1, armed=True, fault=False) | change, "ack", 1, 1)


def test_stop_consumes_prior_ack_and_requires_actual_confirmation():
    port = Port([dict(type="ack"), dict(type="stopped", armed=False, stop_confirmed=True)])
    assert module.stop(port)["stop_confirmed"]; assert port.writes == [b"STOP\n"]
    with pytest.raises(RuntimeError): module.stop(Port([dict(type="stopped", armed=False, stop_confirmed=False)]))


@pytest.mark.parametrize("bad_ack", [False, True])
def test_interactive_exit_or_error_always_stops_and_restores_terminal(monkeypatch, bad_ack):
    class InteractivePort(Port):
        def __init__(self): super().__init__([])
        def write(self, data):
            self.writes.append(data)
            if data == b"INFO\n":
                response = dict(firmware="mobile-base-teleop-v4", host_baud=921600,
                    servo_baud=1000000, ids=[7, 8, 9, 10], max_raw=200, watchdog_ms=400)
            elif data.startswith(b"ARM "):
                response = dict(type="armed", session=123, sequence=0, armed=True, fault=False)
            elif data.startswith(b"D "):
                response = dict(type="ack", session=123, sequence=99 if bad_ack else 1, armed=True, fault=False)
            else:
                assert data == b"STOP\n"
                response = dict(type="stopped", armed=False, stop_confirmed=True)
            self.data.extend(json.dumps(response).encode() + b"\n")
            return len(data)
    restored = []
    keys = iter([b"w", b"x"])
    monkeypatch.setattr(module.os, "isatty", lambda fd: True)
    monkeypatch.setattr(module.termios, "tcgetattr", lambda fd: ["saved"])
    monkeypatch.setattr(module.termios, "tcsetattr", lambda fd, mode, value: restored.append(value))
    monkeypatch.setattr(module.tty, "setcbreak", lambda fd: None)
    monkeypatch.setattr(module.select, "select", lambda *args: ([1], [], []))
    monkeypatch.setattr(module.os, "read", lambda *args: next(keys))
    monkeypatch.setattr(module.secrets, "randbelow", lambda n: 122)
    port = InteractivePort()
    if bad_ack:
        with pytest.raises(RuntimeError): module.run(port, 1)
    else: assert module.run(port, 1)["stop_confirmed"]
    assert restored == [["saved"]]
    assert port.writes[-1] == b"STOP\n"
    assert sum(w.startswith(b"D ") for w in port.writes) == 1


def test_noninteractive_run_never_arms(monkeypatch):
    monkeypatch.setattr(module.os, "isatty", lambda fd: False)
    port = Port([])
    with pytest.raises(RuntimeError): module.run(port, 1)
    assert not port.writes


@pytest.mark.parametrize("scenario,register", [(21,46),(22,40),(23,58),(24,56),(26,58)])
def test_transient_reads_require_fresh_quiet_sweeps_and_retain_evidence(binaries, scenario, register):
    result = subprocess.run([str(binaries / "app"), str(scenario)], check=True, capture_output=True, text=True)
    messages = [json.loads(line) for line in result.stdout.splitlines() if line]
    armed, stopped = messages
    assert armed["type"] == "armed" and not armed["fault"]
    assert all(w["quiet_samples"] >= 3 for w in armed["wheels"])
    first = armed["first_observation"]
    assert (first["phase"], first["id"], first["failed_register"], first["hal_status"]) == ("STARTUP",7,register,3)
    assert stopped["first_observation"] == first
    assert stopped["stop_confirmed"]
    assert all(len(line.encode()) < 2048 for line in result.stdout.splitlines())


@pytest.mark.parametrize("scenario", [25,27,28])
def test_startup_failure_snapshot_survives_successful_cleanup(binaries, scenario):
    result = subprocess.run([str(binaries / "app"), str(scenario)], check=True, capture_output=True, text=True)
    fault, stopped = [json.loads(line) for line in result.stdout.splitlines() if line]
    assert fault["fault"] and not fault["armed"] and fault["session"] == 0
    assert fault["stop_confirmed"] and stopped["stop_confirmed"]
    assert fault["first_failure_wait_ms"] >= 1000
    assert stopped["first_failure_wait_ms"] == fault["first_failure_wait_ms"]
    assert all(w["speed_raw"] == 0 for w in fault["wheels"])
    if scenario != 27:
        first = fault["first_observation"]
        assert stopped["first_observation"] == first
        if scenario == 25:
            assert first["failed_register"] == 58 and first["hal_status"] == 3
            assert first["speed_raw"] == -1 and not first["speed_valid"]
        else:
            assert first["failed_register"] == 0 and first["speed_signed"] == -50
    assert all(len(line.encode()) < 2048 for line in result.stdout.splitlines())


@pytest.mark.parametrize("scenario", [29,30,31])
def test_zero_speed_may_enable_torque_and_torque_off_must_be_verified(binaries, scenario):
    subprocess.run([str(binaries / "app"), str(scenario)], check=True)
