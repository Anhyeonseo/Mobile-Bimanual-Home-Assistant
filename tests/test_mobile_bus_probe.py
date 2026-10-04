"""Maintenance image: HAL TX gate and host identity/scan handling."""
import importlib.util
import json
from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("mobile_probe", ROOT / "tools/run/probe_mobile_bus.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
INFO = dict(firmware="mobile-bus-inspection-v2", motion_enabled=False,
            host_baud=921600, servo_baud=1000000, reference_id=1, ids=[7,8,9,10])

class Port:
    def __init__(self, replies): self.replies=list(replies); self.writes=[]
    def reset_input_buffer(self): pass
    def write(self, data): self.writes.append(data); return len(data)
    def readline(self): return json.dumps(self.replies.pop(0)).encode()+b"\n"

def test_info_never_scans():
    port=Port([INFO])
    assert not module.probe(port)["scanned"]
    assert port.writes==[b"INFO\n"]

@pytest.mark.parametrize("change", [{"firmware":"old"}, {"motion_enabled":True}, {"ids":[8,9,10,11]}])
def test_wrong_image_never_scans(change):
    port=Port([INFO | change])
    with pytest.raises(RuntimeError): module.probe(port,True)
    assert port.writes==[b"INFO\n"]

@pytest.mark.parametrize("firmware", ["mobile-bus-inspection-v2", "mobile-wheel-setup-v1"])
@pytest.mark.parametrize("failed", [False,True])
def test_scan_reports_unread_motor_without_success_claim(failed, firmware):
    motors=[dict(id=i,reported_id=i,identity_ok=True,model_raw=777,baud_code=0,
                 mode_raw=1,torque_raw=0,position_raw=2048) for i in range(7,11)]
    if failed: motors[-1].update(identity_ok=False,mode_raw=-1)
    port=Port([INFO | dict(firmware=firmware),{"scan":"begin"},dict(id=1,reported_id=1,identity_ok=True),*motors,{"scan":"end","motor_write_commands":0}])
    assert module.probe(port,True)["all_reads_ok"] is not failed
    assert port.writes==[b"INFO\n",b"SCAN\n"]

def test_transport_rejects_writes_and_unapproved_reads(tmp_path):
    board=ROOT/"firmware/stm32_g474_single_arm/Core"
    binary=tmp_path/"gate"
    subprocess.run(["gcc","-std=c11","-Wall","-Wextra","-Werror",
        "-DHOST_MOBILE_BUS_INSPECTION_ONLY=1","-DACTUATOR_MOBILE_FIRST_ID=7",
        "-I",str(ROOT/"tests/fixtures/servo_hal"),"-I",str(board/"Inc"),
        "-I",str(ROOT/"firmware/stm32_actuator/include"),
        str(ROOT/"tests/fixtures/mobile_inspection_gate.c"),str(board/"Src/servo_transport.c"),
        str(ROOT/"firmware/stm32_actuator/src/bus_router.c"),
        str(ROOT/"firmware/stm32_actuator/src/shared_bus.c"),"-o",str(binary)],check=True)
    subprocess.run([str(binary)],check=True)


def test_firmware_parser_never_reads_on_boot_or_invalid_commands(tmp_path):
    board = ROOT / "firmware/stm32_g474_single_arm/Core"
    declaration = tmp_path / "receive.h"
    declaration.write_text('#include "stm32g4xx_hal.h"\nHAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);\n')
    binary = tmp_path / "app"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-DACTUATOR_MOBILE_FIRST_ID=7", "-include", str(declaration),
        "-I", str(ROOT / "tests/fixtures/servo_hal"), "-I", str(board / "Inc"),
        "-I", str(ROOT / "firmware/stm32_actuator/include"),
        str(ROOT / "tests/fixtures/mobile_inspection_app.c"),
        str(board / "Src/mobile_bus_inspection_app.c"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


def test_failed_reference_preserves_mobile_results():
    motors=[dict(id=i,reported_id=i,identity_ok=True,model_raw=777,baud_code=0,
                 mode_raw=1,torque_raw=0,position_raw=2048) for i in range(7,11)]
    port=Port([INFO,{"scan":"begin"},dict(id=1,reported_id=-1,identity_ok=False),
               *motors,{"scan":"end","motor_write_commands":0}])
    report=module.probe(port,True)
    assert not report["reference_read_ok"] and not report["all_reads_ok"]
    assert report["motors"]==motors
