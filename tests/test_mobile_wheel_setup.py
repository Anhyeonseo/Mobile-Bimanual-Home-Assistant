"""Wheel EEPROM setup with torque off, failure cleanup, and independent TX limits."""
import importlib.util
import json
from pathlib import Path
import subprocess
import pytest
ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/"firmware/stm32_g474_single_arm/Core"
CORE=ROOT/"firmware/stm32_actuator"

@pytest.fixture(scope="module")
def binaries(tmp_path_factory):
    out=tmp_path_factory.mktemp("wheel-setup")
    decl=out/"delay.h";decl.write_text('#include "stm32g4xx_hal.h"\nvoid HAL_Delay(uint32_t);\n')
    common=["gcc","-O2","-std=c11","-Wall","-Wextra","-Werror","-DACTUATOR_MOBILE_FIRST_ID=7",
        "-DHOST_MOBILE_WHEEL_SETUP_ONLY=1","-I",str(ROOT/"tests/fixtures/servo_hal"),
        "-I",str(BOARD/"Inc"),"-I",str(CORE/"include")]
    subprocess.run([*common,"-include",str(decl),str(ROOT/"tests/fixtures/wheel_setup_harness.c"),
        str(BOARD/"Src/mobile_wheel_setup.c"),"-o",str(out/"setup")],check=True)
    subprocess.run([*common,str(ROOT/"tests/fixtures/wheel_setup_gate.c"),str(BOARD/"Src/servo_transport.c"),
        str(CORE/"src/bus_router.c"),str(CORE/"src/shared_bus.c"),"-o",str(out/"gate")],check=True)
    return out

@pytest.mark.parametrize("scenario",range(6))
def test_setup_readback_failures_and_repeat_guard(binaries,scenario):
    subprocess.run([str(binaries/"setup"),str(scenario)],check=True)

def test_no_nonzero_speed_or_torque_enable_packets(binaries):
    subprocess.run([str(binaries/"gate")],check=True)

spec=importlib.util.spec_from_file_location("setup_wheels",ROOT/"tools/run/prepare_mobile_wheels.py")
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
INFO=dict(firmware="mobile-wheel-setup-v1",motion_enabled=False,ids=[7,8,9,10],host_baud=921600,servo_baud=1000000)
class Port:
    def __init__(self,replies): self.replies=list(replies);self.writes=[]
    def reset_input_buffer(self): pass
    def write(self,data): self.writes.append(data);return len(data)
    def readline(self): return json.dumps(self.replies.pop(0)).encode()+b"\n"

def test_default_only_queries():
    p=Port([INFO]);assert not module.prepare(p)["setup_requested"];assert p.writes==[b"INFO\n"]

def test_wrong_firmware_blocks_setup():
    p=Port([INFO|dict(firmware="mobile-bus-inspection-v2")])
    with pytest.raises(RuntimeError): module.prepare(p,True)
    assert p.writes==[b"INFO\n"]

@pytest.mark.parametrize("ok",[True,False])
def test_setup_request_is_once_and_failure_is_visible(ok):
    result=dict(setup_ok=ok,motion_enabled=False,wheels=[dict(id=i,ok=ok,mode=1 if ok else 0,torque=0,lock=1) for i in (7,8,9)])
    p=Port([INFO,result]);assert module.prepare(p,True)["result"]["setup_ok"] is ok
    assert p.writes==[b"INFO\n",b"PREPARE_WHEELS\n"]
