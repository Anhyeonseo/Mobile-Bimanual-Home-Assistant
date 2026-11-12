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
        "-DACTUATOR_MOBILE_FIRST_ID=7", "-DHOST_MOBILE_BUS_INSPECTION_ONLY=1", "-include", str(declaration),
        "-I", str(ROOT / "tests/fixtures/servo_hal"), "-I", str(board / "Inc"),
        "-I", str(ROOT / "firmware/stm32_actuator/include"),
        str(ROOT / "tests/fixtures/mobile_inspection_app.c"),
        str(board / "Src/mobile_bus_inspection_app.c"),str(board / "Src/device_inventory.c"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


def test_failed_reference_preserves_mobile_results():
    motors=[dict(id=i,reported_id=i,identity_ok=True,model_raw=777,baud_code=0,
                 mode_raw=1,torque_raw=0,position_raw=2048) for i in range(7,11)]
    port=Port([INFO,{"scan":"begin"},dict(id=1,reported_id=-1,identity_ok=False),
               *motors,{"scan":"end","motor_write_commands":0}])
    report=module.probe(port,True)
    assert not report["reference_read_ok"] and not report["all_reads_ok"]
    assert report["motors"]==motors


def wheel_replies():
    return [{'wheels':'begin','sequence':7},
        *[dict(id=i,ok=True,hal_status=0,started_tick_ms=100,ended_tick_ms=102,
               position_raw=4090,speed_raw=0x800a) for i in (7,8,9)],
        {'wheels':'end','motor_write_commands':0}]


def inventory_reply(bus=0,motor=8,tick=100,failed=False):
    import struct
    data=bytearray(56)
    data[:8]=bytes([1,3 if failed else 0,bus,motor,0 if failed else 31,
                   0 if failed else 255,3 if failed else 0,bus+1 if failed else 0])
    struct.pack_into('<I',data,8,tick)
    if not failed:
        struct.pack_into('<H',data,15,777);data[17]=motor
        struct.pack_into('<H',data,37,1671)
        struct.pack_into('<H',data,39,0x8032)
    return {'inventory_hex':data.hex()}


def test_device_inventory_preserves_original_speed_word_and_payload():
    reply=inventory_reply();port=Port([reply])
    row=module.device_inventory(port,INFO|{'device_inventory_supported':True},0,8)
    assert row['complete'] and row['feedback']['speed_raw']==32818
    assert row['raw_payload_hex']==reply['inventory_hex']
    assert port.writes==[b'DEVICE 0 8\n']


def test_old_inspection_image_never_receives_device_command():
    port=Port([])
    with pytest.raises(ValueError,match='capability'):module.device_inventory(port,INFO,1,1)
    assert not port.writes


def test_full_inventory_keeps_right_timeout_without_losing_other_rows():
    info=INFO|{'device_inventory_supported':True}
    replies=[inventory_reply(b,i,failed=(b==1 and i==2)) for b,i in module.EXPECTED_DEVICES]
    report=module.inspect_devices(Port([info,*replies]))
    assert len(report['devices'])==16 and not report['all_reads_ok']
    row=next(x for x in report['devices'] if x['bus']=='right' and x['requested_id']==2)
    assert row['read_error']==3 and row['error_domain']=='right_read_status'
    assert row['feedback'] is None and not row['complete']


def test_device_response_must_match_requested_bus_and_id():
    port=Port([inventory_reply(1,3)])
    with pytest.raises(ValueError):
        module.device_inventory(port,INFO|{'device_inventory_supported':True},0,8)


def test_timed_device_capture_preserves_reset_then_rejects_it():
    import io
    out=io.StringIO()
    port=Port([INFO|{'device_inventory_supported':True},inventory_reply(tick=100),inventory_reply(tick=10)])
    with pytest.raises(ValueError,match='gap or reset'):
        module.inspect_devices(port,[0,8],1,out)
    records=[json.loads(line) for line in out.getvalue().splitlines()]
    assert len(records)==3 and records[-1]['sample_started_tick_ms']==10


def test_wheel_sampling_retains_device_ticks_and_signed_raw_without_commands():
    port=Port(wheel_replies())
    result=module.wheel_sample(port,INFO|{'wheel_samples_supported':True})
    assert port.writes==[b'WHEELS\n']
    assert result['wheels'][0]['speed_raw']==0x800a
    assert result['wheels'][0]['started_tick_ms']==100


def test_old_inspection_image_is_not_sent_new_command():
    port=Port([])
    with pytest.raises(RuntimeError):module.wheel_sample(port,INFO)
    assert not port.writes


@pytest.mark.parametrize('change',[{'ok':False,'speed_raw':-1},{'id':10},
    {'ended_tick_ms':90},{'position_raw':4096},{'speed_raw':65536}])
def test_bad_wheel_sample_does_not_become_a_zero_measurement(change):
    replies=wheel_replies();replies[1].update(change)
    with pytest.raises(ValueError):module.wheel_sample(Port(replies),INFO|{'wheel_samples_supported':True})
