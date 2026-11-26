"""Actual fixed C reads, backward compatibility, and no-motion host behavior."""
from dataclasses import make_dataclass
from pathlib import Path
import struct
import subprocess

import pytest

from so101_arm_bridge.device_inventory import decode_inventory
from so101_arm_bridge.servo_tuning import TuningQueryUnsupported, decode_tuning, tuning_request
from so101_arm_bridge.stream_protocol_v2 import FrameV2, StreamMessageTypeV2 as Msg, decode_frame_v2, encode_frame_v2
from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
from tools.diagnostics.inspect_servo_tuning import inspect, main

ROOT = Path(__file__).resolve().parents[1]


def response(bus=0, motor=2):
    data = bytearray(56)
    data[:8] = bytes((2, 0, bus, motor, 7, 255, 0, 0))
    data[12:19] = bytes((3, 10, 0, 9, 3, motor, 0))
    struct.pack_into('<H', data, 19, 1000)
    data[24:27] = bytes((64, 64, 0))
    data[27:29] = bytes((18, 7))
    data[29:31] = bytes((2, 3))
    struct.pack_into('<H', data, 31, 400)
    data[33:35] = bytes((1, 0))
    struct.pack_into('<4H', data, 35, 1032, 0, 800, 900)
    return data


@pytest.mark.parametrize('bus', [0, 1])
@pytest.mark.parametrize('motor', range(1, 7))
def test_raw_settings_and_identity(bus, motor):
    r = decode_tuning(bytes(response(bus, motor)), bus, motor)
    assert tuning_request(bus, motor) == bytes((2, bus, motor, 0))
    assert r['complete'] and r['identity']['model_raw'] == 777
    assert r['tuning'] == dict(maximum_torque_limit_raw=1000, phase_raw=0, p_gain_raw=64, d_gain_raw=64,
        i_gain_raw=0, minimum_startup_force_raw=18, integral_limit_raw=7, cw_dead_zone_raw=2, ccw_dead_zone_raw=3,
        protection_current_raw=400)
    assert r['command'] == dict(torque_enabled_raw=1, acceleration_raw=0,
        goal_position_raw=1032, goal_time_raw=0, goal_speed_raw=800, torque_limit_raw=900)


@pytest.mark.parametrize('bus,motor', [(0, 0), (0, 7), (1, 7), (2, 2), (True, 2), (0, False)])
def test_invalid_target_never_reaches_wire(bus, motor):
    class NoIO:
        def write(self, _): pytest.fail('invalid target sent')
    with pytest.raises(ValueError): StreamValidationTransportV2(NoIO()).read_servo_tuning(bus, motor)


@pytest.mark.parametrize('offset,value', [(0, 3), (2, 1), (3, 3), (4, 15), (5, 0), (6, 1), (7, 2), (43, 1)])
def test_malformed_reply(offset, value):
    data = response(); data[offset] = value
    with pytest.raises(ValueError): decode_tuning(bytes(data), 0, 2)


@pytest.mark.parametrize('block,offset,size,field', [(0, 12, 7, 'identity'), (1, 19, 14, 'tuning'), (2, 33, 10, 'command')])
def test_partial_reads_never_invent_zero_settings(block, offset, size, field):
    data = response(); data[1] = 3; data[4] &= ~(1 << block)
    data[5:8] = bytes((block, 3, 1)); data[offset:offset+size] = bytes(size)
    r = decode_tuning(bytes(data), 0, 2)
    assert r[field] is None and not r['complete']
    data[offset] = 1
    with pytest.raises(ValueError): decode_tuning(bytes(data), 0, 2)


def test_legacy_rejection_is_not_a_snapshot():
    data = bytearray(56); data[:8] = bytes((1, 1, 0, 2, 0, 255, 0, 0))
    with pytest.raises(TuningQueryUnsupported): decode_tuning(bytes(data), 0, 2)
    data[3] = 3
    with pytest.raises(ValueError) as error: decode_tuning(bytes(data), 0, 2)
    assert not isinstance(error.value, TuningQueryUnsupported)


def test_existing_inventory_now_exposes_already_received_runtime_fields():
    data = bytearray(56); data[:8] = bytes((1, 0, 0, 2, 31, 255, 0, 0)); data[17] = 2
    data[27] = 17; struct.pack_into('<H', data, 30, 1234)
    r = decode_inventory(bytes(data), 0, 2)
    assert r['complete'] and r['command']['acceleration_raw'] == 17 and r['command']['goal_time_raw'] == 1234


def test_single_correlated_read_only_frame():
    class Port:
        def __init__(self): self.pending = b''; self.writes = []
        def write(self, packet):
            self.writes.append(packet); f = decode_frame_v2(packet)
            assert f.message_type == Msg.DEVICE_INVENTORY_REQUEST and f.payload == b'\x02\x00\x02\x00'
            self.pending = encode_frame_v2(FrameV2(message_type=Msg.DEVICE_INVENTORY_RESPONSE,
                sequence=f.sequence, sender_time_ms=100, payload=bytes(response())))
            return len(packet)
        def read_until(self, _):
            packet, self.pending = self.pending, b''
            return packet
    port = Port()
    assert StreamValidationTransportV2(port).read_servo_tuning(0, 2)['complete']
    assert len(port.writes) == 1


@pytest.mark.parametrize('outcome', ['ok', 'unsupported', 'busy', 'wrong_firmware', 'changed', 'wrong_servo'])
def test_diagnostic_never_prepares_unlocks_or_retries(outcome):
    Hello = make_dataclass('Hello', [('firmware_version', int), ('joint_count', int), ('protocol_version', int)])
    State = make_dataclass('State', [('stop_latched', bool), ('status_code', int), ('protocol_version', int),
        ('joint_count', int), ('left_calibration_hash', int), ('right_calibration_hash', int)])
    class Transport:
        def __init__(self): self.calls = []
        def enter_binary_mode(self):
            self.calls.append('hello'); return Hello(0x24930 if outcome != 'wrong_firmware' else 1, 12, 2)
        def get_state(self):
            self.calls.append('state')
            return State(not (outcome == 'changed' and self.calls.count('state') == 2), 0, 2, 12, 11, 11)
        def read_servo_tuning(self, bus, motor):
            assert (bus, motor) == (0, 2); self.calls.append('read')
            if outcome == 'unsupported': raise TuningQueryUnsupported('not supported')
            data = response()
            if outcome == 'busy': data = bytearray(56); data[:8] = bytes((2, 2, 0, 2, 0, 255, 0, 0))
            if outcome == 'wrong_servo': data[17] = 1
            return decode_tuning(bytes(data), bus, motor)
    t = Transport(); r = inspect(t, 'left_shoulder')
    assert r['status'] == dict(ok='READ_COMPLETE', unsupported='FIRMWARE_UPDATE_REQUIRED',
        busy='MAINTENANCE_UNAVAILABLE', wrong_firmware='FAILED', changed='STATE_CHANGED',
        wrong_servo='IDENTITY_MISMATCH')[outcome]
    assert t.calls == (['hello'] if outcome == 'wrong_firmware' else ['hello', 'state', 'read', 'state'])
    assert r['motor_write_commands'] == r['motion_commands'] == 0


def test_default_is_offline(capsys):
    assert main([]) == 0
    assert 'OFFLINE_PREVIEW' in capsys.readouterr().out


def test_actual_c_tuning_reads_both_buses_with_gate_and_failures(tmp_path):
    (tmp_path/'servo_bus.h').write_text('#include <stdint.h>\nuint32_t HAL_GetTick(void);\nint Servo_ReadData(uint8_t,uint8_t,uint8_t,uint8_t*);\n')
    (tmp_path/'right_servo_bus.h').write_text('#include <stdint.h>\nint RightServoBus_ReadInventoryBlock(uint8_t,unsigned,uint8_t*);\nint RightServoBus_ReadTuningBlock(uint8_t,unsigned,uint8_t*);\n')
    source = r'''
#include "device_inventory.h"
#include <assert.h>
#include <string.h>
static unsigned calls,bus,id,fail;
uint32_t HAL_GetTick(void){return 123;}
static int read_block(unsigned b,unsigned i,unsigned block,uint8_t *data){
 assert(b==bus && i==id && block==calls++);
 memset(data,0xa5,device_tuning_length(block));return fail&(1U<<block)?3:0;
}
int Servo_ReadData(uint8_t i,uint8_t address,uint8_t size,uint8_t *data){
 assert(address==device_tuning_address(calls) && size==device_tuning_length(calls));
 return read_block(0,i,calls,data);
}
int RightServoBus_ReadInventoryBlock(uint8_t i,unsigned block,uint8_t *data){
 (void)i;(void)block;(void)data;assert(0);return 3;
}
int RightServoBus_ReadTuningBlock(uint8_t i,unsigned block,uint8_t *data){return read_block(1,i,block,data);}
int main(void){
 struct {uint8_t before,out[56],after;} guarded;
 guarded.before=0x55;guarded.after=0xaa;
 uint8_t *out=guarded.out,q[4]={2,0,2,0};
 for(bus=0;bus<2;bus++)for(id=0;id<256;id++){
  q[1]=bus;q[2]=id;calls=0;DeviceInventory_Read(q,4,true,out);
  bool valid=id>=1&&id<=6;
  assert(calls==(valid?3U:0U) && out[0]==2 && out[1]==(valid?0:1));
  assert(out[4]==(valid?7:0) && guarded.before==0x55 && guarded.after==0xaa);
  for(unsigned j=43;j<56;j++)assert(out[j]==0);
  calls=0;DeviceInventory_Read(q,4,false,out);assert(calls==0 && out[1]==(valid?2:1));
 }
 for(bus=0;bus<2;bus++)for(unsigned block=0;block<3;block++){
  id=2;q[1]=bus;q[2]=id;calls=0;fail=1U<<block;DeviceInventory_Read(q,4,true,out);
  assert(out[1]==3&&out[5]==block&&out[6]==3&&out[7]==bus+1);
  assert(calls==(block==0?1U:3U) && out[4]==(block==0?0:(7U^(1U<<block))));
  for(unsigned j=0;j<device_tuning_length(block);j++)assert(out[device_tuning_offset(block)+j]==0);
 }
 calls=0;q[3]=1;DeviceInventory_Read(q,4,true,out);assert(calls==0&&out[1]==1);
 return 0;
}
'''
    path = tmp_path/'tuning.c'; path.write_text(source); exe = tmp_path/'tuning'
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-DACTUATOR_MOBILE_FIRST_ID=7',
        '-I', str(tmp_path), '-I', str(ROOT/'firmware/stm32_g474_single_arm/Core/Inc'),
        '-I', str(ROOT/'firmware/stm32_actuator/include'), str(path),
        str(ROOT/'firmware/stm32_g474_single_arm/Core/Src/device_inventory.c'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
