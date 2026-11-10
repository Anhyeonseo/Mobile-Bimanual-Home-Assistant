"""Commissioning reads: actual C reader/gate plus Python wire and tool boundaries."""
import importlib.util
from dataclasses import make_dataclass
from pathlib import Path
import struct
import subprocess

import pytest

from so101_arm_bridge.device_inventory import (
    EXPECTED_DEVICES, FIRMWARE_VERSION, collect_inventory, decode_inventory, inventory_request,
)
from so101_arm_bridge.stream_protocol_v2 import FrameV2, StreamMessageTypeV2 as Msg, decode_frame_v2, encode_frame_v2
from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
from test_stm32_servo_uart_circular_dma_contract import BINARY_CONTROL, function_body

ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT/'firmware/stm32_g474_single_arm'
CORE = ROOT/'firmware/stm32_actuator'


def response(bus=0, motor_id=1):
    data = bytearray(56)
    data[:8] = bytes((1, 0, bus, motor_id, 31, 255, 0, 0))
    data[17] = motor_id
    struct.pack_into('<H', data, 37, 2048)
    return data


@pytest.mark.parametrize('bus,motor_id', EXPECTED_DEVICES)
def test_valid_snapshot(bus, motor_id):
    assert inventory_request(bus, motor_id) == bytes((1, bus, motor_id, 0))
    decoded = decode_inventory(bytes(response(bus, motor_id)), bus, motor_id)
    assert decoded['complete'] and decoded['feedback']['position_raw'] == 2048


@pytest.mark.parametrize('bus,motor_id', [(0,0),(0,11),(0,254),(1,7),(2,1),(False,1),(0,True),('0',1)])
def test_invalid_address_never_sends(bus, motor_id):
    class NoIO:
        def write(self, _): pytest.fail('invalid address reached wire')
    with pytest.raises(ValueError):
        StreamValidationTransportV2(NoIO()).read_device_inventory(bus, motor_id)


def test_partial_failure_is_not_zero_position_or_healthy_motor():
    data = response()
    data[1:8] = bytes((3,0,1,15,4,3,1))
    result = decode_inventory(bytes(data),0,1)
    assert result['feedback'] is None and not result['complete']
    data[1:8] = bytes((3,0,1,0,0,3,1))
    result = decode_inventory(bytes(data),0,1)
    assert all(result[k] is None for k in ('identity','limits','offset_and_mode','command','feedback'))


@pytest.mark.parametrize('offset,value', [(0,2),(2,1),(3,2),(4,16),(5,0),(6,1),(7,1),(52,1)])
def test_malformed_or_uncorrelated_reply_rejected(offset,value):
    data = response(); data[offset] = value
    with pytest.raises(ValueError): decode_inventory(bytes(data),0,1)


def test_id_disagreement_not_complete():
    data = response(); data[17] = 2
    assert not decode_inventory(bytes(data),0,1)['complete']


def test_transport_uses_only_correlated_inventory_frames():
    class Port:
        def __init__(self): self.pending=b''; self.writes=[]
        def write(self, packet):
            self.writes.append(packet)
            frame=decode_frame_v2(packet)
            assert frame.message_type == Msg.DEVICE_INVENTORY_REQUEST
            assert frame.payload == b'\x01\x01\x06\x00'
            self.pending = encode_frame_v2(FrameV2(message_type=Msg.DEVICE_INVENTORY_RESPONSE,
                sequence=frame.sequence, sender_time_ms=100, payload=bytes(response(1,6))))
            return len(packet)
        def read_until(self, _):
            packet,self.pending=self.pending,b''
            return packet
    port=Port()
    result=StreamValidationTransportV2(port).read_device_inventory(1,6)
    assert result['complete'] and len(port.writes)==1


def test_collection_keeps_prior_results_on_link_failure():
    class Transport:
        def __init__(self): self.calls=[]
        def read_device_inventory(self,bus,motor_id):
            self.calls.append((bus,motor_id))
            if len(self.calls)==3: raise RuntimeError('disconnected')
            return decode_inventory(bytes(response(bus,motor_id)),bus,motor_id)
    t=Transport(); result=collect_inventory(t)
    assert len(result['devices'])==2 and len(t.calls)==3
    assert not result['all_snapshots_complete'] and result['interrupted']['motor_id']==3


def test_tool_default_does_not_read_motors_and_requires_known_firmware():
    spec=importlib.util.spec_from_file_location('inventory_tool', ROOT/'tools/diagnostics/stm32_device_inventory.py')
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    Hello=make_dataclass('Hello',[('firmware_version',int),('joint_count',int),('protocol_version',int),('stop_latched',bool)])
    State=make_dataclass('State',[('stop_latched',bool),('status_code',int),('protocol_version',int),('joint_count',int),('left_calibration_hash',int),('right_calibration_hash',int)])
    class Transport:
        version=FIRMWARE_VERSION
        stopped=False
        reads=[]
        def enter_binary_mode(self): return Hello(self.version,12,2,False)
        def get_state(self): return State(self.stopped,0,2,12,123,123)
        def read_device_inventory(self,bus,motor_id):
            self.reads.append((bus,motor_id))
            return decode_inventory(bytes(response(bus,motor_id)),bus,motor_id)
    t=Transport()
    assert module.inspect(t)['passed'] and not t.reads
    assert module.inspect(t,True)['all_snapshots_complete']
    assert tuple(t.reads)==EXPECTED_DEVICES
    t.reads.clear(); t.version=0x24908
    with pytest.raises(RuntimeError): module.inspect(t,True)
    assert not t.reads
    t.version=FIRMWARE_VERSION; t.stopped=True
    blocked=module.inspect(t,True)
    assert not blocked['passed'] and blocked['state_before']['stop_latched']
    assert blocked['motor_reads_sent']==0 and not t.reads


def test_actual_c_fixed_reader_rejects_invalid_busy_and_partial_reads(tmp_path):
    (tmp_path/'servo_bus.h').write_text('''#include <stdint.h>
uint32_t HAL_GetTick(void);
int Servo_ReadData(uint8_t id,uint8_t address,uint8_t length,uint8_t *data);
''')
    (tmp_path/'right_servo_bus.h').write_text('''#include <stdint.h>
int RightServoBus_ReadInventoryBlock(uint8_t id,unsigned block,uint8_t *data);
''')
    harness=tmp_path/'harness.c'
    harness.write_text(r'''
#include "device_inventory.h"
#include <assert.h>
#include <string.h>
static unsigned calls, expected_bus, expected_id, fail_mask;
uint32_t HAL_GetTick(void){return 0x12345678;}
static int read_block(unsigned bus,unsigned id,unsigned block,uint8_t *data){
 assert(bus==expected_bus && id==expected_id && block==calls++);
 memset(data,0xa5,device_inventory_length(block));
 return fail_mask & (1U<<block) ? 3 : 0;
}
int Servo_ReadData(uint8_t id,uint8_t address,uint8_t length,uint8_t *data){
 unsigned block=calls;
 assert(address==device_inventory_address(block) && length==device_inventory_length(block));
 return read_block(0,id,block,data);
}
int RightServoBus_ReadInventoryBlock(uint8_t id,unsigned block,uint8_t *data){
 return read_block(1,id,block,data);
}
int main(void){
 uint8_t q[4]={1,0,1,0},out[56];
 for(unsigned bus=0;bus<2;bus++)for(unsigned id=0;id<256;id++){
  q[1]=bus;q[2]=id;calls=0;expected_bus=bus;expected_id=id;
  DeviceInventory_Read(q,4,true,out);
  bool valid=id>=1 && id<=(bus==0?10:6);
  assert(calls==(valid?5U:0U));assert(out[1]==(valid?0:1));
  assert(out[4]==(valid?31:0));assert(out[5]==255);
  calls=0;DeviceInventory_Read(q,4,false,out);assert(calls==0);
 }
 q[1]=0;q[2]=1;expected_bus=0;expected_id=1;
 for(unsigned block=0;block<5;block++){
  calls=0;fail_mask=1U<<block;DeviceInventory_Read(q,4,true,out);
  assert(out[1]==3 && out[5]==block && out[6]==3 && out[7]==1);
  assert(out[4]==(block==0?0:(31U^(1U<<block))));
  assert(calls==(block==0?1U:5U));
  for(unsigned j=0;j<device_inventory_length(block);j++)assert(out[device_inventory_offset(block)+j]==0);
 }
 calls=0;fail_mask=6;DeviceInventory_Read(q,4,true,out);assert(out[5]==1 && out[4]==25);
 q[1]=1;expected_bus=1;calls=0;DeviceInventory_Read(q,4,true,out);assert(out[7]==2);
 calls=0;
 q[0]=2;DeviceInventory_Read(q,4,true,out);assert(out[1]==1);
 q[0]=1;q[1]=2;DeviceInventory_Read(q,4,true,out);assert(out[1]==1);
 q[1]=0;q[3]=1;DeviceInventory_Read(q,4,true,out);assert(out[1]==1);
 DeviceInventory_Read(q,3,true,out);DeviceInventory_Read(0,4,true,out);assert(calls==0);
 assert(out[8]==0x78 && out[11]==0x12);
 return 0;
}
''')
    exe=tmp_path/'reader'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-DACTUATOR_MOBILE_FIRST_ID=7',
        '-I',str(tmp_path),'-I',str(BOARD/'Core/Inc'),'-I',str(CORE/'include'),
        str(harness),str(BOARD/'Core/Src/device_inventory.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


def test_actual_maintenance_gate_rejects_every_motion_owner(tmp_path):
    body=function_body(BINARY_CONTROL,'static bool Host_InventoryPermitted(void)')
    source=r'''
#include <stdbool.h>
#include <assert.h>
enum {ACTUATOR_V2_EXECUTOR_CLOSED=0};
static unsigned host_stop_latched,host_binary_servos_configured,host_right_arm_output_active;
static unsigned host_v2_executor_ready=1,host_v2_executor_clock_active,buffered,mobile,stop,observer;
static struct {unsigned active;} host_binary_motion;
static struct {struct {unsigned state;} diagnostics;} host_v2_stream_executor;
static unsigned Host_BufferedExecutionIsActive(void){return buffered;}
static unsigned MobileBoard_IsConfigured(void){return mobile;}
static unsigned MobileBoard_StopActive(void){return stop;}
static unsigned MobileArmObserver_OwnsFeedback(void){return observer;}
'''
    source+='static bool Host_InventoryPermitted(void){'+body+'}\n'
    source+=r'''
int main(void){
 unsigned *blocked[]={&host_stop_latched,&host_binary_servos_configured,&host_right_arm_output_active,
 &host_binary_motion.active,&buffered,&host_v2_executor_clock_active,
 &host_v2_stream_executor.diagnostics.state,&mobile,&stop,&observer};
 assert(Host_InventoryPermitted());
 for(unsigned i=0;i<sizeof(blocked)/sizeof(blocked[0]);i++){
  *blocked[i]=1;assert(!Host_InventoryPermitted());*blocked[i]=0;
 }
 host_v2_executor_ready=0;assert(!Host_InventoryPermitted());return 0;
}
'''
    path=tmp_path/'gate.c';path.write_text(source);exe=tmp_path/'gate'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',str(path),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
