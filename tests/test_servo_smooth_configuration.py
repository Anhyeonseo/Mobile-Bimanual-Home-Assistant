"""Execute the actual configuration functions against a register-level servo fake.

This checks the emitted register contract, not physical PID performance.
"""
from pathlib import Path
import subprocess
import pytest
from so101_arm_bridge.bimanual_stream_adapter import CALIBRATION_HASH

ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'

def function(path,signature):
    source=path.read_text();a=source.index(signature);start=source.index('{',a)
    depth=1;b=start+1
    while depth:
        depth+=(source[b]=='{')-(source[b]=='}');b+=1
    return source[a:b]

@pytest.fixture(scope='module')
def binary(tmp_path_factory):
    folder=tmp_path_factory.mktemp('smooth-servo');src=folder/'check.c'
    left=function(BOARD/'Src/servo_bus.c','HAL_StatusTypeDef Servo_ConfigureForTrajectory(')
    right=function(BOARD/'Src/right_servo_bus.c','RightServoConfigureSnapshot RightServoBus_ConfigureAtPresentPositionOnce(')
    digest=function(BOARD/'Src/binary_control.c','static uint32_t Host_CalibrationHash(void)')
    src.write_text(r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "servo_bus.h"
#include "right_servo_bus.h"
#include "actuator_core/crc32c.h"
static UART_HandleTypeDef uart;
static UART_HandleTypeDef *right_servo_uart=&uart;
static unsigned char registers[256];
static unsigned scenario,goal_writes,delay_ms;
bool ServoTransport_ServiceAllowed(void){return true;}
void HAL_Delay(uint32_t ms){delay_ms+=ms;}
static uint16_t RightServo_ReadU16Le(const uint8_t *b){return b[0]|((uint16_t)b[1]<<8);}
HAL_StatusTypeDef Servo_ReadPosition(uint8_t id,uint16_t *p){assert(id==2);*p=2345;return HAL_OK;}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t addr,const uint8_t *data,uint8_t len){
 assert(id==2);if(addr<=42&&addr+len>42)goal_writes++;
 if((scenario==1&&addr==41)||(scenario==2&&addr<=44&&addr+len>44))return HAL_OK; /* silent ignored write */
 memcpy(registers+addr,data,len);return HAL_OK;
}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t addr,uint8_t len,uint8_t *data){
 assert(id==2);memcpy(data,registers+addr,len);
 if(scenario==4&&addr==21)data[0]^=1;
 return HAL_OK;
}
static RightServoReadStatus RightServo_ReadData(uint8_t id,uint8_t addr,uint8_t len,uint8_t *data){
 return Servo_ReadData(id,addr,len,data)==HAL_OK?RIGHT_SERVO_READ_OK:RIGHT_SERVO_READ_UNAVAILABLE;
}
static HAL_StatusTypeDef RightServo_WriteData(uint8_t id,uint8_t addr,const uint8_t *data,uint8_t len){return Servo_WriteData(id,addr,data,len);}
static RightServoReadStatus RightServo_ReadPosition(uint8_t id,uint16_t *position){return Servo_ReadPosition(id,position)==HAL_OK?RIGHT_SERVO_READ_OK:RIGHT_SERVO_READ_UNAVAILABLE;}
'''+left+'\n'+right+'\n'+digest+r'''
int main(int argc,char **argv){
 assert(argc==2);scenario=(unsigned)atoi(argv[1]);
 if(scenario==8){printf("%08X\n",Host_CalibrationHash());return 0;}
 memset(registers,0,sizeof(registers));registers[41]=88;registers[44]=77;registers[45]=1;
 registers[42]=123;registers[43]=4;registers[40]=scenario==3?1:0;
 if(scenario>=5){
   unsigned fault=scenario-5;scenario=fault;uint16_t initial=0;
   HAL_StatusTypeDef result=Servo_ConfigureForTrajectory(2,900,16,32,&initial);
   assert(result==(fault?HAL_ERROR:HAL_OK));assert(initial==2345);
   if(!fault){assert(registers[41]==0&&registers[44]==0&&registers[45]==0);assert(RightServo_ReadU16Le(registers+46)==800);}
 }else{
   RightServoConfigureSnapshot s=RightServoBus_ConfigureAtPresentPositionOnce(2,16,32,800,900);
   assert((s.status==RIGHT_SERVO_CONFIGURE_OK)==(scenario==0));
   assert(!goal_writes&&registers[42]==123&&registers[43]==4);
   if(scenario!=3)assert(registers[40]==0);
   if(!scenario){assert(registers[41]==0&&registers[44]==0&&registers[45]==0);assert(s.p_gain==16&&s.d_gain==32&&s.i_gain==0);}
 }
 return 0;
}
''')
    out=folder/'check'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'tests/fixtures/servo_hal'),
        '-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),str(src),str(BOARD/'Src/servo_joint_config.c'),
        str(CORE/'src/crc32c.c'),'-o',str(out)],check=True)
    return out

@pytest.mark.parametrize('scenario',range(8))
def test_register_setup_and_silent_write_readback_failures(binary,scenario):
    subprocess.run([str(binary),str(scenario)],check=True)

def test_compiled_firmware_configuration_hash_matches_ros_bridge(binary):
    actual=int(subprocess.check_output([str(binary),'8'],text=True),16)
    assert actual==CALIBRATION_HASH
