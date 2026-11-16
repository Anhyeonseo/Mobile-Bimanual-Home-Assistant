"""Actual read wrappers and transport gate: late RX must not strand its owner."""
from pathlib import Path
import subprocess
import pytest
from test_servo_smooth_configuration import function
ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'

@pytest.fixture(scope='module',params=['teleop','unified'])
def binary(tmp_path_factory,request):
    out=tmp_path_factory.mktemp('hold-read-lease')
    # The left file also declares a prototype; extract the final definition.
    text=(BOARD/'Src/servo_bus.c').read_text();at=text.rindex('HAL_StatusTypeDef Servo_ReadData(')
    copy=out/'left.c';copy.write_text(text[at:])
    left=function(copy,'HAL_StatusTypeDef Servo_ReadData(')
    right=function(BOARD/'Src/right_servo_bus.c','static RightServoReadStatus RightServo_ReadData(')
    source=out/'check.c'
    source.write_text(r'''
#include <assert.h>
#include <stdlib.h>
#include "servo_bus.h"
#include "right_servo_bus.h"
#include "servo_transport.h"
static UART_HandleTypeDef uart={.gState=HAL_UART_STATE_READY};
static UART_HandleTypeDef *servo_uart_handle=&uart,*right_servo_uart=&uart;
static uint32_t servo_service_token,right_service_token;
static unsigned calls;
uint32_t Timebase_NowUs(void){return 0;}
uint32_t HAL_GetTick(void){return 0;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){return u->fail_abort?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *d,uint16_t n,uint32_t t){(void)u;(void)d;(void)n;(void)t;abort();}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *d,uint16_t n){(void)u;(void)d;(void)n;abort();}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *d,uint16_t n){(void)u;(void)d;(void)n;abort();}
static HAL_StatusTypeDef Servo_ReadDataOwned(uint8_t id,uint8_t addr,uint8_t n,uint8_t *data){
 assert(id==1&&addr==33&&n==1);*data=0;calls++;if(calls==1)uart.wire_busy=1;return HAL_OK;
}
static RightServoReadStatus RightServo_ReadDataOwned(uint8_t id,uint8_t addr,uint8_t n,uint8_t *data){
 Servo_ReadDataOwned(id,addr,n,data);return RIGHT_SERVO_READ_OK;
}
'''+left+'\n'+right+r'''
int main(int argc,char **argv){
 assert(argc==3);unsigned right=atoi(argv[1]),fault=atoi(argv[2]);uint8_t data=255;
 assert(ServoTransport_Register(&uart));
 int first=right?RightServo_ReadData(1,33,1,&data):Servo_ReadData(1,33,1,&data);
 assert(first!=0&&calls==1&&!ServoTransport_ReadReady(&uart));
 if(fault==1)uart.fail_abort=1;else if(fault!=2)uart.wire_busy=0;
 int next=right?RightServo_ReadData(1,33,1,&data):Servo_ReadData(1,33,1,&data);
 assert((next==0)==(fault==0));assert(calls==(fault?1:2));
 uart.fail_abort=0;uart.wire_busy=0;
 int final=right?RightServo_ReadData(1,33,1,&data):Servo_ReadData(1,33,1,&data);
 assert(final==0&&ServoTransport_ReadReady(&uart));return 0;
}
''')
    common=['gcc','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'tests/fixtures/servo_hal'),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include')]
    # Default gate has identical ownership semantics without unrelated profile stubs.
    obj=out/'gate.o'
    subprocess.run([*common,'-c',str(BOARD/'Src/servo_transport.c'),'-o',str(obj)],check=True)
    binary=out/'check'
    defines=(['-DHOST_MOBILE_BASE_TELEOP_ONLY=1','-DHOST_MOBILE_BASE_PRESERVE_HOLD=1'] if request.param=='teleop' else ['-DHOST_S4_UNIFIED_CANDIDATE=1'])
    subprocess.run([*common,*defines,str(source),str(obj),str(CORE/'src/bus_router.c'),str(CORE/'src/shared_bus.c'),'-o',str(binary)],check=True)
    return binary

@pytest.mark.parametrize('right',[0,1])
@pytest.mark.parametrize('fault',[0,1,2])
def test_read_owner_released_only_after_uart_quiet_and_abort_success(binary,right,fault):
    subprocess.run([str(binary),str(right),str(fault)],check=True)
