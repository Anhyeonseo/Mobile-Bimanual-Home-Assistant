"""DMA progresses even with IRQs masked: execute the real bus cursor readers."""
from pathlib import Path
import subprocess
import pytest

ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'


def function(text,signature):
    start=text.index(signature);end=text.index('\n}',start)+3
    return text[start:end]


@pytest.fixture(scope='module')
def cursor_exes(tmp_path_factory):
    folder=tmp_path_factory.mktemp('dma-cursor')
    (folder/'stm32g4xx_hal.h').write_text('''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct {int placeholder;} DMA_HandleTypeDef;
typedef struct {DMA_HandleTypeDef *hdmarx;} UART_HandleTypeDef;
uint32_t mask,reads,inject_at,inject_after,inject_bytes,tc,zero_counter;
uint64_t produced;
volatile uint32_t servo_dma_wrap_count,right_servo_rx_wrap_count;
static void progress(void){uint64_t before=produced/256;produced+=inject_bytes;if(produced/256!=before)tc=1;}
static uint32_t sample(int counter){
 reads++;
 if(reads==inject_at&&!inject_after)progress();
 uint32_t v=counter?(zero_counter?0:256-(uint32_t)(produced%256)):tc;
 if(reads==inject_at&&inject_after)progress();
 return v;
}
#define __get_PRIMASK() mask
#define __disable_irq() (mask=1)
static void enable(void){mask=0;if(tc){servo_dma_wrap_count=right_servo_rx_wrap_count=(uint32_t)(produced/256);tc=0;}}
#define __enable_irq() enable()
#define __DMB() ((void)0)
#define __HAL_DMA_GET_COUNTER(dma) ((void)(dma),sample(1))
#define __HAL_DMA_GET_FLAG(dma,index) ((void)(dma),(void)(index),sample(0))
#define __HAL_DMA_GET_TC_FLAG_INDEX(dma) 1U
''')
    bodies='\n'.join(function((BOARD/'Src'/file).read_text(),signature) for file,signature in (
        ('servo_bus.c','static uint32_t ServoBus_ProducerAbsolute(void)'),
        ('right_servo_bus.c','static uint32_t RightServo_TelemetryProducerAbsolute(void)')))
    code='''#include <assert.h>
#include <stdlib.h>
#include "servo_dma_cursor.h"
#include "servo_rx_window.h"
static uint32_t OldCursor(DMA_HandleTypeDef *dma,const volatile uint32_t *count,uint32_t size){
 uint32_t saved=mask;__disable_irq();uint32_t wraps=*count;
 uint32_t remaining=__HAL_DMA_GET_COUNTER(dma);
 uint32_t pending=__HAL_DMA_GET_FLAG(dma,1);
 if(pending&&remaining)wraps++;
 uint32_t result=wraps*size+size-remaining;
 if(!saved)__enable_irq();return result;
}
#ifdef OLD_CURSOR
#define ServoDmaCursor_Read OldCursor
#endif
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef uart={&dma};
static UART_HandleTypeDef *servo_uart_handle=&uart,*right_servo_uart=&uart;
static struct {uint16_t producer_index;} servo_bus_health;
#define SERVO_BUS_DMA_RING_CAPACITY 256U
#define RIGHT_SERVO_TELEMETRY_RING_CAPACITY 256U
static uint8_t ServoBus_DmaHardwareActive(void){return 1;}
static uint8_t RightServo_TelemetryDmaActive(void){return 1;}
''' + bodies + '''
int main(int argc,char **argv){
 assert(argc==7);
 int bus=atoi(argv[1]);inject_at=atoi(argv[2]);inject_after=atoi(argv[3]);
 produced=strtoull(argv[4],NULL,0);mask=atoi(argv[5]);inject_bytes=atoi(argv[6]);
 uint32_t prior_mask=mask;
 servo_dma_wrap_count=right_servo_rx_wrap_count=(uint32_t)(produced/256);
 uint64_t before=produced;
 uint32_t first=bus?RightServo_TelemetryProducerAbsolute():ServoBus_ProducerAbsolute();
 assert((uint32_t)(first-(uint32_t)before)<=produced-before);
 assert(mask==prior_mask && reads<=6);
 inject_at=0;reads=0;
 uint32_t second=bus?RightServo_TelemetryProducerAbsolute():ServoBus_ProducerAbsolute();
 assert(second==(uint32_t)produced);
 assert((uint32_t)(second-first)<=produced-before);
 assert(mask==prior_mask);
 /* Counter-at-zero before/after pending TC, and delivered TC, same position. */
 for(unsigned pending=0;pending<2;pending++){
  produced=2048;servo_dma_wrap_count=right_servo_rx_wrap_count=7;tc=pending;zero_counter=1;
  assert((bus?RightServo_TelemetryProducerAbsolute():ServoBus_ProducerAbsolute())==2048);
 }
 /* The actual parser must receive a complete packet spanning the boundary. */
 mask=0;zero_counter=0;tc=0;produced=255;
 servo_dma_wrap_count=right_servo_rx_wrap_count=0;
 uint8_t ring[256]={0},packet[8]={255,255,1,4,0,0,8,242},reply[2];
 for(unsigned i=0;i<8;i++)ring[248+i]=packet[i];
 ServoRxWindow window;ServoRxWindow_Init(&window,1,2,248);
 reads=0;inject_at=2;inject_after=1;inject_bytes=1;
 uint32_t absolute=bus?RightServo_TelemetryProducerAbsolute():ServoBus_ProducerAbsolute();
 assert(ServoRxWindow_Consume(&window,ring,256,absolute,reply,2)==SERVO_RX_WINDOW_FRAME_READY);
 assert(reply[0]==0 && reply[1]==8);
 ServoRxWindow_Init(&window,1,2,0);
 assert(ServoRxWindow_Consume(&window,ring,256,257,reply,2)==SERVO_RX_WINDOW_OVERFLOW);
 (void)OldCursor;return 0;
}
'''
    (folder/'harness.c').write_text(code)
    for name,extra in (('fixed',[]),('old',['-DOLD_CURSOR'])):
        subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
            '-I',str(folder),'-I',str(BOARD/'Inc'),*extra,str(folder/'harness.c'),
            str(BOARD/'Src/servo_rx_window.c'),str(BOARD/'Src/servo_response_parser.c'),
            '-I',str(ROOT/'firmware/stm32_actuator/include'),
            str(ROOT/'firmware/stm32_actuator/src/sts3215_response.c'),
            '-o',str(folder/name)],check=True)
    return folder


@pytest.mark.parametrize('bus',[0,1])
@pytest.mark.parametrize('register_read',range(1,7))
@pytest.mark.parametrize('after',[0,1])
@pytest.mark.parametrize('start',[254,255,256,4294967295])
@pytest.mark.parametrize('masked',[0,1])
def test_hardware_wrap_at_each_register_read(cursor_exes,bus,register_read,after,start,masked):
    subprocess.run([str(cursor_exes/'fixed'),str(bus),str(register_read),str(after),str(start),str(masked),'2'],check=True)


@pytest.mark.parametrize('bus',[0,1])
def test_original_read_order_reports_bytes_not_received(cursor_exes,bus):
    # CNDTR=1 is sampled; DMA receives the final byte before TC is sampled.
    # Old calculation reports 511 even though only 256 bytes exist.
    result=subprocess.run([str(cursor_exes/'old'),str(bus),'1','1','255','0','1'],capture_output=True,text=True)
    assert result.returncode!=0 and 'produced-before' in result.stderr
