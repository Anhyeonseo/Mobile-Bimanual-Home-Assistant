#include "mobile_hold_output.h"
#include "actuator_core/motor_groups.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"%d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t now;
static UART_HandleTypeDef right={.Init={1000000},.gState=HAL_UART_STATE_READY},other={.gState=HAL_UART_STATE_READY};
uint32_t Timebase_NowUs(void){return now;}
uint32_t HAL_GetTick(void){return now/1000;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){
 CHECK(n==26&&p[5]==42);if(u->fail_tx)return HAL_ERROR;u->sent++;u->gState=HAL_UART_STATE_BUSY_TX;
 if(u->immediate){u->gState=HAL_UART_STATE_READY;MobileHoldOutput_OnTxComplete(u);}return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit_DMA(u,p,n);}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t){(void)t;return HAL_UART_Transmit_DMA(u,p,n);}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){u->aborted++;if(u->fail_abort)return HAL_ERROR;u->gState=HAL_UART_STATE_READY;return HAL_OK;}
int main(int argc,char **argv){
 CHECK(argc==2);unsigned scenario=(unsigned)atoi(argv[1]);uint8_t packet[26];size_t n;uint16_t pose[6]={1,2,3,4,5,4095};
 CHECK(!MobileHoldOutput_Configure(&right,300,50));CHECK(MobileHoldOutput_Configure(&right,600,50));
 CHECK(!MobileHoldOutput_Configure(&right,600,50));
 actuator_bus_router_t *r=MobileHoldOutput_Router();
 CHECK(actuator_motor_group_positions(ACTUATOR_GROUP_RIGHT_ARM,pose,6,packet,&n)==ACTUATOR_GROUP_OK);
 if(scenario==3)right.fail_tx=1;
 if(scenario==4)right.immediate=1;
 if(scenario==5){packet[5]=40;packet[25]=actuator_sts3215_checksum(packet+2,23);}
 CHECK(actuator_bus_router_submit(r,ACTUATOR_BUS_WORK_STOP,packet,n,0,4000,600));
 MobileHoldOutput_Poll();CHECK(r->completed_token==0);
 if(scenario==5||scenario==3){CHECK(r->bus.faulted&&!right.sent);return 0;}
 CHECK(right.sent==1);
 if(scenario==1){now=600;MobileHoldOutput_Poll();CHECK(r->bus.faulted&&r->completed_token==0);
  CHECK(!MobileHoldOutput_Recover(false));right.fail_abort=1;CHECK(!MobileHoldOutput_Recover(true));
  right.fail_abort=0;CHECK(MobileHoldOutput_Recover(true));CHECK(r->completed_token==0&&r->inhibited);return 0;}
 if(scenario==6){MobileHoldOutput_OnUartError(&right);MobileHoldOutput_Poll();CHECK(r->bus.faulted&&r->completed_token==0);return 0;}
 if(scenario!=4){now=260;right.gState=HAL_UART_STATE_READY;MobileHoldOutput_OnTxComplete(&other);MobileHoldOutput_OnUartError(&other);
  MobileHoldOutput_Poll();CHECK(r->completed_token==0);MobileHoldOutput_OnTxComplete(&right);}
 if(scenario==2){MobileHoldOutput_OnTxComplete(&right);MobileHoldOutput_Poll();CHECK(r->bus.faulted&&r->completed_token==0);return 0;}
 now+=49;MobileHoldOutput_Poll();CHECK(r->completed_token==0&&!ServoTransport_ReadReady(&right));
 now++;MobileHoldOutput_Poll();CHECK(r->completed_token&&r->completed_length==26&&!memcmp(r->completed_bytes,packet,26));
 CHECK(!ServoTransport_Idle(&right)&&ServoTransport_ReadReady(&right));return 0;
}
