#include "servo_transport.h"
#include "bimanual_servo_dispatch.h"
#include <assert.h>
#include <stdlib.h>
UART_HandleTypeDef huart1={.gState=HAL_UART_STATE_READY};
static UART_HandleTypeDef right={.gState=HAL_UART_STATE_READY};
static uint32_t tick;
uint32_t Timebase_NowUs(void){return tick++;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){
 assert(u==&huart1);assert(n>8&&p[0]==255&&p[1]==255&&p[2]==254);
 if(u->fail_tx)return HAL_ERROR;
 u->sent++;u->gState=HAL_UART_STATE_BUSY_TX;
 if(u->immediate){u->gState=HAL_UART_STATE_READY;BimanualServoDispatch_OnTxComplete(u);}
 return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit_DMA(u,p,n);}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t){(void)t;return HAL_UART_Transmit_DMA(u,p,n);}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){assert(u==&huart1);u->aborted++;if(u->fail_abort)return HAL_ERROR;u->gState=HAL_UART_STATE_READY;return HAL_OK;}
int main(int argc,char **argv){
 assert(argc==2);int scenario=atoi(argv[1]);uint32_t lease=0;
 uint16_t pose[6]={2048,2048,2048,2048,2048,2048};
 BimanualServoDispatch_Init(&huart1,&right);
 assert(!ServoTransport_Register(&right));
 assert(!ServoTransport_Begin(&right,ACTUATOR_BUS_WORK_ARM,&lease));
 assert(BimanualServoDispatch_Ready());
 if(scenario==0){
  for(int i=0;i<100;i++){
   assert(BimanualServoDispatch_Launch(pose,NULL,i,tick)==HAL_OK);
   BimanualServoDispatch_OnTxComplete(&right);BimanualServoDispatch_Poll();
   assert(BimanualServoDispatch_GetSnapshot()->active);
   huart1.gState=HAL_UART_STATE_READY;BimanualServoDispatch_OnTxComplete(&huart1);
   BimanualServoDispatch_Poll();assert(BimanualServoDispatch_Ready());
  }
  assert(BimanualServoDispatch_GetSnapshot()->completed_count==100);
  assert(!BimanualServoDispatch_GetSnapshot()->right_complete);
 }else if(scenario==1){
  huart1.immediate=1;assert(BimanualServoDispatch_Launch(pose,NULL,0,0)==HAL_OK);
  assert(BimanualServoDispatch_GetSnapshot()->completed_count==1);assert(BimanualServoDispatch_Ready());
 }else if(scenario==2){
  assert(ServoTransport_Begin(&huart1,ACTUATOR_BUS_WORK_FEEDBACK,&lease));
  assert(!BimanualServoDispatch_Ready());
  const actuator_bimanual_dispatch_snapshot_t *s=BimanualServoDispatch_GetSnapshot();
  assert(!s->active&&!s->faulted&&s->failure_count==0);
  assert(BimanualServoDispatch_Launch(pose,NULL,0,0)==HAL_BUSY);assert(!huart1.sent);
  BimanualServoDispatch_Stop();assert(!huart1.aborted);
  assert(ServoTransport_End(&huart1,lease,false)==HAL_OK);
 }else if(scenario==3){
  huart1.fail_tx=1;assert(BimanualServoDispatch_Launch(pose,NULL,0,0)==HAL_ERROR);
  assert(BimanualServoDispatch_Faulted());assert(!BimanualServoDispatch_Ready());
 }else if(scenario==4){
  assert(BimanualServoDispatch_Launch(pose,NULL,0,0)==HAL_OK);
  BimanualServoDispatch_OnUartError(&huart1);BimanualServoDispatch_Poll();
  assert(BimanualServoDispatch_Faulted());assert(huart1.aborted==1);
 }else if(scenario==5){
  assert(BimanualServoDispatch_Launch(pose,NULL,0,0)==HAL_OK);
  huart1.fail_abort=1;BimanualServoDispatch_Stop();assert(BimanualServoDispatch_Faulted());
  assert(!ServoTransport_Idle(&huart1));huart1.fail_abort=0;BimanualServoDispatch_Stop();
  assert(ServoTransport_Idle(&huart1));assert(!BimanualServoDispatch_Ready());
 }else if(scenario==6){
  assert(BimanualServoDispatch_Launch(pose,NULL,0,0)==HAL_OK);
  huart1.gState=HAL_UART_STATE_READY;
  BimanualServoDispatch_OnTxComplete(&huart1);BimanualServoDispatch_OnTxComplete(&huart1);
  BimanualServoDispatch_Poll();assert(BimanualServoDispatch_Faulted());
 }else return 2;
 assert(right.sent==0&&right.aborted==0);return 0;
}
