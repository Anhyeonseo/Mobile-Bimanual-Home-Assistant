#include "servo_transport.h"
#include <assert.h>
#include <string.h>
UART_HandleTypeDef huart1={.gState=HAL_UART_STATE_READY},other={.gState=HAL_UART_STATE_READY};
uint32_t HAL_GetTick(void){return 0;}uint32_t Timebase_NowUs(void){return 0;}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t){(void)p;(void)n;(void)t;u->sent++;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit(u,p,n,0);}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit(u,p,n,0);}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
int main(void){assert(ServoTransport_Register(&huart1));assert(!ServoTransport_Register(&other));
 for(unsigned id=0;id<256;id++)for(unsigned address=0;address<256;address++)for(unsigned value=0;value<256;value++){
  uint8_t p[8]={255,255,id,4,3,address,value,0};unsigned sum=0;for(unsigned i=2;i<7;i++)sum+=p[i];p[7]=~sum;
  uint32_t token;assert(ServoTransport_BeginService(&huart1,ACTUATOR_BUS_WORK_ARM,&token));
  int allowed=id>=1&&id<=6&&address==40&&(value==0||value==128);
  HAL_StatusTypeDef status=ServoTransport_Transmit(&huart1,token,p,8,10);
  assert((status==HAL_OK)==allowed);
  assert(ServoTransport_End(&huart1,token,false)==HAL_OK);
 }
 assert(other.sent==0);return 0;
}
