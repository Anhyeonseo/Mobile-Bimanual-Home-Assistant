#include "servo_transport.h"
#include <assert.h>
UART_HandleTypeDef huart1={.gState=HAL_UART_STATE_READY};
uint32_t Timebase_NowUs(void){return 1;}
uint32_t HAL_GetTick(void){return 1;}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t){(void)u;(void)p;(void)n;(void)t;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit(u,p,n,0);}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit(u,p,n,0);}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
static void check(unsigned api,unsigned id,unsigned reg,unsigned value,unsigned n,bool allowed,bool readonly){
    uint8_t p[9]={255,255,(uint8_t)id,(uint8_t)(n+3),3,(uint8_t)reg,(uint8_t)value,(uint8_t)(value>>8),0};
    unsigned sum=0;for(unsigned i=2;i<n+6;i++)sum+=p[i];p[n+6]=(uint8_t)~sum;
    uint32_t token;assert(readonly?ServoTransport_BeginReadOnly(&huart1,&token):ServoTransport_BeginService(&huart1,ACTUATOR_BUS_WORK_WHEELS,&token));
    HAL_StatusTypeDef status=api==0?ServoTransport_Transmit(&huart1,token,p,n+7,5):api==1?ServoTransport_TransmitIT(&huart1,token,p,n+7):ServoTransport_TransmitDMA(&huart1,token,p,n+7);
    assert((status==HAL_OK)==allowed);assert(ServoTransport_End(&huart1,token,false)==HAL_OK);
}
int main(void){
    UART_HandleTypeDef other={.gState=HAL_UART_STATE_READY};assert(!ServoTransport_Register(&other));assert(ServoTransport_Register(&huart1));
    for(unsigned api=0;api<3;api++){
        /* Extended telemetry must stay an ID10 read; do not admit neighboring
         * registers, arbitrary lengths or other motors through the new gate. */
        for(unsigned id=0;id<256;id++)for(unsigned n=1;n<=16;n++){
            uint8_t p[8]={255,255,(uint8_t)id,4,2,56,(uint8_t)n,0};
            unsigned sum=0;for(unsigned i=2;i<7;i++)sum+=p[i];p[7]=(uint8_t)~sum;
            uint32_t token;assert(ServoTransport_BeginReadOnly(&huart1,&token));
            HAL_StatusTypeDef status=api==0?ServoTransport_Transmit(&huart1,token,p,8,5):api==1?ServoTransport_TransmitIT(&huart1,token,p,8):ServoTransport_TransmitDMA(&huart1,token,p,8);
            assert((status==HAL_OK)==(id==10&&(n==4||n==15)));
            assert(ServoTransport_End(&huart1,token,false)==HAL_OK);
        }
        for(unsigned id=0;id<256;id++)for(unsigned reg=0;reg<66;reg++)for(unsigned v=0;v<256;v++)
            check(api,id,reg,v,1,id==10&&((reg==33&&v==1)||((reg==40||reg==55)&&v<=1)),false);
        for(unsigned v=0;v<65536;v++)check(api,10,46,v,2,v==0||v==100
#if HOST_LIFT_BIDIRECTIONAL
            ||v==32868
#endif
            ,false);
        for(unsigned id=0;id<256;id++){check(api,id,46,100,2,id==10,false);check(api,id,42,100,2,false,false);}
        check(api,10,40,1,1,false,true);check(api,10,46,100,2,false,true);
    }
    return 0;
}
