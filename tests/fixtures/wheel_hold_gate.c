#include "servo_transport.h"
#include <assert.h>
#include <string.h>
UART_HandleTypeDef huart1={.gState=HAL_UART_STATE_READY},huart4={.gState=HAL_UART_STATE_READY};
bool WheelSetup_ConsumeWrite(UART_HandleTypeDef *u,uint8_t id,uint8_t addr,const uint8_t *d,uint8_t n){
    (void)u;(void)id;(void)addr;(void)d;(void)n;return false; /* No setup capability in normal mode. */
}
static unsigned sent;
uint32_t Timebase_NowUs(void){return 1;}
uint32_t HAL_GetTick(void){return 1;}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t){(void)u;(void)p;(void)n;(void)t;sent++;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit(u,p,n,0);}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit(u,p,n,0);}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
static void check(unsigned bus,unsigned api,uint8_t *packet,unsigned n,bool expected,bool readonly){
    UART_HandleTypeDef *uart=bus?&huart4:&huart1;uint32_t token;
    unsigned sum=0;for(unsigned i=2;i<n-1;i++)sum+=packet[i];packet[n-1]=(uint8_t)~sum;
    assert(readonly?ServoTransport_BeginReadOnly(uart,&token):ServoTransport_BeginService(uart,ACTUATOR_BUS_WORK_WHEELS,&token));
    unsigned old=sent;
    HAL_StatusTypeDef status=api==0?ServoTransport_Transmit(uart,token,packet,n,1):
        api==1?ServoTransport_TransmitIT(uart,token,packet,n):ServoTransport_TransmitDMA(uart,token,packet,n);
    assert((status==HAL_OK)==expected);assert(sent-old==(unsigned)expected);
    assert(ServoTransport_End(uart,token,false)==HAL_OK);
}
int main(void){
    assert(ServoTransport_Register(&huart1));assert(ServoTransport_Register(&huart4));
    const unsigned values[]={0,1,100,200,256,32868,32968,65535};
    for(unsigned b=0;b<2;b++)for(unsigned api=0;api<3;api++)for(unsigned id=0;id<256;id++)
    for(unsigned addr=0;addr<256;addr++)for(unsigned k=0;k<sizeof(values)/sizeof(values[0]);k++){
        unsigned v=values[k];bool wheel=!b && id>=7 && id<=9;
        uint8_t p[9]={255,255,id,5,3,addr,v,v>>8,0};
        bool ok=wheel && addr==46 && (v==0||v==100||v==200||v==32868||v==32968);
        check(b,api,p,9,ok,false);
        p[3]=4;p[6]=(uint8_t)v;
        check(b,api,p,8,wheel&&addr==40&&(uint8_t)v<=1,false);
    }
    for(unsigned b=0;b<2;b++)for(unsigned id=0;id<256;id++)for(unsigned addr=0;addr<72;addr++)
    for(unsigned length=1;length<=16;length++){
        bool held=(id>=1&&id<=6)||(!b&&id==10),wheel=!b&&id>=7&&id<=9;
        bool ok=(held&&((addr==33&&length==1)||(addr==40&&length==8)||((addr==3||addr==56)&&length==4)||(!b&&id==10&&((addr==9&&length==4)||((addr==40||addr==55)&&length==1)||(addr==42&&length==2)))))||
            (wheel&&((addr==3&&length==4)||((addr==33||addr==40||addr==55)&&length==1)||
                ((addr==46||addr==58||addr==56)&&length==2)));
        uint8_t p[8]={255,255,id,4,2,addr,length,0};check(b,0,p,8,ok,false);
    }
    uint8_t torque[8]={255,255,7,4,3,40,1,0};check(0,0,torque,8,false,true);
    /* Broadcast/sync-write never escape by changing instruction bytes. */
    for(unsigned op=0;op<256;op++)if(op!=2&&op!=3){
        uint8_t p[8]={255,255,7,4,op,40,1,0};check(0,0,p,8,false,false);
    }
}
