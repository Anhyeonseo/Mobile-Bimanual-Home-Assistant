#include "servo_transport.h"
#include <assert.h>
#include <string.h>
static unsigned sent;
uint32_t Timebase_NowUs(void) {return 1;}
uint32_t HAL_GetTick(void) {return 1;}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t) {
    (void)u;(void)p;(void)n;(void)t;sent++;return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n) {return HAL_UART_Transmit(u,p,n,0);}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n) {return HAL_UART_Transmit(u,p,n,0);}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u) {(void)u;return HAL_OK;}
static void checksum(uint8_t *p) {unsigned sum=0;for(unsigned i=2;i<7;i++)sum+=p[i];p[7]=(uint8_t)~sum;}
int main(void) {
    UART_HandleTypeDef u={.gState=HAL_UART_STATE_READY};uint32_t token;
    assert(ServoTransport_Register(&u));
    for(unsigned api=0;api<3;api++)for(unsigned id=0;id<256;id++)for(unsigned value=0;value<256;value++) {
        const unsigned addresses[]={5,33,40,42,46,48,55};
        for(unsigned a=0;a<sizeof(addresses)/sizeof(addresses[0]);a++) {
            uint8_t p[8]={255,255,(uint8_t)id,4,3,(uint8_t)addresses[a],(uint8_t)value,0};checksum(p);
            assert(ServoTransport_BeginService(&u,ACTUATOR_BUS_WORK_WHEELS,&token));
            HAL_StatusTypeDef status=api==0?ServoTransport_Transmit(&u,token,p,8,5):
                api==1?ServoTransport_TransmitIT(&u,token,p,8):ServoTransport_TransmitDMA(&u,token,p,8);
            bool allowed=id>=7&&id<=9&&(addresses[a]==40&&value<=1);
            assert((status==HAL_OK)==allowed);
            assert(ServoTransport_End(&u,token,false)==HAL_OK);
        }
    }
    for(unsigned api=0;api<3;api++)for(unsigned id=0;id<256;id++)for(unsigned speed=0;speed<65536;speed++) {
        uint8_t p[9]={255,255,(uint8_t)id,5,3,46,(uint8_t)speed,(uint8_t)(speed>>8),0};
        unsigned sum=0;for(unsigned i=2;i<8;i++)sum+=p[i];p[8]=(uint8_t)~sum;
        assert(ServoTransport_BeginService(&u,ACTUATOR_BUS_WORK_WHEELS,&token));
        HAL_StatusTypeDef status=api==0?ServoTransport_Transmit(&u,token,p,9,5):
            api==1?ServoTransport_TransmitIT(&u,token,p,9):ServoTransport_TransmitDMA(&u,token,p,9);
        assert((status==HAL_OK)==(id>=7&&id<=9&&(speed==0||speed==100||speed==200||speed==(32768+100)||speed==(32768+200))));
        assert(ServoTransport_End(&u,token,false)==HAL_OK);
    }
    uint8_t p[8]={255,255,7,4,3,40,1,0};checksum(p);
    assert(ServoTransport_BeginService(&u,ACTUATOR_BUS_WORK_WHEELS,&token));
    p[7]^=1;assert(ServoTransport_Transmit(&u,token,p,8,5)!=HAL_OK);
    p[7]^=1;assert(ServoTransport_Transmit(&u,token,p,8,5)==HAL_OK);
    assert(ServoTransport_Transmit(&u,token,p,8,5)!=HAL_OK);
    assert(ServoTransport_End(&u,token,false)==HAL_OK);
    return 0;
}
