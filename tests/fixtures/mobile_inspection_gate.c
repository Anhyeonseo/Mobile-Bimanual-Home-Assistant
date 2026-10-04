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
    for(unsigned api=0;api<3;api++) for(unsigned id=0;id<256;id++) for(unsigned op=0;op<256;op++) {
        uint8_t p[8]={255,255,(uint8_t)id,4,(uint8_t)op,33,1,0};checksum(p);
        assert(ServoTransport_BeginService(&u,ACTUATOR_BUS_WORK_WHEELS,&token));
        HAL_StatusTypeDef result=api==0?ServoTransport_Transmit(&u,token,p,8,5):
            api==1?ServoTransport_TransmitIT(&u,token,p,8):ServoTransport_TransmitDMA(&u,token,p,8);
        assert((result==HAL_OK)==(id>=7&&id<=10&&op==2));
        assert(ServoTransport_End(&u,token,false)==HAL_OK);
    }
    assert(sent==12);
    for(unsigned address=0;address<256;address++) for(unsigned size=0;size<17;size++) {
        uint8_t p[8]={255,255,7,4,2,(uint8_t)address,(uint8_t)size,0};checksum(p);
        assert(ServoTransport_BeginService(&u,ACTUATOR_BUS_WORK_FEEDBACK,&token));
        bool allowed=(address==3&&size==4)||(address==33&&size==1)||(address==40&&size==1)||(address==56&&size==2);
        assert((ServoTransport_Transmit(&u,token,p,8,5)==HAL_OK)==allowed);
        assert(ServoTransport_End(&u,token,false)==HAL_OK);
    }
    assert(sent==16);
    /* The left-arm reference permits only identity READ. */
    for(unsigned op=0;op<256;op++) for(unsigned address=0;address<256;address++) {
        uint8_t ref[8]={255,255,1,4,(uint8_t)op,(uint8_t)address,4,0};checksum(ref);
        assert(ServoTransport_BeginService(&u,ACTUATOR_BUS_WORK_FEEDBACK,&token));
        assert((ServoTransport_Transmit(&u,token,ref,8,5)==HAL_OK)==(op==2&&address==3));
        assert(ServoTransport_End(&u,token,false)==HAL_OK);
    }
    assert(sent==17);
    uint8_t p[8]={255,255,7,4,2,33,1,0};checksum(p);
    assert(ServoTransport_BeginService(&u,ACTUATOR_BUS_WORK_FEEDBACK,&token));
    p[7]^=1;assert(ServoTransport_Transmit(&u,token,p,8,5)==HAL_ERROR);
    p[7]^=1;assert(ServoTransport_Transmit(&u,token,p,7,5)==HAL_ERROR);
    assert(ServoTransport_Transmit(&u,token,p,8,5)==HAL_OK);
    assert(ServoTransport_Transmit(&u,token,p,8,5)==HAL_ERROR); /* no retry in same lease */
    assert(sent==18);
    return 0;
}
