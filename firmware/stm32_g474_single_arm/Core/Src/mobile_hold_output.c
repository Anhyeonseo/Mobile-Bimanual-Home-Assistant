#include "mobile_hold_output.h"
#include "actuator_core/sts3215_packet.h"
#include "timebase.h"
#include <stddef.h>
static actuator_bus_router_t router;
static UART_HandleTypeDef *right_uart;
static ServoQueuedTransaction transaction;
static uint32_t quiet_us,budget_us;
static volatile uint32_t gate_token,completed_us;
static volatile uint8_t tx_done,error,launching;
static void clear_events(void) {
    uint32_t mask=__get_PRIMASK();__disable_irq();tx_done=0;error=0;__DMB();if(!mask)__enable_irq();
}
bool MobileHoldOutput_Configure(UART_HandleTypeDef *uart,uint32_t budget,uint32_t quiet) {
    if(right_uart || !uart || !quiet || !uart->Init.BaudRate || budget>=UINT32_C(0x80000000) ||
       (UINT64_C(26)*10000000+uart->Init.BaudRate-1)/uart->Init.BaudRate+quiet>=budget ||
       !ServoTransport_ServiceAllowed() || !ServoTransport_Register(uart))return false;
    actuator_bus_router_init(&router);right_uart=uart;quiet_us=quiet;budget_us=budget;return true;
}
actuator_bus_router_t *MobileHoldOutput_Router(void){return right_uart?&router:NULL;}
void MobileHoldOutput_OnTxComplete(UART_HandleTypeDef *uart) {
    if(uart!=right_uart || !gate_token || uart->gState!=HAL_UART_STATE_READY)return;
    if(tx_done)error=1;
    completed_us=Timebase_NowUs();tx_done=1;
}
void MobileHoldOutput_OnUartError(UART_HandleTypeDef *uart){if(uart==right_uart && gate_token)error=1;}
static void fault(void){
    router.inhibited=true;router.bus.faulted=true;actuator_shared_bus_request_stop(&router.bus);
    ServoTransport_RequestStop(right_uart);
}
static bool is_hold(const uint8_t *p,size_t n) {
    if(n!=26 || p[0]!=255 || p[1]!=255 || p[2]!=254 || p[3]!=22 || p[4]!=131 ||
       p[5]!=42 || p[6]!=2 || p[25]!=actuator_sts3215_checksum(p+2,23))return false;
    for(unsigned i=0;i<6;i++)if(p[7+3*i]!=i+1 || ((uint16_t)p[8+3*i]|((uint16_t)p[9+3*i]<<8))>=4096)return false;
    return true;
}
void MobileHoldOutput_Poll(void) {
    if(!right_uart || launching)return;
    uint32_t now=Timebase_NowUs();actuator_shared_bus_poll(&router.bus,now);
    uint32_t mask=__get_PRIMASK();__disable_irq();uint8_t done=tx_done,bad=error;uint32_t at=completed_us;
    if(!mask)__enable_irq();
    if(bad || router.bus.faulted){fault();return;}
    if(transaction.active) {
        if(!done || now-at<quiet_us)return;
        HAL_StatusTypeDef result=ServoTransport_CompleteQueued(&transaction,true);
        if(result==HAL_BUSY)return;
        if(result!=HAL_OK){fault();return;}
        gate_token=0;clear_events();
    }
    const uint8_t *bytes;size_t length;actuator_bus_work_t kind;
    if(!ServoTransport_ClaimQueued(right_uart,&router,0,&transaction,&bytes,&length,&kind))return;
    gate_token=transaction.gate_token;clear_events();
    if(kind!=ACTUATOR_BUS_WORK_STOP || router.bus.budget_us!=budget_us || !is_hold(bytes,length)) {
        (void)ServoTransport_CompleteQueued(&transaction,false);fault();return;
    }
    launching=1;
    HAL_StatusTypeDef result=ServoTransport_TransmitDMA(right_uart,gate_token,bytes,(uint16_t)length);
    launching=0;
    if(result!=HAL_OK){(void)ServoTransport_CompleteQueued(&transaction,false);fault();}
}
bool MobileHoldOutput_Recover(bool quiet) {
    if(!right_uart || launching || !transaction.active || !quiet)return false;
    if(ServoTransport_RecoverQueued(&transaction,true)!=HAL_OK)return false;
    gate_token=0;clear_events();return true;
}
