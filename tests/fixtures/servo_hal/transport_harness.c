#include "servo_transport.h"
#include "bimanual_servo_dispatch.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);return 1;}}while(0)
static uint32_t tick;
uint32_t Timebase_NowUs(void){return tick++;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){
    (void)p;(void)n;
    if(u->fail_tx)return HAL_ERROR;
    u->sent++;u->gState=HAL_UART_STATE_BUSY_TX;
    if(u->immediate){u->gState=HAL_UART_STATE_READY;BimanualServoDispatch_OnTxComplete(u);}
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit_DMA(u,p,n);}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t timeout){
    (void)timeout;HAL_StatusTypeDef result=HAL_UART_Transmit_DMA(u,p,n);u->gState=HAL_UART_STATE_READY;return result;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){
    u->aborted++;if(u->fail_abort)return HAL_ERROR;u->gState=HAL_UART_STATE_READY;return HAL_OK;
}
static void complete(UART_HandleTypeDef *u){u->gState=HAL_UART_STATE_READY;BimanualServoDispatch_OnTxComplete(u);}
int main(int argc,char **argv){
    UART_HandleTypeDef left={.gState=HAL_UART_STATE_READY},right={.gState=HAL_UART_STATE_READY};
    uint32_t a=0,b=0,stale;uint8_t bytes[]={255,255,1};uint16_t pose[6]={2048,2048,2048,2048,2048,2048};
    CHECK(argc==2);int scenario=atoi(argv[1]);BimanualServoDispatch_Init(&left,&right);
    switch(scenario){
    case 14:
        CHECK(!ServoTransport_SetPeriodicMode(true,false));
        CHECK(ServoTransport_BeginService(&left,ACTUATOR_BUS_WORK_ARM,&a));
        CHECK(!ServoTransport_SetPeriodicMode(true,true));
        CHECK(ServoTransport_End(&left,a,false)==HAL_OK);
        CHECK(ServoTransport_SetPeriodicMode(true,true));
        CHECK(!ServoTransport_ServiceAllowed());
        CHECK(!ServoTransport_BeginService(&left,ACTUATOR_BUS_WORK_ARM,&a));
        CHECK(ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_WHEELS,&a));
        CHECK(ServoTransport_Transmit(&left,a,bytes,3,100)==HAL_ERROR);
        CHECK(ServoTransport_TransmitDMA(&left,a,bytes,3)==HAL_OK);
        CHECK(!ServoTransport_SetPeriodicMode(false,true));
        left.gState=HAL_UART_STATE_READY;
        CHECK(ServoTransport_End(&left,a,false)==HAL_OK);
        CHECK(ServoTransport_SetPeriodicMode(false,true));
        CHECK(ServoTransport_ServiceAllowed());break;
    case 0: /* TX completion does NOT finish a read transaction. */
        CHECK(ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_FEEDBACK,&a));
        CHECK(ServoTransport_Transmit(&left,a,bytes,3,5)==HAL_OK);
        CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_WHEELS,&b));
        CHECK(ServoTransport_Begin(&right,ACTUATOR_BUS_WORK_ARM,&b));
        CHECK(ServoTransport_Register(&left) && !ServoTransport_Idle(&left));
        CHECK(ServoTransport_Transmit(&left,a+1,bytes,3,5)==HAL_ERROR && left.sent==1);
        left.wire_busy=1;
        CHECK(ServoTransport_End(&left,a,false)==HAL_BUSY);
        left.wire_busy=0;
        CHECK(ServoTransport_End(&left,a,false)==HAL_OK);stale=a;
        CHECK(ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_LIFT,&a));
        CHECK(ServoTransport_End(&left,stale,false)==HAL_ERROR && !ServoTransport_Idle(&left));
        CHECK(ServoTransport_End(&left,a,false)==HAL_OK);break;
    case 1: /* Reserve both before starting either. */
        CHECK(ServoTransport_Begin(&right,ACTUATOR_BUS_WORK_FEEDBACK,&b));
        CHECK(BimanualServoDispatch_Launch(pose,pose,0,0)==HAL_BUSY);
        CHECK(left.sent==0 && right.sent==0 && ServoTransport_Idle(&left));
        CHECK(!ServoTransport_Idle(&right));
        BimanualServoDispatch_Stop();CHECK(right.aborted==0);
        CHECK(ServoTransport_End(&right,b,false)==HAL_OK);break;
    case 2: /* Deferred main-loop completion, independent bus release. */
        CHECK(BimanualServoDispatch_Launch(pose,pose,0,0)==HAL_OK);
        complete(&left);CHECK(!ServoTransport_Idle(&left));
        BimanualServoDispatch_Poll();CHECK(ServoTransport_Idle(&left) && !ServoTransport_Idle(&right));
        CHECK(ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_WHEELS,&a));
        complete(&right);BimanualServoDispatch_Poll();
        CHECK(BimanualServoDispatch_GetSnapshot()->completed_count==1);
        CHECK(!BimanualServoDispatch_Ready());
        CHECK(ServoTransport_End(&left,a,false)==HAL_OK);CHECK(BimanualServoDispatch_Ready());break;
    case 3: /* Completion may arrive inside HAL launch before dispatch_begin. */
        left.immediate=right.immediate=1;
        CHECK(BimanualServoDispatch_Launch(pose,pose,0,0)==HAL_OK);
        CHECK(BimanualServoDispatch_GetSnapshot()->completed_count==1 && BimanualServoDispatch_Ready());break;
    case 4: /* Partial pair launch failure cleans only owned buses. */
        right.fail_tx=1;CHECK(BimanualServoDispatch_Launch(pose,pose,0,0)==HAL_ERROR);
        CHECK(left.sent==1 && right.sent==0 && left.aborted==1);
        CHECK(ServoTransport_Idle(&left) && ServoTransport_Idle(&right) && BimanualServoDispatch_Faulted());break;
    case 5: /* Failed abort cannot silently release an active transmitter. */
        CHECK(BimanualServoDispatch_Launch(pose,pose,0,0)==HAL_OK);
        left.fail_abort=1;BimanualServoDispatch_Stop();
        CHECK(!ServoTransport_Idle(&left) && BimanualServoDispatch_Faulted());
        CHECK(ServoTransport_Register(&left) && !ServoTransport_Idle(&left));
        left.fail_abort=0;BimanualServoDispatch_Stop();CHECK(ServoTransport_Idle(&left));break;
    case 6: /* Old callback after abort owns nothing. */
        CHECK(BimanualServoDispatch_Launch(pose,pose,0,0)==HAL_OK);
        BimanualServoDispatch_Stop();complete(&left);complete(&right);BimanualServoDispatch_Poll();
        CHECK(ServoTransport_Idle(&left) && ServoTransport_Idle(&right));
        CHECK(BimanualServoDispatch_GetSnapshot()->completed_count==0);break;
    case 7: /* A callback while hardware is still TX busy is not proof. */
        CHECK(BimanualServoDispatch_Launch(pose,pose,0,0)==HAL_OK);
        BimanualServoDispatch_OnTxComplete(&left);BimanualServoDispatch_Poll();
        CHECK(!ServoTransport_Idle(&left));complete(&left);complete(&right);BimanualServoDispatch_Poll();
        CHECK(BimanualServoDispatch_GetSnapshot()->completed_count==1);break;
    case 8: /* ISR error is consumed in main, with both owned outputs aborted. */
        CHECK(BimanualServoDispatch_Launch(pose,pose,0,0)==HAL_OK);
        BimanualServoDispatch_OnUartError(&left);CHECK(left.aborted==0);
        BimanualServoDispatch_Poll();CHECK(left.aborted==1 && right.aborted==1 && BimanualServoDispatch_Faulted());break;
    case 9: /* Duplicate unconsumed events latch; no false successful pair. */
        CHECK(BimanualServoDispatch_Launch(pose,pose,0,0)==HAL_OK);
        complete(&left);complete(&left);BimanualServoDispatch_Poll();
        CHECK(BimanualServoDispatch_Faulted() && BimanualServoDispatch_GetSnapshot()->completed_count==0);break;
    case 10: /* Real router -> board gate, with legacy read contention. */
    case 11: /* Deadline expiry retains ownership until explicit cleanup. */
    case 12: /* A stale queued completion cannot release a new transaction. */
    {
        actuator_bus_router_t router;ServoQueuedTransaction transfer={0},old;
        const uint8_t *packet;size_t length;actuator_bus_work_t kind;
        actuator_bus_router_init(&router);
        CHECK(actuator_bus_router_submit(&router,ACTUATOR_BUS_WORK_WHEELS,bytes,3,0,1000,100));
        CHECK(ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_FEEDBACK,&a));
        CHECK(!ServoTransport_ClaimQueued(&left,&router,100,&transfer,&packet,&length,&kind));
        CHECK(router.jobs[ACTUATOR_BUS_WORK_WHEELS].pending && !router.bus.active);
        CHECK(ServoTransport_End(&left,a,false)==HAL_OK);
        CHECK(ServoTransport_ClaimQueued(&left,&router,100,&transfer,&packet,&length,&kind));
        old=transfer;
        CHECK(kind==ACTUATOR_BUS_WORK_WHEELS && length==3);
        CHECK(ServoTransport_TransmitDMA(&left,transfer.gate_token,packet,(uint16_t)length)==HAL_OK);
        CHECK(ServoTransport_CompleteQueued(&transfer,true)==HAL_BUSY);
        CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_ARM,&a));
        if(scenario==11){
            tick=200; /* Missing TX completion must still expire the budget. */
            CHECK(ServoTransport_CompleteQueued(&transfer,true)==HAL_ERROR);
            CHECK(router.bus.faulted && !ServoTransport_Idle(&left));
            CHECK(ServoTransport_RecoverQueued(&transfer,false)==HAL_ERROR);
            CHECK(ServoTransport_RecoverQueued(&transfer,true)==HAL_OK);
            CHECK(router.inhibited && router.bus.stop_pending && !ServoTransport_Idle(&left));
            CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_ARM,&a));
            CHECK(!actuator_bus_router_resume(&router,true));
            CHECK(actuator_bus_router_submit(&router,ACTUATOR_BUS_WORK_STOP,bytes,3,tick,tick+1000,100));
            CHECK(ServoTransport_ClaimQueued(&left,&router,100,&transfer,&packet,&length,&kind));
            CHECK(kind==ACTUATOR_BUS_WORK_STOP);
            CHECK(ServoTransport_Transmit(&left,transfer.gate_token,packet,(uint16_t)length,1)==HAL_OK);
            CHECK(ServoTransport_CompleteQueued(&transfer,true)==HAL_OK);
            CHECK(!ServoTransport_ResumeQueued(&left,&router,false));
            CHECK(ServoTransport_ResumeQueued(&left,&router,true) && ServoTransport_Idle(&left));
        } else {
            left.gState=HAL_UART_STATE_READY;
            CHECK(ServoTransport_CompleteQueued(&transfer,true)==HAL_OK);
            CHECK(ServoTransport_Idle(&left) && !router.bus.active);
            CHECK(ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_ARM,&a));
            CHECK(ServoTransport_CompleteQueued(&old,true)==HAL_ERROR && !ServoTransport_Idle(&left));
            if(scenario==12){
                ServoTransport_RequestStop(&left);
                CHECK(ServoTransport_End(&left,a,false)==HAL_OK);
                CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_WHEELS,&a));
                CHECK(!ServoTransport_Idle(&left));
                left.wire_busy=1;
                CHECK(ServoTransport_BeginCleanup(&left,&a));
                CHECK(ServoTransport_Transmit(&left,a,bytes,3,1)==HAL_ERROR);
                CHECK(!ServoTransport_BeginCleanup(&left,&b));
                left.wire_busy=0;
                CHECK(ServoTransport_End(&left,a,true)==HAL_OK);
                CHECK(!ServoTransport_Idle(&left)); /* STOP remains latched */
            }
        }
        break;
    }
    case 13: /* A job expiring before dispatch also blocks legacy writers. */
    {
        actuator_bus_router_t router;ServoQueuedTransaction transfer={0};
        const uint8_t *packet;size_t length;actuator_bus_work_t kind;
        actuator_bus_router_init(&router);
        CHECK(actuator_bus_router_submit(&router,ACTUATOR_BUS_WORK_LIFT,bytes,3,0,1000,100));
        tick=1001;
        CHECK(!ServoTransport_ClaimQueued(&left,&router,100,&transfer,&packet,&length,&kind));
        CHECK(router.expired_jobs==1 && router.bus.stop_pending && !router.bus.active);
        CHECK(!ServoTransport_Idle(&left) && left.sent==0);
        CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_ARM,&a));
        break;
    }
    default:return 2;
    }
    return 0;
}
