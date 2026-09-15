#include "mobile_servo_output.h"
#include "control_tick.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t elapsed,origin,tx_at;
static uint8_t packet[32];static uint16_t packet_size;
static bool automatic=true,clock_valid=true;
static UART_HandleTypeDef left={.gState=HAL_UART_STATE_READY},right={.gState=HAL_UART_STATE_READY};
static actuator_mobile_supervisor_t supervisor;
/* This fixture supplies external feedback; integrated reader is tested separately. */
bool MobileServoFeedback_AcceptsOutput(const actuator_mobile_supervisor_t *s,const actuator_mobile_output_config_t *c){(void)s;(void)c;return true;}
static const actuator_mobile_config_t limits={{1000,1000,1000,100},2,200,50,0,500000};
static const actuator_mobile_output_config_t config={{5000,1000,200,10000,20000,5000,500},500,400,600,4000,{1,1,1,1}};
uint32_t Timebase_NowUs(void){return origin+elapsed;}
uint32_t HAL_GetTick(void){return elapsed/1000;}
bool ControlTick_PeekEpoch(uint32_t *epoch){*epoch=origin+(elapsed/5000)*5000;return clock_valid;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){
    CHECK(n<=sizeof(packet));memcpy(packet,p,n);packet_size=n;tx_at=elapsed;
    if(u->fail_tx)return HAL_ERROR;
    u->sent++;u->gState=HAL_UART_STATE_BUSY_TX;
    if(u->immediate){u->gState=HAL_UART_STATE_READY;MobileServoOutput_OnTxComplete(u);}
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit_DMA(u,p,n);}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t timeout){
    (void)u;(void)p;(void)n;(void)timeout;CHECK(false);return HAL_ERROR;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){
    u->aborted++;if(u->fail_abort)return HAL_ERROR;u->gState=HAL_UART_STATE_READY;return HAL_OK;
}
static void feedback(void){
    actuator_mobile_feedback_t f={.observed_ms=HAL_GetTick(),.lift_position_um=100000,
        .velocity_modes_verified=true,.lift_homed=true,.hardware_ok=true};
    CHECK(actuator_mobile_feedback(&supervisor,&f,HAL_GetTick()));
}
static void setup(void){
    CHECK(actuator_mobile_init(&supervisor,&limits));feedback();
    CHECK(ServoTransport_Register(&right));
    CHECK(!MobileServoOutput_Configure(&left,&supervisor,&config,50,false));
    left.Init.BaudRate=115200;CHECK(!MobileServoOutput_Configure(&left,&supervisor,&config,50,true));
    left.Init.BaudRate=1000000;CHECK(MobileServoOutput_Configure(&left,&supervisor,&config,50,true));
    CHECK(!MobileServoOutput_Configure(&left,&supervisor,&config,50,true));
    CHECK(!ServoTransport_ServiceAllowed());
    CHECK(MobileServoOutput_CommandAllowed(&supervisor));
    CHECK(!MobileServoOutput_CommandAllowed(NULL));
    CHECK(actuator_mobile_arm(&supervisor,1,HAL_GetTick()));
    int32_t values[4]={100,-200,300,20};CHECK(actuator_mobile_command(&supervisor,1,1,values,HAL_GetTick()));
}
static void poll(void){
    if(automatic && left.gState==HAL_UART_STATE_BUSY_TX && elapsed-tx_at>=200){
        left.gState=HAL_UART_STATE_READY;MobileServoOutput_OnTxComplete(&left);
    }
    MobileServoOutput_Poll();
}
static void advance(uint32_t until){for(;elapsed<until;elapsed+=100)poll();poll();}
static void check_zero(void){
    CHECK(packet_size==20 && packet[5]==46 && packet[6]==2);
    for(unsigned i=0;i<4;i++){CHECK(packet[7+3*i]==8+i);CHECK(packet[8+3*i]==0 && packet[9+3*i]==0);}
}
int main(int argc,char **argv){
    CHECK(argc==2);unsigned which=(unsigned)atoi(argv[1]);uint32_t token;
    if(which==0){MobileServoOutput_Poll();MobileServoOutput_Stop();MobileServoOutput_OnTxComplete(&left);
        CHECK(!MobileServoOutput_State()->configured && left.sent==0);return 0;}
    if(which==11)origin=UINT32_MAX-10000u;
    setup();
    switch(which){
    case 1: /* UART is retained through TX callback and quiet time. */
        advance(1000);CHECK(left.sent==1 && packet[7]==8 && packet[5]==46);
        CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_ARM,&token));
        elapsed=1200;poll();CHECK(!ServoTransport_Idle(&left));
        CHECK(ServoTransport_Begin(&right,ACTUATOR_BUS_WORK_ARM,&token));
        CHECK(ServoTransport_End(&right,token,false)==HAL_OK);
        advance(2000);CHECK(MobileServoOutput_State()->writes_completed==2);break;
    case 2: /* A legacy owner cannot be preempted by mobile output. */
        CHECK(ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_ARM,&token));
        advance(2000);CHECK(left.sent==0);
        CHECK(ServoTransport_End(&left,token,false)==HAL_OK);
        advance(3000);CHECK(left.sent==2);break;
    case 3: /* Missing callback: retain ownership until verified abort/recovery. */
        automatic=false;advance(1600);CHECK(MobileServoOutput_State()->router.bus.faulted);
        CHECK(!MobileServoOutput_Recover(false));left.fail_abort=1;
        CHECK(!MobileServoOutput_Recover(true));CHECK(MobileServoOutput_State()->router.bus.active);
        left.fail_abort=0;CHECK(MobileServoOutput_Recover(true));automatic=true;
        advance(2500);check_zero();CHECK(MobileServoOutput_State()->stop_written);break;
    case 4: /* Callback inside HAL start cannot release the transaction. */
        left.immediate=1;advance(1000);CHECK(left.sent==1 && !ServoTransport_Idle(&left));
        advance(1300);CHECK(MobileServoOutput_State()->writes_completed==2);break;
    case 5:
        left.fail_tx=1;advance(1000);CHECK(MobileServoOutput_State()->fault==MOBILE_OUTPUT_TRANSPORT);
        CHECK(MobileServoOutput_State()->router.bus.active && left.sent==0);
        CHECK(MobileServoOutput_Recover(true));left.fail_tx=0;advance(2000);check_zero();break;
    case 6: /* No fresh feedback is manufactured by a successful write. */
        advance(51000);CHECK(supervisor.state==ACTUATOR_MOBILE_STOP_LATCHED);
        CHECK(supervisor.reason==ACTUATOR_MOBILE_REASON_FEEDBACK);check_zero();break;
    case 7: /* Late poll/clock loss closes normal output, including queued lift. */
        advance(1000);elapsed=1600;poll();CHECK(MobileServoOutput_State()->fault==MOBILE_OUTPUT_SCHEDULE);
        CHECK(!MobileServoOutput_State()->router.jobs[ACTUATOR_BUS_WORK_LIFT].pending);
        if(MobileServoOutput_State()->router.bus.active)CHECK(MobileServoOutput_Recover(true));
        advance(2500);check_zero();break;
    case 8: /* Stop preempts pending wheel/lift bytes; its TX is not stop proof. */
        advance(1100);MobileServoOutput_Stop();CHECK(!MobileServoOutput_State()->stop_written);
        advance(2300);check_zero();CHECK(MobileServoOutput_State()->stop_written);
        CHECK(!MobileServoOutput_CommandAllowed(&supervisor));
        CHECK(!MobileServoOutput_Rearm(2,false));CHECK(!MobileServoOutput_Rearm(1,true));
        CHECK(!MobileServoOutput_Rearm(2,true)); /* old zero sample predates STOP TX */
        feedback();CHECK(MobileServoOutput_Rearm(2,true));
        CHECK(supervisor.state==ACTUATOR_MOBILE_READY && supervisor.target_raw[0]==0);
        advance(3500);CHECK(MobileServoOutput_State()->fault==MOBILE_OUTPUT_OK);break;
    case 9:
        automatic=false;advance(1000);left.gState=HAL_UART_STATE_READY;
        MobileServoOutput_OnTxComplete(&left);MobileServoOutput_OnTxComplete(&left);poll();
        CHECK(MobileServoOutput_State()->fault==MOBILE_OUTPUT_TRANSPORT);
        CHECK(!ServoTransport_Idle(&left));break;
    case 10:
        advance(1000);MobileServoOutput_OnUartError(&right);CHECK(MobileServoOutput_State()->fault==MOBILE_OUTPUT_OK);
        MobileServoOutput_OnUartError(&left);poll();CHECK(MobileServoOutput_State()->fault==MOBILE_OUTPUT_TRANSPORT);
        CHECK(MobileServoOutput_State()->router.bus.active);break;
    case 12: /* Queue expiry is detected even while the legacy owner blocks dequeue. */
        CHECK(ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_ARM,&token));
        advance(4700);CHECK(left.sent==0 && MobileServoOutput_State()->fault==MOBILE_OUTPUT_QUEUE);
        CHECK(ServoTransport_End(&left,token,false)==HAL_OK);
        advance(5700);check_zero();CHECK(MobileServoOutput_State()->stop_written);break;
    case 13:
        clock_valid=false;poll();CHECK(MobileServoOutput_State()->fault==MOBILE_OUTPUT_SCHEDULE);
        advance(1000);check_zero();CHECK(MobileServoOutput_State()->stop_written);break;
    case 11:
        advance(24000);CHECK(MobileServoOutput_State()->fault==MOBILE_OUTPUT_OK);
        CHECK(MobileServoOutput_State()->writes_completed==5);break;
    default:CHECK(false);
    }
    printf("mobile board writer scenario %u passed\n",which);return 0;
}
