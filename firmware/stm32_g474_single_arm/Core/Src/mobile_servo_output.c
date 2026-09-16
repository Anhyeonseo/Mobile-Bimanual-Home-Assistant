#include "mobile_servo_output.h"
#include "mobile_servo_feedback.h"
#include "control_tick.h"
#include "timebase.h"
#include <stddef.h>

static actuator_mobile_output_t output;
static UART_HandleTypeDef *left_uart;
static ServoQueuedTransaction transaction;
static actuator_bus_work_t active_kind;
static uint32_t quiet_us;
static volatile uint32_t completion_us;
static volatile uint8_t tx_complete,uart_error,launching;
static volatile uint32_t active_gate_token;

static bool budget_fits(uint32_t baud,uint32_t length,uint32_t quiet,uint32_t budget) {
    /* Both configured servo UARTs are 8N1. Include every wire bit plus quiet. */
    return baud && ((uint64_t)length*10000000u+baud-1u)/baud+quiet<budget;
}
static void clear_events(void) {
    uint32_t mask=__get_PRIMASK();__disable_irq();
    tx_complete=0;uart_error=0;__DMB();
    if(mask==0)__enable_irq();
}
bool MobileServoOutput_Configure(UART_HandleTypeDef *uart,
    actuator_mobile_supervisor_t *s,const actuator_mobile_output_config_t *c,
    uint32_t quiet,bool proof) {
    actuator_mobile_output_t candidate;
    if(output.configured || uart==NULL || c==NULL || !proof || !quiet ||
       !MobileServoFeedback_AcceptsOutput(s,c) ||
       c->schedule.arm_period_us!=CONTROL_TICK_PERIOD_US ||
       !budget_fits(uart->Init.BaudRate,17,quiet,c->wheels_budget_us) ||
       !budget_fits(uart->Init.BaudRate,11,quiet,c->lift_budget_us) ||
       !budget_fits(uart->Init.BaudRate,20,quiet,c->stop_budget_us) ||
       quiet>=c->wheels_budget_us || quiet>=c->lift_budget_us || quiet>=c->stop_budget_us ||
       !actuator_mobile_output_init(&candidate,s,c,Timebase_NowUs()) ||
       !actuator_mobile_axes_stopped(s,HAL_GetTick()) ||
       !ServoTransport_Register(uart) || !ServoTransport_SetPeriodicMode(true,proof))return false;
    output=candidate;left_uart=uart;quiet_us=quiet;return true;
}
const actuator_mobile_output_t *MobileServoOutput_State(void) {return &output;}
bool MobileServoOutput_CommandAllowed(const actuator_mobile_supervisor_t *s) {
    return output.configured && output.supervisor==s && !output.stop_requested &&
        output.fault==MOBILE_OUTPUT_OK && (output.lift_control==NULL || !output.lift_control->active) && !output.router.inhibited && !output.router.bus.faulted;
}
void MobileServoOutput_Stop(void) {
    actuator_mobile_output_stop(&output,MOBILE_OUTPUT_OK);
    if(output.configured)ServoTransport_RequestStop(left_uart);
}
void MobileServoOutput_OnTxComplete(UART_HandleTypeDef *uart) {
    if(!output.configured || uart!=left_uart || !active_gate_token ||
       uart->gState!=HAL_UART_STATE_READY)return;
    if(tx_complete)uart_error=1;
    completion_us=Timebase_NowUs();tx_complete=1;
}
void MobileServoOutput_OnUartError(UART_HandleTypeDef *uart) {
    if(output.configured && uart==left_uart && active_gate_token)uart_error=1;
}
void MobileServoOutput_Poll(void) {
    if(!output.configured || launching)return;
    uint32_t now=Timebase_NowUs(),epoch;
    if(!ControlTick_PeekEpoch(&epoch)) {
        actuator_mobile_output_stop(&output,MOBILE_OUTPUT_SCHEDULE);epoch=now;
    }
    actuator_mobile_output_poll(&output,now,HAL_GetTick(),epoch);
    uint8_t complete,error;uint32_t done;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    complete=tx_complete;error=uart_error;done=completion_us;
    if(mask==0)__enable_irq();
    if(error || output.router.bus.faulted) {
        actuator_mobile_output_stop(&output,MOBILE_OUTPUT_TRANSPORT);
        ServoTransport_RequestStop(left_uart);return; /* explicit RX/abort recovery */
    }
    if(transaction.active) {
        if(!complete || now-done<quiet_us)return;
        HAL_StatusTypeDef result=ServoTransport_CompleteQueued(&transaction,true);
        if(result==HAL_BUSY)return;
        if(result!=HAL_OK) {
            actuator_mobile_output_stop(&output,MOBILE_OUTPUT_TRANSPORT);return;
        }
        active_gate_token=0;clear_events();
        actuator_mobile_output_completed(&output,active_kind,HAL_GetTick());
    }
    const uint8_t *bytes;size_t length;actuator_bus_work_t kind;
    if(!ServoTransport_ClaimQueued(left_uart,&output.router,output.available_us,
            &transaction,&bytes,&length,&kind))return;
    if(kind!=ACTUATOR_BUS_WORK_STOP) {
        /* Recheck after claiming: completion/claim overhead consumes the same
         * arm window. A late claim must never start a packet across that slot. */
        now=Timebase_NowUs();
        uint32_t phase;
        if(!ControlTick_PeekEpoch(&epoch) || (phase=now-epoch)>=CONTROL_TICK_PERIOD_US ||
           phase<output.config.schedule.arm_reserved_us ||
           phase+output.router.bus.budget_us+output.config.schedule.guard_us>CONTROL_TICK_PERIOD_US) {
            actuator_mobile_output_stop(&output,MOBILE_OUTPUT_SCHEDULE);
            (void)ServoTransport_CompleteQueued(&transaction,false);return;
        }
    }
    clear_events();active_kind=kind;active_gate_token=transaction.gate_token;
    launching=1;
    HAL_StatusTypeDef result=ServoTransport_TransmitDMA(left_uart,transaction.gate_token,bytes,(uint16_t)length);
    launching=0;
    if(result!=HAL_OK) {
        actuator_mobile_output_stop(&output,MOBILE_OUTPUT_TRANSPORT);
        (void)ServoTransport_CompleteQueued(&transaction,false);
        ServoTransport_RequestStop(left_uart);
    }
}
bool MobileServoOutput_Recover(bool quiet) {
    if(!output.configured || launching || !transaction.active || !quiet)return false;
    if(ServoTransport_RecoverQueued(&transaction,true)!=HAL_OK)return false;
    active_gate_token=0;clear_events();return true;
}
bool MobileServoOutput_Rearm(uint32_t session,bool proof) {
    if(!output.configured || transaction.active || !proof || !output.stop_written ||
       !actuator_mobile_measured_stopped(output.supervisor,HAL_GetTick()) ||
       !ServoTransport_ResumeQueued(left_uart,&output.router,true))return false;
    if(!actuator_mobile_output_rearm(&output,session,Timebase_NowUs(),HAL_GetTick(),proof)) {
        output.router.inhibited=true;ServoTransport_RequestStop(left_uart);return false;
    }
    return true;
}

bool MobileServoOutput_BindLift(actuator_lift_endpoint_t *endpoint) {
    return actuator_mobile_output_bind_lift(&output,endpoint);
}

actuator_bus_router_t *MobileServoOutput_StopRouter(void) {
    return output.configured && output.stop_requested ? &output.router : NULL;
}
