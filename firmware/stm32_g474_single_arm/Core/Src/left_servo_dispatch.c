/* Six-axis implementation of the resident dispatch interface. No right UART
 * is registered, leased, written, or reported as having completed a transfer. */
#include "bimanual_servo_dispatch.h"
#include "actuator_core/motor_groups.h"
#include "servo_transport.h"
#include "timebase.h"
#include <stddef.h>
#include <string.h>
#if !HOST_LEFT_ONLY_BUILD
#error "Left dispatcher requires the dedicated left-only firmware identity"
#endif
static UART_HandleTypeDef *left_uart;
static actuator_bimanual_dispatch_snapshot_t state;
static uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
static volatile uint32_t token;
static volatile uint8_t launching, completed, failed;

static void clear_events(void) {
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    completed=0U; failed=0U; __DMB();
    if (!irq) __enable_irq();
}
void BimanualServoDispatch_Init(UART_HandleTypeDef *left, UART_HandleTypeDef *right) {
    (void)right; left_uart=left;
    (void)ServoTransport_Register(left);
    memset(&state,0,sizeof(state)); token=0U; launching=0U; clear_events();
}
void BimanualServoDispatch_LatchFault(void) {
    launching=0U;
    if (token && ServoTransport_End(left_uart,token,true)==HAL_OK) token=0U;
    if (!state.faulted) state.failure_count++;
    state.faulted=true; state.active=false; clear_events();
}
void BimanualServoDispatch_Poll(void) {
    if (launching) return;
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    uint8_t done=completed, error=failed; completed=0U; failed=0U; __DMB();
    if (!irq) __enable_irq();
    if (error) {BimanualServoDispatch_LatchFault(); return;}
    if (done) {
        if (!state.active || !token || ServoTransport_End(left_uart,token,false)!=HAL_OK) {
            BimanualServoDispatch_LatchFault(); return;
        }
        token=0U; state.left_complete=true; state.active=false; state.completed_count++;
    }
}
HAL_StatusTypeDef BimanualServoDispatch_Launch(const uint16_t left[6], const uint16_t right[6],
    uint32_t tick, uint32_t started) {
    (void)right; size_t size=0; uint32_t lease;
    BimanualServoDispatch_Poll();
    if (!left_uart || !left || state.active || state.faulted) return HAL_ERROR;
    if (actuator_motor_group_positions(ACTUATOR_GROUP_LEFT_ARM,left,6U,packet,&size)!=ACTUATOR_GROUP_OK) {
        BimanualServoDispatch_LatchFault(); return HAL_ERROR;
    }
    if (!ServoTransport_Begin(left_uart,ACTUATOR_BUS_WORK_ARM,&lease)) return HAL_BUSY;
    token=lease; clear_events(); launching=1U;
    uint32_t now=Timebase_NowUs();
    if (ServoTransport_TransmitDMA(left_uart,token,packet,(uint16_t)size)!=HAL_OK) {
        BimanualServoDispatch_LatchFault(); return HAL_ERROR;
    }
    state.last_control_tick_ms=tick; state.last_left_start_us=now;
    if (now-started>state.maximum_launch_lateness_us) state.maximum_launch_lateness_us=now-started;
    state.left_complete=false; state.active=true; state.launch_count++;
    launching=0U; BimanualServoDispatch_Poll();
    return state.faulted?HAL_ERROR:HAL_OK;
}
void BimanualServoDispatch_OnTxComplete(UART_HandleTypeDef *uart) {
    if (uart!=left_uart || !uart || !token || uart->gState!=HAL_UART_STATE_READY) return;
    if (completed) failed=1U;
    completed=1U;
}
void BimanualServoDispatch_OnUartError(UART_HandleTypeDef *uart) {
    if (uart==left_uart && token) failed=1U;
}
void BimanualServoDispatch_Stop(void) {
    BimanualServoDispatch_Poll();
    if (state.active || token) BimanualServoDispatch_LatchFault();
}
uint8_t BimanualServoDispatch_Faulted(void) {BimanualServoDispatch_Poll();return state.faulted;}
uint8_t BimanualServoDispatch_Ready(void) {
    BimanualServoDispatch_Poll();return left_uart && !state.active && !state.faulted && ServoTransport_Idle(left_uart);
}
const actuator_bimanual_dispatch_snapshot_t *BimanualServoDispatch_GetSnapshot(void) {
    BimanualServoDispatch_Poll();return &state;
}
