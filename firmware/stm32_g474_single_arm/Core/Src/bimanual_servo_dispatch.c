#include "bimanual_servo_dispatch.h"
#include "actuator_core/motor_groups.h"
#include "servo_transport.h"
#include "timebase.h"
#include <stddef.h>

static UART_HandleTypeDef *left_bus_uart;
static UART_HandleTypeDef *right_bus_uart;
static actuator_bimanual_dispatch_t dispatch_state;
static uint8_t left_packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
static uint8_t right_packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
static volatile uint32_t left_token, right_token;
static volatile uint8_t launch_in_progress, tx_events, uart_error_event;
#if HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD
static uint8_t right_dma_fault_injection_consumed;
#endif

static void clear_events(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    tx_events = 0U;
    uart_error_event = 0U;
    __DMB();
    if (mask == 0U) __enable_irq();
}

static void release_bus(UART_HandleTypeDef *uart,
    volatile uint32_t *token, bool abort_tx)
{
    if (*token != 0U && ServoTransport_End(uart, *token, abort_tx) == HAL_OK)
        *token = 0U;
}

void BimanualServoDispatch_Init(UART_HandleTypeDef *left_uart,
    UART_HandleTypeDef *right_uart)
{
    /* Boot-only initialization; transport registration never resets a lease. */
    left_bus_uart = left_uart;
    right_bus_uart = right_uart;
    (void)ServoTransport_Register(left_uart);
    (void)ServoTransport_Register(right_uart);
    actuator_bimanual_dispatch_init(&dispatch_state);
    launch_in_progress = 0U;
    clear_events();
#if HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD
    right_dma_fault_injection_consumed = 0U;
#endif
}

void BimanualServoDispatch_LatchFault(void)
{
    launch_in_progress = 0U;
    /* Abort only transactions this dispatcher actually owns. Telemetry has
     * its own response lifetime and is ended by the coordinated-stop caller. */
    release_bus(left_bus_uart, &left_token, true);
    release_bus(right_bus_uart, &right_token, true);
    clear_events();
    actuator_bimanual_dispatch_fail(&dispatch_state);
}

void BimanualServoDispatch_Poll(void)
{
    uint8_t events, error;
    uint32_t mask;
    if (launch_in_progress != 0U) return;
    mask = __get_PRIMASK();
    __disable_irq();
    events = tx_events;
    error = uart_error_event;
    tx_events = 0U;
    uart_error_event = 0U;
    __DMB();
    if (mask == 0U) __enable_irq();
    if (error != 0U) {
        BimanualServoDispatch_LatchFault();
        return;
    }
    if ((events & 1U) != 0U) {
        if (ServoTransport_End(left_bus_uart, left_token, false) != HAL_OK ||
            actuator_bimanual_dispatch_complete_left(&dispatch_state) != ACTUATOR_BIMANUAL_DISPATCH_OK) {
            BimanualServoDispatch_LatchFault();
            return;
        }
        left_token = 0U;
    }
    if ((events & 2U) != 0U) {
        if (ServoTransport_End(right_bus_uart, right_token, false) != HAL_OK ||
            actuator_bimanual_dispatch_complete_right(&dispatch_state) != ACTUATOR_BIMANUAL_DISPATCH_OK) {
            BimanualServoDispatch_LatchFault();
            return;
        }
        right_token = 0U;
    }
}

HAL_StatusTypeDef BimanualServoDispatch_Launch(
    const uint16_t left_positions[6], const uint16_t right_positions[6],
    uint32_t control_tick_ms, uint32_t control_tick_started_us)
{
    size_t left_length = 0U, right_length = 0U;
    uint32_t left_start_us, right_start_us, token;
    BimanualServoDispatch_Poll();
    if (left_bus_uart == NULL || right_bus_uart == NULL ||
        left_positions == NULL || right_positions == NULL ||
        !actuator_bimanual_dispatch_can_launch(&dispatch_state)) {
        actuator_bimanual_dispatch_fail(&dispatch_state);
        return HAL_ERROR;
    }
    if (actuator_motor_group_positions(ACTUATOR_GROUP_LEFT_ARM, left_positions,
            6U, left_packet, &left_length) != ACTUATOR_GROUP_OK ||
        actuator_motor_group_positions(ACTUATOR_GROUP_RIGHT_ARM, right_positions,
            6U, right_packet, &right_length) != ACTUATOR_GROUP_OK) {
        actuator_bimanual_dispatch_fail(&dispatch_state);
        return HAL_ERROR;
    }
    /* Reserve BOTH response-free buses before starting either arm. Busy
     * admission sends zero frames and does not corrupt an existing owner. */
    if (!ServoTransport_Begin(left_bus_uart, ACTUATOR_BUS_WORK_ARM, &token))
        return HAL_BUSY;
    left_token = token;
    if (!ServoTransport_Begin(right_bus_uart, ACTUATOR_BUS_WORK_ARM, &token)) {
        release_bus(left_bus_uart, &left_token, false);
        return HAL_BUSY;
    }
    right_token = token;
    clear_events();
    launch_in_progress = 1U;
    left_start_us = Timebase_NowUs();
    if (ServoTransport_TransmitDMA(left_bus_uart, left_token,
            left_packet, (uint16_t)left_length) != HAL_OK) goto failed;
#if HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD
    if (right_dma_fault_injection_consumed == 0U &&
        dispatch_state.snapshot.completed_count >= 8U) {
        right_dma_fault_injection_consumed = 1U;
        goto failed;
    }
#endif
    right_start_us = Timebase_NowUs();
    if (ServoTransport_TransmitDMA(right_bus_uart, right_token,
            right_packet, (uint16_t)right_length) != HAL_OK) goto failed;
    if (actuator_bimanual_dispatch_begin(&dispatch_state, control_tick_ms,
            control_tick_started_us, left_start_us, right_start_us) != ACTUATOR_BIMANUAL_DISPATCH_OK)
        goto failed;
    launch_in_progress = 0U;
    BimanualServoDispatch_Poll();
    return dispatch_state.snapshot.faulted ? HAL_ERROR : HAL_OK;
failed:
    BimanualServoDispatch_LatchFault();
    return HAL_ERROR;
}

void BimanualServoDispatch_OnTxComplete(UART_HandleTypeDef *uart)
{
    uint8_t event = 0U;
    /* ISR records facts only. HAL has completed the TX before this callback.
     * Ownership is released by Poll in the main loop, never by the ISR. */
    if (uart == NULL || uart->gState != HAL_UART_STATE_READY) return;
    if (uart == left_bus_uart && left_token != 0U) event = 1U;
    if (uart == right_bus_uart && right_token != 0U) event = 2U;
    if ((tx_events & event) != 0U) uart_error_event = 1U;
    tx_events |= event;
}

void BimanualServoDispatch_OnUartError(UART_HandleTypeDef *uart)
{
    if ((uart == left_bus_uart && left_token != 0U) ||
        (uart == right_bus_uart && right_token != 0U)) uart_error_event = 1U;
}

void BimanualServoDispatch_Stop(void)
{
    bool active = dispatch_state.snapshot.active;
    launch_in_progress = 0U;
    release_bus(left_bus_uart, &left_token, true);
    release_bus(right_bus_uart, &right_token, true);
    clear_events();
    if (active || left_token != 0U || right_token != 0U)
        actuator_bimanual_dispatch_fail(&dispatch_state);
}

uint8_t BimanualServoDispatch_Faulted(void)
{
    BimanualServoDispatch_Poll();
    return dispatch_state.snapshot.faulted ? 1U : 0U;
}

uint8_t BimanualServoDispatch_Ready(void)
{
    BimanualServoDispatch_Poll();
    return actuator_bimanual_dispatch_can_launch(&dispatch_state) &&
        ServoTransport_Idle(left_bus_uart) && ServoTransport_Idle(right_bus_uart);
}

const actuator_bimanual_dispatch_snapshot_t *BimanualServoDispatch_GetSnapshot(void)
{
    BimanualServoDispatch_Poll();
    return actuator_bimanual_dispatch_snapshot(&dispatch_state);
}
