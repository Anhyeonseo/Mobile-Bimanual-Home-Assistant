#include "mobile_servo_feedback.h"
#include "mobile_servo_output.h"
#include "servo_bus_reader.h"
#include "control_tick.h"
#include "timebase.h"
#include <string.h>

static actuator_mobile_feedback_reader_t reader;
static actuator_mobile_lift_evidence_t lift;
static bool have_lift, have_lift_stamp, request_sent, transport_quiet;
static UART_HandleTypeDef *left_uart;
static actuator_bus_schedule_config_t schedule;
static uint32_t period_us, budget_us, quiet_us, required_wire_us;
static uint32_t started_us, last_request_us, last_activity_us, cursor, reader_token;
static uint8_t request[8];
static volatile uint32_t gate_token, completion_us;
static volatile uint8_t tx_complete, uart_error, launching;

static void clear_events(void) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    tx_complete = 0; uart_error = 0; __DMB();
    if (!mask) __enable_irq();
}
static void fault(void) {
    actuator_mobile_feedback_fail(&reader);
    MobileServoOutput_Stop(); ServoTransport_RequestStop(left_uart);
}
bool MobileServoFeedback_Configure(UART_HandleTypeDef *uart,
    actuator_mobile_supervisor_t *s, const actuator_mobile_feedback_config_t *c,
    const actuator_bus_schedule_config_t *timing, uint32_t period, uint32_t budget,
    uint32_t quiet, bool proof) {
    actuator_bus_schedule_t validated;
    if (reader.configured || uart == NULL || c == NULL || timing == NULL || !proof ||
        !ServoTransport_ServiceAllowed() || !uart->Init.BaudRate || !period ||
        period >= UINT32_C(0x80000000) || !quiet ||
        timing->arm_period_us != CONTROL_TICK_PERIOD_US ||
        !actuator_bus_schedule_init(&validated, timing, Timebase_NowUs())) return false;
    uint64_t wire = (UINT64_C(29) * 10000000u + uart->Init.BaudRate - 1u) / uart->Init.BaudRate;
    if (wire + 2u * (uint64_t)quiet >= budget || budget >= period ||
        budget > timing->arm_period_us - timing->arm_reserved_us - timing->guard_us ||
        (uint64_t)budget >= (uint64_t)c->response_timeout_ms * 1000u ||
        8u * (uint64_t)period + budget >= (uint64_t)c->sample_timeout_ms * 1000u ||
        8u * (uint64_t)period + budget >= (uint64_t)c->mode_timeout_ms * 500u ||
        3u * (uint64_t)period > (uint64_t)c->maximum_sample_skew_ms * 1000u) return false;
    actuator_mobile_feedback_reader_t candidate;
    if (!actuator_mobile_feedback_reader_init(&candidate, s, c) ||
        !ServoTransport_Register(uart) || !ServoBus_PrepareReader(uart, proof)) return false;
    reader = candidate; left_uart = uart; schedule = *timing;
    period_us = period; budget_us = budget; quiet_us = quiet;
    required_wire_us = (uint32_t)wire + quiet;
    last_request_us = Timebase_NowUs() - period;
    return true;
}
bool MobileServoFeedback_AcceptsOutput(const actuator_mobile_supervisor_t *s,
    const actuator_mobile_output_config_t *c) {
    if (!reader.configured) return true; /* external feedback adapter path */
    if (c == NULL || reader.supervisor != s ||
        c->schedule.arm_period_us != schedule.arm_period_us ||
        c->schedule.arm_reserved_us != schedule.arm_reserved_us ||
        c->schedule.guard_us != schedule.guard_us) return false;
    for (unsigned i = 0; i < 4; ++i)
        if (c->velocity_direction[i] != reader.config.axes[i].velocity_direction) return false;
    return true;
}
static bool fits_arm_window(uint32_t now, uint32_t needed) {
    uint32_t epoch;
    if (!ControlTick_PeekEpoch(&epoch)) return false;
    uint32_t phase = now - epoch;
    return phase < schedule.arm_period_us && phase >= schedule.arm_reserved_us &&
        (uint64_t)phase + needed + schedule.guard_us <= schedule.arm_period_us;
}
const actuator_mobile_feedback_reader_t *MobileServoFeedback_State(void) { return &reader; }
void MobileServoFeedback_SetLiftEvidence(const actuator_mobile_lift_evidence_t *e) {
    if (e == NULL) { have_lift = false; return; }
    if (have_lift_stamp && (e->observed_ms - lift.observed_ms == 0u ||
        e->observed_ms - lift.observed_ms >= UINT32_C(0x80000000))) {
        have_lift = false; return;
    }
    lift = *e; have_lift = true; have_lift_stamp = true;
}
void MobileServoFeedback_OnTxComplete(UART_HandleTypeDef *uart) {
    if (!reader.configured || uart != left_uart || !gate_token || !request_sent ||
        uart->gState != HAL_UART_STATE_READY) return;
    if (tx_complete) uart_error = 1;
    completion_us = Timebase_NowUs(); tx_complete = 1;
}
void MobileServoFeedback_OnUartError(UART_HandleTypeDef *uart) {
    if (reader.configured && uart == left_uart && gate_token) uart_error = 1;
}
void MobileServoFeedback_Poll(void) {
    if (!reader.configured || launching) return;
    uint32_t now = Timebase_NowUs(), ms = HAL_GetTick();
    actuator_mobile_feedback_reader_poll(&reader, ms);
    (void)actuator_mobile_feedback_publish(&reader, have_lift ? &lift : NULL, ms);
    if (reader.fault != MOBILE_FEEDBACK_OK || uart_error) { fault(); return; }
    if (gate_token) {
        if (now - started_us >= budget_us) { fault(); return; }
        uint8_t bytes[64]; size_t length;
        if (!ServoBus_ReaderSlice(left_uart, gate_token, &cursor, bytes, sizeof(bytes), &length)) {
            fault(); return;
        }
        now = Timebase_NowUs(); ms = HAL_GetTick();
        if (now - started_us >= budget_us) { fault(); return; }
        if (length) {
            last_activity_us = now;
            if (request_sent && !actuator_mobile_feedback_feed(&reader, reader_token, bytes, length, ms)) {
                fault(); return;
            }
        }
        if (!ServoBus_ReaderQuiet(left_uart, gate_token, cursor)) { last_activity_us = now; return; }
        if (now - last_activity_us < quiet_us) return;
        if (!request_sent) {
            if (!fits_arm_window(now, required_wire_us) ||
                budget_us - (now - started_us) <= required_wire_us ||
                !actuator_mobile_feedback_begin(&reader, ms, true, request, &reader_token)) {
                fault(); return;
            }
            clear_events(); request_sent = true; launching = 1;
            HAL_StatusTypeDef result = ServoTransport_TransmitDMA(left_uart, gate_token, request, sizeof(request));
            launching = 0;
            if (result != HAL_OK) fault();
            return;
        }
        uint32_t mask = __get_PRIMASK(); __disable_irq();
        uint8_t complete = tx_complete, error = uart_error; uint32_t done = completion_us;
        if (!mask) __enable_irq();
        if (error) { fault(); return; }
        if (!complete || !reader.reply_ready || now - done < quiet_us) return;
        HAL_StatusTypeDef result = ServoTransport_End(left_uart, gate_token, false);
        if (result == HAL_BUSY) return;
        if (result != HAL_OK) { fault(); return; }
        gate_token = 0; transport_quiet = true; clear_events();
        if (!actuator_mobile_feedback_commit(&reader, reader_token, ms, true, true)) fault();
        request_sent = false;
        (void)actuator_mobile_feedback_publish(&reader, have_lift ? &lift : NULL, ms);
        return;
    }
    if (now - last_request_us < period_us) return;
    const actuator_mobile_output_t *output = MobileServoOutput_State();
    /* Pending velocity zero takes precedence. Afterwards READ remains possible
     * with motion inhibited, to obtain independent post-STOP evidence. */
    if (output->configured && output->stop_requested && !output->stop_written) return;
    if (!fits_arm_window(now, budget_us)) return;
    uint32_t token;
    if (!ServoTransport_BeginReadOnly(left_uart, &token)) return;
    gate_token = token; transport_quiet = false; started_us = now; last_request_us = now; last_activity_us = now;
    request_sent = false; clear_events();
    if (!ServoBus_ReaderCursor(left_uart, token, &cursor)) fault();
}
bool MobileServoFeedback_RecoverTransport(void) {
    if (!reader.configured || launching || !gate_token || reader.fault == MOBILE_FEEDBACK_OK) return false;
    MobileServoOutput_Stop(); ServoTransport_RequestStop(left_uart);
    if (!ServoBus_RecoverReader(left_uart, gate_token) ||
        ServoTransport_End(left_uart, gate_token, false) != HAL_OK) return false;
    gate_token = 0; transport_quiet = true; request_sent = false; clear_events(); return true;
}
bool MobileServoFeedback_Reset(void) {
    const actuator_mobile_output_t *output = MobileServoOutput_State();
    if (!reader.configured || launching || gate_token || !transport_quiet ||
        (output->configured && (!output->stop_requested || !output->stop_written)) ||
        !actuator_mobile_feedback_reset(&reader, true, true)) return false;
    have_lift = false; have_lift_stamp = false; clear_events(); last_request_us = Timebase_NowUs() - period_us;
    return true;
}
