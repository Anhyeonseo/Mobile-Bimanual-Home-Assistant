#ifndef MOBILE_SERVO_FEEDBACK_H
#define MOBILE_SERVO_FEEDBACK_H
#include "servo_transport.h"
#include "actuator_core/mobile_feedback.h"
#include "actuator_core/bus_schedule.h"
#include "actuator_core/mobile_output.h"
/* Configure in maintenance, before MobileServoOutput. No automatic boot call.
 * Whole-stop proof and all limits, directions and time budgets are required. */
bool MobileServoFeedback_Configure(UART_HandleTypeDef *left_uart,
    actuator_mobile_supervisor_t *supervisor,
    const actuator_mobile_feedback_config_t *feedback_config,
    const actuator_bus_schedule_config_t *schedule,
    uint32_t period_us, uint32_t budget_us, uint32_t quiet_us,
    bool whole_stop_confirmed);
bool MobileServoFeedback_AcceptsOutput(const actuator_mobile_supervisor_t *s,
    const actuator_mobile_output_config_t *config);
void MobileServoFeedback_Poll(void);
void MobileServoFeedback_OnTxComplete(UART_HandleTypeDef *uart);
void MobileServoFeedback_OnUartError(UART_HandleTypeDef *uart);
void MobileServoFeedback_SetLiftEvidence(const actuator_mobile_lift_evidence_t *evidence);
/* Explicit fault service: abort/drain/re-arm RX, retain ownership on failure.
 * Can wait in HAL recovery; never called in the periodic main-loop poll. It
 * releases only transport so the queued STOP can transmit; no motion resume. */
bool MobileServoFeedback_RecoverTransport(void);
/* Restart READs after cleanup and STOP transmission, keeping motion inhibited.
 * Physical stop proof is required later by output Rearm, not to observe it. */
bool MobileServoFeedback_Reset(void);
const actuator_mobile_feedback_reader_t *MobileServoFeedback_State(void);
#endif
