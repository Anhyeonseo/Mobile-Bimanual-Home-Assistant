#ifndef MOBILE_SERVO_OUTPUT_H
#define MOBILE_SERVO_OUTPUT_H
#include "servo_transport.h"
#include "actuator_core/mobile_output.h"
/* Unconfigured on boot. A commissioning caller must bind measured supervisor
 * feedback, independent whole-system stop proof and measured TX/quiet budgets.
 * This writer never sets modes, homes, invents feedback or attaches an endpoint. */
bool MobileServoOutput_Configure(UART_HandleTypeDef *left_uart,
    actuator_mobile_supervisor_t *supervisor,
    const actuator_mobile_output_config_t *config,
    uint32_t quiet_us, bool whole_stop_confirmed);
bool MobileServoOutput_BindLift(actuator_lift_endpoint_t *endpoint);
void MobileServoOutput_Poll(void);
void MobileServoOutput_OnTxComplete(UART_HandleTypeDef *uart);
void MobileServoOutput_OnUartError(UART_HandleTypeDef *uart);
void MobileServoOutput_Stop(void);
/* Host ARM/VELOCITY admission also requires this exact bound output. */
bool MobileServoOutput_CommandAllowed(const actuator_mobile_supervisor_t *supervisor);
/* Explicit recovery drains RX outside this writer. A late callback cannot
 * release ownership; faulted transactions remain owned until abort succeeds. */
bool MobileServoOutput_Recover(bool rx_quiet_verified);
bool MobileServoOutput_Rearm(uint32_t session,bool whole_stop_confirmed);
const actuator_mobile_output_t *MobileServoOutput_State(void);
/* Only the whole-stop coordinator may enqueue a left-arm hold after zero. */
actuator_bus_router_t *MobileServoOutput_StopRouter(void);
#endif
