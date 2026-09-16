#ifndef MOBILE_HOLD_OUTPUT_H
#define MOBILE_HOLD_OUTPUT_H
#include "servo_transport.h"
/* Right UART STOP-only DMA writer. Normal arm dispatch retains its own owner.
 * No torque-off command, feedback synthesis or automatic recovery is allowed. */
bool MobileHoldOutput_Configure(UART_HandleTypeDef *right,uint32_t budget_us,uint32_t quiet_us);
actuator_bus_router_t *MobileHoldOutput_Router(void);
void MobileHoldOutput_Poll(void);
void MobileHoldOutput_OnTxComplete(UART_HandleTypeDef *uart);
void MobileHoldOutput_OnUartError(UART_HandleTypeDef *uart);
bool MobileHoldOutput_Recover(bool rx_quiet_verified);
#endif
