#ifndef WHEEL_HOLD_MONITOR_H
#define WHEEL_HOLD_MONITOR_H
#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"
/* Retains existing actuator-side goals. This module has NO motor-write API. */
void WheelHold_Init(UART_HandleTypeDef *right);
bool WheelHold_Capture(void);
bool WheelHold_Poll(void);
bool WheelHold_VerifyAll(void);
bool WheelHold_Confirmed(void);
bool WheelHold_HasBaseline(void);
unsigned WheelHold_FailedSlot(void);
const char *WheelHold_Reason(void);
uint32_t WheelHold_Samples(void);
int WheelHold_LiftPosition(void);
int WheelHold_LiftTarget(void);
#endif
