#ifndef WHEEL_HOLD_SETUP_H
#define WHEEL_HOLD_SETUP_H
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
void WheelSetup_Init(void);
bool WheelSetup_Plan(uint32_t nonce);
bool WheelSetup_Apply(uint32_t nonce);
void WheelSetup_Cancel(void);
void WheelSetup_Describe(char *out,size_t size,bool ok);
const char *WheelSetup_Reason(void);
/* One exact unicast packet, only during synchronous Apply. Closed at boot,
 * during driving, after every transaction, and on every failure. */
bool WheelSetup_ConsumeWrite(UART_HandleTypeDef *uart,uint8_t id,
    uint8_t address,const uint8_t *data,uint8_t length);
#endif
