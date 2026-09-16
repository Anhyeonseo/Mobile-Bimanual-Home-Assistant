#ifndef MOBILE_ARM_OBSERVER_H
#define MOBILE_ARM_OBSERVER_H
#include "stm32g4xx_hal.h"
#include <stdbool.h>
/* Background READ-only observation between finite trajectories. One physical
 * feedback owner; transfer to motion only after both read leases are released. */
bool MobileArmObserver_Configure(UART_HandleTypeDef *left,UART_HandleTypeDef *right,
    uint32_t read_period_ms,uint32_t maximum_age_ms);
bool MobileArmObserver_Poll(void);
bool MobileArmObserver_OwnsFeedback(void);
bool MobileArmObserver_HasStarted(void);
bool MobileArmObserver_Suspend(void);
void MobileArmObserver_Resume(void);
#endif
