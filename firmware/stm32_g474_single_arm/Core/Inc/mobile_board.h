#ifndef MOBILE_BOARD_H
#define MOBILE_BOARD_H
#include "mobile_servo_feedback.h"
#include "mobile_servo_output.h"
#include "actuator_core/device_startup.h"
#include "actuator_core/lift_endpoint.h"
typedef struct {
    actuator_device_profile_t devices;
    actuator_mobile_config_t mobile;
    actuator_mobile_feedback_config_t feedback;
    actuator_mobile_output_config_t output;
    actuator_lift_config_t lift;
    uint32_t read_period_us,read_budget_us,quiet_us,current_microamps_per_raw;
    bool initialize_erased_boot_storage;
} MobileBoardProfile;
/* Runs before host binary mode, in secured maintenance only. Never homes/arms.
 * Missing compiled commissioned profile => Boot returns false, no flash/UART I/O. */
bool MobileBoard_Prepare(UART_HandleTypeDef *left,const MobileBoardProfile *profile,bool secured);
bool MobileBoard_Boot(UART_HandleTypeDef *left);
void MobileBoard_Poll(void);
/* Supplied by independent robot-state/hold monitoring, never command echoes. */
void MobileBoard_ObserveInterlocks(uint32_t observed_ms,bool arms_safe,bool lift_hold_verified);
actuator_lift_endpoint_t *MobileBoard_LiftEndpoint(void);
actuator_mobile_supervisor_t *MobileBoard_Supervisor(void);
#endif
