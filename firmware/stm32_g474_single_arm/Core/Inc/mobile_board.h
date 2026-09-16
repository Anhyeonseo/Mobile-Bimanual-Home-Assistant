#ifndef MOBILE_BOARD_H
#define MOBILE_BOARD_H
#include "mobile_servo_feedback.h"
#include "mobile_servo_output.h"
#include "actuator_core/device_startup.h"
#include "actuator_core/lift_endpoint.h"
#include "actuator_core/system_stop.h"
typedef struct {
    actuator_device_profile_t devices;
    actuator_mobile_config_t mobile;
    actuator_mobile_feedback_config_t feedback;
    actuator_mobile_output_config_t output;
    actuator_lift_config_t lift;
    uint32_t read_period_us,read_budget_us,quiet_us,current_microamps_per_raw;
    actuator_system_stop_config_t stop;
    uint16_t arm_hold_tolerance_raw;
    uint32_t arm_hold_dwell_ms,arm_read_period_ms;
    bool initialize_erased_boot_storage;
} MobileBoardProfile;
/* Runs before host binary mode, in secured maintenance only. Never homes/arms.
 * Missing compiled commissioned profile => Boot returns false, no flash/UART I/O. */
bool MobileBoard_Prepare(UART_HandleTypeDef *left,UART_HandleTypeDef *right,const MobileBoardProfile *profile,bool secured);
/* Explicit secured, torque-off provisioning before Prepare. Never called by Boot. */
bool MobileBoard_ProvisionModes(const actuator_device_profile_t *devices,bool mechanism_secured);
bool MobileBoard_Evidence(const uint8_t request[36],uint8_t response[64]);
bool MobileBoard_Boot(UART_HandleTypeDef *left,UART_HandleTypeDef *right);
void MobileBoard_Poll(void);
/* Called only after BOTH legacy arm configuration sequences succeed. */
bool MobileBoard_ArmsPrepared(void);
/* Transfer background reads to the trajectory; refresh a previously armed anchor. */
bool MobileBoard_PrepareArmMotion(int32_t anchor_urad[12]);
bool MobileBoard_IsConfigured(void);
bool MobileBoard_StopActive(void);
/* Idempotent. Preserves torque; no automatic rearm/reboot recovery. */
void MobileBoard_RequestStop(void);
const actuator_system_stop_t *MobileBoard_StopState(void);
bool MobileBoard_StopQuery(const uint8_t request[36],uint8_t response[64]);
/* Payload retention needs independent evidence, not gripper command success. */
void MobileBoard_ObserveLoad(uint32_t observed_ms,bool retained);
/* Supplied by independent robot-state/hold monitoring, never command echoes. */
void MobileBoard_ObserveInterlocks(uint32_t observed_ms,bool arms_safe,bool lift_hold_verified);
actuator_lift_endpoint_t *MobileBoard_LiftEndpoint(void);
actuator_mobile_supervisor_t *MobileBoard_Supervisor(void);
#endif
