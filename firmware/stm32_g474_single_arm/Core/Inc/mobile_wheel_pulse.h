#ifndef MOBILE_WHEEL_PULSE_H
#define MOBILE_WHEEL_PULSE_H
#include <stdbool.h>
#include <stdint.h>
/* Raised-wheel commissioning only. One attempt per MCU boot, never replayed. */
typedef struct {
    uint8_t id, failed_id;
    bool ok, motion_attempted, cleanup_attempted, stop_confirmed;
    const char *stage;
    int speed_check_raw, speed_check_hal_status;
    int speed_raw, position_before, position_after;
    int goal_after, torque_after, speed_after;
    uint32_t stop_requested_ms;
} MobileWheelPulseResult;
bool MobileWheelPulse_Consumed(void);
void MobileWheelPulse_Run(uint8_t id, int direction, MobileWheelPulseResult *out);
typedef struct {
    bool ok, stop_confirmed, cleanup_attempted;
    char motion;
    const char *stage;
    uint8_t failed_id;
    uint32_t stop_requested_ms;
    MobileWheelPulseResult wheels[3];
} MobileBasePulseResult;
/* Fixed nominal 120-degree bench combinations; same one-attempt latch as single wheel. */
void MobileWheelPulse_RunBase(char motion, MobileBasePulseResult *out);
#endif
