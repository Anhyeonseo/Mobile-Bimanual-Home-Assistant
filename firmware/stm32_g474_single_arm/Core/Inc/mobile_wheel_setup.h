#ifndef MOBILE_WHEEL_SETUP_H
#define MOBILE_WHEEL_SETUP_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint8_t id; int mode, torque, lock; const char *stage; bool ok; } MobileWheelSetupResult;
/* One attempt per MCU boot. Every wheel must pass read-only preflight first.
 * No torque enable, no nonzero velocity, no arm or lift writes. */
bool MobileWheelSetup_Run(MobileWheelSetupResult result[3]);
#endif
