#ifndef ACTUATOR_CORE_MOBILE_IDS_H
#define ACTUATOR_CORE_MOBILE_IDS_H
#include <stdint.h>
/* Three wheel channels followed by the lift; independent of wheel geometry.
 * Existing reference builds retain 8..11. Assembled robot selects 7..10.
 * One compile definition applies to startup, RX, normal output and STOP. */
#ifndef ACTUATOR_MOBILE_FIRST_ID
#define ACTUATOR_MOBILE_FIRST_ID 8
#endif
#if ACTUATOR_MOBILE_FIRST_ID < 7 || ACTUATOR_MOBILE_FIRST_ID > 250
#error "Mobile IDs must exclude arm IDs 1..6 and reserved IDs 254..255"
#endif
#define ACTUATOR_WHEEL_0_ID (ACTUATOR_MOBILE_FIRST_ID)
#define ACTUATOR_WHEEL_1_ID (ACTUATOR_MOBILE_FIRST_ID + 1)
#define ACTUATOR_WHEEL_2_ID (ACTUATOR_MOBILE_FIRST_ID + 2)
#define ACTUATOR_LIFT_ID (ACTUATOR_MOBILE_FIRST_ID + 3)
#define ACTUATOR_MOBILE_IDS {ACTUATOR_WHEEL_0_ID, ACTUATOR_WHEEL_1_ID, ACTUATOR_WHEEL_2_ID, ACTUATOR_LIFT_ID}
static inline uint8_t actuator_mobile_axis_id(unsigned axis) {
    return axis < 4u ? (uint8_t)(ACTUATOR_MOBILE_FIRST_ID + axis) : 0u;
}
#endif
