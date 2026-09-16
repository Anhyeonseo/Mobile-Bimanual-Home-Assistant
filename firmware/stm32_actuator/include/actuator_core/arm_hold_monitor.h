#ifndef ACTUATOR_ARM_HOLD_MONITOR_H
#define ACTUATOR_ARM_HOLD_MONITOR_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint16_t target[12],seen;
    uint32_t after_ms,stamp[6],stable_since[6],dwell_ms,max_age_ms;
    uint16_t tolerance_raw;
    uint8_t stable;
    bool configured;
} actuator_arm_hold_monitor_t;
bool actuator_arm_hold_monitor_init(actuator_arm_hold_monitor_t *m,
    const uint16_t left[6],const uint16_t right[6],uint32_t after_ms,
    uint16_t tolerance_raw,uint32_t dwell_ms,uint32_t max_age_ms);
/* Paired, independent measured positions. Timestamp is the read request time. */
bool actuator_arm_hold_monitor_observe(actuator_arm_hold_monitor_t *m,
    uint8_t joint,uint16_t left,uint16_t right,uint32_t observed_ms,uint32_t now_ms);
bool actuator_arm_hold_monitor_proof(const actuator_arm_hold_monitor_t *m,
    uint32_t now_ms,uint32_t *oldest_sample_ms);
#endif
