#ifndef SERVO_JOINT_CONFIG_H
#define SERVO_JOINT_CONFIG_H

#include "single_arm_config.h"
#include <stdint.h>

typedef struct
{
    uint8_t id;
    const char *name;
    uint8_t motion_enabled;
    uint16_t home_position;
    uint16_t min_position;
    uint16_t max_position;
    uint8_t p_gain;
    uint8_t d_gain;
    int8_t test_direction;
    uint16_t test_delta;
    uint32_t duration_ms;
    uint16_t torque_limit;
} ServoJointConfig;

extern const ServoJointConfig servo_joints[SINGLE_ARM_JOINT_COUNT];
extern const uint8_t servo_joint_count;

#endif
