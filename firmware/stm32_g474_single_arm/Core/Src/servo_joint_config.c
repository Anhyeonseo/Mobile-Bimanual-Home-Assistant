/* Arm calibration/service configuration only; never append mobile motors. */
#include "servo_joint_config.h"
#include "single_arm_config.h"

const ServoJointConfig servo_joints[SINGLE_ARM_JOINT_COUNT] = {
    {1U, "BASE",        1U, 2048U, 1988U, 2610U, 16U, 32U,  1,  34U,  600U, 400U},
    {2U, "SHOULDER",    1U, 2048U, 1988U, 3766U, 64U, 64U,  1,  34U, 1200U,
        SERVO_SHOULDER_TORQUE_LIMIT_RAW},
    {3U, "ELBOW",       1U, 2048U, 627U, 2258U, 56U, 64U, -1,  34U, 1000U,
        SERVO_ELBOW_TORQUE_LIMIT_RAW},
    {4U, "WRIST_FLEX",  1U, 2048U, 1194U, 2108U, 16U, 32U, -1,  34U,  800U, 400U},
    {5U, "WRIST_ROLL",  1U, 2048U, 1874U, 2219U, 16U, 32U,  1,  34U,  500U, 250U},
    {6U, "GRIPPER",     1U, 2048U, 1866U, 2048U, 16U, 32U, -1,  34U,  800U, 150U}
};

const uint8_t servo_joint_count = SINGLE_ARM_JOINT_COUNT;
