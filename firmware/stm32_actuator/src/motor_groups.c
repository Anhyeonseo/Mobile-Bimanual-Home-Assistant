#include "actuator_core/motor_groups.h"

#include <stddef.h>

static const actuator_motor_group_t groups[ACTUATOR_MOTOR_GROUP_COUNT] = {
    {ACTUATOR_BUS_LEFT_SHARED, 0u, 6u, {1u, 2u, 3u, 4u, 5u, 6u}},
    {ACTUATOR_BUS_RIGHT_ARM,   0u, 6u, {1u, 2u, 3u, 4u, 5u, 6u}},
    {ACTUATOR_BUS_LEFT_SHARED, 1u, 3u, {8u, 9u, 10u, 0u, 0u, 0u}},
    {ACTUATOR_BUS_LEFT_SHARED, 1u, 1u, {11u, 0u, 0u, 0u, 0u, 0u}}
};

const actuator_motor_group_t *actuator_motor_group_get(actuator_motor_group_id_t group) {
    if ((unsigned int)group >= ACTUATOR_MOTOR_GROUP_COUNT) {
        return NULL;
    }
    return &groups[group];
}

actuator_motor_group_result_t actuator_motor_group_positions(
    actuator_motor_group_id_t group, const uint16_t *positions, uint8_t count,
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE], size_t *length) {
    const actuator_motor_group_t *spec = actuator_motor_group_get(group);
    if (spec == NULL || positions == NULL || packet == NULL || length == NULL) {
        return ACTUATOR_GROUP_INVALID_ARGUMENT;
    }
    if (spec->expected_operating_mode != 0u) {
        return ACTUATOR_GROUP_WRONG_MODE;
    }
    if (count != spec->count) {
        return ACTUATOR_GROUP_INVALID_COUNT;
    }
    return actuator_sts3215_build_sync_write_positions(spec->ids, positions,
        count, packet, length) == ACTUATOR_STS3215_PACKET_OK ?
        ACTUATOR_GROUP_OK : ACTUATOR_GROUP_PACKET_ERROR;
}

actuator_motor_group_result_t actuator_motor_group_velocities(
    actuator_motor_group_id_t group, const int32_t *velocities_raw, uint8_t count,
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE], size_t *length) {
    const actuator_motor_group_t *spec = actuator_motor_group_get(group);
    uint16_t words[6] = {0u};
    uint8_t i;
    if (spec == NULL || velocities_raw == NULL || packet == NULL || length == NULL) {
        return ACTUATOR_GROUP_INVALID_ARGUMENT;
    }
    if (spec->expected_operating_mode != 1u) {
        return ACTUATOR_GROUP_WRONG_MODE;
    }
    if (count != spec->count) {
        return ACTUATOR_GROUP_INVALID_COUNT;
    }
    for (i = 0u; i < count; ++i) {
        const int32_t value = velocities_raw[i];
        if (value < -32767 || value > 32767) {
            return ACTUATOR_GROUP_VELOCITY_OUT_OF_RANGE;
        }
        words[i] = value < 0 ? (uint16_t)((uint32_t)(-value) | UINT32_C(0x8000)) :
                              (uint16_t)value;
    }
    return actuator_sts3215_build_sync_write_words(46u, spec->ids, words,
        count, packet, length) == ACTUATOR_STS3215_PACKET_OK ?
        ACTUATOR_GROUP_OK : ACTUATOR_GROUP_PACKET_ERROR;
}
