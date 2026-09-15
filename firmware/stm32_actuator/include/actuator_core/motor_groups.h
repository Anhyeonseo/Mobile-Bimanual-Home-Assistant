#ifndef ACTUATOR_CORE_MOTOR_GROUPS_H
#define ACTUATOR_CORE_MOTOR_GROUPS_H

#include "actuator_core/sts3215_packet.h"

/* Both sockets on the left Waveshare adapter are ONE physical UART bus. */
typedef enum {
    ACTUATOR_BUS_LEFT_SHARED = 0,
    ACTUATOR_BUS_RIGHT_ARM = 1
} actuator_motor_bus_t;

typedef enum {
    ACTUATOR_GROUP_LEFT_ARM = 0,
    ACTUATOR_GROUP_RIGHT_ARM,
    ACTUATOR_GROUP_BASE_WHEELS,
    ACTUATOR_GROUP_LIFT,
    ACTUATOR_MOTOR_GROUP_COUNT
} actuator_motor_group_id_t;

typedef struct {
    actuator_motor_bus_t bus;
    uint8_t expected_operating_mode; /* STS3215: position=0, velocity=1 */
    uint8_t count;
    uint8_t ids[6];
} actuator_motor_group_t;

typedef enum {
    ACTUATOR_GROUP_OK = 0,
    ACTUATOR_GROUP_WRONG_MODE,
    ACTUATOR_GROUP_INVALID_ARGUMENT,
    ACTUATOR_GROUP_INVALID_COUNT,
    ACTUATOR_GROUP_VELOCITY_OUT_OF_RANGE,
    ACTUATOR_GROUP_PACKET_ERROR
} actuator_motor_group_result_t;

/* Fixed AlohaMini 1 allocation; this is configuration, not device discovery. */
const actuator_motor_group_t *actuator_motor_group_get(actuator_motor_group_id_t group);

/* Exact whole-group frames: an arm position frame never includes IDs 8..11.
 * These functions only build bytes. They do not set/verify operating mode,
 * transmit, arbitrate the shared bus, or bypass existing safety limits. */
actuator_motor_group_result_t actuator_motor_group_positions(
    actuator_motor_group_id_t group, const uint16_t *positions, uint8_t count,
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE], size_t *length);

/* Signed servo-register units, NOT m/s, rad/s or lift height. STS3215 uses
 * sign-magnitude bit 15, not an int16_t two's-complement wire representation.
 * A zero-speed lift packet does not prove load retention or mechanical braking. */
actuator_motor_group_result_t actuator_motor_group_velocities(
    actuator_motor_group_id_t group, const int32_t *velocities_raw, uint8_t count,
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE], size_t *length);

#endif
