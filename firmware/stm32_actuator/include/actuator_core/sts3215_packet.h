#ifndef ACTUATOR_CORE_STS3215_PACKET_H
#define ACTUATOR_CORE_STS3215_PACKET_H

#include <stddef.h>
#include <stdint.h>

#define ACTUATOR_STS3215_MAX_SYNC_WRITE_SERVOS UINT8_C(6)
#define ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE UINT8_C(26)
#define ACTUATOR_STS3215_MAX_REGISTER_DATA UINT8_C(16)
#define ACTUATOR_STS3215_READ_PACKET_SIZE UINT8_C(8)
#define ACTUATOR_STS3215_WRITE_PACKET_SIZE UINT8_C(23)

typedef enum {
    ACTUATOR_STS3215_PACKET_OK = 0,
    ACTUATOR_STS3215_PACKET_NULL_ARGUMENT,
    ACTUATOR_STS3215_PACKET_INVALID_COUNT,
    ACTUATOR_STS3215_PACKET_INVALID_SERVO_ID,
    ACTUATOR_STS3215_PACKET_DUPLICATE_SERVO_ID,
    ACTUATOR_STS3215_PACKET_INVALID_REGISTER_RANGE
} actuator_sts3215_packet_result_t;

/* Inverted byte sum of ID through the final parameter (no header/checksum). */
uint8_t actuator_sts3215_checksum(const uint8_t *fields, size_t length);

/* Unicast only. Builders never transmit, change modes, or authorize motion.
 * Packet and length outputs remain unchanged on invalid input. */
actuator_sts3215_packet_result_t actuator_sts3215_build_read(
    uint8_t servo_id, uint8_t address, uint8_t data_length,
    uint8_t packet[ACTUATOR_STS3215_READ_PACKET_SIZE]);

actuator_sts3215_packet_result_t actuator_sts3215_build_write(
    uint8_t servo_id, uint8_t address, const uint8_t *data, uint8_t data_length,
    uint8_t packet[ACTUATOR_STS3215_WRITE_PACKET_SIZE], size_t *packet_length);

/* Low-level 16-bit register serialization; caller owns units and motor mode. */
actuator_sts3215_packet_result_t actuator_sts3215_build_sync_write_words(
    uint8_t address, const uint8_t *servo_ids, const uint16_t *values,
    uint8_t servo_count,
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE],
    size_t *packet_length);

/*
 * Build one protocol-0 broadcast SYNC WRITE for Goal Position (address 42).
 * This module is HAL-free so the exact bytes used by both arm buses can be
 * verified on the host before either UART is allowed to transmit them.
 */
actuator_sts3215_packet_result_t
actuator_sts3215_build_sync_write_positions(
    const uint8_t *servo_ids,
    const uint16_t *positions,
    uint8_t servo_count,
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE],
    size_t *packet_length);

#endif
