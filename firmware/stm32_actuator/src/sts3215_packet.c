#include "actuator_core/sts3215_packet.h"

#include <stdbool.h>
#include <string.h>

uint8_t actuator_sts3215_checksum(const uint8_t *fields, size_t length) {
    uint8_t sum = 0u;
    size_t index;

    for (index = 0u; index < length; ++index) {
        sum = (uint8_t)(sum + fields[index]);
    }
    return (uint8_t)(~sum);
}

static actuator_sts3215_packet_result_t validate_register_request(
    uint8_t servo_id, uint8_t address, uint8_t length) {
    if (servo_id == 0u || servo_id >= 0xfeu) {
        return ACTUATOR_STS3215_PACKET_INVALID_SERVO_ID;
    }
    if (length == 0u || length > ACTUATOR_STS3215_MAX_REGISTER_DATA) {
        return ACTUATOR_STS3215_PACKET_INVALID_COUNT;
    }
    if ((uint16_t)address + length > 256u) {
        return ACTUATOR_STS3215_PACKET_INVALID_REGISTER_RANGE;
    }
    return ACTUATOR_STS3215_PACKET_OK;
}

actuator_sts3215_packet_result_t actuator_sts3215_build_read(
    uint8_t servo_id, uint8_t address, uint8_t data_length,
    uint8_t packet[ACTUATOR_STS3215_READ_PACKET_SIZE]) {
    actuator_sts3215_packet_result_t result;
    if (packet == NULL) {
        return ACTUATOR_STS3215_PACKET_NULL_ARGUMENT;
    }
    result = validate_register_request(servo_id, address, data_length);
    if (result != ACTUATOR_STS3215_PACKET_OK) {
        return result;
    }
    packet[0] = 0xffu;
    packet[1] = 0xffu;
    packet[2] = servo_id;
    packet[3] = 4u;
    packet[4] = 2u;
    packet[5] = address;
    packet[6] = data_length;
    packet[7] = actuator_sts3215_checksum(packet + 2u, 5u);
    return ACTUATOR_STS3215_PACKET_OK;
}

actuator_sts3215_packet_result_t actuator_sts3215_build_write(
    uint8_t servo_id, uint8_t address, const uint8_t *data, uint8_t data_length,
    uint8_t packet[ACTUATOR_STS3215_WRITE_PACKET_SIZE], size_t *packet_length) {
    actuator_sts3215_packet_result_t result;
    if (packet == NULL || data == NULL || packet_length == NULL) {
        return ACTUATOR_STS3215_PACKET_NULL_ARGUMENT;
    }
    result = validate_register_request(servo_id, address, data_length);
    if (result != ACTUATOR_STS3215_PACKET_OK) {
        return result;
    }
    packet[0] = 0xffu;
    packet[1] = 0xffu;
    packet[2] = servo_id;
    packet[3] = (uint8_t)(data_length + 3u);
    packet[4] = 3u;
    packet[5] = address;
    memcpy(packet + 6u, data, data_length);
    packet[6u + data_length] = actuator_sts3215_checksum(
        packet + 2u, (size_t)data_length + 4u);
    *packet_length = (size_t)data_length + 7u;
    return ACTUATOR_STS3215_PACKET_OK;
}

actuator_sts3215_packet_result_t
actuator_sts3215_build_sync_write_words(
    uint8_t address,
    const uint8_t *servo_ids,
    const uint16_t *positions,
    uint8_t servo_count,
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE],
    size_t *packet_length) {
    bool seen[254] = {false};
    size_t packet_index;
    uint8_t servo;

    if (servo_ids == NULL || positions == NULL || packet == NULL ||
        packet_length == NULL) {
        return ACTUATOR_STS3215_PACKET_NULL_ARGUMENT;
    }
    if (servo_count == 0u ||
        servo_count > ACTUATOR_STS3215_MAX_SYNC_WRITE_SERVOS) {
        return ACTUATOR_STS3215_PACKET_INVALID_COUNT;
    }
    if (address == UINT8_MAX) {
        return ACTUATOR_STS3215_PACKET_INVALID_REGISTER_RANGE;
    }
    for (servo = 0u; servo < servo_count; ++servo) {
        const uint8_t servo_id = servo_ids[servo];
        if (servo_id == 0u || servo_id >= 0xfeu) {
            return ACTUATOR_STS3215_PACKET_INVALID_SERVO_ID;
        }
        if (seen[servo_id]) {
            return ACTUATOR_STS3215_PACKET_DUPLICATE_SERVO_ID;
        }
        seen[servo_id] = true;
    }

    memset(packet, 0, ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE);
    packet[0] = 0xffu;
    packet[1] = 0xffu;
    packet[2] = 0xfeu;
    packet[3] = (uint8_t)(4u + (uint8_t)(servo_count * 3u));
    packet[4] = 0x83u;
    packet[5] = address;
    packet[6] = 2u;
    packet_index = 7u;

    for (servo = 0u; servo < servo_count; ++servo) {
        packet[packet_index++] = servo_ids[servo];
        packet[packet_index++] = (uint8_t)(positions[servo] & 0xffu);
        packet[packet_index++] = (uint8_t)(positions[servo] >> 8u);
    }
    packet[packet_index] = actuator_sts3215_checksum(packet + 2u, packet_index - 2u);
    ++packet_index;
    *packet_length = packet_index;
    return ACTUATOR_STS3215_PACKET_OK;
}

actuator_sts3215_packet_result_t actuator_sts3215_build_sync_write_positions(
    const uint8_t *servo_ids, const uint16_t *positions, uint8_t servo_count,
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE],
    size_t *packet_length) {
    return actuator_sts3215_build_sync_write_words(
        42u, servo_ids, positions, servo_count, packet, packet_length);
}
