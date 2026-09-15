#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "actuator_core/sts3215_packet.h"
#include "actuator_core/motor_groups.h"

static int failures = 0;

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    ++failures; return; } } while (0)

static void test_six_servo_position_packet(void) {
    const uint8_t ids[6] = {1u, 2u, 3u, 4u, 5u, 6u};
    const uint16_t positions[6] = {
        0x0800u, 0x0801u, 0x0000u, 0x0fffu, 0x1234u, 0x00ffu};
    const uint8_t expected[26] = {
        0xffu, 0xffu, 0xfeu, 0x16u, 0x83u, 0x2au, 0x02u,
        0x01u, 0x00u, 0x08u,
        0x02u, 0x01u, 0x08u,
        0x03u, 0x00u, 0x00u,
        0x04u, 0xffu, 0x0fu,
        0x05u, 0x34u, 0x12u,
        0x06u, 0xffu, 0x00u,
        0xc3u};
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
    size_t length = 0u;

    CHECK(actuator_sts3215_build_sync_write_positions(
              ids, positions, 6u, packet, &length) ==
          ACTUATOR_STS3215_PACKET_OK);
    CHECK(length == sizeof(expected));
    CHECK(memcmp(packet, expected, sizeof(expected)) == 0);
}

static void test_invalid_inputs_are_rejected(void) {
    const uint8_t duplicate_ids[2] = {1u, 1u};
    const uint8_t broadcast_id[1] = {0xfeu};
    const uint16_t positions[2] = {2048u, 2048u};
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
    size_t length = 99u;

    CHECK(actuator_sts3215_build_sync_write_positions(
              duplicate_ids, positions, 2u, packet, &length) ==
          ACTUATOR_STS3215_PACKET_DUPLICATE_SERVO_ID);
    CHECK(actuator_sts3215_build_sync_write_positions(
              broadcast_id, positions, 1u, packet, &length) ==
          ACTUATOR_STS3215_PACKET_INVALID_SERVO_ID);
    CHECK(actuator_sts3215_build_sync_write_positions(
              duplicate_ids, positions, 0u, packet, &length) ==
          ACTUATOR_STS3215_PACKET_INVALID_COUNT);
    CHECK(actuator_sts3215_build_sync_write_positions(
              NULL, positions, 1u, packet, &length) ==
          ACTUATOR_STS3215_PACKET_NULL_ARGUMENT);
}

static void test_unicast_register_packets(void) {
    uint8_t packet[ACTUATOR_STS3215_WRITE_PACKET_SIZE];
    const uint8_t read_expected[8] = {0xff, 0xff, 11, 4, 2, 56, 2, 0xb4};
    const uint8_t write_expected[9] = {0xff, 0xff, 8, 5, 3, 46, 100, 0, 0x5d};
    const uint8_t data[2] = {100, 0};
    size_t length = 123;
    CHECK(actuator_sts3215_build_read(11, 56, 2, packet) == ACTUATOR_STS3215_PACKET_OK);
    CHECK(memcmp(packet, read_expected, sizeof(read_expected)) == 0);
    CHECK(actuator_sts3215_build_write(8, 46, data, 2, packet, &length) == ACTUATOR_STS3215_PACKET_OK);
    CHECK(length == sizeof(write_expected));
    CHECK(memcmp(packet, write_expected, sizeof(write_expected)) == 0);
}

static void test_register_rejections_preserve_outputs(void) {
    uint8_t packet[ACTUATOR_STS3215_WRITE_PACKET_SIZE];
    uint8_t before[sizeof(packet)];
    uint8_t data[16] = {0};
    size_t length = 123;
    memset(packet, 0xa5, sizeof(packet));
    memcpy(before, packet, sizeof(packet));
    CHECK(actuator_sts3215_build_write(0xfe, 40, data, 1, packet, &length) ==
          ACTUATOR_STS3215_PACKET_INVALID_SERVO_ID);
    CHECK(actuator_sts3215_build_read(0, 56, 2, packet) ==
          ACTUATOR_STS3215_PACKET_INVALID_SERVO_ID);
    CHECK(actuator_sts3215_build_read(1, 255, 2, packet) ==
          ACTUATOR_STS3215_PACKET_INVALID_REGISTER_RANGE);
    CHECK(actuator_sts3215_build_write(1, 56, data, 17, packet, &length) ==
          ACTUATOR_STS3215_PACKET_INVALID_COUNT);
    CHECK(actuator_sts3215_build_read(1, 56, 0, packet) ==
          ACTUATOR_STS3215_PACKET_INVALID_COUNT);
    CHECK(actuator_sts3215_build_write(1, 56, NULL, 1, packet, &length) ==
          ACTUATOR_STS3215_PACKET_NULL_ARGUMENT);
    CHECK(length == 123 && memcmp(packet, before, sizeof(packet)) == 0);
    CHECK(actuator_sts3215_build_read(253, 255, 1, packet) == ACTUATOR_STS3215_PACKET_OK);
    CHECK(actuator_sts3215_build_write(253, 240, data, 16, packet, &length) ==
          ACTUATOR_STS3215_PACKET_OK);
    CHECK(length == 23);
}

static void test_shared_bus_topology_has_unique_addresses(void) {
    uint8_t occupied[2][254] = {{0}};
    unsigned int bus_count[2] = {0, 0};
    unsigned int group;
    CHECK(actuator_motor_group_get((actuator_motor_group_id_t)-1) == NULL);
    CHECK(actuator_motor_group_get(ACTUATOR_MOTOR_GROUP_COUNT) == NULL);
    for (group = 0; group < ACTUATOR_MOTOR_GROUP_COUNT; ++group) {
        const actuator_motor_group_t *spec = actuator_motor_group_get((actuator_motor_group_id_t)group);
        uint8_t i;
        CHECK(spec != NULL && spec->count > 0 && spec->count <= 6);
        CHECK((unsigned int)spec->bus < 2);
        for (i = 0; i < spec->count; ++i) {
            CHECK(spec->ids[i] > 0 && spec->ids[i] < 254);
            CHECK(occupied[spec->bus][spec->ids[i]] == 0);
            occupied[spec->bus][spec->ids[i]] = 1;
            ++bus_count[spec->bus];
        }
    }
    CHECK(bus_count[ACTUATOR_BUS_LEFT_SHARED] == 10);
    CHECK(bus_count[ACTUATOR_BUS_RIGHT_ARM] == 6);
    CHECK(occupied[ACTUATOR_BUS_LEFT_SHARED][7] == 0);
}

static void test_velocity_packets_are_scoped_and_signed(void) {
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
    const int32_t wheels[3] = {100, -100, 0};
    const int32_t lift[1] = {-1};
    const uint8_t expected[17] = {
        0xff, 0xff, 0xfe, 13, 0x83, 46, 2,
        8, 100, 0, 9, 100, 0x80, 10, 0, 0, 0xde};
    size_t length = 0;
    CHECK(actuator_motor_group_velocities(ACTUATOR_GROUP_BASE_WHEELS,
          wheels, 3, packet, &length) == ACTUATOR_GROUP_OK);
    CHECK(length == sizeof(expected));
    CHECK(memcmp(packet, expected, sizeof(expected)) == 0);
    CHECK(actuator_motor_group_velocities(ACTUATOR_GROUP_LIFT,
          lift, 1, packet, &length) == ACTUATOR_GROUP_OK);
    CHECK(length == 11 && packet[7] == 11 && packet[8] == 1 && packet[9] == 0x80);
}

static void test_modes_counts_and_velocity_overflow_are_rejected(void) {
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
    uint8_t before[sizeof(packet)];
    const uint16_t positions[6] = {0};
    const int32_t invalid[3] = {1, -32768, INT32_MAX};
    size_t length = 123;
    memset(packet, 0xa5, sizeof(packet));
    memcpy(before, packet, sizeof(packet));
    CHECK(actuator_motor_group_positions(ACTUATOR_GROUP_BASE_WHEELS,
          positions, 3, packet, &length) == ACTUATOR_GROUP_WRONG_MODE);
    CHECK(actuator_motor_group_positions(ACTUATOR_GROUP_LIFT,
          positions, 1, packet, &length) == ACTUATOR_GROUP_WRONG_MODE);
    CHECK(actuator_motor_group_velocities(ACTUATOR_GROUP_LEFT_ARM,
          invalid, 3, packet, &length) == ACTUATOR_GROUP_WRONG_MODE);
    CHECK(actuator_motor_group_positions(ACTUATOR_GROUP_RIGHT_ARM,
          positions, 5, packet, &length) == ACTUATOR_GROUP_INVALID_COUNT);
    CHECK(actuator_motor_group_velocities(ACTUATOR_GROUP_BASE_WHEELS,
          invalid, 2, packet, &length) == ACTUATOR_GROUP_INVALID_COUNT);
    CHECK(actuator_motor_group_velocities(ACTUATOR_GROUP_BASE_WHEELS,
          invalid, 3, packet, &length) == ACTUATOR_GROUP_VELOCITY_OUT_OF_RANGE);
    CHECK(actuator_motor_group_velocities(ACTUATOR_GROUP_LIFT,
          &invalid[2], 1, packet, &length) == ACTUATOR_GROUP_VELOCITY_OUT_OF_RANGE);
    CHECK(length == 123 && memcmp(packet, before, sizeof(packet)) == 0);
    CHECK(actuator_motor_group_positions(ACTUATOR_GROUP_LEFT_ARM,
          positions, 6, packet, &length) == ACTUATOR_GROUP_OK);
    CHECK(length == 26 && packet[5] == 42);
    for (uint8_t i = 0; i < 6; ++i) {
        CHECK(packet[7 + 3 * i] == i + 1);
    }
}

int main(void) {
    test_six_servo_position_packet();
    test_invalid_inputs_are_rejected();
    test_unicast_register_packets();
    test_register_rejections_preserve_outputs();
    test_shared_bus_topology_has_unique_addresses();
    test_velocity_packets_are_scoped_and_signed();
    test_modes_counts_and_velocity_overflow_are_rejected();
    if (failures != 0) {
        fprintf(stderr, "%d test(s) failed\n", failures);
        return 1;
    }
    puts("sts3215 packet tests passed");
    return 0;
}
