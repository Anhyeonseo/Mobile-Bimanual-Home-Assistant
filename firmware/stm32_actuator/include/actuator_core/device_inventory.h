#ifndef ACTUATOR_DEVICE_INVENTORY_H
#define ACTUATOR_DEVICE_INVENTORY_H
#include <stdbool.h>
#include <stdint.h>
/* Protocol-v2 maintenance query; no runtime configuration or motor writes. */
#define ACTUATOR_V2_MSG_DEVICE_INVENTORY_REQUEST 66U
#define ACTUATOR_V2_MSG_DEVICE_INVENTORY_RESPONSE 67U
#define DEVICE_INVENTORY_REPLY_SIZE 56U
#define DEVICE_INVENTORY_BLOCKS 5U
static inline uint8_t device_inventory_address(unsigned block) {
    const uint8_t values[5] = {0, 9, 31, 40, 55};
    return block < 5 ? values[block] : 0;
}
static inline uint8_t device_inventory_length(unsigned block) {
    const uint8_t values[5] = {7, 4, 3, 10, 16};
    return block < 5 ? values[block] : 0;
}
static inline uint8_t device_inventory_offset(unsigned block) {
    const uint8_t values[5] = {12, 19, 23, 26, 36};
    return block < 5 ? values[block] : 0;
}
#endif
