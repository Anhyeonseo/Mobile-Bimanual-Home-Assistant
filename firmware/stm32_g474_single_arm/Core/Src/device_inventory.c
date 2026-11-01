#include "device_inventory.h"
#include "servo_bus.h"
#include "right_servo_bus.h"
#include "actuator_core/mobile_ids.h"
#include <string.h>

void DeviceInventory_Read(const uint8_t *q, uint16_t length, bool permitted,
                          uint8_t out[DEVICE_INVENTORY_REPLY_SIZE]) {
    memset(out, 0, DEVICE_INVENTORY_REPLY_SIZE);
    out[0] = 1; out[1] = 1; out[5] = 255;
    uint32_t tick = HAL_GetTick();
    for (unsigned i=0; i<4; ++i) out[8+i] = (uint8_t)(tick >> (8*i));
    if (q == NULL || length != 4) return;
    out[2] = q[1]; out[3] = q[2];
    bool left_id = (q[2] >= 1 && q[2] <= 6) ||
                   (q[2] >= ACTUATOR_MOBILE_FIRST_ID && q[2] <= ACTUATOR_LIFT_ID);
    if (q[0] != 1 || q[3] != 0 || q[1] > 1 ||
        (q[1] == 0 ? !left_id : (q[2] < 1 || q[2] > 6))) return;
    out[1] = 2;
    if (!permitted) return; /* Gate before even a UART recovery/read. */
    out[1] = 0;
    for (unsigned block=0; block<DEVICE_INVENTORY_BLOCKS; ++block) {
        uint8_t *data = out + device_inventory_offset(block);
        uint8_t status = q[1] == 0
            ? (uint8_t)Servo_ReadData(q[2], device_inventory_address(block),
                                     device_inventory_length(block), data)
            : (uint8_t)RightServoBus_ReadInventoryBlock(q[2], block, data);
        if (status == 0) out[4] |= (uint8_t)(1U << block);
        else {
            memset(data, 0, device_inventory_length(block));
            if (out[1] == 0) { out[5] = (uint8_t)block; out[6] = status; out[7] = q[1]+1; }
            out[1] = 3;
            /* A missing identity needs one bounded read, not four more timeouts. */
            if (block == 0) break;
        }
    }
}
