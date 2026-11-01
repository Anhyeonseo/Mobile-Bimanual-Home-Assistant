#ifndef HOME_DEVICE_INVENTORY_H
#define HOME_DEVICE_INVENTORY_H
#include "actuator_core/device_inventory.h"
/* status: 0 complete, 1 invalid request, 2 maintenance unavailable, 3 partial. */
void DeviceInventory_Read(const uint8_t *request, uint16_t length, bool permitted,
                          uint8_t response[DEVICE_INVENTORY_REPLY_SIZE]);
#endif
