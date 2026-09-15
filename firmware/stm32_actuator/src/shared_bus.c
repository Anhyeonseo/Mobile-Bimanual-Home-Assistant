#include "actuator_core/shared_bus.h"
#include <stddef.h>
#include <string.h>

void actuator_shared_bus_init(actuator_shared_bus_t *bus) {
    if (bus != NULL) memset(bus, 0, sizeof(*bus));
}
void actuator_shared_bus_request_stop(actuator_shared_bus_t *bus) {
    if (bus != NULL) bus->stop_pending = true;
}
void actuator_shared_bus_poll(actuator_shared_bus_t *bus, uint32_t now_us) {
    if (bus != NULL && bus->active &&
        (uint32_t)(now_us - bus->started_us) >= bus->budget_us) {
        bus->faulted = true;
        bus->stop_pending = true;
    }
}
bool actuator_shared_bus_acquire(actuator_shared_bus_t *bus,
    actuator_bus_work_t work, uint32_t now_us, uint32_t budget_us,
    uint32_t available_us, uint32_t *token) {
    if (bus == NULL || token == NULL) return false;
    actuator_shared_bus_poll(bus, now_us);
    if ((unsigned)work >= ACTUATOR_BUS_WORK_COUNT || budget_us == 0u ||
        budget_us >= UINT32_C(0x80000000) ||
        available_us >= UINT32_C(0x80000000) || bus->faulted || bus->active ||
        (bus->stop_pending && work != ACTUATOR_BUS_WORK_STOP) ||
        (work != ACTUATOR_BUS_WORK_STOP && budget_us > available_us)) return false;
    /* Never reuse a transaction identity within a boot. */
    if (bus->token == UINT32_MAX) {
        bus->faulted = true;
        bus->stop_pending = true;
        return false;
    }
    bus->token++;
    bus->active = true;
    bus->owner = work;
    bus->started_us = now_us;
    bus->budget_us = budget_us;
    *token = bus->token;
    return true;
}
bool actuator_shared_bus_complete(actuator_shared_bus_t *bus,
    uint32_t token, uint32_t now_us, bool transport_ok) {
    if (bus == NULL) return false;
    actuator_shared_bus_poll(bus, now_us);
    if (!bus->active || token != bus->token || bus->faulted) return false;
    if (!transport_ok) {
        bus->faulted = true;
        bus->stop_pending = true;
        return false;
    }
    bus->active = false;
    if (bus->owner == ACTUATOR_BUS_WORK_STOP) bus->stop_pending = false;
    return true;
}
bool actuator_shared_bus_recover(actuator_shared_bus_t *bus, bool uart_idle) {
    if (bus == NULL || !uart_idle || !bus->faulted || bus->token == UINT32_MAX)
        return false;
    bus->active = false;
    bus->faulted = false;
    bus->stop_pending = true;
    return true;
}
