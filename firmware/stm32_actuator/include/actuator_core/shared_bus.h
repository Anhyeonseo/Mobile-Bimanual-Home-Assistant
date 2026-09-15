#ifndef ACTUATOR_CORE_SHARED_BUS_H
#define ACTUATOR_CORE_SHARED_BUS_H
#include <stdbool.h>
#include <stdint.h>

/* One instance per physical UART, including BOTH left adapter sockets.
 * Called by one main-loop owner; ISR completions must be queued to that owner.
 * Ownership covers TX, response RX and recovery/quiet time, not just DMA TX.
 * This arbiter does not drive HAL or schedule periodic arm/telemetry work. */
typedef enum {
    ACTUATOR_BUS_WORK_STOP = 0, ACTUATOR_BUS_WORK_ARM,
    ACTUATOR_BUS_WORK_WHEELS, ACTUATOR_BUS_WORK_LIFT,
    ACTUATOR_BUS_WORK_FEEDBACK, ACTUATOR_BUS_WORK_COUNT
} actuator_bus_work_t;
typedef struct {
    bool active, stop_pending, faulted;
    uint32_t token, started_us, budget_us;
    actuator_bus_work_t owner;
} actuator_shared_bus_t;

void actuator_shared_bus_init(actuator_shared_bus_t *bus);
void actuator_shared_bus_request_stop(actuator_shared_bus_t *bus);
/* Poll every loop even without new requests. An expired transaction keeps
 * ownership until HAL abort/RX cleanup is confirmed by recover(). */
void actuator_shared_bus_poll(actuator_shared_bus_t *bus, uint32_t now_us);
/* budget and available window must be < 2^31 us. Caller provides time until
 * the next reserved arm slot; complete TX/RX/recovery must fit in it.
 * STOP bypasses the arm window, never an in-flight transaction. */
bool actuator_shared_bus_acquire(actuator_shared_bus_t *bus,
    actuator_bus_work_t work, uint32_t now_us, uint32_t budget_us,
    uint32_t available_us, uint32_t *token);
bool actuator_shared_bus_complete(actuator_shared_bus_t *bus,
    uint32_t token, uint32_t now_us, bool transport_ok);
/* Explicit recovery after the physical UART is idle. Keeps stop pending;
 * delayed completions from the old token cannot release new ownership. */
bool actuator_shared_bus_recover(actuator_shared_bus_t *bus, bool uart_idle);
#endif
