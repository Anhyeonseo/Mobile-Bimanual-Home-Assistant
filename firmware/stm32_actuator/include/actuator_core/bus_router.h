#ifndef ACTUATOR_CORE_BUS_ROUTER_H
#define ACTUATOR_CORE_BUS_ROUTER_H
#include "actuator_core/shared_bus.h"
#include <stddef.h>
#define ACTUATOR_ROUTER_PACKET_MAX 32u
typedef struct {bool pending;uint8_t bytes[32];size_t length;uint32_t deadline_us,budget_us;} actuator_bus_job_t;
typedef struct {
    actuator_shared_bus_t bus;
    actuator_bus_job_t jobs[ACTUATOR_BUS_WORK_COUNT];
    uint8_t active_bytes[32];size_t active_length;
    uint32_t expired_jobs;
    bool inhibited;
    unsigned cursor;
    /* Immutable last successful TX/quiet receipt. Claims, expiry and recovery
     * never advance it; STOP proof must identify the exact transmitted bytes. */
    uint32_t completed_token, completed_us;
    actuator_bus_work_t completed_kind;
    uint8_t completed_bytes[ACTUATOR_ROUTER_PACKET_MAX];
    size_t completed_length;
} actuator_bus_router_t;
/* One router per UART, including left arm + wheels + lift. Single owner thread.
 * All jobs include TX + RX/quiet-time budget, and remain owned until completion.
 * Callers supply validated arm packets, mobile frames, and telemetry requests.
 * STOP discards queued motion; in-flight TX must finish/abort before next work. */
void actuator_bus_router_init(actuator_bus_router_t *router);
bool actuator_bus_router_submit(actuator_bus_router_t *router,actuator_bus_work_t kind,
    const uint8_t *bytes,size_t length,uint32_t now_us,uint32_t deadline_us,uint32_t budget_us);
/* Returns pointer valid until complete/recover; suitable for HAL DMA lifetime.
 * Next arm slot is a hard available-time budget for every normal job. */
bool actuator_bus_router_next(actuator_bus_router_t *router,uint32_t now_us,uint32_t available_us,
    const uint8_t **bytes,size_t *length,uint32_t *token,actuator_bus_work_t *kind);
bool actuator_bus_router_complete(actuator_bus_router_t *router,uint32_t token,uint32_t now_us,bool ok);
/* Resume only after system stop proof and explicit reactivation. */
bool actuator_bus_router_resume(actuator_bus_router_t *router,bool stop_confirmed);
#endif
