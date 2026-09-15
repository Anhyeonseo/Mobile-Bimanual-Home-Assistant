#ifndef SERVO_TRANSPORT_H
#define SERVO_TRANSPORT_H

#include "stm32g4xx_hal.h"
#include "actuator_core/shared_bus.h"
#include "actuator_core/bus_router.h"
#include <stdbool.h>
#include <stdint.h>

/* Board transport gate, one entry per PHYSICAL UART. Main-loop calls only.
 * The scheduler decides deadlines/STOP priority; this gate also covers legacy
 * synchronous service calls and holds ownership through their response phase.
 * A TX callback never releases a response-bearing transaction.
 * Register is idempotent and never resets an existing owner's identity. */
bool ServoTransport_Register(UART_HandleTypeDef *uart);
/* Switch only with independent whole-system quiescence proof and both buses
 * idle. Default remains legacy/maintenance. Periodic mode rejects blocking
 * service calls; IT/DMA and explicit error cleanup remain available. */
bool ServoTransport_SetPeriodicMode(bool enabled, bool quiescent_confirmed);
bool ServoTransport_ServiceAllowed(void);
bool ServoTransport_BeginService(UART_HandleTypeDef *uart,
    actuator_bus_work_t kind, uint32_t *token);
bool ServoTransport_Begin(UART_HandleTypeDef *uart,
    actuator_bus_work_t kind, uint32_t *token);
/* Receiver teardown may need to acquire an unowned but noisy UART while STOP
 * is latched. Cleanup leases cannot transmit and cannot preempt an owner. */
/* A read-only lease can obtain new stop evidence while motion stays inhibited.
 * Transmit functions enforce a single valid unicast READ packet (no writes,
 * broadcasts or second request); caller still reserves TX/RX/quiet time. */
bool ServoTransport_BeginReadOnly(UART_HandleTypeDef *uart, uint32_t *token);
bool ServoTransport_Owns(UART_HandleTypeDef *uart, uint32_t token);
bool ServoTransport_BeginCleanup(UART_HandleTypeDef *uart, uint32_t *token);
/* Caller has consumed/drained RX, including required turnaround/quiet time.
 * abort_tx is for a failed/cancelled transaction only. Failed abort or a busy
 * transmitter keeps ownership. End cannot release another transaction. */
HAL_StatusTypeDef ServoTransport_End(UART_HandleTypeDef *uart,
    uint32_t token, bool abort_tx);
bool ServoTransport_Idle(UART_HandleTypeDef *uart);
void ServoTransport_RequestStop(UART_HandleTypeDef *uart);
HAL_StatusTypeDef ServoTransport_Transmit(UART_HandleTypeDef *uart,
    uint32_t token, const uint8_t *data, uint16_t length, uint32_t timeout_ms);
HAL_StatusTypeDef ServoTransport_TransmitIT(UART_HandleTypeDef *uart,
    uint32_t token, const uint8_t *data, uint16_t length);
HAL_StatusTypeDef ServoTransport_TransmitDMA(UART_HandleTypeDef *uart,
    uint32_t token, const uint8_t *data, uint16_t length);

typedef struct {
    UART_HandleTypeDef *uart;
    actuator_bus_router_t *router;
    uint32_t gate_token, router_token;
    bool active;
} ServoQueuedTransaction;

/* Zero-initialize transaction. Claim BOTH scheduler and board ownership before
 * touching UART RX or TX. bytes remains owned by the router through completion.
 * Caller prepares RX if needed, then uses Transmit/IT/DMA with gate_token.
 * Complete only after TX, response parsing and quiet time have finished.
 * This does not install a periodic scheduler or enable mobile motion at boot. */
bool ServoTransport_ClaimQueued(UART_HandleTypeDef *uart,
    actuator_bus_router_t *router, uint32_t available_us,
    ServoQueuedTransaction *transaction, const uint8_t **bytes,
    size_t *length, actuator_bus_work_t *kind);
HAL_StatusTypeDef ServoTransport_CompleteQueued(
    ServoQueuedTransaction *transaction, bool transport_ok);
/* Explicit abort/cleanup. Caller first aborts/drains RX and verifies quiet.
 * Scheduler STOP remains pending; recovery never resumes an old velocity. */
HAL_StatusTypeDef ServoTransport_RecoverQueued(
    ServoQueuedTransaction *transaction, bool rx_quiet_verified);
bool ServoTransport_ResumeQueued(UART_HandleTypeDef *uart,
    actuator_bus_router_t *router, bool stop_confirmed);

#endif
