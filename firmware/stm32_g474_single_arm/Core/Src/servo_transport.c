#include "servo_transport.h"
#include <stddef.h>
#include <limits.h>
#include "timebase.h"

typedef struct {
    UART_HandleTypeDef *uart;
    uint32_t token;
    actuator_bus_work_t kind;
    bool active;
    bool stop_required;
    bool transmit_allowed;
    bool read_only, read_sent;
} ServoTransportOwner;

static ServoTransportOwner buses[2];
static bool periodic_mode;

static ServoTransportOwner *find_bus(UART_HandleTypeDef *uart)
{
    if (uart == NULL) return NULL;
    for (unsigned i = 0; i < 2; ++i)
        if (buses[i].uart == uart) return &buses[i];
    return NULL;
}

static bool owns(ServoTransportOwner *bus, uint32_t token)
{
    return bus != NULL && bus->active && token != 0 && bus->token == token;
}

bool ServoTransport_Register(UART_HandleTypeDef *uart)
{
    if (uart == NULL) return false;
    if (find_bus(uart) != NULL) return true;
    for (unsigned i = 0; i < 2; ++i) {
        if (buses[i].uart == NULL) {
            buses[i].uart = uart;
            return true;
        }
    }
    return false;
}

bool ServoTransport_Idle(UART_HandleTypeDef *uart)
{
    ServoTransportOwner *bus = find_bus(uart);
    return bus != NULL && !bus->active && !bus->stop_required &&
        uart->gState == HAL_UART_STATE_READY &&
        __HAL_UART_GET_FLAG(uart, UART_FLAG_BUSY) == RESET;
}

void ServoTransport_RequestStop(UART_HandleTypeDef *uart)
{
    ServoTransportOwner *bus = find_bus(uart);
    if (bus != NULL) bus->stop_required = true;
}

bool ServoTransport_Begin(UART_HandleTypeDef *uart,
    actuator_bus_work_t kind, uint32_t *token)
{
    ServoTransportOwner *bus = find_bus(uart);
    if (bus == NULL || token == NULL || (unsigned)kind >= ACTUATOR_BUS_WORK_COUNT ||
        bus->active || uart->gState != HAL_UART_STATE_READY ||
        __HAL_UART_GET_FLAG(uart, UART_FLAG_BUSY) != RESET ||
        (bus->stop_required && kind != ACTUATOR_BUS_WORK_STOP) ||
        bus->token == UINT32_MAX) return false;
    if (kind == ACTUATOR_BUS_WORK_STOP) bus->stop_required = true;
    bus->active = true;
    bus->transmit_allowed = true;
    bus->read_only = false; bus->read_sent = false;
    bus->kind = kind;
    *token = ++bus->token;
    return true;
}

bool ServoTransport_SetPeriodicMode(bool enabled, bool quiescent_confirmed)
{
    if (!quiescent_confirmed) return false;
    for (unsigned i = 0; i < 2; ++i)
        if (buses[i].uart != NULL && !ServoTransport_Idle(buses[i].uart)) return false;
    periodic_mode = enabled;
    return true;
}

bool ServoTransport_ServiceAllowed(void) { return !periodic_mode; }

bool ServoTransport_BeginService(UART_HandleTypeDef *uart,
    actuator_bus_work_t kind, uint32_t *token)
{
    return !periodic_mode && ServoTransport_Begin(uart, kind, token);
}

bool ServoTransport_BeginCleanup(UART_HandleTypeDef *uart, uint32_t *token)
{
    ServoTransportOwner *bus = find_bus(uart);
    if (bus == NULL || token == NULL || bus->active || bus->token == UINT32_MAX)
        return false;
    bus->active = true;
    bus->transmit_allowed = false;
    bus->kind = ACTUATOR_BUS_WORK_FEEDBACK;
    *token = ++bus->token;
    return true;
}

bool ServoTransport_Owns(UART_HandleTypeDef *uart, uint32_t token)
{
    return owns(find_bus(uart), token);
}

bool ServoTransport_BeginReadOnly(UART_HandleTypeDef *uart, uint32_t *token)
{
    ServoTransportOwner *bus = find_bus(uart);
    if (bus == NULL || token == NULL || bus->active || bus->token == UINT32_MAX ||
        uart->gState != HAL_UART_STATE_READY ||
        __HAL_UART_GET_FLAG(uart, UART_FLAG_BUSY) != RESET) return false;
    bus->active = true; bus->transmit_allowed = true;
    bus->read_only = true; bus->read_sent = false;
    bus->kind = ACTUATOR_BUS_WORK_FEEDBACK; *token = ++bus->token;
    return true;
}

static bool may_transmit(UART_HandleTypeDef *uart, uint32_t token,
    const uint8_t *data, uint16_t length)
{
    ServoTransportOwner *bus = find_bus(uart);
    if (!owns(bus, token) || !bus->transmit_allowed || data == NULL || !length) return false;
    if (!bus->read_only) return true;
    /* Inverted sum, ID through last parameter. Limit responses to our parser. */
    if (bus->read_sent || length != 8 || data[0] != 255 || data[1] != 255 ||
        !data[2] || data[2] >= 254 || data[3] != 4 || data[4] != 2 ||
        !data[6] || data[6] > 16 || (unsigned)data[5] + data[6] > 256) return false;
    unsigned sum = 0;
    for (unsigned i = 2; i < 8; ++i) sum += data[i];
    if ((sum & 255u) != 255u) return false;
    /* A failed/partial HAL launch cannot be retried using the same lease. */
    bus->read_sent = true;
    return true;
}

HAL_StatusTypeDef ServoTransport_End(UART_HandleTypeDef *uart,
    uint32_t token, bool abort_tx)
{
    ServoTransportOwner *bus = find_bus(uart);
    if (!owns(bus, token)) return HAL_ERROR;
    if (abort_tx && HAL_UART_AbortTransmit(uart) != HAL_OK) return HAL_ERROR;
    if (uart->gState != HAL_UART_STATE_READY ||
        __HAL_UART_GET_FLAG(uart, UART_FLAG_BUSY) != RESET) return HAL_BUSY;
    bus->active = false;
    return HAL_OK;
}

HAL_StatusTypeDef ServoTransport_Transmit(UART_HandleTypeDef *uart,
    uint32_t token, const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if (!may_transmit(uart, token, data, length)) return HAL_ERROR;
    if (periodic_mode) return HAL_ERROR;
    return HAL_UART_Transmit(uart, data, length, timeout_ms);
}

HAL_StatusTypeDef ServoTransport_TransmitIT(UART_HandleTypeDef *uart,
    uint32_t token, const uint8_t *data, uint16_t length)
{
    if (!may_transmit(uart, token, data, length)) return HAL_ERROR;
    return HAL_UART_Transmit_IT(uart, data, length);
}

HAL_StatusTypeDef ServoTransport_TransmitDMA(UART_HandleTypeDef *uart,
    uint32_t token, const uint8_t *data, uint16_t length)
{
    if (!may_transmit(uart, token, data, length)) return HAL_ERROR;
    return HAL_UART_Transmit_DMA(uart, data, length);
}

bool ServoTransport_ClaimQueued(UART_HandleTypeDef *uart,
    actuator_bus_router_t *router, uint32_t available_us,
    ServoQueuedTransaction *transaction, const uint8_t **bytes,
    size_t *length, actuator_bus_work_t *kind)
{
    uint32_t gate_token, router_token;
    if (router == NULL || transaction == NULL || transaction->active ||
        bytes == NULL || length == NULL || kind == NULL) return false;
    if (router->inhibited || router->bus.stop_pending || router->bus.faulted)
        ServoTransport_RequestStop(uart);
    /* Never dequeue a mobile job while legacy code owns this UART. */
    if (!ServoTransport_Begin(uart, router->jobs[ACTUATOR_BUS_WORK_STOP].pending ?
            ACTUATOR_BUS_WORK_STOP : ACTUATOR_BUS_WORK_FEEDBACK, &gate_token))
        return false;
    if (!actuator_bus_router_next(router, Timebase_NowUs(), available_us,
            bytes, length, &router_token, kind)) {
        if (router->inhibited || router->bus.stop_pending || router->bus.faulted)
            ServoTransport_RequestStop(uart);
        (void)ServoTransport_End(uart, gate_token, false);
        return false;
    }
    find_bus(uart)->kind = *kind;
    *transaction = (ServoQueuedTransaction){uart, router, gate_token, router_token, true};
    return true;
}

HAL_StatusTypeDef ServoTransport_CompleteQueued(
    ServoQueuedTransaction *transaction, bool transport_ok)
{
    if (transaction == NULL || !transaction->active ||
        !owns(find_bus(transaction->uart), transaction->gate_token)) return HAL_ERROR;
    actuator_shared_bus_poll(&transaction->router->bus, Timebase_NowUs());
    if (transaction->router->bus.faulted) {
        transaction->router->inhibited = true;
        ServoTransport_RequestStop(transaction->uart);
        return HAL_ERROR;
    }
    if (transport_ok && (transaction->uart->gState != HAL_UART_STATE_READY ||
        __HAL_UART_GET_FLAG(transaction->uart, UART_FLAG_BUSY) != RESET))
        return HAL_BUSY;
    if (!actuator_bus_router_complete(transaction->router, transaction->router_token,
            Timebase_NowUs(), transport_ok)) {
        transaction->router->inhibited = true;
        ServoTransport_RequestStop(transaction->uart);
        return HAL_ERROR;
    }
    /* Commit the two logical releases together in this main-loop call. Do not
     * re-check a changing UART flag after completing only the router half.
     * Any new line activity is checked again before the next Begin. */
    find_bus(transaction->uart)->active = false;
    transaction->active = false;
    return HAL_OK;
}

HAL_StatusTypeDef ServoTransport_RecoverQueued(
    ServoQueuedTransaction *transaction, bool rx_quiet_verified)
{
    if (transaction == NULL || !transaction->active || !rx_quiet_verified ||
        !owns(find_bus(transaction->uart), transaction->gate_token)) return HAL_ERROR;
    (void)actuator_bus_router_complete(transaction->router, transaction->router_token,
        Timebase_NowUs(), false);
    transaction->router->inhibited = true;
    ServoTransport_RequestStop(transaction->uart);
    HAL_StatusTypeDef status = ServoTransport_End(transaction->uart,
        transaction->gate_token, true);
    if (status != HAL_OK) return status;
    if (!actuator_shared_bus_recover(&transaction->router->bus, true)) return HAL_ERROR;
    transaction->active = false;
    return HAL_OK;
}

bool ServoTransport_ResumeQueued(UART_HandleTypeDef *uart,
    actuator_bus_router_t *router, bool stop_confirmed)
{
    ServoTransportOwner *bus = find_bus(uart);
    if (bus == NULL || bus->active || uart->gState != HAL_UART_STATE_READY ||
        __HAL_UART_GET_FLAG(uart, UART_FLAG_BUSY) != RESET ||
        !actuator_bus_router_resume(router, stop_confirmed)) return false;
    bus->stop_required = false;
    return true;
}
