#ifndef ACTUATOR_CORE_BUS_SCHEDULE_H
#define ACTUATOR_CORE_BUS_SCHEDULE_H
#include <stdbool.h>
#include <stdint.h>
/* Timing plan only; does not invent baud/response budgets or hardware limits.
 * The caller must align epoch_us with the actual 5 ms arm output clock, reserve
 * both arm UARTs first, and pass available_us to the shared router. Poll from
 * ONE main-loop owner at least once per maximum_poll_gap_us. No catch-up burst.
 * Periodic due bits mean submit fresh data; never repeat a stale command. */
#define ACTUATOR_SCHEDULE_WHEELS UINT8_C(1)
#define ACTUATOR_SCHEDULE_LIFT UINT8_C(2)
#define ACTUATOR_SCHEDULE_FEEDBACK UINT8_C(4)
typedef struct {
    uint32_t arm_period_us, arm_reserved_us, guard_us;
    uint32_t wheels_period_us, lift_period_us, feedback_period_us;
    uint32_t maximum_poll_gap_us;
} actuator_bus_schedule_config_t;
typedef struct {
    actuator_bus_schedule_config_t config;
    uint32_t last_poll_us, phase_us, due_in_us[3];
    bool initialized, faulted;
} actuator_bus_schedule_t;
typedef struct {
    uint32_t available_us;
    uint8_t due;
    bool arm_reserved, stop_required;
} actuator_bus_schedule_window_t;
bool actuator_bus_schedule_init(actuator_bus_schedule_t *schedule,
    const actuator_bus_schedule_config_t *config, uint32_t epoch_us);
actuator_bus_schedule_window_t actuator_bus_schedule_poll(
    actuator_bus_schedule_t *schedule, uint32_t now_us);
#endif
