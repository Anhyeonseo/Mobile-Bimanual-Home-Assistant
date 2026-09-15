#ifndef ACTUATOR_CORE_MOBILE_OUTPUT_H
#define ACTUATOR_CORE_MOBILE_OUTPUT_H
#include "actuator_core/mobile_supervisor.h"
#include "actuator_core/bus_schedule.h"
#include "actuator_core/bus_router.h"
#include "actuator_core/lift_endpoint.h"
/* One main-loop owner. Produces bounded broadcast writes only; feedback must
 * come from a separate real RX decoder, never from transmitted target values. */
typedef enum { MOBILE_OUTPUT_OK=0, MOBILE_OUTPUT_SCHEDULE, MOBILE_OUTPUT_QUEUE,
               MOBILE_OUTPUT_TRANSPORT, MOBILE_OUTPUT_COMMAND_EXPIRING } actuator_mobile_output_fault_t;
typedef struct {
    actuator_bus_schedule_config_t schedule;
    uint32_t wheels_budget_us, lift_budget_us, stop_budget_us, job_lifetime_us;
    int8_t velocity_direction[4]; /* logical -> motor; must match RX signs */
} actuator_mobile_output_config_t;
typedef struct {
    actuator_mobile_supervisor_t *supervisor;
    actuator_mobile_output_config_t config;
    actuator_bus_schedule_t schedule;
    actuator_bus_router_t router;
    actuator_mobile_output_fault_t fault;
    bool configured, stop_requested, stop_written, feedback_due;
    uint32_t available_us, writes_completed, stop_written_ms;
    actuator_lift_endpoint_t *lift_control;
    bool lift_engaged;
} actuator_mobile_output_t;
/* Boot-only init. No measured limits, baud, turnaround or current are invented.
 * poll receives the latest actual arm ISR epoch without consuming its event. */
bool actuator_mobile_output_init(actuator_mobile_output_t *output,
    actuator_mobile_supervisor_t *supervisor,
    const actuator_mobile_output_config_t *config, uint32_t now_us);
void actuator_mobile_output_poll(actuator_mobile_output_t *output,
    uint32_t now_us, uint32_t now_ms, uint32_t arm_epoch_us);
/* Bind before any mobile ARM. Homing may use lift-only output while the mobile
 * supervisor is DISABLED, with wheel-zero and independent base/arm interlocks. */
bool actuator_mobile_output_bind_lift(actuator_mobile_output_t *output,actuator_lift_endpoint_t *endpoint);
void actuator_mobile_output_stop(actuator_mobile_output_t *output,
    actuator_mobile_output_fault_t fault);
/* Called only after router+UART completion, including the configured quiet
 * interval. stop_written is a wire result, NOT measured stop/load retention. */
void actuator_mobile_output_completed(actuator_mobile_output_t *output,
    actuator_bus_work_t kind, uint32_t completed_ms);
/* Explicit whole-system stop proof plus newer session. UART/router ownership
 * must already be recovered and idle; failed checks cannot clear the latch. */
bool actuator_mobile_output_rearm(actuator_mobile_output_t *output,
    uint32_t session, uint32_t epoch_us, uint32_t now_ms, bool whole_stop_confirmed);
#endif
