#ifndef ACTUATOR_CORE_SYSTEM_STOP_H
#define ACTUATOR_CORE_SYSTEM_STOP_H
#include "actuator_core/bus_router.h"
#include <stdint.h>
typedef struct {
    uint32_t observed_ms;
    bool base_stopped,lift_holding,arms_holding,load_retained;
} actuator_stop_proof_t;
typedef enum { SYSTEM_STOP_IDLE=0,SYSTEM_STOP_PENDING,SYSTEM_STOP_CONFIRMED,SYSTEM_STOP_UNCONFIRMED } actuator_system_stop_state_t;
typedef struct {
    uint32_t job_lifetime_us, zero_budget_us, hold_budget_us;
    uint32_t timeout_ms, feedback_max_age_ms;
} actuator_system_stop_config_t;
typedef struct {
    actuator_system_stop_state_t state;
    uint32_t started_ms,timeout_ms,feedback_max_age_ms;
    bool preserve_load,arm_feedback_valid,left_hold_queued,actions_completed;
    uint32_t actions_completed_ms;
    uint16_t left_positions[6], right_positions[6];
    actuator_system_stop_config_t config;
    uint32_t left_zero_after_token,left_hold_after_token,right_hold_after_token;
} actuator_system_stop_t;
/* Zero-initialize this structure before first use; do not restart a pending stop.
 * Queue a single zero-speed frame for IDs8..11 and position-hold frames for
 * both arms. Input arm poses MUST be fresh, validated modulo servo positions.
 * No torque-disable is issued. This command is not proof of lift load holding.
 * If arm evidence is invalid, only mobile zero can be queued and false is
 * returned; caller keeps fault ownership and uses its escalation policy. */
bool actuator_system_stop_begin(actuator_system_stop_t *stop,
    actuator_bus_router_t *left,actuator_bus_router_t *right,
    const uint16_t left_positions[6],const uint16_t right_positions[6],
    bool arm_feedback_valid,bool preserve_load,uint32_t now_ms,
    uint32_t timeout_ms,uint32_t feedback_max_age_ms);
void actuator_system_stop_poll(actuator_system_stop_t *stop,actuator_bus_router_t *left,
    actuator_bus_router_t *right,const actuator_stop_proof_t *proof,uint32_t now_ms);
/* Hardware uses the independent microsecond clock. HAL_GetTick()*1000 is not
 * interchangeable with Timebase_NowUs() after different timer start epochs. */
bool actuator_system_stop_begin_timed(actuator_system_stop_t *stop,
    actuator_bus_router_t *left,actuator_bus_router_t *right,
    const uint16_t left_positions[6],const uint16_t right_positions[6],
    bool arm_feedback_valid,bool preserve_load,uint32_t now_us,uint32_t now_ms,
    const actuator_system_stop_config_t *config);
void actuator_system_stop_poll_timed(actuator_system_stop_t *stop,
    actuator_bus_router_t *left,actuator_bus_router_t *right,
    const actuator_stop_proof_t *proof,uint32_t now_us,uint32_t now_ms);
#endif
