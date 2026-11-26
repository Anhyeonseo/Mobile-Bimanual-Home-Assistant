#ifndef ACTUATOR_CORE_LIFT_CONTROLLER_H
#define ACTUATOR_CORE_LIFT_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { LIFT_UNHOMED=0, LIFT_HOMING, LIFT_READY, LIFT_MOVING,
               LIFT_HOLD_PENDING, LIFT_HOLDING, LIFT_FAULT } actuator_lift_state_t;
typedef enum { LIFT_OK=0, LIFT_BAD_FEEDBACK, LIFT_TIMEOUT, LIFT_ENCODER_JUMP,
               LIFT_TRAVEL_LIMIT, LIFT_OVER_CURRENT } actuator_lift_fault_t;
typedef struct {
    int32_t um_per_turn, maximum_height_um, tolerance_um;
    int32_t maximum_speed_raw, homing_speed_raw, stopped_speed_raw;
    int32_t slowdown_distance_um, home_current_ma, maximum_current_ma;
    uint32_t feedback_timeout_ms, homing_timeout_ms, contact_dwell_ms;
    uint16_t maximum_encoder_step_raw;
    int8_t encoder_up_direction;
    int32_t reference_top_um; /* Mechanical upper datum; 0 disables top registration. */
} actuator_lift_config_t;
typedef struct {
    uint32_t observed_ms;
    uint16_t position_raw;
    int32_t velocity_raw, current_ma;
    bool healthy, hold_verified;
    uint8_t hold_report_remaining_ms; /* Source sample lifetime for position-backed hold; never a motion request. */
    bool bottom_transport_verified;
    uint32_t bottom_drive_observed_ms;
} actuator_lift_feedback_t;
typedef struct {
    uint32_t first_ms,last_ms,last_quiet_ms,report_started_ms;
    uint16_t anchor_raw,quiet_position_raw;
    uint8_t samples;
    bool tracking,proven,report_pending;
} actuator_lift_hold_monitor_t;
/* Observed stationary position plus fresh torque-ON/zero-goal readback.
 * Never accepts a desired-command echo as zero_drive_verified. */
bool actuator_lift_hold_observe(actuator_lift_hold_monitor_t *monitor,
    const actuator_lift_feedback_t *feedback,bool zero_drive_verified,
    uint32_t now_ms,uint32_t maximum_age_ms,uint32_t dwell_ms,uint16_t tolerance_raw);
/* Existing HOLDING only: each fresh position inside the fixed hold envelope plus zero-drive
 * readback retains hold for <100 ms despite bounded speed indication.
 * Repeated timestamps never renew the source sample lifetime.
 * Does not establish initial hold, home, moving completion or STOP proof. */
bool actuator_lift_hold_observe_report(actuator_lift_hold_monitor_t *monitor,
    actuator_lift_feedback_t *feedback,bool zero_drive_verified,uint8_t moving,
    bool maintaining_hold,uint32_t now_ms,uint32_t maximum_age_ms,
    uint32_t dwell_ms,uint16_t tolerance_raw,uint16_t hold_tolerance_raw);
typedef struct {
    actuator_lift_config_t config;
    actuator_lift_state_t state;
    actuator_lift_fault_t fault;
    bool configured, have_sample, homed, contact_tracking;
    uint32_t observed_ms, homing_started_ms, contact_started_ms;
    uint16_t previous_raw;
    int64_t accumulated_raw, home_raw;
    int32_t height_um, target_um, command_raw, reference_height_um;
    actuator_lift_feedback_t feedback;
    bool zero_sent;
    uint32_t zero_sent_ms;
} actuator_lift_t;
/* No defaults or auto-homing. Positive logical command is UP; physical motor
 * sign and speed scale are the adapter's responsibility. Current thresholds,
 * braking margin and verified hold behavior require physical commissioning. */
/* Base-only evidence: preserve raw lift height. Registered bottom transport
 * uses the commissioned hold envelope; legacy quiet hold uses <=2 counts. */
bool actuator_lift_top_hold_verified(const actuator_lift_t *lift, uint32_t now_ms);
int32_t actuator_lift_base_height(const actuator_lift_t *lift,uint32_t now_ms);
/* Existing registered-bottom HOLDING only. Use the commissioned position
 * envelope and independently read zero-drive state, not speed/moving bits.
 * Does not establish home, initial hold, or completion of a lift movement. */
bool actuator_lift_bottom_transport_observe(const actuator_lift_t *lift,
    const actuator_lift_feedback_t *feedback,bool zero_drive_verified,
    uint32_t drive_observed_ms,uint32_t now_ms);
bool actuator_lift_bottom_transport_ready(const actuator_lift_t *lift,uint32_t now_ms);

bool actuator_lift_init(actuator_lift_t *lift,const actuator_lift_config_t *config);
bool actuator_lift_observe(actuator_lift_t *lift,const actuator_lift_feedback_t *feedback,uint32_t now_ms);
bool actuator_lift_home(actuator_lift_t *lift,uint32_t now_ms);
/* Explicit operator-confirmed bottom reference. No motion or torque writes;
 * normal zero completion and independent hold proof are still required. */
bool actuator_lift_reference_bottom(actuator_lift_t *lift,uint32_t now_ms);
/* Register the configured mechanical top without moving. The command ceiling
 * remains maximum_height_um, including when the top datum is above it. */
bool actuator_lift_reference_top(actuator_lift_t *lift,int32_t expected_top_um,uint32_t now_ms);
bool actuator_lift_move(actuator_lift_t *lift,int32_t height_um,uint32_t now_ms);
/* Actual zero-speed packet completion; not the desired command value. */
void actuator_lift_zero_sent(actuator_lift_t *lift,uint32_t completed_ms);
void actuator_lift_cancel(actuator_lift_t *lift);
void actuator_lift_poll(actuator_lift_t *lift,uint32_t now_ms);
/* Fault recovery loses the home reference and requires fresh observation and
 * explicit homing. It never restarts a previous height command. */
void actuator_lift_inhibit(actuator_lift_t *lift,actuator_lift_fault_t reason);
void actuator_lift_reset(actuator_lift_t *lift);
#endif
