#ifndef ACTUATOR_CORE_MOBILE_SUPERVISOR_H
#define ACTUATOR_CORE_MOBILE_SUPERVISOR_H
#include <stdbool.h>
#include <stdint.h>
#define ACTUATOR_MOBILE_AXES 4u /* wheel IDs 8,9,10; lift ID 11 */

typedef struct {
    int32_t max_velocity_raw[ACTUATOR_MOBILE_AXES];
    int32_t stopped_velocity_raw;
    uint32_t command_timeout_ms, feedback_timeout_ms;
    int32_t lift_min_um, lift_max_um;
} actuator_mobile_config_t;
typedef struct {
    uint32_t observed_ms;
    int32_t velocity_raw[ACTUATOR_MOBILE_AXES];
    int32_t lift_position_um;
    bool velocity_modes_verified, lift_homed, hardware_ok;
} actuator_mobile_feedback_t;
typedef enum {
    ACTUATOR_MOBILE_DISABLED = 0, ACTUATOR_MOBILE_READY,
    ACTUATOR_MOBILE_ACTIVE, ACTUATOR_MOBILE_STOP_LATCHED
} actuator_mobile_state_t;
typedef enum {
    ACTUATOR_MOBILE_REASON_NONE = 0, ACTUATOR_MOBILE_REASON_REQUEST,
    ACTUATOR_MOBILE_REASON_COMMAND_TIMEOUT, ACTUATOR_MOBILE_REASON_FEEDBACK,
    ACTUATOR_MOBILE_REASON_LIFT_LIMIT
} actuator_mobile_reason_t;
typedef struct {
    actuator_mobile_config_t config;
    actuator_mobile_feedback_t feedback;
    actuator_mobile_state_t state;
    actuator_mobile_reason_t reason;
    bool configured, have_feedback, have_sequence;
    uint32_t session, sequence, command_ms, command_valid_for_ms;
    int32_t target_raw[ACTUATOR_MOBILE_AXES];
} actuator_mobile_supervisor_t;

/* No physical defaults: caller supplies measured limits and time budgets.
 * Positive lift velocity must mean UP in this interface; the hardware adapter
 * must establish/convert the physical motor direction before use. Feedback
 * observed_ms is the oldest sample of ALL four axes, never a receive timestamp.
 * This boundary check is not a braking-distance or lift holding controller.
 * Single main-loop owner. Outputs are requested register speeds only; zero
 * does not mean stopped, lift load held, torque disabled, or emergency stop. */
bool actuator_mobile_init(actuator_mobile_supervisor_t *s,
    const actuator_mobile_config_t *config);
bool actuator_mobile_feedback(actuator_mobile_supervisor_t *s,
    const actuator_mobile_feedback_t *feedback, uint32_t now_ms);
/* Explicit re-arm with strictly increasing nonzero session; requires fresh,
 * healthy, homed feedback and all four axes measured stationary. No auto-resume. */
bool actuator_mobile_arm(actuator_mobile_supervisor_t *s,
    uint32_t session, uint32_t now_ms);
/* Receive clock is local STM32 time; host wire expiry is a separate admission
 * check. Sequence must increase within session (renew session before wrap).
 * One command specifies all four axes to avoid reviving an old lift target. */
bool actuator_mobile_command(actuator_mobile_supervisor_t *s, uint32_t session,
    uint32_t sequence, const int32_t velocity_raw[ACTUATOR_MOBILE_AXES],
    uint32_t now_ms);
/* Wire deadline expressed in synchronized MCU time. Accepted remaining life
 * must be positive and <= configured command timeout; expiry cannot be renewed
 * by delayed transport. Existing command() uses the configured timeout. */
bool actuator_mobile_command_until(actuator_mobile_supervisor_t *s, uint32_t session,
    uint32_t sequence, const int32_t velocity_raw[ACTUATOR_MOBILE_AXES],
    uint32_t valid_until_ms, uint32_t now_ms);
void actuator_mobile_stop(actuator_mobile_supervisor_t *s);
void actuator_mobile_poll(actuator_mobile_supervisor_t *s, uint32_t now_ms);
/* Initialization/homing only: checks fresh healthy modes and all velocities,
 * without claiming a valid lift origin or height. */
bool actuator_mobile_axes_stopped(const actuator_mobile_supervisor_t *s,uint32_t now_ms);
bool actuator_mobile_measured_stopped(const actuator_mobile_supervisor_t *s,
    uint32_t now_ms);
#endif
