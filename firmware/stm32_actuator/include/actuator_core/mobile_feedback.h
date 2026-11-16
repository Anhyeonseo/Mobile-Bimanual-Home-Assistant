#ifndef ACTUATOR_CORE_MOBILE_FEEDBACK_H
#define ACTUATOR_CORE_MOBILE_FEEDBACK_H
#include "actuator_core/mobile_supervisor.h"
#include "actuator_core/sts3215_response.h"
#include "actuator_core/sts3215_packet.h"
#include <stddef.h>

typedef struct {
    int8_t velocity_direction; /* device raw -> logical; lift positive is UP */
    uint8_t minimum_voltage_raw, maximum_voltage_raw, maximum_temperature;
    uint16_t maximum_current_raw; /* raw magnitude, never assumed milliamps */
} actuator_mobile_axis_feedback_config_t;
typedef struct {
    actuator_mobile_axis_feedback_config_t axes[4];
    uint32_t sample_timeout_ms, mode_timeout_ms, response_timeout_ms;
    uint32_t maximum_sample_skew_ms;
} actuator_mobile_feedback_config_t;
typedef struct {
    int32_t position_raw, velocity_raw, load_raw, current_raw;
    uint8_t voltage_raw, temperature, moving;
    uint32_t observed_ms, mode_ms;
    bool valid, mode_verified;
    /* Optional same-transaction mode/torque/goal readback (33..48). */
    bool drive_valid;
    uint8_t torque_enabled;
    int32_t goal_velocity_raw;
} actuator_mobile_axis_sample_t;
/* Supplied by the independently observed/homed lift controller. A position
 * register alone cannot establish height, home, or mechanical load holding. */
typedef struct {
    uint32_t observed_ms;
    int32_t position_um;
    bool homed, hardware_ok;
} actuator_mobile_lift_evidence_t;
typedef enum {
    MOBILE_FEEDBACK_OK = 0, MOBILE_FEEDBACK_TIMEOUT, MOBILE_FEEDBACK_STATUS,
    MOBILE_FEEDBACK_OVERFLOW, MOBILE_FEEDBACK_AMBIGUOUS, MOBILE_FEEDBACK_MODE,
    MOBILE_FEEDBACK_DEVICE, MOBILE_FEEDBACK_TRANSPORT
} actuator_mobile_feedback_fault_t;
enum {
    MOBILE_FEEDBACK_FIELD_MODE=1, MOBILE_FEEDBACK_FIELD_TORQUE=2,
    MOBILE_FEEDBACK_FIELD_VOLTAGE=4, MOBILE_FEEDBACK_FIELD_TEMPERATURE=8,
    MOBILE_FEEDBACK_FIELD_CURRENT=16, MOBILE_FEEDBACK_FIELD_MOVING=32,
    MOBILE_FEEDBACK_FIELD_LOAD=64
};
typedef struct {
    actuator_mobile_feedback_fault_t fault;
    uint32_t started_ms;
    actuator_mobile_axis_sample_t previous;
    actuator_sts_response_t response;
    uint8_t axis, fields;
    bool reading_mode;
} actuator_mobile_feedback_failure_t;
typedef struct {
    actuator_mobile_supervisor_t *supervisor;
    actuator_mobile_feedback_config_t config;
    actuator_mobile_axis_sample_t axes[4];
    actuator_sts_response_t parser;
    actuator_mobile_feedback_fault_t fault;
    uint32_t token, started_ms;
    uint8_t axis, cursor, dirty, received_bytes;
    bool configured, active, reading_mode, reply_ready;
    bool read_drive_state;
    /* Opt-in for commissioned unified wheels: reject a lone implausible
     * temperature jump and reread this axis without renewing its sample age. */
    bool confirm_temperature_spike, temperature_pending;
    uint32_t temperature_rechecks, temperature_recovered;
    uint32_t temperature_last_recheck_ms[3];
    uint8_t temperature_recheck_seen_mask;
    uint8_t rejected_fields;
    uint32_t failure_count;
    actuator_mobile_feedback_failure_t first_failure;
} actuator_mobile_feedback_reader_t;

/* Single main-loop owner. Bind only while DISABLED, clearing previous evidence.
 * No HAL calls, writes, EEPROM/mode changes, homing or automatic arming.
 * Limits/directions/timing must be commissioned; this API provides no defaults. */
bool actuator_mobile_feedback_reader_init(actuator_mobile_feedback_reader_t *r,
    actuator_mobile_supervisor_t *s, const actuator_mobile_feedback_config_t *config);
/* Round-robin READ: refresh mode 33=1 before telemetry registers 56..70.
 * Call only after acquiring the physical UART and flushing/verifying RX quiet.
 * Timestamp is actual request start, conservatively older than the response.
 * Monotonic token cannot wrap; start a new reader only at a new boot boundary. */
bool actuator_mobile_feedback_begin(actuator_mobile_feedback_reader_t *r,
    uint32_t now_ms, bool rx_quiet_verified,
    uint8_t request[ACTUATOR_STS3215_READ_PACKET_SIZE], uint32_t *token);
/* Feed only bytes in that transaction's DMA window. A completed reply remains
 * private until commit; duplicate/trailing bytes before commit fault the read. */
bool actuator_mobile_feedback_feed(actuator_mobile_feedback_reader_t *r,
    uint32_t token, const uint8_t *bytes, size_t length, uint32_t now_ms);
/* Physical owner must prove TX complete AND RX quiet before release/commit.
 * False proof keeps ownership logically pending; deadlines still run. */
bool actuator_mobile_feedback_commit(actuator_mobile_feedback_reader_t *r,
    uint32_t token, uint32_t now_ms, bool tx_completed, bool rx_quiet_verified);
void actuator_mobile_feedback_reader_poll(actuator_mobile_feedback_reader_t *r, uint32_t now_ms);
void actuator_mobile_feedback_fail(actuator_mobile_feedback_reader_t *r);
/* Publish only a new, complete, fresh four-axis sweep. Slow/missing axes cannot
 * be hidden by another axis or mode refresh. Includes independent lift age.
 * NULL lift publishes unhomed/unhealthy evidence, never zero-height success. */
bool actuator_mobile_feedback_publish(actuator_mobile_feedback_reader_t *r,
    const actuator_mobile_lift_evidence_t *lift, uint32_t now_ms);
/* Explicit recovery invalidates ALL samples/modes and preserves token counter.
 * Caller must abort/drain pending I/O and keep all motion outputs inhibited.
 * Never releases the supervisor STOP latch or resumes a prior command. */
bool actuator_mobile_feedback_reset(actuator_mobile_feedback_reader_t *r,
    bool transport_idle_and_quiet, bool motion_outputs_inhibited);
typedef struct {
    uint32_t first_ms,last_ms;
    int32_t anchor,low,high;
    uint8_t samples;
    bool tracking;
} actuator_mobile_stationary_window_t;
/* Same powered maintenance evidence: zero goal readback, >=6 independent
 * samples across >=400 ms, total encoder span <=2, bounded speed indication.
 * Only supplements STOP proof after an actual zero write; never motion/ARM. */
bool actuator_mobile_wheels_stop_observe(actuator_mobile_stationary_window_t windows[3],
    const actuator_mobile_feedback_reader_t *reader,uint32_t zero_written_ms,uint32_t now_ms);
#endif
