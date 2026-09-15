#include "actuator_core/mobile_supervisor.h"
#include <stddef.h>
#include <string.h>

static void latch(actuator_mobile_supervisor_t *s, actuator_mobile_reason_t reason) {
    if (s->state != ACTUATOR_MOBILE_STOP_LATCHED) s->reason = reason;
    s->state = ACTUATOR_MOBILE_STOP_LATCHED;
    memset(s->target_raw, 0, sizeof(s->target_raw));
}
static bool fresh(const actuator_mobile_supervisor_t *s, uint32_t now_ms) {
    return s->have_feedback &&
        (uint32_t)(now_ms - s->feedback.observed_ms) < s->config.feedback_timeout_ms;
}
static bool healthy(const actuator_mobile_supervisor_t *s, uint32_t now_ms) {
    return fresh(s, now_ms) && s->feedback.hardware_ok &&
        s->feedback.velocity_modes_verified && s->feedback.lift_homed;
}
static bool lift_inside(const actuator_mobile_supervisor_t *s) {
    return s->feedback.lift_position_um >= s->config.lift_min_um &&
        s->feedback.lift_position_um <= s->config.lift_max_um;
}
bool actuator_mobile_init(actuator_mobile_supervisor_t *s,
    const actuator_mobile_config_t *config) {
    unsigned i;
    if (s == NULL) return false;
    memset(s, 0, sizeof(*s));
    if (config == NULL || config->command_timeout_ms == 0u ||
        config->command_timeout_ms >= UINT32_C(0x80000000) ||
        config->feedback_timeout_ms == 0u ||
        config->feedback_timeout_ms >= UINT32_C(0x80000000) ||
        config->lift_min_um >= config->lift_max_um ||
        config->stopped_velocity_raw < 0 || config->stopped_velocity_raw > 32767)
        return false;
    for (i = 0u; i < ACTUATOR_MOBILE_AXES; i++) {
        if (config->max_velocity_raw[i] <= config->stopped_velocity_raw ||
            config->max_velocity_raw[i] > 32767) return false;
    }
    s->config = *config;
    s->configured = true;
    return true;
}
bool actuator_mobile_feedback(actuator_mobile_supervisor_t *s,
    const actuator_mobile_feedback_t *feedback, uint32_t now_ms) {
    unsigned i;
    if (s == NULL || feedback == NULL || !s->configured) return false;
    actuator_mobile_poll(s, now_ms);
    if ((uint32_t)(now_ms - feedback->observed_ms) >= s->config.feedback_timeout_ms)
        return false;
    if (s->have_feedback &&
        ((uint32_t)(feedback->observed_ms - s->feedback.observed_ms) == 0u ||
         (uint32_t)(feedback->observed_ms - s->feedback.observed_ms) >= UINT32_C(0x80000000)))
        return false;
    for (i = 0u; i < ACTUATOR_MOBILE_AXES; i++) {
        if (feedback->velocity_raw[i] < -32767 || feedback->velocity_raw[i] > 32767)
            return false;
    }
    s->feedback = *feedback;
    s->have_feedback = true;
    actuator_mobile_poll(s, now_ms);
    return true;
}
bool actuator_mobile_axes_stopped(const actuator_mobile_supervisor_t *s,uint32_t now_ms) {
    if(s==NULL||!s->configured||!fresh(s,now_ms)||!s->feedback.hardware_ok||!s->feedback.velocity_modes_verified)return false;
    for(unsigned i=0;i<4;i++)if(s->feedback.velocity_raw[i]>s->config.stopped_velocity_raw||
        s->feedback.velocity_raw[i]<-s->config.stopped_velocity_raw)return false;
    return true;
}
bool actuator_mobile_measured_stopped(const actuator_mobile_supervisor_t *s,
    uint32_t now_ms) {
    unsigned i;
    if (s == NULL || !s->configured || !healthy(s, now_ms) || !lift_inside(s))
        return false;
    for (i = 0u; i < ACTUATOR_MOBILE_AXES; i++) {
        if (s->feedback.velocity_raw[i] > s->config.stopped_velocity_raw ||
            s->feedback.velocity_raw[i] < -s->config.stopped_velocity_raw) return false;
    }
    return true;
}
bool actuator_mobile_arm(actuator_mobile_supervisor_t *s,
    uint32_t session, uint32_t now_ms) {
    if (s == NULL) return false;
    actuator_mobile_poll(s, now_ms);
    if (session == 0u || session <= s->session ||
        (s->state != ACTUATOR_MOBILE_DISABLED && s->state != ACTUATOR_MOBILE_STOP_LATCHED) ||
        !actuator_mobile_measured_stopped(s, now_ms)) return false;
    s->session = session;
    s->have_sequence = false;
    s->state = ACTUATOR_MOBILE_READY;
    s->reason = ACTUATOR_MOBILE_REASON_NONE;
    memset(s->target_raw, 0, sizeof(s->target_raw));
    return true;
}
void actuator_mobile_poll(actuator_mobile_supervisor_t *s, uint32_t now_ms) {
    if (s == NULL || !s->configured ||
        (s->state != ACTUATOR_MOBILE_READY && s->state != ACTUATOR_MOBILE_ACTIVE)) return;
    if (!healthy(s, now_ms)) {
        latch(s, ACTUATOR_MOBILE_REASON_FEEDBACK);
    } else if (!lift_inside(s) ||
        (s->target_raw[3] > 0 && s->feedback.lift_position_um >= s->config.lift_max_um) ||
        (s->target_raw[3] < 0 && s->feedback.lift_position_um <= s->config.lift_min_um)) {
        latch(s, ACTUATOR_MOBILE_REASON_LIFT_LIMIT);
    } else if (s->state == ACTUATOR_MOBILE_ACTIVE &&
        (uint32_t)(now_ms - s->command_ms) >= s->command_valid_for_ms) {
        latch(s, ACTUATOR_MOBILE_REASON_COMMAND_TIMEOUT);
    }
}
bool actuator_mobile_command(actuator_mobile_supervisor_t *s, uint32_t session,
    uint32_t sequence, const int32_t velocity_raw[ACTUATOR_MOBILE_AXES],
    uint32_t now_ms) {
    unsigned i;
    if (s == NULL) return false;
    actuator_mobile_poll(s, now_ms);
    if (velocity_raw == NULL || !s->configured || session != s->session ||
        (s->state != ACTUATOR_MOBILE_READY && s->state != ACTUATOR_MOBILE_ACTIVE) ||
        (s->have_sequence && sequence <= s->sequence)) return false;
    for (i = 0u; i < ACTUATOR_MOBILE_AXES; i++) {
        if (velocity_raw[i] < -s->config.max_velocity_raw[i] ||
            velocity_raw[i] > s->config.max_velocity_raw[i]) return false;
    }
    if ((velocity_raw[3] > 0 && s->feedback.lift_position_um >= s->config.lift_max_um) ||
        (velocity_raw[3] < 0 && s->feedback.lift_position_um <= s->config.lift_min_um))
        return false;
    memcpy(s->target_raw, velocity_raw, sizeof(s->target_raw));
    s->sequence = sequence;
    s->have_sequence = true;
    s->command_ms = now_ms;
    s->command_valid_for_ms = s->config.command_timeout_ms;
    s->state = ACTUATOR_MOBILE_ACTIVE;
    return true;
}
bool actuator_mobile_command_until(actuator_mobile_supervisor_t *s, uint32_t session,
    uint32_t sequence, const int32_t velocity_raw[ACTUATOR_MOBILE_AXES],
    uint32_t valid_until_ms, uint32_t now_ms) {
    uint32_t remaining = valid_until_ms - now_ms;
    if (s == NULL) return false;
    actuator_mobile_poll(s, now_ms);
    if (!s->configured || remaining == 0u || remaining > s->config.command_timeout_ms)
        return false;
    if (!actuator_mobile_command(s, session, sequence, velocity_raw, now_ms)) return false;
    s->command_valid_for_ms = remaining;
    return true;
}
void actuator_mobile_stop(actuator_mobile_supervisor_t *s) {
    if (s != NULL && s->configured) latch(s, ACTUATOR_MOBILE_REASON_REQUEST);
}
