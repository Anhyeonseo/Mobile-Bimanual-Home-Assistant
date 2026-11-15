#include "actuator_core/mobile_ids.h"
#include "actuator_core/mobile_feedback.h"
#include <string.h>

static bool duration(uint32_t value) { return value && value < UINT32_C(0x80000000); }
static bool fresh(uint32_t then, uint32_t now, uint32_t limit) { return now - then < limit; }
static void invalidate(actuator_mobile_feedback_reader_t *r) {
    r->supervisor->feedback.hardware_ok = false;
    r->supervisor->feedback.velocity_modes_verified = false;
    r->supervisor->feedback.lift_homed = false;
}
static void fail(actuator_mobile_feedback_reader_t *r, actuator_mobile_feedback_fault_t why) {
    if (r == NULL || !r->configured) return;
    if (r->fault == MOBILE_FEEDBACK_OK) {
        if (!r->failure_count) {
            r->first_failure=(actuator_mobile_feedback_failure_t){
                .fault=why,.started_ms=r->started_ms,.previous=r->axes[r->axis],
                .response=r->parser,.axis=r->axis,.fields=r->rejected_fields,
                .reading_mode=r->reading_mode};
        }
        if(r->failure_count!=UINT32_MAX)++r->failure_count;
        r->fault = why;
    }
    r->dirty = 0;
    for (unsigned i = 0; i < 4; ++i) {
        r->axes[i].valid = false; r->axes[i].mode_verified = false; r->axes[i].drive_valid = false;
    }
    invalidate(r);
    /* Preserve the pending transaction until physical abort/quiet recovery. */
    actuator_mobile_poll(r->supervisor, r->started_ms);
}
bool actuator_mobile_feedback_reader_init(actuator_mobile_feedback_reader_t *r,
    actuator_mobile_supervisor_t *s, const actuator_mobile_feedback_config_t *c) {
    if (r == NULL || s == NULL || c == NULL || !s->configured ||
        s->state != ACTUATOR_MOBILE_DISABLED || !duration(c->sample_timeout_ms) ||
        !duration(c->mode_timeout_ms) || !duration(c->response_timeout_ms) ||
        c->sample_timeout_ms > s->config.feedback_timeout_ms ||
        c->response_timeout_ms >= c->sample_timeout_ms ||
        c->response_timeout_ms >= c->mode_timeout_ms ||
        c->maximum_sample_skew_ms >= c->sample_timeout_ms) return false;
    for (unsigned i = 0; i < 4; ++i) {
        const actuator_mobile_axis_feedback_config_t *a = &c->axes[i];
        if ((a->velocity_direction != 1 && a->velocity_direction != -1) ||
            !a->minimum_voltage_raw || a->minimum_voltage_raw >= a->maximum_voltage_raw ||
            !a->maximum_temperature || !a->maximum_current_raw || a->maximum_current_raw > 32767)
            return false;
    }
    memset(r, 0, sizeof(*r)); r->config = *c; r->supervisor = s; r->configured = true;
    s->have_feedback = false; invalidate(r); return true;
}
void actuator_mobile_feedback_reader_poll(actuator_mobile_feedback_reader_t *r, uint32_t now) {
    if (r == NULL || !r->configured) return;
    /* A delayed scheduler must not turn a single reread into an open-ended
     * temperature exemption. The original sample lease is never renewed. */
    if (r->temperature_pending &&
        now-r->temperature_last_recheck_ms[r->axis]>=50u)
        fail(r, MOBILE_FEEDBACK_TIMEOUT);
    if (r->active && !fresh(r->started_ms, now, r->config.response_timeout_ms))
        fail(r, MOBILE_FEEDBACK_TIMEOUT);
    bool complete = r->fault == MOBILE_FEEDBACK_OK;
    for (unsigned i = 0; i < (r->supervisor->transport_only ? 3u : 4u); ++i) {
        actuator_mobile_axis_sample_t *a = &r->axes[i];
        if (a->valid && !fresh(a->observed_ms, now, r->config.sample_timeout_ms)) {
            a->valid = false; r->dirty &= (uint8_t)~(1u << i);
        }
        if (a->mode_verified && !fresh(a->mode_ms, now, r->config.mode_timeout_ms)) {
            a->mode_verified = false; a->drive_valid = false;
        }
        complete = complete && a->valid && a->mode_verified;
    }
    if (!complete || (r->supervisor->have_feedback &&
        !fresh(r->supervisor->feedback.observed_ms, now, r->config.sample_timeout_ms))) invalidate(r);
    actuator_mobile_poll(r->supervisor, now);
}
bool actuator_mobile_feedback_begin(actuator_mobile_feedback_reader_t *r,
    uint32_t now, bool quiet, uint8_t request[8], uint32_t *token) {
    if (r == NULL || !r->configured) return false;
    actuator_mobile_feedback_reader_poll(r, now);
    if (request == NULL || token == NULL || !quiet || r->active ||
        r->fault != MOBILE_FEEDBACK_OK || r->token == UINT32_MAX) return false;
    unsigned axis = r->cursor % (r->supervisor->transport_only ? 3u : 4u);
    /* Proactively refresh before expiry. A mode read replaces one telemetry
     * opportunity; do not keep returning to the same motor and starve peers. */
    uint32_t refresh=r->config.mode_timeout_ms/2u;
    if(r->read_drive_state && refresh>r->config.sample_timeout_ms/2u)refresh=r->config.sample_timeout_ms/2u;
    bool mode = !r->temperature_pending && (!r->axes[axis].mode_verified || now - r->axes[axis].mode_ms >= refresh);
    uint8_t mode_length=r->read_drive_state?16u:1u;
    if (actuator_sts3215_build_read(actuator_mobile_axis_id(axis), mode ? 33u : 56u,
            mode ? mode_length : 15u, request) != ACTUATOR_STS3215_PACKET_OK) return false;
    r->rejected_fields=0;
    r->axis = (uint8_t)axis; r->reading_mode = mode; r->started_ms = now;
    r->active = true; r->reply_ready = false; r->received_bytes = 0;
    actuator_sts_response_init(&r->parser, actuator_mobile_axis_id(axis), mode ? mode_length : 15u);
    *token = ++r->token; return true;
}
bool actuator_mobile_feedback_feed(actuator_mobile_feedback_reader_t *r,
    uint32_t token, const uint8_t *bytes, size_t length, uint32_t now) {
    if (r == NULL || !r->configured || !r->active || token != r->token || !token ||
        (length && bytes == NULL)) return false;
    actuator_mobile_feedback_reader_poll(r, now);
    if (r->fault != MOBILE_FEEDBACK_OK) return false;
    if (length > (size_t)(64u - r->received_bytes)) { fail(r, MOBILE_FEEDBACK_OVERFLOW); return false; }
    for (size_t i = 0; i < length; ++i) {
        if (r->reply_ready) { fail(r, MOBILE_FEEDBACK_AMBIGUOUS); return false; }
        ++r->received_bytes;
        actuator_sts_response_result_t parsed = actuator_sts_response_push(&r->parser, bytes[i]);
        if (parsed == ACTUATOR_STS_RESPONSE_STATUS_ERROR) { fail(r, MOBILE_FEEDBACK_STATUS); return false; }
        r->reply_ready = parsed == ACTUATOR_STS_RESPONSE_FRAME_READY;
    }
    if (r->received_bytes == 64u && !r->reply_ready) { fail(r, MOBILE_FEEDBACK_OVERFLOW); return false; }
    return true;
}
static int32_t signed_word(const uint8_t *p, unsigned sign_bit) {
    uint16_t word = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
    int32_t magnitude = word & ((1u << sign_bit) - 1u);
    return word & (1u << sign_bit) ? -magnitude : magnitude;
}
bool actuator_mobile_feedback_commit(actuator_mobile_feedback_reader_t *r,
    uint32_t token, uint32_t now, bool tx_done, bool quiet) {
    if (r == NULL || !r->configured || !r->active || !token || token != r->token) return false;
    actuator_mobile_feedback_reader_poll(r, now);
    if (r->fault != MOBILE_FEEDBACK_OK || !r->reply_ready || !tx_done || !quiet) return false;
    const uint8_t *data = actuator_sts_response_data(&r->parser);
    actuator_mobile_axis_sample_t sample = r->axes[r->axis];
    if (r->reading_mode) {
        if (data[0] != 1u) { r->rejected_fields=MOBILE_FEEDBACK_FIELD_MODE; fail(r, MOBILE_FEEDBACK_MODE); return false; }
        sample.mode_ms = r->started_ms; sample.mode_verified = true;
        if(r->read_drive_state){
            if(data[7]!=1u){r->rejected_fields=MOBILE_FEEDBACK_FIELD_TORQUE;fail(r,MOBILE_FEEDBACK_DEVICE);return false;}
            sample.torque_enabled=data[7];sample.goal_velocity_raw=signed_word(data+13,15);
            sample.drive_valid=true;
        }
    } else {
        const actuator_mobile_axis_feedback_config_t *c = &r->config.axes[r->axis];
        sample.position_raw = signed_word(data, 15);
        sample.velocity_raw = signed_word(data + 2, 15) * c->velocity_direction;
        sample.load_raw = signed_word(data + 4, 10);
        sample.current_raw = signed_word(data + 13, 15);
        sample.voltage_raw = data[6]; sample.temperature = data[7]; sample.moving = data[10];
        if(sample.voltage_raw<c->minimum_voltage_raw || sample.voltage_raw>c->maximum_voltage_raw)
            r->rejected_fields|=MOBILE_FEEDBACK_FIELD_VOLTAGE;
        if(sample.temperature>c->maximum_temperature)r->rejected_fields|=MOBILE_FEEDBACK_FIELD_TEMPERATURE;
        if(sample.moving>1u)r->rejected_fields|=MOBILE_FEEDBACK_FIELD_MOVING;
        if(sample.current_raw>c->maximum_current_raw || sample.current_raw<-(int32_t)c->maximum_current_raw)
            r->rejected_fields|=MOBILE_FEEDBACK_FIELD_CURRENT;
        if(data[5]&0xf8u)r->rejected_fields|=MOBILE_FEEDBACK_FIELD_LOAD;
        if(r->rejected_fields){
            const actuator_mobile_axis_sample_t *previous=&r->axes[r->axis];
            /* One complete checksummed telemetry packet may have a plausible
             * but isolated temperature byte. Keep the previous whole packet,
             * including its timestamp, and prioritize exactly one reread.
             * Judge isolation by the fresh cold baseline and independent
             * reread, not the magnitude of the corrupt byte (89/92 observed).
             * Initial/near-limit/multiple-field faults still fail immediately;
             * a second hot sample or absent reply fails, regardless of value.
             * Independent recovered spikes may recur within one second. A
             * wall-clock cooldown must not skip their independent reread. */
            if(r->confirm_temperature_spike && r->axis<3 && !r->temperature_pending &&
               r->rejected_fields==MOBILE_FEEDBACK_FIELD_TEMPERATURE && previous->valid &&
               fresh(previous->observed_ms,now,r->config.sample_timeout_ms/2u) &&
               (unsigned)previous->temperature+10u<=c->maximum_temperature &&
               sample.temperature>=(unsigned)previous->temperature+20u){
                r->temperature_pending=true;r->temperature_recheck_seen_mask|=(uint8_t)(1u<<r->axis);
                r->temperature_last_recheck_ms[r->axis]=now;
                if(r->temperature_rechecks!=UINT32_MAX)++r->temperature_rechecks;
                r->active=false;r->reply_ready=false;r->cursor=r->axis;
                return true;
            }
            fail(r,MOBILE_FEEDBACK_DEVICE);return false;
        }
        if(r->temperature_pending){
            r->temperature_pending=false;
            if(r->temperature_recovered!=UINT32_MAX)++r->temperature_recovered;
        }
        if (sample.valid && (r->started_ms - sample.observed_ms == 0u ||
            r->started_ms - sample.observed_ms >= UINT32_C(0x80000000))) return false;
        sample.observed_ms = r->started_ms; sample.valid = true;
        r->dirty |= (uint8_t)(1u << r->axis);
    }
    r->axes[r->axis] = sample; r->active = false; r->reply_ready = false;
    r->cursor = (uint8_t)((r->axis + 1u) % (r->supervisor->transport_only ? 3u : 4u)); return true;
}
bool actuator_mobile_feedback_publish(actuator_mobile_feedback_reader_t *r,
    const actuator_mobile_lift_evidence_t *lift, uint32_t now) {
    if (r == NULL || !r->configured) return false;
    actuator_mobile_feedback_reader_poll(r, now);
    if (!r->supervisor->transport_only && (lift == NULL || !lift->homed || !lift->hardware_ok ||
        !fresh(lift->observed_ms, now, r->config.sample_timeout_ms))) {
        invalidate(r); actuator_mobile_poll(r->supervisor, now);
    }
    uint8_t required = r->supervisor->transport_only ? 7u : 15u;
    if (r->fault != MOBILE_FEEDBACK_OK || (r->dirty & required) != required) return false;
    uint32_t oldest_age = 0, youngest_age = UINT32_MAX;
    actuator_mobile_feedback_t f = {0};
    f.velocity_modes_verified = true; f.hardware_ok = true;
    for (unsigned i = 0; i < (r->supervisor->transport_only ? 3u : 4u); ++i) {
        actuator_mobile_axis_sample_t *a = &r->axes[i];
        if (!a->valid || !a->mode_verified) return false;
        if(r->read_drive_state && (!a->drive_valid || a->torque_enabled!=1u ||
            !fresh(a->mode_ms,now,r->config.sample_timeout_ms))) {
            invalidate(r);return false;
        }
        uint32_t age = now - a->observed_ms;
        if (age > oldest_age) oldest_age = age;
        if (age < youngest_age) youngest_age = age;
        f.velocity_raw[i] = a->velocity_raw;
    }
    if (oldest_age - youngest_age > r->config.maximum_sample_skew_ms) {
        invalidate(r); actuator_mobile_poll(r->supervisor, now); return false;
    }
    if (r->supervisor->transport_only) {
        /* No live lift claim: AS scope bit identifies a three-wheel sweep. */
        f.lift_position_um = r->supervisor->feedback.lift_position_um;
    } else if (lift != NULL && fresh(lift->observed_ms, now, r->config.sample_timeout_ms)) {
        uint32_t age = now - lift->observed_ms;
        if (age > oldest_age) oldest_age = age;
        f.lift_position_um = lift->position_um; f.lift_homed = lift->homed;
        f.hardware_ok = lift->hardware_ok;
    } else f.hardware_ok = false;
    f.observed_ms = now - oldest_age;
    if (!actuator_mobile_feedback(r->supervisor, &f, now)) return false;
    r->dirty = 0; return true;
}
void actuator_mobile_feedback_fail(actuator_mobile_feedback_reader_t *r) {
    fail(r, MOBILE_FEEDBACK_TRANSPORT);
}
bool actuator_mobile_feedback_reset(actuator_mobile_feedback_reader_t *r, bool quiet, bool stopped) {
    if (r == NULL || !r->configured || !quiet || !stopped ||
        r->supervisor->state == ACTUATOR_MOBILE_READY || r->supervisor->state == ACTUATOR_MOBILE_ACTIVE) return false;
    memset(r->axes, 0, sizeof(r->axes)); r->active = false; r->reply_ready = false;
    r->dirty = 0; r->cursor = 0; r->fault = MOBILE_FEEDBACK_OK;r->temperature_pending=false;
    r->supervisor->have_feedback = false; invalidate(r); return true;
}


bool actuator_mobile_wheels_stop_observe(actuator_mobile_stationary_window_t w[3],
    const actuator_mobile_feedback_reader_t *r,uint32_t zero_ms,uint32_t now) {
    if(!w || !r)return false;
    bool proven=true;
    for(unsigned i=0;i<3;i++){
        const actuator_mobile_axis_sample_t *a=&r->axes[i];
        bool good=r->configured && r->fault==MOBILE_FEEDBACK_OK && a->valid && a->mode_verified &&
            a->drive_valid && a->torque_enabled==1 && a->goal_velocity_raw==0 &&
            now-a->observed_ms<r->config.sample_timeout_ms && now-a->mode_ms<r->config.sample_timeout_ms &&
            a->observed_ms-zero_ms>0 && a->observed_ms-zero_ms<UINT32_C(0x80000000) &&
            a->mode_ms-zero_ms>0 && a->mode_ms-zero_ms<UINT32_C(0x80000000) &&
            a->velocity_raw>=-50 && a->velocity_raw<=50 && a->moving<=1 &&
            a->position_raw>=0 && a->position_raw<4096;
        if(!good){memset(&w[i],0,sizeof(w[i]));proven=false;continue;}
        uint32_t step=a->observed_ms-w[i].last_ms;
        if(w[i].tracking && (step>=UINT32_C(0x80000000) || step>=r->config.sample_timeout_ms))
            memset(&w[i],0,sizeof(w[i]));
        if(!w[i].tracking){
            w[i]=(actuator_mobile_stationary_window_t){.first_ms=a->observed_ms,.last_ms=a->observed_ms,
                .anchor=a->position_raw,.samples=1,.tracking=true};
        }else if(step){
            int32_t delta=a->position_raw-w[i].anchor;
            if(delta>2048)delta-=4096;
            if(delta< -2048)delta+=4096;
            if(delta<w[i].low)w[i].low=delta;
            if(delta>w[i].high)w[i].high=delta;
            if(w[i].high-w[i].low>2){
                w[i]=(actuator_mobile_stationary_window_t){.first_ms=a->observed_ms,.last_ms=a->observed_ms,
                    .anchor=a->position_raw,.samples=1,.tracking=true};
            }else{
                w[i].last_ms=a->observed_ms;
                if(w[i].samples<255)++w[i].samples;
            }
        }
        proven=proven && w[i].samples>=6 && w[i].last_ms-w[i].first_ms>=400;
    }
    return proven;
}
