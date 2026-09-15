#include "actuator_core/lift_controller.h"
#include <stddef.h>
#include <string.h>
static bool fresh(const actuator_lift_t *l,uint32_t now) {
    return l->have_sample && now-l->observed_ms<l->config.feedback_timeout_ms;
}
static void fault(actuator_lift_t *l,actuator_lift_fault_t reason) {
    l->fault=reason;l->state=LIFT_FAULT;l->command_raw=0;l->homed=false;
}
static bool stopped(const actuator_lift_t *l) {
    return l->feedback.velocity_raw<=l->config.stopped_speed_raw &&
        l->feedback.velocity_raw>=-l->config.stopped_speed_raw;
}
bool actuator_lift_init(actuator_lift_t *l,const actuator_lift_config_t *c) {
    if(l==NULL) return false;
    memset(l,0,sizeof(*l));
    if(c==NULL || c->um_per_turn<=0 || c->um_per_turn>10000000 ||
        c->maximum_height_um<=0 || c->maximum_height_um>10000000 ||
        c->tolerance_um<=0 || c->tolerance_um>=c->maximum_height_um ||
        c->maximum_speed_raw<=0 || c->maximum_speed_raw>32767 ||
        c->homing_speed_raw<=0 || c->homing_speed_raw>c->maximum_speed_raw ||
        c->stopped_speed_raw<0 || c->stopped_speed_raw>=c->homing_speed_raw ||
        c->slowdown_distance_um<=c->tolerance_um ||
        c->home_current_ma<=0 || c->maximum_current_ma<=c->home_current_ma ||
        !c->feedback_timeout_ms || c->feedback_timeout_ms>=UINT32_C(0x80000000) ||
        !c->homing_timeout_ms || c->homing_timeout_ms>=UINT32_C(0x80000000) ||
        !c->contact_dwell_ms || c->contact_dwell_ms>=c->homing_timeout_ms ||
        !c->maximum_encoder_step_raw || c->maximum_encoder_step_raw>=2048 ||
        (c->encoder_up_direction!=1 && c->encoder_up_direction!=-1)) return false;
    l->config=*c;l->configured=true;return true;
}
void actuator_lift_reset(actuator_lift_t *l) {
    if(l==NULL || !l->configured) return;
    l->state=LIFT_UNHOMED;l->fault=LIFT_OK;l->command_raw=0;
    l->homed=false;l->have_sample=false;l->contact_tracking=false;
    l->accumulated_raw=0;l->home_raw=0;
}
void actuator_lift_poll(actuator_lift_t *l,uint32_t now) {
    int64_t error,magnitude,speed;
    if(l==NULL || !l->configured || l->state==LIFT_FAULT || l->state==LIFT_UNHOMED) return;
    if(!fresh(l,now) || !l->feedback.healthy) {fault(l,LIFT_BAD_FEEDBACK);return;}
    if(l->feedback.current_ma>l->config.maximum_current_ma) {fault(l,LIFT_OVER_CURRENT);return;}
    if(l->state==LIFT_HOMING) {
        if(now-l->homing_started_ms>=l->config.homing_timeout_ms) {fault(l,LIFT_TIMEOUT);return;}
        l->command_raw=-l->config.homing_speed_raw;
        /* Dwell is evaluated only from fresh advancing samples in observe(). */
    } else if(l->homed && (l->height_um< -l->config.tolerance_um ||
                          l->height_um>l->config.maximum_height_um)) {
        fault(l,LIFT_TRAVEL_LIMIT);
    } else if(l->state==LIFT_MOVING) {
        error=(int64_t)l->target_um-l->height_um;
        magnitude=error<0?-error:error;
        if(magnitude<=l->config.tolerance_um) {
            l->state=LIFT_HOLD_PENDING;l->command_raw=0;l->zero_sent=false;
        } else {
            speed=(int64_t)l->config.maximum_speed_raw*magnitude/l->config.slowdown_distance_um;
            if(speed>l->config.maximum_speed_raw) speed=l->config.maximum_speed_raw;
            if(speed<1) speed=1;
            l->command_raw=(int32_t)(error<0?-speed:speed);
        }
    }
    if(l->state==LIFT_HOLD_PENDING && l->zero_sent && l->observed_ms-l->zero_sent_ms!=0 &&
       l->observed_ms-l->zero_sent_ms<UINT32_C(0x80000000) && stopped(l) && l->feedback.hold_verified) {
        l->state=LIFT_HOLDING;l->command_raw=0;
    }
    if((l->state==LIFT_HOLDING || l->state==LIFT_READY) &&
       (!stopped(l) || !l->feedback.hold_verified ||
        (int64_t)l->height_um-l->target_um>l->config.tolerance_um ||
        (int64_t)l->target_um-l->height_um>l->config.tolerance_um)) fault(l,LIFT_BAD_FEEDBACK);
}
bool actuator_lift_observe(actuator_lift_t *l,const actuator_lift_feedback_t *f,uint32_t now) {
    int32_t delta;
    int64_t height;
    if(l==NULL || f==NULL || !l->configured) return false;
    actuator_lift_poll(l,now);
    if(f->position_raw>=4096 || f->velocity_raw< -32767 || f->velocity_raw>32767 ||
       f->current_ma<0 || now-f->observed_ms>=l->config.feedback_timeout_ms ||
       (l->have_sample && (f->observed_ms-l->observed_ms==0 ||
                          f->observed_ms-l->observed_ms>=UINT32_C(0x80000000)))) return false;
    if(l->have_sample) {
        delta=(int32_t)f->position_raw-l->previous_raw;
        if(delta>2048) delta-=4096;
        if(delta< -2048) delta+=4096;
        if(delta>l->config.maximum_encoder_step_raw || delta< -(int32_t)l->config.maximum_encoder_step_raw) {
            fault(l,LIFT_ENCODER_JUMP);return false;
        }
        /* Keep arithmetic bounded even during prolonged unhomed observations. */
        if(l->accumulated_raw>INT64_C(1000000000) || l->accumulated_raw< -INT64_C(1000000000)) {
            fault(l,LIFT_ENCODER_JUMP);return false;
        }
        l->accumulated_raw+=(int64_t)delta*l->config.encoder_up_direction;
    }
    l->previous_raw=f->position_raw;l->observed_ms=f->observed_ms;l->feedback=*f;l->have_sample=true;
    if(l->homed) {
        height=(l->accumulated_raw-l->home_raw)*l->config.um_per_turn/4096;
        if(height<INT32_MIN || height>INT32_MAX) {fault(l,LIFT_TRAVEL_LIMIT);return false;}
        l->height_um=(int32_t)height;
    }
    if(l->state==LIFT_HOMING) {
        if(f->healthy && stopped(l) && f->current_ma>=l->config.home_current_ma &&
           f->current_ma<=l->config.maximum_current_ma) {
            if(!l->contact_tracking) {l->contact_tracking=true;l->contact_started_ms=f->observed_ms;}
            else if(f->observed_ms-l->contact_started_ms>=l->config.contact_dwell_ms) {
                l->home_raw=l->accumulated_raw;l->height_um=0;l->target_um=0;
                l->homed=true;l->state=LIFT_HOLD_PENDING;l->command_raw=0;l->zero_sent=false;
            }
        } else l->contact_tracking=false;
    }
    actuator_lift_poll(l,now);
    return true;
}
bool actuator_lift_home(actuator_lift_t *l,uint32_t now) {
    if(l==NULL || !l->configured) return false;
    actuator_lift_poll(l,now);
    if(l->state!=LIFT_UNHOMED || !fresh(l,now) || !l->feedback.healthy || !stopped(l) ||
       l->feedback.current_ma>l->config.maximum_current_ma) return false;
    l->homing_started_ms=now;l->contact_tracking=false;l->state=LIFT_HOMING;
    l->command_raw=-l->config.homing_speed_raw;return true;
}
bool actuator_lift_move(actuator_lift_t *l,int32_t height,uint32_t now) {
    if(l==NULL || !l->configured) return false;
    actuator_lift_poll(l,now);
    if(!l->homed || (l->state!=LIFT_READY && l->state!=LIFT_HOLDING) ||
       height<0 || height>l->config.maximum_height_um || !fresh(l,now)) return false;
    l->target_um=height;l->state=LIFT_MOVING;actuator_lift_poll(l,now);return true;
}
void actuator_lift_cancel(actuator_lift_t *l) {
    if(l==NULL || !l->configured || l->state==LIFT_FAULT) return;
    if(l->state==LIFT_HOLD_PENDING || l->state==LIFT_HOLDING) return; /* idempotent stop */
    l->command_raw=0;l->contact_tracking=false;l->target_um=l->height_um;
    l->state=l->homed?LIFT_HOLD_PENDING:LIFT_UNHOMED;l->zero_sent=false;
}

void actuator_lift_inhibit(actuator_lift_t *l,actuator_lift_fault_t reason) {
    if(l!=NULL&&l->configured)fault(l,reason==LIFT_OK?LIFT_BAD_FEEDBACK:reason);
}

void actuator_lift_zero_sent(actuator_lift_t *l,uint32_t now) {
    if(l!=NULL&&l->configured&&l->state==LIFT_HOLD_PENDING&&!l->zero_sent){l->zero_sent=true;l->zero_sent_ms=now;}
}
