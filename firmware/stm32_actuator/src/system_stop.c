#include "actuator_core/system_stop.h"
#include "actuator_core/motor_groups.h"
#include <string.h>

static bool zero_packet(uint8_t packet[26],size_t *length) {
    const uint8_t ids[4]={8,9,10,11};const uint16_t zero[4]={0};
    return actuator_sts3215_build_sync_write_words(46,ids,zero,4,packet,length)==ACTUATOR_STS3215_PACKET_OK;
}
static bool receipt(const actuator_bus_router_t *r,uint32_t after,
                    const uint8_t *bytes,size_t length) {
    return !r->bus.active && !r->bus.faulted && r->completed_token>after &&
        r->completed_kind==ACTUATOR_BUS_WORK_STOP && r->completed_length==length &&
        memcmp(r->completed_bytes,bytes,length)==0;
}
bool actuator_system_stop_begin_timed(actuator_system_stop_t *s,
    actuator_bus_router_t *left,actuator_bus_router_t *right,const uint16_t lp[6],
    const uint16_t rp[6],bool valid,bool preserve,uint32_t us,uint32_t ms,
    const actuator_system_stop_config_t *c) {
    uint8_t packet[26];size_t length;
    if(s==NULL || s->state!=SYSTEM_STOP_IDLE || left==NULL || right==NULL || c==NULL ||
       !c->timeout_ms || c->timeout_ms>=UINT32_C(0x80000000) || !c->feedback_max_age_ms ||
       c->feedback_max_age_ms>=UINT32_C(0x80000000) || !c->zero_budget_us || !c->hold_budget_us ||
       c->job_lifetime_us>=UINT32_C(0x80000000) || c->job_lifetime_us<=c->zero_budget_us ||
       c->job_lifetime_us<=c->hold_budget_us)return false;
    memset(s,0,sizeof(*s));s->state=SYSTEM_STOP_PENDING;s->started_ms=ms;s->config=*c;
    s->timeout_ms=c->timeout_ms;s->feedback_max_age_ms=c->feedback_max_age_ms;s->preserve_load=preserve;
    s->left_zero_after_token=left->bus.token;s->right_hold_after_token=right->bus.token;
    left->inhibited=true;actuator_shared_bus_request_stop(&left->bus);
    right->inhibited=true;actuator_shared_bus_request_stop(&right->bus);
    bool ok=zero_packet(packet,&length) && actuator_bus_router_submit(left,
        ACTUATOR_BUS_WORK_STOP,packet,length,us,us+c->job_lifetime_us,c->zero_budget_us);
    if(!valid || lp==NULL || rp==NULL)return false;
    for(unsigned i=0;i<6;i++)if(lp[i]>=4096 || rp[i]>=4096)return false;
    memcpy(s->left_positions,lp,sizeof(s->left_positions));
    memcpy(s->right_positions,rp,sizeof(s->right_positions));s->arm_feedback_valid=true;
    if(actuator_motor_group_positions(ACTUATOR_GROUP_RIGHT_ARM,rp,6,packet,&length)!=ACTUATOR_GROUP_OK)return false;
    return actuator_bus_router_submit(right,ACTUATOR_BUS_WORK_STOP,packet,length,
        us,us+c->job_lifetime_us,c->hold_budget_us) && ok;
}
void actuator_system_stop_poll_timed(actuator_system_stop_t *s,actuator_bus_router_t *left,
    actuator_bus_router_t *right,const actuator_stop_proof_t *p,uint32_t us,uint32_t now) {
    uint8_t packet[26];size_t length;
    if(s==NULL || left==NULL || right==NULL || s->state==SYSTEM_STOP_IDLE)return;
    if(s->arm_feedback_valid && !s->left_hold_queued && !left->jobs[0].pending &&
       zero_packet(packet,&length) && receipt(left,s->left_zero_after_token,packet,length)) {
        if(actuator_motor_group_positions(ACTUATOR_GROUP_LEFT_ARM,s->left_positions,6,packet,&length)==ACTUATOR_GROUP_OK) {
            s->left_hold_after_token=left->bus.token;
            s->left_hold_queued=actuator_bus_router_submit(left,ACTUATOR_BUS_WORK_STOP,
                packet,length,us,us+s->config.job_lifetime_us,s->config.hold_budget_us);
        }
    }
    bool written=s->left_hold_queued && !left->jobs[0].pending && !right->jobs[0].pending &&
        actuator_motor_group_positions(ACTUATOR_GROUP_LEFT_ARM,s->left_positions,6,packet,&length)==ACTUATOR_GROUP_OK &&
        receipt(left,s->left_hold_after_token,packet,length);
    written=written && actuator_motor_group_positions(ACTUATOR_GROUP_RIGHT_ARM,s->right_positions,6,packet,&length)==ACTUATOR_GROUP_OK &&
        receipt(right,s->right_hold_after_token,packet,length);
    if(written && !s->actions_completed){s->actions_completed=true;s->actions_completed_ms=now;}
    if(written && p!=NULL && now-p->observed_ms<s->feedback_max_age_ms &&
       p->observed_ms-s->actions_completed_ms!=0 &&
       p->observed_ms-s->actions_completed_ms<UINT32_C(0x80000000) &&
       p->base_stopped && p->lift_holding && p->arms_holding && (!s->preserve_load || p->load_retained)) {
        s->state=SYSTEM_STOP_CONFIRMED;return;
    }
    /* Confirmation is a live measurement, not a permanent latch. Keep the
     * output inhibit while stale feedback/load loss revokes confirmation. */
    if(s->state==SYSTEM_STOP_CONFIRMED || now-s->started_ms>=s->timeout_ms)
        s->state=SYSTEM_STOP_UNCONFIRMED;
}
bool actuator_system_stop_begin(actuator_system_stop_t *s,actuator_bus_router_t *l,
    actuator_bus_router_t *r,const uint16_t lp[6],const uint16_t rp[6],bool valid,
    bool preserve,uint32_t now,uint32_t timeout,uint32_t age) {
    const actuator_system_stop_config_t c={10000,2000,2000,timeout,age};
    return actuator_system_stop_begin_timed(s,l,r,lp,rp,valid,preserve,now*1000u,now,&c);
}
void actuator_system_stop_poll(actuator_system_stop_t *s,actuator_bus_router_t *l,
    actuator_bus_router_t *r,const actuator_stop_proof_t *p,uint32_t now) {
    actuator_system_stop_poll_timed(s,l,r,p,now*1000u,now);
}
