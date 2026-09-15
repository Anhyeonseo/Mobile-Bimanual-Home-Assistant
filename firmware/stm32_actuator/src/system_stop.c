#include "actuator_core/system_stop.h"
#include "actuator_core/motor_groups.h"
#include <string.h>
/* A STOP router has one pending slot. Left stop combines four mobile velocity
 * values in one frame; arm hold is queued only AFTER the zero frame completes.
 * begin queues mobile zero and right hold. poll queues left hold after
 * STOP completion; normal left work remains blocked by its stop latch until then. */
static bool mobile_zero(actuator_bus_router_t *left,uint32_t now) {
    uint8_t ids[4]={8,9,10,11},packet[26];uint16_t zero[4]={0};size_t length;
    if(actuator_sts3215_build_sync_write_words(46,ids,zero,4,packet,&length)!=ACTUATOR_STS3215_PACKET_OK)return false;
    return actuator_bus_router_submit(left,ACTUATOR_BUS_WORK_STOP,packet,length,now,now+10000,2000);
}
bool actuator_system_stop_begin(actuator_system_stop_t *s,actuator_bus_router_t *left,actuator_bus_router_t *right,
    const uint16_t lp[6],const uint16_t rp[6],bool valid,bool preserve,uint32_t now,uint32_t timeout,uint32_t age) {
    uint8_t packet[26];size_t length;bool ok;
    if(s==NULL || s->state!=SYSTEM_STOP_IDLE || left==NULL || right==NULL || !timeout || timeout>=UINT32_C(0x80000000) ||
       !age || age>=UINT32_C(0x80000000))return false;
    memset(s,0,sizeof(*s));s->state=SYSTEM_STOP_PENDING;s->started_ms=now;s->timeout_ms=timeout;
    s->feedback_max_age_ms=age;s->preserve_load=preserve;
    s->left_zero_after_token=left->bus.token;s->right_hold_after_token=right->bus.token;
    left->inhibited=true;actuator_shared_bus_request_stop(&left->bus);
    right->inhibited=true;actuator_shared_bus_request_stop(&right->bus);
    ok=mobile_zero(left,now*1000u);
    if(!valid || lp==NULL || rp==NULL)return false;
    for(unsigned i=0;i<6;i++)if(lp[i]>=4096 || rp[i]>=4096)return false;
    memcpy(s->left_positions,lp,sizeof(s->left_positions));s->arm_feedback_valid=true;
    if(actuator_motor_group_positions(ACTUATOR_GROUP_RIGHT_ARM,rp,6,packet,&length)!=ACTUATOR_GROUP_OK)return false;
    return actuator_bus_router_submit(right,ACTUATOR_BUS_WORK_STOP,packet,length,now*1000u,now*1000u+10000,2000) && ok;
}
void actuator_system_stop_poll(actuator_system_stop_t *s,actuator_bus_router_t *left,
    actuator_bus_router_t *right,const actuator_stop_proof_t *p,uint32_t now) {
    uint8_t packet[26];size_t length;bool written;
    if(s==NULL || left==NULL || right==NULL || s->state==SYSTEM_STOP_IDLE || s->state==SYSTEM_STOP_CONFIRMED)return;
    if(s->arm_feedback_valid && !s->left_hold_queued && !left->bus.active && !left->jobs[0].pending &&
       !left->bus.faulted && left->bus.token>s->left_zero_after_token && left->bus.owner==ACTUATOR_BUS_WORK_STOP) {
        if(actuator_motor_group_positions(ACTUATOR_GROUP_LEFT_ARM,s->left_positions,6,packet,&length)==ACTUATOR_GROUP_OK) {
            s->left_hold_after_token=left->bus.token;
            s->left_hold_queued=actuator_bus_router_submit(left,ACTUATOR_BUS_WORK_STOP,packet,length,now*1000u,now*1000u+10000,2000);
        }
    }
    written=s->left_hold_queued && !left->bus.active && !right->bus.active &&
        !left->bus.faulted && !right->bus.faulted && !left->jobs[0].pending && !right->jobs[0].pending &&
        left->bus.token>s->left_hold_after_token && right->bus.token>s->right_hold_after_token;
    if(written && !s->actions_completed){s->actions_completed=true;s->actions_completed_ms=now;}
    if(written && p!=NULL && now-p->observed_ms<s->feedback_max_age_ms &&
       p->observed_ms-s->actions_completed_ms!=0 &&
       p->observed_ms-s->actions_completed_ms<UINT32_C(0x80000000) &&
       p->base_stopped && p->lift_holding && p->arms_holding && (!s->preserve_load || p->load_retained)) {
        s->state=SYSTEM_STOP_CONFIRMED;return;
    }
    if(now-s->started_ms>=s->timeout_ms)s->state=SYSTEM_STOP_UNCONFIRMED;
}
