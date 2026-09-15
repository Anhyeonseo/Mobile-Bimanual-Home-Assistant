#include "actuator_core/mobile_output.h"
#include "actuator_core/motor_groups.h"
#include <string.h>
#include <stddef.h>

bool actuator_mobile_output_init(actuator_mobile_output_t *o,
    actuator_mobile_supervisor_t *s,const actuator_mobile_output_config_t *c,uint32_t epoch) {
    if(o==NULL || s==NULL || c==NULL || !s->configured || s->state!=ACTUATOR_MOBILE_DISABLED)return false;
    for(unsigned i=0;i<4;i++)
        if(c->velocity_direction[i]!=1 && c->velocity_direction[i]!=-1)return false;
    actuator_bus_schedule_t schedule;
    if(!actuator_bus_schedule_init(&schedule,&c->schedule,epoch))return false;
    uint32_t window=c->schedule.arm_period_us-c->schedule.arm_reserved_us-c->schedule.guard_us;
    if(!c->wheels_budget_us || !c->lift_budget_us || !c->stop_budget_us ||
       c->wheels_budget_us>window || c->lift_budget_us>window || c->stop_budget_us>=UINT32_C(0x80000000) ||
       c->job_lifetime_us<=c->wheels_budget_us || c->job_lifetime_us<=c->lift_budget_us ||
       c->job_lifetime_us<=c->stop_budget_us || c->job_lifetime_us>1000000u)return false;
    memset(o,0,sizeof(*o));o->supervisor=s;o->config=*c;o->schedule=schedule;o->configured=true;
    actuator_bus_router_init(&o->router);return true;
}
void actuator_mobile_output_stop(actuator_mobile_output_t *o,actuator_mobile_output_fault_t fault) {
    if(o==NULL || !o->configured)return;
    if(o->fault==MOBILE_OUTPUT_OK)o->fault=fault;
    actuator_mobile_stop(o->supervisor);o->stop_requested=true;
    if(o->lift_control!=NULL)actuator_lift_endpoint_stop(o->lift_control);
    o->router.inhibited=true;
    /* Repeated fault polls must not recreate an unsendable STOP after its
     * completion. The current stop episode stays latched until explicit rearm. */
    if(!o->stop_written)actuator_shared_bus_request_stop(&o->router.bus);
    for(unsigned i=1;i<ACTUATOR_BUS_WORK_COUNT;i++)o->router.jobs[i].pending=false;
}
static void queue_stop(actuator_mobile_output_t *o,uint32_t now) {
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];size_t length;
    const uint8_t ids[4]={8,9,10,11};const uint16_t zero[4]={0,0,0,0};
    if(o->stop_written || o->router.jobs[0].pending ||
       (o->router.bus.active && o->router.bus.owner==ACTUATOR_BUS_WORK_STOP))return;
    if(actuator_sts3215_build_sync_write_words(46,ids,zero,4,packet,&length)!=ACTUATOR_STS3215_PACKET_OK ||
       !actuator_bus_router_submit(&o->router,ACTUATOR_BUS_WORK_STOP,packet,length,now,
           now+o->config.job_lifetime_us,o->config.stop_budget_us)) {
        if(o->fault==MOBILE_OUTPUT_OK)o->fault=MOBILE_OUTPUT_QUEUE;
    }
}
void actuator_mobile_output_poll(actuator_mobile_output_t *o,uint32_t us,uint32_t ms,uint32_t arm_epoch) {
    if(o==NULL || !o->configured)return;
    o->feedback_due=false;o->available_us=0;
    actuator_mobile_poll(o->supervisor,ms);
    actuator_shared_bus_poll(&o->router.bus,us);
    /* Check queued deadlines even when another UART owner prevents dequeue.
     * A later periodic refresh must not hide a missed transmission deadline. */
    for(unsigned i=0;i<ACTUATOR_BUS_WORK_COUNT;i++)if(o->router.jobs[i].pending) {
        uint32_t remaining=o->router.jobs[i].deadline_us-us;
        if(!remaining || remaining>=UINT32_C(0x80000000) || remaining<=o->router.jobs[i].budget_us) {
            o->router.jobs[i].pending=false;
            if(o->router.expired_jobs!=UINT32_MAX)o->router.expired_jobs++;
            if(i!=ACTUATOR_BUS_WORK_STOP)actuator_mobile_output_stop(o,MOBILE_OUTPUT_QUEUE);
        }
    }
    uint32_t phase=us-arm_epoch,period=o->config.schedule.arm_period_us;
    if(phase>=period)actuator_mobile_output_stop(o,MOBILE_OUTPUT_SCHEDULE);
    /* Align to the real ISR epoch each call; do not accumulate interrupt jitter
     * against a free-running synthetic phase or consume the arm event. */
    if(phase<period)o->schedule.phase_us=(phase+period-(us-o->schedule.last_poll_us)%period)%period;
    actuator_bus_schedule_window_t window=actuator_bus_schedule_poll(&o->schedule,us);
    if(window.stop_required)actuator_mobile_output_stop(o,MOBILE_OUTPUT_SCHEDULE);
    if(o->router.bus.faulted || (!o->stop_requested && (o->router.inhibited || o->router.bus.stop_pending)))
        actuator_mobile_output_stop(o,MOBILE_OUTPUT_TRANSPORT);
    if(o->supervisor->state==ACTUATOR_MOBILE_STOP_LATCHED && !o->stop_requested)
        actuator_mobile_output_stop(o,MOBILE_OUTPUT_OK);
    if(o->stop_requested) {
        /* A host ARM cannot bypass this output latch. Rearm requires whole-stop proof. */
        actuator_mobile_stop(o->supervisor);
        queue_stop(o,us);return;
    }
    o->available_us=window.available_us;
    o->feedback_due=(window.due&ACTUATOR_SCHEDULE_FEEDBACK)!=0;
    bool lift_active=o->lift_control!=NULL && o->lift_control->active;
    if(o->lift_engaged&&!lift_active){
        actuator_lift_t *l=o->lift_control->lift;
        if(l->state==LIFT_HOLDING && l->zero_sent && l->command_raw==0 &&
           l->have_sample && ms-l->observed_ms<l->config.feedback_timeout_ms) {
            /* Completion already requires a transmitted zero and newer hold
             * evidence. Discard queued refreshes; normal completion is not an
             * emergency stop requiring a new whole-system arm session. */
            o->router.jobs[ACTUATOR_BUS_WORK_WHEELS].pending=false;
            o->router.jobs[ACTUATOR_BUS_WORK_LIFT].pending=false;
            o->lift_engaged=false;
        }else{actuator_mobile_output_stop(o,MOBILE_OUTPUT_OK);queue_stop(o,us);return;}
    }
    if(lift_active) {
        o->lift_engaged=true;
        actuator_lift_endpoint_poll(o->lift_control,ms);
        if(!o->lift_control->active || o->supervisor->state==ACTUATOR_MOBILE_ACTIVE) {
            actuator_mobile_output_stop(o,MOBILE_OUTPUT_TRANSPORT);queue_stop(o,us);return;
        }
    }
    if(!lift_active && o->supervisor->state!=ACTUATOR_MOBILE_READY && o->supervisor->state!=ACTUATOR_MOBILE_ACTIVE)return;
    uint32_t lifetime=o->config.job_lifetime_us;
    if(o->supervisor->state==ACTUATOR_MOBILE_ACTIVE) {
        uint32_t remaining=o->supervisor->command_valid_for_ms-(ms-o->supervisor->command_ms);
        /* Leave a millisecond for the phase between the two local clocks. */
        if(remaining<=1) {actuator_mobile_output_stop(o,MOBILE_OUTPUT_COMMAND_EXPIRING);queue_stop(o,us);return;}
        uint64_t command_life=(uint64_t)(remaining-1)*1000u;
        if(command_life<lifetime)lifetime=(uint32_t)command_life;
    }
    int32_t motor_velocity[4];
    for(unsigned i=0;i<4;i++)motor_velocity[i]=lift_active?0:o->supervisor->target_raw[i]*o->config.velocity_direction[i];
    if(lift_active) {
        motor_velocity[3]=o->lift_control->lift->command_raw*o->config.velocity_direction[3];
        uint32_t remaining=o->lift_control->valid_until_ms-ms;
        if(remaining<=1) {actuator_mobile_output_stop(o,MOBILE_OUTPUT_COMMAND_EXPIRING);queue_stop(o,us);return;}
        if((uint64_t)(remaining-1)*1000u<lifetime)lifetime=(remaining-1)*1000u;
    }
    const actuator_motor_group_id_t groups[2]={ACTUATOR_GROUP_BASE_WHEELS,ACTUATOR_GROUP_LIFT};
    const actuator_bus_work_t kinds[2]={ACTUATOR_BUS_WORK_WHEELS,ACTUATOR_BUS_WORK_LIFT};
    const uint32_t budget[2]={o->config.wheels_budget_us,o->config.lift_budget_us};
    for(unsigned i=0;i<2;i++)if(window.due&(1u<<i)) {
        uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];size_t length;
        if(lifetime<=budget[i] || actuator_motor_group_velocities(groups[i],
               motor_velocity+(i?3:0),i?1:3,packet,&length)!=ACTUATOR_GROUP_OK ||
           !actuator_bus_router_submit(&o->router,kinds[i],packet,length,us,us+lifetime,budget[i])) {
            actuator_mobile_output_stop(o,MOBILE_OUTPUT_QUEUE);queue_stop(o,us);return;
        }
    }
}
void actuator_mobile_output_completed(actuator_mobile_output_t *o,actuator_bus_work_t kind,uint32_t ms) {
    if(o==NULL || !o->configured || o->router.bus.active || o->router.bus.faulted || kind!=o->router.bus.owner)return;
    if(o->lift_control!=NULL) {
        const uint8_t *p=o->router.active_bytes;
        bool zero_lift=kind==ACTUATOR_BUS_WORK_LIFT && o->router.active_length==11 && p[5]==46 && p[7]==11 && !p[8] && !p[9];
        bool zero_all=kind==ACTUATOR_BUS_WORK_STOP && o->router.active_length==20 && p[5]==46 && p[16]==11 && !p[17] && !p[18];
        if(zero_lift||zero_all)actuator_lift_zero_sent(o->lift_control->lift,ms);
    }
    if(o->writes_completed!=UINT32_MAX)o->writes_completed++;
    if(kind==ACTUATOR_BUS_WORK_STOP) {o->stop_written=true;o->stop_written_ms=ms;}
}
bool actuator_mobile_output_rearm(actuator_mobile_output_t *o,uint32_t session,uint32_t epoch,uint32_t ms,bool proof) {
    if(o==NULL || !o->configured || !proof || !o->stop_written || !o->stop_requested ||
       o->router.bus.active || o->router.bus.faulted || o->router.bus.stop_pending ||
       o->router.jobs[0].pending || o->supervisor->state!=ACTUATOR_MOBILE_STOP_LATCHED || !actuator_mobile_measured_stopped(o->supervisor,ms) ||
       o->supervisor->feedback.observed_ms-o->stop_written_ms==0 ||
       o->supervisor->feedback.observed_ms-o->stop_written_ms>=UINT32_C(0x80000000) ||
       session<=o->supervisor->session || !session)return false;
    actuator_bus_schedule_t schedule;
    if(!actuator_bus_schedule_init(&schedule,&o->config.schedule,epoch) ||
       !actuator_bus_router_resume(&o->router,true) || !actuator_mobile_arm(o->supervisor,session,ms))return false;
    o->schedule=schedule;o->stop_requested=false;o->stop_written=false;o->fault=MOBILE_OUTPUT_OK;o->lift_engaged=false;
    return true;
}

bool actuator_mobile_output_bind_lift(actuator_mobile_output_t *o,actuator_lift_endpoint_t *e) {
    if(o==NULL||e==NULL||!o->configured||o->lift_control!=NULL||e->lift==NULL||
        o->supervisor->state!=ACTUATOR_MOBILE_DISABLED||e->active||
        e->lift->config.maximum_speed_raw>o->supervisor->config.max_velocity_raw[3])return false;
    o->lift_control=e;return true;
}
