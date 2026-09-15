#include "actuator_core/bus_schedule.h"
#include <stddef.h>
#include <string.h>
bool actuator_bus_schedule_init(actuator_bus_schedule_t *s,
    const actuator_bus_schedule_config_t *c,uint32_t epoch) {
    if(s==NULL || c==NULL)return false;
    memset(s,0,sizeof(*s));
    if(!c->arm_period_us || c->arm_period_us>1000000u ||
       !c->arm_reserved_us || !c->guard_us ||
       (uint64_t)c->arm_reserved_us+c->guard_us>=c->arm_period_us ||
       !c->maximum_poll_gap_us || c->maximum_poll_gap_us>c->arm_period_us ||
       c->wheels_period_us<c->arm_period_us || c->wheels_period_us>1000000u ||
       c->lift_period_us<c->arm_period_us || c->lift_period_us>1000000u ||
       c->feedback_period_us<c->arm_period_us || c->feedback_period_us>1000000u)return false;
    s->config=*c;s->last_poll_us=epoch;s->initialized=true;return true;
}
actuator_bus_schedule_window_t actuator_bus_schedule_poll(actuator_bus_schedule_t *s,uint32_t now) {
    actuator_bus_schedule_window_t result={0,0,false,true};
    if(s==NULL || !s->initialized || s->faulted)return result;
    uint32_t elapsed=now-s->last_poll_us;
    if(elapsed>s->config.maximum_poll_gap_us) {s->faulted=true;return result;}
    s->last_poll_us=now;
    /* Accumulate short deltas: phase stays continuous across uint32 wrap. */
    s->phase_us=(s->phase_us+elapsed)%s->config.arm_period_us;
    for(unsigned i=0;i<3;i++)
        s->due_in_us[i]=elapsed>=s->due_in_us[i]?0:s->due_in_us[i]-elapsed;
    result.stop_required=false;
    result.arm_reserved=s->phase_us<s->config.arm_reserved_us;
    if(result.arm_reserved)return result;
    uint32_t until_arm=s->config.arm_period_us-s->phase_us;
    if(until_arm<=s->config.guard_us)return result;
    result.available_us=until_arm-s->config.guard_us;
    const uint32_t periods[3]={s->config.wheels_period_us,s->config.lift_period_us,s->config.feedback_period_us};
    for(unsigned i=0;i<3;i++)if(s->due_in_us[i]==0) {
        result.due|=(uint8_t)(1u<<i);s->due_in_us[i]=periods[i];
    }
    return result;
}
