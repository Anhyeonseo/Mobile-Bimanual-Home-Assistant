#include "actuator_core/bus_router.h"
#include <string.h>
void actuator_bus_router_init(actuator_bus_router_t *r) {if(r!=NULL)memset(r,0,sizeof(*r));}
bool actuator_bus_router_submit(actuator_bus_router_t *r,actuator_bus_work_t kind,
    const uint8_t *bytes,size_t length,uint32_t now,uint32_t deadline,uint32_t budget) {
    actuator_bus_job_t *job;unsigned i;uint32_t remaining=deadline-now;
    if(r==NULL || bytes==NULL || (unsigned)kind>=ACTUATOR_BUS_WORK_COUNT || !length || length>32 ||
       !budget || budget>=UINT32_C(0x80000000) || !remaining || remaining>=UINT32_C(0x80000000) || budget>=remaining)return false;
    actuator_shared_bus_poll(&r->bus,now);
    if(r->bus.faulted || ((r->bus.stop_pending || r->inhibited) && kind!=ACTUATOR_BUS_WORK_STOP))return false;
    if(kind==ACTUATOR_BUS_WORK_STOP) {
        r->inhibited=true;
        for(i=1;i<ACTUATOR_BUS_WORK_COUNT;i++)r->jobs[i].pending=false;
        actuator_shared_bus_request_stop(&r->bus);
    }
    job=&r->jobs[kind];memcpy(job->bytes,bytes,length);job->length=length;
    job->deadline_us=deadline;job->budget_us=budget;job->pending=true;return true;
}
bool actuator_bus_router_next(actuator_bus_router_t *r,uint32_t now,uint32_t available,
    const uint8_t **bytes,size_t *length,uint32_t *token,actuator_bus_work_t *kind) {
    unsigned i,index;actuator_bus_job_t *job;uint32_t remaining;
    if(r==NULL || bytes==NULL || length==NULL || token==NULL || kind==NULL)return false;
    actuator_shared_bus_poll(&r->bus,now);
    if(r->bus.active || r->bus.faulted)return false;
    for(i=0;i<ACTUATOR_BUS_WORK_COUNT;i++) {
        index=i==0?0:1+(r->cursor+i-1)%(ACTUATOR_BUS_WORK_COUNT-1);
        job=&r->jobs[index];if(!job->pending || (r->inhibited && index!=0))continue;
        remaining=job->deadline_us-now;
        if(remaining==0 || remaining>=UINT32_C(0x80000000) || job->budget_us>=remaining) {
            job->pending=false;if(r->expired_jobs!=UINT32_MAX)r->expired_jobs++;
            /* An expired control job must not simply vanish while a motor
             * retains its last velocity. Latch STOP, including arm misses. */
            r->inhibited=true;actuator_shared_bus_request_stop(&r->bus);
            continue;
        }
        if(!actuator_shared_bus_acquire(&r->bus,(actuator_bus_work_t)index,now,job->budget_us,available,token))continue;
        memcpy(r->active_bytes,job->bytes,job->length);r->active_length=job->length;
        job->pending=false;r->cursor=index%(ACTUATOR_BUS_WORK_COUNT-1);
        *bytes=r->active_bytes;*length=r->active_length;*kind=(actuator_bus_work_t)index;return true;
    }
    return false;
}
bool actuator_bus_router_complete(actuator_bus_router_t *r,uint32_t token,uint32_t now,bool ok) {
    return r!=NULL && actuator_shared_bus_complete(&r->bus,token,now,ok);
}

bool actuator_bus_router_resume(actuator_bus_router_t *r,bool confirmed) {
    if(r==NULL || !confirmed || r->bus.active || r->bus.faulted || r->bus.stop_pending || r->jobs[0].pending)return false;
    r->inhibited=false;return true;
}
