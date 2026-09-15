#include "actuator_core/mobile_output.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"%d: %s\n",__LINE__,#x);exit(1);}}while(0)
static const actuator_mobile_config_t limits={{1000,1000,1000,100},2,200,50,0,500000};
static const actuator_mobile_output_config_t config={{5000,1000,200,10000,20000,5000,500},500,400,600,4000,{1,1,1,1}};
static void observe(actuator_mobile_supervisor_t *s,uint32_t ms) {
    actuator_mobile_feedback_t f={.observed_ms=ms,.lift_position_um=100000,
        .velocity_modes_verified=true,.lift_homed=true,.hardware_ok=true};
    CHECK(actuator_mobile_feedback(s,&f,ms));
}
static void exercise(uint32_t start_us,uint32_t start_ms) {
    actuator_mobile_supervisor_t s;actuator_mobile_output_t o;
    CHECK(actuator_mobile_init(&s,&limits));observe(&s,start_ms);
    CHECK(actuator_mobile_output_init(&o,&s,&config,start_us));
    CHECK(actuator_mobile_arm(&s,1,start_ms));
    const int32_t values[4]={100,-200,300,20};uint32_t sequence=0,count[5]={0};
    for(uint32_t delta=0;delta<60000000u;delta+=100) {
        uint32_t us=start_us+delta,ms=start_ms+delta/1000;
        if(delta && delta%1000==0)observe(&s,ms);
        if(delta%10000==0)CHECK(actuator_mobile_command(&s,1,++sequence,values,ms));
        actuator_mobile_output_poll(&o,us,ms,start_us+(delta/5000)*5000);
        CHECK(o.fault==MOBILE_OUTPUT_OK);
        const uint8_t *bytes;size_t length;uint32_t token;actuator_bus_work_t kind;
        if(actuator_bus_router_next(&o.router,us,o.available_us,&bytes,&length,&token,&kind)) {
            CHECK(delta%5000>=1000 && delta%5000+o.router.bus.budget_us<=4800);
            CHECK(bytes[5]==46);
            if(kind==ACTUATOR_BUS_WORK_WHEELS)CHECK(length==17 && bytes[7]==8 && bytes[10]==9 && bytes[13]==10 && bytes[12]==0x80);
            else CHECK(kind==ACTUATOR_BUS_WORK_LIFT && length==11 && bytes[7]==11);
            count[kind]++;
            CHECK(actuator_bus_router_complete(&o.router,token,us+50,true));
            actuator_mobile_output_completed(&o,kind,ms);
        }
    }
    CHECK(count[ACTUATOR_BUS_WORK_WHEELS]==6000 && count[ACTUATOR_BUS_WORK_LIFT]==3000);
    CHECK(o.writes_completed==9000);
}
int main(void) {
    exercise(0,0);exercise(UINT32_MAX-20000u,UINT32_MAX-30000u);
    actuator_mobile_supervisor_t s;actuator_mobile_output_t o;
    CHECK(actuator_mobile_init(&s,&limits));observe(&s,0);
    CHECK(actuator_mobile_output_init(&o,&s,&config,0));CHECK(actuator_mobile_arm(&s,1,0));
    int32_t v[4]={100,200,300,0};CHECK(actuator_mobile_command_until(&s,1,1,v,4,0));
    for(uint32_t us=0;us<=1000;us+=100)actuator_mobile_output_poll(&o,us,us/1000,0);
    CHECK(o.router.jobs[ACTUATOR_BUS_WORK_WHEELS].deadline_us==3000);
    /* A blocked job expires without requiring next()/dequeue, before refresh. */
    for(uint32_t us=1100;us<=2500;us+=100)actuator_mobile_output_poll(&o,us,us/1000,0);
    CHECK(o.stop_requested && o.router.expired_jobs>0 && o.router.jobs[0].pending);
    CHECK(!o.router.jobs[ACTUATOR_BUS_WORK_WHEELS].pending && !o.router.jobs[ACTUATOR_BUS_WORK_LIFT].pending);
    CHECK(!actuator_mobile_output_rearm(&o,2,2500,2,true)); /* no stop TX */
    puts("periodic mobile output: 120 simulated seconds, two clock wraps, signed frames and deadline failure passed");
    return 0;
}
