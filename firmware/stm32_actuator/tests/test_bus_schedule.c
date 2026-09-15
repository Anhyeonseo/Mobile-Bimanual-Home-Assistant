#include "actuator_core/bus_schedule.h"
#include "actuator_core/bus_router.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"check failed %s:%d: %s\n",__FILE__,__LINE__,#x);exit(1);}}while(0)
static const actuator_bus_schedule_config_t config={5000,1000,200,10000,20000,5000,500};
static void run(uint32_t epoch) {
    actuator_bus_schedule_t schedule;
    actuator_bus_router_t router;
    unsigned wheel=0,lift=0,feedback=0;
    CHECK(actuator_bus_schedule_init(&schedule,&config,epoch));
    actuator_bus_router_init(&router);
    for(uint32_t delta=0;delta<60000000;delta+=100) {
        uint32_t now=epoch+delta;
        actuator_bus_schedule_window_t window=actuator_bus_schedule_poll(&schedule,now);
        CHECK(!window.stop_required);
        uint32_t phase=delta%5000;
        CHECK(window.arm_reserved==(phase<1000));
        if(phase<1000 || phase>=4800) CHECK(window.available_us==0 && window.due==0);
        else CHECK(window.available_us==4800-phase);
        const uint8_t bytes[]={255,255,8};
        const actuator_bus_work_t kinds[]={ACTUATOR_BUS_WORK_WHEELS,ACTUATOR_BUS_WORK_LIFT,ACTUATOR_BUS_WORK_FEEDBACK};
        for(unsigned i=0;i<3;i++)if(window.due&(1u<<i))
            CHECK(actuator_bus_router_submit(&router,kinds[i],bytes,3,now,now+5000,200));
        const uint8_t *packet;size_t length;uint32_t token;actuator_bus_work_t kind;
        if(actuator_bus_router_next(&router,now,window.available_us,&packet,&length,&token,&kind)) {
            CHECK(phase>=1000 && phase+200<=4800 && length==3 && packet[2]==8);
            CHECK(actuator_bus_router_complete(&router,token,now+50,true));
            if(kind==ACTUATOR_BUS_WORK_WHEELS)wheel++;
            if(kind==ACTUATOR_BUS_WORK_LIFT)lift++;
            if(kind==ACTUATOR_BUS_WORK_FEEDBACK)feedback++;
        }
    }
    CHECK(wheel==6000 && lift==3000 && feedback==12000);
}
int main(void) {
    run(0);run(UINT32_MAX-20000u);
    actuator_bus_schedule_t s;
    CHECK(actuator_bus_schedule_init(&s,&config,100));
    CHECK(actuator_bus_schedule_poll(&s,601).stop_required);
    CHECK(actuator_bus_schedule_poll(&s,602).stop_required); /* no auto resume */
    CHECK(actuator_bus_schedule_init(&s,&config,100));
    CHECK(actuator_bus_schedule_poll(&s,99).stop_required); /* reverse clock */
    actuator_bus_schedule_config_t bad=config;bad.guard_us=4000;
    CHECK(!actuator_bus_schedule_init(&s,&bad,0));
    CHECK(actuator_bus_schedule_poll(&s,0).stop_required);
    CHECK(actuator_bus_schedule_init(&s,&config,0));
    actuator_bus_router_t r;actuator_bus_router_init(&r);
    uint8_t zero[]={255,255,8};const uint8_t *packet;size_t length;uint32_t token;actuator_bus_work_t kind;
    CHECK(actuator_bus_router_submit(&r,ACTUATOR_BUS_WORK_STOP,zero,3,0,1000,200));
    CHECK(actuator_bus_router_next(&r,0,0,&packet,&length,&token,&kind));
    CHECK(kind==ACTUATOR_BUS_WORK_STOP); /* stop bypasses arm reservation */
    puts("schedule/router: 120 simulated seconds, rollover, deadlines and stop passed");
    return 0;
}
