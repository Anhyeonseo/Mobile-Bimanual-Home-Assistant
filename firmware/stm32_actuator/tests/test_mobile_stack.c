#include "actuator_core/lift_controller.h"
#include "actuator_core/system_stop.h"
#include "actuator_core/mobile_endpoint.h"
#include "actuator_core/bus_router.h"
#include "actuator_core/crc32c.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);return 1;}} while(0)
static void w32(uint8_t *p,uint32_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);}
static int lift_test(void){
 actuator_lift_t l;
 actuator_lift_config_t c={409600,500000,100,100,20,2,10000,500,1000,100,1000,20,200,1};
 actuator_lift_feedback_t f={0,4090,0,0,true,false};
 CHECK(actuator_lift_init(&l,&c));CHECK(!actuator_lift_home(&l,0));
 CHECK(actuator_lift_observe(&l,&f,0));CHECK(!actuator_lift_move(&l,10000,0));
 CHECK(actuator_lift_home(&l,0));CHECK(l.command_raw==-20);
 f.observed_ms=10;f.position_raw=4080;f.velocity_raw=-20;
 CHECK(actuator_lift_observe(&l,&f,10));
 f.observed_ms=20;f.velocity_raw=0;f.current_ma=600;
 CHECK(actuator_lift_observe(&l,&f,20));CHECK(!l.homed);
 actuator_lift_poll(&l,39);CHECK(!l.homed); /* repeated polls cannot manufacture contact samples */
 f.observed_ms=40;CHECK(actuator_lift_observe(&l,&f,40));CHECK(l.homed && l.state==LIFT_HOLD_PENDING);
 actuator_lift_zero_sent(&l,49);
 f.observed_ms=50;f.current_ma=0;f.hold_verified=true;
 CHECK(actuator_lift_observe(&l,&f,50));CHECK(l.state==LIFT_HOLDING);
 CHECK(actuator_lift_move(&l,10000,50));CHECK(l.command_raw==100);
 f.observed_ms=60;f.position_raw=4085;f.velocity_raw=20;f.hold_verified=false;
 CHECK(actuator_lift_observe(&l,&f,60));CHECK(l.height_um==500 && l.command_raw<100);
 f.observed_ms=70;f.position_raw=10;CHECK(actuator_lift_observe(&l,&f,70));CHECK(l.height_um==2600);
 f.observed_ms=80;f.position_raw=40;CHECK(actuator_lift_observe(&l,&f,80));
 f.observed_ms=90;f.position_raw=84;CHECK(actuator_lift_observe(&l,&f,90));CHECK(l.state==LIFT_HOLD_PENDING);
 actuator_lift_zero_sent(&l,99);
 f.observed_ms=100;f.velocity_raw=0;f.hold_verified=true;
 CHECK(actuator_lift_observe(&l,&f,100));CHECK(l.state==LIFT_HOLDING);
 f.observed_ms=110;f.position_raw=100;CHECK(actuator_lift_observe(&l,&f,110));CHECK(l.state==LIFT_FAULT);
 actuator_lift_reset(&l);CHECK(!l.homed && !l.have_sample && !actuator_lift_move(&l,0,110));
 f.observed_ms=120;f.position_raw=100;CHECK(actuator_lift_observe(&l,&f,120));
 CHECK(actuator_lift_home(&l,120));actuator_lift_cancel(&l);CHECK(l.state==LIFT_UNHOMED && !l.command_raw);
 CHECK(actuator_lift_home(&l,120));actuator_lift_poll(&l,220);CHECK(l.fault==LIFT_BAD_FEEDBACK);
 actuator_lift_reset(&l);f.observed_ms=230;CHECK(actuator_lift_observe(&l,&f,230));
 f.observed_ms=240;f.position_raw=1000;CHECK(!actuator_lift_observe(&l,&f,240));CHECK(l.fault==LIFT_ENCODER_JUMP);
 return 0;
}
static int endpoint_test(void){
 actuator_mobile_config_t c={{100,100,100,50},2,100,200,0,100000};
 actuator_mobile_supervisor_t s;actuator_mobile_endpoint_t e;
 actuator_mobile_feedback_t f={0,{0,0,0,0},50000,true,true,true};
 uint8_t q[36]={ 'A','M',1,4 },out[64];unsigned i;bool ready=false;
 CHECK(actuator_mobile_init(&s,&c));CHECK(actuator_mobile_feedback(&s,&f,0));
 CHECK(actuator_mobile_endpoint_init(&e,&s,42));w32(q+4,1);w32(q+8,7);w32(q+32,actuator_crc32c(q,32));
 CHECK(!actuator_mobile_endpoint_feed(&e,0xff,0,out));
 for(i=0;i<36;i++)ready=actuator_mobile_endpoint_feed(&e,q[i],1,out);
 CHECK(ready && out[0]=='A' && out[1]=='S' && out[3]==4 && out[4]==7 && out[12]==42);
 CHECK(s.state==ACTUATOR_MOBILE_DISABLED);
 q[16]=1;w32(q+32,actuator_crc32c(q,32));
 for(i=0;i<36;i++)ready=actuator_mobile_endpoint_feed(&e,q[i],2,out);
 CHECK(ready && out[3]==(4|128));
 q[16]=0;w32(q+32,actuator_crc32c(q,32));
 for(i=0;i<15;i++)CHECK(!actuator_mobile_endpoint_feed(&e,q[i],3,out));
 for(i=0;i<36;i++)ready=actuator_mobile_endpoint_feed(&e,q[i],103,out);
 CHECK(ready && out[3]==4);
 return 0;
}
static int router_test(void){
 actuator_bus_router_t r;const uint8_t *data;uint8_t a[]={1,2,3},b[]={4,5};
 size_t length;uint32_t token;actuator_bus_work_t kind;
 actuator_bus_router_init(&r);
 CHECK(actuator_bus_router_submit(&r,ACTUATOR_BUS_WORK_ARM,a,3,0,1000,100));
 CHECK(actuator_bus_router_submit(&r,ACTUATOR_BUS_WORK_WHEELS,b,2,0,1000,100));
 CHECK(actuator_bus_router_next(&r,0,100,&data,&length,&token,&kind));CHECK(kind==ACTUATOR_BUS_WORK_ARM && length==3);
 a[0]=99;CHECK(data[0]==1);CHECK(!actuator_bus_router_next(&r,1,100,&data,&length,&token,&kind));
 CHECK(actuator_bus_router_complete(&r,token,50,true));
 CHECK(actuator_bus_router_submit(&r,ACTUATOR_BUS_WORK_STOP,b,2,50,1000,100));
 CHECK(!r.jobs[ACTUATOR_BUS_WORK_WHEELS].pending);
 CHECK(actuator_bus_router_next(&r,50,0,&data,&length,&token,&kind));CHECK(kind==ACTUATOR_BUS_WORK_STOP);
 CHECK(actuator_bus_router_complete(&r,token,100,true));
 CHECK(!actuator_bus_router_resume(&r,false));CHECK(actuator_bus_router_resume(&r,true));
 CHECK(actuator_bus_router_submit(&r,ACTUATOR_BUS_WORK_LIFT,b,2,100,300,100));
 CHECK(!actuator_bus_router_next(&r,200,100,&data,&length,&token,&kind));CHECK(r.expired_jobs==1 && r.bus.stop_pending);
 return 0;
}
static int flush(actuator_bus_router_t *r,uint32_t us){
 const uint8_t *bytes;size_t length;uint32_t token;actuator_bus_work_t kind;
 CHECK(actuator_bus_router_next(r,us,3000,&bytes,&length,&token,&kind));
 CHECK(kind==ACTUATOR_BUS_WORK_STOP);CHECK(actuator_bus_router_complete(r,token,us+500,true));return 0;
}
static int stop_test(void){
 actuator_bus_router_t left,right;actuator_system_stop_t stop={0};uint16_t pose[6]={2048,2048,2048,2048,2048,2048};
 actuator_stop_proof_t proof={0,true,true,true,true};
 actuator_bus_router_init(&left);actuator_bus_router_init(&right);
 CHECK(actuator_system_stop_begin(&stop,&left,&right,pose,pose,true,true,0,100,20));
 CHECK(!actuator_system_stop_begin(&stop,&left,&right,pose,pose,true,true,1,100,20));
 actuator_system_stop_poll(&stop,&left,&right,&proof,0);CHECK(stop.state==SYSTEM_STOP_PENDING);
 CHECK(flush(&left,0)==0);CHECK(flush(&right,0)==0);
 actuator_system_stop_poll(&stop,&left,&right,&proof,1);CHECK(stop.left_hold_queued);
 CHECK(flush(&left,1000)==0);
 actuator_system_stop_poll(&stop,&left,&right,&proof,2);CHECK(stop.state==SYSTEM_STOP_PENDING);
 proof.observed_ms=3;proof.load_retained=false;
 actuator_system_stop_poll(&stop,&left,&right,&proof,3);CHECK(stop.state==SYSTEM_STOP_PENDING);
 actuator_system_stop_poll(&stop,&left,&right,NULL,100);CHECK(stop.state==SYSTEM_STOP_UNCONFIRMED);
 proof.observed_ms=101;proof.load_retained=true;
 actuator_system_stop_poll(&stop,&left,&right,&proof,101);CHECK(stop.state==SYSTEM_STOP_CONFIRMED);
 CHECK(left.inhibited && right.inhibited);CHECK(actuator_bus_router_resume(&left,true));return 0;
}
int main(void){CHECK(lift_test()==0);CHECK(endpoint_test()==0);CHECK(router_test()==0);CHECK(stop_test()==0);return 0;}
