#include "actuator_core/system_stop.h"
#include "actuator_core/arm_hold_monitor.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"%d: %s\n",__LINE__,#x);return 1;}}while(0)
static const uint16_t pose[6]={4095,1000,2000,3000,100,50};
static const actuator_system_stop_config_t config={4000,600,600,100,30};
static int flush(actuator_bus_router_t *r,uint32_t us,bool ok){
 const uint8_t *p;size_t n;uint32_t t;actuator_bus_work_t k;
 CHECK(actuator_bus_router_next(r,us,0,&p,&n,&t,&k));CHECK(k==ACTUATOR_BUS_WORK_STOP);
 CHECK(actuator_bus_router_complete(r,t,us+300,ok)==ok);return 0;
}
static int stop(unsigned scenario){
 actuator_bus_router_t l,r;actuator_bus_router_init(&l);actuator_bus_router_init(&r);
 actuator_system_stop_t s={0};uint32_t us=UINT32_MAX-1000u,ms=9000;
 CHECK(actuator_system_stop_begin_timed(&s,&l,&r,pose,pose,true,true,us,ms,&config));
 if(scenario==1){ /* Other STOP bytes cannot acknowledge our mobile zero. */
  uint8_t wrong[2]={42,0};CHECK(actuator_bus_router_submit(&l,ACTUATOR_BUS_WORK_STOP,wrong,2,us,us+4000,600));
 }
 if(scenario==2){ /* Aborted/recovered work is not a successful receipt. */
  CHECK(flush(&l,us,false)==0);CHECK(l.completed_token==0);
  actuator_system_stop_poll_timed(&s,&l,&r,NULL,us+1000,ms+1);CHECK(!s.left_hold_queued);return 0;
 }
 if(scenario==3){ /* Expired queue with advanced token is not a receipt. */
  const uint8_t *p;size_t n;uint32_t t;actuator_bus_work_t k;
  CHECK(!actuator_bus_router_next(&l,us+4000,0,&p,&n,&t,&k));
  l.bus.token+=2;actuator_system_stop_poll_timed(&s,&l,&r,NULL,us+4000,ms+4);
  CHECK(!s.left_hold_queued&&l.completed_token==0);return 0;
 }
 CHECK(flush(&l,us,true)==0);CHECK(flush(&r,us,true)==0);
 actuator_system_stop_poll_timed(&s,&l,&r,NULL,us+400,ms+1);
 if(scenario==1){CHECK(!s.left_hold_queued);return 0;}
 CHECK(s.left_hold_queued);CHECK(flush(&l,us+400,true)==0);
 actuator_stop_proof_t p={ms+1,true,true,true,true};
 actuator_system_stop_poll_timed(&s,&l,&r,&p,us+800,ms+2);
 CHECK(s.actions_completed&&s.state==SYSTEM_STOP_PENDING);
 p.observed_ms=ms+3;actuator_system_stop_poll_timed(&s,&l,&r,&p,us+1100,ms+3);CHECK(s.state==SYSTEM_STOP_CONFIRMED);
 actuator_system_stop_poll_timed(&s,&l,&r,&p,us+31000,ms+33);CHECK(s.state==SYSTEM_STOP_UNCONFIRMED);
 p.observed_ms=ms+34;actuator_system_stop_poll_timed(&s,&l,&r,&p,us+32000,ms+34);CHECK(s.state==SYSTEM_STOP_CONFIRMED);
 p.load_retained=false;actuator_system_stop_poll_timed(&s,&l,&r,&p,us+32001,ms+34);CHECK(s.state==SYSTEM_STOP_UNCONFIRMED);
 CHECK(l.inhibited&&r.inhibited);return 0;
}
static int monitor(void){
 actuator_arm_hold_monitor_t m;uint32_t oldest;
 CHECK(actuator_arm_hold_monitor_init(&m,pose,pose,UINT32_MAX-5u,5,10,30));
 for(unsigned j=0;j<6;j++)CHECK(actuator_arm_hold_monitor_observe(&m,j,pose[j],pose[j],1,1));
 CHECK(!actuator_arm_hold_monitor_proof(&m,12,&oldest)); /* poll alone creates no dwell */
 for(unsigned j=0;j<6;j++)CHECK(actuator_arm_hold_monitor_observe(&m,j,j?pose[j]:1,pose[j],12,12));
 CHECK(actuator_arm_hold_monitor_proof(&m,12,&oldest)&&oldest==12);
 CHECK(!actuator_arm_hold_monitor_observe(&m,0,1,4095,12,12)); /* replay revokes */
 CHECK(!actuator_arm_hold_monitor_proof(&m,12,&oldest));
 CHECK(actuator_arm_hold_monitor_observe(&m,0,1,4095,13,13));
 CHECK(!actuator_arm_hold_monitor_proof(&m,13,&oldest));
 CHECK(actuator_arm_hold_monitor_observe(&m,0,1,4095,23,23));
 CHECK(actuator_arm_hold_monitor_proof(&m,23,&oldest)&&oldest==12);
 CHECK(!actuator_arm_hold_monitor_proof(&m,42,&oldest));
 CHECK(!actuator_arm_hold_monitor_observe(&m,1,1000,1000,50,49)); /* future */
 CHECK(!actuator_arm_hold_monitor_observe(&m,2,2050,2000,50,50)); /* motion */
 CHECK(!actuator_arm_hold_monitor_proof(&m,50,&oldest));return 0;
}
int main(void){for(unsigned i=0;i<4;i++)CHECK(stop(i)==0);CHECK(monitor()==0);return 0;}
