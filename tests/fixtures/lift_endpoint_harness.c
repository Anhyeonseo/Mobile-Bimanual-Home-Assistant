#include "actuator_core/lift_endpoint.h"
static actuator_lift_t l;
static actuator_lift_endpoint_t e;
static actuator_lift_hold_monitor_t monitor;
void lift_test_init(void){
 actuator_lift_config_t c={409600,500000,100,100,20,2,10000,500,1000,100,1000,20,200,1,0};
 actuator_lift_init(&l,&c);actuator_lift_endpoint_init(&e,&l,42,200);
 monitor=(actuator_lift_hold_monitor_t){0};
 e.transport_ready=true;
}
int lift_test_hold(unsigned now,unsigned stamp,int pos,int speed,int drive,int healthy){
 actuator_lift_feedback_t f={stamp,(uint16_t)pos,speed,0,healthy!=0,false,false,false,0};
 return actuator_lift_hold_observe(&monitor,&f,drive!=0,now,250,100,2);
}
int lift_test_sample(unsigned now,int pos,int velocity,int current,int hold,int arms){
 actuator_lift_feedback_t f={now,(uint16_t)pos,velocity,current,true,hold!=0,false,false,0};
 e.have_interlock=true;e.interlock=(actuator_lift_interlock_t){now,true,arms!=0,true,true};
 return actuator_lift_observe(&l,&f,now);
}
void lift_test_zero(unsigned now){actuator_lift_zero_sent(&l,now);}
int lift_test_exchange(const unsigned char *in,unsigned size,unsigned now,unsigned char *out){
 return actuator_lift_endpoint_exchange(&e,in,size,now,out);
}

static uint16_t hold_tolerance(void){
 uint64_t counts=(uint64_t)l.config.tolerance_um*4096u/l.config.um_per_turn;
 return (uint16_t)(counts<2?2:counts>2047?2047:counts);
}
int lift_test_report(unsigned now,unsigned stamp,int pos,int speed,int drive,int healthy,int moving,int maintaining){
 actuator_lift_feedback_t f={stamp,(uint16_t)pos,speed,0,healthy!=0,false,0,false,0};
 int held=actuator_lift_hold_observe_report(&monitor,&f,drive!=0,(uint8_t)moving,maintaining!=0,now,250,100,2,hold_tolerance());
 return held|(f.hold_report_remaining_ms?2:0);
}
int lift_test_transient_sample(unsigned now,int pos,int speed,int remaining){
 actuator_lift_feedback_t f={now,(uint16_t)pos,speed,0,true,true,(uint8_t)remaining,false,0};
 return actuator_lift_observe(&l,&f,now);
}
int lift_test_poll_state(unsigned now){actuator_lift_poll(&l,now);return l.state;}

int lift_test_move(unsigned now,int height){return actuator_lift_move(&l,height,now);}

int lift_test_reported_sample(unsigned now,int pos,int speed,int moving){
 actuator_lift_feedback_t f={now,(uint16_t)pos,speed,0,true,false,0,false,0};
 f.hold_verified=actuator_lift_hold_observe_report(&monitor,&f,true,(uint8_t)moving,
  l.state==LIFT_HOLDING,now,250,100,2,hold_tolerance());
 return actuator_lift_observe(&l,&f,now);
}

int lift_test_base_height(unsigned now){
 return actuator_lift_base_height(&l,now);
}

void lift_test_mini1_scale(void){l.config.um_per_turn=84000;}

void lift_test_transport_scale(void){l.config.um_per_turn=84000;l.config.tolerance_um=2000;l.config.feedback_timeout_ms=250;}
int lift_test_transport_sample(unsigned now,unsigned stamp,unsigned drive_stamp,int pos,int speed,int current,int drive,int healthy){
 actuator_lift_feedback_t f={.observed_ms=stamp,.position_raw=(uint16_t)pos,.velocity_raw=speed,.current_ma=current,.healthy=healthy!=0};
 f.bottom_drive_observed_ms=drive_stamp;
 f.bottom_transport_verified=actuator_lift_bottom_transport_observe(&l,&f,drive!=0,drive_stamp,now);
 f.hold_verified=f.bottom_transport_verified;
 return actuator_lift_observe(&l,&f,now);
}
int lift_test_transport_ready(unsigned now){return actuator_lift_bottom_transport_ready(&l,now);}
void lift_test_transport_target(int height){l.target_um=height;}
