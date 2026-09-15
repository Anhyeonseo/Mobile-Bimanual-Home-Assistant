#include "actuator_core/lift_endpoint.h"
static actuator_lift_t l;
static actuator_lift_endpoint_t e;
void lift_test_init(void){
 actuator_lift_config_t c={409600,500000,100,100,20,2,10000,500,1000,100,1000,20,200,1};
 actuator_lift_init(&l,&c);actuator_lift_endpoint_init(&e,&l,42,200);
 e.transport_ready=true;
}
int lift_test_sample(unsigned now,int pos,int velocity,int current,int hold,int arms){
 actuator_lift_feedback_t f={now,(uint16_t)pos,velocity,current,true,hold!=0};
 e.have_interlock=true;e.interlock=(actuator_lift_interlock_t){now,true,arms!=0,true,true};
 return actuator_lift_observe(&l,&f,now);
}
void lift_test_zero(unsigned now){actuator_lift_zero_sent(&l,now);}
int lift_test_exchange(const unsigned char *in,unsigned size,unsigned now,unsigned char *out){
 return actuator_lift_endpoint_exchange(&e,in,size,now,out);
}
