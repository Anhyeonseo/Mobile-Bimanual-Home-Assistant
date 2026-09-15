#include "actuator_core/device_startup.h"
#include "actuator_core/lift_endpoint.h"
#include "actuator_core/crc32c.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);return 1;}} while(0)
static void w32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static actuator_device_profile_t profile={{777,777,777,777},0,1000};
static int startup(void){
 actuator_device_startup_t s={0};actuator_device_operation_t o;
 CHECK(!actuator_device_startup_begin(&s,&profile,0,true,false));
 CHECK(actuator_device_startup_begin(&s,&profile,0,false,false));
 CHECK(!actuator_device_startup_begin(&s,&profile,0,false,false));
 for(unsigned i=0;i<4;i++){
  uint8_t id[]={9,3,(uint8_t)(8+i),0},one=1;
  CHECK(actuator_device_startup_next(&s,i,&o));CHECK(o.id==8+i&&o.address==3&&o.length==4&&!o.write);
  CHECK(!actuator_device_startup_reply(&s,o.token+1,id,4,true,i));
  CHECK(!actuator_device_startup_next(&s,i,&o));
  CHECK(actuator_device_startup_reply(&s,s.pending.token,id,4,true,i));
  CHECK(actuator_device_startup_next(&s,i,&o)&&o.address==33&&!o.write);
  CHECK(actuator_device_startup_reply(&s,o.token,&one,1,true,i));
  CHECK(actuator_device_startup_next(&s,i,&o)&&o.address==55&&!o.write);
  CHECK(actuator_device_startup_reply(&s,o.token,&one,1,true,i));
 }
 CHECK(s.state==DEVICE_STARTUP_READY&&!s.mode_changed);
 memset(&s,0,sizeof(s));CHECK(actuator_device_startup_begin(&s,&profile,0,true,true));
 const unsigned addresses[]={3,33,40,55,33,55,33,55};
 for(unsigned i=0;i<8;i++){
  uint8_t id[]={9,3,8,0},value=(i>=6?1:0);
  CHECK(actuator_device_startup_next(&s,i,&o)&&o.address==addresses[i]);
  CHECK(o.write==(i>=3&&i<=5));
  if(o.write)CHECK(o.data[0]==(i==3?0:1));
  CHECK(actuator_device_startup_reply(&s,o.token,i==0?id:&value,o.write?0:(i==0?4:1),true,i));
 }
 CHECK(s.mode_changed&&!s.eprom_unlocked&&s.axis==1);
 actuator_device_startup_poll(&s,1000);CHECK(s.state==DEVICE_STARTUP_FAULT);
 memset(&s,0,sizeof(s));CHECK(actuator_device_startup_begin(&s,&profile,0,true,true));
 for(unsigned i=0;i<3;i++){
  uint8_t id[]={9,3,8,0},zero=0;
  CHECK(actuator_device_startup_next(&s,i,&o));
  CHECK(actuator_device_startup_reply(&s,o.token,i==0?id:&zero,i==0?4:1,true,i));
 }
 CHECK(actuator_device_startup_next(&s,3,&o)&&s.eprom_unlocked);
 CHECK(!actuator_device_startup_reply(&s,o.token,NULL,0,false,4));
 CHECK(!actuator_device_startup_begin(&s,&profile,5,true,true));
 return 0;
}
typedef struct {uint8_t slots[2][16];bool fail_read,fail_write,torn;} Storage;
static bool rd(void *p,unsigned i,uint8_t b[16]){Storage *s=p;if(s->fail_read)return false;memcpy(b,s->slots[i],16);return true;}
static bool wr(void *p,unsigned i,const uint8_t b[16]){Storage *s=p;if(s->fail_write)return false;memcpy(s->slots[i],b,s->torn?8:16);return true;}
static int boot(void){
 Storage s={0};memset(s.slots,255,sizeof(s.slots));uint32_t id=99;
 CHECK(!actuator_boot_advance(rd,wr,&s,false,&id)&&id==99);
 CHECK(actuator_boot_advance(rd,wr,&s,true,&id)&&id==1);
 CHECK(actuator_boot_advance(rd,wr,&s,false,&id)&&id==2);
 s.fail_write=true;CHECK(!actuator_boot_advance(rd,wr,&s,false,&id)&&id==2);s.fail_write=false;
 CHECK(actuator_boot_advance(rd,wr,&s,false,&id)&&id==3);
 s.torn=true;CHECK(!actuator_boot_advance(rd,wr,&s,false,&id)&&id==3);s.torn=false;
 CHECK(!actuator_boot_advance(rd,wr,&s,false,&id)&&id==3);
 CHECK(!actuator_boot_advance(rd,wr,&s,true,&id));
 memset(s.slots,255,sizeof(s.slots));CHECK(actuator_boot_advance(rd,wr,&s,true,&id));
 s.slots[0][4]^=1;CHECK(!actuator_boot_advance(rd,wr,&s,true,&id));
 return 0;
}
static bool exchange(actuator_lift_endpoint_t *e,uint8_t op,uint32_t seq,uint32_t now,uint32_t boot,int32_t target,uint8_t out[64]){
 uint8_t p[36]={'A','L',1,op};w32(p+4,1);w32(p+8,seq);w32(p+12,op==4?0:now+50);w32(p+16,(uint32_t)target);w32(p+20,boot);w32(p+32,actuator_crc32c(p,32));
 return actuator_lift_endpoint_exchange(e,p,36,now,out)&&out[3]==op;
}
static int lift(void){
 actuator_lift_t l;actuator_lift_endpoint_t e;uint8_t out[64];
 actuator_lift_config_t c={409600,500000,100,100,20,2,10000,500,1000,100,1000,20,200,1};
 actuator_lift_feedback_t f={0,1000,0,0,true,false};
 CHECK(actuator_lift_init(&l,&c));CHECK(actuator_lift_endpoint_init(&e,&l,42,200));
 CHECK(actuator_lift_observe(&l,&f,0));
 CHECK(!exchange(&e,1,1,0,42,0,out));
 e.interlock=(actuator_lift_interlock_t){0,true,true,true,true};e.have_interlock=true;e.transport_ready=true;
 CHECK(!exchange(&e,1,2,0,43,0,out));
 CHECK(exchange(&e,1,3,0,42,0,out)&&l.state==LIFT_HOMING);
 CHECK(!exchange(&e,5,3,1,42,0,out));
 CHECK(exchange(&e,5,4,1,42,0,out));
 CHECK(!exchange(&e,2,5,2,42,1000,out));
 actuator_lift_endpoint_poll(&e,51);CHECK(!e.active&&l.state==LIFT_FAULT&&!l.homed&&!l.command_raw);
 CHECK(!exchange(&e,6,6,100,42,0,out));
 f.observed_ms=101;CHECK(actuator_lift_observe(&l,&f,101));e.interlock.observed_ms=101;
 CHECK(exchange(&e,6,7,101,42,0,out));CHECK(l.state==LIFT_UNHOMED&&!l.have_sample);
 f.observed_ms=102;CHECK(actuator_lift_observe(&l,&f,102));
 CHECK(exchange(&e,1,8,102,42,0,out));
 f.observed_ms=110;f.current_ma=600;CHECK(actuator_lift_observe(&l,&f,110));
 f.observed_ms=130;CHECK(actuator_lift_observe(&l,&f,130));CHECK(l.state==LIFT_HOLD_PENDING);
 f.observed_ms=131;f.current_ma=0;f.hold_verified=true;CHECK(actuator_lift_observe(&l,&f,131));CHECK(l.state==LIFT_HOLD_PENDING);
 actuator_lift_zero_sent(&l,132);actuator_lift_zero_sent(&l,133);CHECK(l.zero_sent_ms==132);
 f.observed_ms=134;CHECK(actuator_lift_observe(&l,&f,134));actuator_lift_endpoint_poll(&e,134);
 CHECK(l.state==LIFT_HOLDING&&!e.active);
 actuator_lift_cancel(&l);actuator_lift_cancel(&l);CHECK(l.state==LIFT_HOLDING&&l.zero_sent);
 CHECK(exchange(&e,4,9,134,42,0,out)&&out[26]&16);
 CHECK(exchange(&e,2,10,134,42,10000,out));e.interlock.arms_safe=false;
 actuator_lift_endpoint_poll(&e,135);CHECK(!e.active&&l.state==LIFT_FAULT);
 return 0;
}
int main(void){CHECK(startup()==0);CHECK(boot()==0);CHECK(lift()==0);return 0;}
