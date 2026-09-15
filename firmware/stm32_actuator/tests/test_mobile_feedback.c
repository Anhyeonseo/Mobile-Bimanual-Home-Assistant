#include "actuator_core/mobile_feedback.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

static actuator_mobile_supervisor_t s;
static actuator_mobile_feedback_reader_t r;
static actuator_mobile_lift_evidence_t lift;
static uint32_t now;
static size_t reply(uint8_t *out, uint8_t id, uint8_t status, const uint8_t *data, size_t n) {
    out[0]=255; out[1]=255; out[2]=id; out[3]=(uint8_t)(n+2); out[4]=status;
    if(n)memcpy(out+5,data,n);
    out[n+5]=actuator_sts3215_checksum(out+2,n+3); return n+6;
}
static void setup(uint32_t start) {
    actuator_mobile_config_t c={{100,100,100,100},2,100,200,0,300000};
    actuator_mobile_feedback_config_t f={.sample_timeout_ms=100,.mode_timeout_ms=400,
        .response_timeout_ms=5,.maximum_sample_skew_ms=30};
    for(unsigned i=0;i<4;i++)f.axes[i]=(actuator_mobile_axis_feedback_config_t){i==1?-1:1,90,140,70,1000};
    assert(actuator_mobile_init(&s,&c)); assert(actuator_mobile_feedback_reader_init(&r,&s,&f));
    now=start; lift=(actuator_mobile_lift_evidence_t){now,12000,true,true};
}
static void telemetry(uint8_t *data) {
    memset(data,0,15); data[0]=0xff;data[1]=0x0f;data[6]=120;data[7]=30;
    data[13]=25;
}
static void read_one(void) {
    uint8_t req[8], bytes[21],data[15]; uint32_t token;
    now+=4;assert(actuator_mobile_feedback_begin(&r,now,true,req,&token));
    assert(req[2]==8+r.axis && req[4]==2 && req[5]==(r.reading_mode?33:56));
    assert(req[7]==actuator_sts3215_checksum(req+2,5));
    telemetry(data); if(r.reading_mode)data[0]=1;
    size_t n=reply(bytes,req[2],0,data,r.reading_mode?1:15);
    /* READ echo and arbitrarily split response go through the same parser. */
    assert(actuator_mobile_feedback_feed(&r,token,req,8,now));
    for(size_t i=0;i<n;i++)assert(actuator_mobile_feedback_feed(&r,token,bytes+i,1,now+1));
    assert(!actuator_mobile_feedback_commit(&r,token,now+1,false,true));
    assert(!actuator_mobile_feedback_commit(&r,token,now+1,true,false));
    assert(actuator_mobile_feedback_commit(&r,token,now+1,true,true));
}
static void sweep(void) { for(unsigned i=0;i<8;i++)read_one(); }
static void publish(void) {
    lift.observed_ms=now;
    assert(actuator_mobile_feedback_publish(&r,&lift,now+1));
}
static void good_sweep_and_expiry(uint32_t start) {
    setup(start);
    assert(!actuator_mobile_arm(&s,1,now));
    for(unsigned i=0;i<7;i++) {read_one(); assert(!actuator_mobile_feedback_publish(&r,&lift,now));}
    read_one();publish();
    assert(s.feedback.observed_ms==start+20);
    assert(s.feedback.lift_position_um==12000 && actuator_mobile_arm(&s,1,now+1));
    assert(!actuator_mobile_feedback_publish(&r,&lift,now+1));
    /* Re-reading only one motor cannot replace the other three timestamps. */
    read_one(); assert(!actuator_mobile_feedback_publish(&r,&lift,now+1));
    actuator_mobile_feedback_reader_poll(&r,start+120);
    assert(s.state==ACTUATOR_MOBILE_STOP_LATCHED && !s.feedback.hardware_ok);
    assert(actuator_mobile_feedback_reset(&r,true,true));
    assert(!s.have_feedback && s.state==ACTUATOR_MOBILE_STOP_LATCHED);
    assert(!actuator_mobile_arm(&s,2,start+120));
}
static void bad_mode_and_recovery(void) {
    setup(100); uint8_t request[8],bytes[8],data=0;uint32_t token;
    assert(!actuator_mobile_feedback_begin(&r,now,false,request,&token));
    assert(actuator_mobile_feedback_begin(&r,now,true,request,&token));
    assert(!actuator_mobile_feedback_begin(&r,now,true,request,&token));
    size_t n=reply(bytes,8,0,&data,1);
    assert(!actuator_mobile_feedback_feed(&r,token+1,bytes,n,now));
    assert(actuator_mobile_feedback_feed(&r,token,bytes,n,now));
    assert(!actuator_mobile_feedback_commit(&r,token,now,true,true));
    assert(r.fault==MOBILE_FEEDBACK_MODE && r.active);
    assert(!actuator_mobile_feedback_reset(&r,false,true));
    assert(!actuator_mobile_feedback_reset(&r,true,false));
    assert(actuator_mobile_feedback_reset(&r,true,true));
    assert(actuator_mobile_feedback_begin(&r,now+1,true,request,&token) && token==2);
    assert(!actuator_mobile_feedback_commit(&r,1,now+1,true,true));
}
static void corruption_and_terminal(void) {
    setup(100); uint8_t request[8],bytes[21],data=1;uint32_t token;
    assert(actuator_mobile_feedback_begin(&r,now,true,request,&token));
    size_t n=reply(bytes,8,0,&data,1); bytes[n-1]^=1;
    assert(actuator_mobile_feedback_feed(&r,token,bytes,n,now));
    assert(!r.reply_ready);
    bytes[n-1]^=1;
    assert(actuator_mobile_feedback_feed(&r,token,bytes,n,now+1));
    assert(!actuator_mobile_feedback_feed(&r,token,bytes,n,now+1));
    assert(r.fault==MOBILE_FEEDBACK_AMBIGUOUS);
    setup(100); assert(actuator_mobile_feedback_begin(&r,now,true,request,&token));
    n=reply(bytes,8,4,NULL,0); /* short, checksum-valid device fault */
    assert(!actuator_mobile_feedback_feed(&r,token,bytes,n,now));
    assert(r.fault==MOBILE_FEEDBACK_STATUS);
    actuator_sts_response_t parser;
    actuator_sts_response_init(&parser,8,15);
    for(size_t i=0;i<n;i++)actuator_sts_response_push(&parser,bytes[i]);
    for(unsigned i=0;i<100000;i++)assert(actuator_sts_response_push(&parser,255)==ACTUATOR_STS_RESPONSE_STATUS_ERROR);
    assert(actuator_sts_response_data(&parser)==NULL);
    actuator_sts_response_init(&parser,8,1);
    n=reply(bytes,8,0,&data,1);
    for(size_t i=0;i<n;i++)actuator_sts_response_push(&parser,bytes[i]);
    for(unsigned i=0;i<100000;i++)assert(actuator_sts_response_push(&parser,255)==ACTUATOR_STS_RESPONSE_FRAME_READY);
    assert(*actuator_sts_response_data(&parser)==1);
    for(unsigned len=17;len<256;len++) {
        actuator_sts_response_init(&parser,8,(uint8_t)len);
        assert(actuator_sts_response_push(&parser,255)==ACTUATOR_STS_RESPONSE_FRAME_REJECTED);
    }
}
static void timeout_and_overflow(void) {
    setup(UINT32_MAX-2);uint8_t req[8],junk[65]={0};uint32_t token;
    assert(actuator_mobile_feedback_begin(&r,now,true,req,&token));
    assert(!actuator_mobile_feedback_feed(&r,token,junk,1,now+5));
    assert(r.fault==MOBILE_FEEDBACK_TIMEOUT);
    assert(actuator_mobile_feedback_reset(&r,true,true));
    assert(actuator_mobile_feedback_begin(&r,now+6,true,req,&token));
    assert(!actuator_mobile_feedback_feed(&r,token,junk,sizeof(junk),now+6));
    assert(r.fault==MOBILE_FEEDBACK_OVERFLOW);
    assert(actuator_mobile_feedback_reset(&r,true,true));r.token=UINT32_MAX;
    assert(!actuator_mobile_feedback_begin(&r,now+6,true,req,&token));
}
static void decode_and_device_limits(void) {
    for(unsigned issue=0;issue<7;issue++) {
        setup(100);for(unsigned i=0;i<4;i++)read_one();
        uint8_t req[8],data[15],bytes[21];uint32_t token;
        now+=4;assert(actuator_mobile_feedback_begin(&r,now,true,req,&token));
        telemetry(data); data[2]=123;data[3]=0x80;data[4]=5;data[5]=4;data[13]=20;data[14]=0x80;
        if(issue==1)data[6]=89;
        if(issue==2)data[6]=141;
        if(issue==3)data[7]=71;
        if(issue==4){data[13]=255;data[14]=255;}
        if(issue==5)data[10]=2;
        if(issue==6)data[5]=0x80;
        size_t n=reply(bytes,req[2],0,data,15);
        assert(actuator_mobile_feedback_feed(&r,token,bytes,n,now));
        bool ok=actuator_mobile_feedback_commit(&r,token,now,true,true);
        assert(ok==(issue==0));
        if(ok)assert(r.axes[0].velocity_raw==-123 && r.axes[0].position_raw==4095 &&
            r.axes[0].load_raw==-5 && r.axes[0].current_raw==-20);
        else assert(r.fault==MOBILE_FEEDBACK_DEVICE);
    }
}
static void missing_lift_and_modes(void) {
    setup(100);sweep();assert(actuator_mobile_feedback_publish(&r,NULL,now));
    assert(!s.feedback.lift_homed && !actuator_mobile_arm(&s,1,now));
    setup(100);sweep();publish();assert(actuator_mobile_arm(&s,1,now));
    lift.hardware_ok=false;
    assert(!actuator_mobile_feedback_publish(&r,&lift,now));
    assert(s.state==ACTUATOR_MOBILE_STOP_LATCHED && !s.feedback.hardware_ok);
    setup(100);sweep();publish();assert(actuator_mobile_arm(&s,1,now));
    r.axes[2].mode_ms=now-r.config.mode_timeout_ms;
    actuator_mobile_feedback_reader_poll(&r,now);
    assert(s.state==ACTUATOR_MOBILE_STOP_LATCHED && !s.feedback.velocity_modes_verified);
    setup(100);r.config.maximum_sample_skew_ms=5;sweep();
    assert(!actuator_mobile_feedback_publish(&r,&lift,now));
}
int main(void) {
    good_sweep_and_expiry(100);good_sweep_and_expiry(UINT32_MAX-24);
    bad_mode_and_recovery();corruption_and_terminal();timeout_and_overflow();
    decode_and_device_limits();missing_lift_and_modes();
    puts("mobile feedback: replies, freshness, modes, limits, fault recovery and clock wrap passed");
    return 0;
}
