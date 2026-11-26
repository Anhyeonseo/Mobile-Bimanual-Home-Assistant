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
static void drive_readback(void){
    for(unsigned torque=0;torque<=1;torque++){
        setup(100);r.read_drive_state=true;
        uint8_t req[8],bytes[22],data[16]={0};uint32_t token;
        assert(actuator_mobile_feedback_begin(&r,now,true,req,&token));
        assert(req[5]==33&&req[6]==16);data[0]=1;data[7]=(uint8_t)torque;
        data[13]=123;data[14]=128;
        size_t n=reply(bytes,req[2],0,data,16);
        assert(actuator_mobile_feedback_feed(&r,token,bytes,n,now+1));
        bool ok=actuator_mobile_feedback_commit(&r,token,now+1,true,true);
        assert(ok==(torque==1));
        if(ok){
            assert(r.axes[0].drive_valid&&r.axes[0].torque_enabled==1&&r.axes[0].goal_velocity_raw==-123);
            r.cursor=0;now+=50;
            assert(actuator_mobile_feedback_begin(&r,now,true,req,&token));
            assert(req[5]==33&&req[6]==16); /* refresh before telemetry age expires */
        }else assert(r.fault==MOBILE_FEEDBACK_DEVICE);
    }
}
static void stopped_position_window(uint32_t start){
    for(unsigned issue=0;issue<9;issue++){
        setup(start);actuator_mobile_stationary_window_t windows[3]={0};
        bool proof=false;
        for(unsigned pass=0;pass<7;pass++){
            now=start+1+pass*80;
            for(unsigned i=0;i<3;i++){
                r.axes[i]=(actuator_mobile_axis_sample_t){.position_raw=4095+(int)(pass%2),
                    .velocity_raw=pass%2?50:-50,.moving=1,.observed_ms=now,.mode_ms=now,
                    .valid=true,.mode_verified=true,.drive_valid=true,.torque_enabled=1};
                r.axes[i].position_raw%=4096;
            }
            if(issue==1)r.axes[1].position_raw=(4095+pass)%4096; /* accumulated drift */
            if(issue==2)r.axes[0].velocity_raw=51;
            if(issue==3)r.axes[2].goal_velocity_raw=1;
            if(issue==4)r.axes[2].torque_enabled=0;
            if(issue==5)r.axes[2].observed_ms=start+1; /* repeated cached read */
            if(issue==6)r.axes[2].mode_ms=start; /* pre-zero drive */
            if(issue==7)r.fault=MOBILE_FEEDBACK_DEVICE;
            if(issue==8)r.axes[0].moving=2;
            proof=actuator_mobile_wheels_stop_observe(windows,&r,start,now);
            if(pass<5)assert(!proof);
        }
        assert(proof==(issue==0));
        assert(!actuator_mobile_wheels_stop_observe(windows,&r,start,now+100));
    }
}
static bool read_temperature(uint8_t temperature,bool bad_voltage){
    uint8_t req[8],data[15],bytes[21];uint32_t token;now+=4;
    assert(actuator_mobile_feedback_begin(&r,now,true,req,&token));
    assert(!r.reading_mode);telemetry(data);data[7]=temperature;
    if(bad_voltage)data[6]=1;
    assert(req[5]==(r.temperature_pending?63:56) && req[6]==(r.temperature_pending?1:15));
    size_t n=reply(bytes,req[2],0,r.temperature_pending?&temperature:data,req[6]);
    assert(actuator_mobile_feedback_feed(&r,token,bytes,n,now));
    return actuator_mobile_feedback_commit(&r,token,now+1,true,true);
}
static void temperature_confirmation(uint32_t start){
    for(unsigned issue=0;issue<12;issue++){
        setup(start);sweep();r.confirm_temperature_spike=true;r.cursor=0;
        uint32_t previous=r.axes[0].observed_ms;
        if(issue==1)r.confirm_temperature_spike=false;
        if(issue==2)r.axes[0].valid=false;
        if(issue==3)r.axes[0].temperature=65;
        if(issue==4)r.cursor=3; /* Lift still fails immediately. */
        if(issue==5)r.axes[0].observed_ms=now-50;
        bool eligible=issue==0||issue==6||issue>=8;
        assert(read_temperature(issue==6?90:76,issue==7)==eligible);
        if(!eligible){assert(r.fault==MOBILE_FEEDBACK_DEVICE);continue;}
        assert(r.temperature_pending&&r.temperature_rechecks==1&&r.failure_count==0);
        assert(r.axes[0].observed_ms==previous&&r.axes[0].temperature==30);
        if(issue==8){assert(!read_temperature(76,false));assert(r.fault==MOBILE_FEEDBACK_DEVICE);}
        else if(issue==9){
            uint8_t req[8];uint32_t token;now+=4;
            assert(actuator_mobile_feedback_begin(&r,now,true,req,&token));
            actuator_mobile_feedback_reader_poll(&r,now+5);assert(r.fault==MOBILE_FEEDBACK_TIMEOUT);
        }else if(issue==10){
            /* No retry is scheduled: bounded even with no UART transaction. */
            actuator_mobile_feedback_reader_poll(&r,r.temperature_last_recheck_ms[0]+50);
            assert(r.fault==MOBILE_FEEDBACK_TIMEOUT);
        }else if(issue==11){
            /* A positive but still over-limit second sample is not recovery. */
            assert(!read_temperature(71,false));assert(r.fault==MOBILE_FEEDBACK_DEVICE);
        }else{
            assert(read_temperature(35,false)&&!r.temperature_pending&&r.temperature_recovered==1);
            assert(r.axes[0].observed_ms==previous&&r.axes[0].temperature==30);
            r.cursor=0;assert(read_temperature(76,false));
            assert(r.temperature_pending&&r.temperature_rechecks==2);
            assert(!read_temperature(76,false)); /* Consecutive hot replies still stop. */
        }
    }
}
static void recorded_temperature_spikes(uint32_t start){
    const uint8_t spikes[]={89,92,255};
    for(unsigned axis=0;axis<3;axis++)for(unsigned k=0;k<3;k++){
        setup(start);sweep();r.confirm_temperature_spike=true;r.cursor=axis;
        uint32_t previous=r.axes[axis].observed_ms;
        assert(read_temperature(spikes[k],false));
        assert(r.axes[axis].observed_ms==previous&&r.temperature_pending);
        assert(read_temperature(34,false));
        assert(r.fault==MOBILE_FEEDBACK_OK&&r.temperature_recovered==1);
        assert(r.axes[axis].temperature==30 && r.axes[axis].observed_ms==previous);
    }
    /* Real thermal rise at the threshold never enters confirmation. */
    setup(start);sweep();r.confirm_temperature_spike=true;r.cursor=0;
    r.axes[0].temperature=69;
    assert(!read_temperature(71,false));assert(r.fault==MOBILE_FEEDBACK_DEVICE);
}
static void recurrent_isolated_temperature_spikes(uint32_t start){
    for(unsigned axis=0;axis<3;axis++){
        setup(start);sweep();r.confirm_temperature_spike=true;
        for(unsigned k=0;k<6;k++){
            r.cursor=axis;uint32_t previous=r.axes[axis].observed_ms;
            /* Recorded 35 -> 79 after an earlier recovered spike: the old
             * 1000 ms cooldown rejected it without obtaining another sample. */
            assert(read_temperature(79,false));
            assert(r.temperature_pending&&r.axes[axis].observed_ms==previous);
            assert(read_temperature(35,false));
            assert(!r.temperature_pending&&r.temperature_recovered==k+1);
            assert(r.fault==MOBILE_FEEDBACK_OK&&r.failure_count==0);
            r.cursor=axis;
            assert(read_temperature(35,false)); /* a new full telemetry frame */
        }
        r.cursor=axis;assert(read_temperature(79,false));
        assert(!read_temperature(79,false));
        assert(r.fault==MOBILE_FEEDBACK_DEVICE&&r.failure_count==1);
    }
}
static void moving_wheel_temperature(uint32_t start) {
    setup(start); sweep(); publish();
    assert(actuator_mobile_arm(&s,1,now+1));
    int32_t drive[4]={20,20,20,0};
    assert(actuator_mobile_command(&s,1,1,drive,now+1));
    r.confirm_temperature_spike=true; r.cursor=1; r.config.sample_timeout_ms=250;
    r.axes[1].temperature=33; r.axes[1].observed_ms=now+4-50;
    uint32_t previous=r.axes[1].observed_ms;
    assert(read_temperature(90,false));
    assert(s.wheel_temperature_hold && !s.wheel_zero_written && r.axes[1].observed_ms==previous);
    uint8_t req[8]; uint32_t token;
    assert(!actuator_mobile_feedback_begin(&r,now+2,true,req,&token));
    assert(actuator_mobile_command(&s,1,2,drive,now+2));
    for(unsigned i=0;i<3;++i)assert(!s.target_raw[i]);
    s.wheel_zero_written=true; s.wheel_zero_ms=now+2;
    assert(read_temperature(33,false));
    assert(!r.temperature_pending && s.wheel_temperature_hold && r.axes[1].observed_ms==previous);
    for(unsigned i=0;i<4;i++) read_one();
    r.axes[2].velocity_raw=20;
    actuator_mobile_feedback_reader_poll(&r,now+1);
    assert(s.wheel_temperature_hold);
    r.axes[2].velocity_raw=0;
    actuator_mobile_feedback_reader_poll(&r,now+1);
    assert(!s.wheel_temperature_hold && !s.target_raw[0]);
    assert(actuator_mobile_command(&s,1,3,drive,now+1) && s.target_raw[0]==20);
}
int main(void) {
    moving_wheel_temperature(200); moving_wheel_temperature(UINT32_MAX-40);
    good_sweep_and_expiry(100);good_sweep_and_expiry(UINT32_MAX-24);
    bad_mode_and_recovery();corruption_and_terminal();timeout_and_overflow();
    decode_and_device_limits();missing_lift_and_modes();drive_readback();
    stopped_position_window(100);stopped_position_window(UINT32_MAX-200);
    temperature_confirmation(100);temperature_confirmation(UINT32_MAX-24);
    recorded_temperature_spikes(100);recorded_temperature_spikes(UINT32_MAX-24);
    recurrent_isolated_temperature_spikes(100);recurrent_isolated_temperature_spikes(UINT32_MAX-24);
    puts("mobile feedback: replies, freshness, modes, limits, fault recovery and clock wrap passed");
    return 0;
}
