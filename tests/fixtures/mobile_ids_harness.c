#include "actuator_core/mobile_ids.h"
#include "actuator_core/motor_groups.h"
#include "actuator_core/device_startup.h"
#include "actuator_core/mobile_output.h"
#include "actuator_core/mobile_feedback.h"
#include "actuator_core/system_stop.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
static unsigned first;
static void check_zero(const actuator_bus_job_t *job) {
    assert(job->pending && job->length==20 && job->bytes[5]==46);
    for(unsigned i=0;i<4;i++)assert(job->bytes[7+3*i]==first+i && !job->bytes[8+3*i] && !job->bytes[9+3*i]);
}
int main(int argc,char **argv) {
    assert(argc==2);first=(unsigned)atoi(argv[1]);
    uint8_t packet[26];size_t length;int32_t rates[3]={1,-2,3};
    assert(actuator_motor_group_velocities(ACTUATOR_GROUP_BASE_WHEELS,rates,3,packet,&length)==ACTUATOR_GROUP_OK);
    assert(length==17);for(unsigned i=0;i<3;i++)assert(packet[7+3*i]==first+i);
    assert(actuator_motor_group_velocities(ACTUATOR_GROUP_LIFT,rates,1,packet,&length)==ACTUATOR_GROUP_OK);
    assert(length==11 && packet[7]==first+3);
    uint16_t pose[6]={2048,2048,2048,2048,2048,2048};
    assert(actuator_motor_group_positions(ACTUATOR_GROUP_LEFT_ARM,pose,6,packet,&length)==ACTUATOR_GROUP_OK);
    for(unsigned i=0;i<6;i++)assert(packet[7+3*i]==i+1);
    actuator_device_profile_t dp={{777,777,777,777},0,1000};actuator_device_startup_t ds={0};
    assert(actuator_device_startup_begin(&ds,&dp,0,false,true));
    for(unsigned i=0;i<4;i++) {
        actuator_device_operation_t op;uint8_t identity[4]={9,3,(uint8_t)(first+i),0},one=1;
        assert(actuator_device_startup_next(&ds,0,&op) && !op.write && op.id==first+i && op.address==3);
        assert(actuator_device_startup_reply(&ds,op.token,identity,4,true,0));
        for(unsigned phase=0;phase<2;phase++) {
            assert(actuator_device_startup_next(&ds,0,&op) && !op.write && op.id==first+i);
            assert(actuator_device_startup_reply(&ds,op.token,&one,1,true,0));
        }
    }
    assert(ds.state==DEVICE_STARTUP_READY);
    actuator_mobile_config_t limits={{1000,1000,1000,100},2,200,200,0,500000};
    actuator_mobile_supervisor_t supervisor;assert(actuator_mobile_init(&supervisor,&limits));
    actuator_mobile_feedback_config_t fc={.sample_timeout_ms=100,.mode_timeout_ms=400,.response_timeout_ms=5,.maximum_sample_skew_ms=30};
    for(unsigned i=0;i<4;i++)fc.axes[i]=(actuator_mobile_axis_feedback_config_t){1,90,140,70,1000};
    actuator_mobile_feedback_reader_t reader;assert(actuator_mobile_feedback_reader_init(&reader,&supervisor,&fc));
    for(unsigned i=0;i<8;i++) {
        uint8_t req[8],response[21]={255,255};uint32_t token,now=i*4;
        assert(actuator_mobile_feedback_begin(&reader,now,true,req,&token));
        assert(req[2]==first+i%4);
        unsigned n=reader.reading_mode?1:15;response[2]=req[2];response[3]=(uint8_t)(n+2);
        if(n==1)response[5]=1;else {response[11]=120;response[12]=30;}
        response[n+5]=actuator_sts3215_checksum(response+2,n+3);
        assert(actuator_mobile_feedback_feed(&reader,token,response,n+6,now+1));
        assert(actuator_mobile_feedback_commit(&reader,token,now+1,true,true));
    }
    actuator_mobile_output_config_t oc={{5000,1000,200,10000,20000,5000,500},500,400,600,4000,{1,1,1,1}};
    actuator_mobile_output_t output;assert(actuator_mobile_output_init(&output,&supervisor,&oc,0));
    actuator_mobile_output_stop(&output,MOBILE_OUTPUT_TRANSPORT);
    actuator_mobile_output_poll(&output,100,0,0);check_zero(&output.router.jobs[0]);
    actuator_bus_router_t left,right;actuator_bus_router_init(&left);actuator_bus_router_init(&right);
    actuator_system_stop_t stop={0};actuator_system_stop_config_t sc={4000,600,600,200,100};
    assert(actuator_system_stop_begin_timed(&stop,&left,&right,pose,pose,true,true,100,0,&sc));
    check_zero(&left.jobs[0]);
    return 0;
}
