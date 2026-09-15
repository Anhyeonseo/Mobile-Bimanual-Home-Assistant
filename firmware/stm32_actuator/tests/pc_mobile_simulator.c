/* Deterministic test plant around the REAL endpoint/supervisor/router/packet
 * code. Not firmware for flashing. Servo feedback follows last targets ideally;
 * no physical mode, voltage, traction, torque or lift hold is proven. */
#include "actuator_core/mobile_endpoint.h"
#include "actuator_core/bus_router.h"
#include "actuator_core/motor_groups.h"
#include <string.h>
#include "actuator_core/mobile_framed.h"
#if ACTUATOR_PROTOCOL_VERSION == 2
static actuator_stream_parser_t host_parser;
static bool host_attached=true;
#endif
static actuator_mobile_supervisor_t supervisor;
static actuator_mobile_endpoint_t endpoint;
static actuator_bus_router_t router;
static uint32_t transmitted;
void pc_mobile_reset(uint32_t boot_id) {
#if ACTUATOR_PROTOCOL_VERSION == 2
    actuator_stream_parser_init(&host_parser);host_attached=true;
#endif
    actuator_mobile_config_t config={{1000,1000,1000,100},2,250,500,0,500000};
    actuator_mobile_init(&supervisor,&config);
    actuator_mobile_endpoint_init(&endpoint,&supervisor,boot_id);
    actuator_bus_router_init(&router);transmitted=0;
}
static int output(uint32_t now) {
    uint8_t packet[26];size_t length;const uint8_t *bytes;uint32_t token;
    actuator_bus_work_t kind;uint32_t us=now*1000u;
    if(supervisor.state==ACTUATOR_MOBILE_STOP_LATCHED) {
        uint8_t ids[4]={8,9,10,11};uint16_t zero[4]={0};
        if(actuator_sts3215_build_sync_write_words(46,ids,zero,4,packet,&length)!=ACTUATOR_STS3215_PACKET_OK)return -1;
        if(!actuator_bus_router_submit(&router,ACTUATOR_BUS_WORK_STOP,packet,length,us,us+5000,500))return -2;
    } else {
        if(actuator_motor_group_velocities(ACTUATOR_GROUP_BASE_WHEELS,supervisor.target_raw,3,packet,&length)!=ACTUATOR_GROUP_OK)return -3;
        if(!actuator_bus_router_submit(&router,ACTUATOR_BUS_WORK_WHEELS,packet,length,us,us+5000,500))return -4;
        if(actuator_motor_group_velocities(ACTUATOR_GROUP_LIFT,supervisor.target_raw+3,1,packet,&length)!=ACTUATOR_GROUP_OK)return -5;
        if(!actuator_bus_router_submit(&router,ACTUATOR_BUS_WORK_LIFT,packet,length,us,us+5000,500))return -6;
    }
    while(actuator_bus_router_next(&router,us,1000,&bytes,&length,&token,&kind)) {
        if(length<8 || bytes[0]!=255 || bytes[1]!=255)return -7;
        transmitted++;
        if(!actuator_bus_router_complete(&router,token,us+100,true))return -8;
        us+=150;
    }
    return 0;
}
int pc_mobile_exchange(const uint8_t *request,int length,uint32_t now,uint8_t *reply) {
    actuator_mobile_feedback_t feedback={0};int replies=0;
    actuator_mobile_poll(&supervisor,now);
    feedback.observed_ms=now;memcpy(feedback.velocity_raw,supervisor.target_raw,sizeof(feedback.velocity_raw));
    feedback.lift_position_um=100000;feedback.velocity_modes_verified=true;feedback.lift_homed=true;feedback.hardware_ok=true;
    actuator_mobile_feedback(&supervisor,&feedback,now);
    for(int i=0;i<length;i++)replies+=actuator_mobile_endpoint_feed(&endpoint,request[i],now,reply)?1:0;
    if(supervisor.state==ACTUATOR_MOBILE_READY && actuator_mobile_measured_stopped(&supervisor,now))
        actuator_bus_router_resume(&router,true);
    if(output(now)!=0)return -1;
    return replies;
}
uint32_t pc_mobile_transmissions(void){return transmitted;}

#if ACTUATOR_PROTOCOL_VERSION == 2
void pc_host_set_attached(int attached) {host_attached=attached!=0;}
/* Exactly one framed response, or zero while a fragmented frame is incomplete.
 * The UART fixture passes arbitrary chunks; actual C COBS/CRC parsing is used. */
int pc_host_feed(const uint8_t *bytes,int count,uint32_t now,uint8_t *reply,int capacity) {
    actuator_frame_t request,response;size_t length=0;int responses=0;
    if(count<0 || capacity<(int)ACTUATOR_PROTOCOL_MAX_ENCODED_SIZE)return -1;
    for(int i=0;i<count;i++) {
        if(actuator_stream_parser_push(&host_parser,bytes[i],&request)!=ACTUATOR_PROTOCOL_OK)continue;
        actuator_mobile_feedback_t feedback={0};
        actuator_mobile_poll(&supervisor,now);
        feedback.observed_ms=now;memcpy(feedback.velocity_raw,supervisor.target_raw,sizeof(feedback.velocity_raw));
        feedback.lift_position_um=100000;feedback.velocity_modes_verified=true;feedback.lift_homed=true;feedback.hardware_ok=true;
        actuator_mobile_feedback(&supervisor,&feedback,now);
        if(request.message_type==ACTUATOR_MSG_GET_STATE || request.message_type==ACTUATOR_MSG_HEARTBEAT) {
            /* Synthetic arm status fixture, not the real arm executor. */
            memset(&response,0,sizeof(response));response.message_type=ACTUATOR_MSG_STATE_FEEDBACK;
            response.sequence=request.sequence;response.sender_time_ms=now;response.payload_length=24;
            response.payload[2]=12;response.payload[3]=2;
        } else if(!actuator_mobile_framed_reply(host_attached?&endpoint:NULL,&request,now,&response))continue;
        if(supervisor.state==ACTUATOR_MOBILE_READY && actuator_mobile_measured_stopped(&supervisor,now))
            actuator_bus_router_resume(&router,true);
        if(output(now)!=0)return -2;
        if(responses++ || actuator_frame_encode(&response,reply,(size_t)capacity,&length)!=ACTUATOR_PROTOCOL_OK)return -3;
    }
    return responses?(int)length:0;
}
#endif
