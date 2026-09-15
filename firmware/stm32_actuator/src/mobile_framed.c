#include "actuator_core/mobile_framed.h"
#include "actuator_core/crc32c.h"
#include <string.h>
static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
bool actuator_mobile_framed_reply(actuator_mobile_endpoint_t *endpoint,
    const actuator_frame_t *request, uint32_t now_ms, actuator_frame_t *response) {
    if(request==NULL || response==NULL || request->message_type!=ACTUATOR_V2_MSG_MOBILE_REQUEST)
        return false;
    memset(response,0,sizeof(*response));
    response->message_type=ACTUATOR_V2_MSG_MOBILE_RESPONSE;
    response->sequence=request->sequence;response->sender_time_ms=now_ms;
    response->payload_length=1;response->payload[0]=2;
    const uint8_t *p=request->payload;
    if(request->flags!=0 || request->payload_length!=ACTUATOR_MOBILE_WIRE_SIZE || p[0]!='A' || p[1]!='M' || p[2]!=1 ||
       p[3]<1 || p[3]>6 || read32(p+32)!=actuator_crc32c(p,32))return true;
    if(endpoint==NULL || endpoint->supervisor==NULL) {response->payload[0]=1;return true;}
    /* Outer frame is complete and CRC checked: discard any raw-stream residue.
     * This endpoint must be owned only by the framed route once attached. */
    endpoint->used=0;
    bool replied=false;
    for(unsigned i=0;i<ACTUATOR_MOBILE_WIRE_SIZE;i++)
        replied=actuator_mobile_endpoint_feed(endpoint,p[i],now_ms,response->payload+1) || replied;
    if(replied) {response->payload[0]=0;response->payload_length=1+ACTUATOR_MOBILE_STATUS_SIZE;}
    return true;
}
