#include "actuator_core/wire_bytes.h"
#include "actuator_core/mobile_endpoint.h"
#include "actuator_core/crc32c.h"
#include <string.h>
static void reject(actuator_mobile_endpoint_t *e) {if(e->rejected_frames!=UINT32_MAX)e->rejected_frames++;}
bool actuator_mobile_endpoint_init(actuator_mobile_endpoint_t *e,actuator_mobile_supervisor_t *s,uint32_t boot) {
    if(e==NULL || s==NULL || !s->configured || boot==0)return false;
    memset(e,0,sizeof(*e));e->supervisor=s;e->boot_id=boot;return true;
}
static void status(actuator_mobile_endpoint_t *e,uint8_t kind,uint32_t seq,uint32_t now,uint8_t *out) {
    actuator_mobile_supervisor_t *s=e->supervisor;
    unsigned i;
    memset(out,0,64);out[0]='A';out[1]='S';out[2]=1;out[3]=kind;
    actuator_write_u32_le(out+4,seq);actuator_write_u32_le(out+8,now);actuator_write_u32_le(out+12,e->boot_id);
    actuator_write_u32_le(out+16,UINT32_C(0x00000007) | (e->park_proven ? 8u : 0u)); /* commands, feedback, clock; no lift height controller */
    actuator_write_u32_le(out+20,s->session);actuator_write_u32_le(out+24,s->sequence);out[28]=(uint8_t)s->state;out[29]=(uint8_t)s->reason;
    out[30]=(s->have_feedback?1u:0u)|(s->feedback.velocity_modes_verified?2u:0u)|
        (s->feedback.lift_homed?4u:0u)|(actuator_mobile_measured_stopped(s,now)?8u:0u)|
        (s->feedback.hardware_ok?16u:0u)|(s->transport_only?32u:0u);
    actuator_write_u32_le(out+32,s->feedback.observed_ms);
    for(i=0;i<4;i++)actuator_write_u32_le(out+36+4*i,(uint32_t)s->feedback.velocity_raw[i]);
    actuator_write_u32_le(out+52,(uint32_t)s->feedback.lift_position_um);actuator_write_u32_le(out+56,e->rejected_frames);
    actuator_write_u32_le(out+60,actuator_crc32c(out,60));
}
bool actuator_mobile_endpoint_feed(actuator_mobile_endpoint_t *e,uint8_t byte,uint32_t now,uint8_t reply[64]) {
    uint8_t kind;unsigned i;bool accepted;
    if(e==NULL || reply==NULL || e->supervisor==NULL)return false;
    actuator_mobile_poll(e->supervisor,now);
    if(e->used && now-e->last_byte_ms>=100u) {e->used=0;reject(e);}
    e->last_byte_ms=now;e->buffer[e->used++]=byte;
    while(e->used && (e->buffer[0]!='A' || (e->used>1 && e->buffer[1]!='M') ||
          (e->used>2 && e->buffer[2]!=1))) {
        memmove(e->buffer,e->buffer+1,--e->used);reject(e);
    }
    if(e->used<36)return false;
    if(actuator_read_u32_le(e->buffer+32)!=actuator_crc32c(e->buffer,32)) {
        memmove(e->buffer,e->buffer+1,35);e->used=35;reject(e);return false;
    }
    kind=e->buffer[3];accepted=false;
    if(kind>=4 && kind<=6 && actuator_read_u32_le(e->buffer+4)!=0) {
        accepted=true;
        for(i=16;i<32;i++)if(e->buffer[i]!=0)accepted=false;
    } else if(kind==7 && e->park_proven) {
        actuator_mobile_supervisor_t *s=e->supervisor;
        uint32_t sequence=actuator_read_u32_le(e->buffer+8);
        uint32_t remaining=actuator_read_u32_le(e->buffer+12)-now;
        bool zero=true;
        for(i=0;i<4;i++)if(actuator_read_u32_le(e->buffer+16+4*i)||s->target_raw[i])zero=false;
        if(zero && !s->transport_only && s->state==ACTUATOR_MOBILE_ACTIVE &&
           s->session && actuator_read_u32_le(e->buffer+4)==s->session &&
           s->have_sequence && sequence>s->sequence && remaining && remaining<=s->config.command_timeout_ms &&
           actuator_mobile_measured_stopped(s,now) && e->park_proven(now)){
            s->last_parked_session=s->session;
            s->session=0;s->have_sequence=false;s->sequence=0;
            s->state=ACTUATOR_MOBILE_DISABLED;accepted=true;
        }
    } else if(kind>=1 && kind<=3) {
        accepted=actuator_mobile_wire_apply(e->supervisor,e->buffer,36,now);
    }
    if(!accepted)reject(e);
    status(e,accepted?kind:(uint8_t)(kind|0x80u),actuator_read_u32_le(e->buffer+8),now,reply);
    e->used=0;return true;
}
