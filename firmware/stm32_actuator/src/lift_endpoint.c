#include "actuator_core/lift_endpoint.h"
#include "actuator_core/crc32c.h"
#include <string.h>
static uint32_t read32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void write32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static bool interlocked(const actuator_lift_endpoint_t *e,uint32_t now){
    return e->transport_ready&&e->have_interlock&&now-e->interlock.observed_ms<e->lift->config.feedback_timeout_ms&&
        e->interlock.base_stopped&&e->interlock.arms_safe&&e->interlock.mode_ok&&e->interlock.hardware_ok;
}
bool actuator_lift_endpoint_init(actuator_lift_endpoint_t *e,actuator_lift_t *l,uint32_t boot,uint32_t lease){
    if(e==NULL||l==NULL||!l->configured||!boot||!lease||lease>1000)return false;
    memset(e,0,sizeof(*e));e->lift=l;e->boot_id=boot;e->lease_max_ms=lease;return true;
}
void actuator_lift_endpoint_stop(actuator_lift_endpoint_t *e){
    if(e==NULL||e->lift==NULL)return;
    actuator_lift_cancel(e->lift);e->active=false;
}
void actuator_lift_endpoint_poll(actuator_lift_endpoint_t *e,uint32_t now){
    if(e==NULL||e->lift==NULL)return;
    actuator_lift_poll(e->lift,now);
    if(e->active){
        uint32_t remaining=e->valid_until_ms-now;
        if(!remaining||remaining>e->lease_max_ms||!interlocked(e,now)){
            actuator_lift_inhibit(e->lift,LIFT_BAD_FEEDBACK);e->active=false;
        }else if(e->lift->state==LIFT_HOLDING||e->lift->state==LIFT_FAULT)e->active=false;
    }
}
bool actuator_lift_endpoint_exchange(actuator_lift_endpoint_t *e,const uint8_t *p,size_t n,uint32_t now,uint8_t out[64]){
    if(e==NULL||e->lift==NULL||p==NULL||out==NULL||n!=36||p[0]!='A'||p[1]!='L'||p[2]!=1||
       p[3]<1||p[3]>6||read32(p+32)!=actuator_crc32c(p,32))return false;
    actuator_lift_endpoint_poll(e,now);
    uint8_t op=p[3];uint32_t session=read32(p+4),seq=read32(p+8),deadline=read32(p+12);
    int32_t target=(int32_t)read32(p+16);uint32_t boot=read32(p+20),remaining=deadline-now;
    bool clean=true;for(unsigned i=24;i<32;i++)if(p[i])clean=false;
    bool ok=clean&&session!=0&&boot==e->boot_id&&(op==2||target==0);
    if(op==4){ok=ok&&deadline==0;}
    else {
        ok=ok&&remaining!=0&&remaining<=e->lease_max_ms;
        if(op==1||op==2){
            ok=ok&&!e->active&&interlocked(e,now)&&
                ((session==e->session&&seq>e->sequence)||session>e->session);
            if(ok)ok=op==1?actuator_lift_home(e->lift,now):actuator_lift_move(e->lift,target,now);
            if(ok){e->active=true;e->session=session;}
        }else {
            ok=ok&&session==e->session&&seq>e->sequence;
            if(op==3&&ok)actuator_lift_endpoint_stop(e);
            if(op==5)ok=ok&&e->active&&interlocked(e,now);
            if(op==6){ok=ok&&!e->active&&interlocked(e,now)&&e->lift->have_sample&&
                now-e->lift->observed_ms<e->lift->config.feedback_timeout_ms&&e->lift->feedback.healthy&&e->lift->feedback.velocity_raw<=e->lift->config.stopped_speed_raw&&
                e->lift->feedback.velocity_raw>=-e->lift->config.stopped_speed_raw;
                if(ok)actuator_lift_reset(e->lift);}
        }
        if(ok){e->sequence=seq;e->valid_until_ms=deadline;}
    }
    if(!ok&&e->rejected!=UINT32_MAX)e->rejected++;
    actuator_lift_t *l=e->lift;memset(out,0,64);out[0]='L';out[1]='S';out[2]=1;out[3]=ok?op:(uint8_t)(op|128);
    write32(out+4,seq);write32(out+8,now);write32(out+12,e->boot_id);write32(out+16,e->session);write32(out+20,e->sequence);
    out[24]=(uint8_t)l->state;out[25]=(uint8_t)l->fault;out[26]=(l->homed?1:0)|(e->active?2:0)|(interlocked(e,now)?4:0)|
        (l->have_sample?8:0)|(l->state==LIFT_HOLDING?16:0);
    write32(out+28,l->observed_ms);write32(out+32,(uint32_t)l->height_um);write32(out+36,(uint32_t)l->target_um);
    write32(out+40,(uint32_t)l->command_raw);write32(out+44,(uint32_t)l->feedback.current_ma);
    write32(out+48,l->previous_raw);write32(out+52,e->rejected);write32(out+56,e->valid_until_ms);
    write32(out+60,actuator_crc32c(out,60));return true;
}
