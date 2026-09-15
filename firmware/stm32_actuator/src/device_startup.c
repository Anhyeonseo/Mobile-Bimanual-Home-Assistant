#include "actuator_core/device_startup.h"
#include "actuator_core/crc32c.h"
#include <string.h>
static uint32_t r32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void w32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static void fault(actuator_device_startup_t *s){s->state=DEVICE_STARTUP_FAULT;}
bool actuator_device_startup_begin(actuator_device_startup_t *s,const actuator_device_profile_t *p,
    uint32_t now,bool change,bool secured){
    if(s==NULL||p==NULL||s->state==DEVICE_STARTUP_CHECKING||s->eprom_unlocked||!p->timeout_ms||p->timeout_ms>=UINT32_C(0x80000000)||p->baud_code>7||
       (change&&!secured))return false;
    for(unsigned i=0;i<4;i++)if(!p->model[i]||p->model[i]==UINT16_MAX)return false;
    memset(s,0,sizeof(*s));s->profile=*p;s->state=DEVICE_STARTUP_CHECKING;
    s->started_ms=now;s->allow_mode_change=change;return true;
}
void actuator_device_startup_poll(actuator_device_startup_t *s,uint32_t now){
    if(s!=NULL&&s->state==DEVICE_STARTUP_CHECKING&&now-s->started_ms>=s->profile.timeout_ms)fault(s);
}
bool actuator_device_startup_next(actuator_device_startup_t *s,uint32_t now,actuator_device_operation_t *op){
    if(s==NULL||op==NULL)return false;
    actuator_device_startup_poll(s,now);
    if(s->state!=DEVICE_STARTUP_CHECKING||s->active||s->token==UINT32_MAX)return false;
    actuator_device_operation_t x={.id=(uint8_t)(8+s->axis),.token=++s->token};
    /* identity -> mode -> (torque-off, unlock, mode, lock) -> mode readback -> lock readback */
    switch(s->phase){
    case 0:x.address=3;x.length=4;break;
    case 1:case 6:x.address=33;x.length=1;break;
    case 2:x.address=40;x.length=1;break;
    case 3:x.address=55;x.length=1;x.write=true;x.data[0]=0;s->eprom_unlocked=true;break;
    case 4:x.address=33;x.length=1;x.write=true;x.data[0]=1;break;
    case 5:x.address=55;x.length=1;x.write=true;x.data[0]=1;break;
    case 7:x.address=55;x.length=1;break;
    default:fault(s);return false;
    }
    s->pending=x;s->active=true;*op=x;return true;
}
bool actuator_device_startup_reply(actuator_device_startup_t *s,uint32_t token,const uint8_t *data,
    size_t length,bool ok,uint32_t now){
    if(s==NULL||!s->active||token!=s->pending.token)return false;
    actuator_device_startup_poll(s,now);
    if(s->state!=DEVICE_STARTUP_CHECKING)return false;
    if(!ok||length!=(s->pending.write?0:s->pending.length)||(length&&data==NULL)){fault(s);return false;}
    switch(s->phase){
    case 0:if((uint16_t)(data[0]|((uint16_t)data[1]<<8))!=s->profile.model[s->axis]||
              data[2]!=8+s->axis||data[3]!=s->profile.baud_code){fault(s);return false;}s->phase=1;break;
    case 1:if(data[0]==1)s->phase=7;
           else if(s->allow_mode_change)s->phase=2;
           else {fault(s);return false;}break;
    case 2:if(data[0]!=0){fault(s);return false;}s->phase=3;break;
    case 3:s->eprom_unlocked=true;s->phase=4;break;
    case 4:s->mode_changed=true;s->phase=5;break;
    case 5:s->phase=6;break;
    case 6:if(data[0]!=1){fault(s);return false;}s->phase=7;break;
    case 7:if(data[0]!=1){fault(s);return false;}s->eprom_unlocked=false;
           if(++s->axis==4)s->state=DEVICE_STARTUP_READY;else s->phase=0;break;
    default:fault(s);return false;
    }
    s->active=false;return true;
}
static bool record(const uint8_t *p){return r32(p)==UINT32_C(0x31425441)&&r32(p+4)!=0&&
    r32(p+8)==~r32(p+4)&&r32(p+12)==actuator_crc32c(p,12);}
bool actuator_boot_advance(actuator_boot_read_fn read,actuator_boot_write_fn write,void *ctx,
    bool initialize,uint32_t *id){
    uint8_t a[16],b[16],out[16],check[16];
    if(read==NULL||write==NULL||id==NULL||!read(ctx,0,a)||!read(ctx,1,b))return false;
    bool av=record(a),bv=record(b),ae=true,be=true;uint32_t last=0;unsigned target;
    for(unsigned i=0;i<16;i++){if(a[i]!=255)ae=false;if(b[i]!=255)be=false;}
    /* A corrupt newest record cannot be distinguished from a torn write.
     * Never fall back and risk reusing an already exposed identity. */
    if((!av&&!ae)||(!bv&&!be))return false;
    if(av&&bv){uint32_t x=r32(a+4),y=r32(b+4);if(x==y||(x>y?x-y:y-x)!=1)return false;last=x>y?x:y;target=x>y?1:0;}
    else if(av||bv){last=r32((av?a:b)+4);target=av?1:0;}
    else {if(!initialize)return false;for(unsigned i=0;i<16;i++)if(a[i]!=255||b[i]!=255)return false;target=0;}
    if(last==UINT32_MAX)return false;
    w32(out,UINT32_C(0x31425441));w32(out+4,last+1);w32(out+8,~(last+1));w32(out+12,actuator_crc32c(out,12));
    if(!write(ctx,target,out)||!read(ctx,target,check)||memcmp(check,out,16))return false;
    *id=last+1;return true;
}
