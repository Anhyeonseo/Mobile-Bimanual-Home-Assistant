#include "actuator_core/robot_evidence.h"
#include "actuator_core/crc32c.h"
#include <string.h>
static uint32_t r32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void w32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
bool actuator_robot_evidence_fresh(const actuator_robot_evidence_t *s,uint32_t now,uint32_t age){
    return s&&s->valid&&age&&age<UINT32_C(0x80000000)&&now-s->observed_ms<age&&
        s->expires_ms-now>0&&s->expires_ms-now<UINT32_C(0x80000000);
}
bool actuator_robot_evidence_accept(actuator_robot_evidence_t *s,uint32_t boot,uint32_t age,
    uint32_t now,const uint8_t q[36],uint8_t out[64]){
    if(!s||!boot||!age||age>1000||!q||!out||memcmp(q,"AE\x01\x01",4)||
       r32(q+32)!=actuator_crc32c(q,32))return false;
    for(unsigned i=21;i<32;i++)if(q[i])return false;
    uint32_t seq=r32(q+4),observed=r32(q+12),expires=r32(q+16);
    if(!seq||seq<=s->sequence||r32(q+8)!=boot||(q[20]&~7u)||
       now-observed>=age||expires-now==0||expires-now>age||expires-observed>age||
       (s->sequence&&(observed-s->observed_ms==0||observed-s->observed_ms>=UINT32_C(0x80000000))))return false;
    s->sequence=seq;s->observed_ms=observed;s->expires_ms=expires;s->flags=q[20];s->valid=true;
    memset(out,0,64);memcpy(out,"AF\x01\x01",4);w32(out+4,seq);w32(out+8,boot);
    w32(out+12,now);w32(out+16,observed);out[20]=s->flags;
    w32(out+60,actuator_crc32c(out,60));return true;
}
