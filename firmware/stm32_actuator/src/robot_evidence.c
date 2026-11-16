#include "actuator_core/wire_bytes.h"
#include "actuator_core/robot_evidence.h"
#include "actuator_core/crc32c.h"
#include <string.h>
bool actuator_robot_evidence_fresh(const actuator_robot_evidence_t *s,uint32_t now,uint32_t age){
    return s&&s->valid&&age&&age<UINT32_C(0x80000000)&&now-s->observed_ms<age&&
        s->expires_ms-now>0&&s->expires_ms-now<UINT32_C(0x80000000);
}
bool actuator_robot_evidence_accept(actuator_robot_evidence_t *s,uint32_t boot,uint32_t age,
    uint32_t now,const uint8_t q[36],uint8_t out[64]){
    if(!s||!boot||!age||age>1000||!q||!out||memcmp(q,"AE\x01\x01",4)||
       actuator_read_u32_le(q+32)!=actuator_crc32c(q,32))return false;
    for(unsigned i=21;i<32;i++)if(q[i])return false;
    uint32_t seq=actuator_read_u32_le(q+4),observed=actuator_read_u32_le(q+12),expires=actuator_read_u32_le(q+16);
    if(!seq||seq<=s->sequence||actuator_read_u32_le(q+8)!=boot||(q[20]&~7u)||
       now-observed>=age||expires-now==0||expires-now>age||expires-observed>age||
       (s->sequence&&(observed-s->observed_ms==0||observed-s->observed_ms>=UINT32_C(0x80000000))))return false;
    s->sequence=seq;s->observed_ms=observed;s->expires_ms=expires;s->flags=q[20];s->valid=true;
    memset(out,0,64);memcpy(out,"AF\x01\x01",4);actuator_write_u32_le(out+4,seq);actuator_write_u32_le(out+8,boot);
    actuator_write_u32_le(out+12,now);actuator_write_u32_le(out+16,observed);out[20]=s->flags;
    actuator_write_u32_le(out+60,actuator_crc32c(out,60));return true;
}
