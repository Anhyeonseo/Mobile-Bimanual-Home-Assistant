#include "actuator_core/arm_hold_monitor.h"
#include <string.h>
static bool newer(uint32_t a,uint32_t b){return a-b && a-b<UINT32_C(0x80000000);}
static uint16_t distance(uint16_t a,uint16_t b){uint16_t d=a>b?a-b:b-a;return d>2048?4096-d:d;}
bool actuator_arm_hold_monitor_init(actuator_arm_hold_monitor_t *m,
    const uint16_t l[6],const uint16_t r[6],uint32_t after,uint16_t tol,uint32_t dwell,uint32_t age){
    if(!m||!l||!r||!tol||tol>=2048||!dwell||dwell>=UINT32_C(0x80000000)||!age||age>=UINT32_C(0x80000000))return false;
    for(unsigned i=0;i<6;i++)if(l[i]>=4096||r[i]>=4096)return false;
    memset(m,0,sizeof(*m));memcpy(m->target,l,12);memcpy(m->target+6,r,12);
    m->after_ms=after;m->tolerance_raw=tol;m->dwell_ms=dwell;m->max_age_ms=age;m->configured=true;return true;
}
bool actuator_arm_hold_monitor_observe(actuator_arm_hold_monitor_t *m,
    uint8_t j,uint16_t l,uint16_t r,uint32_t stamp,uint32_t now){
    if(!m||!m->configured||j>=6)return false;
    uint16_t bit=(uint16_t)(1u<<j);
    if(l>=4096||r>=4096||now-stamp>=m->max_age_ms||!newer(stamp,m->after_ms)||
       ((m->seen&bit)&&!newer(stamp,m->stamp[j]))){m->stable&=(uint8_t)~bit;return false;}
    bool continuous=(m->seen&bit)&&stamp-m->stamp[j]<m->max_age_ms;
    m->seen|=bit;m->stamp[j]=stamp;
    if(distance(l,m->target[j])>m->tolerance_raw||distance(r,m->target[j+6])>m->tolerance_raw){m->stable&=(uint8_t)~bit;return false;}
    if(!continuous||!(m->stable&bit)){m->stable_since[j]=stamp;m->stable|=(uint8_t)bit;}
    return true;
}
bool actuator_arm_hold_monitor_proof(const actuator_arm_hold_monitor_t *m,uint32_t now,uint32_t *oldest){
    if(!m||!m->configured||!oldest||m->stable!=63||m->seen!=63)return false;
    uint32_t age=0;
    for(unsigned j=0;j<6;j++){
        uint32_t a=now-m->stamp[j];
        if(a>=m->max_age_ms||m->stamp[j]-m->stable_since[j]<m->dwell_ms)return false;
        if(a>age)age=a;
    }
    *oldest=now-age;return true;
}
