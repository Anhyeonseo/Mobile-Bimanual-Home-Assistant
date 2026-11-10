#include "s3_platform_snapshot.h"
#include <string.h>
static bool retry_pending;
static uint8_t retry_slot;
static struct { bool enabled,due; uint8_t next,mask; uint16_t raw[4]; uint32_t stamp[4],count; } s;
static void put32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
void S3Platform_Enable(bool enabled){
    if(enabled && !s.enabled && s.mask==15U){
        /* A handoff preserves measurements, but may cancel the read whose
         * scheduling already advanced next. Refresh the oldest actual sample
         * before another six-arm sweep. Never refresh a timestamp here. */
        uint8_t oldest=0;
        for(uint8_t j=1;j<4;j++)
            if((int32_t)(s.stamp[j]-s.stamp[oldest])<0)oldest=j;
        s.next=oldest;s.due=true;
    }
    s.enabled=enabled;
}
uint8_t S3Platform_Select(uint8_t j){return retry_pending?retry_slot:s.enabled&&s.due&&j==0?(uint8_t)(6+s.next):j;}
void S3Platform_Scheduled(uint8_t j){if(retry_pending && j==retry_slot){retry_pending=false;return;}if(j==5)s.due=true;else if(j>=6&&j<10){s.due=false;s.next=(s.next+1)%4;}}
void S3Platform_Update(uint8_t j,uint16_t raw,uint32_t stamp){
    if(j<6||j>=10)return;
    j-=6;s.raw[j]=raw;s.stamp[j]=stamp;s.mask|=(1u<<j);s.count++;
}
void S3Platform_CancelRetry(void){retry_pending=false;}
void S3Platform_Retry(uint8_t j){if(j<10){retry_slot=j;retry_pending=true;}}
void S3Platform_Invalidate(uint8_t j){if(j>=6&&j<10)s.mask&=~(1u<<(j-6));}
void S3Platform_Copy(uint32_t now,uint8_t out[44]){
    memset(out,0,44);out[0]=1;out[1]=s.enabled;out[2]=s.mask;
    put32(out+4,now);put32(out+8,s.count);
    for(unsigned j=0;j<4;j++){put32(out+12+4*j,s.raw[j]);put32(out+28+4*j,(s.mask&(1u<<j))?now-s.stamp[j]:UINT32_MAX);}
}
