#include "mobile_arm_observer.h"
#include "single_arm_config.h"
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD && HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD && HOST_BIMANUAL_DMA_DISPATCH_BUILD
#include "bimanual_tracking_feedback.h"
#include "bimanual_feedback_snapshot.h"
#include "bimanual_operational_limits.h"
#include "servo_transport.h"
static struct {
    UART_HandleTypeDef *left,*right;
    uint32_t period_ms,age_ms,resumed_ms,last_read_ms;
    uint16_t raw_left[6],raw_right[6];
    uint8_t joint;
    bool configured,owned,paused,fault,started;
} observer;

bool MobileArmObserver_Configure(UART_HandleTypeDef *left,UART_HandleTypeDef *right,
    uint32_t period,uint32_t age)
{
    if(observer.configured||!left||!right||left==right||!period||
       age>=UINT32_C(0x80000000)||(uint64_t)period*6>=age)return false;
    observer.left=left;observer.right=right;observer.period_ms=period;observer.age_ms=age;
    observer.resumed_ms=HAL_GetTick();observer.last_read_ms=HAL_GetTick()-period;
    observer.configured=true;observer.paused=true;return true;
}
bool MobileArmObserver_OwnsFeedback(void){return observer.owned;}
bool MobileArmObserver_HasStarted(void){return observer.started;}
bool MobileArmObserver_Suspend(void)
{
    if(!observer.configured)return true;
    observer.paused=true;
    if(observer.owned){
        BimanualTrackingFeedback_End();
        if(BimanualTrackingFeedback_Active()){observer.fault=true;return false;}
        observer.owned=false;
    }
    return !observer.fault;
}
void MobileArmObserver_Resume(void)
{
    if(!observer.configured||observer.fault)return;
    if(BimanualTrackingFeedback_Active()){observer.fault=true;return;}
    observer.paused=false;observer.started=true;observer.resumed_ms=HAL_GetTick();
}
static bool measured(uint8_t j,uint16_t raw,BimanualArm arm,int32_t *urad)
{
    int32_t unwrapped;
    const BimanualOperationalLimit *limit=BimanualOperationalLimits_Get(arm,j);
    return limit&&BimanualOperationalLimits_UnwrapModuloRaw(arm,j,raw,&unwrapped)&&
        actuator_unwrapped_raw_to_urad(limit->zero_raw,limit->positive_raw_direction,
            unwrapped,urad)==ACTUATOR_UNWRAP_OK&&
        *urad>=limit->urad.minimum_urad&&*urad<=limit->urad.maximum_urad;
}
bool MobileArmObserver_Poll(void)
{
    if(!observer.configured)return true;
    if(observer.fault)return false;
    if(observer.paused)return true;
    uint32_t now=HAL_GetTick();
    if(!observer.owned){
        /* Legitimate trajectory/STOP ownership requires Suspend first. */
        if(BimanualTrackingFeedback_Active()){observer.fault=true;return false;}
        if(ServoTransport_ReadReady(observer.left)&&ServoTransport_ReadReady(observer.right)){
            if(BimanualTrackingFeedback_Begin()!=HAL_OK){observer.fault=true;return false;}
            observer.owned=true;
        }
    }
    if(observer.owned){
        BimanualTrackingFeedbackSample sample;
        BimanualTrackingFeedbackResult result=BimanualTrackingFeedback_Poll(now,&sample);
        if(result==BIMANUAL_TRACKING_SAMPLE_READY){
            int32_t lp,rp;
            if(now-sample.observed_ms>=observer.age_ms||
               !measured(sample.joint_index,sample.left_position_raw,BIMANUAL_ARM_LEFT,&lp)||
               !measured(sample.joint_index,sample.right_position_raw,BIMANUAL_ARM_RIGHT,&rp)){
                BimanualFeedbackSnapshot_InvalidatePair(sample.joint_index);
                observer.fault=true;return false;
            }
            BimanualFeedbackSnapshot_UpdatePair(sample.joint_index,lp,rp,sample.observed_ms);
            observer.raw_left[sample.joint_index]=sample.left_position_raw;
            observer.raw_right[sample.joint_index]=sample.right_position_raw;
            observer.joint=(sample.joint_index+1u)%6u;
        }else if(result==BIMANUAL_TRACKING_TRANSIENT_FAILURE||result==BIMANUAL_TRACKING_FAULT){
            /* Keep the failed pair absent until a new successful READ, even
             * when the old sample has not reached its age limit yet. */
            BimanualFeedbackSnapshot_InvalidatePair(observer.joint);
            if(result==BIMANUAL_TRACKING_FAULT){observer.fault=true;return false;}
        }
        if(now-observer.last_read_ms>=observer.period_ms&&BimanualTrackingFeedback_CanStart()){
            observer.last_read_ms=now;
            if(BimanualTrackingFeedback_Start(observer.joint,now,observer.raw_left,
                    observer.raw_right,0,0)!=HAL_OK){observer.fault=true;return false;}
        }
    }
    if(now-observer.resumed_ms>=observer.age_ms){
        BimanualFeedbackSnapshot snapshot;BimanualFeedbackSnapshot_Copy(now,&snapshot);
        if(snapshot.present_mask!=BIMANUAL_FEEDBACK_COMPLETE_MASK){observer.fault=true;return false;}
        for(unsigned i=0;i<12;i++)if(snapshot.sample_age_ms[i]>=observer.age_ms){observer.fault=true;return false;}
    }
    return true;
}
#else
bool MobileArmObserver_Configure(UART_HandleTypeDef *l,UART_HandleTypeDef *r,uint32_t p,uint32_t a){(void)l;(void)r;(void)p;(void)a;return false;}
bool MobileArmObserver_Poll(void){return true;}
bool MobileArmObserver_OwnsFeedback(void){return false;}
bool MobileArmObserver_HasStarted(void){return false;}
bool MobileArmObserver_Suspend(void){return true;}
void MobileArmObserver_Resume(void){}
#endif
