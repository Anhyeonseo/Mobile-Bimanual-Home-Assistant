#include "bimanual_tracking_feedback.h"
#include "bimanual_feedback_snapshot.h"
#include "servo_bus.h"
#include <assert.h>
static unsigned pending,fail_read,fail_end,wait_read;
static ServoInMotionTelemetrySnapshot measured;
HAL_StatusTypeDef Servo_InMotionTelemetryBegin(void){return HAL_OK;}
void Servo_InMotionTelemetryEnd(void){if(!fail_end)pending=0;}
uint8_t Servo_InMotionTelemetryReleased(void){return !pending;}
uint8_t Servo_InMotionTelemetryCanStart(void){return !pending;}
uint8_t Servo_InMotionTelemetryPending(void){return pending;}
HAL_StatusTypeDef Servo_InMotionTelemetryStart(uint8_t j,uint32_t t){(void)t;pending=1;measured.last_joint_index=j;return HAL_OK;}
HAL_StatusTypeDef Servo_InMotionTelemetryPoll(uint32_t t,const uint16_t p[6]){(void)t;(void)p;if(wait_read)return HAL_BUSY;pending=0;measured.last_position_raw=2100;return fail_read?HAL_TIMEOUT:HAL_OK;}
const ServoInMotionTelemetrySnapshot *Servo_InMotionTelemetryGetSnapshot(void){return &measured;}
/* No right servo functions are linked: any accidental reference fails build. */
int main(void){
 uint16_t raw[6]={0};BimanualTrackingFeedbackSample sample;BimanualFeedbackSnapshot feedback;
 int32_t anchor[12]={100,200,300,400,500,600,0,0,0,0,0,0};
 BimanualFeedbackSnapshot_Seed(anchor,10);BimanualFeedbackSnapshot_Copy(20,&feedback);
 assert(feedback.present_mask==0x3f);
 for(unsigned i=6;i<12;i++)assert(feedback.positions_urad[i]==0&&feedback.sample_age_ms[i]==UINT32_MAX);
 assert(BimanualTrackingFeedback_Begin()==HAL_OK);
 assert(BimanualTrackingFeedback_Start(2,20,raw,raw,300,0)==HAL_OK);
 wait_read=1;assert(BimanualTrackingFeedback_Poll(21,&sample)==BIMANUAL_TRACKING_PENDING);
 wait_read=0;assert(BimanualTrackingFeedback_Poll(22,&sample)==BIMANUAL_TRACKING_SAMPLE_READY);
 assert(sample.observed_ms==20&&sample.left_position_raw==2100&&sample.right_position_raw==0);
 BimanualFeedbackSnapshot_UpdatePair(2,333,999,20);BimanualFeedbackSnapshot_Copy(25,&feedback);
 assert(feedback.positions_urad[2]==333&&feedback.positions_urad[8]==0&&feedback.present_mask==0x3f);
 fail_read=1;
 for(int i=0;i<3;i++){
  assert(BimanualTrackingFeedback_Start(i,30,raw,raw,0,0)==HAL_OK);
  assert(BimanualTrackingFeedback_Poll(31,&sample)==(i==2?BIMANUAL_TRACKING_FAULT:BIMANUAL_TRACKING_TRANSIENT_FAILURE));
 }
 fail_read=0;assert(BimanualTrackingFeedback_Start(4,40,raw,raw,0,0)==HAL_OK);
 fail_end=1;BimanualTrackingFeedback_End();assert(BimanualTrackingFeedback_Active());
 fail_end=0;BimanualTrackingFeedback_End();assert(!BimanualTrackingFeedback_Active());
 BimanualFeedbackSnapshot_InvalidatePair(1);BimanualFeedbackSnapshot_Copy(40,&feedback);
 assert(feedback.present_mask==0x3d&&feedback.sample_age_ms[1]==UINT32_MAX);return 0;
}
