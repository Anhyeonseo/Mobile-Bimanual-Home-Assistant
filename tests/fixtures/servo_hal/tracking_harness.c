#include "bimanual_tracking_feedback.h"
#include "servo_bus.h"
#include "right_servo_bus.h"
#include <stdio.h>
#if HOST_S3_STATIONARY_PICK
#include "s3_platform_snapshot.h"
#endif
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"%d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint8_t busy_left,busy_right,pending_left,pending_right,failed,failed_end,defer_left;
static ServoInMotionTelemetrySnapshot left;
static RightServoInMotionTelemetrySnapshot right;
HAL_StatusTypeDef Servo_InMotionTelemetryBegin(void){return HAL_OK;}
void Servo_InMotionTelemetryEnd(void){if(!failed_end)pending_left=0;}
HAL_StatusTypeDef RightServoBus_InMotionTelemetryBegin(void){return HAL_OK;}
void RightServoBus_InMotionTelemetryEnd(void){pending_right=0;}
uint8_t Servo_InMotionTelemetryReleased(void){return !pending_left;}
uint8_t RightServoBus_InMotionTelemetryReleased(void){return !pending_right;}
uint8_t Servo_InMotionTelemetryCanStart(void){return !busy_left&&!pending_left;}
uint8_t RightServoBus_InMotionTelemetryCanStart(void){return !busy_right&&!pending_right;}
uint8_t Servo_InMotionTelemetryPending(void){return pending_left;}
uint8_t RightServoBus_InMotionTelemetryPending(void){return pending_right;}
HAL_StatusTypeDef Servo_InMotionTelemetryStart(uint8_t j,uint32_t t){(void)t;if(defer_left)return HAL_BUSY;pending_left=1;left.last_joint_index=j;return HAL_OK;}
HAL_StatusTypeDef RightServoBus_InMotionTelemetryStart(uint8_t j,uint32_t t){(void)t;pending_right=1;right.last_joint_index=j;return HAL_OK;}
HAL_StatusTypeDef Servo_InMotionTelemetryPoll(uint32_t t,const uint16_t p[6]){(void)t;(void)p;pending_left=0;left.last_position_raw=123;return failed?HAL_TIMEOUT:HAL_OK;}
HAL_StatusTypeDef RightServoBus_InMotionTelemetryPoll(uint32_t t,const uint16_t p[6]){(void)p;if(t==10)return HAL_BUSY;pending_right=0;right.last_position_raw=456;return HAL_OK;}
const ServoInMotionTelemetrySnapshot *Servo_InMotionTelemetryGetSnapshot(void){return &left;}
const RightServoInMotionTelemetrySnapshot *RightServoBus_InMotionTelemetryGetSnapshot(void){return &right;}
int main(void){
 uint16_t pose[6]={0};BimanualTrackingFeedbackSample sample;
 CHECK(!BimanualTrackingFeedback_CanStart());CHECK(BimanualTrackingFeedback_Begin()==HAL_OK);
 defer_left=1;CHECK(BimanualTrackingFeedback_Start(2,9,pose,pose,0,0)==HAL_BUSY);
 CHECK(!pending_left&&!pending_right&&!BimanualTrackingFeedback_Pending());
 CHECK(BimanualTrackingFeedback_GetSnapshot()->requested_pairs==0);
 CHECK(BimanualTrackingFeedback_GetSnapshot()->failed_pairs==0);defer_left=0;
 busy_left=1;CHECK(!BimanualTrackingFeedback_CanStart());busy_left=0;busy_right=1;CHECK(!BimanualTrackingFeedback_CanStart());busy_right=0;
 CHECK(BimanualTrackingFeedback_CanStart());CHECK(BimanualTrackingFeedback_Start(2,9,pose,pose,0,0)==HAL_OK);
 CHECK(BimanualTrackingFeedback_Poll(10,&sample)==BIMANUAL_TRACKING_PENDING);
 CHECK(BimanualTrackingFeedback_Poll(11,&sample)==BIMANUAL_TRACKING_SAMPLE_READY);
 CHECK(sample.observed_ms==9&&sample.left_position_raw==123&&sample.right_position_raw==456);
#if HOST_S3_STATIONARY_PICK
 S3Platform_Enable(true);
 uint8_t out[44];S3Platform_Copy(20,out);CHECK(out[2]==0);
 for(unsigned cycle=0;cycle<4;cycle++){
  for(unsigned j=0;j<6;j++){
   CHECK(S3Platform_Select(j)==j);
   CHECK(BimanualTrackingFeedback_Start(j,20+j,pose,pose,0,0)==HAL_OK);
   CHECK(BimanualTrackingFeedback_Poll(21+j,&sample)==BIMANUAL_TRACKING_SAMPLE_READY);
  }
  unsigned aux=S3Platform_Select(0);CHECK(aux==6+cycle);
  uint32_t pairs=BimanualTrackingFeedback_GetSnapshot()->completed_pairs;
  CHECK(BimanualTrackingFeedback_Start(aux,30,pose,pose,0,0)==HAL_OK);
  CHECK(BimanualTrackingFeedback_Poll(31,&sample)==BIMANUAL_TRACKING_AUX_READY);
  CHECK(BimanualTrackingFeedback_GetSnapshot()->completed_pairs==pairs);
  CHECK(S3Platform_Select(0)==0);
 }
 S3Platform_Copy(35,out);CHECK(out[2]==15);CHECK(out[40]==5);
 CHECK(BimanualTrackingFeedback_Start(9,40,pose,pose,0,0)==HAL_OK);
 failed=1;CHECK(BimanualTrackingFeedback_Poll(41,&sample)==BIMANUAL_TRACKING_TRANSIENT_FAILURE);failed=0;
 S3Platform_Copy(42,out);CHECK(out[2]==15);CHECK(out[40]==12);
 CHECK(S3Platform_Select(0)==9); /* Original sample ages; failed auxiliary is retried. */
#endif
 failed=1;CHECK(BimanualTrackingFeedback_Start(3,9,pose,pose,0,0)==HAL_OK);
 CHECK(BimanualTrackingFeedback_Poll(10,&sample)==BIMANUAL_TRACKING_PENDING);
 CHECK(BimanualTrackingFeedback_Pending()&&!BimanualTrackingFeedback_CanStart());
 failed=0;CHECK(BimanualTrackingFeedback_Poll(11,&sample)==BIMANUAL_TRACKING_TRANSIENT_FAILURE);
 CHECK(BimanualTrackingFeedback_CanStart());CHECK(BimanualTrackingFeedback_GetSnapshot()->failed_pairs==(HOST_S3_STATIONARY_PICK?2:1));
 CHECK(BimanualTrackingFeedback_Start(4,12,pose,pose,0,0)==HAL_OK);
 CHECK(BimanualTrackingFeedback_Poll(13,&sample)==BIMANUAL_TRACKING_SAMPLE_READY);
 CHECK(BimanualTrackingFeedback_GetSnapshot()->consecutive_failed_pairs==0);
 CHECK(BimanualTrackingFeedback_Start(5,14,pose,pose,0,0)==HAL_OK);
 failed_end=1;BimanualTrackingFeedback_End();
 CHECK(BimanualTrackingFeedback_Active()&&BimanualTrackingFeedback_Pending());
 CHECK(BimanualTrackingFeedback_Begin()==HAL_BUSY);
 failed_end=0;BimanualTrackingFeedback_End();
 CHECK(!BimanualTrackingFeedback_Active()&&!BimanualTrackingFeedback_Pending());return 0;
}
