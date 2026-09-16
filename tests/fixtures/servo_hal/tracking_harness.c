#include "bimanual_tracking_feedback.h"
#include "servo_bus.h"
#include "right_servo_bus.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"%d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint8_t busy_left,busy_right,pending_left,pending_right,failed,failed_end;
static ServoInMotionTelemetrySnapshot left;
static RightServoInMotionTelemetrySnapshot right;
void Servo_InMotionTelemetryBegin(void){}
void Servo_InMotionTelemetryEnd(void){if(!failed_end)pending_left=0;}
HAL_StatusTypeDef RightServoBus_InMotionTelemetryBegin(void){return HAL_OK;}
void RightServoBus_InMotionTelemetryEnd(void){pending_right=0;}
uint8_t Servo_InMotionTelemetryReleased(void){return !pending_left;}
uint8_t RightServoBus_InMotionTelemetryReleased(void){return !pending_right;}
uint8_t Servo_InMotionTelemetryCanStart(void){return !busy_left&&!pending_left;}
uint8_t RightServoBus_InMotionTelemetryCanStart(void){return !busy_right&&!pending_right;}
uint8_t Servo_InMotionTelemetryPending(void){return pending_left;}
uint8_t RightServoBus_InMotionTelemetryPending(void){return pending_right;}
HAL_StatusTypeDef Servo_InMotionTelemetryStart(uint8_t j,uint32_t t){(void)t;pending_left=1;left.last_joint_index=j;return HAL_OK;}
HAL_StatusTypeDef RightServoBus_InMotionTelemetryStart(uint8_t j,uint32_t t){(void)t;pending_right=1;right.last_joint_index=j;return HAL_OK;}
HAL_StatusTypeDef Servo_InMotionTelemetryPoll(uint32_t t,const uint16_t p[6]){(void)t;(void)p;pending_left=0;left.last_position_raw=123;return failed?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef RightServoBus_InMotionTelemetryPoll(uint32_t t,const uint16_t p[6]){(void)p;if(t==10)return HAL_BUSY;pending_right=0;right.last_position_raw=456;return HAL_OK;}
const ServoInMotionTelemetrySnapshot *Servo_InMotionTelemetryGetSnapshot(void){return &left;}
const RightServoInMotionTelemetrySnapshot *RightServoBus_InMotionTelemetryGetSnapshot(void){return &right;}
int main(void){
 uint16_t pose[6]={0};BimanualTrackingFeedbackSample sample;
 CHECK(!BimanualTrackingFeedback_CanStart());CHECK(BimanualTrackingFeedback_Begin()==HAL_OK);
 busy_left=1;CHECK(!BimanualTrackingFeedback_CanStart());busy_left=0;busy_right=1;CHECK(!BimanualTrackingFeedback_CanStart());busy_right=0;
 CHECK(BimanualTrackingFeedback_CanStart());CHECK(BimanualTrackingFeedback_Start(2,9,pose,pose,0,0)==HAL_OK);
 CHECK(BimanualTrackingFeedback_Poll(10,&sample)==BIMANUAL_TRACKING_PENDING);
 CHECK(BimanualTrackingFeedback_Poll(11,&sample)==BIMANUAL_TRACKING_SAMPLE_READY);
 CHECK(sample.observed_ms==9&&sample.left_position_raw==123&&sample.right_position_raw==456);
 failed=1;CHECK(BimanualTrackingFeedback_Start(3,9,pose,pose,0,0)==HAL_OK);
 CHECK(BimanualTrackingFeedback_Poll(10,&sample)==BIMANUAL_TRACKING_PENDING);
 CHECK(BimanualTrackingFeedback_Pending()&&!BimanualTrackingFeedback_CanStart());
 failed=0;CHECK(BimanualTrackingFeedback_Poll(11,&sample)==BIMANUAL_TRACKING_TRANSIENT_FAILURE);
 CHECK(BimanualTrackingFeedback_CanStart());CHECK(BimanualTrackingFeedback_GetSnapshot()->failed_pairs==1);
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
