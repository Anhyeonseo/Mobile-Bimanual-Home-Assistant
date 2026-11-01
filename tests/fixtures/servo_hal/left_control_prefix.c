#include <assert.h>
#include <string.h>
#include "single_arm_config.h"
#include "actuator_core/protocol.h"
#include "actuator_core/calibration.h"
#include "actuator_core/joint_unwrap.h"
#include "actuator_core/stream_executor_v2.h"
#include "bimanual_operational_limits.h"
#include "bimanual_feedback_snapshot.h"
#include "bimanual_tracking_feedback.h"
#include "bimanual_servo_dispatch.h"
static actuator_v2_stream_executor_t host_v2_stream_executor;
static uint8_t host_v2_executor_clock_active,host_v2_shadow_anchor_ready,host_binary_servos_configured,host_right_arm_output_active;
static uint16_t host_v2_shadow_raw[12],host_v2_last_left_raw[6],host_v2_last_right_raw[6];
static int32_t host_v2_shadow_anchor_urad[12],host_v2_shadow_executor_anchor_urad[12],host_v2_shadow_unwrapped_raw[12],host_v2_output_urad[12];
static actuator_joint_unwrapper_t host_v2_shadow_unwrappers[12];
static uint32_t host_v2_terminal_settle_baseline_completed_pairs,host_v2_terminal_settle_started_ms,host_v2_tracking_last_dispatch_completed;
static uint8_t host_v2_terminal_settle_active,host_v2_tracking_next_joint;
static int host_binary_safety,stop_count,fault_count,disable_fails,read_fails;
static uint16_t physical[6]={2048,2048,2048,2048,2048,2048};
static uint32_t now=100;
static actuator_frame_t response;
static BimanualTrackingFeedbackSnapshot tracking;
static actuator_bimanual_dispatch_snapshot_t dispatch;
static BimanualTrackingFeedbackResult next_result;
static BimanualTrackingFeedbackSample next_sample;
uint32_t HAL_GetTick(void){return now;}
static void Host_ResetPositionReadFailure(void){}
static void BinaryControl_LatchStop(void){stop_count++;}
static void Host_RequestV2CoordinatedStop(void){stop_count++;}
static void actuator_safety_report_fault(int *state,uint16_t reason){(void)state;(void)reason;fault_count++;}
static uint16_t Host_ReadU16Le(const uint8_t *p){uint16_t v;memcpy(&v,p,2);return v;}
static int32_t Host_ReadI32Le(const uint8_t *p){int32_t v;memcpy(&v,p,4);return v;}
static void Host_WriteU16Le(uint8_t *p,uint16_t v){memcpy(p,&v,2);}
static void Host_WriteU32Le(uint8_t *p,uint32_t v){memcpy(p,&v,4);}
static int Host_SendBinaryFrame(const actuator_frame_t *frame){response=*frame;return 0;}
static HAL_StatusTypeDef Servo_DisableTorqueAll(void){return disable_fails?HAL_ERROR:HAL_OK;}
static HAL_StatusTypeDef Servo_ReadAllPositions(uint16_t *p){memcpy(p,physical,sizeof(physical));return read_fails?HAL_ERROR:HAL_OK;}
static actuator_joint_calibration_t Host_JointCalibration(uint8_t j){
 const BimanualOperationalLimit *limit=BimanualOperationalLimits_Get(BIMANUAL_ARM_LEFT,j);
 return (actuator_joint_calibration_t){limit->zero_raw,0,4095,limit->positive_raw_direction};
}
static uint8_t Host_V2TrackingRawToUrad(uint8_t j,uint16_t raw,int32_t *q){assert(j<6);*q=raw;return 1;}
static uint32_t Host_V2TrackingCompletedPairs(void){return tracking.completed_pairs;}
static int MobileBoard_StopActive(void){return 0;}
static int MobileArmObserver_OwnsFeedback(void){return 0;}
static void MobileArmObserver_Resume(void){}
uint8_t BimanualTrackingFeedback_Active(void){return tracking.active;}
uint8_t BimanualTrackingFeedback_CanStart(void){return 0;}
uint8_t BimanualTrackingFeedback_Pending(void){return tracking.pending;}
void BimanualTrackingFeedback_End(void){tracking.active=0;}
BimanualTrackingFeedbackResult BimanualTrackingFeedback_Poll(uint32_t t,BimanualTrackingFeedbackSample *s){(void)t;*s=next_sample;return next_result;}
HAL_StatusTypeDef BimanualTrackingFeedback_Start(uint8_t j,uint32_t t,const uint16_t l[6],const uint16_t r[6],int32_t ql,int32_t qr){(void)j;(void)t;(void)l;(void)r;(void)ql;(void)qr;assert(0);return HAL_ERROR;}
const actuator_bimanual_dispatch_snapshot_t *BimanualServoDispatch_GetSnapshot(void){return &dispatch;}
actuator_v2_executor_result_t actuator_v2_stream_executor_check_joint_feedback(actuator_v2_stream_executor_t *e,uint32_t t,uint8_t j,int32_t expected,int32_t actual){(void)e;(void)t;(void)expected;(void)actual;assert(j<6);return ACTUATOR_V2_EXECUTOR_OK;}
