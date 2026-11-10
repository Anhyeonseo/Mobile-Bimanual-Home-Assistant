#include "bimanual_tracking_feedback.h"

#include "right_servo_bus.h"
#include "servo_bus.h"
#include "single_arm_config.h"
#if HOST_S3_STATIONARY_PICK
#include "s3_platform_snapshot.h"
#endif

#include <stddef.h>
#include <string.h>

typedef struct
{
    BimanualTrackingFeedbackSnapshot snapshot;
    uint8_t joint_index;
    uint32_t started_at_ms;
    uint8_t pair_failed;
    uint8_t faulted;
    uint16_t left_commanded_raw[BIMANUAL_TRACKING_ARM_JOINT_COUNT];
    uint16_t right_commanded_raw[BIMANUAL_TRACKING_ARM_JOINT_COUNT];
    int32_t left_commanded_urad;
    int32_t right_commanded_urad;
} BimanualTrackingFeedbackState;

static BimanualTrackingFeedbackState tracking = {0};

static BimanualTrackingFeedbackResult RecordPairFailure(void)
{
    #if HOST_S3_STATIONARY_PICK
    S3Platform_Retry(tracking.joint_index);
#endif
    tracking.snapshot.failed_pairs++;
    tracking.snapshot.pending = 0U;
    if (tracking.snapshot.consecutive_failed_pairs < UINT8_MAX)
    {
        tracking.snapshot.consecutive_failed_pairs++;
    }
    if (tracking.snapshot.consecutive_failed_pairs >=
        HOST_BIMANUAL_TRACKING_READ_FAILURE_LIMIT)
    {
        tracking.faulted = 1U;
        return BIMANUAL_TRACKING_FAULT;
    }
    return BIMANUAL_TRACKING_TRANSIENT_FAILURE;
}

HAL_StatusTypeDef BimanualTrackingFeedback_Begin(void)
{
    if (tracking.snapshot.active != 0U)
    {
        return HAL_BUSY;
    }
    memset(&tracking, 0, sizeof(tracking));
#if HOST_S3_STATIONARY_PICK
    S3Platform_CancelRetry();
#endif
#if HOST_S3_STATIONARY_PICK
    /* Own the lifecycle before setup. A partially acquired UART must remain
     * visible until End proves cleanup; never report an idle owner on failure. */
    tracking.snapshot.active = 1U;
    if (Servo_InMotionTelemetryBegin() != HAL_OK ||
        RightServoBus_InMotionTelemetryBegin() != HAL_OK) {
        tracking.faulted = 1U;
        BimanualTrackingFeedback_End();
        return HAL_ERROR;
    }
#else
    if (Servo_InMotionTelemetryBegin() != HAL_OK) return HAL_ERROR;
    if (RightServoBus_InMotionTelemetryBegin() != HAL_OK)
    {
        Servo_InMotionTelemetryEnd();
        return HAL_ERROR;
    }
#endif
    tracking.snapshot.active = 1U;
    return HAL_OK;
}

void BimanualTrackingFeedback_End(void)
{
    Servo_InMotionTelemetryEnd();
    RightServoBus_InMotionTelemetryEnd();
    if (Servo_InMotionTelemetryReleased() && RightServoBus_InMotionTelemetryReleased())
    {
        tracking.snapshot.active = 0U;
#if HOST_S3_STATIONARY_PICK
        S3Platform_CancelRetry();
#endif
        tracking.snapshot.pending = 0U;
    }
}

uint8_t BimanualTrackingFeedback_Active(void)
{
    return tracking.snapshot.active;
}

uint8_t BimanualTrackingFeedback_CanStart(void)
{
    return tracking.snapshot.active && !tracking.faulted && !tracking.snapshot.pending &&
        Servo_InMotionTelemetryCanStart() && RightServoBus_InMotionTelemetryCanStart();
}

uint8_t BimanualTrackingFeedback_Pending(void)
{
    return tracking.snapshot.pending;
}

HAL_StatusTypeDef BimanualTrackingFeedback_Start(
    uint8_t joint_index,
    uint32_t started_at_ms,
    const uint16_t left_commanded_raw[BIMANUAL_TRACKING_ARM_JOINT_COUNT],
    const uint16_t right_commanded_raw[BIMANUAL_TRACKING_ARM_JOINT_COUNT],
    int32_t left_commanded_urad,
    int32_t right_commanded_urad)
{
    if ((tracking.snapshot.active == 0U) ||
        (tracking.snapshot.pending != 0U) || (tracking.faulted != 0U) ||
        (joint_index >= BIMANUAL_TRACKING_ARM_JOINT_COUNT
#if HOST_S3_STATIONARY_PICK
         && joint_index >= 10U
#endif
        ) ||
        (left_commanded_raw == NULL) || (right_commanded_raw == NULL))
    {
        return HAL_BUSY;
    }
    tracking.pair_failed = 0U;
    tracking.joint_index = joint_index;
    tracking.started_at_ms = started_at_ms;
    memcpy(tracking.left_commanded_raw, left_commanded_raw,
           sizeof(tracking.left_commanded_raw));
    memcpy(tracking.right_commanded_raw, right_commanded_raw,
           sizeof(tracking.right_commanded_raw));
    tracking.left_commanded_urad = left_commanded_urad;
    tracking.right_commanded_urad = right_commanded_urad;
    HAL_StatusTypeDef left_started = Servo_InMotionTelemetryStart(joint_index, started_at_ms);
    if (left_started == HAL_BUSY) return HAL_BUSY; /* No frame sent, no pair owns RX. */
    if (left_started != HAL_OK)
    {
        tracking.snapshot.failed_pairs++;
        tracking.faulted = 1U;
        return HAL_ERROR;
    }
    if (RightServoBus_InMotionTelemetryStart(
            joint_index % BIMANUAL_TRACKING_ARM_JOINT_COUNT, started_at_ms) != HAL_OK)
    {
        tracking.snapshot.failed_pairs++;
        tracking.faulted = 1U;
#if HOST_S3_STATIONARY_PICK
        /* A left request is already on the wire. Return the fatal cause now;
         * the coordinated STOP owns abort/drain, outside the 5 ms output loop. */
        tracking.snapshot.pending = Servo_InMotionTelemetryPending() ||
            RightServoBus_InMotionTelemetryPending();
#else
        BimanualTrackingFeedback_End();
#endif
        return HAL_ERROR;
    }
    #if HOST_S3_STATIONARY_PICK
    S3Platform_Scheduled(joint_index);
    if(joint_index < 6U)
#endif
    tracking.snapshot.requested_pairs++;
    tracking.snapshot.pending = 1U;
    return HAL_OK;
}

BimanualTrackingFeedbackResult BimanualTrackingFeedback_Poll(
    uint32_t now_ms,
    BimanualTrackingFeedbackSample *sample)
{
    HAL_StatusTypeDef left_status;
    HAL_StatusTypeDef right_status;
    const ServoInMotionTelemetrySnapshot *left;
    const RightServoInMotionTelemetrySnapshot *right;

    if (tracking.snapshot.active && tracking.faulted) return BIMANUAL_TRACKING_FAULT;
    if ((tracking.snapshot.active == 0U) ||
        (tracking.snapshot.pending == 0U))
    {
        return BIMANUAL_TRACKING_IDLE;
    }
    left_status = Servo_InMotionTelemetryPoll(
        now_ms, tracking.left_commanded_raw);
    right_status = RightServoBus_InMotionTelemetryPoll(
        now_ms, tracking.right_commanded_raw);
    if ((left_status == HAL_ERROR) || (left_status == HAL_TIMEOUT) ||
        (right_status == HAL_ERROR) || (right_status == HAL_TIMEOUT))
    {
        tracking.pair_failed = 1U;
    }
#if HOST_S3_STATIONARY_PICK
    if (left_status == HAL_ERROR || right_status == HAL_ERROR) {
        /* A broken receiver/lease cannot recover in the output loop. */
        S3Platform_Invalidate(tracking.joint_index);
        tracking.faulted = 1U;
        tracking.snapshot.failed_pairs++;
        return BIMANUAL_TRACKING_FAULT;
    }
#endif
    /* One failed response cannot orphan the other UART's pending read lease.
     * Drain/timeout both sides before accounting a recoverable pair failure. */
    if ((Servo_InMotionTelemetryPending() != 0U) ||
        (RightServoBus_InMotionTelemetryPending() != 0U))
    {
        return BIMANUAL_TRACKING_PENDING;
    }
    if (tracking.pair_failed) return RecordPairFailure();
    left = Servo_InMotionTelemetryGetSnapshot();
    right = RightServoBus_InMotionTelemetryGetSnapshot();
    if ((left == NULL) || (right == NULL) ||
        (left->last_joint_index != tracking.joint_index) ||
        (right->last_joint_index != tracking.joint_index % BIMANUAL_TRACKING_ARM_JOINT_COUNT) ||
        (sample == NULL))
    {
        return RecordPairFailure();
    }
    #if HOST_S3_STATIONARY_PICK
    if(tracking.joint_index >= 6U){
        S3Platform_Update(tracking.joint_index,left->last_position_raw,tracking.started_at_ms);
        tracking.snapshot.pending=0U;
        tracking.snapshot.consecutive_failed_pairs=0U;
        return BIMANUAL_TRACKING_AUX_READY;
    }
#endif
    sample->joint_index = tracking.joint_index;
    sample->observed_ms = tracking.started_at_ms;
    sample->left_position_raw = left->last_position_raw;
    sample->right_position_raw = right->last_position_raw;
    sample->left_commanded_urad = tracking.left_commanded_urad;
    sample->right_commanded_urad = tracking.right_commanded_urad;
    tracking.snapshot.completed_pairs++;
    tracking.snapshot.consecutive_failed_pairs = 0U;
    tracking.snapshot.pending = 0U;
    {
        const uint32_t latency = now_ms - tracking.started_at_ms;
        if (latency > tracking.snapshot.maximum_reply_latency_ms)
        {
            tracking.snapshot.maximum_reply_latency_ms = latency;
        }
    }
    return BIMANUAL_TRACKING_SAMPLE_READY;
}

const BimanualTrackingFeedbackSnapshot *
BimanualTrackingFeedback_GetSnapshot(void)
{
    return &tracking.snapshot;
}
