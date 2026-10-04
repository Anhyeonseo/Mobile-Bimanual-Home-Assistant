#include "mobile_wheel_pulse.h"
#include "mobile_bench_patterns.h"
#include "servo_bus.h"
#include "actuator_core/mobile_ids.h"
#include <string.h>

static bool consumed;
bool MobileWheelPulse_Consumed(void) { return consumed; }
static int read_reg(uint8_t id, uint8_t address, uint8_t size) {
    uint8_t value[2] = {0};
    if (Servo_ReadData(id, address, size, value) != HAL_OK) return -1;
    return value[0] | (value[1] << 8);
}
static bool write_checked(uint8_t id, uint8_t address, const uint8_t *data, uint8_t size) {
    uint8_t value[2] = {0};
    if (Servo_WriteData(id, address, data, size) != HAL_OK) return false;
    return Servo_ReadData(id, address, size, value) == HAL_OK && !memcmp(value, data, size);
}
static bool preflight(uint8_t id, MobileWheelPulseResult *out) {
    /* Validate one motor. Single-wheel callers invoke this only for their
     * selected ID; a group caller validates each participant before any write.
     * These raised-wheel checks do not authorize driving on the ground. */
    {
        uint8_t wheel=id, identity[4];
        out->failed_id=wheel; out->stage="IDENTITY";
        if (Servo_ReadData(wheel,3,4,identity)!=HAL_OK || identity[0]!=9 ||
            identity[1]!=3 || identity[2]!=wheel || identity[3]!=0) return false;
        out->stage="MODE_MUST_BE_ONE";
        if (read_reg(wheel,33,1)!=1) return false;
        out->stage="TORQUE_MUST_BE_OFF";
        if (read_reg(wheel,40,1)!=0) return false;
        out->stage="EEPROM_MUST_BE_LOCKED";
        if (read_reg(wheel,55,1)!=1) return false;
        uint8_t observed_speed[2]={0};
        out->speed_check_hal_status=Servo_ReadData(wheel,58,2,observed_speed);
        out->stage="SPEED_READ_FAILED";
        if (out->speed_check_hal_status!=HAL_OK) return false;
        out->speed_check_raw=observed_speed[0] | (observed_speed[1]<<8);
        out->stage="WHEEL_MUST_BE_STOPPED";
        if (out->speed_check_raw!=0) return false;
    }
    out->failed_id=id; out->stage="POSITION_READ";
    out->position_before=read_reg(id,56,2);
    if (out->position_before<0) return false;
    return true;
}
void MobileWheelPulse_Run(uint8_t id, int direction, MobileWheelPulseResult *out) {
    if (!out) return;
    *out = (MobileWheelPulseResult){.id=id, .stage="INVALID_REQUEST", .speed_raw=-1,
        .position_before=-1, .position_after=-1, .goal_after=-1, .torque_after=-1,
        .speed_after=-1, .speed_check_raw=-1, .speed_check_hal_status=-1};
    if (id < ACTUATOR_WHEEL_0_ID || id > ACTUATOR_WHEEL_2_ID ||
        (direction != 1 && direction != -1)) return;
    if (consumed) {out->stage="ALREADY_ATTEMPTED"; return;}
    consumed = true;
    if (!preflight(id,out)) return;
    const uint8_t zero[2]={0,0}, one=1;
    const uint8_t speed[2]={200, direction>0 ? 0 : 128};
    out->speed_raw=direction*200;
    bool pulse_ok=false;
    out->stage="ZERO_BEFORE_ENABLE";
    if (!write_checked(id,46,zero,2)) goto cleanup;
    out->stage="TORQUE_ENABLE";
    out->motion_attempted=true; /* an uncertain enable must also be cleaned up */
    if (!write_checked(id,40,&one,1)) goto cleanup;
    out->stage="SPEED_WRITE";
    uint32_t start=HAL_GetTick();
    if (!write_checked(id,46,speed,2)) {
        out->stop_requested_ms=(uint32_t)(HAL_GetTick()-start);
        goto cleanup;
    }
    /* No host reads/writes in this window. Host disconnect cannot extend it.
     * This is NOT an independent emergency stop: MCU/bus/power failure can
     * prevent stopping. Motor supply cut-off must remain accessible. */
    while ((uint32_t)(HAL_GetTick()-start)<3000u) HAL_Delay(1);
    out->stop_requested_ms=(uint32_t)(HAL_GetTick()-start);
    pulse_ok=true;
cleanup:
    out->cleanup_attempted=true;
    /* Attempt BOTH writes even when the first fails, before any readback. */
    bool zero_sent=Servo_WriteData(id,46,zero,2)==HAL_OK;
    bool off_sent=Servo_WriteData(id,40,zero,1)==HAL_OK;
    HAL_Delay(100); /* coast/feedback settling, still wheels raised */
    out->goal_after=read_reg(id,46,2);
    out->torque_after=read_reg(id,40,1);
    out->speed_after=read_reg(id,58,2);
    out->position_after=read_reg(id,56,2);
    out->stop_confirmed=zero_sent && off_sent && out->goal_after==0 &&
        out->torque_after==0 && out->speed_after==0;
    out->ok=pulse_ok && out->stop_confirmed && out->position_after>=0;
    if (!out->stop_confirmed) out->stage="STOP_UNCONFIRMED_CUT_MOTOR_POWER";
    else if (out->ok) {out->stage="PULSE_COMPLETE_TORQUE_OFF"; out->failed_id=0;}
    else if (pulse_ok) out->stage="FINAL_POSITION_READ";
}

/* +raw = clockwise viewed from outside (operator observation).
 * Nominal radial mounting: left 120deg, right 240deg, front 0deg;
 * positive rolling tangent is (-sin(theta), cos(theta)), yaw CCW positive.
 * Order: 7 left, 8 right, 9 front. Ratios only; no measured m/s claim. */
void MobileWheelPulse_RunBase(char motion, MobileBasePulseResult *out) {
    if (!out) return;
    *out=(MobileBasePulseResult){.motion=motion,.stage="INVALID_REQUEST"};
    int speeds[3]={0};
    for (unsigned i=0;i<3;++i) out->wheels[i]=(MobileWheelPulseResult){
        .id=actuator_mobile_axis_id(i),.stage="NOT_ATTEMPTED",.speed_raw=0,
        .position_before=-1,.position_after=-1,.goal_after=-1,.torque_after=-1,
        .speed_after=-1,.speed_check_raw=-1,.speed_check_hal_status=-1};
    if (!MobileBench_Pattern(motion,speeds)) return;
    for (unsigned i=0;i<3;++i) out->wheels[i].speed_raw=speeds[i];
    if (consumed) {out->stage="ALREADY_ATTEMPTED"; return;}
    consumed=true;
    /* In this group test all three participate, including a held zero-speed
     * front wheel during forward motion. Single-wheel tests remain isolated. */
    for (unsigned i=0;i<3;++i) if (!preflight(out->wheels[i].id,&out->wheels[i])) {
        out->stage=out->wheels[i].stage;out->failed_id=out->wheels[i].failed_id;return;
    }
    const uint8_t zero[2]={0,0},one=1;
    bool pulse_ok=false,zero_sent[3],off_sent[3];
    out->stage="ZERO_BEFORE_ENABLE";
    for (unsigned i=0;i<3;++i) {
        out->failed_id=out->wheels[i].id;
        if (!write_checked(out->failed_id,46,zero,2)) goto cleanup;
    }
    out->stage="TORQUE_ENABLE";
    for (unsigned i=0;i<3;++i) {
        out->failed_id=out->wheels[i].id;
        if (!write_checked(out->failed_id,40,&one,1)) goto cleanup;
    }
    out->stage="SPEED_WRITE";
    uint32_t start=HAL_GetTick();
    for (unsigned i=0;i<3;++i) {
        out->failed_id=out->wheels[i].id;
        uint16_t encoded=speeds[i]<0 ? (uint16_t)(-speeds[i])|0x8000u : (uint16_t)speeds[i];
        uint8_t data[2]={(uint8_t)encoded,(uint8_t)(encoded>>8)};
        if ((uint32_t)(HAL_GetTick()-start)>=3000u ||
            !write_checked(out->failed_id,46,data,2)) {
            out->stop_requested_ms=(uint32_t)(HAL_GetTick()-start);
            goto cleanup;
        }
    }
    while ((uint32_t)(HAL_GetTick()-start)<3000u) HAL_Delay(1);
    out->stop_requested_ms=(uint32_t)(HAL_GetTick()-start);
    pulse_ok=true;
cleanup:
    out->cleanup_attempted=true;
    /* Issue stop to ALL participants before waiting on any readback.
     * An error on one wheel never skips the other wheels' stop attempts. */
    for (unsigned i=0;i<3;++i) zero_sent[i]=Servo_WriteData(out->wheels[i].id,46,zero,2)==HAL_OK;
    for (unsigned i=0;i<3;++i) off_sent[i]=Servo_WriteData(out->wheels[i].id,40,zero,1)==HAL_OK;
    HAL_Delay(100);
    out->stop_confirmed=true;
    bool positions_ok=true;
    for (unsigned i=0;i<3;++i) {
        MobileWheelPulseResult *w=&out->wheels[i];
        w->goal_after=read_reg(w->id,46,2);w->torque_after=read_reg(w->id,40,1);
        w->speed_after=read_reg(w->id,58,2);w->position_after=read_reg(w->id,56,2);
        w->stop_confirmed=zero_sent[i] && off_sent[i] && w->goal_after==0 &&
            w->torque_after==0 && w->speed_after==0;
        if (!w->stop_confirmed) {
            if (out->stop_confirmed) out->failed_id=w->id;
            out->stop_confirmed=false;
        }
        if (w->position_after<0) positions_ok=false;
    }
    out->ok=pulse_ok && out->stop_confirmed && positions_ok;
    if (!out->stop_confirmed) out->stage="STOP_UNCONFIRMED_CUT_MOTOR_POWER";
    else if (out->ok) {out->stage="BASE_COMPLETE_TORQUE_OFF";out->failed_id=0;}
    else if (pulse_ok) out->stage="FINAL_POSITION_READ";
}
