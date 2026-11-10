#include "wheel_hold_monitor.h"
#include "servo_bus.h"
#include "right_servo_bus.h"
#include <string.h>
#include <stdlib.h>
#define SLOTS 13u
#define MAX_AGE_MS 1200u
/* Slots 0..5 left arm, 6..11 right arm, 12 left-bus lift ID10. */
static uint8_t baseline[SLOTS][9],next_slot;
static uint32_t stamp[SLOTS],last_read,samples;
static bool captured,valid;
static unsigned failed_slot;
static const char *reason;
static int lift_position=-1,lift_target=-1;
static bool bad(unsigned slot,const char *why){valid=false;failed_slot=slot+1;reason=why;return false;}
static bool read_slot(unsigned slot,uint8_t out[9]){
    uint8_t id=slot<6?slot+1:slot<12?slot-5:10;
    HAL_StatusTypeDef a,b;
    if(slot>=6 && slot<12){
        a=RightServoBus_HoldRead(id,33,1,out);
        b=a==HAL_OK?RightServoBus_HoldRead(id,40,8,out+1):a;
    }else{
        a=Servo_ReadData(id,33,1,out);
        b=a==HAL_OK?Servo_ReadData(id,40,8,out+1):a;
    }
    if(a!=HAL_OK || b!=HAL_OK)return bad(slot,"HOLD_READ_FAILED");
    if(out[1]!=1)return bad(slot,"HOLD_TORQUE_OFF");
    if(out[0]!=0)return bad(slot,slot==12?"LIFT_POSITION_HOLD_REQUIRED":"HOLD_MODE_INVALID");
    if(slot==12){
        lift_position=-1;lift_target=out[3]|out[4]<<8;
        uint8_t feedback[4];
        if(Servo_ReadData(10,56,4,feedback)!=HAL_OK)return bad(slot,"LIFT_POSITION_READ_FAILED");
        int actual=feedback[0]|feedback[1]<<8,target=out[3]|out[4]<<8;
        lift_position=actual;
        /* About 2 mm on the measured lift. Do not modulo-wrap a lost turn
         * into an apparent success; velocity mode is rejected above. */
        if(actual>4095 || abs(actual-target)>96)return bad(slot,"LIFT_POSITION_DRIFT");
    }
    stamp[slot]=HAL_GetTick();samples++;
    return true;
}
void WheelHold_Init(UART_HandleTypeDef *right){
    RightServoBus_Init(right);captured=valid=false;next_slot=0;samples=last_read=0;
    failed_slot=0;reason="HOLD_NOT_CAPTURED";memset(stamp,0,sizeof(stamp));
    lift_position=lift_target=-1;
}
bool WheelHold_Capture(void){
    captured=valid=false;failed_slot=0;reason="HOLD_NOT_CAPTURED";
    for(unsigned i=0;i<SLOTS;i++)if(!read_slot(i,baseline[i]))return false;
    captured=valid=true;next_slot=0;last_read=HAL_GetTick();reason="HOLD_UNCHANGED";
    return true;
}
static bool verify(unsigned slot){
    uint8_t now[9];
    if(!read_slot(slot,now))return false;
    if(memcmp(now,baseline[slot],sizeof(now)))return bad(slot,"HOLD_COMMAND_CHANGED");
    return true;
}
bool WheelHold_VerifyAll(void){
    if(!captured)return false;
    for(unsigned i=0;i<SLOTS;i++)if(!verify(i))return false;
    valid=true;failed_slot=0;reason="HOLD_UNCHANGED";last_read=HAL_GetTick();
    return true;
}
bool WheelHold_Confirmed(void){
    if(!captured || !valid)return false;
    uint32_t now=HAL_GetTick();
    for(unsigned i=0;i<SLOTS;i++)if(now-stamp[i]>MAX_AGE_MS)return false;
    return true;
}
bool WheelHold_Poll(void){
    if(!captured || !valid)return false;
    uint32_t now=HAL_GetTick();
    if(now-last_read>=20u){
        last_read=now;
        if(!verify(next_slot))return false;
        next_slot=(next_slot+1u)%SLOTS;
    }
    for(unsigned i=0;i<SLOTS;i++)if(HAL_GetTick()-stamp[i]>MAX_AGE_MS)return bad(i,"HOLD_FEEDBACK_STALE");
    return true;
}
bool WheelHold_HasBaseline(void){return captured;}
unsigned WheelHold_FailedSlot(void){return failed_slot;}
const char *WheelHold_Reason(void){return reason;}
uint32_t WheelHold_Samples(void){return samples;}
int WheelHold_LiftPosition(void){return lift_position;}
int WheelHold_LiftTarget(void){return lift_target;}
