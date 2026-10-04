#include "mobile_wheel_setup.h"
#include "servo_bus.h"
#include "actuator_core/mobile_ids.h"
#include <string.h>
static bool attempted;
static int byte(uint8_t id, uint8_t address) {
    uint8_t data;
    return Servo_ReadData(id,address,1,&data)==HAL_OK ? data : -1;
}
static bool write_checked(uint8_t id,uint8_t address,const uint8_t *data,uint8_t size) {
    uint8_t actual[2]={0};
    if(Servo_WriteData(id,address,data,size)!=HAL_OK)return false;
    if(address==33)HAL_Delay(20); /* allow EEPROM mode write to settle */
    return Servo_ReadData(id,address,size,actual)==HAL_OK&&!memcmp(data,actual,size);
}
bool MobileWheelSetup_Run(MobileWheelSetupResult result[3]) {
    if(!result)return false;
    for(unsigned i=0;i<3;i++) result[i]=(MobileWheelSetupResult){
        .id=actuator_mobile_axis_id(i),.mode=-1,.torque=-1,.lock=-1,
        .stage=attempted?"ALREADY_ATTEMPTED":"NOT_ATTEMPTED"};
    if(attempted)return false;
    attempted=true;
    /* Validate all three BEFORE changing any register. */
    for(unsigned i=0;i<3;i++) {
        MobileWheelSetupResult *x=&result[i];uint8_t data[4];
        x->stage="IDENTITY";
        if(Servo_ReadData(x->id,3,4,data)!=HAL_OK||data[0]!=9||data[1]!=3||
           data[2]!=x->id||data[3]!=0)return false; /* observed model 777, baud 1M */
        x->stage="TORQUE_MUST_BE_OFF";x->torque=byte(x->id,40);
        if(x->torque!=0)return false;
        x->stage="MODE";x->mode=byte(x->id,33);
        if(x->mode!=0&&x->mode!=1)return false;
        x->stage="LOCK";x->lock=byte(x->id,55);
        if(x->lock!=0&&x->lock!=1)return false;
        x->stage="PREFLIGHT_OK";
    }
    const uint8_t zero[2]={0,0},one=1;
    for(unsigned i=0;i<3;i++) {
        MobileWheelSetupResult *x=&result[i];
        x->stage="ZERO_SPEED";
        if(!write_checked(x->id,46,zero,2))return false;
        if(x->mode==0) {
            x->stage="UNLOCK";
            bool unlocked=write_checked(x->id,55,zero,1);
            bool mode_ok=false;
            if(unlocked) {x->stage="MODE_WRITE";mode_ok=write_checked(x->id,33,&one,1);}
            /* Always attempt relock after an uncertain unlock/mode write.
             * No automatic EEPROM retry; preserve the first failure stage. */
            const char *failed_stage=x->stage;
            bool locked=write_checked(x->id,55,&one,1);
            if(!unlocked||!mode_ok||!locked) {
                x->stage=!locked?"RELOCK":failed_stage;
                x->mode=byte(x->id,33);x->lock=byte(x->id,55);x->torque=byte(x->id,40);
                return false;
            }
        } else if(x->lock!=1) {
            x->stage="RELOCK";
            if(!write_checked(x->id,55,&one,1))return false;
        }
        x->stage="FINAL_READBACK";
        x->mode=byte(x->id,33);x->torque=byte(x->id,40);x->lock=byte(x->id,55);
        if(x->mode!=1||x->torque!=0||x->lock!=1)return false;
        x->ok=true;x->stage="READY_TORQUE_OFF";
    }
    return true;
}
