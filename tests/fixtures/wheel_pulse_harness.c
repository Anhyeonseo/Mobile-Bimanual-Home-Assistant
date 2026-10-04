#include "mobile_wheel_pulse.h"
#include "servo_bus.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
static uint8_t regs[3][72];
static unsigned writes, enables, moves, zeros, offs;
static uint32_t tick, motion_start;
static int scenario, injected;
uint32_t HAL_GetTick(void) {return tick;}
void HAL_Delay(uint32_t ms) {tick+=ms;}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t address,uint8_t length,uint8_t *data) {
    assert(id==8); assert(address+length<=72); tick+=2;
    if(scenario==13&&address==58)return HAL_TIMEOUT;
    if(scenario==4&&address==46&&moves&&!injected){injected=1;return HAL_TIMEOUT;}
    if(scenario==7&&address==40&&enables&&!injected){injected=1;return HAL_TIMEOUT;}
    if(scenario==8&&address==56&&offs)return HAL_TIMEOUT;
    memcpy(data,&regs[id-7][address],length);return HAL_OK;
}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t address,const uint8_t *data,uint8_t length) {
    assert(id==8); writes++; tick+=2;
    if(address==40) {
        assert(length==1&&data[0]<=1);
        if(data[0]){enables++;assert(zeros==1&&regs[1][46]==0&&regs[1][47]==0);}
        else offs++;
    } else {
        assert(address==46&&length==2);
        if(data[0]){assert(data[0]==200&&(data[1]==0||data[1]==128));assert(regs[1][40]==1);moves++;motion_start=tick;}
        else {zeros++;if(moves)assert((uint32_t)(tick-motion_start)<=3005);}
    }
    if(scenario==5&&moves&&address==46&&data[0]==0)return HAL_TIMEOUT;
    memcpy(&regs[id-7][address],data,length);
    if(scenario==3&&moves&&!injected){injected=1;return HAL_TIMEOUT;} /* applied but uncertain */
    if(scenario==6&&offs)regs[1][58]=1; /* moving after cleanup */
    return HAL_OK;
}
int main(int argc,char **argv) {
    assert(argc==2);scenario=atoi(argv[1]);
    for(unsigned i=0;i<3;i++) {regs[i][3]=9;regs[i][4]=3;regs[i][5]=i+7;regs[i][55]=1;regs[i][33]=1;}
    if(scenario==1)regs[1][40]=1;
    if(scenario==2)regs[1][33]=0;
    if(scenario==9)tick=UINT32_MAX-200; /* deadline handles rollover */
    if(scenario==11)regs[1][58]=1;
    if(scenario==12){regs[0][58]=100;regs[2][33]=0;regs[2][40]=1;} /* irrelevant wheels */
    MobileWheelPulseResult x;
    MobileWheelPulse_Run(10,1,&x);assert(!x.ok&&!MobileWheelPulse_Consumed()&&writes==0);
    MobileWheelPulse_Run(8,0,&x);assert(!x.ok&&!MobileWheelPulse_Consumed()&&writes==0);
    MobileWheelPulse_Run(8,scenario==10?-1:1,&x);assert(MobileWheelPulse_Consumed());
    if(scenario==1||scenario==2||scenario==11||scenario==13) {assert(!x.ok&&writes==0&&x.failed_id==8);}
    else {
        assert(offs==1&&enables==1&&zeros==2&&x.cleanup_attempted);
        if(scenario==5||scenario==6)assert(!x.ok&&!x.stop_confirmed);
        else assert(x.stop_confirmed&&regs[1][40]==0&&regs[1][46]==0&&regs[1][47]==0);
        if(scenario==0||scenario==9||scenario==10||scenario==12)assert(x.ok&&x.stop_requested_ms==3000&&moves==1);
        else assert(!x.ok);
        if(scenario==7)assert(moves==0);
    }
    if(scenario==11)assert(x.speed_check_raw==1&&x.speed_check_hal_status==HAL_OK&&
        !strcmp(x.stage,"WHEEL_MUST_BE_STOPPED"));
    if(scenario==13)assert(x.speed_check_raw==-1&&x.speed_check_hal_status==HAL_TIMEOUT&&
        !strcmp(x.stage,"SPEED_READ_FAILED"));
    unsigned count=writes;MobileWheelPulse_Run(8,1,&x);
    assert(!x.ok&&!strcmp(x.stage,"ALREADY_ATTEMPTED")&&writes==count);
    MobileBasePulseResult group;MobileWheelPulse_RunBase('F',&group);assert(!group.ok&&writes==count);
    return 0;
}
