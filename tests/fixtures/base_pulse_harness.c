#include "mobile_wheel_pulse.h"
#include "servo_bus.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
static uint8_t regs[3][72];
static unsigned writes,enables,zeros[3],offs[3],moves;
static int scenario,injected,actual[3];
static uint32_t tick,started;
uint32_t HAL_GetTick(void){return tick;}
void HAL_Delay(uint32_t ms){tick+=ms;}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t addr,uint8_t n,uint8_t *data){
    assert(id>=7&&id<=9);tick+=2;
    if(scenario==4&&id==8&&addr==46&&moves&&!injected){injected=1;return HAL_TIMEOUT;}
    if(scenario==7&&id==8&&addr==40&&enables&&!injected){injected=1;return HAL_TIMEOUT;}
    memcpy(data,&regs[id-7][addr],n);return HAL_OK;
}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t addr,const uint8_t *data,uint8_t n){
    assert(id>=7&&id<=9);unsigned i=id-7;tick+=2;writes++;
    if(addr==40){
        assert(n==1&&data[0]<=1);
        if(data[0]){assert(zeros[0]==1&&zeros[1]==1&&zeros[2]==1);enables++;}
        else {assert(zeros[0]>=2&&zeros[1]>=2&&zeros[2]>=2);offs[i]++;}
    }else{
        assert(addr==46&&n==2);
        unsigned raw=data[0]|(data[1]<<8);
        assert((raw&32767)==0||(raw&32767)==100||(raw&32767)==200);
        if(raw){assert(enables==3);if(!moves)started=tick;moves++;actual[i]=(raw&32768)?-(int)(raw&32767):(int)raw;}
        else {zeros[i]++;if(moves)assert((uint32_t)(tick-started)<=3012);}
    }
    if(scenario==5&&id==7&&addr==46&&data[0]==0&&moves)return HAL_TIMEOUT;
    memcpy(&regs[i][addr],data,n);
    if(scenario==3&&id==8&&addr==46&&data[0]&&!injected){injected=1;return HAL_TIMEOUT;}
    if(scenario==6&&offs[i])regs[i][58]=1;
    return HAL_OK;
}
int main(int argc,char **argv){
    assert(argc==3);scenario=atoi(argv[1]);char motion=argv[2][0];
    for(unsigned i=0;i<3;i++){regs[i][3]=9;regs[i][4]=3;regs[i][5]=i+7;regs[i][33]=1;regs[i][55]=1;}
    if(scenario==1)regs[2][40]=1;
    if(scenario==2)regs[1][58]=20;
    if(scenario==8)tick=UINT32_MAX-100;
    MobileBasePulseResult x;
    MobileWheelPulse_RunBase('X',&x);assert(!x.ok&&!MobileWheelPulse_Consumed()&&writes==0);
    MobileWheelPulse_RunBase(motion,&x);assert(MobileWheelPulse_Consumed());
    if(scenario==1||scenario==2){assert(!x.ok&&writes==0);}
    else{
        for(unsigned i=0;i<3;i++)assert(offs[i]==1&&zeros[i]>=2);
        if(scenario==0||scenario==8){
            assert(x.ok&&x.stop_confirmed&&x.stop_requested_ms==3000);
            printf("%d %d %d\n",x.wheels[0].speed_raw,x.wheels[1].speed_raw,x.wheels[2].speed_raw);
            for(unsigned i=0;i<3;i++)assert(actual[i]==x.wheels[i].speed_raw&&x.wheels[i].stop_confirmed);
        }else assert(!x.ok);
        if(scenario==5||scenario==6)assert(!x.stop_confirmed);
        else assert(x.stop_confirmed);
    }
    unsigned count=writes;MobileWheelPulse_RunBase(motion,&x);assert(!x.ok&&writes==count);
    MobileWheelPulseResult single;MobileWheelPulse_Run(8,1,&single);assert(!single.ok&&writes==count);
    return 0;
}
