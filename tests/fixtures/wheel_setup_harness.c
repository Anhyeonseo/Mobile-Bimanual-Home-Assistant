#include "mobile_wheel_setup.h"
#include "servo_bus.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
static uint8_t regs[3][72];
static unsigned writes,mode_writes,unlocks;
static int scenario,failed;
void HAL_Delay(uint32_t ms) {assert(ms==20);}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t address,uint8_t length,uint8_t *data) {
    assert(id>=7&&id<=9);assert(address+length<=72);
    if(scenario==4&&id==8&&address==33&&regs[1][33]==1&&!failed) {failed=1;return HAL_TIMEOUT;}
    memcpy(data,&regs[id-7][address],length);return HAL_OK;
}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t address,const uint8_t *data,uint8_t length) {
    assert(id>=7&&id<=9);writes++;
    assert(regs[id-7][40]==0);
    if(address==33){assert(length==1&&data[0]==1&&regs[id-7][55]==0);mode_writes++;}
    else if(address==55){assert(length==1&&data[0]<=1);if(!data[0])unlocks++;}
    else {assert(address==46&&length==2&&data[0]==0&&data[1]==0);}
    memcpy(&regs[id-7][address],data,length);
    if(scenario==5&&id==8&&address==33&&!failed){failed=1;return HAL_TIMEOUT;}
    return HAL_OK;
}
int main(int argc,char **argv) {
    assert(argc==2);scenario=atoi(argv[1]);
    for(unsigned i=0;i<3;i++){regs[i][3]=9;regs[i][4]=3;regs[i][5]=i+7;regs[i][55]=1;}
    if(scenario==1)regs[2][40]=1;
    if(scenario==2)regs[2][5]=1;
    if(scenario==3)for(unsigned i=0;i<3;i++)regs[i][33]=1;
    MobileWheelSetupResult results[3];bool ok=MobileWheelSetup_Run(results);
    if(scenario==0){assert(ok&&mode_writes==3&&unlocks==3);}
    if(scenario==1||scenario==2){assert(!ok&&writes==0);}
    if(scenario==3){assert(ok&&mode_writes==0&&unlocks==0&&writes==3);}
    if(scenario==4||scenario==5){assert(!ok&&regs[1][55]==1&&regs[2][33]==0&&mode_writes==2);}
    if(ok)for(unsigned i=0;i<3;i++)assert(results[i].ok&&results[i].mode==1&&results[i].torque==0&&results[i].lock==1);
    unsigned count=writes;assert(!MobileWheelSetup_Run(results)&&writes==count);
    assert(!strcmp(results[0].stage,"ALREADY_ATTEMPTED"));return 0;
}
