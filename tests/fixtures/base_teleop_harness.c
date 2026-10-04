#include "single_arm_app.h"
#include "servo_bus.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static UART_HandleTypeDef host,servo;
static uint8_t regs[3][72];
static const unsigned char *input;
static unsigned remaining,writes,offs[3],zeroes[3],nonzero[3];
static unsigned scenario,inject,enabled;
static uint32_t tick, stopped_at;
static unsigned startup_sweeps;
static char output[16384];
uint32_t HAL_GetTick(void){return tick;}
void HAL_Delay(uint32_t ms){tick+=ms;}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t wait){
    assert(u==&host&&n==1);tick++;
    if(!remaining){tick+=wait;return HAL_TIMEOUT;}
    *p=*input++;remaining--;return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t wait){
    (void)wait;assert(u==&host&&strlen(output)+n<sizeof(output));strncat(output,(const char *)p,n);return HAL_OK;
}
void ServoBus_Init(UART_HandleTypeDef *u,ServoStopRequestedFn a,ServoReadFailureFn b){assert(u==&servo&&!a&&!b);}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t addr,uint8_t n,uint8_t *v){
    assert(id>=7&&id<=9);tick++;
    if(id==7&&addr==46&&zeroes[2]&&!enabled&&zeroes[0]==1)startup_sweeps++;
    if(id==7&&zeroes[2]&&!enabled&&zeroes[0]==1){
        const unsigned regs_to_fail[4]={46,40,58,56};
        if(scenario>=21&&scenario<=24&&addr==regs_to_fail[scenario-21]&&!inject){inject++;return HAL_TIMEOUT;}
        if(scenario==25&&addr==58)return HAL_TIMEOUT;
        if(scenario==26&&addr==58&&startup_sweeps==3)return HAL_TIMEOUT;
        if(scenario==27)tick+=100; /* healthy reads cannot qualify after deadline */
        if(scenario==28&&addr==58){v[0]=50;v[1]=128;return HAL_OK;}
    }
    if(scenario==4&&nonzero[0]&&id==7&&addr==46&&!inject){inject++;return HAL_TIMEOUT;}
    if(scenario==13&&id==8&&addr==40&&enabled&&!inject){inject++;return HAL_TIMEOUT;}
    if(id==7&&addr==58){
        if(scenario==16&&tick>250){regs[0][58]=0;regs[0][59]=0;}
        if(scenario==19)return HAL_TIMEOUT;
        if(scenario==20&&offs[0]&&(uint32_t)(tick-stopped_at)>250){regs[0][58]=0;regs[0][59]=0;}
    }
    memcpy(v,&regs[id-7][addr],n);return HAL_OK;
}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t addr,const uint8_t *v,uint8_t n){
    assert(id>=7&&id<=9);unsigned i=id-7;tick++;writes++;
    if(addr==40){assert(n==1&&v[0]<=1);if(v[0]){if(scenario==26)assert(startup_sweeps>=6);enabled++;assert(!regs[i][46]&&!regs[i][47]);}else offs[i]++;}
    else{
        assert(addr==46&&n==2);unsigned mag=v[0]|((v[1]&127)<<8);assert(mag==0||mag==100||mag==200);
        if(mag){nonzero[i]++;assert(enabled>=3);}
        else zeroes[i]++;
    }
    memcpy(&regs[i][addr],v,n);
    /* Model observed hardware: goal-speed writes may restore torque, even for
     * zero. Explicit torque-off must follow before standstill qualification. */
    if(scenario>=29&&scenario<=31&&addr==46)regs[i][40]=1;
    if((scenario==30||scenario==31)&&id==8&&addr==40&&!v[0]&&!enabled&&!inject){
        inject++;regs[i][40]=1;
        if(scenario==30)return HAL_TIMEOUT;
    }
    if(scenario==15&&addr==46&&v[0]&&!inject){inject++;tick+=450;}
    if(scenario==5&&id==7&&addr==46&&v[0]&&!inject){inject++;return HAL_TIMEOUT;}
    if(scenario==6&&id==7&&addr==40&&!v[0]&&enabled)regs[0][58]=1;
    if(scenario==20&&id==7&&addr==40&&!v[0]&&enabled){regs[0][58]=50;regs[0][59]=128;stopped_at=tick;}
    return HAL_OK;
}
static unsigned crc(const char *s){
    unsigned c=65535;for(;*s;s++){c^=(unsigned char)*s<<8;for(unsigned i=0;i<8;i++)c=((c&32768)?(c<<1)^0x1021:c<<1)&65535;}return c;
}
static void feed(const char *s){input=(const unsigned char *)s;remaining=strlen(s);while(remaining)SingleArmApp_Process();input=NULL;}
static void packet(const char *prefix){char line[80];snprintf(line,sizeof(line),"%s %04X\n",prefix,crc(prefix));feed(line);}
static void idle(void){tick+=401;SingleArmApp_Process();}
static void stopped(void){for(unsigned i=0;i<3;i++){assert(offs[i]&&regs[i][40]==0&&regs[i][46]==0&&regs[i][47]==0);}}
int main(int argc,char **argv){
    assert(argc==2);scenario=atoi(argv[1]);
    for(unsigned i=0;i<3;i++){regs[i][3]=9;regs[i][4]=3;regs[i][5]=i+7;regs[i][33]=1;regs[i][55]=1;}
    if(scenario==16||scenario==17){regs[0][58]=50;regs[0][59]=128;}
    if(scenario==18)regs[0][59]=128; /* negative zero */
    if(scenario==9)tick=UINT32_MAX-200;
    SingleArmApp_Init(&host,&servo);assert(writes==0);feed("INFO\n");assert(writes==0&&strstr(output,"mobile-base-teleop-v4"));output[0]=0;
    packet("ARM 1234ABCD");
    if(scenario>=25&&scenario<=28&&scenario!=26){
        assert(enabled==0&&strstr(output,"STARTUP_NOT_QUIET_OR_READ_FAILED"));
        assert(strstr(output,"\"stop_confirmed\":true"));
        puts(output);output[0]=0;feed("STOP\n");puts(output);
        unsigned before=writes;packet("ARM 11111111");assert(writes==before);
        return 0;
    }
    if((scenario>=21&&scenario<=24)||scenario==26){
        assert(strstr(output,"\"type\":\"armed\""));puts(output);output[0]=0;
        feed("STOP\n");stopped();puts(output);return 0;
    }
    if(scenario==30||scenario==31){
        assert(enabled==0&&strstr(output,"TORQUE_OFF_BEFORE_SETTLE"));
        assert(nonzero[0]==0&&nonzero[1]==0&&nonzero[2]==0);stopped();
        assert(strstr(output,"\"stop_confirmed\":true"));
        unsigned before=writes;packet("ARM 11111111");assert(writes==before);return 0;
    }
    if(scenario==17||scenario==19){
        assert(strstr(output,"STARTUP_NOT_QUIET_OR_READ_FAILED")&&strstr(output,"\"fault\":true"));
        assert(enabled==0&&nonzero[0]==0&&nonzero[1]==0&&nonzero[2]==0);
        if(scenario==17)assert(strstr(output,"\"speed_signed\":-50"));
        unsigned before=writes;packet("ARM 11111111");assert(writes==before);return 0;
    }
    if(scenario==13){assert(strstr(output,"TORQUE_ENABLE")&&strstr(output,"\"fault\":true"));stopped();return 0;}
    assert(strstr(output,"\"type\":\"armed\""));output[0]=0;
    packet("D 1234ABCD 00000001 F");
    if(scenario==4||scenario==5||scenario==15){stopped();assert(strstr(output,"\"fault\":true"));assert(nonzero[1]==0);return 0;}
    assert(regs[0][46]==200&&regs[0][47]==128&&regs[1][46]==200&&regs[1][47]==0&&regs[2][46]==0);
    output[0]=0;
    if(scenario==0||scenario==6||scenario==16||scenario==18||scenario==20||scenario==29)feed("STOP\n");
    else if(scenario==1)packet("D 1234ABCD 00000001 F"); /* replay */
    else if(scenario==2)feed("D 1234ABCD 00000002 F 0000\n");
    else if(scenario==3)packet("D FFFFFFFF 00000002 F");
    else if(scenario==7){
        feed("STOP\n");stopped();output[0]=0;packet("ARM 11111111");assert(strstr(output,"\"type\":\"armed\""));
        packet("D 11111111 00000001 Z");feed("STOP\n");stopped();return 0;
    }else if(scenario==8){
        const char *keys="BLRADZ";unsigned seq=2;
        for(const char *p=keys;*p;p++){char line[40];snprintf(line,sizeof(line),"D 1234ABCD %08X %c",seq++,*p);packet(line);assert(!strstr(output,"\"fault\":true"));}
        feed("STOP\n");
    }else if(scenario==10){feed("D 1234ABCD");idle();}
    else if(scenario==11)feed("\001");
    else if(scenario==12||scenario==9)idle();
    else if(scenario==14)feed("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    stopped();
    if(scenario==0||scenario==7||scenario==8||scenario==16||scenario==18||scenario==20||scenario==29)assert(strstr(output,"\"stop_confirmed\":true"));
    else{
        assert(strstr(output,"\"fault\":true"));if(scenario==6)assert(strstr(output,"STOP_UNCONFIRMED"));
        unsigned before=writes;packet("ARM 11111111");assert(writes==before); /* no fault auto-rearm */
    }
    return 0;
}
