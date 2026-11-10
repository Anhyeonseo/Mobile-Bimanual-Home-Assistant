#include "single_arm_app.h"
#include "servo_bus.h"
#include "right_servo_bus.h"
#include "wheel_hold_monitor.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
UART_HandleTypeDef huart1,huart4;
static UART_HandleTypeDef host;
static uint8_t reg[2][11][72],original[2][11][72];
static unsigned scenario,writes,nonzero,sequence;
static uint32_t tick;
static const char *input;
static char output[131072];
static bool inject;
uint32_t HAL_GetTick(void){return tick;}
void HAL_Delay(uint32_t ms){tick+=ms;}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t wait){
    assert(u==&host && n==1);tick++;
    if(!input || !*input){tick+=wait;return HAL_TIMEOUT;}
    *p=(uint8_t)*input++;return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t wait){
    (void)wait;assert(u==&host && strlen(output)+n<sizeof(output));strncat(output,(const char *)p,n);return HAL_OK;
}
void ServoBus_Init(UART_HandleTypeDef *u,ServoStopRequestedFn a,ServoReadFailureFn b){assert(u==&huart1&&!a&&!b);}
void RightServoBus_Init(UART_HandleTypeDef *u){assert(u==&huart4);}
static HAL_StatusTypeDef read_bus(unsigned bus,uint8_t id,uint8_t addr,uint8_t n,uint8_t *v){
    /* Polling host UART has no ring buffer: motor I/O while an incoming
     * command is incomplete would leave its next bytes unserviced. */
    if(scenario>=21)assert(!input || !*input);
    assert(id>=1 && id<=10 && addr+n<=72);tick++;
    if(inject && ((scenario==9 && bus==0 && id==6) || (scenario==10 && bus==1 && id==6)))return HAL_TIMEOUT;
    if(scenario>=26 && !bus && id==7 && addr==58){reg[bus][id][58]=50;reg[bus][id][59]=0;}
    if(scenario==27 && !bus && id==7 && addr==56)reg[bus][id][56]+=4;
    memcpy(v,reg[bus][id]+addr,n);return HAL_OK;
}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t addr,uint8_t n,uint8_t *v){return read_bus(0,id,addr,n,v);}
HAL_StatusTypeDef RightServoBus_HoldRead(uint8_t id,uint8_t addr,uint8_t n,uint8_t *v){return read_bus(1,id,addr,n,v);}
HAL_StatusTypeDef RightServoBus_HoldSetupWrite(uint8_t id,uint8_t addr,const uint8_t *v,uint8_t n){
    (void)id;(void)addr;(void)v;(void)n;assert(0);return HAL_ERROR; /* Never used in wheel mode. */
}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t addr,const uint8_t *v,uint8_t n){
    /* Every success and fault path must remain wheel-only. */
    assert(id>=7 && id<=9);assert((addr==40 && n==1) || (addr==46 && n==2));
    tick++;writes++;
    if(inject && scenario==16 && id==7 && addr==46 && v[0]==0)return HAL_TIMEOUT;
    memcpy(reg[0][id]+addr,v,n);
    if(addr==46){
        reg[0][id][58]=v[0];reg[0][id][59]=v[1];
        reg[0][id][40]=1; /* observed servo goal-write/torque side effect */
        if(v[0])nonzero++;
    }
    return HAL_OK;
}
static unsigned crc(const char *s){unsigned c=65535;for(;*s;s++){c^=(unsigned char)*s<<8;for(unsigned i=0;i<8;i++)c=((c&32768)?(c<<1)^0x1021:c<<1)&65535;}return c;}
static void feed(const char *s){input=s;while(*input)SingleArmApp_Process();input=NULL;}
static void packet(const char *prefix){char line[90];snprintf(line,sizeof(line),"%s %04X\n",prefix,crc(prefix));feed(line);}
static void drive(char c){char text[80];snprintf(text,sizeof(text),"D 1234ABCD %08X %c",++sequence,c);packet(text);}
static void zero_wheels(void){for(unsigned id=7;id<=9;id++)assert(reg[0][id][40]==0 && reg[0][id][46]==0 && reg[0][id][47]==0);}
static void preserved(void){
    for(unsigned b=0;b<2;b++)for(unsigned id=1;id<=6;id++)assert(!memcmp(reg[b][id],original[b][id],72));
    assert(!memcmp(reg[0][10],original[0][10],72));
}
int main(int argc,char **argv){
    assert(argc==2);scenario=(unsigned)atoi(argv[1]);
    for(unsigned b=0;b<2;b++)for(unsigned id=1;id<=10;id++){
        reg[b][id][3]=9;reg[b][id][4]=3;reg[b][id][5]=id;reg[b][id][55]=1;
        reg[b][id][40]=1;reg[b][id][42]=100+id;reg[b][id][43]=8;
        reg[b][id][56]=200;reg[b][id][57]=8; /* not equal to gripper's goal */
    }
    for(unsigned id=7;id<=9;id++){reg[0][id][33]=1;reg[0][id][40]=0;}
    if(scenario==11)reg[0][6][40]=0;
    if(scenario==12){reg[0][10][33]=1;reg[0][10][46]=0;}
    if(scenario==13)reg[1][4][33]=1;
    reg[0][10][42]=200; /* existing position hold matches encoder */
    memcpy(original,reg,sizeof(reg));
    if(scenario==23)tick=UINT32_MAX-300000u;
    SingleArmApp_Init(&host,&huart1);assert(writes==0);
    feed("INFO\n");assert(writes==0 && strstr(output,"mobile-base-hold-teleop-v7"));
    feed("HOLD\n");assert(writes==0);
    packet("ARM 1234ABCD");
    if((scenario>=11 && scenario<=13) || scenario==27){assert((writes==0 || scenario==27) && !strstr(output,"\"type\":\"armed\""));if(scenario!=27)preserved();puts(output);return 0;}
    assert(strstr(output,"\"type\":\"armed\"") && WheelHold_Confirmed());
    if(scenario>=21 && scenario<=23){
        const uint32_t start=tick;
        unsigned commands=0;
        while((uint32_t)(tick-start)<600000u){
            uint32_t cycle=tick;
            output[0]=0;
            drive("FZLZAZBZRZDZ"[(commands/40u)%12u]);
            assert(strstr(output,"\"type\":\"ack\"") && !strstr(output,"\"fault\":true"));
            assert(WheelHold_Confirmed());commands++;
            /* Typical 20 Hz traffic with occasional 150 ms host jitter. */
            unsigned interval=commands%100u==0?150u:50u;
            while((uint32_t)(tick-cycle)<interval)SingleArmApp_Process();
            if(scenario==22 && commands==7000){
                tick+=401;SingleArmApp_Process();zero_wheels();preserved();
                assert(strstr(output,"HOST_TIMEOUT"));
                puts("{\"expected_link_loss_stop\":true,\"arm_lift_preserved\":true}");return 0;
            }
        }
        output[0]=0;feed("STOP\n");zero_wheels();preserved();
        assert(WheelHold_Confirmed());
        printf("{\"simulated_ms\":%lu,\"commands\":%u,\"arm_lift_preserved\":true}\n",(unsigned long)(tick-start),commands);
        puts(output);return 0;
    }
    drive('F');assert(nonzero==2);inject=true;
    switch(scenario){
    case 26:case 0:drive('Z');feed("STOP\n");zero_wheels();preserved();assert(WheelHold_Confirmed());break;
    case 1:tick+=401;SingleArmApp_Process();zero_wheels();preserved();break;
    case 2:packet("D 1234ABCD 00000001 F");zero_wheels();preserved();break;
    case 3:reg[0][6][40]=0;break;
    case 4:reg[1][6][40]=0;break;
    case 5:reg[0][10][40]=0;break;
    case 6:reg[0][6][42]++;break;
    case 7:reg[1][6][42]++;break;
    case 8:reg[0][10][42]++;break;
    case 9:case 10:break;
    case 14:reg[1][1][33]=1;break;
    case 15:feed("invalid\n");zero_wheels();preserved();break;
    case 16:feed("STOP\n");assert(strstr(output,"\"stop_confirmed\":false"));preserved();break;
    case 17:packet("D FFFFFFFF 00000002 F");zero_wheels();preserved();break;
    case 18:tick=UINT32_MAX-10;WheelHold_Capture();tick+=100;assert(WheelHold_Poll());break;
    case 19:tick+=1201;assert(!WheelHold_Poll());assert(!WheelHold_Confirmed());break;
    case 20:reg[0][6][42]++;feed("HOLD\n");assert(!WheelHold_Confirmed());break;
    case 24:reg[0][10][56]=41;reg[0][10][57]=9;break; /* excessive drift despite unchanged ON/goal */
    case 25:reg[0][10][33]=1;break; /* zero speed can never qualify as height hold */
    default:assert(0);
    }
    if((scenario>=3 && scenario<=10) || scenario==14 || scenario==20 || scenario==24 || scenario==25){
        for(unsigned i=0;i<30;i++){tick+=20;drive('F');}
        assert(strstr(output,"\"fault\":true"));assert(!WheelHold_Confirmed());zero_wheels();
        if(scenario==24)assert(strstr(output,"LIFT_POSITION_DRIFT"));
        if(scenario==25)assert(strstr(output,"LIFT_POSITION_HOLD_REQUIRED"));
    }
    unsigned prior=nonzero;
    if(scenario!=18 && scenario!=19 && scenario!=0 && scenario!=26){packet("ARM 1234ABCD");assert(nonzero==prior);}
    puts(output);
}
