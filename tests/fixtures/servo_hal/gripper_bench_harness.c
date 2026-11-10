#include "stm32g4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned tick, reads, motor_writes, fail_off, fail_read, implicit_enable, swapped_speed;
static uint8_t regs[80];
static const char *input;
static char output[1024];
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t timeout){
    (void)u;(void)n;(void)timeout;
    if(!input || !*input) return HAL_TIMEOUT;
    *p=(uint8_t)*input++; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t timeout){
    (void)u;(void)timeout;assert(n<sizeof(output));memcpy(output,p,n);output[n]=0;return HAL_OK;
}
#include "../../../../firmware/stm32_g474_single_arm/Core/Src/gripper_bench_app.c"
void ServoBus_Init(UART_HandleTypeDef *u,uint8_t(*stop)(void),void(*failure)(uint8_t)){
    (void)u;(void)stop;(void)failure;
}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t addr,uint8_t n,uint8_t *p){
    assert(id==6);++reads;if(fail_read && addr==56)return HAL_ERROR;
    memcpy(p,regs+addr,n);return HAL_OK;
}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t addr,const uint8_t *p,uint8_t n){
    assert(id==6);
    assert((addr==40 && n==1 && p[0]<=1) || (n==2 && (addr==42||addr==44||addr==46||addr==48)));
    if(addr==42){assert(word(regs+46)==GRIP_SPEED && word(regs+48)<=GRIP_CAP);
        if(implicit_enable) regs[40]=1;
    }
    if(addr==46 && swapped_speed) {regs[46]=9;return HAL_OK;}
    ++motor_writes;
    if(fail_off && addr==40 && !p[0])return HAL_ERROR;
    memcpy(regs+addr,p,n);return HAL_OK;
}
static void set_word(unsigned addr,int v){regs[addr]=v;regs[addr+1]=v>>8;}
static void line(const char *cmd){input=cmd;output[0]=0;while(*input)SingleArmApp_Process();input=NULL;}
static void init(void){
    memset(regs,0,sizeof(regs));regs[3]=9;regs[4]=3;regs[5]=6;set_word(31,0x253);
    set_word(56,3000);set_word(48,150);motor_writes=reads=fail_off=fail_read=implicit_enable=swapped_speed=tick=0;
    SingleArmApp_Init(NULL,NULL);assert(!reads && !motor_writes);
}
static void start(void){line("BIND 41\n");assert(bound && !active && off_verified);
    line("BEGIN 41 1950 3172\n");assert(active && regs[40]==1 && goal==3000 && word(regs+48)==80);}
static void terminal(const char *why){
    assert(!active && off_verified && !locked && regs[40]==0 && !strcmp(reason,why));
}
static void start_hold(void) {
    init();line("BIND 41\n");line("BEGIN_RIGID 41 2994 3100 2000\n");
    assert(active && hold_duration==2000 && hold_goal<0);
    tick=100;set_word(60,60);line("STEP 41 1\n");
    tick=225;line("STEP 41 2\n");
    tick=350;line("STEP 41 3\n");
    assert(active && preloading && hold_goal<0 && !strcmp(hold_cause,"EFFORT_PERSISTED"));
    tick=450;line("STEP 41 4\n");
    assert(active && !preloading && hold_goal==2994 && goal==lower_bound);
}
int main(void){
    init();line("INFO\n");line("BEGIN 41 1950 3172\n");line("STEP 41 1\n");assert(!motor_writes);
    line("BIND 41\n");tick=3001;line("BEGIN 41 1950 3172\n");assert(!active && !motor_writes);
    start();tick+=100;line("STEP 41 1\n");assert(goal==2994 && active);
    line("STEP 41 1\n");terminal("SEQUENCE_MISMATCH");
    init();start();tick=301;SingleArmApp_Process();terminal("HOST_LEASE");
    init();start();tick=100;set_word(60,1024+60);line("STEP 41 1\n");assert(active && !strcmp(reason,"OBSERVING_SAME_GOAL"));
    unsigned fixed_writes=motor_writes;
    tick=225;line("STEP 41 2\n");assert(active && motor_writes==fixed_writes && goal==3000);
    tick=350;line("STEP 41 3\n");terminal("EFFORT_PERSISTED");
    init();start();tick=100;set_word(60,60);line("STEP 41 1\n");assert(goal==3000);
    tick=190;set_word(60,0);line("STEP 41 2\n");assert(active && goal==2994);
    tick=280;set_word(60,81);line("STEP 41 3\n");terminal("EFFORT_BOUND");
    init();start();tick=100;set_word(69,32768+31);line("STEP 41 1\n");terminal("EFFORT_BOUND");
    init();start();tick=100;fail_read=1;line("STEP 41 1\n");terminal("FEEDBACK_OR_CONFIG");
    init();start();tick=100;regs[46]=0;line("STEP 41 1\n");terminal("FEEDBACK_OR_CONFIG");
    init();start();fail_off=1;line("STOP\n");assert(locked && !active && !off_verified);
    line("BIND 42\n");assert(locked && !bound);fail_off=0;line("STOP\n");terminal("OPERATOR_STOP");
    init();regs[31]++;line("BIND 41\n");assert(!bound && !motor_writes);
    init();regs[33]=1;line("BIND 41\n");assert(!bound && !motor_writes);
    init();set_word(48,0);line("BIND 41\n");line("BEGIN 41 1950 3172\n");assert(!active&&!motor_writes);
    init();set_word(48,35);line("BIND 41\n");line("BEGIN 41 1950 3172\n");assert(active && cap==35);
    for(int i=1;i<=6 && active;i++){
        char c[30];tick+=100;snprintf(c,sizeof(c),"STEP 41 %d\n",i);line(c);
    }
    terminal("NO_PROGRESS");
    init();set_word(56,1957);line("BIND 41\n");line("BEGIN 41 1950 3172\n");
    for(int i=1;i<=4 && active;i++){
        char c[30];tick+=100;set_word(56,goal);snprintf(c,sizeof(c),"STEP 41 %d\n",i);line(c);
    }
    terminal("LOWER_GUARD");assert(goal==1950);
    init();implicit_enable=1;start();assert(seed_torque==1);line("STOP\n");terminal("OPERATOR_STOP");
    init();swapped_speed=1;line("BIND 41\n");line("BEGIN 41 1950 3172\n");terminal("CAP_OR_SPEED_READBACK");
    assert(failed_reg==46 && returned_value==9 && expected_value==75);
    init();line("BIND 41\n");line("BEGIN 41\n");assert(!active && !motor_writes);
    init();line("BIND 41\n");line("BEGIN 41 4294967295 3000\n");assert(!active && !motor_writes);
    init();line("BIND 41\n");line("BEGIN 41 2998 3000\n");assert(!active && !motor_writes);
    init();line("BIND 41\n");line("BEGIN 41 2900 2999\n");assert(!active && !motor_writes);
    init();line("BIND 41\n");line("BEGIN 41 2993 3000\n");assert(active);
    for (int i=1;i<=4 && active;i++){
        char c[30];tick+=100;set_word(56,goal);snprintf(c,sizeof(c),"STEP 41 %d\n",i);line(c);
        assert(goal>=2993 && lower_bound==2993 && upper_bound==3000);
    }
    terminal("LOWER_GUARD");
    init();line("BIND 41\n");line("BEGIN 41 2988 3001\n");set_word(56,2987);
    tick=100;line("STEP 41 1\n");terminal("TRACKING_LIMIT");
    start_hold();
    fixed_writes=motor_writes;
    /* Losing load never resumes closure. The previously selected goal stays fixed. */
    set_word(60,0);
    for (unsigned i=5;i<=19;i++) {
        char c[30];tick+=125;snprintf(c,sizeof(c),"STEP 41 %u\n",i);line(c);
        assert(active && motor_writes==fixed_writes && goal==2994 && hold_elapsed<2000);
    }
    tick=2450;SingleArmApp_Process();terminal("HOLD_COMPLETE");
    assert(hold_elapsed==2000 && motor_writes==fixed_writes+1);
    line("STEP 41 20\n");assert(sequence==20 && !active && motor_writes==fixed_writes+1);
    start_hold();tick+=300;SingleArmApp_Process();terminal("HOST_LEASE");
    start_hold();tick+=100;set_word(60,81);line("STEP 41 5\n");terminal("EFFORT_BOUND");
    start_hold();tick+=100;fail_read=1;line("STEP 41 5\n");terminal("FEEDBACK_OR_CONFIG");
    start_hold();tick+=100;set_word(42,2988);line("STEP 41 5\n");terminal("FEEDBACK_OR_CONFIG");
    start_hold();line("STEP 41 4\n");terminal("SEQUENCE_MISMATCH");
    start_hold();line("STOP\n");terminal("OPERATOR_STOP");
    init();line("BIND 41\n");line("BEGIN_RIGID 41 2900 3100 3001\n");assert(!active && !motor_writes);
    init();line("BIND 41\n");line("BEGIN_RIGID 41 2900 3100 0\n");assert(!active && !motor_writes);
    init();line("BIND 41\n");line("BEGIN_RIGID 41 2900 3100 2000\n");
    for (unsigned i=1;i<=6;i++) {
        char c[30];tick+=100;snprintf(c,sizeof(c),"STEP 41 %u\n",i);line(c);
    }
    assert(active && preloading && !strcmp(hold_cause,"NO_PROGRESS"));
    unsigned seq=6;
    while (preloading) {
        char c[30];tick+=100;snprintf(c,sizeof(c),"STEP 41 %u\n",++seq);line(c);
    }
    assert(active && hold_goal==lower_bound);
    /* Timer completion validates current motor data, even without a new host STEP. */
    hold_at=tick-2000;heartbeat=tick;fail_read=1;SingleArmApp_Process();terminal("FEEDBACK_OR_CONFIG");
    return 0;
}
