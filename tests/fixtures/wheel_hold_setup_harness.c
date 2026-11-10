#include "single_arm_app.h"
#include "servo_bus.h"
#include "servo_transport.h"
#include "right_servo_bus.h"
#include "wheel_hold_setup.h"
#include "wheel_hold_monitor.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
UART_HandleTypeDef huart1={.gState=HAL_UART_STATE_READY},huart4={.gState=HAL_UART_STATE_READY};
static UART_HandleTypeDef host;
static uint8_t regs[2][11][72],original[2][11][72];
static unsigned scenario,writes,wheel_writes,held_writes,read_fail,write_fail;
static uint32_t tick;
static const char *input;
static char output[16384];
uint32_t HAL_GetTick(void){return tick;}
uint32_t Timebase_NowUs(void){return tick*1000;}
void HAL_Delay(uint32_t ms){tick+=ms;}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t wait){
    assert(u==&host && n==1);tick++;
    if(!input||!*input){tick+=wait;return HAL_TIMEOUT;}
    *p=*input++;return HAL_OK;
}
static bool held(unsigned bus,unsigned id){return id>=1 && (id<=6 || (!bus && id==10));}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t wait){
    (void)wait;tick++;
    if(u==&host){assert(strlen(output)+n<sizeof(output));strncat(output,(const char *)p,n);return HAL_OK;}
    if(p[4]==2)return HAL_OK;
    assert(p[4]==3);unsigned b=u==&huart4,id=p[2],addr=p[5],len=n-7;writes++;
    if(held(b,id)){
        held_writes++;
        /* Initial ON axes have loaded goals intentionally different from
         * measured position. Setup must never rewrite or disable them. */
        assert((!b && id==10) || original[b][id][40]==0);
        assert((addr==40 && len==1 && (p[6]==1 || (!b && id==10 && p[6]==0))) ||
               (addr==42 && len==2 && p[6]==regs[b][id][56] && p[7]==regs[b][id][57]) ||
               (!b && id==10 && len==1 && ((addr==55 && p[6]<=1) || (addr==33 && p[6]==0))));
        if(write_fail && held_writes==write_fail)return HAL_ERROR;
    }else wheel_writes++;
    if(held(b,id) && addr==33){
        assert(regs[b][id][40]==0 && regs[b][id][55]==0);
        assert(abs((int)(regs[b][id][42]|regs[b][id][43]<<8)-(int)(regs[b][id][56]|regs[b][id][57]<<8))<=8);
    }
    memcpy(regs[b][id]+addr,p+6,len);
    if(!b && id==10 && addr==33){regs[b][id][58]=regs[b][id][59]=0;}
    if(scenario==35 && !b && id==10 && addr==33)return HAL_TIMEOUT; /* applied, ACK lost */
    if(addr==42 || addr==46){
        if(scenario!=1)regs[b][id][40]=1; /* goal-write torque side effect */
        if(!held(b,id)){regs[b][id][58]=p[6];regs[b][id][59]=p[7];}
    }
    if(scenario==10 && held_writes==1 && held(b,id))regs[b][id][42]++;
    if(scenario==11 && held_writes==1 && held(b,id))regs[b][id][33]=1;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit(u,p,n,0);}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit(u,p,n,0);}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
void ServoBus_Init(UART_HandleTypeDef *u,ServoStopRequestedFn a,ServoReadFailureFn b){(void)a;(void)b;assert(ServoTransport_Register(u));}
void RightServoBus_Init(UART_HandleTypeDef *u){assert(ServoTransport_Register(u));}
static HAL_StatusTypeDef transmit(unsigned b,uint8_t id,uint8_t addr,uint8_t op,const uint8_t *v,uint8_t n){
    UART_HandleTypeDef *u=b?&huart4:&huart1;uint32_t token;
    uint8_t p[10]={255,255,id,(uint8_t)(n+3),op,addr};memcpy(p+6,v,n);
    unsigned sum=0;for(unsigned i=2;i<n+6u;i++)sum+=p[i];p[n+6]=(uint8_t)~sum;
    if(!ServoTransport_BeginService(u,ACTUATOR_BUS_WORK_ARM,&token))return HAL_ERROR;
    HAL_StatusTypeDef r=ServoTransport_Transmit(u,token,p,n+7,5);
    assert(ServoTransport_End(u,token,r!=HAL_OK)==HAL_OK);return r;
}
static HAL_StatusTypeDef read_bus(unsigned b,uint8_t id,uint8_t addr,uint8_t n,uint8_t *v){
    if(read_fail && id==read_fail)return HAL_TIMEOUT;
    if(transmit(b,id,addr,2,&n,1)!=HAL_OK)return HAL_ERROR;
    if(!b && id==10 && addr==56 && n==4){
        if(scenario==24)regs[b][id][56]+=4;
        if(scenario==25)regs[b][id][56]^=1;
    }
    if(scenario==37 && !b && id==7 && addr==56)regs[b][id][56]+=4;
    memcpy(v,regs[b][id]+addr,n);return HAL_OK;
}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t addr,uint8_t n,uint8_t *v){return read_bus(0,id,addr,n,v);}
HAL_StatusTypeDef RightServoBus_HoldRead(uint8_t id,uint8_t addr,uint8_t n,uint8_t *v){return read_bus(1,id,addr,n,v);}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t addr,const uint8_t *v,uint8_t n){return transmit(0,id,addr,3,v,n);}
HAL_StatusTypeDef RightServoBus_HoldSetupWrite(uint8_t id,uint8_t addr,const uint8_t *v,uint8_t n){return transmit(1,id,addr,3,v,n);}
static unsigned crc(const char *s){unsigned c=65535;for(;*s;s++){c^=(unsigned char)*s<<8;for(unsigned i=0;i<8;i++)c=((c&32768)?(c<<1)^0x1021:c<<1)&65535;}return c;}
static void feed(const char *s){input=s;while(*input)SingleArmApp_Process();input=NULL;}
static void packet(const char *s){char buf[90];snprintf(buf,sizeof(buf),"%s %04X\n",s,crc(s));feed(buf);}
static void forbidden(void){
    const uint8_t one=1,v[2]={0,8};unsigned before=writes;
    assert(Servo_WriteData(1,42,v,2)!=HAL_OK);
    assert(Servo_WriteData(1,40,&one,1)!=HAL_OK);
    assert(RightServoBus_HoldSetupWrite(6,42,v,2)!=HAL_OK);
    assert(Servo_WriteData(10,46,v,2)!=HAL_OK);assert(writes==before);
}
int main(int argc,char **argv){
    assert(argc==2);scenario=atoi(argv[1]);
    for(unsigned b=0;b<2;b++)for(unsigned id=1;id<=10;id++){
        regs[b][id][3]=9;regs[b][id][4]=3;regs[b][id][5]=id;
        regs[b][id][42]=0xe4;regs[b][id][43]=0x0c; /* stale 3300 goal */
        regs[b][id][11]=255;regs[b][id][12]=15;
        regs[b][id][56]=0x34;regs[b][id][57]=8;regs[b][id][55]=1;
    }
    for(unsigned id=7;id<=10;id++)regs[0][id][33]=1;
    if(scenario==2){regs[0][6][40]=1;regs[1][6][40]=1;}
    if(scenario==12)regs[0][10][33]=0;
    if(scenario==14)for(unsigned b=0;b<2;b++)for(unsigned id=1;id<=10;id++)if(held(b,id))regs[b][id][40]=1;
    memcpy(original,regs,sizeof(regs));
    SingleArmApp_Init(&host,&huart1);assert(writes==0);forbidden();
    if(scenario==3)regs[0][2][58]=30;
    if(scenario==4)regs[0][10][46]=100;
    if(scenario==5)read_fail=4;
    if(scenario==15)regs[1][3][3]=0;
    if(scenario==16)regs[0][7][40]=1;
    if(scenario==17)regs[0][2][57]=32;
    if(scenario==18)regs[1][4][33]=2;
    if(scenario>=22 && scenario<=25)regs[0][10][58]=scenario==22?51:50;
    if(scenario>=19 && scenario<=21){
        regs[0][7][40]=1;
        if(scenario==20)regs[0][7][46]=100;
        if(scenario==21)regs[0][7][47]=128;
        packet("PREP ABCD1234");assert(strstr(output,"SETUP_WHEELS_NOT_OFF") && writes==0);
        assert(strstr(output,"\"axes\":[]"));
        output[0]=0;feed("STOP\n");
        assert(strstr(output,"\"stop_confirmed\":true") && strstr(output,"\"fault\":false"));
        assert(!WheelHold_Confirmed() && held_writes==0);
        for(unsigned id=7;id<=9;id++)assert(regs[0][id][40]==0 && regs[0][id][46]==0 && regs[0][id][47]==0);
        wheel_writes=writes=0;output[0]=0;
    }
    if(scenario==26)regs[0][10][11]=regs[0][10][12]=0;
    if(scenario==27)regs[0][10][55]=0;
    if(scenario>=36)regs[0][7][58]=scenario==38?51:50;
    bool good_plan=WheelSetup_Plan(123);
    if(scenario==37 || scenario==38){assert(!good_plan && writes==0);return 0;}
    if(scenario==22 || scenario==24){assert(!good_plan && writes==0);forbidden();return 0;}
    if((scenario>=3 && scenario<=5) || (scenario>=15 && scenario<=18)){assert(!good_plan && writes==0);forbidden();return 0;}
    assert(good_plan && writes==0);forbidden();
    if(scenario==6)tick+=30001;
    if(scenario==7)regs[0][1][56]+=30;
    if(scenario==8)regs[1][2][42]++;
    if(scenario==9)write_fail=3;
    if(scenario==13){assert(!WheelSetup_Apply(999));assert(!WheelSetup_Apply(123));assert(writes==0);forbidden();return 0;}
    if(scenario>=28 && scenario<=34)write_fail=(scenario-28)+13;
    bool good=WheelSetup_Apply(123);
    if(scenario>=26 && scenario<=35){assert(!good);assert(regs[0][7][40]==0);
        if(scenario==35)assert(regs[0][10][55]==1 && regs[0][10][40]==0);
        forbidden();return 0;}
    assert(wheel_writes==0);forbidden();
    if(scenario>=6 && scenario<=11){assert(!good);if(scenario<=8)assert(writes==0);return 0;}
    assert(good && WheelHold_Confirmed());
    for(unsigned b=0;b<2;b++)for(unsigned id=1;id<=10;id++)if(held(b,id)){
        assert(regs[b][id][40]==1);
        if(original[b][id][40] && (b || id!=10))assert(!memcmp(regs[b][id],original[b][id],72));
    }
    assert(regs[0][10][33]==0 && regs[0][10][55]==1);
    assert(abs((int)(regs[0][10][42]|regs[0][10][43]<<8)-(int)(regs[0][10][56]|regs[0][10][57]<<8))<=8);
    unsigned before=writes;assert(!WheelSetup_Apply(123));assert(before==writes);
    /* Host protocol: plan is read-only; arming cancels that plan. HSET while
     * wheels are armed cannot re-anchor held axes or interrupt holding. */
    packet("PREP ABCD1234");assert(strstr(output,"SETUP_PLAN_READY"));
    packet("ARM 1234ABCD");assert(strstr(output,"\"type\":\"armed\""));
    unsigned old_held=held_writes;
    packet("HSET ABCD1234");assert(strstr(output,"SETUP_REQUIRES_IDLE"));assert(held_writes==old_held);
    feed("STOP\n");assert(WheelHold_Confirmed() && held_writes==old_held);forbidden();
    puts("setup and wheel isolation passed");
}
