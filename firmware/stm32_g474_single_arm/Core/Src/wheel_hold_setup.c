#include "wheel_hold_setup.h"
#include "wheel_hold_monitor.h"
#include "servo_bus.h"
#include "right_servo_bus.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
extern UART_HandleTypeDef huart1,huart4;
#define AXES 13u
#define MAX_SHIFT 8
#define MAX_SPEED 10u
#define PLAN_TTL 30000u
typedef struct {uint8_t mode,runtime[8];uint16_t position,speed;} Sample;
static Sample planned[AXES];
static Sample last_observation;
static unsigned observed_slot;
static bool observation_valid;
static uint32_t token,created;
static bool pending,applying,permit;
static unsigned permit_slot,failed_slot;
static unsigned sampled_slots,wheel_id;
static int wheel_torque,wheel_goal,wheel_speed;
static uint8_t permit_addr,permit_size,permit_data[2];
static const char *why;
static uint8_t id_for(unsigned s){return s<6?s+1:s<12?s-5:10;}
static bool right(unsigned s){return s>=6 && s<12;}
static bool bad(unsigned s,const char *reason){failed_slot=s+1;why=reason;permit=false;return false;}
static bool read_data(unsigned s,uint8_t addr,uint8_t n,uint8_t *out){
    HAL_StatusTypeDef r=right(s)?RightServoBus_HoldRead(id_for(s),addr,n,out):Servo_ReadData(id_for(s),addr,n,out);
    return r==HAL_OK || bad(s,"SETUP_READ_FAILED");
}
static bool sample(unsigned s,Sample *v){
    uint8_t feedback[4];
    observation_valid=false;observed_slot=s+1;
    if(!read_data(s,33,1,&v->mode)||!read_data(s,40,8,v->runtime)||!read_data(s,56,4,feedback))return false;
    v->position=(uint16_t)(feedback[0]|feedback[1]<<8);
    v->speed=(uint16_t)((feedback[2]|feedback[3]<<8)&0x7fff);
    last_observation=*v;observation_valid=true;
    if(v->runtime[0]>1 || (s<12 && v->mode!=0) || (s==12 && v->mode>1))return bad(s,"SETUP_MODE_OR_TORQUE_INVALID");
    if(v->mode==0 && v->position>4095)return bad(s,"SETUP_POSITION_OUT_OF_RANGE");
    /* A stationary velocity-mode lift was observed at position 1860..1861
     * while speed alternated 0/50. Admit only that bounded report range here;
     * Plan AND Apply additionally require a 400+ ms position stability window.
     * Arm/position-mode limits remain unchanged. */
    unsigned speed_limit=(s==12 && v->mode==1)?50u:MAX_SPEED;
    if(v->speed>speed_limit)return bad(s,"SETUP_NOT_STATIONARY");
    if(v->mode==1 && (v->runtime[6] || v->runtime[7]))return bad(s,"SETUP_LIFT_GOAL_NOT_ZERO");
    return true;
}
static bool unchanged(unsigned s,const Sample *a,const Sample *b){
    if(a->mode!=b->mode || memcmp(a->runtime,b->runtime,8))return bad(s,"SETUP_SETTINGS_CHANGED");
    if(abs((int)a->position-(int)b->position)>MAX_SHIFT)return bad(s,"SETUP_POSE_CHANGED");
    return true;
}
static bool lift_stationary(void){
    Sample first,now;
    if(!sample(12,&first)||!unchanged(12,&planned[12],&first))return false;
    int low=0,high=0;
    for(unsigned i=0;i<5;i++){
        HAL_Delay(80);
        if(!sample(12,&now)||!unchanged(12,&first,&now))return false;
        int delta=(int)now.position-(int)first.position;
        if(delta<low)low=delta;
        if(delta>high)high=delta;
        if(high-low>MAX_SHIFT)return bad(12,"SETUP_LIFT_POSITION_DRIFT");
    }
    return true;
}
void WheelSetup_Cancel(void){pending=applying=permit=false;token=0;}
void WheelSetup_Init(void){WheelSetup_Cancel();created=0;failed_slot=0;why="SETUP_NOT_PLANNED";}
static bool wheels_quiet(void){
    int first[3]={0},low[3]={0},high[3]={0};
    /* Powered-on servos can report 0/50 with a stationary encoder. Prove
     * standstill over 400 ms instead of accepting/rejecting one speed sample. */
    for(unsigned pass=0;pass<6;pass++){
    if(pass)HAL_Delay(80);
    for(uint8_t id=7;id<=9;id++){
        uint8_t t,g[2],v[2],p[2];
        wheel_id=id;wheel_torque=wheel_goal=wheel_speed=-1;
        if(Servo_ReadData(id,40,1,&t)!=HAL_OK || Servo_ReadData(id,46,2,g)!=HAL_OK ||
           Servo_ReadData(id,58,2,v)!=HAL_OK || Servo_ReadData(id,56,2,p)!=HAL_OK)
            return bad(AXES,"SETUP_WHEEL_READ_FAILED");
        wheel_torque=t;wheel_goal=g[0]|g[1]<<8;wheel_speed=v[0]|v[1]<<8;
        int pos=p[0]|p[1]<<8;unsigned i=id-7;
        if(t || g[0] || g[1] || (wheel_speed&0x7fff)>50 || pos>4095)
            return bad(AXES,"SETUP_WHEELS_NOT_OFF");
        if(!pass)first[i]=pos;
        int delta=pos-first[i];if(delta>2048)delta-=4096;if(delta< -2048)delta+=4096;
        if(delta<low[i])low[i]=delta;
        if(delta>high[i])high[i]=delta;
        if(high[i]-low[i]>8)return bad(AXES,"SETUP_WHEEL_POSITION_DRIFT");
    }
    }
    return true;
}
bool WheelSetup_Plan(uint32_t nonce){
    WheelSetup_Cancel();failed_slot=0;sampled_slots=0;
    if(!nonce)return bad(AXES,"SETUP_INVALID_TOKEN");
    if(!wheels_quiet())return false;
    for(unsigned s=0;s<AXES;s++){
        uint8_t ident[4];
        if(!read_data(s,3,4,ident))return false;
        if(ident[0]!=9 || ident[1]!=3 || ident[2]!=id_for(s) || ident[3]!=0)return bad(s,"SETUP_IDENTITY_MISMATCH");
        if(!sample(s,&planned[s]))return false;
        sampled_slots=s+1;
    }
    HAL_Delay(100);
    for(unsigned s=0;s<AXES;s++){
        Sample now;if(!sample(s,&now)||!unchanged(s,&planned[s],&now))return false;
        planned[s]=now;
    }
    if(!lift_stationary())return false;
    token=nonce;created=HAL_GetTick();pending=true;why="SETUP_PLAN_READY";return true;
}
bool WheelSetup_ConsumeWrite(UART_HandleTypeDef *uart,uint8_t id,uint8_t address,const uint8_t *data,uint8_t length){
    if(!applying || !permit || uart!=(right(permit_slot)?&huart4:&huart1) || id!=id_for(permit_slot) ||
       address!=permit_addr || length!=permit_size || memcmp(data,permit_data,length))return false;
    permit=false;return true;
}
static bool write_exact(unsigned s,uint8_t addr,const uint8_t *value,uint8_t n){
    permit_slot=s;permit_addr=addr;permit_size=n;memcpy(permit_data,value,n);permit=true;
    HAL_StatusTypeDef status=right(s)?RightServoBus_HoldSetupWrite(id_for(s),addr,value,n):Servo_WriteData(id_for(s),addr,value,n);
    permit=false;
    return status==HAL_OK || bad(s,"SETUP_WRITE_FAILED");
}
static bool lift_write_checked(uint8_t addr,unsigned value,uint8_t n){
    uint8_t data[2]={(uint8_t)value,(uint8_t)(value>>8)},back[2];
    return write_exact(12,addr,data,n) && read_data(12,addr,n,back) &&
        (!memcmp(data,back,n) || bad(12,"SETUP_LIFT_READBACK_FAILED"));
}
/* Supported-load setup only. Zero speed is NOT a height setpoint. Stage the
 * measured encoder angle while still in speed mode, then disable torque before
 * changing mode. Thus neither the mode switch nor a goal-write side effect can
 * activate a stale position target. No homing or multi-turn height inference. */
static bool anchor_lift(Sample *now){
    uint8_t limits[4],lock;
    if(!read_data(12,9,4,limits)||!read_data(12,55,1,&lock))return false;
    unsigned low=limits[0]|limits[1]<<8,high=limits[2]|limits[3]<<8;
    if(lock!=1 || low>=high || high>4095 || now->position<low || now->position>high)
        return bad(12,"SETUP_LIFT_POSITION_LIMITS_INVALID");
    if(now->mode==1){
        unsigned anchor=now->position;
        if(!lift_write_checked(40,0,1))return false;
        if(!lift_write_checked(42,now->position,2))return false;
        if(!lift_write_checked(40,0,1))return false;
        /* A support slip during staging must not become a commanded move. */
        Sample staged;if(!sample(12,&staged))return false;
        if(abs((int)staged.position-(int)now->position)>MAX_SHIFT)
            return bad(12,"SETUP_POSE_CHANGED");
        bool unlocked=lift_write_checked(55,0,1);
        bool changed=unlocked && lift_write_checked(33,0,1);
        /* Relock even if the write/read acknowledgement was lost. */
        bool locked=lift_write_checked(55,1,1);
        if(!changed||!locked)return false;
        if(!sample(12,now)||now->mode!=0||now->runtime[0]!=0)
            return bad(12,"SETUP_LIFT_MODE_SWITCH_FAILED");
        if(now->position<low || now->position>high ||
           abs((int)now->position-(int)anchor)>MAX_SHIFT)
            return bad(12,"SETUP_LIFT_MODE_POSITION_CHANGED");
    }
    /* The lift has no contact-pressure target to preserve. Explicit setup
     * always binds its current angle, including an already powered lift. */
    if(!lift_write_checked(42,now->position,2))return false;
    Sample after;if(!sample(12,&after))return false;
    if(after.runtime[0]==0 && !lift_write_checked(40,1,1))return false;
    if(!sample(12,&after))return false;
    if(after.mode!=0 || after.runtime[0]!=1 ||
       (after.runtime[2]|after.runtime[3]<<8)!=now->position ||
       abs((int)after.position-(int)now->position)>MAX_SHIFT)
        return bad(12,"SETUP_LIFT_ANCHOR_FAILED");
    return true;
}
bool WheelSetup_Apply(uint32_t nonce){
    bool accepted=pending && nonce==token && (uint32_t)(HAL_GetTick()-created)<=PLAN_TTL;
    pending=false;token=0;permit=false;
    if(!accepted)return bad(AXES,"SETUP_TOKEN_OR_EXPIRY");
    if(!wheels_quiet())return false;
    for(unsigned s=0;s<AXES;s++){
        Sample now;if(!sample(s,&now)||!unchanged(s,&planned[s],&now))return false;
    }
    if(!lift_stationary())return false;
    applying=true;
    for(unsigned s=0;s<AXES;s++){
        Sample now;
        if(!sample(s,&now)||!unchanged(s,&planned[s],&now))goto failed;
        if(s==12){if(!anchor_lift(&now))goto failed;continue;}
        if(now.runtime[0]==1)continue; /* Preserve existing loaded goals. */
        /* Goal writes can themselves enable torque. The FIRST write is
         * therefore the freshly measured position (or lift zero velocity),
         * never speed/acceleration/mode against an old stored goal. */
        uint8_t value[2]={(uint8_t)now.position,(uint8_t)(now.position>>8)};
        uint8_t addr=42;
        if(now.mode==1){addr=46;value[0]=value[1]=0;}
        if(!write_exact(s,addr,value,2))goto failed;
        Sample after;if(!sample(s,&after))goto failed;
        uint8_t expected[8];memcpy(expected,now.runtime,8);expected[0]=after.runtime[0];
        expected[addr-40]=value[0];expected[addr-39]=value[1];
        if(after.mode!=now.mode || memcmp(after.runtime,expected,8) ||
           abs((int)now.position-(int)after.position)>MAX_SHIFT){bad(s,"SETUP_GOAL_READBACK_FAILED");goto failed;}
        if(after.runtime[0]==0){
            const uint8_t on=1;if(!write_exact(s,40,&on,1))goto failed;
        }
        if(!sample(s,&after))goto failed;
        expected[0]=1;
        if(after.mode!=now.mode || memcmp(after.runtime,expected,8) ||
           abs((int)now.position-(int)after.position)>MAX_SHIFT){bad(s,"SETUP_TORQUE_READBACK_FAILED");goto failed;}
    }
    applying=permit=false;
    if(!WheelHold_Capture()){why=WheelHold_Reason();return false;}
    why="SETUP_HOLD_READY";return true;
failed:
    applying=permit=false;return false; /* Do not drop already supported loads. */
}
const char *WheelSetup_Reason(void){return why;}
void WheelSetup_Describe(char *out,size_t size,bool ok){
    int n=snprintf(out,size,"{\"type\":\"setup\",\"ok\":%s,\"pending\":%s,\"token\":%lu,\"reason\":\"%s\",\"failed_slot\":%u,\"wheel\":{\"id\":%u,\"torque\":%d,\"goal\":%d,\"speed\":%d},\"axes\":[",
        ok?"true":"false",pending?"true":"false",(unsigned long)token,why,failed_slot,wheel_id,wheel_torque,wheel_goal,wheel_speed);
    for(unsigned s=0;s<sampled_slots;s++)n+=snprintf(out+n,size-(unsigned)n,"%s{\"slot\":%u,\"position\":%u,\"torque\":%u,\"mode\":%u}",s?",":"",s+1,planned[s].position,planned[s].runtime[0],planned[s].mode);
    (void)snprintf(out+n,size-(unsigned)n,
        "],\"observation\":{\"valid\":%s,\"slot\":%u,\"position\":%u,\"speed_magnitude\":%u,\"torque\":%u,\"mode\":%u}}\r\n",
        observation_valid?"true":"false",observed_slot,last_observation.position,last_observation.speed,last_observation.runtime[0],last_observation.mode);
}
