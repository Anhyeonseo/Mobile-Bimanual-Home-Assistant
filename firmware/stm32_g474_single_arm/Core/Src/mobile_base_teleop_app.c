/* Manual base only. Blocking servo transactions remain individually bounded;
 * watchdog checks run between them. MCU/bus failure requires motor power cut. */
#include "single_arm_app.h"
#include "servo_bus.h"
#include "actuator_core/mobile_ids.h"
#include "mobile_bench_patterns.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#define LEASE_MS 400u
static UART_HandleTypeDef *host;
static bool armed, fault, stop_confirmed;
static uint32_t session, sequence, last_command;
static char command[40];
static unsigned used;
static bool discard;
static const char *reason="BOOT", *first_failure="NONE";
static int first_failed_id;
typedef struct {int goal,torque,speed,position;unsigned quiet;} WheelEvidence;
static WheelEvidence evidence[3];
/* Preserve the first anomalous observation across cleanup; it never qualifies
 * as fresh standstill evidence. A recovered read is diagnostic, not a fault. */
typedef struct {
    bool valid; const char *phase; WheelEvidence wheel;
    int id, failed_register, hal_status; uint32_t elapsed_ms;
} FirstObservation;
static FirstObservation first_observation;
static uint32_t wait_elapsed_ms, first_failure_wait_ms;
static void observe_problem(const char *phase,uint8_t id,const WheelEvidence *e,
                            int reg,int status,uint32_t elapsed){
    if(!first_observation.valid)
        first_observation=(FirstObservation){true,phase,*e,id,reg,status,elapsed};
}
static int signed_speed(int raw){return raw<0 ? 0 : (raw&0x8000)?-(raw&0x7fff):raw;}
static bool speed_zero(int raw){return raw>=0 && (raw&0x7fff)==0;}
static int bad_id, observed_speed=-1, speed_status=-1;
static void send(const char *s){(void)HAL_UART_Transmit(host,(const uint8_t *)s,(uint16_t)strlen(s),20);}
static int read_reg(uint8_t id,uint8_t addr,uint8_t n){
    uint8_t v[2]={0};return Servo_ReadData(id,addr,n,v)==HAL_OK ? v[0]|(v[1]<<8) : -1;
}
static bool write_checked(uint8_t id,uint8_t addr,const uint8_t *v,uint8_t n){
    uint8_t actual[2]={0};
    return Servo_WriteData(id,addr,v,n)==HAL_OK &&
        Servo_ReadData(id,addr,n,actual)==HAL_OK && !memcmp(v,actual,n);
}
/* Read-only recovery is bounded to the original window, with torque off.
 * Any invalid sweep resets all counters. Never retry a drive write here. */
static bool wait_stopped(const char *phase){
    uint32_t start=HAL_GetTick();wait_elapsed_ms=0;
    for(unsigned i=0;i<3;i++)evidence[i].quiet=0;
    do{
        bool all=true,valid_round=true;
        for(unsigned i=0;i<3;i++){
            uint8_t id=actuator_mobile_axis_id(i);WheelEvidence *e=&evidence[i];
            const uint8_t addresses[4]={46,40,58,56},sizes[4]={2,1,2,2};
            int values[4],statuses[4],failed_reg=0,failed_status=HAL_OK;
            for(unsigned j=0;j<4;j++){
                uint8_t v[2]={0};
                statuses[j]=Servo_ReadData(id,addresses[j],sizes[j],v);
                values[j]=statuses[j]==HAL_OK ? v[0]|(v[1]<<8) : -1;
                if(statuses[j]!=HAL_OK&&!failed_reg){failed_reg=addresses[j];failed_status=statuses[j];}
            }
            e->goal=values[0];e->torque=values[1];e->speed=values[2];e->position=values[3];
            bool quiet=!failed_reg&&speed_zero(e->goal)&&e->torque==0&&speed_zero(e->speed);
            e->quiet=quiet ? e->quiet+1 : 0;
            if(!quiet)observe_problem(phase,id,e,failed_reg,failed_status,(uint32_t)(HAL_GetTick()-start));
            if(failed_reg)valid_round=false;
            if(e->quiet<3){if(all){bad_id=id;observed_speed=e->speed;speed_status=statuses[2];}all=false;}
        }
        if(!valid_round)for(unsigned i=0;i<3;i++)evidence[i].quiet=0;
        wait_elapsed_ms=(uint32_t)(HAL_GetTick()-start);
        if(wait_elapsed_ms>=1000u)break;
        if(all&&valid_round)return true;
        HAL_Delay(50);
    }while((uint32_t)(HAL_GetTick()-start)<1000u);
    wait_elapsed_ms=(uint32_t)(HAL_GetTick()-start);
    return false;
}
static void reply(const char *type){
    char line[2048];
    int n=snprintf(line,sizeof(line),
        "{\"type\":\"%s\",\"armed\":%s,\"fault\":%s,\"session\":%lu,\"sequence\":%lu,"
        "\"reason\":\"%s\",\"first_failure\":\"%s\",\"first_failed_id\":%d,"
        "\"stop_confirmed\":%s,\"failed_id\":%d,\"speed_raw\":%d,\"speed_hal_status\":%d",
        type,armed?"true":"false",fault?"true":"false",(unsigned long)session,
        (unsigned long)sequence,reason,first_failure,first_failed_id,
        stop_confirmed?"true":"false",bad_id,observed_speed,speed_status);
    if(strcmp(type,"ack")){
        n+=snprintf(line+n,sizeof(line)-(unsigned)n,
            ",\"wait_elapsed_ms\":%lu,\"first_failure_wait_ms\":%lu",
            (unsigned long)wait_elapsed_ms,(unsigned long)first_failure_wait_ms);
        FirstObservation *o=&first_observation;
        if(o->valid){
            n+=snprintf(line+n,sizeof(line)-(unsigned)n,
                ",\"first_observation\":{\"phase\":\"%s\",\"id\":%d,"
                "\"goal_raw\":%d,\"torque_raw\":%d,\"speed_raw\":%d,"
                "\"speed_signed\":%d,\"speed_valid\":%s,\"position_raw\":%d,"
                "\"failed_register\":%d,\"hal_status\":%d,\"elapsed_ms\":%lu}",
                o->phase,o->id,o->wheel.goal,o->wheel.torque,o->wheel.speed,
                signed_speed(o->wheel.speed),o->wheel.speed>=0?"true":"false",o->wheel.position,
                o->failed_register,o->hal_status,(unsigned long)o->elapsed_ms);
        }
        n+=snprintf(line+n,sizeof(line)-(unsigned)n,",\"wheels\":[");
        for(unsigned i=0;i<3;i++){
            WheelEvidence *e=&evidence[i];
            n+=snprintf(line+n,sizeof(line)-(unsigned)n,
                "%s{\"id\":%u,\"goal_raw\":%d,\"torque_raw\":%d,\"speed_raw\":%d,"
                "\"speed_signed\":%d,\"speed_valid\":%s,\"position_raw\":%d,\"quiet_samples\":%u}",
                i?",":"",(unsigned)actuator_mobile_axis_id(i),e->goal,e->torque,e->speed,
                signed_speed(e->speed),e->speed>=0?"true":"false",e->position,e->quiet);
        }
        n+=snprintf(line+n,sizeof(line)-(unsigned)n,"]");
    }
    (void)snprintf(line+n,sizeof(line)-(unsigned)n,"}\r\n");send(line);
}
static void stop_all(bool latch,const char *why){
    if(latch&&!strcmp(first_failure,"NONE")){first_failure=why;first_failed_id=bad_id;first_failure_wait_ms=wait_elapsed_ms;}
    armed=false;fault=fault||latch;reason=why;stop_confirmed=false;
    const uint8_t zero[2]={0};bool sent=true;
    for(unsigned i=0;i<3;i++)if(Servo_WriteData(actuator_mobile_axis_id(i),46,zero,2)!=HAL_OK)sent=false;
    for(unsigned i=0;i<3;i++)if(Servo_WriteData(actuator_mobile_axis_id(i),40,zero,1)!=HAL_OK)sent=false;
    bool quiet=wait_stopped("STOP");
    stop_confirmed=sent&&quiet;
    if(!stop_confirmed){
        fault=true;reason="STOP_UNCONFIRMED_CUT_MOTOR_POWER";
        if(!strcmp(first_failure,"NONE")){first_failure=reason;first_failed_id=bad_id;first_failure_wait_ms=wait_elapsed_ms;}
    }
}
static bool lease_ok(void){
    if(armed && (uint32_t)(HAL_GetTick()-last_command)>=LEASE_MS){
        stop_all(true,"HOST_TIMEOUT");reply("fault");return false;
    }
    return armed;
}
static void fail(const char *why){stop_all(true,why);reply("fault");}
static uint16_t crc16(const char *p,unsigned n){
    uint16_t crc=0xffff;
    for(unsigned i=0;i<n;i++){
        crc^=(uint16_t)(uint8_t)p[i]<<8;
        for(unsigned j=0;j<8;j++)crc=(crc&0x8000)?(uint16_t)((crc<<1)^0x1021):(uint16_t)(crc<<1);
    }
    return crc;
}
static bool hex(const char *p,unsigned n,uint32_t *out){
    uint32_t v=0;
    for(unsigned i=0;i<n;i++){
        unsigned c=(unsigned char)p[i],digit;
        if(c>='0'&&c<='9')digit=c-'0';
        else if(c>='A'&&c<='F')digit=c-'A'+10;
        else return false;
        v=(v<<4)|digit;
    }
    *out=v;return true;
}
static bool frame_crc(unsigned prefix,unsigned total){
    uint32_t v;
    return used==total && command[prefix]==' ' && hex(command+prefix+1,4,&v) && v==crc16(command,prefix);
}
static void enable(uint32_t nonce){
    if(armed){fail("DUPLICATE_ARM");return;}
    if(fault){reply("fault");return;}
    bad_id=0;observed_speed=-1;speed_status=-1;stop_confirmed=false;
    first_failure="NONE";first_failed_id=0;
    first_observation=(FirstObservation){0};wait_elapsed_ms=first_failure_wait_ms=0;
    for(unsigned i=0;i<3;i++){
        uint8_t id=actuator_mobile_axis_id(i),ident[4];bad_id=id;
        if(Servo_ReadData(id,3,4,ident)!=HAL_OK || ident[0]!=9 || ident[1]!=3 || ident[2]!=id || ident[3]!=0){fail("IDENTITY");return;}
        if(read_reg(id,33,1)!=1 || read_reg(id,40,1)!=0 || read_reg(id,55,1)!=1){fail("MODE_TORQUE_LOCK");return;}
    }
    const uint8_t zero[2]={0},one=1;
    for(unsigned i=0;i<3;i++){
        bad_id=actuator_mobile_axis_id(i);
        if(!write_checked((uint8_t)bad_id,46,zero,2)){fail("ZERO_BEFORE_ENABLE");return;}
        /* A zero goal write does not prove torque remains disabled. Hardware
         * reported torque=1 afterward. Establish and verify OFF explicitly. */
        if(!write_checked((uint8_t)bad_id,40,zero,1)){fail("TORQUE_OFF_BEFORE_SETTLE");return;}
    }
    /* No further goal writes before the torque-off standstill observations.
     * Enable only after all axes produce fresh, repeated zero-speed evidence. */
    if(!wait_stopped("STARTUP")){fail("STARTUP_NOT_QUIET_OR_READ_FAILED");return;}
    for(unsigned i=0;i<3;i++){
        bad_id=actuator_mobile_axis_id(i);
        if(!write_checked((uint8_t)bad_id,40,&one,1)){fail("TORQUE_ENABLE");return;}
    }
    session=nonce;sequence=0;last_command=HAL_GetTick();armed=true;bad_id=0;reason="ARMED_ZERO";
    reply("armed");
}
static void drive(uint32_t nonce,uint32_t seq,char motion){
    int speeds[3]={0};
    if(!armed || fault){reply("fault");return;}
    if(!lease_ok())return;
    if(nonce!=session || sequence==UINT32_MAX || seq!=sequence+1){fail("SESSION_OR_SEQUENCE");return;}
    if(motion!='Z'&&!MobileBench_Pattern(motion,speeds)){fail("INVALID_MOTION");return;}
    /* Timestamp reception, never the end of slow bus transactions. */
    last_command=HAL_GetTick();sequence=seq;stop_confirmed=false;
    for(unsigned i=0;i<3;i++){
        if(!lease_ok())return;
        uint8_t id=actuator_mobile_axis_id(i);bad_id=id;
        uint16_t raw=speeds[i]<0?(uint16_t)(-speeds[i])|0x8000u:(uint16_t)speeds[i];
        uint8_t v[2]={(uint8_t)raw,(uint8_t)(raw>>8)};
        if(!write_checked(id,46,v,2)){fail("SPEED_WRITE_READBACK");return;}
        if(!lease_ok())return;
        if(read_reg(id,40,1)!=1){fail("TORQUE_LOST");return;}
    }
    if(!lease_ok())return;
    bad_id=0;reason=motion=='Z'?"ZERO_COMMAND":"DRIVE";reply("ack");
}
static void dispatch(void){
    uint32_t nonce,seq;
    if(!strcmp(command,"STOP")){stop_all(false,"OPERATOR_STOP");reply("stopped");return;}
    if(!strcmp(command,"INFO")){
        send("{\"firmware\":\"mobile-base-teleop-v4\",\"host_baud\":921600,\"servo_baud\":1000000,\"ids\":[7,8,9,10],\"max_raw\":200,\"watchdog_ms\":400}\r\n");return;
    }
    if(used==17 && !memcmp(command,"ARM ",4) && frame_crc(12,17) && hex(command+4,8,&nonce) && nonce){enable(nonce);return;}
    if(used==26 && !memcmp(command,"D ",2) && command[10]==' ' && command[19]==' ' &&
       frame_crc(21,26) && hex(command+2,8,&nonce) && hex(command+11,8,&seq)){
        drive(nonce,seq,command[20]);return;
    }
    if(armed)fail("INVALID_FRAME");else{reason="INVALID_FRAME";reply("error");}
}
void SingleArmApp_Init(UART_HandleTypeDef *h,UART_HandleTypeDef *servo){
    host=h;armed=false;fault=false;stop_confirmed=false;used=0;discard=false;
    session=sequence=last_command=0;reason="BOOT";bad_id=0;observed_speed=speed_status=-1;
    first_failure="NONE";first_failed_id=0;
    first_observation=(FirstObservation){0};wait_elapsed_ms=first_failure_wait_ms=0;
    for(unsigned i=0;i<3;i++)evidence[i]=(WheelEvidence){-1,-1,-1,-1,0};
    ServoBus_Init(servo,NULL,NULL); /* no motor writes at boot */
}
void SingleArmApp_Process(void){
    (void)lease_ok();
    uint8_t c;HAL_StatusTypeDef status=HAL_UART_Receive(host,&c,1,5);
    (void)lease_ok();
    if(status==HAL_TIMEOUT)return;
    if(status!=HAL_OK){used=0;discard=true;if(armed)fail("HOST_RX_ERROR");return;}
    if(c=='\r')return;
    if(c=='\n'){
        command[used]=0;
        if(!discard)dispatch();
        used=0;discard=false;
    }else if(c<32||c>126||used+1>=sizeof(command)){
        used=0;discard=true;if(armed)fail("INVALID_FRAME");
    }else if(!discard)command[used++]=(char)c;
}
