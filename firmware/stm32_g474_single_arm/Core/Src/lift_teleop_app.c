/* Isolated ID 10 commissioning. Positive speed/down only, no homing or arm I/O.
 * STOP commands zero velocity and retains torque. Never release a raised load
 * automatically. A broken motor bus still requires operator intervention. */
#include "single_arm_app.h"
#include "servo_bus.h"
#include "host_uart_rx.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#define ID 10u
#define LEASE_MS 400u
#define SAMPLE_MS 20u
#define MAX_GAP_MS 200u
static UART_HandleTypeDef *host;
static bool armed, fault, prepared, velocity_known, stop_confirmed, tracking;
static bool setup_attempted, discard, test_only;
static uint32_t session, sequence, last_command, sample_tick, sample_count;
static uint32_t pulse_start;
static bool pulse_started;
static int position, speed, goal, torque;
static int load_raw, voltage_raw, temperature_c, current_raw, motor_status_raw;
static bool telemetry_valid;
static int32_t total;
static const char *reason;
static char command[40];
static unsigned used;
static uint32_t frame_tick, motor_write_count, nonzero_speed_writes;
static const char *first_fault_reason;
static int checked_mode, checked_torque, failure_register, failure_hal_status;
static bool bus_failure_recorded;
static ServoBusDiagnostics first_bus_failure;
static void capture_bus_failure(uint8_t reg,HAL_StatusTypeDef status){
    if(status==HAL_OK || fault || bus_failure_recorded)return;
    failure_register=reg;failure_hal_status=(int)status;
    first_bus_failure=*ServoBus_GetDiagnostics();bus_failure_recorded=true;
}
static void send(const char *s){(void)HAL_UART_Transmit(host,(const uint8_t *)s,(uint16_t)strlen(s),20);}
static int read_reg(uint8_t reg,uint8_t n){
    uint8_t b[2]={0};HAL_StatusTypeDef status=Servo_ReadData(ID,reg,n,b);
    capture_bus_failure(reg,status);return status==HAL_OK ? b[0]|(b[1]<<8) : -1;
}
static bool write_checked(uint8_t reg,unsigned value,uint8_t n){
    uint8_t b[2]={(uint8_t)value,(uint8_t)(value>>8)};
    motor_write_count++;
    if(reg==46 && value)nonzero_speed_writes++;
    HAL_StatusTypeDef status=Servo_WriteData(ID,reg,b,n);capture_bus_failure(reg,status);
    return status==HAL_OK && read_reg(reg,n)==(int)value;
}
static bool identity(void){
    uint8_t b[4];return Servo_ReadData(ID,3,4,b)==HAL_OK && b[0]==9 && b[1]==3 && b[2]==ID && b[3]==0;
}
static bool sample(void){
    /* Same contiguous 56..70 register map as Servo_ReadTelemetry. Preserve raw
     * units; no motor-specific current conversion or inferred load threshold. */
    uint8_t b[15];telemetry_valid=false;
    HAL_StatusTypeDef status=Servo_ReadData(ID,56,sizeof(b),b);capture_bus_failure(56,status);
    if(status!=HAL_OK){reason="POSITION_READ_FAILED";return false;}
    uint32_t now=HAL_GetTick();int p=b[0]|(b[1]<<8),s=b[2]|(b[3]<<8);
    if(p>4095){reason="POSITION_OUT_OF_RANGE";return false;}
    if(tracking){
        int delta=p-position;if(delta>2048)delta-=4096;if(delta< -2048)delta+=4096;
        if((uint32_t)(now-sample_tick)>MAX_GAP_MS || abs(delta)>256){reason="ENCODER_CONTINUITY_LOST";return false;}
        if(total>10000000 || total< -10000000){reason="ENCODER_RANGE";return false;}
        total+=delta;
        if(goal==100 && total < -16){reason="UNEXPECTED_REVERSE_MOTION";return false;}
    }
    position=p;speed=s;sample_tick=now;sample_count++;
    load_raw=b[4]|(b[5]<<8);voltage_raw=b[6];temperature_c=b[7];
    motor_status_raw=b[9];current_raw=b[13]|(b[14]<<8);telemetry_valid=true;
    return true;
}
static void reply(const char *type){
    char line[1280];
    snprintf(line,sizeof(line),
        "{\"type\":\"%s\",\"armed\":%s,\"fault\":%s,\"prepared\":%s,"
        "\"session\":%lu,\"sequence\":%lu,\"reason\":\"%s\",\"stop_confirmed\":%s,"
        "\"position_raw\":%d,\"speed_raw\":%d,\"goal_raw\":%d,\"torque_raw\":%d,"
        "\"relative_counts\":%ld,\"sample_tick\":%lu,\"sample_count\":%lu,\"test_only\":%s,"
        "\"motor_write_count\":%lu,\"nonzero_speed_writes\":%lu,"
        "\"first_fault_reason\":\"%s\",\"checked_mode\":%d,\"checked_torque\":%d,"
        "\"failure_register\":%d,\"failure_hal_status\":%d,\"bus_failure_reason\":%u,"
        "\"bus_uart_error\":%lu,\"bus_servo_status\":%u,"
        "\"telemetry_valid\":%s,\"telemetry_age_ms\":%lu,\"load_raw\":%d,"
        "\"voltage_raw\":%d,\"temperature_c\":%d,\"current_raw\":%d,\"motor_status_raw\":%d}\r\n",
        type,armed?"true":"false",fault?"true":"false",prepared?"true":"false",
        (unsigned long)session,(unsigned long)sequence,reason,stop_confirmed?"true":"false",
        position,speed,goal,torque,(long)total,(unsigned long)sample_tick,
        (unsigned long)sample_count,test_only?"true":"false",
        (unsigned long)motor_write_count,(unsigned long)nonzero_speed_writes,
        first_fault_reason,checked_mode,checked_torque,failure_register,failure_hal_status,
        (unsigned)first_bus_failure.reason,(unsigned long)first_bus_failure.uart_error_code,
        (unsigned)first_bus_failure.servo_status,telemetry_valid?"true":"false",
        (unsigned long)(telemetry_valid?HAL_GetTick()-sample_tick:UINT32_MAX),
        load_raw,voltage_raw,temperature_c,current_raw,motor_status_raw);send(line);
}
/* Stop readback is fresh and bounded; failed samples are never used to repair
 * the calibration trace. No torque-disable during STOP, including on timeout. */
static void halt(bool latch,const char *why){
    if(latch && !fault)first_fault_reason=why;
    armed=false;fault|=latch;reason=why;stop_confirmed=false;
    if(velocity_known){int mode=read_reg(33,1);if(mode>=0 && mode!=1)velocity_known=false;}
    if(!velocity_known){reply(latch?"fault":"stopped");return;}
    bool sent=write_checked(46,0,2);goal=read_reg(46,2);torque=read_reg(40,1);
    uint32_t begin=HAL_GetTick();int previous=-1;unsigned quiet=0;
    while((uint32_t)(HAL_GetTick()-begin)<350u){
        if(!sample()){fault=true;tracking=false;quiet=0;HAL_Delay(20);continue;}
        int delta=position-previous;if(delta>2048)delta-=4096;if(delta< -2048)delta+=4096;
        quiet=((speed&0x7fff)==0 && previous>=0 && abs(delta)<=2)?quiet+1:0;
        previous=position;if(quiet>=3)break;HAL_Delay(20);
    }
    stop_confirmed=sent && goal==0 && torque==1 && quiet>=3;
    reason=why;
    if(!stop_confirmed){fault=true;reason="STOP_UNCONFIRMED_SUPPORT_AND_CUT_POWER";}
    reply(latch?"fault":"stopped");
}
static bool poll_safety(void){
    if(!armed)return false;
    if(HostUartRx_TakeFault()){
        used=0;discard=true;halt(true,"HOST_RX_ERROR");return false;
    }
    if((uint32_t)(HAL_GetTick()-last_command)>=LEASE_MS){halt(true,"HOST_TIMEOUT");return false;}
    if(test_only && pulse_started && (uint32_t)(HAL_GetTick()-pulse_start)>=150u){halt(false,"TEST_PULSE_COMPLETE");return false;}
    if((uint32_t)(HAL_GetTick()-sample_tick)>=SAMPLE_MS){
        if(!sample()){const char *why=reason;tracking=false;halt(true,why);return false;}
    }
    return armed;
}
static void snapshot(void){
    if(armed){halt(true,"SNAPSHOT_WHILE_ARMED");return;}
    telemetry_valid=false;
    uint8_t b[40];bool ok=true;
    for(unsigned offset=0;offset<40;offset+=16){
        unsigned n=40-offset;if(n>16)n=16;
        if(Servo_ReadData(ID,(uint8_t)offset,(uint8_t)n,b+offset)!=HAL_OK){ok=false;break;}
    }
    torque=read_reg(40,1);goal=read_reg(46,2);
    tracking=false;
    if(!ok || !sample()){reason="SNAPSHOT_FAILED";reply("error");return;}
    char line[768];int n=snprintf(line,sizeof(line),"{\"type\":\"snapshot\",\"id\":10,\"torque\":%d,\"goal\":%d,\"position\":%d,\"speed\":%d,\"lock\":%d,\"eeprom\":[",torque,goal,position,speed,read_reg(55,1));
    for(unsigned i=0;i<40;i++)n+=snprintf(line+n,sizeof(line)-(unsigned)n,"%s%u",i?",":"",b[i]);
    snprintf(line+n,sizeof(line)-(unsigned)n,"],\"telemetry_valid\":%s,\"sample_tick\":%lu,\"telemetry_age_ms\":%lu,\"load_raw\":%d,\"voltage_raw\":%d,\"temperature_c\":%d,\"current_raw\":%d,\"motor_status_raw\":%d}\r\n",
        telemetry_valid?"true":"false",(unsigned long)sample_tick,
        (unsigned long)(HAL_GetTick()-sample_tick),load_raw,voltage_raw,temperature_c,current_raw,motor_status_raw);send(line);
}
static void prepare(void){
    if(armed){halt(true,"PREPARE_WHILE_ARMED");return;}
    if(fault || setup_attempted){reason="PREPARE_NOT_ALLOWED";reply("error");return;}
    setup_attempted=true;
    int mode=read_reg(33,1);
    if(!identity() || (mode!=0 && mode!=1) || read_reg(40,1)!=0 || read_reg(55,1)!=1){
        fault=true;reason="PREPARE_ID_MODE_TORQUE_LOCK";reply("error");return;
    }
    bool ok=true;
    if(mode==0){
        ok=write_checked(55,0,1);
        if(ok){ok=write_checked(33,1,1);HAL_Delay(20);}
        /* Always relock after even an uncertain unlock or mode write. */
        bool locked=write_checked(55,1,1);ok=ok&&locked;
    }
    velocity_known=read_reg(33,1)==1;
    if(!ok || !velocity_known){fault=true;reason="MODE_CHANGE_FAILED";reply("error");return;}
    /* No position-mode goal write. Zero first in verified speed mode, then OFF:
     * actual servos can enable torque as a side effect of a goal write. */
    ok=write_checked(46,0,2);
    bool off=write_checked(40,0,1);
    prepared=ok&&off&&read_reg(55,1)==1;
    fault=!prepared;reason=prepared?"PREPARED_TORQUE_OFF":"PREPARE_ZERO_OFF_FAILED";
    goal=read_reg(46,2);torque=read_reg(40,1);reply(prepared?"prepared":"error");
}
static void enable(uint32_t nonce,bool test){
    if(armed){halt(true,"DUPLICATE_ARM");return;}
    if(fault){reply("fault");return;}
    bus_failure_recorded=false;memset(&first_bus_failure,0,sizeof(first_bus_failure));
    failure_register=failure_hal_status=checked_mode=checked_torque=-1;
    first_fault_reason="NONE";
    /* A separate prepare process may have exited or MCU rebooted. Re-establish
     * mode and stationary zero goal before ANY enabling/drive write. */
    if(!identity() || read_reg(33,1)!=1 || read_reg(55,1)!=1 || read_reg(46,2)!=0){reason="ARM_ID_MODE_LOCK_GOAL";reply("error");return;}
    torque=read_reg(40,1);
    if(torque!=0 && torque!=1){reason="ARM_TORQUE_READ";reply("error");return;}
    tracking=false;
    if(!sample() || (speed&0x7fff)!=0){reason="ARM_NOT_STATIONARY";reply("error");return;}
    int anchor=position;HAL_Delay(40);
    if(!sample() || (speed&0x7fff)!=0 || abs(position-anchor)>2){reason="ARM_NOT_STATIONARY";reply("error");return;}
    prepared=velocity_known=true;
    if(!write_checked(40,1,1)){halt(true,"TORQUE_ENABLE_FAILED");return;}
    torque=1;goal=0;total=0;tracking=true;sample_tick=HAL_GetTick();
    session=nonce;sequence=0;test_only=test;pulse_started=false;
    armed=true;stop_confirmed=false;last_command=HAL_GetTick();reason="ARMED_ZERO";reply("armed");
}
static uint16_t crc16(const char *p,unsigned n){
    uint16_t c=0xffff;for(unsigned i=0;i<n;i++){c^=(uint16_t)(uint8_t)p[i]<<8;
        for(unsigned j=0;j<8;j++)c=(c&0x8000)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);}
    return c;
}
static bool hex(const char *p,unsigned n,uint32_t *v){
    *v=0;for(unsigned i=0;i<n;i++){unsigned c=(unsigned char)p[i],d;
        if(c>='0'&&c<='9')d=c-'0';else if(c>='A'&&c<='F')d=c-'A'+10;else return false;
        *v=(*v<<4)|d;}return true;
}
static bool frame_crc(unsigned prefix,unsigned length){
    uint32_t c;return used==length && command[prefix]==' ' && hex(command+prefix+1,4,&c) && c==crc16(command,prefix);
}
static void drive(uint32_t nonce,uint32_t seq,char motion){
    if(!poll_safety()){if(!armed)reply("inactive");return;}
    if(nonce!=session || sequence==UINT32_MAX || seq!=sequence+1 || (motion!='D' && motion!='Z')){halt(true,"SESSION_SEQUENCE_OR_MOTION");return;}
    /* IRQ arrival time prevents queued traffic from renewing a dead link. */
    last_command=frame_tick;sequence=seq;
    checked_torque=-1;checked_mode=read_reg(33,1);
    if(checked_mode<0){halt(true,"MODE_READ_FAILED");return;}
    if(checked_mode!=1){halt(true,"MODE_CHANGED");return;}
    checked_torque=read_reg(40,1);
    if(checked_torque<0){halt(true,"TORQUE_READ_FAILED");return;}
    if(checked_torque!=1){halt(true,"TORQUE_CHANGED");return;}
    if(!poll_safety())return;
    int requested=motion=='D'?100:0;
    if(requested!=goal){
        /* Start the test timer before the first possible nonzero write. */
        if(requested && !pulse_started){pulse_started=true;pulse_start=HAL_GetTick();}
        if(!write_checked(46,(unsigned)requested,2)){halt(true,"SPEED_WRITE_READBACK");return;}
        goal=requested;
    }
    if(!poll_safety())return;
    if(!sample()){const char *why=reason;tracking=false;halt(true,why);return;}
    reason=requested?"DOWN":"ZERO";reply("ack");
}
static void dispatch(void){
    uint32_t nonce,seq;
    if(!strcmp(command,"STOP")){halt(false,"OPERATOR_STOP");return;}
    if(!strcmp(command,"INFO")){
        send("{\"firmware\":\"lift-teleop-v4\",\"host_baud\":921600,\"id\":10,\"max_raw\":100,\"watchdog_ms\":400,\"sample_gap_ms\":200,\"down_only\":true,\"stop_keeps_torque\":true}\r\n");return;
    }
    if(!strcmp(command,"STATUS")){reply("status");return;}
    if((uint32_t)(HAL_GetTick()-frame_tick)>=LEASE_MS){
        if(armed)halt(true,"STALE_HOST_FRAME");else{reason="STALE_HOST_FRAME";reply("error");}return;
    }
    /* No motor I/O. Deliberate processing latency exercises IRQ reception while
     * the main loop is busy, including on a powered-off assembled controller. */
    if(!memcmp(command,"ECHO ",5) && frame_crc(13,18) && hex(command+5,8,&nonce)){
        if(armed){halt(true,"LINK_CHECK_WHILE_ARMED");return;}
        HAL_Delay(25);char line[128];
        snprintf(line,sizeof(line),"{\"type\":\"echo\",\"nonce\":%lu,\"motor_write_count\":%lu}\r\n",(unsigned long)nonce,(unsigned long)motor_write_count);
        send(line);return;
    }
    if(!strcmp(command,"SNAP")){snapshot();return;}
    if(!memcmp(command,"PREP ",5) && frame_crc(13,18) && hex(command+5,8,&nonce) && nonce){prepare();return;}
    if(!memcmp(command,"ARM ",4) && frame_crc(12,17) && hex(command+4,8,&nonce) && nonce){enable(nonce,false);return;}
    if(!memcmp(command,"TEST ",5) && frame_crc(13,18) && hex(command+5,8,&nonce) && nonce){enable(nonce,true);return;}
    if(used==26 && !memcmp(command,"D ",2) && command[10]==' ' && command[19]==' ' && frame_crc(21,26) && hex(command+2,8,&nonce) && hex(command+11,8,&seq)){drive(nonce,seq,command[20]);return;}
    if(armed)halt(true,"INVALID_FRAME");else{reason="INVALID_FRAME";reply("error");}
}
void SingleArmApp_Init(UART_HandleTypeDef *h,UART_HandleTypeDef *servo){
    host=h;armed=fault=prepared=velocity_known=stop_confirmed=tracking=false;
    setup_attempted=discard=test_only=pulse_started=false;
    session=sequence=last_command=sample_tick=sample_count=pulse_start=0;
    position=speed=goal=torque=-1;total=0;used=0;reason="BOOT";
    load_raw=voltage_raw=temperature_c=current_raw=motor_status_raw=-1;telemetry_valid=false;
    frame_tick=motor_write_count=nonzero_speed_writes=0;
    first_fault_reason="NONE";checked_mode=checked_torque=failure_register=failure_hal_status=-1;
    bus_failure_recorded=false;memset(&first_bus_failure,0,sizeof(first_bus_failure));
    ServoBus_Init(servo,NULL,NULL); /* no register writes or reads at boot */
    HostUartRx_Init(host);
    if(HostUartRx_Start()!=HAL_OK){fault=true;reason="HOST_RX_START_FAILED";reply("fault");}
}
void SingleArmApp_Process(void){
    (void)poll_safety();
    if(HostUartRx_TakeFault()){
        used=0;discard=true;fault=true;reason="HOST_RX_ERROR";reply("fault");
    }
    /* The IRQ ring receives complete commands while servo reads/STOP block.
     * Polling the one-byte hardware register here used to drop bytes at 921600. */
    for(unsigned processed=0;processed<64;processed++){
        uint8_t c;uint32_t received_at;
        if(!HostUartRx_Pop(&c,&received_at))break;
        (void)poll_safety();
        if(c=='\r')continue;
        if(c=='\n'){
            command[used]=0;frame_tick=received_at;
            if(!discard)dispatch();
            used=0;discard=false;
        }else if(c<32 || c>126 || used+1>=sizeof(command)){
            used=0;discard=true;if(armed)halt(true,"INVALID_FRAME");
        }else if(!discard)command[used++]=(char)c;
        (void)poll_safety();
        if(HostUartRx_TakeFault()){
            used=0;discard=true;fault=true;reason="HOST_RX_ERROR";reply("fault");
        }
    }
}
