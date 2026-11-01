/* Temporary left-arm maintenance. No runtime command handler is linked here.
 * The transport independently prohibits torque-enable and position commands.
 * Center operation follows ftservo/FTServo_Arduino SMS_STS::CalibrationOfs.
 */
#include "single_arm_app.h"
#include "servo_bus.h"
#include "single_arm_config.h"
#if HOST_BIMANUAL_CALIBRATION_ONLY
#include "right_servo_bus.h"
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static UART_HandleTypeDef *host;
static char command[64];
static unsigned used;
static int discard;
#if HOST_BIMANUAL_CALIBRATION_ONLY
static unsigned selected_arm, consumed_by_arm[2];
#define consumed consumed_by_arm[selected_arm]
#else
static unsigned consumed;
#endif
static HAL_StatusTypeDef read_data(uint8_t id,uint8_t address,uint8_t length,uint8_t *data) {
#if HOST_BIMANUAL_CALIBRATION_ONLY
    if(selected_arm) return RightServoBus_CalibrationRead(id,address,length,data);
#endif
    return Servo_ReadData(id,address,length,data);
}
static HAL_StatusTypeDef write_value(uint8_t id,uint8_t value) {
    if(id<1||id>6||(value!=0&&value!=128))return HAL_ERROR;
#if HOST_BIMANUAL_CALIBRATION_ONLY
    if(selected_arm) return RightServoBus_CalibrationWrite(id,value);
#endif
    return Servo_WriteData(id,40,&value,1);
}
static void send_line(const char *s) {
    (void)HAL_UART_Transmit(host,(const uint8_t *)s,(uint16_t)strlen(s),100);
}
static int read_value(uint8_t id,uint8_t address,uint8_t length) {
    uint8_t b[2]={0};
    if (read_data(id,address,length,b)!=HAL_OK) return -1;
    return b[0] | (length==2 ? b[1]<<8 : 0);
}
static int write_off(uint8_t id) {
    (void)write_value(id,0);
    HAL_Delay(5);
    return read_value(id,40,1)==0;
}
static int all_off(void) {
    int ok=1;
    for (uint8_t id=1;id<=6;id++) if (!write_off(id)) ok=0;
    return ok;
}
static int eeprom(uint8_t id,uint8_t bytes[40]) {
    return read_data(id,0,16,bytes)==HAL_OK &&
        read_data(id,16,16,bytes+16)==HAL_OK &&
        read_data(id,32,8,bytes+32)==HAL_OK;
}
static void info(void) {
    char line[240];
#if HOST_BIMANUAL_CALIBRATION_ONLY
    snprintf(line,sizeof(line),"{\"firmware\":\"arm-calibration-v1\",\"arm\":\"%s\",\"motion_enabled\":false,\"ids\":[1,2,3,4,5,6],\"consumed_mask\":%u}\r\n",selected_arm?"right":"left",consumed);
#else
    snprintf(line,sizeof(line),"{\"firmware\":\"left-calibration-v1\",\"motion_enabled\":false,\"ids\":[1,2,3,4,5,6],\"consumed_mask\":%u}\r\n",consumed);
#endif
    send_line(line);
}
static void scan(void) {
    for (uint8_t id=1;id<=6;id++) {
        uint8_t data[40]={0};char hex[81],line[400];
        int ok=eeprom(id,data),position=read_value(id,56,2),torque=read_value(id,40,1),lock=read_value(id,55,1);
        for(unsigned i=0;i<40;i++) snprintf(hex+2*i,3,"%02x",data[i]);
        snprintf(line,sizeof(line),"{\"id\":%u,\"ok\":%s,\"position\":%d,\"torque\":%d,\"lock\":%d,\"eeprom_hex\":\"%s\"}\r\n",id,ok&&position>=0&&torque>=0&&lock>=0?"true":"false",position,torque,lock,hex);
        send_line(line);
    }
    send_line("{\"scan_end\":true}\r\n");
}
static int distance(int a,int b) {
    int d=abs(a-b);return d>2048?4096-d:d;
}
static void center(unsigned id,unsigned expected,unsigned expected_offset) {
    const char *reason="invalid_request";int attempted=0,ok=0,before=-1,after=-1,offset_after=-1,off=0;
    uint8_t old[40]={0},fresh[40]={0};
    if(id<1||id>6||expected>4095||expected_offset>65535) goto done;
    if(consumed&(1u<<(id-1))) {reason="already_attempted";goto done;}
    for(uint8_t motor=1;motor<=6;motor++) {
        if(read_value(motor,40,1)!=0) {reason="torque_not_off";goto done;}
    }
    if(!eeprom(id,old)||old[5]!=id||old[33]!=0||
        (unsigned)(old[31]|old[32]<<8)!=expected_offset) {reason="configuration_changed";goto done;}
    before=read_value(id,56,2);HAL_Delay(100);
    int second=read_value(id,56,2);
    if(before<0||before>4095||second<0||second>4095||distance(before,expected)>8||distance(second,expected)>8) {
        reason="pose_changed";goto done;
    }
    consumed|=1u<<(id-1);attempted=1;
    HAL_StatusTypeDef written=write_value(id,128);
    HAL_Delay(500);
    off=write_off(id);after=read_value(id,56,2);
    if(!eeprom(id,fresh)) {reason="readback_failed";goto done;}
    offset_after=fresh[31]|fresh[32]<<8;
    for(unsigned i=0;i<40;i++) if(i!=31&&i!=32&&old[i]!=fresh[i]) {
        reason="unexpected_configuration_change";goto done;
    }
    if(written!=HAL_OK) {reason="write_reply_uncertain";goto done;}
    if(!off||after<2040||after>2056) {reason="center_verification_failed";goto done;}
    ok=1;reason="verified";
done:
    if(attempted&&!ok) off=all_off();
    char line[400];
    snprintf(line,sizeof(line),"{\"id\":%u,\"center_ok\":%s,\"reason\":\"%s\",\"attempted\":%s,\"before\":%d,\"after\":%d,\"offset_after\":%d,\"torque_off_verified\":%s}\r\n",id,ok?"true":"false",reason,attempted?"true":"false",before,after,offset_after,off?"true":"false");
    send_line(line);
}
void SingleArmApp_Init(UART_HandleTypeDef *h,UART_HandleTypeDef *servo) {
    host=h;used=0;discard=0;
#if HOST_BIMANUAL_CALIBRATION_ONLY
    extern UART_HandleTypeDef huart4;
    selected_arm=0;consumed_by_arm[0]=consumed_by_arm[1]=0;
    RightServoBus_Init(&huart4);
#else
    consumed=0;
#endif
    ServoBus_Init(servo,NULL,NULL);info(); /* No boot servo transactions. */
}
void SingleArmApp_Process(void) {
    uint8_t byte;HAL_StatusTypeDef status=HAL_UART_Receive(host,&byte,1,10);
    if(status==HAL_TIMEOUT)return;
    if(status!=HAL_OK){used=0;discard=1;return;}
    if(byte=='\r')return;
    if(byte=='\n') {
        command[used]=0;unsigned id=0,pos=0,offset=0;int end=0;
        const char *input=command;
#if HOST_BIMANUAL_CALIBRATION_ONLY
        if(!strncmp(input,"LEFT ",5)){selected_arm=0;input+=5;}
        else if(!strncmp(input,"RIGHT ",6)){selected_arm=1;input+=6;}
        else discard=1;
#endif
        if(!discard&&!strcmp(input,"INFO"))info();
        else if(!discard&&!strcmp(input,"SCAN"))scan();
        else if(!discard&&!strcmp(input,"OFF"))send_line(all_off()?"{\"off_ok\":true}\r\n":"{\"off_ok\":false}\r\n");
        else if(!discard&&sscanf(input,"CENTER %u %u %u%n",&id,&pos,&offset,&end)==3&&end==(int)strlen(input))center(id,pos,offset);
        else send_line("{\"error\":\"unsupported_command\"}\r\n");
        used=0;discard=0;
    } else if(byte<32||byte>126||used+1>=sizeof(command)){used=0;discard=1;}
    else if(!discard)command[used++]=(char)byte;
}
