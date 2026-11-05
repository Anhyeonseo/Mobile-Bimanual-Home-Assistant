/* Isolated S2 acquisition. No servo writes or discovery at boot. Transport
 * independently rejects all writes. A host nonce is lost on reset, so old
 * readers cannot silently continue after a controller restart. */
#include "single_arm_app.h"
#include "servo_bus.h"
#include "right_servo_bus.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
static UART_HandleTypeDef *host;
static char input[64];
static unsigned used, discard;
static uint32_t epoch, sequence;
static void send(const char *s) {
    (void)HAL_UART_Transmit(host,(const uint8_t *)s,(uint16_t)strlen(s),100);
}
static void info(void) {
    char out[180];
    snprintf(out,sizeof(out),"{\"firmware\":\"s2-observer-v1\",\"epoch\":%lu,\"sequence\":%lu,\"tick_ms\":%lu,\"motor_writes\":0}\r\n",(unsigned long)epoch,(unsigned long)sequence,(unsigned long)HAL_GetTick());
    send(out);
}
static HAL_StatusTypeDef read_data(unsigned bus,uint8_t id,uint8_t address,uint8_t n,uint8_t *data) {
    return bus ? RightServoBus_CalibrationRead(id,address,n,data) : Servo_ReadData(id,address,n,data);
}
static void sample(void) {
    if (!epoch || sequence==UINT32_MAX) {send("{\"error\":\"bind_required\"}\r\n");return;}
    sequence++;
    char out[230];
    for(unsigned index=0;index<13;index++) {
        unsigned bus=index>=7;uint8_t id=index<6?(uint8_t)(index+1):index==6?10:(uint8_t)(index-6);
        uint8_t data[4]={0},torque=0;
        uint32_t start=HAL_GetTick();
        int ok=read_data(bus,id,56,4,data)==HAL_OK;
        uint32_t end=HAL_GetTick();
        if(ok)ok=read_data(bus,id,40,1,&torque)==HAL_OK;
        int position=ok?(data[0]|data[1]<<8):-1;
        if(position>4095||torque>1)ok=0;
        snprintf(out,sizeof(out),"{\"epoch\":%lu,\"sequence\":%lu,\"bus\":%u,\"id\":%u,\"start_ms\":%lu,\"end_ms\":%lu,\"position\":%d,\"speed\":%d,\"torque\":%d,\"ok\":%s}\r\n",
          (unsigned long)epoch,(unsigned long)sequence,bus,id,(unsigned long)start,(unsigned long)end,
          position,ok?(data[2]|data[3]<<8):-1,ok?torque:-1,ok?"true":"false");
        send(out);
        if(!ok) {send("{\"error\":\"motor_read_failed\"}\r\n");epoch=0;return;}
    }
    snprintf(out,sizeof(out),"{\"sample_end\":true,\"epoch\":%lu,\"sequence\":%lu,\"tick_ms\":%lu}\r\n",(unsigned long)epoch,(unsigned long)sequence,(unsigned long)HAL_GetTick());
    send(out);
}
void SingleArmApp_Init(UART_HandleTypeDef *h,UART_HandleTypeDef *servo) {
    extern UART_HandleTypeDef huart4;
    host=h;used=discard=0;epoch=sequence=0;
    ServoBus_Init(servo,NULL,NULL);RightServoBus_Init(&huart4);info();
}
void SingleArmApp_Process(void) {
    uint8_t c;
    HAL_StatusTypeDef status=HAL_UART_Receive(host,&c,1,10);
    if(status==HAL_TIMEOUT)return;
    if(status!=HAL_OK){discard=1;used=0;epoch=0;return;}
    if(c=='\r')return;
    if(c=='\n') {
        input[used]=0;unsigned long token=0;int end=0;
        if(!discard&&!strcmp(input,"INFO"))info();
        else if(!discard&&sscanf(input,"BIND %lu%n",&token,&end)==1&&end==(int)strlen(input)&&token>0&&token<=UINT32_MAX&&input[5]>='0'&&input[5]<='9') {
            epoch=(uint32_t)token;sequence=0;info();
        } else if(!discard&&!strcmp(input,"SAMPLE"))sample();
        else send("{\"error\":\"unsupported_command\"}\r\n");
        used=discard=0;
    } else if(c<32||c>126||used+1>=sizeof(input)){discard=1;used=0;}
    else if(!discard)input[used++]=(char)c;
}
