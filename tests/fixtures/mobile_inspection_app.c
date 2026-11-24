#include "single_arm_app.h"
#include "servo_bus.h"
#include "right_servo_bus.h"
#include "actuator_core/device_inventory.h"
#include <assert.h>
#include <string.h>
static UART_HandleTypeDef host, servo;
UART_HandleTypeDef huart4;
static int right_initialized, fail_right;
void RightServoBus_Init(UART_HandleTypeDef *u) {assert(u==&huart4);right_initialized=1;}
static const unsigned char *input;
static unsigned remaining, reads;
uint32_t HAL_GetTick(void) {static uint32_t tick;return tick++;}
static int fail_identity, fail_reference, skip_service;
static ServoBusHealth health;
static ServoBusDiagnostics diag;
const ServoBusHealth *ServoBus_GetHealth(void) { return &health; }
const ServoBusDiagnostics *ServoBus_GetDiagnostics(void) { return &diag; }
static char output[4096];
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t t) {
    (void)t;assert(u==&host&&n==1);
    if(!remaining)return HAL_TIMEOUT;
    *p=*input++;remaining--;return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t) {
    (void)t;assert(u==&host);assert(strlen(output)+n<sizeof(output));
    strncat(output,(const char *)p,n);return HAL_OK;
}
void ServoBus_Init(UART_HandleTypeDef *u,ServoStopRequestedFn a,ServoReadFailureFn b) {
    assert(u==&servo&&a==NULL&&b==NULL);
}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t address,uint8_t length,uint8_t *data) {
    assert(id>=1&&id<=10);reads++;
    if(skip_service)return HAL_BUSY;
    health.transaction_count++;
    memset(&diag,0,sizeof(diag));
    assert((address==3&&length==4)||(address==33&&length==1)||
           (address==40&&length==1)||(address==56&&(length==2||length==4))||
           (address==0&&length==7)||(address==9&&length==4)||
           (address==31&&length==3)||(address==40&&length==10)||(address==55&&length==16));
    if((fail_identity|| (fail_reference&&id==1))&&address==3) {
        diag.reason=SERVO_BUS_FAILURE_RX_TIMEOUT;return HAL_TIMEOUT;
    }
    memset(data,0,length);
    if(address==3){data[0]=9;data[1]=3;data[2]=id;}
    if(address==0){data[3]=9;data[4]=3;data[5]=id;}
    return HAL_OK;
}
RightServoReadStatus RightServoBus_ReadInventoryBlock(uint8_t id,unsigned block,uint8_t *data) {
    assert(right_initialized&&id>=1&&id<=6&&block<5);
    if(fail_right)return RIGHT_SERVO_READ_RX_TIMEOUT;
    return Servo_ReadData(id,device_inventory_address(block),device_inventory_length(block),data)==HAL_OK?RIGHT_SERVO_READ_OK:RIGHT_SERVO_READ_TX;
}
RightServoReadStatus RightServoBus_ReadTuningBlock(uint8_t id,unsigned block,uint8_t *data) {
    (void)id;(void)block;(void)data;assert(0);return RIGHT_SERVO_READ_UNAVAILABLE;
}
static void feed(const char *text) {
    input=(const unsigned char *)text;remaining=strlen(text);
    while(remaining)SingleArmApp_Process();
}
int main(void) {
    SingleArmApp_Init(&host,&servo);
    assert(reads==0&&strstr(output,"mobile-bus-inspection-v2"));output[0]=0;
    feed("INFO\n");assert(reads==0);output[0]=0;
    feed("ARM\nP\nM\nSCANxxxxxxxxxxxxxxxx\nSCAN\001\nSCANN\n");
    assert(reads==0);output[0]=0;
    feed("SCAN\r\n");assert(reads==17&&strstr(output,"\"scan\":\"end\""));output[0]=0;
    fail_identity=1;feed("SCAN\n");assert(reads==22);
    assert(strstr(output,"\"identity_ok\":false")&&strstr(output,"\"mode_raw\":-1"));
    assert(strstr(output,"\"identity_failure\":\"RX_TIMEOUT\""));output[0]=0;
    fail_identity=0;fail_reference=1;feed("SCAN\n");assert(reads==39);output[0]=0;
    skip_service=1;feed("SCAN\n");assert(reads==44);
    assert(strstr(output,"SERVICE_NOT_STARTED"));
    skip_service=0;fail_reference=0;output[0]=0;
    unsigned before=reads;
    feed("LIFT\n");
    assert(reads==before+4&&strstr(output,"\"id\":10"));
    assert(strstr(output,"\"read_sequence\":")&&!strstr(output,"\"id\":7"));
    before=reads;output[0]=0;feed("WHEELS\n");
    assert(reads==before+3&&strstr(output,"\"sequence\":1"));
    assert(strstr(output,"\"id\":7")&&strstr(output,"\"id\":9")&&!strstr(output,"\"id\":10"));
    assert(strstr(output,"\"position_raw\":0")&&strstr(output,"\"speed_raw\":0"));
    skip_service=1;output[0]=0;feed("WHEELS\n");
    assert(strstr(output,"\"sequence\":2")&&strstr(output,"\"ok\":false"));
    assert(strstr(output,"\"speed_raw\":-1")&&strstr(output,"\"motor_write_commands\":0"));
    skip_service=0;output[0]=0;before=reads;
    feed("DEVICE 1 3\n");assert(reads==before+5&&strstr(output,"inventory_hex"));
    assert(strstr(output,"010001031fff0000")); /* complete right ID 3 */
    fail_right=1;output[0]=0;before=reads;feed("DEVICE 1 3\n");
    assert(reads==before&&strstr(output,"0103010300000302")); /* missing identity, RX timeout */
    output[0]=0;feed("DEVICE 1 7\nDEVICE 2 1\nDEVICE 0 0\nDEVICE 0 8 x\n");
    assert(reads==before&&!strstr(output,"inventory_hex"));
    return 0;
}
