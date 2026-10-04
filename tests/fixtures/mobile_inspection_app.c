#include "single_arm_app.h"
#include "servo_bus.h"
#include <assert.h>
#include <string.h>
static UART_HandleTypeDef host, servo;
static const unsigned char *input;
static unsigned remaining, reads;
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
    assert(id==1||(id>=7&&id<=10));reads++;
    if(id==1)assert(address==3&&length==4);
    if(skip_service)return HAL_BUSY;
    health.transaction_count++;
    memset(&diag,0,sizeof(diag));
    assert((address==3&&length==4)||(address==33&&length==1)||
           (address==40&&length==1)||(address==56&&length==2));
    if((fail_identity|| (fail_reference&&id==1))&&address==3) {
        diag.reason=SERVO_BUS_FAILURE_RX_TIMEOUT;return HAL_TIMEOUT;
    }
    memset(data,0,length);
    if(address==3){data[0]=9;data[1]=3;data[2]=id;}
    return HAL_OK;
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
    return 0;
}
