#include "stm32g4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static const char *incoming;
static unsigned tick,reads,failed;
static char output[8192];
UART_HandleTypeDef huart4={0};
uint32_t HAL_GetTick(void){return tick++;}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t t){
 (void)u;(void)n;(void)t;if(!incoming||!*incoming)return HAL_TIMEOUT;*p=(uint8_t)*incoming++;return HAL_OK;
}
#include "../../../../firmware/stm32_g474_single_arm/Core/Src/s2_observer_app.c"
void ServoBus_Init(UART_HandleTypeDef *u,uint8_t(*stop)(void),void(*failure)(uint8_t)){(void)u;(void)stop;(void)failure;}
void RightServoBus_Init(UART_HandleTypeDef *u){assert(u==&huart4);}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t){
 (void)u;(void)t;size_t at=strlen(output);assert(at+n<sizeof(output));memcpy(output+at,p,n);output[at+n]=0;return HAL_OK;
}
static HAL_StatusTypeDef read_bus(unsigned bus,uint8_t id,uint8_t address,uint8_t length,uint8_t *data){
 assert((id>=1&&id<=6)||(!bus&&id==10));
 assert((address==56&&length==4)||(address==40&&length==1)||
        (!bus&&id==6&&address==56&&length==15)||
        (id<=6&&((address==3&&length==4)||(address==31&&length==3))));reads++;
 if(failed)return HAL_ERROR;
 memset(data,0,length);if(address==56)data[1]=8;
 if(address==3){data[0]=9;data[1]=3;data[2]=id;}
 if(address==31){data[0]=83;data[1]=2;}
 if(length==15){data[4]=18;data[5]=4;data[9]=8;data[13]=52;data[14]=18;}
 return HAL_OK;
}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t a,uint8_t n,uint8_t *data){return read_bus(0,id,a,n,data);}
HAL_StatusTypeDef RightServoBus_CalibrationRead(uint8_t id,uint8_t a,uint8_t n,uint8_t *data){return read_bus(1,id,a,n,data);}
static void command_line(const char *s){output[0]=0;incoming=s;while(*incoming)SingleArmApp_Process();}
int main(void){
 UART_HandleTypeDef h={0},l={0};SingleArmApp_Init(&h,&l);assert(reads==0);
 command_line("SAMPLE\n");assert(reads==0&&strstr(output,"bind_required"));
 command_line("BIND -1\n");assert(epoch==0);command_line("BIND 0\n");assert(epoch==0);
 command_line("BIND 123 junk\n");assert(epoch==0);command_line("BIND 123\n");assert(epoch==123);
 command_line("MOVE 2048\n");assert(reads==0);command_line("SAMPLE\n");
 assert(reads==26&&strstr(output,"sample_end")&&strstr(output,"\"id\":10")&&sequence==1);
 command_line("SAMPLE\n");assert(sequence==2&&reads==52);
 failed=1;command_line("SAMPLE\n");assert(epoch==0&&reads==53&&strstr(output,"motor_read_failed"));
 command_line("SAMPLE\n");assert(reads==53);command_line("BIND 456\n");assert(epoch==456);
 SingleArmApp_Init(&h,&l);assert(epoch==0&&reads==53);
 command_line("GRIP\n");assert(reads==53&&strstr(output,"bind_required"));
 failed=0;command_line("BIND 111\n");command_line("GRIP\n");
 assert(reads==55&&strstr(output,"\"id\":6")&&strstr(output,"\"load_raw\":1042"));
 assert(strstr(output,"\"current_raw\":4660")&&strstr(output,"\"motor_status_raw\":8")&&strstr(output,"sample_end"));
 command_line("GRIP 1\n");assert(reads==55&&strstr(output,"unsupported_command"));
 command_line("CALIB\n");assert(reads==79&&sequence==2&&strstr(output,"sample_end"));
 assert(strstr(output,"\"offset_raw\":595")&&strstr(output,"\"model_high\":9")&&!strstr(output,"\"id\":10"));
 failed=1;command_line("CALIB\n");assert(reads==80&&epoch==0&&strstr(output,"calibration_read_failed"));
 command_line("CALIB\n");assert(reads==80&&strstr(output,"bind_required"));
 return 0;
}
