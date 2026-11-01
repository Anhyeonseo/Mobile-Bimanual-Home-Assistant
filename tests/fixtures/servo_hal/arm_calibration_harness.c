#include "stm32g4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static const char *incoming;
UART_HandleTypeDef huart4={0};
void HAL_Delay(uint32_t t){(void)t;}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t t){
 (void)u;(void)n;(void)t;if(!incoming||!*incoming)return HAL_TIMEOUT;*p=(uint8_t)*incoming++;return HAL_OK;
}
#include "../../../../firmware/stm32_g474_single_arm/Core/Src/left_calibration_app.c"
static uint8_t registers[2][7][66];
static unsigned writes[2],centers[2],reads[2];
static int failed_read,uncertain_write,changed_eeprom;
static char output[8192];
void ServoBus_Init(UART_HandleTypeDef *u,uint8_t(*stop)(void),void(*failure)(uint8_t)){(void)u;(void)stop;(void)failure;}
void RightServoBus_Init(UART_HandleTypeDef *u){assert(u==&huart4);}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t){
 (void)u;(void)t;size_t at=strlen(output);assert(at+n<sizeof(output));memcpy(output+at,p,n);output[at+n]=0;return HAL_OK;
}
static HAL_StatusTypeDef read_bus(unsigned bus,uint8_t id,uint8_t address,uint8_t length,uint8_t *data){
 assert(id>=1&&id<=6&&length<=16&&address+length<=66);reads[bus]++;
 if(failed_read)return HAL_ERROR;memcpy(data,registers[bus][id]+address,length);return HAL_OK;
}
static HAL_StatusTypeDef write_bus(unsigned bus,uint8_t id,uint8_t value){
 assert(id>=1&&id<=6&&(value==0||value==128));writes[bus]++;
 if(value==128){centers[bus]++;registers[bus][id][56]=0;registers[bus][id][57]=8;registers[bus][id][31]=99;
  if(changed_eeprom)registers[bus][id][21]++;
  if(uncertain_write)return HAL_ERROR;
 }else registers[bus][id][40]=0;
 return HAL_OK;
}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t a,uint8_t n,uint8_t *data){return read_bus(0,id,a,n,data);}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t a,const uint8_t *data,uint8_t n){assert(a==40&&n==1);return write_bus(0,id,*data);}
HAL_StatusTypeDef RightServoBus_CalibrationRead(uint8_t id,uint8_t a,uint8_t n,uint8_t *data){return read_bus(1,id,a,n,data);}
HAL_StatusTypeDef RightServoBus_CalibrationWrite(uint8_t id,uint8_t value){return write_bus(1,id,value);}
static void command_line(const char *s){output[0]=0;incoming=s;while(*incoming)SingleArmApp_Process();}
int main(int argc,char **argv){
 assert(argc==2);int scenario=atoi(argv[1]);UART_HandleTypeDef host_uart={0},left_uart={0};
 for(unsigned bus=0;bus<2;bus++)for(unsigned id=1;id<=6;id++){
  registers[bus][id][5]=id;registers[bus][id][56]=0xe8;registers[bus][id][57]=3;
 }
 SingleArmApp_Init(&host_uart,&left_uart);assert(writes[0]+writes[1]+reads[0]+reads[1]==0);
 if(scenario==0){
  command_line("RIGHT CENTER 1 1000 0\n");assert(strstr(output,"\"center_ok\":true"));assert(writes[0]==0&&reads[0]==0&&centers[1]==1);
  command_line("LEFT CENTER 1 1000 0\n");assert(strstr(output,"\"center_ok\":true"));assert(centers[0]==1&&centers[1]==1);
  command_line("RIGHT CENTER 1 2048 99\n");assert(strstr(output,"already_attempted"));assert(centers[1]==1);
  command_line("RIGHT INFO\n");assert(strstr(output,"\"arm\":\"right\"")&&strstr(output,"\"consumed_mask\":1"));
 }else if(scenario==1){
  command_line("CENTER 1 1000 0\n");command_line("BOTH CENTER 1 1000 0\n");
  command_line("RIGHT CENTER 7 1000 0\n");command_line("LEFT CENTER 254 1000 0\n");
  command_line("RIGHT CENTER 1 1000 0 extra\n");command_line("LEFT MOVE 2048\n");
  command_line("RIGHT CENTER 1 4096 0\n");assert(writes[0]+writes[1]+reads[0]+reads[1]==0);
 }else if(scenario==2){
  registers[1][6][40]=1;command_line("RIGHT CENTER 1 1000 0\n");assert(strstr(output,"torque_not_off"));assert(writes[0]+writes[1]==0);
 }else if(scenario==3){
  command_line("RIGHT CENTER 1 1100 0\n");assert(strstr(output,"pose_changed"));assert(writes[0]+writes[1]==0);
 }else if(scenario==4){
  uncertain_write=1;command_line("RIGHT CENTER 1 1000 0\n");assert(strstr(output,"write_reply_uncertain"));
  assert(writes[0]==0&&centers[1]==1);command_line("RIGHT CENTER 1 2048 99\n");assert(strstr(output,"already_attempted"));assert(centers[1]==1);
 }else if(scenario==5){
  changed_eeprom=1;command_line("LEFT CENTER 1 1000 0\n");assert(strstr(output,"unexpected_configuration_change"));assert(writes[1]==0);
 }else if(scenario==6){
  failed_read=1;command_line("RIGHT CENTER 1 1000 0\n");assert(writes[0]+writes[1]==0);
 }else if(scenario==7){
  command_line("RIGHT SCAN\n");assert(strstr(output,"scan_end"));assert(reads[0]==0&&reads[1]>0&&writes[0]+writes[1]==0);
 }else if(scenario==8){
  command_line("RIGHT OFF\n");assert(strstr(output,"off_ok\":true"));assert(writes[0]==0&&writes[1]==6&&centers[1]==0);
 }else return 2;
 return 0;
}
