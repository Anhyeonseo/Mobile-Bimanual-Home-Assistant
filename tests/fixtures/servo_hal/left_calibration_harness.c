#include "stm32g4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
void HAL_Delay(uint32_t t){(void)t;}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t t){(void)u;(void)p;(void)n;(void)t;return HAL_TIMEOUT;}
#include "../../../../firmware/stm32_g474_single_arm/Core/Src/left_calibration_app.c"
static uint8_t registers[7][66];
static unsigned writes,center_writes;static int fail_write,fail_read,unexpected_change;
static char output[8192];
void ServoBus_Init(UART_HandleTypeDef *u,uint8_t(*stop)(void),void(*failure)(uint8_t)){(void)u;(void)stop;(void)failure;}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t){(void)u;(void)t;assert(strlen(output)+n<sizeof(output));size_t at=strlen(output);memcpy(output+at,p,n);output[at+n]=0;return HAL_OK;}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t a,uint8_t n,uint8_t *b){assert(id>=1&&id<=6&&n<=16&&a+n<=66);if(fail_read)return HAL_ERROR;memcpy(b,registers[id]+a,n);return HAL_OK;}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t a,const uint8_t *b,uint8_t n){assert(id>=1&&id<=6&&a==40&&n==1&&(b[0]==0||b[0]==128));writes++;
 if(b[0]==128){center_writes++;registers[id][56]=0;registers[id][57]=8;registers[id][31]=99;if(unexpected_change)registers[id][21]++;}
 else registers[id][40]=0;
 return fail_write&&b[0]==128?HAL_ERROR:HAL_OK;
}
int main(int argc,char **argv){assert(argc==2);int scenario=atoi(argv[1]);UART_HandleTypeDef u={0};
 for(unsigned id=1;id<=6;id++){registers[id][5]=id;registers[id][56]=0xe8;registers[id][57]=3;}
 SingleArmApp_Init(&u,&u);assert(writes==0);output[0]=0;
 if(scenario==0){for(unsigned id=1;id<=6;id++){center(id,1000,0);assert(strstr(output,"\"center_ok\":true"));output[0]=0;}assert(center_writes==6);center(1,2048,99);assert(strstr(output,"already_attempted"));assert(center_writes==6);}
 else if(scenario==1){registers[6][40]=1;center(1,1000,0);assert(strstr(output,"torque_not_off"));assert(writes==0);}
 else if(scenario==2){center(1,2000,0);assert(strstr(output,"pose_changed"));assert(writes==0);}
 else if(scenario==3){center(1,1000,1);assert(strstr(output,"configuration_changed"));assert(writes==0);}
 else if(scenario==4){fail_write=1;center(1,1000,0);assert(strstr(output,"write_reply_uncertain"));assert(center_writes==1);center(1,2048,99);assert(center_writes==1);}
 else if(scenario==5){fail_read=1;center(1,1000,0);assert(writes==0);}
 else if(scenario==6){unexpected_change=1;center(1,1000,0);assert(strstr(output,"unexpected_configuration_change"));}
 else if(scenario==7){center(0,1000,0);center(7,1000,0);center(1,4096,0);assert(writes==0);}
 else if(scenario==8){scan();assert(strstr(output,"scan_end"));assert(writes==0);}
 else return 2;
 return 0;
}
