#include "mobile_boot_identity.h"
#include "actuator_core/device_startup.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#define BOOT_SLOT0 UINT32_C(0x0807E000)
#define BOOT_SLOT_SIZE UINT32_C(4096)
static bool layout(void){return FLASH_SIZE==UINT32_C(0x80000)&&(FLASH->OPTR&FLASH_OPTR_DBANK)!=0;}
static bool read_slot(void *unused,unsigned slot,uint8_t bytes[16]){
    (void)unused;if(slot>1||!layout())return false;
    memcpy(bytes,(const void *)(uintptr_t)(BOOT_SLOT0+slot*BOOT_SLOT_SIZE),16);return true;
}
static bool write_slot(void *unused,unsigned slot,const uint8_t bytes[16]){
    (void)unused;if(slot>1||!layout())return false;
    FLASH_EraseInitTypeDef erase={0};uint32_t error;uint32_t address=BOOT_SLOT0+slot*BOOT_SLOT_SIZE;
    erase.TypeErase=FLASH_TYPEERASE_PAGES;erase.Banks=FLASH_BANK_2;
    erase.Page=(address-FLASH_BASE-FLASH_BANK_SIZE)/FLASH_PAGE_SIZE;erase.NbPages=BOOT_SLOT_SIZE/FLASH_PAGE_SIZE;
    if(HAL_FLASH_Unlock()!=HAL_OK)return false;
    bool ok=HAL_FLASHEx_Erase(&erase,&error)==HAL_OK;
    for(unsigned i=0;ok&&i<2;i++){uint64_t word;memcpy(&word,bytes+8*i,8);
        ok=HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,address+8*i,word)==HAL_OK;}
    if(HAL_FLASH_Lock()!=HAL_OK)ok=false;
    return ok;
}
bool MobileBootIdentity_Advance(bool initialize,uint32_t *id){
    return actuator_boot_advance(read_slot,write_slot,NULL,initialize,id);
}
