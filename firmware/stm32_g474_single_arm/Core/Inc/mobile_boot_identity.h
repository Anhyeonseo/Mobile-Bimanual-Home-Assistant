#ifndef MOBILE_BOOT_IDENTITY_H
#define MOBILE_BOOT_IDENTITY_H
#include <stdbool.h>
#include <stdint.h>
/* Cold, inhibited boot only. Reserved final 8 KiB; supports the measured target
 * STM32G474RE 512 KiB dual-bank layout, rejects other flash option layouts. */
bool MobileBootIdentity_Advance(bool initialize_erased,uint32_t *boot_id);
#endif
