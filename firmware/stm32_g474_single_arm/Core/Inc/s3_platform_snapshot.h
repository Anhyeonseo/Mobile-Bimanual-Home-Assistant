#ifndef S3_PLATFORM_SNAPSHOT_H
#define S3_PLATFORM_SNAPSHOT_H
#include <stdint.h>
#include <stdbool.h>
enum {
    S3_PLATFORM_FIRST_AUX_SLOT = 6,
    S3_PLATFORM_AUX_COUNT = 4,
    S3_PLATFORM_SLOT_COUNT = S3_PLATFORM_FIRST_AUX_SLOT + S3_PLATFORM_AUX_COUNT,
    S3_PLATFORM_PRESENT_MASK = (1U << S3_PLATFORM_AUX_COUNT) - 1U,
    S3_PLATFORM_SNAPSHOT_SIZE = 44
};

/* Read-only slots on IDs 7..10. Never configure/enable a mobile motor. */
void S3Platform_Enable(bool enabled);
uint8_t S3Platform_Select(uint8_t arm_joint);
void S3Platform_Scheduled(uint8_t slot);
void S3Platform_Update(uint8_t slot, uint16_t raw, uint32_t observed_ms);
/* Failed arm or auxiliary slot gets the next available read opportunity.
 * It retains the old value and original timestamp until a valid reply. */
void S3Platform_Retry(uint8_t joint);
void S3Platform_CancelRetry(void);
void S3Platform_Invalidate(uint8_t slot);
void S3Platform_Copy(uint32_t now, uint8_t out[S3_PLATFORM_SNAPSHOT_SIZE]);
#endif
