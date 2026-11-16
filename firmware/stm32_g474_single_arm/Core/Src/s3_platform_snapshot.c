#include "s3_platform_snapshot.h"
#include "actuator_core/wire_bytes.h"

#include <string.h>

static struct {
    bool enabled;
    bool due;
    bool retry_pending;
    uint8_t retry_slot;
    uint8_t next;
    uint8_t mask;
    uint16_t raw[S3_PLATFORM_AUX_COUNT];
    uint32_t stamp[S3_PLATFORM_AUX_COUNT];
    uint32_t count;
} snapshot;

static bool IsAuxiliarySlot(uint8_t slot)
{
    return slot >= S3_PLATFORM_FIRST_AUX_SLOT && slot < S3_PLATFORM_SLOT_COUNT;
}

void S3Platform_Enable(bool enabled)
{
    if (enabled && !snapshot.enabled && snapshot.mask == S3_PLATFORM_PRESENT_MASK) {
        /* A handoff may cancel a read whose scheduling already advanced next.
         * Refresh the oldest actual sample before another arm sweep. Preserve
         * all sample timestamps, including across the millisecond clock wrap. */
        uint8_t oldest = 0U;
        for (uint8_t i = 1U; i < S3_PLATFORM_AUX_COUNT; i++) {
            if ((int32_t)(snapshot.stamp[i] - snapshot.stamp[oldest]) < 0) {
                oldest = i;
            }
        }
        snapshot.next = oldest;
        snapshot.due = true;
    }
    snapshot.enabled = enabled;
}

uint8_t S3Platform_Select(uint8_t arm_joint)
{
    if (snapshot.retry_pending) {
        return snapshot.retry_slot;
    }
    if (snapshot.enabled && snapshot.due && arm_joint == 0U) {
        return S3_PLATFORM_FIRST_AUX_SLOT + snapshot.next;
    }
    return arm_joint;
}

void S3Platform_Scheduled(uint8_t slot)
{
    if (snapshot.retry_pending && slot == snapshot.retry_slot) {
        snapshot.retry_pending = false;
        return;
    }
    if (slot == S3_PLATFORM_FIRST_AUX_SLOT - 1U) {
        snapshot.due = true;
    } else if (IsAuxiliarySlot(slot)) {
        snapshot.due = false;
        snapshot.next = (snapshot.next + 1U) % S3_PLATFORM_AUX_COUNT;
    }
}

void S3Platform_Update(uint8_t slot, uint16_t raw, uint32_t observed_ms)
{
    if (!IsAuxiliarySlot(slot)) {
        return;
    }
    uint8_t index = slot - S3_PLATFORM_FIRST_AUX_SLOT;
    snapshot.raw[index] = raw;
    snapshot.stamp[index] = observed_ms;
    snapshot.mask |= 1U << index;
    snapshot.count++;
}

void S3Platform_CancelRetry(void)
{
    snapshot.retry_pending = false;
}

void S3Platform_Retry(uint8_t slot)
{
    if (slot < S3_PLATFORM_SLOT_COUNT) {
        snapshot.retry_slot = slot;
        snapshot.retry_pending = true;
    }
}

void S3Platform_Invalidate(uint8_t slot)
{
    if (IsAuxiliarySlot(slot)) {
        snapshot.mask &= ~(1U << (slot - S3_PLATFORM_FIRST_AUX_SLOT));
    }
}

void S3Platform_Copy(uint32_t now, uint8_t out[S3_PLATFORM_SNAPSHOT_SIZE])
{
    memset(out, 0, S3_PLATFORM_SNAPSHOT_SIZE);
    out[0] = 1U;
    out[1] = snapshot.enabled;
    out[2] = snapshot.mask;
    actuator_write_u32_le(out + 4, now);
    actuator_write_u32_le(out + 8, snapshot.count);
    for (unsigned i = 0U; i < S3_PLATFORM_AUX_COUNT; i++) {
        uint32_t age = (snapshot.mask & (1U << i)) ? now - snapshot.stamp[i] : UINT32_MAX;
        actuator_write_u32_le(out + 12 + 4 * i, snapshot.raw[i]);
        actuator_write_u32_le(out + 28 + 4 * i, age);
    }
}
