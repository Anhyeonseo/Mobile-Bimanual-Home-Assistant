#ifndef SERVO_DMA_CURSOR_H
#define SERVO_DMA_CURSOR_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* IRQ masking freezes the software wrap counter, NOT the DMA engine. Sample
 * TC on both sides of CNDTR so a reload cannot combine an old remaining count
 * with a new wrap. TC cannot clear while IRQs are masked, so at most one retry
 * is needed. The critical section must remain shorter than one ring period.
 * Neither pending flags nor data are discarded here. Actual RX overruns still
 * belong to ServoRxWindow_Consume, with its unchanged capacity checks. */
static inline uint32_t ServoDmaCursor_Read(
    DMA_HandleTypeDef *dma, const volatile uint32_t *completed_wraps,
    uint32_t capacity)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t wraps = *completed_wraps;
    uint32_t before, after, remaining;
    do {
        before = __HAL_DMA_GET_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma));
        remaining = __HAL_DMA_GET_COUNTER(dma);
        after = __HAL_DMA_GET_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma));
    } while (before != after);
    if (after != 0U && remaining != 0U) wraps++;
    uint32_t absolute = wraps * capacity + capacity - remaining;
    __DMB();
    if (mask == 0U) __enable_irq();
    return absolute;
}
#endif
