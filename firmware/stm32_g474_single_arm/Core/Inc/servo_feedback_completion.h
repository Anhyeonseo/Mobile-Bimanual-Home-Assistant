#ifndef SERVO_FEEDBACK_COMPLETION_H
#define SERVO_FEEDBACK_COMPLETION_H

#include "servo_transport.h"
#include <string.h>

/* A response and the UART lease have different lifetimes. Never end the
 * reader to retire one response. No delays, RX reset or HAL abort here.
 * The caller reserves 4450 us before the next 5 ms output tick. */
#define SERVO_FEEDBACK_RETIRE_LIMIT_US 4200U
#define SERVO_FEEDBACK_QUIET_US 100U
typedef struct {
    uint32_t started_us, activity_us, producer;
    HAL_StatusTypeDef result;
    bool completing, faulted, reported;
} ServoFeedbackCompletion;

static inline void ServoFeedbackCompletion_Start(
    ServoFeedbackCompletion *s, uint32_t now, uint32_t producer)
{
    memset(s, 0, sizeof(*s));
    s->started_us = s->activity_us = now;
    s->producer = producer;
}

static inline HAL_StatusTypeDef ServoFeedbackCompletion_Poll(
    ServoFeedbackCompletion *s, UART_HandleTypeDef *uart, uint32_t *token,
    HAL_StatusTypeDef parsed, uint32_t now, uint32_t producer,
    bool healthy, bool tx_completed)
{
    if (s->faulted) return HAL_ERROR;
    if (!healthy || parsed == HAL_ERROR ||
        (uint32_t)(now - s->started_us) >= SERVO_FEEDBACK_RETIRE_LIMIT_US) {
        s->faulted = true;
        return HAL_ERROR; /* Coordinated STOP owns abort/recovery. */
    }
    if (producer != s->producer ||
        __HAL_UART_GET_FLAG(uart, UART_FLAG_BUSY) != RESET ||
        !tx_completed) {
        s->producer = producer;
        s->activity_us = now;
    }
    if (!s->completing && parsed != HAL_BUSY) {
        s->completing = true;
        s->result = parsed;
        /* Even a timeout must retire late/partial bytes before another owner. */
        s->activity_us = now;
    }
    if (!s->completing || !tx_completed ||
        (uint32_t)(now - s->activity_us) < SERVO_FEEDBACK_QUIET_US)
        return HAL_BUSY;
    HAL_StatusTypeDef ended = ServoTransport_End(uart, *token, false);
    if (ended == HAL_OK) {
        *token = 0U;
        return s->result;
    }
    if (ended == HAL_ERROR) s->faulted = true;
    return ended;
}
/* Sticky first failure survives successful reads and explicit reader End.
 * Entries: count, tick, slot, HAL status, parser reject/status/bus reason, UART errors,
 * DMA errors, phase (1 TX, 2 reply, 3 lease retirement). */
static inline void ServoFeedbackCompletion_Record(
    ServoFeedbackCompletion *s, uint32_t out[8], uint32_t now_ms,
    uint8_t slot, HAL_StatusTypeDef result, uint32_t parser,
    uint32_t uart_error, uint32_t dma_error, uint32_t phase)
{
    if (s->reported || (result != HAL_ERROR && result != HAL_TIMEOUT)) return;
    s->reported = true;
    if (out[0] != 0U) { if (out[0] != UINT32_MAX) out[0]++; return; }
    out[0] = 1U;
    out[1]=now_ms; out[2]=slot; out[3]=(uint32_t)result; out[4]=parser;
    out[5]=uart_error; out[6]=dma_error; out[7]=phase;
}
#endif
