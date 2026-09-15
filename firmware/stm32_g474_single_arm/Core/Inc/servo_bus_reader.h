#ifndef SERVO_BUS_READER_H
#define SERVO_BUS_READER_H
#include "servo_transport.h"
#include <stddef.h>
/* Maintenance only: initialize the existing left circular RX after independent
 * whole-stop proof. Runtime functions below never arm/reset RX or block. */
bool ServoBus_PrepareReader(UART_HandleTypeDef *uart, bool whole_stop_confirmed);
bool ServoBus_RecoverReader(UART_HandleTypeDef *uart, uint32_t token);
bool ServoBus_ReaderCursor(UART_HandleTypeDef *uart, uint32_t token, uint32_t *cursor);
bool ServoBus_ReaderSlice(UART_HandleTypeDef *uart, uint32_t token,
    uint32_t *cursor, uint8_t *bytes, size_t capacity, size_t *length);
bool ServoBus_ReaderQuiet(UART_HandleTypeDef *uart, uint32_t token, uint32_t cursor);
#endif
