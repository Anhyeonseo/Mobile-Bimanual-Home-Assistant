#ifndef FAKE_STM32_HAL_H
#define FAKE_STM32_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#define HAL_UART_STATE_READY 1U
#define HAL_UART_STATE_BUSY_TX 2U
typedef struct { struct {uint32_t BaudRate;} Init; uint32_t gState; unsigned sent, aborted; int fail_tx, fail_abort, immediate, wire_busy; } UART_HandleTypeDef;
#define UART_FLAG_BUSY 1U
#define RESET 0U
#define __HAL_UART_GET_FLAG(uart, flag) ((void)(flag), (uart)->wire_busy)
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *,const uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *,const uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *,const uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
static inline uint32_t __get_PRIMASK(void){return 0;}
static inline void __disable_irq(void){}
static inline void __enable_irq(void){}
static inline void __DMB(void){}
#endif
