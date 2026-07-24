#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_uart7_rx;

void MX_USART1_UART_Init(void);
void MX_UART7_Init(void);
void MX_USART10_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif
