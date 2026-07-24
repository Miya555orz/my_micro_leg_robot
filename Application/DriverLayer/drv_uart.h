#ifndef __DRV_UART_H
#define __DRV_UART_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define UART7_RX_BUF_LEN 400U

HAL_StatusTypeDef DRV_UART7_StartRx(void);
void UART7_rxDataHandler(const uint8_t *rx_buf, uint16_t length);

#endif
