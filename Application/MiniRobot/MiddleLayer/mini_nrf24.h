#ifndef MINI_NRF24_H
#define MINI_NRF24_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef MiniNrf24_Init(SPI_HandleTypeDef *hspi);
uint8_t MiniNrf24_Poll(uint8_t payload[32]);
uint8_t MiniNrf24_IsOnline(void);

#endif
