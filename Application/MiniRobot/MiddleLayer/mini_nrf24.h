/**
 ******************************************************************************
 * @file    mini_nrf24.h
 * @brief   Public nRF24L01 driver interface.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#ifndef MINI_NRF24_H
#define MINI_NRF24_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/**
 * @brief External API: MiniNrf24_Init.
 * @param hspi Input/output value owned by the caller; units follow module config and structure comments.
 * @return HAL_OK when the operation is accepted, otherwise HAL error/status code.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
HAL_StatusTypeDef MiniNrf24_Init(SPI_HandleTypeDef *hspi);
/**
 * @brief External API: MiniNrf24_Poll.
 * @param payload Input/output value owned by the caller; units follow module config and structure comments.
 * @return Count, state flag, or boolean-style result as described by the function name.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
uint8_t MiniNrf24_Poll(uint8_t payload[32]);
/**
 * @brief External API: MiniNrf24_IsOnline.
 * @param None.
 * @return Count, state flag, or boolean-style result as described by the function name.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
uint8_t MiniNrf24_IsOnline(void);

#endif

