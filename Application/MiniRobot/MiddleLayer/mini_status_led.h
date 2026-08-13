/**
 ******************************************************************************
 * @file    mini_status_led.h
 * @brief   Public status LED interface and LED channel definitions.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#ifndef MINI_STATUS_LED_H
#define MINI_STATUS_LED_H

#include <stdint.h>

typedef enum
{
    MINI_STATUS_LED_USART10 = 0,
    MINI_STATUS_LED_CAN2,
    MINI_STATUS_LED_CAN1,
    MINI_STATUS_LED_USART1,
    MINI_STATUS_LED_COUNT
} MiniStatusLedId_t;

/**
 * @brief External API: MiniStatusLed_Init.
 * @param None.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniStatusLed_Init(void);
/**
 * @brief External API: MiniStatusLed_Pulse.
 * @param id Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniStatusLed_Pulse(MiniStatusLedId_t id);
/**
 * @brief External API: MiniStatusLed_Update.
 * @param now_ms Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniStatusLed_Update(uint32_t now_ms);

#endif

