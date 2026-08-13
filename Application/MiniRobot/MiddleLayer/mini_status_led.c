/**
 ******************************************************************************
 * @file    mini_status_led.c
 * @brief   GPIO status LED driver for heartbeat and communication activity indication.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#include "mini_status_led.h"

#include "main.h"
#include "mini_robot_config.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} MiniLedGpio_t;

static const MiniLedGpio_t activity_leds[MINI_STATUS_LED_COUNT] = {
    {LED_USART10_GPIO_Port, LED_USART10_Pin},
    {LED_CAN2_GPIO_Port, LED_CAN2_Pin},
    {LED_CAN1_GPIO_Port, LED_CAN1_Pin},
    {LED_USART1_GPIO_Port, LED_USART1_Pin},
};

static volatile uint32_t off_deadline_ms[MINI_STATUS_LED_COUNT];
static volatile uint32_t last_pulse_ms[MINI_STATUS_LED_COUNT];
static uint32_t heartbeat_last_ms;
static GPIO_PinState heartbeat_state;

void MiniStatusLed_Init(void)
{
    uint8_t i;

    for (i = 0U; i < MINI_STATUS_LED_COUNT; ++i)
    {
        HAL_GPIO_WritePin(activity_leds[i].port, activity_leds[i].pin, GPIO_PIN_RESET);
        off_deadline_ms[i] = 0U;
        last_pulse_ms[i] = 0U;
    }
    heartbeat_last_ms = HAL_GetTick();
    heartbeat_state = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, heartbeat_state);
}

void MiniStatusLed_Pulse(MiniStatusLedId_t id)
{
    const uint32_t now = HAL_GetTick();

    if (id >= MINI_STATUS_LED_COUNT)
    {
        return;
    }
    if (last_pulse_ms[id] != 0U &&
        (now - last_pulse_ms[id]) < MINI_LED_ACTIVITY_INTERVAL_MS)
    {
        return;
    }

    last_pulse_ms[id] = now;
    off_deadline_ms[id] = now + MINI_LED_ACTIVITY_PULSE_MS;
    HAL_GPIO_WritePin(activity_leds[id].port, activity_leds[id].pin, GPIO_PIN_SET);
}

void MiniStatusLed_Update(uint32_t now_ms)
{
    uint8_t i;

    for (i = 0U; i < MINI_STATUS_LED_COUNT; ++i)
    {
        if (off_deadline_ms[i] != 0U &&
            (int32_t)(now_ms - off_deadline_ms[i]) >= 0)
        {
            off_deadline_ms[i] = 0U;
            HAL_GPIO_WritePin(activity_leds[i].port, activity_leds[i].pin, GPIO_PIN_RESET);
        }
    }

    if ((now_ms - heartbeat_last_ms) >= MINI_LED_HEARTBEAT_HALF_PERIOD_MS)
    {
        heartbeat_last_ms = now_ms;
        heartbeat_state = (heartbeat_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, heartbeat_state);
    }
}

