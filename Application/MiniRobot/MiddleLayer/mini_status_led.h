#ifndef MINI_STATUS_LED_H
#define MINI_STATUS_LED_H

#include <stdint.h>

typedef enum {
    MINI_STATUS_LED_USART10 = 0,
    MINI_STATUS_LED_CAN2,
    MINI_STATUS_LED_CAN1,
    MINI_STATUS_LED_USART1,
    MINI_STATUS_LED_COUNT
} MiniStatusLedId_t;

void MiniStatusLed_Init(void);
void MiniStatusLed_Pulse(MiniStatusLedId_t id);
void MiniStatusLed_Update(uint32_t now_ms);

#endif
