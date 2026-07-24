#ifndef MINI_FOC_CAN_H
#define MINI_FOC_CAN_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef enum {
    MINI_FOC_MODE_STOP = 0,
    MINI_FOC_MODE_CURRENT = 1,
    MINI_FOC_MODE_SPEED = 2,
} MiniFocMode_t;

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t node_id;
    uint8_t online;
    uint32_t last_rx_ms;
    float position_rad;
    float speed_rps;
    float current_a;
    float command;
    MiniFocMode_t mode;
} MiniFocMotor_t;

void MiniFoc_Init(void);
MiniFocMotor_t *MiniFoc_GetMotor(uint8_t index);
void MiniFoc_SetCommand(uint8_t index, MiniFocMode_t mode, float value);
void MiniFoc_SendAll(void);
void MiniFoc_OnCanRx(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const uint8_t data[8]);
void MiniFoc_Heartbeat(uint32_t now_ms);

#endif
