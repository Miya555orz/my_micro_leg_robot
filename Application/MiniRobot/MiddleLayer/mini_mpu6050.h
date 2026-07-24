#ifndef MINI_MPU6050_H
#define MINI_MPU6050_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef struct {
    float accel_mps2[3];
    float gyro_rps[3];
    float temperature_c;
    float pitch_rad;
    float pitch_rate_rps;
    uint8_t online;
    uint32_t last_update_ms;
} MiniMpu6050Data_t;

HAL_StatusTypeDef MiniMpu6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MiniMpu6050_Update(float dt_s);
const MiniMpu6050Data_t *MiniMpu6050_GetData(void);

#endif
