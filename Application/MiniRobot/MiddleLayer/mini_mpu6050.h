/**
 ******************************************************************************
 * @file    mini_mpu6050.h
 * @brief   Public MPU6050 attitude sensor interface and physical-unit data structure.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#ifndef MINI_MPU6050_H
#define MINI_MPU6050_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef struct
{
    float accel_mps2[3];
    float gyro_rps[3];
    float temperature_c;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float roll_rate_rps;
    float pitch_rate_rps;
    float yaw_rate_rps;
    uint8_t online;
    uint32_t last_update_ms;
} MiniMpu6050Data_t;

/**
 * @brief External API: MiniMpu6050_Init.
 * @param hi2c Input/output value owned by the caller; units follow module config and structure comments.
 * @return HAL_OK when the operation is accepted, otherwise HAL error/status code.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
HAL_StatusTypeDef MiniMpu6050_Init(I2C_HandleTypeDef *hi2c);
/**
 * @brief External API: MiniMpu6050_Update.
 * @param dt_s Input/output value owned by the caller; units follow module config and structure comments.
 * @return HAL_OK when the operation is accepted, otherwise HAL error/status code.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
HAL_StatusTypeDef MiniMpu6050_Update(float dt_s);
const MiniMpu6050Data_t *MiniMpu6050_GetData(void);

#endif

