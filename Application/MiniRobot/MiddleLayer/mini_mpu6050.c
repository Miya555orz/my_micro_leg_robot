/**
 ******************************************************************************
 * @file    mini_mpu6050.c
 * @brief   MPU6050 initialization, raw data acquisition, and roll/pitch/yaw attitude estimation.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#include "mini_mpu6050.h"

#include "mini_robot_config.h"
#include <math.h>
#include <string.h>

#define MPU_REG_SMPLRT_DIV    0x19U
#define MPU_REG_CONFIG        0x1AU
#define MPU_REG_GYRO_CONFIG   0x1BU
#define MPU_REG_ACCEL_CONFIG  0x1CU
#define MPU_REG_ACCEL_XOUT_H  0x3BU
#define MPU_REG_PWR_MGMT_1    0x6BU
#define MPU_REG_WHO_AM_I      0x75U

#define MPU_WHO_AM_I_VALUE    0x68U
#define MPU_GRAVITY_MPS2      9.80665f
#define MPU_DEG_TO_RAD        0.01745329252f
#define MPU_PI                3.14159265359f

static I2C_HandleTypeDef *mpu_i2c;
static MiniMpu6050Data_t mpu_data;

static HAL_StatusTypeDef write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(mpu_i2c,
                             MINI_MPU6050_I2C_ADDRESS,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             20U);
}

static int16_t be_i16(const uint8_t *src)
{
    return (int16_t)(((uint16_t)src[0] << 8U) | src[1]);
}

HAL_StatusTypeDef MiniMpu6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i = 0U;

    if (hi2c == 0)
    {
        return HAL_ERROR;
    }
    mpu_i2c = hi2c;
    memset(&mpu_data, 0, sizeof(mpu_data));

    if (HAL_I2C_Mem_Read(mpu_i2c,
                         MINI_MPU6050_I2C_ADDRESS,
                         MPU_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT,
                         &who_am_i,
                         1U,
                         50U) != HAL_OK ||
        who_am_i != MPU_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    if (write_reg(MPU_REG_PWR_MGMT_1, 0x01U) != HAL_OK ||
        write_reg(MPU_REG_SMPLRT_DIV, 0x09U) != HAL_OK ||
        write_reg(MPU_REG_CONFIG, 0x03U) != HAL_OK ||
        write_reg(MPU_REG_GYRO_CONFIG, 0x08U) != HAL_OK ||
        write_reg(MPU_REG_ACCEL_CONFIG, 0x08U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(10U);
    mpu_data.online = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef MiniMpu6050_Update(float dt_s)
{
    uint8_t raw[14];
    const float accel_scale = MPU_GRAVITY_MPS2 / 8192.0f;
    const float gyro_scale = MPU_DEG_TO_RAD / 65.5f;
    float accel_roll;
    float accel_pitch;

    if (mpu_i2c == 0 || dt_s <= 0.0f)
    {
        return HAL_ERROR;
    }
    if (HAL_I2C_Mem_Read(mpu_i2c,
                         MINI_MPU6050_I2C_ADDRESS,
                         MPU_REG_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT,
                         raw,
                         sizeof(raw),
                         30U) != HAL_OK)
    {
        mpu_data.online = 0U;
        return HAL_ERROR;
    }

    mpu_data.accel_mps2[0] = (float)be_i16(&raw[0]) * accel_scale;
    mpu_data.accel_mps2[1] = (float)be_i16(&raw[2]) * accel_scale;
    mpu_data.accel_mps2[2] = (float)be_i16(&raw[4]) * accel_scale;
    mpu_data.temperature_c = (float)be_i16(&raw[6]) / 340.0f + 36.53f;
    mpu_data.gyro_rps[0] = (float)be_i16(&raw[8]) * gyro_scale;
    mpu_data.gyro_rps[1] = (float)be_i16(&raw[10]) * gyro_scale;
    mpu_data.gyro_rps[2] = (float)be_i16(&raw[12]) * gyro_scale;

    accel_roll = atan2f(mpu_data.accel_mps2[1], mpu_data.accel_mps2[2]);
    accel_pitch = atan2f(-mpu_data.accel_mps2[0],
                         sqrtf(mpu_data.accel_mps2[1] * mpu_data.accel_mps2[1] +
                               mpu_data.accel_mps2[2] * mpu_data.accel_mps2[2]));
    mpu_data.roll_rate_rps = MINI_MPU6050_ROLL_SIGN * mpu_data.gyro_rps[0];
    mpu_data.pitch_rate_rps = MINI_MPU6050_PITCH_SIGN * mpu_data.gyro_rps[1];
    mpu_data.yaw_rate_rps = MINI_MPU6050_YAW_SIGN * mpu_data.gyro_rps[2];
    mpu_data.roll_rad = MINI_MPU6050_COMPLEMENTARY_ALPHA *
                        (mpu_data.roll_rad + mpu_data.roll_rate_rps * dt_s) +
                        (1.0f - MINI_MPU6050_COMPLEMENTARY_ALPHA) *
                        (MINI_MPU6050_ROLL_SIGN * accel_roll + MINI_MPU6050_ROLL_OFFSET_RAD);
    mpu_data.pitch_rad = MINI_MPU6050_COMPLEMENTARY_ALPHA *
                         (mpu_data.pitch_rad + mpu_data.pitch_rate_rps * dt_s) +
                         (1.0f - MINI_MPU6050_COMPLEMENTARY_ALPHA) *
                         (MINI_MPU6050_PITCH_SIGN * accel_pitch + MINI_MPU6050_PITCH_OFFSET_RAD);
    mpu_data.yaw_rad += mpu_data.yaw_rate_rps * dt_s;
    if (mpu_data.yaw_rad > MPU_PI)
    {
        mpu_data.yaw_rad -= 2.0f * MPU_PI;
    }
    else if (mpu_data.yaw_rad < -MPU_PI)
    {
        mpu_data.yaw_rad += 2.0f * MPU_PI;
    }
    mpu_data.last_update_ms = HAL_GetTick();
    mpu_data.online = 1U;
    return HAL_OK;
}

const MiniMpu6050Data_t *MiniMpu6050_GetData(void)
    {
    return &mpu_data;
}

