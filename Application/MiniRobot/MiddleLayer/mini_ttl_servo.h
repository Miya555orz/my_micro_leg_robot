/**
 ******************************************************************************
 * @file    mini_ttl_servo.h
 * @brief   Feetech STS serial servo driver.
 * @author  Miya Zheng
 * @date    2026-08-02
 ******************************************************************************
 */
#ifndef MINI_TTL_SERVO_H
#define MINI_TTL_SERVO_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define MINI_SERVO_SIDE_LEFT                0U
#define MINI_SERVO_SIDE_RIGHT               1U
#define MINI_SERVO_SIDE_ALL                 0xFEU

typedef struct
{
    uint8_t id;
    uint8_t error;
    uint16_t position;
    uint16_t speed;
    uint16_t load;
    uint8_t voltage_0v1;
    uint8_t temperature_c;
    uint8_t moving;
    uint16_t current;
} MiniServoStatus_t;

typedef enum
{
    MINI_SERVO_FAULT_NONE = 0,
    MINI_SERVO_FAULT_RX_TIMEOUT = 1U << 0,
    MINI_SERVO_FAULT_INVALID_PACKET = 1U << 1,
    MINI_SERVO_FAULT_POSITION_RANGE = 1U << 2,
    MINI_SERVO_FAULT_COMM_CHECK = 1U << 3,
} MiniServoFault_t;

typedef struct
{
    uint16_t last_position[2];
    uint16_t target_position[2];
    uint8_t position_valid[2];
    uint8_t safe_pose_active;
    uint32_t fault_flags;
} MiniServoSafetyState_t;

void MiniServo_Init(UART_HandleTypeDef *left_uart, UART_HandleTypeDef *right_uart);
HAL_StatusTypeDef MiniServo_SetBaud(uint32_t baud);
HAL_StatusTypeDef MiniServo_PingSide(uint8_t side);
HAL_StatusTypeDef MiniServo_TorqueEnableSide(uint8_t side, uint8_t enable);
HAL_StatusTypeDef MiniServo_TorqueEnableSideNoAck(uint8_t side, uint8_t enable);
HAL_StatusTypeDef MiniServo_SetPositionModeSideNoAck(uint8_t side);
HAL_StatusTypeDef MiniServo_ReadStatusSide(uint8_t side, MiniServoStatus_t *status);
HAL_StatusTypeDef MiniServo_WritePositionSide(uint8_t side, uint16_t position, uint16_t time_ms, uint16_t speed);
HAL_StatusTypeDef MiniServo_WritePositionSideNoAck(uint8_t side, uint16_t position, uint16_t time_ms, uint16_t speed);
HAL_StatusTypeDef MiniServo_Ping(uint8_t id);
HAL_StatusTypeDef MiniServo_TorqueEnable(uint8_t id, uint8_t enable);
HAL_StatusTypeDef MiniServo_ReadStatus(uint8_t id, MiniServoStatus_t *status);
HAL_StatusTypeDef MiniServo_WritePosition(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed);
HAL_StatusTypeDef MiniServo_WritePair(uint16_t left_pos, uint16_t right_pos, uint16_t speed);
HAL_StatusTypeDef MiniServo_WritePairNoAck(uint16_t left_pos, uint16_t right_pos, uint16_t speed);
HAL_StatusTypeDef MiniServo_ShutdownPose(void);
uint8_t MiniServo_GetLastTx(uint8_t *out, uint8_t max_len);
uint8_t MiniServo_GetLastRx(uint8_t *out, uint8_t max_len);
void MiniServo_Poll(uint32_t now_ms);
uint8_t MiniServo_IsOnline(void);
void MiniServo_SafetyInit(void);
HAL_StatusTypeDef MiniServo_CheckCommunication(void);
HAL_StatusTypeDef MiniServo_EnterSafePose(void);
HAL_StatusTypeDef MiniServo_SetPositionSafeSide(uint8_t side, uint16_t target, uint16_t speed);
HAL_StatusTypeDef MiniServo_SetPairSafe(uint16_t left_pos, uint16_t right_pos, uint16_t speed);
void MiniServo_SafetyPoll(uint32_t now_ms);
uint8_t MiniServo_HasFault(void);
uint32_t MiniServo_GetFaultFlags(void);
const MiniServoSafetyState_t *MiniServo_GetSafetyState(void);
void MiniServo_ClearFaults(void);

#endif
