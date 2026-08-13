/**
 ******************************************************************************
 * @file    mini_foc_can.h
 * @brief   Public FOC CAN interface and hub motor feedback/command structure definitions.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#ifndef MINI_FOC_CAN_H
#define MINI_FOC_CAN_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef enum
{
    MINI_FOC_MODE_STOP = 0,
    MINI_FOC_MODE_CURRENT = 1,
    MINI_FOC_MODE_SPEED = 2,
    MINI_FOC_MODE_POSITION = 3,
} MiniFocMode_t;

typedef struct
{
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

/**
 * @brief External API: MiniFoc_Init.
 * @param None.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniFoc_Init(void);
MiniFocMotor_t *MiniFoc_GetMotor(uint8_t index);
/**
 * @brief External API: MiniFoc_SetCommand.
 * @param index Input/output value owned by the caller; units follow module config and structure comments.
 * @param mode Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniFoc_SetCommand(uint8_t index, MiniFocMode_t mode, float value);
/**
 * @brief External API: MiniFoc_SendIndex.
 * @param index Input/output value owned by the caller; units follow module config and structure comments.
 * @return HAL_OK when the operation is accepted, otherwise HAL error/status code.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
HAL_StatusTypeDef MiniFoc_SendIndex(uint8_t index);
/**
 * @brief External API: MiniFoc_CommandIndex.
 * @param index Input/output value owned by the caller; units follow module config and structure comments.
 * @param mode Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return HAL_OK when the operation is accepted, otherwise HAL error/status code.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
HAL_StatusTypeDef MiniFoc_CommandIndex(uint8_t index, MiniFocMode_t mode, float value);
/**
 * @brief External API: MiniFoc_CommandNode.
 * @param node Input/output value owned by the caller; units follow module config and structure comments.
 * @param mode Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return HAL_OK when the operation is accepted, otherwise HAL error/status code.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
HAL_StatusTypeDef MiniFoc_CommandNode(uint8_t node, MiniFocMode_t mode, float value);
/**
 * @brief External API: MiniFoc_SendAll.
 * @param None.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniFoc_SendAll(void);
/**
 * @brief External API: MiniFoc_OnCanRx.
 * @param hfdcan Input/output value owned by the caller; units follow module config and structure comments.
 * @param std_id Input/output value owned by the caller; units follow module config and structure comments.
 * @param data Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniFoc_OnCanRx(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const uint8_t data[8]);
/**
 * @brief External API: MiniFoc_Heartbeat.
 * @param now_ms Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniFoc_Heartbeat(uint32_t now_ms);

#endif

