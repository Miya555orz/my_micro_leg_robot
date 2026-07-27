#include "mini_foc_can.h"

#include "drv_can.h"
#include "mini_robot_config.h"
#include "mini_status_led.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

static MiniFocMotor_t motors[MINI_ROBOT_WHEEL_COUNT];
static uint8_t tx_sequence;

static uint8_t checksum8(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0U;
    for (uint8_t i = 0U; i < len; ++i) {
        sum = (uint8_t)(sum + data[i]);
    }
    return (uint8_t)(~sum);
}

void MiniFoc_Init(void)
{
    memset(motors, 0, sizeof(motors));
    motors[0].hfdcan = &hfdcan1;
    motors[0].node_id = MINI_FOC_CAN_ID_LEFT;
    motors[1].hfdcan = &hfdcan2;
    motors[1].node_id = MINI_FOC_CAN_ID_RIGHT;
}

MiniFocMotor_t *MiniFoc_GetMotor(uint8_t index)
{
    if (index >= MINI_ROBOT_WHEEL_COUNT) {
        return 0;
    }
    return &motors[index];
}

void MiniFoc_SetCommand(uint8_t index, MiniFocMode_t mode, float value)
{
    if (index >= MINI_ROBOT_WHEEL_COUNT) {
        return;
    }
    motors[index].mode = mode;
    motors[index].command = value;
}

HAL_StatusTypeDef MiniFoc_SendIndex(uint8_t index)
{
    uint8_t frame[8];

    if (index >= MINI_ROBOT_WHEEL_COUNT) {
        return HAL_ERROR;
    }

    memset(frame, 0, sizeof(frame));
    frame[0] = (uint8_t)motors[index].mode;
    frame[1] = motors[index].node_id;
    memcpy(&frame[2], &motors[index].command, sizeof(float));
    frame[6] = tx_sequence++;
    frame[7] = checksum8(frame, 7U);

    return CAN_SendData(motors[index].hfdcan,
                        MINI_FOC_CMD_BASE_ID + motors[index].node_id,
                        frame);
}

HAL_StatusTypeDef MiniFoc_CommandIndex(uint8_t index, MiniFocMode_t mode, float value)
{
    MiniFoc_SetCommand(index, mode, value);
    return MiniFoc_SendIndex(index);
}

HAL_StatusTypeDef MiniFoc_CommandNode(uint8_t node, MiniFocMode_t mode, float value)
{
    if (node == 0U) {
        HAL_StatusTypeDef left_status = MiniFoc_CommandIndex(0U, mode, value);
        HAL_StatusTypeDef right_status = MiniFoc_CommandIndex(1U, mode, value);
        return (left_status == HAL_OK && right_status == HAL_OK) ? HAL_OK : HAL_ERROR;
    }
    if (node >= 1U && node <= MINI_ROBOT_WHEEL_COUNT) {
        return MiniFoc_CommandIndex((uint8_t)(node - 1U), mode, value);
    }
    return HAL_ERROR;
}

void MiniFoc_SendAll(void)
{
    for (uint8_t i = 0U; i < MINI_ROBOT_WHEEL_COUNT; ++i) {
        (void)MiniFoc_SendIndex(i);
    }
}

void MiniFoc_OnCanRx(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const uint8_t data[8])
{
    for (uint8_t i = 0U; i < MINI_ROBOT_WHEEL_COUNT; ++i) {
        if (motors[i].hfdcan == hfdcan && std_id == (MINI_FOC_FB_BASE_ID + motors[i].node_id)) {
            memcpy(&motors[i].speed_rps, &data[0], sizeof(float));
            memcpy(&motors[i].position_rad, &data[4], sizeof(float));
            motors[i].online = 1U;
            motors[i].last_rx_ms = HAL_GetTick();
            MiniStatusLed_Pulse((i == 0U) ? MINI_STATUS_LED_CAN1 : MINI_STATUS_LED_CAN2);
            return;
        }
    }
}

void MiniFoc_Heartbeat(uint32_t now_ms)
{
    for (uint8_t i = 0U; i < MINI_ROBOT_WHEEL_COUNT; ++i) {
        if ((now_ms - motors[i].last_rx_ms) > MINI_ROBOT_COMMAND_TIMEOUT_MS) {
            motors[i].online = 0U;
        }
    }
}
