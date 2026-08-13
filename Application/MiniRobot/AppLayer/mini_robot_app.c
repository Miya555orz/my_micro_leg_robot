/**
 ******************************************************************************
 * @file    mini_robot_app.c
 * @brief   Main application glue for UART7 commands, STS servos, CAN FOC and MPU6050.
 * @author  Miya Zheng
 * @date    2026-08-02
 ******************************************************************************
 */
#include "mini_robot_app.h"

#include "cmsis_os.h"
#include "drv_can.h"
#include "drv_uart.h"
#include "fdcan.h"
#include "i2c.h"
#include "mini_chassis.h"
#include "mini_foc_can.h"
#include "mini_mpu6050.h"
#include "mini_robot_config.h"
#include "mini_status_led.h"
#include "mini_ttl_servo.h"
#include "mini_vofa.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

static volatile uint8_t uart_command_ready;
static uint8_t uart_command_buf[MINI_VOFA_RX_BUF_LEN];
static uint16_t uart_command_len;
static uint8_t telemetry_enabled;
static uint16_t servo_left_pos = MINI_SERVO_CENTER_POS;
static uint16_t servo_right_pos = MINI_SERVO_CENTER_POS;
static uint32_t can_last_send_ms;

static uint16_t clamp_servo_pos(float value)
{
    if (value < (float)MINI_SERVO_POSITION_MIN)
    {
        return MINI_SERVO_POSITION_MIN;
    }

    if (value > (float)MINI_SERVO_POSITION_MAX)
    {
        return MINI_SERVO_POSITION_MAX;
    }

    return (uint16_t)(value + 0.5f);
}

static void send_result(HAL_StatusTypeDef status)
{
    if (status == HAL_OK)
    {
        MiniVofa_SendText("ok\r\n");
    }
    else
    {
        MiniVofa_SendText("err\r\n");
    }
}

static void send_servo_status(uint8_t side)
{
    MiniServoStatus_t status;
    char text[96];

    if (MiniServo_ReadStatusSide(side, &status) != HAL_OK)
    {
        MiniVofa_SendText("servo timeout\r\n");
        return;
    }

    snprintf(text,
             sizeof(text),
             "servo id=%u pos=%u spd=%u volt=%.1f temp=%u\r\n",
             status.id,
             status.position,
             status.speed,
             (float)status.voltage_0v1 * 0.1f,
             status.temperature_c);
    MiniVofa_SendText(text);
}

static HAL_StatusTypeDef apply_servo_position(const MiniVofaCommand_t *cmd)
{
    uint16_t pos = clamp_servo_pos(cmd->a);

    if (cmd->index == MINI_SERVO_SIDE_LEFT)
    {
        servo_left_pos = pos;
        return MiniServo_WritePositionSide(MINI_SERVO_SIDE_LEFT, pos, MINI_SERVO_MOVE_TIME_MS, MINI_SERVO_MOVE_SPEED);
    }

    if (cmd->index == MINI_SERVO_SIDE_RIGHT)
    {
        servo_right_pos = pos;
        return MiniServo_WritePositionSide(MINI_SERVO_SIDE_RIGHT, pos, MINI_SERVO_MOVE_TIME_MS, MINI_SERVO_MOVE_SPEED);
    }

    if (cmd->index == MINI_SERVO_SIDE_ALL)
    {
        servo_left_pos = pos;
        servo_right_pos = pos;
        return MiniServo_WritePair(pos, pos, MINI_SERVO_MOVE_SPEED);
    }

    return HAL_ERROR;
}

static HAL_StatusTypeDef apply_servo_pair(const MiniVofaCommand_t *cmd)
{
    servo_left_pos = clamp_servo_pos(cmd->a);
    servo_right_pos = clamp_servo_pos(cmd->b);
    return MiniServo_WritePair(servo_left_pos, servo_right_pos, MINI_SERVO_MOVE_SPEED);
}

static void apply_vofa_command(const MiniVofaCommand_t *cmd)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (cmd == NULL)
    {
        MiniVofa_SendText("err\r\n");
        return;
    }

    switch (cmd->type)
    {
        case MINI_VOFA_CMD_VEL:
        {
            MiniChassis_SetVelocity(cmd->a, cmd->b);
            break;
        }

        case MINI_VOFA_CMD_ENABLE:
        {
            MiniChassis_SetEnabled(1U);
            break;
        }

        case MINI_VOFA_CMD_STOP:
        case MINI_VOFA_CMD_SLEEP:
        {
            MiniChassis_Sleep();
            (void)MiniFoc_CommandNode(0U, MINI_FOC_MODE_STOP, 0.0f);
            (void)MiniServo_TorqueEnableSide(MINI_SERVO_SIDE_ALL, 0U);
            break;
        }

        case MINI_VOFA_CMD_TELEMETRY:
        {
            telemetry_enabled = cmd->enable ? 1U : 0U;
            break;
        }

        case MINI_VOFA_CMD_SERVO_PING:
        {
            status = MiniServo_PingSide(cmd->index);
            break;
        }

        case MINI_VOFA_CMD_SERVO_TORQUE:
        {
            status = MiniServo_TorqueEnableSide(cmd->index, cmd->enable);
            break;
        }

        case MINI_VOFA_CMD_SERVO_READ:
        {
            send_servo_status(cmd->index);
            return;
        }

        case MINI_VOFA_CMD_SERVO_POS:
        {
            status = apply_servo_position(cmd);
            break;
        }

        case MINI_VOFA_CMD_SERVO_PAIR:
        {
            status = apply_servo_pair(cmd);
            break;
        }

        case MINI_VOFA_CMD_FOC_DIRECT:
        {
            status = MiniFoc_CommandNode(cmd->index, (MiniFocMode_t)cmd->mode, cmd->a);
            break;
        }

        case MINI_VOFA_CMD_CAN_RESTART:
        {
            if (cmd->index == 1U)
            {
                FDCAN1_Restart();
            }
            else if (cmd->index == 2U)
            {
                FDCAN2_Restart();
            }
            else
            {
                status = HAL_ERROR;
            }
            break;
        }

        default:
        {
            status = HAL_ERROR;
            break;
        }
    }

    send_result(status);
}

void MiniRobot_Init(void)
{
    MiniStatusLed_Init();
    MiniFoc_Init();
    MiniChassis_Init();
    MiniServo_Init(&huart10, &huart1);
    (void)MiniMpu6050_Init(&hi2c2);

    CAN1_Filter_Init();
    CAN2_Filter_Init();
    (void)DRV_UART7_StartRx();

    MiniChassis_Sleep();
    telemetry_enabled = 0U;
    can_last_send_ms = HAL_GetTick();

    MiniVofa_SendText("\r\nMiniWheelLegRobot ready\r\n");
    MiniVofa_SendText("UART7 115200, STS servos 1Mbps, CAN1/CAN2 FOC\r\n");
    MiniVofa_SendText("cmd: servo ping l | servo torque all 1 | servo both 2048 | foc speed 1 3\r\n");
}

void MiniRobot_ControlStep(void)
{
    uint32_t now = HAL_GetTick();

    MiniChassis_Update((float)MINI_ROBOT_CONTROL_PERIOD_MS * 0.001f);

    if ((now - can_last_send_ms) >= MINI_ROBOT_CAN_PERIOD_MS)
    {
        can_last_send_ms = now;
        MiniFoc_SendAll();
    }

    MiniFoc_Heartbeat(now);
    MiniServo_Poll(now);
    MiniStatusLed_Update(now);
}

void MiniRobot_CommandStep(void)
{
    uint8_t rx_copy[MINI_VOFA_RX_BUF_LEN];
    uint16_t rx_len;
    MiniVofaCommand_t command;

    if (uart_command_ready == 0U)
    {
        return;
    }

    __disable_irq();
    rx_len = uart_command_len;
    if (rx_len > MINI_VOFA_RX_BUF_LEN)
    {
        rx_len = MINI_VOFA_RX_BUF_LEN;
    }
    memcpy(rx_copy, uart_command_buf, rx_len);
    uart_command_ready = 0U;
    __enable_irq();

    if (MiniVofa_ParseCommand(rx_copy, rx_len, &command) != 0U)
    {
        apply_vofa_command(&command);
    }
    else
    {
        MiniVofa_SendText("unknown cmd\r\n");
    }
}

void MiniRobot_TelemetryStep(void)
{
    const MiniChassisCommand_t *command = MiniChassis_GetCommand();
    const MiniChassisState_t *state = MiniChassis_GetState();
    const MiniFocMotor_t *left_motor = MiniFoc_GetMotor(0U);
    const MiniFocMotor_t *right_motor = MiniFoc_GetMotor(1U);
    MiniVofaTelemetry_t telemetry;

    (void)MiniMpu6050_Update((float)MINI_ROBOT_VOFA_PERIOD_MS * 0.001f);

    if (telemetry_enabled == 0U || command == NULL || state == NULL || left_motor == NULL || right_motor == NULL)
    {
        return;
    }

    telemetry.wheel_speed_l = left_motor->speed_rps;
    telemetry.wheel_speed_r = right_motor->speed_rps;
    telemetry.wheel_target_l = state->wheel_target_rps[0];
    telemetry.wheel_target_r = state->wheel_target_rps[1];
    telemetry.chassis_vx = command->vx_mps;
    telemetry.chassis_wz = command->wz_rps;
    telemetry.servo_pos_l = (float)servo_left_pos;
    telemetry.servo_pos_r = (float)servo_right_pos;
    MiniVofa_SendTelemetry(&telemetry);
}

void CAN1_rxDataHandler(uint32_t rx_id, uint8_t *rx_data)
{
    MiniFoc_OnCanRx(&hfdcan1, rx_id, rx_data);
}

void CAN2_rxDataHandler(uint32_t rx_id, uint8_t *rx_data)
{
    MiniFoc_OnCanRx(&hfdcan2, rx_id, rx_data);
}

void UART7_rxDataHandler(const uint8_t *rx_data, uint16_t length)
{
    if (rx_data == NULL || length == 0U)
    {
        return;
    }

    if (length > (MINI_VOFA_RX_BUF_LEN - 1U))
    {
        length = MINI_VOFA_RX_BUF_LEN - 1U;
    }

    memcpy(uart_command_buf, rx_data, length);
    uart_command_buf[length] = 0U;
    uart_command_len = length;
    uart_command_ready = 1U;
}

void StartCtrlTask(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();

    (void)argument;
    for (;;)
    {
        MiniRobot_ControlStep();
        wake_tick += MINI_ROBOT_CONTROL_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}

void StartCommandTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        MiniRobot_CommandStep();
        osDelay(20U);
    }
}

void StartMonitorTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        MiniRobot_TelemetryStep();
        osDelay(MINI_ROBOT_VOFA_PERIOD_MS);
    }
}
