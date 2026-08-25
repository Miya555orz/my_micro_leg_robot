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
static uint16_t servo_left_pos = MINI_SERVO_LEFT_SAFE_POSITION;
static uint16_t servo_right_pos = MINI_SERVO_RIGHT_SAFE_POSITION;
static uint32_t can_last_send_ms;
static HAL_StatusTypeDef imu_init_status = HAL_ERROR;

static uint32_t age_ms_or_invalid(uint32_t now_ms, uint32_t tick_ms)
{
    if (tick_ms == 0U)
    {
        return 0xFFFFFFFFUL;
    }
    if (tick_ms > now_ms)
    {
        return 0U;
    }
    return now_ms - tick_ms;
}

static float clamp_absf(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static uint16_t round_servo_pos(float value)
{
    if (value < 0.0f)
    {
        return 0U;
    }
    if (value > 4095.0f)
    {
        return 4095U;
    }
    return (uint16_t)(value + 0.5f);
}

static uint8_t servo_ready_for_motion(void)
{
    return (MiniServo_CheckCommunication() == HAL_OK) ? 1U : 0U;
}

static void refresh_servo_cached_positions(void)
{
    const MiniServoSafetyState_t *safety = MiniServo_GetSafetyState();

    servo_left_pos = safety->last_position[MINI_SERVO_SIDE_LEFT];
    servo_right_pos = safety->last_position[MINI_SERVO_SIDE_RIGHT];
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

static void send_help(void)
{
    MiniVofa_SendText("help\r\n");
    MiniVofa_SendText("servo status | servo left pos | servo right pos | servo safe\r\n");
    MiniVofa_SendText("servo left set <pos> | servo right set <pos> | servo pair <l> <r>\r\n");
    MiniVofa_SendText("imu status | imu raw | imu angle\r\n");
    MiniVofa_SendText("motor status | motor left <speed> | motor right <speed>\r\n");
    MiniVofa_SendText("control status | control enable | control disable | stop\r\n");
}

static void send_servo_status(uint8_t side)
{
    MiniServoStatus_t status;
    const MiniServoSafetyState_t *safety = MiniServo_GetSafetyState();
    char text[128];

    if (side == MINI_SERVO_SIDE_ALL)
    {
        send_servo_status(MINI_SERVO_SIDE_LEFT);
        send_servo_status(MINI_SERVO_SIDE_RIGHT);
        snprintf(text,
                 sizeof(text),
                 "servo safety fault=0x%08lX safe=%u target_l=%u target_r=%u\r\n",
                 (unsigned long)MiniServo_GetFaultFlags(),
                 safety->safe_pose_active,
                 safety->target_position[MINI_SERVO_SIDE_LEFT],
                 safety->target_position[MINI_SERVO_SIDE_RIGHT]);
        MiniVofa_SendText(text);
        return;
    }

    if (MiniServo_ReadStatusSide(side, &status) != HAL_OK)
    {
        MiniVofa_SendText("servo timeout\r\n");
        return;
    }

    snprintf(text,
             sizeof(text),
             "servo %s id=%u pos=%u spd=%u volt=%.1f temp=%u\r\n",
             (side == MINI_SERVO_SIDE_LEFT) ? "left" : "right",
             status.id,
             status.position,
             status.speed,
             (float)status.voltage_0v1 * 0.1f,
             status.temperature_c);
    MiniVofa_SendText(text);
}

static HAL_StatusTypeDef apply_servo_position(const MiniVofaCommand_t *cmd)
{
    uint16_t pos = round_servo_pos(cmd->a);

    if (cmd->index == MINI_SERVO_SIDE_LEFT)
    {
        HAL_StatusTypeDef ret = MiniServo_SetPositionSafeSide(MINI_SERVO_SIDE_LEFT, pos, MINI_SERVO_MOVE_SPEED);
        refresh_servo_cached_positions();
        return ret;
    }

    if (cmd->index == MINI_SERVO_SIDE_RIGHT)
    {
        HAL_StatusTypeDef ret = MiniServo_SetPositionSafeSide(MINI_SERVO_SIDE_RIGHT, pos, MINI_SERVO_MOVE_SPEED);
        refresh_servo_cached_positions();
        return ret;
    }

    if (cmd->index == MINI_SERVO_SIDE_ALL)
    {
        HAL_StatusTypeDef ret = MiniServo_SetPairSafe(pos, pos, MINI_SERVO_MOVE_SPEED);
        refresh_servo_cached_positions();
        return ret;
    }

    return HAL_ERROR;
}

static HAL_StatusTypeDef apply_servo_pair(const MiniVofaCommand_t *cmd)
{
    servo_left_pos = round_servo_pos(cmd->a);
    servo_right_pos = round_servo_pos(cmd->b);
    HAL_StatusTypeDef ret = MiniServo_SetPairSafe(servo_left_pos, servo_right_pos, MINI_SERVO_MOVE_SPEED);
    refresh_servo_cached_positions();
    return ret;
}

static void send_imu_status(uint8_t raw, uint8_t angle)
{
    HAL_StatusTypeDef update_status;
    const MiniMpu6050Data_t *imu;
    char text[160];

    update_status = MiniMpu6050_Update((float)MINI_ROBOT_VOFA_PERIOD_MS * 0.001f);
    imu = MiniMpu6050_GetData();
    if (update_status != HAL_OK)
    {
        snprintf(text,
                 sizeof(text),
                 "imu timeout init=%u online=%u who=0x%02X last=%lu fail=%lu\r\n",
                 (imu_init_status == HAL_OK) ? 1U : 0U,
                 imu->online,
                 imu->who_am_i,
                 (unsigned long)imu->last_update_ms,
                 (unsigned long)imu->update_fail_count);
        MiniVofa_SendText(text);
        return;
    }

    if (raw)
    {
        snprintf(text,
                 sizeof(text),
                 "imu raw init=%u online=%u who=0x%02X last=%lu acc=%.3f %.3f %.3f gyro=%.3f %.3f %.3f temp=%.1f\r\n",
                 imu->init_ok,
                 imu->online,
                 imu->who_am_i,
                 (unsigned long)imu->last_update_ms,
                 imu->accel_mps2[0],
                 imu->accel_mps2[1],
                 imu->accel_mps2[2],
                 imu->gyro_rps[0],
                 imu->gyro_rps[1],
                 imu->gyro_rps[2],
                 imu->temperature_c);
    }
    else if (angle)
    {
        snprintf(text,
                 sizeof(text),
                 "imu angle init=%u online=%u who=0x%02X last=%lu roll=%.4f pitch=%.4f yaw=%.4f pitch_rate=%.4f\r\n",
                 imu->init_ok,
                 imu->online,
                 imu->who_am_i,
                 (unsigned long)imu->last_update_ms,
                 imu->roll_rad,
                 imu->pitch_rad,
                 imu->yaw_rad,
                 imu->pitch_rate_rps);
    }
    else
    {
        snprintf(text,
                 sizeof(text),
                 "imu init=%u online=%u who=0x%02X last=%lu fail=%lu acc=%.3f %.3f %.3f gyro=%.3f %.3f %.3f pitch=%.4f rate=%.4f\r\n",
                 (imu_init_status == HAL_OK) ? 1U : 0U,
                 imu->online,
                 imu->who_am_i,
                 (unsigned long)imu->last_update_ms,
                 (unsigned long)imu->update_fail_count,
                 imu->accel_mps2[0],
                 imu->accel_mps2[1],
                 imu->accel_mps2[2],
                 imu->gyro_rps[0],
                 imu->gyro_rps[1],
                 imu->gyro_rps[2],
                 imu->pitch_rad,
                 imu->pitch_rate_rps);
    }
    MiniVofa_SendText(text);
}

static void send_motor_status(void)
{
    const MiniFocMotor_t *left_motor = MiniFoc_GetMotor(0U);
    const MiniFocMotor_t *right_motor = MiniFoc_GetMotor(1U);
    const CAN_DiagTypeDef *can1 = CAN_GetDiag(&hfdcan1);
    const CAN_DiagTypeDef *can2 = CAN_GetDiag(&hfdcan2);
    uint32_t now = HAL_GetTick();
    char text[220];

    if (left_motor == NULL || right_motor == NULL)
    {
        MiniVofa_SendText("motor err\r\n");
        return;
    }

    MiniFoc_Heartbeat(now);

    snprintf(text,
             sizeof(text),
             "motor left online=%u age=%lu pos=%.3f speed=%.3f cmd=%.3f mode=%u\r\n",
             left_motor->online,
             (unsigned long)age_ms_or_invalid(now, left_motor->last_rx_ms),
             left_motor->position_rad,
             left_motor->speed_rps,
             left_motor->command,
             left_motor->mode);
    MiniVofa_SendText(text);
    snprintf(text,
             sizeof(text),
             "motor right online=%u age=%lu pos=%.3f speed=%.3f cmd=%.3f mode=%u\r\n",
             right_motor->online,
             (unsigned long)age_ms_or_invalid(now, right_motor->last_rx_ms),
             right_motor->position_rad,
             right_motor->speed_rps,
             right_motor->command,
             right_motor->mode);
    MiniVofa_SendText(text);
    if (can1 != NULL)
    {
        snprintf(text,
                 sizeof(text),
                 "can1 rx_count=%lu last_id=0x%03lX last_age=%lu data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                 (unsigned long)can1->rx_count,
                 (unsigned long)can1->last_rx_id,
                 (unsigned long)age_ms_or_invalid(now, can1->last_rx_tick_ms),
                 can1->last_rx_data[0],
                 can1->last_rx_data[1],
                 can1->last_rx_data[2],
                 can1->last_rx_data[3],
                 can1->last_rx_data[4],
                 can1->last_rx_data[5],
                 can1->last_rx_data[6],
                 can1->last_rx_data[7]);
        MiniVofa_SendText(text);
    }
    if (can2 != NULL)
    {
        snprintf(text,
                 sizeof(text),
                 "can2 rx_count=%lu last_id=0x%03lX last_age=%lu data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                 (unsigned long)can2->rx_count,
                 (unsigned long)can2->last_rx_id,
                 (unsigned long)age_ms_or_invalid(now, can2->last_rx_tick_ms),
                 can2->last_rx_data[0],
                 can2->last_rx_data[1],
                 can2->last_rx_data[2],
                 can2->last_rx_data[3],
                 can2->last_rx_data[4],
                 can2->last_rx_data[5],
                 can2->last_rx_data[6],
                 can2->last_rx_data[7]);
        MiniVofa_SendText(text);
    }
}

static void send_control_status(void)
{
    const MiniChassisCommand_t *command = MiniChassis_GetCommand();
    const MiniChassisState_t *state = MiniChassis_GetState();
    char text[160];

    snprintf(text,
             sizeof(text),
             "control enabled=%u safe=%u fault=%u lqr=%u vx=%.3f wz=%.3f servo_fault=0x%08lX\r\n",
             command->enabled,
             state->safe,
             state->fault,
             state->lqr_enabled,
             command->vx_mps,
             command->wz_rps,
             (unsigned long)MiniServo_GetFaultFlags());
    MiniVofa_SendText(text);
}

static HAL_StatusTypeDef apply_foc_direct_safe(const MiniVofaCommand_t *cmd)
{
    float value = cmd->a;

    if (cmd->mode == MINI_FOC_MODE_CURRENT)
    {
        value = clamp_absf(value, MINI_ROBOT_MAX_CURRENT_A);
    }
    else if (cmd->mode == MINI_FOC_MODE_SPEED)
    {
        value = clamp_absf(value, MINI_ROBOT_MAX_WHEEL_SPEED_RPS);
    }

    return MiniFoc_CommandNode(cmd->index, (MiniFocMode_t)cmd->mode, value);
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
        case MINI_VOFA_CMD_HELP:
        {
            send_help();
            return;
        }

        case MINI_VOFA_CMD_VEL:
        {
            if (!servo_ready_for_motion())
            {
                MiniChassis_Sleep();
                status = HAL_ERROR;
                break;
            }
            MiniChassis_SetVelocity(cmd->a, cmd->b);
            break;
        }

        case MINI_VOFA_CMD_ENABLE:
        {
            if (!servo_ready_for_motion())
            {
                MiniChassis_Sleep();
                status = HAL_ERROR;
                break;
            }
            MiniChassis_SetEnabled(1U);
            break;
        }

        case MINI_VOFA_CMD_STOP:
        case MINI_VOFA_CMD_SLEEP:
        {
            MiniChassis_Sleep();
            (void)MiniFoc_CommandNode(0U, MINI_FOC_MODE_STOP, 0.0f);
            status = MiniServo_EnterSafePose();
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
            if (cmd->enable)
            {
                status = MiniServo_CheckCommunication();
                if (status == HAL_OK)
                {
                    status = MiniServo_EnterSafePose();
                }
            }
            else
            {
                status = MiniServo_TorqueEnableSide(cmd->index, 0U);
            }
            break;
        }

        case MINI_VOFA_CMD_SERVO_READ:
        {
            send_servo_status(cmd->index);
            return;
        }

        case MINI_VOFA_CMD_SERVO_SAFE:
        {
            status = MiniServo_EnterSafePose();
            refresh_servo_cached_positions();
            break;
        }

        case MINI_VOFA_CMD_SERVO_MODE_POS:
        {
            status = MiniServo_SetPositionModeSideNoAck(cmd->index);
            break;
        }

        case MINI_VOFA_CMD_SERVO_BAUD:
        {
            status = MiniServo_SetBaud((uint32_t)cmd->a);
            break;
        }

        case MINI_VOFA_CMD_SERVO_SHUTDOWN:
        {
            status = MiniServo_EnterSafePose();
            refresh_servo_cached_positions();
            break;
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
            status = apply_foc_direct_safe(cmd);
            break;
        }

        case MINI_VOFA_CMD_IMU_STATUS:
        {
            send_imu_status(0U, 0U);
            return;
        }

        case MINI_VOFA_CMD_IMU_RAW:
        {
            send_imu_status(1U, 0U);
            return;
        }

        case MINI_VOFA_CMD_IMU_ANGLE:
        {
            send_imu_status(0U, 1U);
            return;
        }

        case MINI_VOFA_CMD_MOTOR_STATUS:
        {
            send_motor_status();
            return;
        }

        case MINI_VOFA_CMD_CONTROL_STATUS:
        {
            send_control_status();
            return;
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
    HAL_StatusTypeDef servo_status;

    MiniStatusLed_Init();
    MiniFoc_Init();
    MiniChassis_Init();
    MiniServo_Init(&huart10, &huart1);
    imu_init_status = MiniMpu6050_Init(&hi2c2);
    servo_status = MiniServo_CheckCommunication();
    if (servo_status == HAL_OK)
    {
        (void)MiniServo_EnterSafePose();
        refresh_servo_cached_positions();
    }
    else
    {
        (void)MiniServo_TorqueEnableSideNoAck(MINI_SERVO_SIDE_ALL, 0U);
    }

    CAN1_Filter_Init();
    CAN2_Filter_Init();
    (void)DRV_UART7_StartRx();

    MiniChassis_Sleep();
    telemetry_enabled = 0U;
    can_last_send_ms = HAL_GetTick();

    MiniVofa_SendText("\r\nMiniWheelLegRobot ready\r\n");
    MiniVofa_SendText("UART7 115200, STS servos 1Mbps, CAN1/CAN2 FOC\r\n");
    MiniVofa_SendText((imu_init_status == HAL_OK) ? "imu init ok\r\n" : "imu init err\r\n");
    if (servo_status == HAL_OK)
    {
        MiniVofa_SendText("servo safe pose active\r\n");
    }
    else
    {
        MiniVofa_SendText("servo comm fault, motion locked\r\n");
    }
    MiniVofa_SendText("cmd: help | servo status | servo safe | imu angle | motor status\r\n");
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

    if (MiniMpu6050_Update((float)MINI_ROBOT_VOFA_PERIOD_MS * 0.001f) == HAL_OK)
    {
        const MiniMpu6050Data_t *imu = MiniMpu6050_GetData();
        MiniChassis_SetMeasuredAttitude(imu->pitch_rad, imu->pitch_rate_rps);
    }

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
