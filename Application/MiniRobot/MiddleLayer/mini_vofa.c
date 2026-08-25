/**
 ******************************************************************************
 * @file    mini_vofa.c
 * @brief   VOFA/serial command parser and telemetry formatter for PC-side debugging.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#include "mini_vofa.h"

#include "mini_robot_config.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart7;

static volatile uint8_t vofa_tx_busy;

static uint8_t vofa_try_lock_tx(void)
{
    uint8_t locked = 0U;

    __disable_irq();
    if (vofa_tx_busy == 0U)
    {
        vofa_tx_busy = 1U;
        locked = 1U;
    }
    __enable_irq();
    return locked;
}

static void vofa_unlock_tx(void)
{
    __disable_irq();
    vofa_tx_busy = 0U;
    __enable_irq();
}

static HAL_StatusTypeDef vofa_send_bytes(const uint8_t *data, uint16_t length)
{
    uint32_t start;
    uint32_t elapsed;
    HAL_StatusTypeDef status;

    if (data == 0 || length == 0U)
    {
        return HAL_ERROR;
    }

    start = HAL_GetTick();
    while (vofa_try_lock_tx() == 0U)
    {
        if ((HAL_GetTick() - start) >= MINI_VOFA_TX_TIMEOUT_MS)
        {
            return HAL_BUSY;
        }
    }

    while (huart7.gState != HAL_UART_STATE_READY)
    {
        if ((HAL_GetTick() - start) >= MINI_VOFA_TX_TIMEOUT_MS)
        {
            vofa_unlock_tx();
            return HAL_BUSY;
        }
    }

    elapsed = HAL_GetTick() - start;
    status = HAL_UART_Transmit(&huart7,
                               (uint8_t *)data,
                               length,
                               MINI_VOFA_TX_TIMEOUT_MS - elapsed);
    vofa_unlock_tx();
    return status;
}

static uint16_t bounded_strlen(const uint8_t *rx, uint16_t max_len)
{
    uint16_t n = 0U;
    while (n < max_len && rx[n] != 0U)
    {
        ++n;
    }
    return n;
}

static uint8_t parse_servo_selector(const char *selector, uint8_t *index)
{
    int id;

    if (selector == 0 || index == 0)
    {
        return 0U;
    }
    if (sscanf(selector, "%d", &id) == 1)
    {
        if (id == 0 || id == 1)
        {
            *index = 0U;
            return 1U;
        }
        if (id == 2)
        {
            *index = 1U;
            return 1U;
        }
        if (id == 254)
        {
            *index = 0xFEU;
            return 1U;
        }
        return 0U;
    }
    if (strcmp(selector, "l") == 0 ||
        strcmp(selector, "left") == 0 ||
        strcmp(selector, "L") == 0 ||
        strcmp(selector, "LEFT") == 0)
    {
        *index = 0U;
        return 1U;
    }
    if (strcmp(selector, "r") == 0 ||
        strcmp(selector, "right") == 0 ||
        strcmp(selector, "R") == 0 ||
        strcmp(selector, "RIGHT") == 0)
    {
        *index = 1U;
        return 1U;
    }
    if (strcmp(selector, "all") == 0 ||
        strcmp(selector, "ALL") == 0)
    {
        *index = 0xFEU;
        return 1U;
    }
    return 0U;
}

uint8_t MiniVofa_ParseCommand(const uint8_t *rx, uint16_t max_len, MiniVofaCommand_t *out)
{
    char line[MINI_VOFA_RX_BUF_LEN + 1U];
    char selector[12];
    uint16_t len;
    int index;
    int enable;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;

    if (rx == 0 || out == 0)
    {
        return 0U;
    }

    memset(out, 0, sizeof(*out));
    len = bounded_strlen(rx, max_len);
    if (len == 0U || len > MINI_VOFA_RX_BUF_LEN)
    {
        return 0U;
    }
    memcpy(line, rx, len);
    line[len] = '\0';

    if (strncmp(line, "help", 4U) == 0 || strncmp(line, "?", 1U) == 0)
    {
        out->type = MINI_VOFA_CMD_HELP;
        return 1U;
    }
    if (sscanf(line, "vel %f %f", &a, &b) == 2 || sscanf(line, "vx=%f wz=%f", &a, &b) == 2)
    {
        out->type = MINI_VOFA_CMD_VEL;
        out->a = a;
        out->b = b;
        return 1U;
    }
    if (sscanf(line, "pos %f %f", &a, &b) == 2)
    {
        out->type = MINI_VOFA_CMD_WHEEL_POS;
        out->a = a;
        out->b = b;
        return 1U;
    }
    if (sscanf(line, "enable %d", &enable) == 1)
    {
        out->type = enable ? MINI_VOFA_CMD_ENABLE : MINI_VOFA_CMD_STOP;
        out->enable = (uint8_t)(enable != 0);
        return 1U;
    }
    if (sscanf(line, "standpos %f %f", &a, &b) == 2 ||
        sscanf(line, "stand pos %f %f", &a, &b) == 2)
    {
        out->type = MINI_VOFA_CMD_STAND_POS;
        out->a = a;
        out->b = b;
        return 1U;
    }
    if (strncmp(line, "stand", 5U) == 0)
    {
        out->type = MINI_VOFA_CMD_STAND;
        return 1U;
    }
    if (strncmp(line, "sleep", 5U) == 0)
    {
        out->type = MINI_VOFA_CMD_SLEEP;
        return 1U;
    }
    if (strncmp(line, "stop", 4U) == 0)
    {
        out->type = MINI_VOFA_CMD_STOP;
        return 1U;
    }
    if (sscanf(line, "telemetry %d", &enable) == 1 || sscanf(line, "monitor %d", &enable) == 1)
    {
        out->type = MINI_VOFA_CMD_TELEMETRY;
        out->enable = (uint8_t)(enable != 0);
        return 1U;
    }
    if (strncmp(line, "can stat", 8U) == 0)
    {
        out->type = MINI_VOFA_CMD_CAN_STAT;
        return 1U;
    }
    if (sscanf(line, "can auto %d", &enable) == 1)
    {
        out->type = MINI_VOFA_CMD_CAN_AUTO;
        out->enable = (uint8_t)(enable != 0);
        return 1U;
    }
    if (sscanf(line, "can tx %d", &index) == 1)
    {
        out->type = MINI_VOFA_CMD_CAN_TX;
        out->index = (uint8_t)index;
        return 1U;
    }
    if (sscanf(line, "foc stop %d", &index) == 1 ||
        sscanf(line, "stop %d", &index) == 1)
    {
        out->type = MINI_VOFA_CMD_FOC_DIRECT;
        out->index = (uint8_t)index;
        out->mode = 0U;
        out->a = 0.0f;
        return 1U;
    }
    if (sscanf(line, "foc enable %d", &index) == 1 ||
        sscanf(line, "enable %d", &index) == 1)
    {
        out->type = MINI_VOFA_CMD_FOC_DIRECT;
        out->index = (uint8_t)index;
        out->mode = 2U;
        out->a = 0.0f;
        return 1U;
    }
    if (sscanf(line, "foc speed %d %f", &index, &a) == 2 ||
        sscanf(line, "speed %d %f", &index, &a) == 2)
    {
        out->type = MINI_VOFA_CMD_FOC_DIRECT;
        out->index = (uint8_t)index;
        out->mode = 2U;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "foc pos %d %f", &index, &a) == 2 ||
        sscanf(line, "foc position %d %f", &index, &a) == 2 ||
        sscanf(line, "position %d %f", &index, &a) == 2)
    {
        out->type = MINI_VOFA_CMD_FOC_DIRECT;
        out->index = (uint8_t)index;
        out->mode = 3U;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "foc torque %d %f", &index, &a) == 2 ||
        sscanf(line, "torque %d %f", &index, &a) == 2)
    {
        out->type = MINI_VOFA_CMD_FOC_DIRECT;
        out->index = (uint8_t)index;
        out->mode = 1U;
        out->a = a;
        return 1U;
    }
    if (strncmp(line, "block reset", 11U) == 0)
    {
        out->type = MINI_VOFA_CMD_BLOCK_RESET;
        return 1U;
    }
    if (strncmp(line, "control status", 14U) == 0)
    {
        out->type = MINI_VOFA_CMD_CONTROL_STATUS;
        return 1U;
    }
    if (strncmp(line, "control enable", 14U) == 0)
    {
        out->type = MINI_VOFA_CMD_ENABLE;
        out->enable = 1U;
        return 1U;
    }
    if (strncmp(line, "control disable", 15U) == 0)
    {
        out->type = MINI_VOFA_CMD_STOP;
        return 1U;
    }
    if (strncmp(line, "motor status", 12U) == 0)
    {
        out->type = MINI_VOFA_CMD_MOTOR_STATUS;
        return 1U;
    }
    if (sscanf(line, "motor left %f", &a) == 1)
    {
        out->type = MINI_VOFA_CMD_FOC_DIRECT;
        out->index = 1U;
        out->mode = 2U;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "motor right %f", &a) == 1)
    {
        out->type = MINI_VOFA_CMD_FOC_DIRECT;
        out->index = 2U;
        out->mode = 2U;
        out->a = a;
        return 1U;
    }
    if (strncmp(line, "imu status", 10U) == 0)
    {
        out->type = MINI_VOFA_CMD_IMU_STATUS;
        return 1U;
    }
    if (strncmp(line, "imu raw", 7U) == 0)
    {
        out->type = MINI_VOFA_CMD_IMU_RAW;
        return 1U;
    }
    if (strncmp(line, "imu angle", 9U) == 0)
    {
        out->type = MINI_VOFA_CMD_IMU_ANGLE;
        return 1U;
    }
    if (sscanf(line, "can restart %d", &index) == 1)
    {
        out->type = MINI_VOFA_CMD_CAN_RESTART;
        out->index = (uint8_t)index;
        return 1U;
    }
    if (sscanf(line, "pid speed %d %f %f %f", &index, &a, &b, &c) == 4)
    {
        out->type = MINI_VOFA_CMD_PID_SPEED;
        out->index = (uint8_t)index;
        out->a = a;
        out->b = b;
        out->c = c;
        return 1U;
    }
    if (sscanf(line, "pid pos %d %f %f %f", &index, &a, &b, &c) == 4)
    {
        out->type = MINI_VOFA_CMD_PID_POSITION;
        out->index = (uint8_t)index;
        out->a = a;
        out->b = b;
        out->c = c;
        return 1U;
    }
    if (sscanf(line, "lqr enable %d", &enable) == 1)
    {
        out->type = MINI_VOFA_CMD_LQR_ENABLE;
        out->enable = (uint8_t)(enable != 0);
        return 1U;
    }
    if (sscanf(line, "lqr k %d %d %f", &index, &enable, &a) == 3)
    {
        out->type = MINI_VOFA_CMD_LQR_GAIN;
        out->index = (uint8_t)index;
        out->enable = (uint8_t)enable;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "lqr a %d %d %f", &index, &enable, &a) == 3)
    {
        out->type = MINI_VOFA_CMD_LQR_A;
        out->index = (uint8_t)index;
        out->enable = (uint8_t)enable;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "lqr b %d %d %f", &index, &enable, &a) == 3)
    {
        out->type = MINI_VOFA_CMD_LQR_B;
        out->index = (uint8_t)index;
        out->enable = (uint8_t)enable;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "lqr state %f %f %f %f %f %f", &a, &b, &c, &d, &e, &f) == 6)
    {
        out->type = MINI_VOFA_CMD_LQR_STATE;
        out->a = a;
        out->b = b;
        out->c = c;
        out->d = d;
        out->e = e;
        out->f = f;
        return 1U;
    }
    if (sscanf(line, "lqr target %f %f %f %f %f %f", &a, &b, &c, &d, &e, &f) == 6)
    {
        out->type = MINI_VOFA_CMD_LQR_TARGET;
        out->a = a;
        out->b = b;
        out->c = c;
        out->d = d;
        out->e = e;
        out->f = f;
        return 1U;
    }
    if (sscanf(line, "lqr ff %d %f", &index, &a) == 2)
    {
        out->type = MINI_VOFA_CMD_LQR_FEEDFORWARD;
        out->index = (uint8_t)index;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "lqr limit %f", &a) == 1)
    {
        out->type = MINI_VOFA_CMD_LQR_LIMIT;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "servo angle %f", &a) == 1 ||
        sscanf(line, "servo deg %f", &a) == 1)
    {
        out->type = MINI_VOFA_CMD_SERVO_ANGLE;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "servo mode pos %11s", selector) == 1 &&
        parse_servo_selector(selector, &out->index))
    {
        out->type = MINI_VOFA_CMD_SERVO_MODE_POS;
        return 1U;
    }
    if (strncmp(line, "servo mode pos", 14U) == 0 ||
        strncmp(line, "servo posmode", 13U) == 0)
    {
        out->type = MINI_VOFA_CMD_SERVO_MODE_POS;
        out->index = 0xFEU;
        return 1U;
    }
    if (sscanf(line, "servo both %f", &a) == 1)
    {
        out->type = MINI_VOFA_CMD_SERVO_POS;
        out->index = 0xFEU;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "servo left set %f", &a) == 1)
    {
        out->type = MINI_VOFA_CMD_SERVO_POS;
        out->index = 0U;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "servo right set %f", &a) == 1)
    {
        out->type = MINI_VOFA_CMD_SERVO_POS;
        out->index = 1U;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "servo pos %11s %f", selector, &a) == 2 &&
        parse_servo_selector(selector, &out->index))
    {
        out->type = MINI_VOFA_CMD_SERVO_POS;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "servo ping %11s", selector) == 1 &&
        parse_servo_selector(selector, &out->index))
    {
        out->type = MINI_VOFA_CMD_SERVO_PING;
        return 1U;
    }
    if (sscanf(line, "servo torque %11s %d", selector, &enable) == 2 &&
        parse_servo_selector(selector, &out->index))
    {
        out->type = MINI_VOFA_CMD_SERVO_TORQUE;
        out->enable = (uint8_t)(enable != 0);
        return 1U;
    }
    if (sscanf(line, "servo read %11s", selector) == 1 &&
        parse_servo_selector(selector, &out->index))
    {
        out->type = MINI_VOFA_CMD_SERVO_READ;
        return 1U;
    }
    if (sscanf(line, "servo %11s pos", selector) == 1 &&
        parse_servo_selector(selector, &out->index))
    {
        out->type = MINI_VOFA_CMD_SERVO_READ;
        return 1U;
    }
    if (strncmp(line, "servo status", 12U) == 0)
    {
        out->type = MINI_VOFA_CMD_SERVO_READ;
        out->index = 0xFEU;
        return 1U;
    }
    if (strncmp(line, "servo safe", 10U) == 0)
    {
        out->type = MINI_VOFA_CMD_SERVO_SAFE;
        return 1U;
    }
    if (sscanf(line, "servo baud %f", &a) == 1)
    {
        out->type = MINI_VOFA_CMD_SERVO_BAUD;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "servo pair %f %f", &a, &b) == 2)
    {
        out->type = MINI_VOFA_CMD_SERVO_PAIR;
        out->a = a;
        out->b = b;
        return 1U;
    }
    if (strncmp(line, "servo shutdown", 14U) == 0)
    {
        out->type = MINI_VOFA_CMD_SERVO_SHUTDOWN;
        return 1U;
    }
    if (sscanf(line, "wheel %d %f %f %f %f", &index, &a, &b, &c, &d) == 5)
    {
        out->type = MINI_VOFA_CMD_PID_SPEED;
        out->index = (uint8_t)index;
        out->a = b;
        out->b = c;
        out->c = d;
        return 1U;
    }

    return 0U;
}

void MiniVofa_SendText(const char *text)
{
    if (text == 0)
    {
        return;
    }
    (void)vofa_send_bytes((const uint8_t *)text, (uint16_t)strlen(text));
}

void MiniVofa_SendTelemetry(const MiniVofaTelemetry_t *telemetry)
{
    uint8_t tx[MINI_VOFA_CHANNEL_COUNT * sizeof(float) + 4U];
    float channels[MINI_VOFA_CHANNEL_COUNT];
    uint16_t offset = 0U;

    if (telemetry == 0)
    {
        return;
    }

    channels[0] = telemetry->wheel_speed_l;
    channels[1] = telemetry->wheel_speed_r;
    channels[2] = telemetry->wheel_target_l;
    channels[3] = telemetry->wheel_target_r;
    channels[4] = telemetry->chassis_vx;
    channels[5] = telemetry->chassis_wz;
    channels[6] = telemetry->servo_pos_l;
    channels[7] = telemetry->servo_pos_r;

    memcpy(tx, channels, sizeof(channels));
    offset = (uint16_t)sizeof(channels);
    tx[offset + 0U] = 0x00U;
    tx[offset + 1U] = 0x00U;
    tx[offset + 2U] = 0x80U;
    tx[offset + 3U] = 0x7FU;
    (void)vofa_send_bytes(tx, sizeof(tx));
}





