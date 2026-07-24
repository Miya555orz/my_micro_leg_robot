#include "mini_vofa.h"

#include "mini_robot_config.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart7;

static uint16_t bounded_strlen(const uint8_t *rx, uint16_t max_len)
{
    uint16_t n = 0U;
    while (n < max_len && rx[n] != 0U) {
        ++n;
    }
    return n;
}

uint8_t MiniVofa_ParseCommand(const uint8_t *rx, uint16_t max_len, MiniVofaCommand_t *out)
{
    char line[MINI_VOFA_RX_BUF_LEN + 1U];
    uint16_t len;
    int index;
    int enable;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;

    if (rx == 0 || out == 0) {
        return 0U;
    }

    memset(out, 0, sizeof(*out));
    len = bounded_strlen(rx, max_len);
    if (len == 0U || len > MINI_VOFA_RX_BUF_LEN) {
        return 0U;
    }
    memcpy(line, rx, len);
    line[len] = '\0';

    if (sscanf(line, "vel %f %f", &a, &b) == 2 || sscanf(line, "vx=%f wz=%f", &a, &b) == 2) {
        out->type = MINI_VOFA_CMD_VEL;
        out->a = a;
        out->b = b;
        return 1U;
    }
    if (sscanf(line, "pos %f %f", &a, &b) == 2) {
        out->type = MINI_VOFA_CMD_WHEEL_POS;
        out->a = a;
        out->b = b;
        return 1U;
    }
    if (sscanf(line, "enable %d", &enable) == 1) {
        out->type = enable ? MINI_VOFA_CMD_ENABLE : MINI_VOFA_CMD_STOP;
        out->enable = (uint8_t)(enable != 0);
        return 1U;
    }
    if (strncmp(line, "stop", 4U) == 0) {
        out->type = MINI_VOFA_CMD_STOP;
        return 1U;
    }
    if (sscanf(line, "pid speed %d %f %f %f", &index, &a, &b, &c) == 4) {
        out->type = MINI_VOFA_CMD_PID_SPEED;
        out->index = (uint8_t)index;
        out->a = a;
        out->b = b;
        out->c = c;
        return 1U;
    }
    if (sscanf(line, "pid pos %d %f %f %f", &index, &a, &b, &c) == 4) {
        out->type = MINI_VOFA_CMD_PID_POSITION;
        out->index = (uint8_t)index;
        out->a = a;
        out->b = b;
        out->c = c;
        return 1U;
    }
    if (sscanf(line, "lqr enable %d", &enable) == 1) {
        out->type = MINI_VOFA_CMD_LQR_ENABLE;
        out->enable = (uint8_t)(enable != 0);
        return 1U;
    }
    if (sscanf(line, "lqr k %d %d %f", &index, &enable, &a) == 3) {
        out->type = MINI_VOFA_CMD_LQR_GAIN;
        out->index = (uint8_t)index;
        out->enable = (uint8_t)enable;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "lqr a %d %d %f", &index, &enable, &a) == 3) {
        out->type = MINI_VOFA_CMD_LQR_A;
        out->index = (uint8_t)index;
        out->enable = (uint8_t)enable;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "lqr b %d %d %f", &index, &enable, &a) == 3) {
        out->type = MINI_VOFA_CMD_LQR_B;
        out->index = (uint8_t)index;
        out->enable = (uint8_t)enable;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "lqr state %f %f %f %f %f %f", &a, &b, &c, &d, &e, &f) == 6) {
        out->type = MINI_VOFA_CMD_LQR_STATE;
        out->a = a;
        out->b = b;
        out->c = c;
        out->d = d;
        out->e = e;
        out->f = f;
        return 1U;
    }
    if (sscanf(line, "lqr target %f %f %f %f %f %f", &a, &b, &c, &d, &e, &f) == 6) {
        out->type = MINI_VOFA_CMD_LQR_TARGET;
        out->a = a;
        out->b = b;
        out->c = c;
        out->d = d;
        out->e = e;
        out->f = f;
        return 1U;
    }
    if (sscanf(line, "lqr ff %d %f", &index, &a) == 2) {
        out->type = MINI_VOFA_CMD_LQR_FEEDFORWARD;
        out->index = (uint8_t)index;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "lqr limit %f", &a) == 1) {
        out->type = MINI_VOFA_CMD_LQR_LIMIT;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "servo pos %d %f", &index, &a) == 2) {
        out->type = MINI_VOFA_CMD_SERVO_POS;
        out->index = (uint8_t)index;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "servo ping %d", &index) == 1) {
        out->type = MINI_VOFA_CMD_SERVO_PING;
        out->index = (uint8_t)index;
        return 1U;
    }
    if (sscanf(line, "servo torque %d %d", &index, &enable) == 2) {
        out->type = MINI_VOFA_CMD_SERVO_TORQUE;
        out->index = (uint8_t)index;
        out->enable = (uint8_t)(enable != 0);
        return 1U;
    }
    if (sscanf(line, "servo read %d", &index) == 1) {
        out->type = MINI_VOFA_CMD_SERVO_READ;
        out->index = (uint8_t)index;
        return 1U;
    }
    if (sscanf(line, "servo baud %f", &a) == 1) {
        out->type = MINI_VOFA_CMD_SERVO_BAUD;
        out->a = a;
        return 1U;
    }
    if (sscanf(line, "servo pair %f %f", &a, &b) == 2) {
        out->type = MINI_VOFA_CMD_SERVO_PAIR;
        out->a = a;
        out->b = b;
        return 1U;
    }
    if (strncmp(line, "servo shutdown", 14U) == 0) {
        out->type = MINI_VOFA_CMD_SERVO_SHUTDOWN;
        return 1U;
    }
    if (sscanf(line, "wheel %d %f %f %f %f", &index, &a, &b, &c, &d) == 5) {
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
    if (text == 0) {
        return;
    }
    HAL_UART_Transmit(&huart7, (uint8_t *)text, (uint16_t)strlen(text), MINI_VOFA_TX_TIMEOUT_MS);
}

void MiniVofa_SendTelemetry(const MiniVofaTelemetry_t *telemetry)
{
    uint8_t tx[MINI_VOFA_CHANNEL_COUNT * sizeof(float) + 4U];
    float channels[MINI_VOFA_CHANNEL_COUNT];
    uint16_t offset = 0U;

    if (telemetry == 0) {
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
    HAL_UART_Transmit(&huart7, tx, sizeof(tx), MINI_VOFA_TX_TIMEOUT_MS);
}
