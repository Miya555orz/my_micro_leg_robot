#ifndef MINI_VOFA_H
#define MINI_VOFA_H

#include <stdint.h>

typedef enum {
    MINI_VOFA_CMD_NONE = 0,
    MINI_VOFA_CMD_VEL,
    MINI_VOFA_CMD_ENABLE,
    MINI_VOFA_CMD_STOP,
    MINI_VOFA_CMD_WHEEL_POS,
    MINI_VOFA_CMD_PID_SPEED,
    MINI_VOFA_CMD_PID_POSITION,
    MINI_VOFA_CMD_LQR_ENABLE,
    MINI_VOFA_CMD_LQR_GAIN,
    MINI_VOFA_CMD_LQR_A,
    MINI_VOFA_CMD_LQR_B,
    MINI_VOFA_CMD_LQR_STATE,
    MINI_VOFA_CMD_LQR_TARGET,
    MINI_VOFA_CMD_LQR_FEEDFORWARD,
    MINI_VOFA_CMD_LQR_LIMIT,
    MINI_VOFA_CMD_SERVO_POS,
    MINI_VOFA_CMD_SERVO_PING,
    MINI_VOFA_CMD_SERVO_TORQUE,
    MINI_VOFA_CMD_SERVO_READ,
    MINI_VOFA_CMD_SERVO_BAUD,
    MINI_VOFA_CMD_SERVO_PAIR,
    MINI_VOFA_CMD_SERVO_SHUTDOWN,
    MINI_VOFA_CMD_TELEMETRY,
    MINI_VOFA_CMD_CAN_STAT,
    MINI_VOFA_CMD_CAN_RESTART,
    MINI_VOFA_CMD_CAN_AUTO,
    MINI_VOFA_CMD_CAN_TX,
    MINI_VOFA_CMD_FOC_DIRECT,
} MiniVofaCmdType_t;

typedef struct {
    MiniVofaCmdType_t type;
    uint8_t index;
    uint8_t enable;
    uint8_t mode;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
} MiniVofaCommand_t;

typedef struct {
    float wheel_speed_l;
    float wheel_speed_r;
    float wheel_target_l;
    float wheel_target_r;
    float chassis_vx;
    float chassis_wz;
    float servo_pos_l;
    float servo_pos_r;
} MiniVofaTelemetry_t;

uint8_t MiniVofa_ParseCommand(const uint8_t *rx, uint16_t max_len, MiniVofaCommand_t *out);
void MiniVofa_SendText(const char *text);
void MiniVofa_SendTelemetry(const MiniVofaTelemetry_t *telemetry);

#endif
