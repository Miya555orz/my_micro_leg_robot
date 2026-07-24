#ifndef MINI_CHASSIS_H
#define MINI_CHASSIS_H

#include "mini_pid.h"
#include "mini_lqr.h"
#include <stdint.h>

typedef enum {
    MINI_PID_SLOT_SPEED = 0,
    MINI_PID_SLOT_POSITION = 1,
} MiniPidSlot_t;

typedef struct {
    float vx_mps;
    float wz_rps;
    uint8_t enabled;
    uint8_t servo_shutdown_request;
    uint32_t last_command_ms;
} MiniChassisCommand_t;

typedef struct {
    float wheel_target_rps[2];
    float wheel_current_cmd[2];
    float lqr_state[MINI_LQR_STATE_DIM];
    float lqr_output[MINI_LQR_INPUT_DIM];
    uint8_t lqr_enabled;
    uint8_t safe;
} MiniChassisState_t;

void MiniChassis_Init(void);
void MiniChassis_SetEnabled(uint8_t enabled);
void MiniChassis_SetVelocity(float vx_mps, float wz_rps);
void MiniChassis_SetWheelPosition(float left_rad, float right_rad);
void MiniChassis_SetPid(MiniPidSlot_t slot, uint8_t wheel, float kp, float ki, float kd);
void MiniChassis_SetLqrEnabled(uint8_t enabled);
void MiniChassis_SetLqrGain(uint8_t input, uint8_t state, float value);
void MiniChassis_SetLqrA(uint8_t row, uint8_t col, float value);
void MiniChassis_SetLqrB(uint8_t row, uint8_t input, float value);
void MiniChassis_SetLqrState(const float lqr_state[MINI_LQR_STATE_DIM]);
void MiniChassis_SetLqrTarget(const float lqr_target[MINI_LQR_STATE_DIM]);
void MiniChassis_SetLqrFeedforward(uint8_t input, float value);
void MiniChassis_SetLqrOutputLimit(float limit);
void MiniChassis_SetMeasuredAttitude(float pitch_rad, float pitch_rate_rps);
void MiniChassis_RequestServoShutdown(void);
void MiniChassis_ClearServoShutdownRequest(void);
void MiniChassis_Update(float dt_s);
const MiniChassisCommand_t *MiniChassis_GetCommand(void);
const MiniChassisState_t *MiniChassis_GetState(void);

#endif
