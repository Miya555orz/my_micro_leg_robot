#include "mini_chassis.h"

#include "mini_foc_can.h"
#include "mini_robot_config.h"
#include "stm32h7xx_hal.h"

static MiniChassisCommand_t command;
static MiniChassisState_t state;
static MiniPid_t speed_pid[MINI_ROBOT_WHEEL_COUNT];
static MiniPid_t position_pid[MINI_ROBOT_WHEEL_COUNT];
static MiniLqr_t lqr;
static uint8_t position_mode;
static float wheel_position_target[MINI_ROBOT_WHEEL_COUNT];

static float clampf(float v, float limit)
{
    if (v > limit) {
        return limit;
    }
    if (v < -limit) {
        return -limit;
    }
    return v;
}

void MiniChassis_Init(void)
{
    MiniPidParam_t speed_param = { .kp = MINI_PID_SPEED_KP, .ki = MINI_PID_SPEED_KI, .kd = MINI_PID_SPEED_KD, .integral_limit = MINI_PID_SPEED_INTEGRAL_LIMIT, .output_limit = MINI_ROBOT_MAX_CURRENT_A };
    MiniPidParam_t pos_param = { .kp = MINI_PID_POSITION_KP, .ki = MINI_PID_POSITION_KI, .kd = MINI_PID_POSITION_KD, .integral_limit = MINI_PID_POSITION_INTEGRAL_LIMIT, .output_limit = MINI_ROBOT_MAX_WHEEL_SPEED_RPS };

    command.vx_mps = 0.0f;
    command.wz_rps = 0.0f;
    command.enabled = 0U;
    command.servo_shutdown_request = 0U;
    command.last_command_ms = HAL_GetTick();
    position_mode = 0U;
    MiniLqr_Init(&lqr);

    for (uint8_t i = 0U; i < MINI_ROBOT_WHEEL_COUNT; ++i) {
        MiniPid_Init(&speed_pid[i], &speed_param);
        MiniPid_Init(&position_pid[i], &pos_param);
        state.wheel_target_rps[i] = 0.0f;
        state.wheel_current_cmd[i] = 0.0f;
        wheel_position_target[i] = 0.0f;
    }
    for (uint8_t i = 0U; i < MINI_LQR_STATE_DIM; ++i) {
        state.lqr_state[i] = 0.0f;
    }
    for (uint8_t i = 0U; i < MINI_LQR_INPUT_DIM; ++i) {
        state.lqr_output[i] = 0.0f;
    }
    state.lqr_enabled = 0U;
    state.safe = 1U;
}

void MiniChassis_SetEnabled(uint8_t enabled)
{
    command.enabled = enabled ? 1U : 0U;
    command.last_command_ms = HAL_GetTick();
    if (!command.enabled) {
        command.vx_mps = 0.0f;
        command.wz_rps = 0.0f;
        position_mode = 0U;
    }
}

void MiniChassis_SetVelocity(float vx_mps, float wz_rps)
{
    command.vx_mps = vx_mps;
    command.wz_rps = wz_rps;
    command.enabled = 1U;
    position_mode = 0U;
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_SetWheelPosition(float left_rad, float right_rad)
{
    wheel_position_target[0] = left_rad;
    wheel_position_target[1] = right_rad;
    command.enabled = 1U;
    position_mode = 1U;
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_SetLqrEnabled(uint8_t enabled)
{
    MiniLqr_SetEnabled(&lqr, enabled);
    state.lqr_enabled = lqr.enabled;
    command.enabled = enabled ? 1U : command.enabled;
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_SetLqrGain(uint8_t input, uint8_t state_index, float value)
{
    MiniLqr_SetGain(&lqr, input, state_index, value);
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_SetLqrA(uint8_t row, uint8_t col, float value)
{
    MiniLqr_SetA(&lqr, row, col, value);
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_SetLqrB(uint8_t row, uint8_t input, float value)
{
    MiniLqr_SetB(&lqr, row, input, value);
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_SetLqrState(const float lqr_state[MINI_LQR_STATE_DIM])
{
    MiniLqr_SetState(&lqr, lqr_state);
    for (uint8_t i = 0U; i < MINI_LQR_STATE_DIM; ++i) {
        state.lqr_state[i] = lqr.x[i];
    }
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_SetLqrTarget(const float lqr_target[MINI_LQR_STATE_DIM])
{
    MiniLqr_SetTarget(&lqr, lqr_target);
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_SetLqrFeedforward(uint8_t input, float value)
{
    MiniLqr_SetFeedforward(&lqr, input, value);
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_SetLqrOutputLimit(float limit)
{
    MiniLqr_SetOutputLimit(&lqr, limit);
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_SetMeasuredAttitude(float pitch_rad, float pitch_rate_rps)
{
    lqr.x[0] = pitch_rad;
    lqr.x[1] = pitch_rate_rps;
    state.lqr_state[0] = pitch_rad;
    state.lqr_state[1] = pitch_rate_rps;
}

void MiniChassis_SetPid(MiniPidSlot_t slot, uint8_t wheel, float kp, float ki, float kd)
{
    MiniPidParam_t param;

    if (wheel >= MINI_ROBOT_WHEEL_COUNT) {
        return;
    }

    if (slot == MINI_PID_SLOT_SPEED) {
        param = speed_pid[wheel].param;
        param.output_limit = MINI_ROBOT_MAX_CURRENT_A;
        param.integral_limit = 20.0f;
        param.kp = kp;
        param.ki = ki;
        param.kd = kd;
        MiniPid_SetParam(&speed_pid[wheel], &param);
    } else {
        param = position_pid[wheel].param;
        param.output_limit = MINI_ROBOT_MAX_WHEEL_SPEED_RPS;
        param.kp = kp;
        param.ki = ki;
        param.kd = kd;
        MiniPid_SetParam(&position_pid[wheel], &param);
    }
}

void MiniChassis_RequestServoShutdown(void)
{
    command.servo_shutdown_request = 1U;
    command.last_command_ms = HAL_GetTick();
}

void MiniChassis_ClearServoShutdownRequest(void)
{
    command.servo_shutdown_request = 0U;
}

void MiniChassis_Update(float dt_s)
{
    const uint32_t now = HAL_GetTick();
    const float half_track = MINI_ROBOT_WHEEL_TRACK_M * 0.5f;
    const float left_mps = command.vx_mps - command.wz_rps * half_track;
    const float right_mps = command.vx_mps + command.wz_rps * half_track;
    MiniFocMotor_t *left_motor = MiniFoc_GetMotor(0U);
    MiniFocMotor_t *right_motor = MiniFoc_GetMotor(1U);

    lqr.x[2] = (left_motor != 0) ? left_motor->position_rad : 0.0f;
    lqr.x[3] = (left_motor != 0) ? left_motor->speed_rps : 0.0f;
    lqr.x[4] = (right_motor != 0) ? right_motor->position_rad : 0.0f;
    lqr.x[5] = (right_motor != 0) ? right_motor->speed_rps : 0.0f;
    for (uint8_t i = 0U; i < MINI_LQR_STATE_DIM; ++i) {
        state.lqr_state[i] = lqr.x[i];
    }

    state.safe = ((now - command.last_command_ms) <= MINI_ROBOT_COMMAND_TIMEOUT_MS) ? 1U : 0U;
    if (!state.safe) {
        command.enabled = 0U;
        command.vx_mps = 0.0f;
        command.wz_rps = 0.0f;
        position_mode = 0U;
        MiniLqr_SetEnabled(&lqr, 0U);
    }

    if (lqr.enabled && command.enabled && state.safe) {
        MiniLqr_Calc(&lqr);
        for (uint8_t i = 0U; i < MINI_LQR_INPUT_DIM; ++i) {
            state.lqr_output[i] = lqr.u[i];
            state.wheel_current_cmd[i] = lqr.u[i];
            MiniFoc_SetCommand(i, MINI_FOC_MODE_CURRENT, lqr.u[i]);
        }
        state.lqr_enabled = 1U;
        return;
    }

    state.lqr_enabled = 0U;
    for (uint8_t i = 0U; i < MINI_ROBOT_WHEEL_COUNT; ++i) {
        MiniFocMotor_t *motor = MiniFoc_GetMotor(i);
        float measured = (motor != 0) ? motor->speed_rps : 0.0f;
        float pos_measured = (motor != 0) ? motor->position_rad : 0.0f;
        float out;

        if (position_mode) {
            state.wheel_target_rps[i] = MiniPid_Calc(&position_pid[i], wheel_position_target[i], pos_measured, dt_s);
        } else {
            float target_mps = (i == 0U) ? left_mps : right_mps;
            state.wheel_target_rps[i] = clampf(target_mps / (6.2831853f * MINI_ROBOT_WHEEL_RADIUS_M),
                                               MINI_ROBOT_MAX_WHEEL_SPEED_RPS);
        }
        out = MiniPid_Calc(&speed_pid[i], state.wheel_target_rps[i], measured, dt_s);

        if (!command.enabled || !state.safe) {
            out = 0.0f;
            MiniPid_Reset(&speed_pid[i]);
            MiniFoc_SetCommand(i, MINI_FOC_MODE_STOP, 0.0f);
        } else {
            MiniFoc_SetCommand(i, MINI_FOC_MODE_CURRENT, out);
        }
        state.wheel_current_cmd[i] = out;
    }
}

const MiniChassisCommand_t *MiniChassis_GetCommand(void)
{
    return &command;
}

const MiniChassisState_t *MiniChassis_GetState(void)
{
    return &state;
}
