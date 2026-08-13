/**
 ******************************************************************************
 * @file    mini_chassis.c
 * @brief   Wheel chassis control logic, including velocity, position, PID, and LQR command generation.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
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
    if (v > limit)
    {
        return limit;
    }
    if (v < -limit)
    {
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
    command.balance_stand = 0U;
    command.servo_shutdown_request = 0U;
    command.last_command_ms = HAL_GetTick();
    position_mode = 0U;
    MiniLqr_Init(&lqr);

    for (uint8_t i = 0U; i < MINI_ROBOT_WHEEL_COUNT; ++i)
    {
        MiniPid_Init(&speed_pid[i], &speed_param);
        MiniPid_Init(&position_pid[i], &pos_param);
        state.wheel_target_rps[i] = 0.0f;
        state.wheel_current_cmd[i] = 0.0f;
        wheel_position_target[i] = 0.0f;
    }
    for (uint8_t i = 0U; i < MINI_LQR_STATE_DIM; ++i)
    {
        state.lqr_state[i] = 0.0f;
    }
    for (uint8_t i = 0U; i < MINI_LQR_INPUT_DIM; ++i)
    {
        state.lqr_output[i] = 0.0f;
    }
    state.lqr_enabled = 0U;
    state.safe = 1U;
    state.fault = 0U;
}

void MiniChassis_SetEnabled(uint8_t enabled)
{
    command.enabled = enabled ? 1U : 0U;
    command.last_command_ms = HAL_GetTick();
    if (!command.enabled)
    {
        command.vx_mps = 0.0f;
        command.wz_rps = 0.0f;
        command.balance_stand = 0U;
        position_mode = 0U;
        MiniLqr_SetEnabled(&lqr, 0U);
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

void MiniChassis_Stand(float pitch_target_rad)
{
    MiniFocMotor_t *left_motor = MiniFoc_GetMotor(0U);
    MiniFocMotor_t *right_motor = MiniFoc_GetMotor(1U);
    MiniWheelLegLqrInput_t lqr_input = {
        .body_pitch_rad = pitch_target_rad,
        .body_pitch_rate_rps = 0.0f,
        .left_wheel_position_rad = (left_motor != 0) ? left_motor->position_rad : 0.0f,
        .left_wheel_speed_rps = 0.0f,
        .right_wheel_position_rad = (right_motor != 0) ? right_motor->position_rad : 0.0f,
        .right_wheel_speed_rps = 0.0f,
        .wheel_radius_m = MINI_ROBOT_WHEEL_RADIUS_M,
    };
    float target[MINI_LQR_STATE_DIM] = {0};

    MiniLqr_BuildWheelLegState(target, &lqr_input);

    command.vx_mps = 0.0f;
    command.wz_rps = 0.0f;
    command.enabled = 1U;
    command.balance_stand = 1U;
    command.last_command_ms = HAL_GetTick();
    position_mode = 0U;
    state.fault = 0U;
    MiniLqr_SetTarget(&lqr, target);
    MiniLqr_SetOutputLimit(&lqr, MINI_BALANCE_LQR_OUTPUT_LIMIT_A);
    MiniLqr_SetEnabled(&lqr, 1U);
    state.lqr_enabled = 1U;
}

void MiniChassis_Sleep(void)
{
    command.vx_mps = 0.0f;
    command.wz_rps = 0.0f;
    command.enabled = 0U;
    command.balance_stand = 0U;
    command.last_command_ms = HAL_GetTick();
    position_mode = 0U;
    state.fault = 0U;
    MiniLqr_SetEnabled(&lqr, 0U);
    state.lqr_enabled = 0U;
    for (uint8_t i = 0U; i < MINI_ROBOT_WHEEL_COUNT; ++i)
    {
        MiniPid_Reset(&speed_pid[i]);
        MiniPid_Reset(&position_pid[i]);
        state.wheel_target_rps[i] = 0.0f;
        state.wheel_current_cmd[i] = 0.0f;
        state.lqr_output[i] = 0.0f;
        MiniFoc_SetCommand(i, MINI_FOC_MODE_STOP, 0.0f);
    }
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
    for (uint8_t i = 0U; i < MINI_LQR_STATE_DIM; ++i)
    {
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
    lqr.x[MINI_LQR_STATE_BODY_PITCH_RAD] = pitch_rad;
    lqr.x[MINI_LQR_STATE_BODY_PITCH_RATE_RPS] = pitch_rate_rps;
    state.lqr_state[MINI_LQR_STATE_BODY_PITCH_RAD] = pitch_rad;
    state.lqr_state[MINI_LQR_STATE_BODY_PITCH_RATE_RPS] = pitch_rate_rps;
}

void MiniChassis_SetPid(MiniPidSlot_t slot, uint8_t wheel, float kp, float ki, float kd)
{
    MiniPidParam_t param;

    if (wheel >= MINI_ROBOT_WHEEL_COUNT)
    {
        return;
    }

    if (slot == MINI_PID_SLOT_SPEED)
    {
        param = speed_pid[wheel].param;
        param.output_limit = MINI_ROBOT_MAX_CURRENT_A;
        param.integral_limit = 20.0f;
        param.kp = kp;
        param.ki = ki;
        param.kd = kd;
        MiniPid_SetParam(&speed_pid[wheel], &param);
    }
    else
    {
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

    MiniWheelLegLqrInput_t lqr_input = {
        .body_pitch_rad = lqr.x[MINI_LQR_STATE_BODY_PITCH_RAD],
        .body_pitch_rate_rps = lqr.x[MINI_LQR_STATE_BODY_PITCH_RATE_RPS],
        .left_wheel_position_rad = (left_motor != 0) ? left_motor->position_rad : 0.0f,
        .left_wheel_speed_rps = (left_motor != 0) ? left_motor->speed_rps : 0.0f,
        .right_wheel_position_rad = (right_motor != 0) ? right_motor->position_rad : 0.0f,
        .right_wheel_speed_rps = (right_motor != 0) ? right_motor->speed_rps : 0.0f,
        .wheel_radius_m = MINI_ROBOT_WHEEL_RADIUS_M,
    };

    MiniLqr_BuildWheelLegState(lqr.x, &lqr_input);
    for (uint8_t i = 0U; i < MINI_LQR_STATE_DIM; ++i)
    {
        state.lqr_state[i] = lqr.x[i];
    }

    state.safe = (command.balance_stand ||
                  ((now - command.last_command_ms) <= MINI_ROBOT_COMMAND_TIMEOUT_MS))
                     ? 1U
                     : 0U;
    if (!state.safe)
    {
        command.enabled = 0U;
        command.balance_stand = 0U;
        command.vx_mps = 0.0f;
        command.wz_rps = 0.0f;
        position_mode = 0U;
        MiniLqr_SetEnabled(&lqr, 0U);
    }

    if (lqr.enabled && command.enabled && state.safe)
    {
        const float left_target_rps =
            clampf(left_mps / (6.2831853f * MINI_ROBOT_WHEEL_RADIUS_M),
                   MINI_ROBOT_MAX_WHEEL_SPEED_RPS);
        const float right_target_rps =
            clampf(right_mps / (6.2831853f * MINI_ROBOT_WHEEL_RADIUS_M),
                   MINI_ROBOT_MAX_WHEEL_SPEED_RPS);

        state.wheel_target_rps[0] = left_target_rps;
        state.wheel_target_rps[1] = right_target_rps;
        float left_lqr_speed = left_target_rps;
        float right_lqr_speed = right_target_rps;

#if (MINI_LQR_WHEEL_STATE_USE_METER != 0U)
        left_lqr_speed = left_mps;
        right_lqr_speed = right_mps;
#endif

        lqr.model.x_ref[MINI_LQR_STATE_LEFT_WHEEL_SPEED] = left_lqr_speed;
        lqr.model.x_ref[MINI_LQR_STATE_RIGHT_WHEEL_SPEED] = right_lqr_speed;
        if (left_target_rps != 0.0f || right_target_rps != 0.0f)
        {
            lqr.model.x_ref[MINI_LQR_STATE_LEFT_WHEEL_DISPLACEMENT] = lqr.x[MINI_LQR_STATE_LEFT_WHEEL_DISPLACEMENT];
            lqr.model.x_ref[MINI_LQR_STATE_RIGHT_WHEEL_DISPLACEMENT] = lqr.x[MINI_LQR_STATE_RIGHT_WHEEL_DISPLACEMENT];
        }
        MiniLqr_Calc(&lqr);
        for (uint8_t i = 0U; i < MINI_LQR_INPUT_DIM; ++i)
        {
            state.lqr_output[i] = lqr.u[i];
            state.wheel_current_cmd[i] = lqr.u[i];
            MiniFoc_SetCommand(i, MINI_FOC_MODE_CURRENT, lqr.u[i]);
        }
        state.lqr_enabled = 1U;
        return;
    }

    state.lqr_enabled = 0U;
    for (uint8_t i = 0U; i < MINI_ROBOT_WHEEL_COUNT; ++i)
    {
        MiniFocMotor_t *motor = MiniFoc_GetMotor(i);
        float measured = (motor != 0) ? motor->speed_rps : 0.0f;
        float pos_measured = (motor != 0) ? motor->position_rad : 0.0f;
        float out;

        if (position_mode)
        {
            state.wheel_target_rps[i] = MiniPid_Calc(&position_pid[i], wheel_position_target[i], pos_measured, dt_s);
        }
        else
        {
            float target_mps = (i == 0U) ? left_mps : right_mps;
            state.wheel_target_rps[i] = clampf(target_mps / (6.2831853f * MINI_ROBOT_WHEEL_RADIUS_M),
                                               MINI_ROBOT_MAX_WHEEL_SPEED_RPS);
        }
        out = MiniPid_Calc(&speed_pid[i], state.wheel_target_rps[i], measured, dt_s);

        if (!command.enabled || !state.safe)
        {
            out = 0.0f;
            MiniPid_Reset(&speed_pid[i]);
            MiniFoc_SetCommand(i, MINI_FOC_MODE_STOP, 0.0f);
        }
        else
        {
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


