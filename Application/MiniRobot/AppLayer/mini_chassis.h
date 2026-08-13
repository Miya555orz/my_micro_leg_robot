/**
 ******************************************************************************
 * @file    mini_chassis.h
 * @brief   Public chassis control API and chassis command/state data structures.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#ifndef MINI_CHASSIS_H
#define MINI_CHASSIS_H

#include "mini_pid.h"
#include "mini_lqr.h"
#include <stdint.h>

typedef enum
{
    MINI_PID_SLOT_SPEED = 0,
    MINI_PID_SLOT_POSITION = 1,
} MiniPidSlot_t;

typedef struct
{
    float vx_mps;
    float wz_rps;
    uint8_t enabled;
    uint8_t balance_stand;
    uint8_t servo_shutdown_request;
    uint32_t last_command_ms;
} MiniChassisCommand_t;

typedef struct
{
    float wheel_target_rps[2];
    float wheel_current_cmd[2];
    float lqr_state[MINI_LQR_STATE_DIM];
    float lqr_output[MINI_LQR_INPUT_DIM];
    uint8_t lqr_enabled;
    uint8_t safe;
    uint8_t fault;
} MiniChassisState_t;

/**
 * @brief External API: MiniChassis_Init.
 * @param None.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_Init(void);
/**
 * @brief External API: MiniChassis_SetEnabled.
 * @param enabled Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetEnabled(uint8_t enabled);
/**
 * @brief External API: MiniChassis_SetVelocity.
 * @param vx_mps Input/output value owned by the caller; units follow module config and structure comments.
 * @param wz_rps Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetVelocity(float vx_mps, float wz_rps);
/**
 * @brief External API: MiniChassis_SetWheelPosition.
 * @param left_rad Input/output value owned by the caller; units follow module config and structure comments.
 * @param right_rad Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetWheelPosition(float left_rad, float right_rad);
/**
 * @brief External API: MiniChassis_Stand.
 * @param pitch_target_rad Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_Stand(float pitch_target_rad);
/**
 * @brief External API: MiniChassis_Sleep.
 * @param None.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_Sleep(void);
/**
 * @brief External API: MiniChassis_SetPid.
 * @param slot Input/output value owned by the caller; units follow module config and structure comments.
 * @param wheel Input/output value owned by the caller; units follow module config and structure comments.
 * @param kp Input/output value owned by the caller; units follow module config and structure comments.
 * @param ki Input/output value owned by the caller; units follow module config and structure comments.
 * @param kd Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetPid(MiniPidSlot_t slot, uint8_t wheel, float kp, float ki, float kd);
/**
 * @brief External API: MiniChassis_SetLqrEnabled.
 * @param enabled Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetLqrEnabled(uint8_t enabled);
/**
 * @brief External API: MiniChassis_SetLqrGain.
 * @param input Input/output value owned by the caller; units follow module config and structure comments.
 * @param state Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetLqrGain(uint8_t input, uint8_t state, float value);
/**
 * @brief External API: MiniChassis_SetLqrA.
 * @param row Input/output value owned by the caller; units follow module config and structure comments.
 * @param col Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetLqrA(uint8_t row, uint8_t col, float value);
/**
 * @brief External API: MiniChassis_SetLqrB.
 * @param row Input/output value owned by the caller; units follow module config and structure comments.
 * @param input Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetLqrB(uint8_t row, uint8_t input, float value);
/**
 * @brief External API: MiniChassis_SetLqrState.
 * @param lqr_state Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetLqrState(const float lqr_state[MINI_LQR_STATE_DIM]);
/**
 * @brief External API: MiniChassis_SetLqrTarget.
 * @param lqr_target Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetLqrTarget(const float lqr_target[MINI_LQR_STATE_DIM]);
/**
 * @brief External API: MiniChassis_SetLqrFeedforward.
 * @param input Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetLqrFeedforward(uint8_t input, float value);
/**
 * @brief External API: MiniChassis_SetLqrOutputLimit.
 * @param limit Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetLqrOutputLimit(float limit);
/**
 * @brief External API: MiniChassis_SetMeasuredAttitude.
 * @param pitch_rad Input/output value owned by the caller; units follow module config and structure comments.
 * @param pitch_rate_rps Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_SetMeasuredAttitude(float pitch_rad, float pitch_rate_rps);
/**
 * @brief External API: MiniChassis_RequestServoShutdown.
 * @param None.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_RequestServoShutdown(void);
/**
 * @brief External API: MiniChassis_ClearServoShutdownRequest.
 * @param None.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_ClearServoShutdownRequest(void);
/**
 * @brief External API: MiniChassis_Update.
 * @param dt_s Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniChassis_Update(float dt_s);
const MiniChassisCommand_t *MiniChassis_GetCommand(void);
const MiniChassisState_t *MiniChassis_GetState(void);

#endif

