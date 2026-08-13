/**
 ******************************************************************************
 * @file    mini_lqr.h
 * @brief   Reusable LQR model interface and state/input dimension definitions.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#ifndef MINI_LQR_H
#define MINI_LQR_H

#include <stdint.h>

#define MINI_LQR_STATE_DIM 6U
#define MINI_LQR_INPUT_DIM 2U

/* Wheel-leg LQR bridge state order.
 * The team model describes theta/theta_dot, wheel displacement/speed, and body
 * pitch/pitch_dot. Current hardware has no independent leg angle sensor yet, so
 * theta is temporarily represented by body pitch. Keep this mapping explicit so
 * later five-link/VMC data can replace the placeholder without changing K users.
 */
typedef enum
{
    MINI_LQR_STATE_BODY_PITCH_RAD = 0,
    MINI_LQR_STATE_BODY_PITCH_RATE_RPS = 1,
    MINI_LQR_STATE_LEFT_WHEEL_DISPLACEMENT = 2,
    MINI_LQR_STATE_LEFT_WHEEL_SPEED = 3,
    MINI_LQR_STATE_RIGHT_WHEEL_DISPLACEMENT = 4,
    MINI_LQR_STATE_RIGHT_WHEEL_SPEED = 5,
} MiniLqrStateIndex_t;

typedef enum
{
    MINI_LQR_INPUT_LEFT_WHEEL_CURRENT_A = 0,
    MINI_LQR_INPUT_RIGHT_WHEEL_CURRENT_A = 1,
} MiniLqrInputIndex_t;

typedef struct
{
    float body_pitch_rad;          /* Body pitch angle from MPU6050 (rad). */
    float body_pitch_rate_rps;     /* Body pitch angular velocity (rad/s). */
    float left_wheel_position_rad; /* Left hub integrated rotor/wheel position (rad). */
    float left_wheel_speed_rps;    /* Left hub wheel speed (rad/s). */
    float right_wheel_position_rad; /* Right hub integrated rotor/wheel position (rad). */
    float right_wheel_speed_rps;   /* Right hub wheel speed (rad/s). */
    float wheel_radius_m;          /* Wheel radius used when meter-state mode is enabled (m). */
} MiniWheelLegLqrInput_t;
typedef struct
{
    float a[MINI_LQR_STATE_DIM][MINI_LQR_STATE_DIM];
    float b[MINI_LQR_STATE_DIM][MINI_LQR_INPUT_DIM];
    float k[MINI_LQR_INPUT_DIM][MINI_LQR_STATE_DIM];
    float x_ref[MINI_LQR_STATE_DIM];
    float u_ff[MINI_LQR_INPUT_DIM];
    float output_limit;
} MiniLqrModel_t;

typedef struct
{
    MiniLqrModel_t model;
    float x[MINI_LQR_STATE_DIM];
    float u[MINI_LQR_INPUT_DIM];
    uint8_t enabled;
} MiniLqr_t;

/**
 * @brief External API: MiniLqr_Init.
 * @param lqr Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniLqr_Init(MiniLqr_t *lqr);
/**
 * @brief External API: MiniLqr_SetEnabled.
 * @param lqr Input/output value owned by the caller; units follow module config and structure comments.
 * @param enabled Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniLqr_SetEnabled(MiniLqr_t *lqr, uint8_t enabled);
/**
 * @brief External API: MiniLqr_SetGain.
 * @param lqr Input/output value owned by the caller; units follow module config and structure comments.
 * @param input Input/output value owned by the caller; units follow module config and structure comments.
 * @param state Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniLqr_SetGain(MiniLqr_t *lqr, uint8_t input, uint8_t state, float value);
/**
 * @brief External API: MiniLqr_SetA.
 * @param lqr Input/output value owned by the caller; units follow module config and structure comments.
 * @param row Input/output value owned by the caller; units follow module config and structure comments.
 * @param col Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniLqr_SetA(MiniLqr_t *lqr, uint8_t row, uint8_t col, float value);
/**
 * @brief External API: MiniLqr_SetB.
 * @param lqr Input/output value owned by the caller; units follow module config and structure comments.
 * @param row Input/output value owned by the caller; units follow module config and structure comments.
 * @param input Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniLqr_SetB(MiniLqr_t *lqr, uint8_t row, uint8_t input, float value);
/**
 * @brief External API: MiniLqr_SetState.
 * @param lqr Input/output value owned by the caller; units follow module config and structure comments.
 * @param state Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniLqr_SetState(MiniLqr_t *lqr, const float state[MINI_LQR_STATE_DIM]);
/**
 * @brief External API: MiniLqr_SetTarget.
 * @param lqr Input/output value owned by the caller; units follow module config and structure comments.
 * @param target Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniLqr_SetTarget(MiniLqr_t *lqr, const float target[MINI_LQR_STATE_DIM]);
/**
 * @brief External API: MiniLqr_BuildWheelLegState.
 * @param state Output state vector, ordered by MiniLqrStateIndex_t.
 * @param input Physical attitude and wheel feedback used to build the state.
 * @return None.
 * @note This is the adapter between the team wheel-leg model and current hardware feedback.
 */
void MiniLqr_BuildWheelLegState(float state[MINI_LQR_STATE_DIM], const MiniWheelLegLqrInput_t *input);
/**
 * @brief External API: MiniLqr_SetFeedforward.
 * @param lqr Input/output value owned by the caller; units follow module config and structure comments.
 * @param input Input/output value owned by the caller; units follow module config and structure comments.
 * @param value Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniLqr_SetFeedforward(MiniLqr_t *lqr, uint8_t input, float value);
/**
 * @brief External API: MiniLqr_SetOutputLimit.
 * @param lqr Input/output value owned by the caller; units follow module config and structure comments.
 * @param limit Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniLqr_SetOutputLimit(MiniLqr_t *lqr, float limit);
/**
 * @brief External API: MiniLqr_Calc.
 * @param lqr Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniLqr_Calc(MiniLqr_t *lqr);

#endif



