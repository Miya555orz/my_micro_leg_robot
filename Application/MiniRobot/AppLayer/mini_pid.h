/**
 ******************************************************************************
 * @file    mini_pid.h
 * @brief   Reusable PID controller interface and parameter/state data structures.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#ifndef MINI_PID_H
#define MINI_PID_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} MiniPidParam_t;

typedef struct
{
    MiniPidParam_t param;
    float target;
    float measure;
    float error;
    float last_error;
    float integral;
    float output;
} MiniPid_t;

/**
 * @brief External API: MiniPid_Init.
 * @param pid Input/output value owned by the caller; units follow module config and structure comments.
 * @param param Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniPid_Init(MiniPid_t *pid, const MiniPidParam_t *param);
/**
 * @brief External API: MiniPid_SetParam.
 * @param pid Input/output value owned by the caller; units follow module config and structure comments.
 * @param param Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniPid_SetParam(MiniPid_t *pid, const MiniPidParam_t *param);
/**
 * @brief External API: MiniPid_Reset.
 * @param pid Input/output value owned by the caller; units follow module config and structure comments.
 * @return None.
 * @note Validate hardware wiring, IDs, and call period before using this API in closed-loop control.
 */
void MiniPid_Reset(MiniPid_t *pid);
float MiniPid_Calc(MiniPid_t *pid, float target, float measure, float dt_s);

#endif

