#include "mini_pid.h"

static float mini_absf(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static float mini_clampf(float v, float limit)
{
    if (limit <= 0.0f) {
        return v;
    }
    if (v > limit) {
        return limit;
    }
    if (v < -limit) {
        return -limit;
    }
    return v;
}

void MiniPid_Init(MiniPid_t *pid, const MiniPidParam_t *param)
{
    if (pid == 0 || param == 0) {
        return;
    }
    pid->param = *param;
    MiniPid_Reset(pid);
}

void MiniPid_SetParam(MiniPid_t *pid, const MiniPidParam_t *param)
{
    if (pid == 0 || param == 0) {
        return;
    }
    pid->param = *param;
}

void MiniPid_Reset(MiniPid_t *pid)
{
    if (pid == 0) {
        return;
    }
    pid->target = 0.0f;
    pid->measure = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float MiniPid_Calc(MiniPid_t *pid, float target, float measure, float dt_s)
{
    float p_out;
    float i_out;
    float d_out = 0.0f;

    if (pid == 0 || dt_s <= 0.0f) {
        return 0.0f;
    }

    pid->target = target;
    pid->measure = measure;
    pid->error = target - measure;

    pid->integral += pid->error * dt_s;
    pid->integral = mini_clampf(pid->integral, pid->param.integral_limit);

    p_out = pid->param.kp * pid->error;
    i_out = pid->param.ki * pid->integral;
    if (mini_absf(dt_s) > 0.000001f) {
        d_out = pid->param.kd * (pid->error - pid->last_error) / dt_s;
    }

    pid->output = mini_clampf(p_out + i_out + d_out, pid->param.output_limit);
    pid->last_error = pid->error;
    return pid->output;
}
