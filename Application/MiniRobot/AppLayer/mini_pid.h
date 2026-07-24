#ifndef MINI_PID_H
#define MINI_PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} MiniPidParam_t;

typedef struct {
    MiniPidParam_t param;
    float target;
    float measure;
    float error;
    float last_error;
    float integral;
    float output;
} MiniPid_t;

void MiniPid_Init(MiniPid_t *pid, const MiniPidParam_t *param);
void MiniPid_SetParam(MiniPid_t *pid, const MiniPidParam_t *param);
void MiniPid_Reset(MiniPid_t *pid);
float MiniPid_Calc(MiniPid_t *pid, float target, float measure, float dt_s);

#endif
