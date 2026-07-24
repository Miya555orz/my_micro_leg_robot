#ifndef MINI_LQR_H
#define MINI_LQR_H

#include <stdint.h>

#define MINI_LQR_STATE_DIM 6U
#define MINI_LQR_INPUT_DIM 2U

typedef struct {
    float a[MINI_LQR_STATE_DIM][MINI_LQR_STATE_DIM];
    float b[MINI_LQR_STATE_DIM][MINI_LQR_INPUT_DIM];
    float k[MINI_LQR_INPUT_DIM][MINI_LQR_STATE_DIM];
    float x_ref[MINI_LQR_STATE_DIM];
    float u_ff[MINI_LQR_INPUT_DIM];
    float output_limit;
} MiniLqrModel_t;

typedef struct {
    MiniLqrModel_t model;
    float x[MINI_LQR_STATE_DIM];
    float u[MINI_LQR_INPUT_DIM];
    uint8_t enabled;
} MiniLqr_t;

void MiniLqr_Init(MiniLqr_t *lqr);
void MiniLqr_SetEnabled(MiniLqr_t *lqr, uint8_t enabled);
void MiniLqr_SetGain(MiniLqr_t *lqr, uint8_t input, uint8_t state, float value);
void MiniLqr_SetA(MiniLqr_t *lqr, uint8_t row, uint8_t col, float value);
void MiniLqr_SetB(MiniLqr_t *lqr, uint8_t row, uint8_t input, float value);
void MiniLqr_SetState(MiniLqr_t *lqr, const float state[MINI_LQR_STATE_DIM]);
void MiniLqr_SetTarget(MiniLqr_t *lqr, const float target[MINI_LQR_STATE_DIM]);
void MiniLqr_SetFeedforward(MiniLqr_t *lqr, uint8_t input, float value);
void MiniLqr_SetOutputLimit(MiniLqr_t *lqr, float limit);
void MiniLqr_Calc(MiniLqr_t *lqr);

#endif
