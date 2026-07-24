#include "mini_lqr.h"

#include "mini_robot_config.h"
#include <string.h>

static const float default_a[MINI_LQR_STATE_DIM][MINI_LQR_STATE_DIM] = MINI_LQR_DEFAULT_A;
static const float default_b[MINI_LQR_STATE_DIM][MINI_LQR_INPUT_DIM] = MINI_LQR_DEFAULT_B;
static const float default_k[MINI_LQR_INPUT_DIM][MINI_LQR_STATE_DIM] = MINI_LQR_DEFAULT_K;
static const float default_x_ref[MINI_LQR_STATE_DIM] = MINI_LQR_DEFAULT_X_REF;
static const float default_u_ff[MINI_LQR_INPUT_DIM] = MINI_LQR_DEFAULT_U_FF;

static float clampf(float v, float limit)
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

void MiniLqr_Init(MiniLqr_t *lqr)
{
    if (lqr == 0) {
        return;
    }

    memset(lqr, 0, sizeof(*lqr));
    memcpy(lqr->model.a, default_a, sizeof(lqr->model.a));
    memcpy(lqr->model.b, default_b, sizeof(lqr->model.b));
    memcpy(lqr->model.k, default_k, sizeof(lqr->model.k));
    memcpy(lqr->model.x_ref, default_x_ref, sizeof(lqr->model.x_ref));
    memcpy(lqr->model.u_ff, default_u_ff, sizeof(lqr->model.u_ff));
    lqr->model.output_limit = MINI_LQR_OUTPUT_LIMIT_A;

    /* State order for tuning:
     * x0 body_pitch_rad, x1 body_pitch_rate_rps,
     * x2 wheel_pos_l_rad, x3 wheel_speed_l_rps,
     * x4 wheel_pos_r_rad, x5 wheel_speed_r_rps.
     */
}

void MiniLqr_SetEnabled(MiniLqr_t *lqr, uint8_t enabled)
{
    if (lqr == 0) {
        return;
    }
    lqr->enabled = enabled ? 1U : 0U;
}

void MiniLqr_SetGain(MiniLqr_t *lqr, uint8_t input, uint8_t state, float value)
{
    if (lqr == 0 || input >= MINI_LQR_INPUT_DIM || state >= MINI_LQR_STATE_DIM) {
        return;
    }
    lqr->model.k[input][state] = value;
}

void MiniLqr_SetA(MiniLqr_t *lqr, uint8_t row, uint8_t col, float value)
{
    if (lqr == 0 || row >= MINI_LQR_STATE_DIM || col >= MINI_LQR_STATE_DIM) {
        return;
    }
    lqr->model.a[row][col] = value;
}

void MiniLqr_SetB(MiniLqr_t *lqr, uint8_t row, uint8_t input, float value)
{
    if (lqr == 0 || row >= MINI_LQR_STATE_DIM || input >= MINI_LQR_INPUT_DIM) {
        return;
    }
    lqr->model.b[row][input] = value;
}

void MiniLqr_SetState(MiniLqr_t *lqr, const float state[MINI_LQR_STATE_DIM])
{
    if (lqr == 0 || state == 0) {
        return;
    }
    memcpy(lqr->x, state, sizeof(lqr->x));
}

void MiniLqr_SetTarget(MiniLqr_t *lqr, const float target[MINI_LQR_STATE_DIM])
{
    if (lqr == 0 || target == 0) {
        return;
    }
    memcpy(lqr->model.x_ref, target, sizeof(lqr->model.x_ref));
}

void MiniLqr_SetFeedforward(MiniLqr_t *lqr, uint8_t input, float value)
{
    if (lqr == 0 || input >= MINI_LQR_INPUT_DIM) {
        return;
    }
    lqr->model.u_ff[input] = value;
}

void MiniLqr_SetOutputLimit(MiniLqr_t *lqr, float limit)
{
    if (lqr == 0) {
        return;
    }
    lqr->model.output_limit = limit;
}

void MiniLqr_Calc(MiniLqr_t *lqr)
{
    if (lqr == 0) {
        return;
    }

    for (uint8_t input = 0U; input < MINI_LQR_INPUT_DIM; ++input) {
        float out = lqr->model.u_ff[input];

        for (uint8_t state = 0U; state < MINI_LQR_STATE_DIM; ++state) {
            const float err = lqr->x[state] - lqr->model.x_ref[state];
            out -= lqr->model.k[input][state] * err;
        }
        lqr->u[input] = clampf(out, lqr->model.output_limit);
    }
}
