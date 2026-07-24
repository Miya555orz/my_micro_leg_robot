#include "shoot_motor.h"

static pid_ctrl_t dail_speed =
    {
        .kp = 0.15f,
        .ki = 0.2f,
        .kd = 0.f,
        .integral_max = 500.f,
        .out_max = 1000.f,
};
static pid_ctrl_t dail_position_out =
    {
        .kp = 0.10,
        .ki = 0.f,
        .kd = 0.f,
        .integral_max = 0.f,
        .out_max = 1000000.f,
};
static pid_ctrl_t dail_position_inner =
    {
        .kp = 0.06f,
        .ki = 0.f,
        .kd = 0.f,
        .integral_max = 0.f,
        .out_max = 1000.f,
};

dail_pid_info_t dail_pid = {
    .speed = &dail_speed,
    .position_inner = &dail_position_inner,
    .position_outer = &dail_position_out,
};

KT_motor_t dail_motor = {

    .KT_motor_info = {
        .id = {
            .tx_id = DAIL_MOTOR_ID,
            .rx_id = DAIL_MOTOR_ID,
            .drive_type = M_CAN1,
            .motor_type = KT4005,
        },
        .tx_info = {
            .angle_single_Control = 0,
            .angle_single_Control_maxSpeed = 0,
            .angle_single_Control_spinDirection = 0,
            .angle_add_Control = 0,
            .angle_add_Control_maxSpeed = 0,
            .angle_sum_Control = 0,
            .angle_sum_Control_maxSpeed = 0,
            .iqControl = 0,
            .speedControl = 0,
        },
    },
    .init = KT_motor_class_init,
};
