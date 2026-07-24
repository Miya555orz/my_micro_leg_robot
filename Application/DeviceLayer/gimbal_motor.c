#include "gimbal_motor.h"
#include "DM_Motor.h"

// yaw陀螺仪外环pid
pid_ctrl_t yaw_gyro_out =
    {
        .kp = 30.f,
        .ki = 0.08f,
        .kd = 0.f,
        .integral_max = 100.f,
        .out_max = 700.f,
};

// yaw陀螺仪内环pid
pid_ctrl_t yaw_gyro_inner =
    {
        .kp = 0.02f,
        .ki = 0.f,
        .kd = 0.f,
        .integral_max = 0.f,
        .out_max = 12.f,
};

// pitch陀螺仪外环pid
pid_ctrl_t pitch_gyro_out =
    {
        .kp = 35.f,
        .ki = 0.13f,
        .kd = 0.f,
        .integral_max = 500.f,
        .out_max = 1000.f,
};

// pitch陀螺仪内环pid
pid_ctrl_t pitch_gyro_inner =
    {
        .kp = 0.025f,
        .ki = 0.f,
        .kd = 0.f,
        .integral_max = 0.f,
        .out_max = 12.f,
};

// yaw机械外环pid
pid_ctrl_t yaw_mec_out =
    {
        .kp = 80.f,
        .ki = 0.f,
        .kd = 0.f,
        .integral_max = 400.f,
        .out_max = 1000.f,
};

// yaw机械内环pid
pid_ctrl_t yaw_mec_inner =
    {
        .kp = 0.01f,
        .ki = 0.f,
        .kd = 0.f,
        .integral_max = 0.f,
        .out_max = 12.f,
};

// pitch机械外环pid
pid_ctrl_t pitch_mec_out =
    {
        .kp = 30.f,
        .ki = 0.1f,
        .kd = 0.f,
        .integral_max = 500.f,
        .out_max = 1000.f,
};

// pitch机械内环pid
pid_ctrl_t pitch_mec_inner =
    {
        .kp = 0.01f,
        .ki = 0.f,
        .kd = 0.f,
        .integral_max = 0.f,
        .out_max = 12.f,
};
gimbal_pid_info_t gimbal_pid =
    {
        .yaw_gyro_inner = &yaw_gyro_inner,
        .yaw_gyro_outer = &yaw_gyro_out,
        .pitch_gyro_inner = &pitch_gyro_inner,
        .pitch_gyro_outer = &pitch_gyro_out,
        .yaw_mec_inner = &yaw_mec_inner,
        .yaw_mec_outer = &yaw_mec_out,
        .pitch_mec_inner = &pitch_mec_inner,
        .pitch_mec_outer = &pitch_mec_out,
};

Motor_DM_Born_Info_t Yaw_Born_Info =
    {
        .txId = YAW_TX_ID,
        .hcan = &hfdcan2,
};
Motor_DM_Rx_Info_t Yaw_Rx_Info;
Motor_DM_Tx_Info_t Yaw_Tx_Info;
Motor_DM_State_t Yaw_State;
Motor_DM_Ctrl_Info_t Yaw_Ctrl;
Motor_DM_t Yaw_Motor =
    {
        .born_info = &Yaw_Born_Info,
        .rx_info = &Yaw_Rx_Info,
        .tx_info = &Yaw_Tx_Info,
        .state = &Yaw_State,
        .ctrl = &Yaw_Ctrl,
        .single_init = &DM_Single_Motor_Init,
        .type = dm_4310,
};
