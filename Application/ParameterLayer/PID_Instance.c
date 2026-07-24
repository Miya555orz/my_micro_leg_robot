#include "PID_Instance.h"

/*
 * PID实例索引（便于搜索查阅）
 * 腿长外环: My_Link_Length_Pid
 * 腿长内环: My_Link_Length_Speed_Pid
 * 单环Roll轴控制: My_Link_Roll_Pid
 * C_Boss:IMU_Yaw外环: My_yaw_imu_Pid
 * C_Boss:IMU_Yaw内环: My_yaw_speed_imu_Pid
 * C_Follow:电机Angle_Yaw外环: My_yaw_mec_Pid
 * C_Follow:电机Angle_Yaw内环: My_yaw_mec_speed_Pid
 * 双腿协调: My_Link_sync_Pid
 * vir_phi0外环: My_Link_vir_phi0_Pid
 * vir_phi0内环: My_Link_vir_phi0_speed_Pid
 * 自救vir_phi0d1默认: My_Link_vir_phi0_d1_Pid
 * 自救vir_phi0d1-PRNormalBackwardLeg: PRNormalBackwardLeg_vir_phi0_d1_Pid
 * 自救vir_phi0d1-ForwardFlip: ForwardFlip_vir_phi0_d1_Pid
 * 自救vir_phi0d1-BackwardFlip: BackwardFlip_vir_phi0_d1_Pid
 * 自救phi0位置环: My_Link_phi0_Pid
 * PID总表: chassis_PID
 */

/* 腿长控制外环 */
pid_ctrl_t My_Link_Length_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 30.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 20.f,
            },
        [L_Leg] =
            {
                .kp = 30.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 20.f,
            },
};

/* 腿长控制内环 */
pid_ctrl_t My_Link_Length_Speed_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 60.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 5.f,
                .out_max = 200,
            },
        [L_Leg] =
            {
                .kp = 60.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 5.f,
                .out_max = 200,
            },
};

/* 单环Roll轴控制 */
pid_ctrl_t My_Link_Roll_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 0.4f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 1.f,
                .out_max = 0.15f,
            },
        [L_Leg] =
            {
                .kp = 0.4f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 1.f,
                .out_max = 0.15f,
            },
};

/* C_Boss:IMU_Yaw外环 */
pid_ctrl_t My_yaw_imu_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 30.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 400.f,
            },
        [L_Leg] =
            {
                .kp = 30.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 400.f,
            },
};

/* C_Boss:IMU_Yaw内环 */
pid_ctrl_t My_yaw_speed_imu_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 0.04f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 20.f,
            },
        [L_Leg] =
            {
                .kp = 0.04f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 20.f,
            },
};

/* 小陀螺模式专用 - imu_yaw速度控制 */
pid_ctrl_t My_cycle_yaw_imu_speed_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 0.04f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 5.f,
            },
        [L_Leg] =
            {
                .kp = 0.04f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 5.f,
            },
};

/* 调整偏心小陀螺用 两轮转速差环 */
pid_ctrl_t cycle_wheel_speed_err_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 0.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 5.f,
            },
        [L_Leg] =
            {
                .kp = 0.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 5.f,
            },
};
/* C_Follow:Yaw外环 */
pid_ctrl_t My_yaw_mec_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 12.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 1000.f,
            },
        [L_Leg] =
            {
                .kp = 12.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 1000.f,
            },
};

/* C_Follow:Yaw内环 */
pid_ctrl_t My_yaw_mec_speed_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 0.1f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 10.f,
            },
        [L_Leg] =
            {
                .kp = 0.1f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 10.f,
            },
};

/* 单环双腿协调控制 */
pid_ctrl_t My_Link_sync_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 10.f,
                .ki = 0.01f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 300.f,
                .out_max = 10.f,
            },
        [L_Leg] =
            {
                .kp = 10.f,
                .ki = 0.01f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 300.f,
                .out_max = 10.f,
            },
};

/* vir_phi0控制外环 */
pid_ctrl_t My_Link_vir_phi0_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 15.f,
                .ki = 0.15f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 5.f,
            },
        [L_Leg] =
            {
                .kp = 15.f,
                .ki = 0.15f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 5.f,
            },
};
/* vir_phi0控制内环 */
pid_ctrl_t My_Link_vir_phi0_speed_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 6.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 10.f,
            },
        [L_Leg] =
            {
                .kp = 6.f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 10.f,
            },
};

/*--------------------------自救控腿角速度pid参数 begin----------------------------*/
/* vir_phi0d1 单速度环控制————默认参数°为单位 */
pid_ctrl_t My_Link_vir_phi0_d1_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 0.2f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 2.f,
            },
        [L_Leg] =
            {
                .kp = 0.2f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2.f,
                .out_max = 2.f,
            },
};

/* vir_phi0d1 单速度环控制———— PRNormalBackwardLeg自救参数°为单位 */
pid_ctrl_t PRNormalBackwardLeg_vir_phi0_d1_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 0.2f,
                .ki = 0.01f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2000.f,
                .out_max = 50.f,
            },
        [L_Leg] =
            {
                .kp = 0.2f,
                .ki = 0.01f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2000.f,
                .out_max = 50.f,
            },
};

/* vir_phi0d1 单速度环控制———— ForwardFlip自救参数°为单位 */
pid_ctrl_t ForwardFlip_vir_phi0_d1_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 0.2f,
                .ki = 0.01f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2000.f,
                .out_max = 50.f,
            },
        [L_Leg] =
            {
                .kp = 0.2f,
                .ki = 0.01f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2000.f,
                .out_max = 50.f,
            },
};

/* vir_phi0d1 单速度环控制———— BackwardFlip自救参数°为单位 */
pid_ctrl_t BackwardFlip_vir_phi0_d1_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 0.2f,
                .ki = 0.05f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2000.f,
                .out_max = 1000.f,
            },
        [L_Leg] =
            {
                .kp = 0.2f,
                .ki = 0.05f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 2000.f,
                .out_max = 1000.f,
            },
};

/* vir_phi0d1 单速度环控制———— 操作手手动自救参数°为单位 */
pid_ctrl_t Manual_Rescue_vir_phi0_d1_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 0.2f,
                .ki = 0.0f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 1000.f,
                .out_max = 500.f,
            },
        [L_Leg] =
            {
                .kp = 0.2f,
                .ki = 0.0f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 1000.f,
                .out_max = 500.f,
            },
};
/*--------------------------自救控腿角速度pid参数 end----------------------------*/
/* 单环phi0控制 */
pid_ctrl_t My_Link_phi0_Pid[Leg_Num] =
    {
        [R_Leg] =
            {
                .kp = 1.2f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 0.f,
                .out_max = 15.f,
            },
        [L_Leg] =
            {
                .kp = 1.2f,
                .ki = 0.f,
                .kd = 0.f,
                .a = 1.f,
                .integral_max = 0.f,
                .out_max = 15.f,
            },
};

Chassis_Pid_t chassis_PID = {
    // roll pid
    .roll_cal[R_Leg] = &My_Link_Roll_Pid[R_Leg],
    .roll_cal[L_Leg] = &My_Link_Roll_Pid[L_Leg],
    // 双腿协调pid
    .sync_cal[R_Leg] = &My_Link_sync_Pid[R_Leg],
    .sync_cal[L_Leg] = &My_Link_sync_Pid[L_Leg],
    // 腿长外环pid
    .length_cal[R_Leg] = &My_Link_Length_Pid[R_Leg],
    .length_cal[L_Leg] = &My_Link_Length_Pid[L_Leg],
    // 腿长内环pid
    .length_speed_cal[R_Leg] = &My_Link_Length_Speed_Pid[R_Leg],
    .length_speed_cal[L_Leg] = &My_Link_Length_Speed_Pid[L_Leg],
    // IMU yaw外环pid
    .yaw_imu_cal[R_Leg] = &My_yaw_imu_Pid[R_Leg],
    .yaw_imu_cal[L_Leg] = &My_yaw_imu_Pid[L_Leg],
    // IMU yaw内环pid
    .yaw_imu_speed_cal[R_Leg] = &My_yaw_speed_imu_Pid[R_Leg],
    .yaw_imu_speed_cal[L_Leg] = &My_yaw_speed_imu_Pid[L_Leg],
    // motor-angle-based yaw outer pid
    .yaw_mec_cal[R_Leg] = &My_yaw_mec_Pid[R_Leg],
    .yaw_mec_cal[L_Leg] = &My_yaw_mec_Pid[L_Leg],
    // motor-angle-based yaw inner pid
    .yaw_mec_speed_cal[R_Leg] = &My_yaw_mec_speed_Pid[R_Leg],
    .yaw_mec_speed_cal[L_Leg] = &My_yaw_mec_speed_Pid[L_Leg],

    // vir_phi0外环pid
    .vir_phi0_cal[R_Leg] = &My_Link_vir_phi0_Pid[R_Leg],
    .vir_phi0_cal[L_Leg] = &My_Link_vir_phi0_Pid[L_Leg],
    // vir_phi0内环pid
    .vir_phi0_speed_cal[R_Leg] = &My_Link_vir_phi0_speed_Pid[R_Leg],
    .vir_phi0_speed_cal[L_Leg] = &My_Link_vir_phi0_speed_Pid[L_Leg],
    // 小陀螺模式专用 - imu_yaw速度环pid
    .cycle_yaw_imu_speed_cal[R_Leg] = &My_cycle_yaw_imu_speed_Pid[R_Leg],
    .cycle_yaw_imu_speed_cal[L_Leg] = &My_cycle_yaw_imu_speed_Pid[L_Leg],

    .cycle_wheel_speed_err_Pid[R_Leg] = &cycle_wheel_speed_err_Pid[R_Leg],
    .cycle_wheel_speed_err_Pid[L_Leg] = &cycle_wheel_speed_err_Pid[L_Leg],
    // 自救vir_phi0速度环pid
    .vir_phi0d1_cal[R_Leg] = &My_Link_vir_phi0_d1_Pid[R_Leg],
    .vir_phi0d1_cal[L_Leg] = &My_Link_vir_phi0_d1_Pid[L_Leg],
    // 自救phi0位置环pid
    .phi0_cal[R_Leg] = &My_Link_phi0_Pid[R_Leg],
    .phi0_cal[L_Leg] = &My_Link_phi0_Pid[L_Leg],
};
