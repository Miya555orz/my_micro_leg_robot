#ifndef __PID_Instance_H
#define __PID_Instance_H

#include "PID.h"
#include "car_info.h"

typedef struct Link_Pid_struct_t {
    pid_ctrl_t *roll_cal[Leg_Num];

    pid_ctrl_t *sync_cal[Leg_Num];

    pid_ctrl_t *length_cal[Leg_Num];
    pid_ctrl_t *length_speed_cal[Leg_Num];

    pid_ctrl_t *yaw_imu_cal[Leg_Num];               // IMU-based yaw outer
    pid_ctrl_t *yaw_imu_speed_cal[Leg_Num];         // IMU-based yaw inner
    pid_ctrl_t *cycle_yaw_imu_speed_cal[Leg_Num];   // 小陀螺模式专用速度PID
    pid_ctrl_t *cycle_wheel_speed_err_Pid[Leg_Num]; // 小陀螺模式轮速误差PID
    pid_ctrl_t *yaw_mec_cal[Leg_Num];               // motor-angle-based yaw outer
    pid_ctrl_t *yaw_mec_speed_cal[Leg_Num];         // motor-angle-based yaw inner

    pid_ctrl_t *vir_phi0_cal[Leg_Num];
    pid_ctrl_t *vir_phi0_speed_cal[Leg_Num];
    pid_ctrl_t *vir_phi0d1_cal[Leg_Num];

    pid_ctrl_t *phi0_cal[Leg_Num];
} Chassis_Pid_t;

extern Chassis_Pid_t chassis_PID;
// 自救相关pid结构体
extern pid_ctrl_t PRNormalBackwardLeg_vir_phi0_d1_Pid[Leg_Num];
extern pid_ctrl_t ForwardFlip_vir_phi0_d1_Pid[Leg_Num];
extern pid_ctrl_t BackwardFlip_vir_phi0_d1_Pid[Leg_Num];
extern pid_ctrl_t Manual_Rescue_vir_phi0_d1_Pid[Leg_Num];
extern pid_ctrl_t My_Link_vir_phi0_d1_Pid[Leg_Num];

#endif
