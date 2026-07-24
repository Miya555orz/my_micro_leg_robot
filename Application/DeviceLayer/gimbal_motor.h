#ifndef __GIMBAL_MOTOR_H
#define __GIMBAL_MOTOR_H

#include "DM_Motor.h"
#include "drv_can.h"

#define YAW_TX_ID 0x03
#define YAW_RX_ID 0x33

typedef struct
{
    pid_ctrl_t *yaw_gyro_outer; // yaw陀螺仪角度环外环
    pid_ctrl_t *yaw_gyro_inner; // yaw陀螺仪角度环内环
    pid_ctrl_t *yaw_mec_outer;  // yaw机械角度环外环
    pid_ctrl_t *yaw_mec_inner;  // yaw机械角度环内环

    pid_ctrl_t *pitch_gyro_outer; // pitch陀螺仪角度环外环
    pid_ctrl_t *pitch_gyro_inner; // pitch陀螺仪角度环内环
    pid_ctrl_t *pitch_mec_outer;  // pitch机械角度环外环
    pid_ctrl_t *pitch_mec_inner;  // pitch机械角度环内环

} gimbal_pid_info_t;
extern gimbal_pid_info_t gimbal_pid;
extern Motor_DM_t Yaw_Motor;

#endif
