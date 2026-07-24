#ifndef __GIMBAL_H

#define __GIMBAL_H

/* Includes ------------------------------------------------------------------*/

#include "rp_config.h"

#include "communicate.h"

#include "chassis_motor.h"

#include "gimbal_motor.h"

#include "bmi.h"

#include "rc_sensor.h"

#include "Balance.h"

#include "DM_Motor.h"

#include "rp_device_config.h"

#include "rp_math.h"

#include "communicate.h"

#include <stdint.h>

#define YAW_MOTOR_ANGLE_MIDDLE     (-2.4147625f) // YAW电机中值

#define PITCH_MOTOR_ENCODER_MIDDLE (1.72887063f) // pitch电机编码器中值

#define GIMBAL_MAX_MEC_ANGEL       (35.f * M_PI / 180.f) // pitch机械角度电控限位最大值

#define GIMBAL_MIN_MEC_ANGEL       (-20.f * M_PI / 180.f) // pitch机械角度电控限位最小值

#define GIMBAL_MAX_GYRO_ANGEL      (gimbal->base_info.pitch_imu_angle + (GIMBAL_MAX_MEC_ANGEL - gimbal->base_info.pitch_motor_angle) / M_PI * 360.f)

#define GIMBAL_MIN_GYRO_ANGEL      (gimbal->base_info.pitch_imu_angle - (gimbal->base_info.pitch_motor_angle - GIMBAL_MIN_MEC_ANGEL) / M_PI * 360.f)

/*云台模式*/

typedef enum {

    GIMB_SLEEP, // 卸力
    G_INIT,     // 归中初始化模式，等待云台复位完成
    G_GYRO,     // 底盘跟头
    G_MEC,      // 头跟底盘
    G_MEC_MOVE, // 用机械角控云台而非跟随底盘，测散步用

} gimbal_mode_e;

/*视觉偏置*/

typedef struct __attribute__((packed)) {

    float vision_yaw_offset;

} gimbal_offset_info_t;

/*云台初始化信息*/

typedef struct {

    uint16_t init_time; // 初始化时间

    uint16_t init_time_max; // 初始化yaw、pitch归零点超时时间

    float pitchInitAngleTolerance; // pitch初始化机械角度容忍度

    float yawInitAngleTolerance; // yaw初始化机械角度容忍度

    float pitchInitSpeedTolerance; // pitch初始化机械角速度容忍度

    float yawInitSpeedTolerance; // yaw初始化机械角速度容忍度

} gimbal_init_info_t;

typedef struct
{
    float yaw_imu_angle;        // 云台陀螺仪yaw轴角度
    float yaw_imu_angle_target; // 陀螺仪模式目标yaw   世界坐标系 (-180°~180°)
    float yaw_imu_speed;
    float yaw_motor_angle;      // yaw轴 相对底盘
    float yaw_mec_angle_target; // 机械模式目标yaw		底盘坐标系(-180°~180°)
    float yaw_motor_speed;      // yaw轴 相对底盘  速度(dps)
    float yaw_mec_360_angle;

    float pitch_imu_angle; // 云台陀螺仪pitch轴角度
    float pitch_imu_angle_target;
    float pitch_imu_speed; // 云台陀螺仪pitch轴速度
    float pitch_motor_angle;
    float pitch_mec_360_angle;
    float pitch_mec_angle_target;

    float output_gimbal_y; // yaw轴电机输出
    float output_gimbal_p; // pitch轴电机输出

} gimbal_base_info_t;

typedef struct gimbal_all {

    Motor_DM_t *gimbal_y;
    gimbal_pid_info_t *pid_info;
    gimbal_base_info_t base_info;

    gimbal_mode_e mode;
    gimbal_mode_e gimbal_last_mode; // 上一周期云台模式，用于检测模式跳变

    Dev_Reset_State_e gimbal_reset_state; // 云台初始化状态

    gimbal_offset_info_t *offset_info; // 偏置信息

    gimbal_init_info_t initInfo; // 云台初始化复位信息

    float (*all_pid_calc)(pid_ctrl_t *out, pid_ctrl_t *inn, float target, float mea_out, float mea_in, float inner_kp, uint8_t err_cal_mode);

    void (*work)(struct gimbal_all *gimbal);

} gimbal_t;

/*云台180度旋转状态*/
typedef struct {
    bool is_rotating;      // 是否正在旋转
    float target_angle;    // 旋转目标角度(相对值，180或-180)
    float angle_tolerance; // 角度容差，默认3.0度
    float speed_tolerance; // 速度容差，默认0.5rad/s
} gimbal_180_state_t;

extern gimbal_t gimbal;

void Gimbal_Work(gimbal_t *gimbal);

#endif
