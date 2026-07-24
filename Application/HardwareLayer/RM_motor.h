/**
 ******************************************************************************
 * @file    RM_motor.h
 * @brief   RM电机驱动
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */
#ifndef __RM_MOTOR_H
#define __RM_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "pid.h"
#include "drv_can.h"

/* Exported typedef ----------------------------------------------------------*/
#define _3508_VALUE_TO_CURRENT_COEFFICIENT ( 20.f/16384.f)               // 3508电流数值转化为电流的系数，手册-20~20A对应-16384~16384
#define _3508_GUANGONG_TORQUE_CONSTANT     (0.246f)                       // 3508管工减速箱的扭矩常数，N*m/A
#define _3508_SELFMADE_TORQUE_CONSTANT     (0.2703f)     //电流到转矩系数                 // 3508自制减速箱的扭矩常数，N*m/A
#define _3508_REDUCT_TORQUE_COEFFICIENT    _3508_SELFMADE_TORQUE_CONSTANT // 带减速箱情况下的转矩电流系数
#define _2006_TORQUE_CONSTANT              (0.18f)                        // 2006的扭矩常数，N*m/A
#define _3508_MAX_CURRENT                  (20.f)                         // 3508输出最大电流，手册-20~20A
#define _2006_MAX_CURRENT                  (10.f)                         // 2006输出最大电流，手册-10~10A

#define _6020_TORQUE_CONSTANT              (1.f)     // 6020的转速常数，rpm/V
#define _6020_MAX_CURRENT                  (25000.f) // 3508输出最大电流，手册-20~20A

#define _3508_GUANGONG_REDUCT_RATIO        (268.f / 17.f) // 3508莞工减速箱减速比
#define _3508_SELFMADE_REDUCT_RATIO        (17.255)       // 自制减速箱减速比，如果修改方向在order_correction里改
#define _3508_REDUCT_RATIO                 _3508_SELFMADE_REDUCT_RATIO
#define _2006_REDUCT_RATIO                 (36.f / 1.f)
/*电机模式*/
typedef enum Motor_RM_Type {
    _3508_Single,    // 3508不加减速箱
    _3508_Reduction, // 3508加减速箱
    _6020_Single,    // 单6020电机
    _2006_Single,    // 单2006电机
} Motor_RM_Type_e;

typedef struct Motor_RM_Born_Info_struct_t {
    int8_t order_correction;

    uint8_t rxId; // 对应一拖四的序号0~3

    uint32_t stdId;

    Motor_RM_Type_e type; // 电机类型

#ifdef __STM32F4xx_HAL_H
    CAN_HandleTypeDef *hcan; // can口选择
#endif

#ifdef STM32H7xx_HAL_H
    FDCAN_HandleTypeDef *hcan; // can口选择
#endif
} Motor_RM_Born_Info_t;

typedef struct Motor_RM_Rx_Info_struct_t {
    float torque;

    float torque_current;

    int16_t torque_current_raw;

    // 针对encoder_speed
    // 如果带原厂减速箱，则从上往下看输出轴时，逆时针转为正，转子与输出轴同向转动；
    // 如果是莞工减速箱，从上往下看莞工减速箱白色壳子，顺时针转为正，转子与输出轴反向转动
    int16_t encoder_speed; // rpm(r/min)

    float speed; // rad/s,

    uint16_t encoder; // 0~8191

    int32_t encoder_sum;

    uint16_t encoder_last;

    float motor_angle_sum;

    float motor_angle; // 电机弧度制绝对角度，0~2PI

    float motor_angle_last;

    int8_t temperature;
} Motor_RM_Rx_Info_t;

typedef struct Motor_RM_Ctrl_Info_struct_t {
    bool Speed_Input_Flag;        // 使用外部传感器的速度标志位：0不使用，1使用
    pid_ctrl_t *angle_ctrl_inner; // 角度环内环

    bool Angle_Input_Flag;        // 使用外部传感器的角度标志位：0不使用，1使用
    bool Nearest_Return;          // 半圈处理标志位：0不使用，1使用
    pid_ctrl_t *angle_ctrl_outer; // 角度环外环

    pid_ctrl_t *speed_ctrl; // 速度环
} Motor_RM_Ctrl_Info_t;

typedef struct Motor_RM_Tx_Info_struct_t {
    float torque; // 需要发送的转矩

    float torque_current;

    int16_t torque_current_raw;

    uint8_t tx_buff[8];

} Motor_RM_Tx_Info_t;

typedef struct Motor_RM_State_struct_t {
    uint32_t offline_cnt;

    uint32_t offline_cnt_max;

    dev_work_state_t status;

} Motor_RM_State_t;

typedef struct Motor_RM_struct_t {
    Motor_RM_Born_Info_t *born_info;

    Motor_RM_Rx_Info_t *rx_info;

    Motor_RM_Tx_Info_t *tx_info;

    Motor_RM_State_t *state;

    Motor_RM_Ctrl_Info_t *ctrl;

    void (*single_set_torque)(struct Motor_RM_struct_t *motor);

    void (*single_set_speed)(struct Motor_RM_struct_t *motor);

    void (*single_set_angle)(struct Motor_RM_struct_t *motor);

    void (*rx)(struct Motor_RM_struct_t *motor, uint8_t *rxBuf);

    void (*single_sleep)(struct Motor_RM_struct_t *motor);

    void (*single_ctrl)(struct Motor_RM_struct_t *group);

    void (*single_init)(struct Motor_RM_struct_t *motor);

    void (*single_heart_beat)(struct Motor_RM_struct_t *motor);
} Motor_RM_t;

typedef struct Motor_RM_Group_struct_t {
    Motor_RM_t *motor[4];

    uint8_t tx_buff[8];

    uint32_t stdId;

    CAN_HandleTypeDef *hcan;

    void (*group_set_torque)(struct Motor_RM_Group_struct_t *group);

    void (*group_ctrl)(struct Motor_RM_Group_struct_t *group);

    void (*group_sleep)(struct Motor_RM_Group_struct_t *group);

    void (*group_init)(struct Motor_RM_Group_struct_t *group);

    void (*group_heartbeat)(struct Motor_RM_Group_struct_t *group);

} Motor_RM_Group_t;

/* Exported functions --------------------------------------------------------*/
void RM_Motor_Init(Motor_RM_t *motor);
void RM_Group_Motor_Init(Motor_RM_Group_t *group);

#endif
