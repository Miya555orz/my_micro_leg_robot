#ifndef __KT_motor_H
#define __KT_motor_H

#include "stm32h7xx.h"
#include "driver.h"
#include "motor_def.h"
#include "PID.h"
#include "algo.h"
// 填充力矩变量：kt_motor[0].W_iqControl(&kt_motor[0],kt_motor[0].base_info.motor_out);
// CAN发送：kt_motor[0].tx_W_cmd(&kt_motor[0],TORQUE_CLOSE_LOOP_ID);
typedef enum motor_kt9025_command_e {
    PID_RX_ID            = 0x30,
    PID_TX_RAM_ID        = 0x31, // 断电失效
    PID_TX_ROM_ID        = 0x32, // 断电有效
    ACCEL_RX_ID          = 0x33,
    ACCEL_TX_ID          = 0x34,
    ENCODER_RX_ID        = 0x90,
    ZERO_ENCODER_TX_ID   = 0x91,
    ZERO_POSNOW_TX_ID    = 0x19,
    MOTOR_ANGLE_ID       = 0x92,
    CIRCLE_ANGLE_ID      = 0x94,
    STATE1_ID            = 0x9A,
    CLEAR_ERROR_State_ID = 0x9B,
    STATE2_ID            = 0x9C,
    STATE3_ID            = 0x9D,
    MOTOR_CLOSE_ID       = 0x80,
    MOTOR_STOP_ID        = 0x81,
    MOTOR_RUN_ID         = 0x88,
    TORQUE_OPEN_LOOP_ID  = 0xA0,
    TORQUE_CLOSE_LOOP_ID = 0xA1,
    SPEED_CLOSE_LOOP_ID  = 0XA2,
    POSI_CLOSE_LOOP_ID1  = 0XA3,
    POSI_CLOSE_LOOP_ID2  = 0XA4,
    POSI_CLOSE_LOOP_ID3  = 0XA5,
    POSI_CLOSE_LOOP_ID4  = 0XA6,
    POSI_CLOSE_LOOP_ID5  = 0XA7,
    POSI_CLOSE_LOOP_ID6  = 0XA8,

} motor_kt9025_command_e;
typedef struct KT_motor_rx_info_t {
    int32_t accel;          // 加速度 1dps/s
    uint16_t encoder;       // 编码器位置             （0~16383 * 4）
    uint32_t encoder_sum;   // 角度和
    uint32_t last_encoder;  // 上一循环编码器位置，用于计算多圈角度
    uint16_t encoderRaw;    // 编码器原始位置         （0~16383 * 4）
    uint16_t encoderOffset; // 编码器零偏             （0~16383 * 4）
    int8_t temperature;     // 温度    1°C/LSB
    uint16_t voltage;       // 电压    1V
    uint8_t errorState;     // 高四位无效    低四位0xx0都正常；0xx1温度正常+低压保护；1xx0过温保护+电压正常；1001过温保护+低压保护
    int16_t current;        // 转矩电流    返回的值无单位说明   -2048 ~ 2048  （-33~33A）
    int16_t speed;          // 电机转速    1dps/LSB
    int16_t current_A;      // ABC三相电流数据     1A/64LSB
    int16_t current_B;
    int16_t current_C;
    int16_t powerControl; // 输出功率（-1000~1000）  无单位说明
    int64_t motorAngle;   // 电机多圈角度   0.01°/LSB  （-2^63~2^63）    顺时针增加，逆时针减少
    uint32_t circleAngle; // 电机单圈角度   0.01°/LSB  （ 0~36000 * 减速比-1）
                          // 以编码零点作为起始点，顺时针增加，再次到达零点时数值变为0
    int16_t angle_add;    //-4096~4096
} KT_motor_rx_info_t;

typedef struct KT_motor_tx_info_t {
    int32_t accel;                              // 加速度     1dps/s
    uint16_t encoderOffset;                     // 编码器零偏（0~16383*4）
    int16_t powerControl;                       // 输出功率 （-1000~1000）     不受上位机的Max Power限制
    int16_t iqControl;                          // 扭矩电流 （-2000~2000，对应的实际扭矩电流-32A~32A） 不受上位机限制
    int32_t speedControl;                       // 实际转速为0.01dps/LSB       最大值受上位机设置
    int32_t angle_sum_Control;                  // 多圈角度，  0.01degree/LSB     36000代表360°      最大值受上位机设置
    uint16_t angle_sum_Control_maxSpeed;        // 多圈角度的最大速度   1dps/LSB   最大值受上位机设置
    uint16_t angle_single_Control;              // 单圈角度    0.01degree/LSB       0~35999对应实际角度0~359.99°   最大值受上位机设置
    uint16_t angle_single_Control_maxSpeed;     // 单圈角度的最大速度   1dps/LSB   最大值受上位机设置
    uint8_t angle_single_Control_spinDirection; // 单圈角度旋转方向，0x00顺时针，0x01逆时针
    int32_t angle_add_Control;                  // 角度位置增量，转动方向由控制量符号决定   0.01degree/LSB     最大值受上位机设置
    uint16_t angle_add_Control_maxSpeed;        // 角度位置增量的最大速度  1dps/LSB   最大值受上位机设置

} KT_motor_tx_info_t;

typedef struct KT_motor_id_info_t {
    uint32_t tx_id; // 发送id
    uint32_t rx_id; // 接收id

    motor_drive_e drive_type;
    motor_type_e motor_type;

} KT_motor_id_info_t;

typedef struct KT_motor_state_info_t {
    motor_init_e init_flag;

    uint8_t offline_cnt_max;
    uint8_t offline_cnt;
    uint8_t selfprotect_cnt_max;
    uint8_t selfprotect_cnt;
    motor_state_e work_state;

    motor_protect_e selfprotect_flag;
} KT_motor_state_info_t;
typedef struct KT_motor_pid_tx_info_t {
    uint8_t angleKp;
    uint8_t angleKi;
    uint8_t speedKp;
    uint8_t speedKi;
    uint8_t iqKp; // 转矩环
    uint8_t iqKi;

} KT_motor_pid_tx_info_t;
typedef struct KT_motor_pid_rx_info_t {
    uint8_t angleKp;
    uint8_t angleKi;
    uint8_t speedKp;
    uint8_t speedKi;
    uint8_t iqKp; // 转矩环
    uint8_t iqKi;

} KT_motor_pid_rx_info_t;

typedef struct KT_motor_pid_t {
    motor_init_e init_flag;
    KT_motor_pid_rx_info_t rx;
    KT_motor_pid_tx_info_t tx;

} KT_motor_pid_t;

typedef struct KT_motor_info_t {
    KT_motor_pid_t pid_info;
    KT_motor_rx_info_t rx_info;
    KT_motor_tx_info_t tx_info;
    KT_motor_id_info_t id;
    KT_motor_state_info_t state_info;

} KT_motor_info_t;
/**
 *	@brief	电机PID
 */

typedef struct
{
    int16_t motor_out;
} kt_motor_base_info_t;

typedef struct KT_motor_class_t {
    KT_motor_info_t KT_motor_info;
    uint8_t tx_buff[8];
    kt_motor_base_info_t base_info;
    void (*init)(struct KT_motor_class_t *motor);
    void (*heartbeat)(struct KT_motor_class_t *motor);

    // W表示写，R表示读，cmd是命令

    void (*get_info)(struct KT_motor_class_t *motor, uint8_t *rxBuf);
    void (*tx_W_cmd)(struct KT_motor_class_t *motor, uint8_t command); // 发送写命令
    void (*tx_R_cmd)(struct KT_motor_class_t *motor, uint8_t command); // 发送主动读取信息命令

    // 下面是基本功能函数，负责写电机的发送结构体中的参数，内部有逻辑限制范围操作

    void (*W_pid)(struct KT_motor_class_t *motor, uint8_t *buff);
    void (*W_accel)(struct KT_motor_class_t *motor, int32_t accel);
    void (*W_encoderOffset)(struct KT_motor_class_t *motor, uint16_t encoderOffset);
    void (*W_powerControl)(struct KT_motor_class_t *motor, int16_t powerControl); // 开环功率
    void (*W_iqControl)(struct KT_motor_class_t *motor, int16_t iqControl);       // 闭环扭矩/电流
    void (*W_speedControl)(struct KT_motor_class_t *motor, int32_t speedControl); // 闭环速度
    void (*W_angle_sum_Control)(struct KT_motor_class_t *motor,
                                int32_t angle_sum_Control,
                                uint16_t angle_sum_Control_maxSpeed); // 闭环多圈角度
    void (*W_angle_single_Control)(struct KT_motor_class_t *motor,
                                   uint16_t angle_single_Control,
                                   uint8_t angle_single_Control_spinDirection,
                                   uint16_t angle_single_Control_maxSpeed); // 闭环单圈角度
    void (*W_angle_add_Control)(struct KT_motor_class_t *motor,
                                int32_t angle_add_Control,
                                uint16_t angle_add_Control_maxSpeed); // 闭环角度增量

} KT_motor_t;

void KT_motor_class_init(KT_motor_t *motor);
void kt_motor_multi_control(int16_t *iqControl, char kt_motor_num, motor_drive_e drive_type);

#endif
