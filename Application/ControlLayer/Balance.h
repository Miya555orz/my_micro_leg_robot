#ifndef __BALANCE_H
#define __BALANCE_H

#include "rc_sensor.h"
#include "chassis.h"

#include "Command_Instance.h"

typedef enum {
    RC_CTRL = 0,
    KEY_CTRL,
} Balance_Ctrl_e;
typedef enum {
    Balance_reset_NO,
    Balance_reset_OK,

} Balance_reset_state_e;

typedef enum {

    Sleep_Mode = 0,
    Rescue_Mode,
    Manual_Rescue_Mode,
    Init_Mode,
    Imu_Mode,
    Mec_Mode,
    Cycle_Mode,

    SitDown_Mode,
    LEG_TEST_Mode,
} Balance_Mode_e;

// 把一些标志位放这里比较方便阅读，如果用command枚举的话是看不到标志位名字的
// 通常由别的地方执行完后清标志位，sleep 模式下会清标志位
typedef struct Balance_Flag_struct_t {
    bool Chassis_Online_Flag;
    bool KNEE_STRIKE_Flag;            // 磕膝上台阶标志位
    bool JUMP_THEN_KNEE_STRIKE_Flag;  // 跳跃后磕膝上台阶标志位
    bool Chassis_Alignment_Flag;      // 底盘归正标志位，由Chassis_Yaw_Target_Process_All负责维护
    bool KNEE_STRIKE_after_Alignment; // 想磕膝上台阶，但是底盘未归正，在检测到底盘归正后再设置KNEE_STRIKE_Flag标志位
    bool Rescue_Flag;                 // 自救进程标志位
    bool NoCheckRescueFlag;           // 不检测自救标志位，如磕膝上台阶、自救后一小段时间内等特殊情况
    uint32_t NoCheckRescue_StartTick; // 自救完成后屏蔽计时起始时间戳
    bool Leg_length_ctrl_Flag;        // 腿长控制标志位
    bool Jumping_Flag;                // 跳跃过程中，用于给chassis状态信号量
    bool Middle_Flag;                 // 切换中腿长模式标志位
    bool Pre_Charge_Flag;             // 预充电模式，不给超电充电
    bool JUMP_AND_MID_Flag;           // 跳+中腿长下台阶模式标志位
    bool RTS_Flag;                    // 收腿下二级台阶模式标志位
    bool GIMBAL_180_Flag;             // 云台180度转向命令标志位
    bool Wheel_Down_5_Flag;           // 波轮短时间内向下5次
} Balance_Flag_t;

typedef struct Balance_Remote_Ctrl_struct_t {
    rc_sensor_t *sensor;
    uint8_t *last_thumbwheel_step;
    uint32_t rc_online_tick; // 开控后从0开始计时用于屏蔽某些由开控造成的误操作
} Balance_Remote_Ctrl;

typedef struct Launch_Command_struct_t {

    bool Enable_Shoot_Flag; // 是否允许发射标志位，决定是否sleep
    bool Shoot_Ctrl_Flag;   // 开火扳机标志位，单发和连发都需要这个标志位来控制开火电平
    uint8_t Shoot_Mode;     // 0单发，1连发

} Shoot_Flag_t;

typedef struct Vision_Command_struct_t {

    bool Vision_Online_Flag;
    bool Auto_Catch_Flag;         // 操作手开启自瞄标志位
    bool Auto_Base_Flag;          // 基地自瞄
    bool Auto_Catch_Engi_Flag;    // 能量机关自瞄
    bool Auto_Catch_Outpost_Flag; // 前哨自瞄
} Vision_Flag_t;

typedef struct Balance_struct_t {
    Balance_Ctrl_e ctrl; // 遥控还是键盘控
    Balance_Mode_e mode;
    Balance_reset_state_e reset_state; // 由云台、底盘复位状态来判断是否复位完成
    Balance_Mode_e last_mode;

    Balance_Remote_Ctrl *rc;

    Balance_Flag_t *Flag;

    command_t *command;

    Shoot_Flag_t Shoot_Flag_struct;

    Vision_Flag_t Vision;

    void (*init)(struct Balance_struct_t *balance);

    void (*update)(struct Balance_struct_t *balance);
} Balance_t;

extern Balance_t Balance;
void check_z_key_5times(void);

#endif
