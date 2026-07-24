#ifndef __RP_LOG_TASK_H
#define __RP_LOG_TASK_H

#include "cmsis_os.h"
#include "main.h"
#include "RP_Log.h"
#include "chassis.h"

void StartRP_LogTask(void const *argument);
// 打印左右腿氮气弹簧补偿力
void My_Log_print_leg_spring_compensation(void);
// 打印左右腿重力力矩补偿
void My_Log_print_leg_gravity_torque(void);
// 打印机体姿态角度(pitch/roll/yaw)及角速度
void My_Log_print_posture_degree(void);
// 打印机体世界坐标x/y/z
void My_Log_print_posture_world(void);
// 打印左右腿phi1/phi4角度及电机角度
void My_Log_print_phi1_phi4(void);
// 打印左右腿thetal/thetab/s/sd1等状态量
void My_Log_print_leg_state_info(void);
// 打印左右腿各状态量的误差（目标-测量）
void My_Log_print_leg_state_err(void);
// 打印左右腿支持力
void My_Log_print_leg_F_support(void);
// 打印指定腿的sd1及目标sd1
void My_Log_print_leg_sd1(Leg_e leg);
// 打印底盘功率、超电电压电流
void My_Log_print_power_cap(void);
// 打印自救状态机当前状态及左右腿Tp输出
void My_Log_print_rescue_state(void);
// Monitor chassis_power_buffer, control buzzer when buffer <= 40
// Beeps for 2s then stops. If buffer stays <= 40, continues beeping with 200ms gaps.
void PowerWarning_Buzzer_Check(void);
// Monitor super capacitor voltage, control buzzer when cap_v < 19V
// Beeps for 5s then stops. If voltage still low, continues beeping with 200ms gaps.
void CapLowVoltage_Buzzer_Check(void);
// 检测自瞄相关标志位跳变并触发短鸣提示
void My_Log_Check_Vision_Flag_Edge_Beep(void);

#endif