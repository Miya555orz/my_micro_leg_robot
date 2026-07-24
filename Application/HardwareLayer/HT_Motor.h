#ifndef __HT_MOTOR_H
#define __HT_MOTOR_H

#include "stm32h7xx_hal.h"
#include "rp_config.h"
#include "drv_can.h"
#include "drv_tick.h"
#include "rp_math.h"
#include "arm_math.h"


#define HT_P_MIN -95.5f    // Radians
#define HT_P_MAX 95.5f        
#define HT_V_MIN -45.0f    // Rad/s
#define HT_V_MAX 45.0f
#define HT_KP_MIN 0.0f     // N-m/rad
#define HT_KP_MAX 500.0f
#define HT_KD_MIN 0.0f     // N-m/rad/s
#define HT_KD_MAX 5.0f
#define HT_T_MIN -18.0f    // N.m
#define HT_T_MAX 18.0f
#define HT_C_MIN -40.0f    // A
#define HT_C_MAX 40.0f
#define TORQUE_CONSTANT 0.45f //转矩常数N.m/A

/*电机指令集*/
typedef enum Motor_MIT_Command_enum_e
{
	Enter_Motor_Mode,//使能电机控制(指示灯变绿)
	Exit_Motor_Mode,//失能电机控制(指示灯变红)
	Zero_Position_Sensor,//设定当前编码角度为零
	Clear_Err,
}Motor_MIT_Command_e;

/*电机模式*/
typedef enum Motor_HT_Work_mode_enum_e
{
	Motor_Control,//电机可控
	Motor_UnControl,//电机不可控
}Motor_HT_Work_mode_e;

/*电机初始化参数*/
typedef struct Motor_HT_Born_Info_struct_t
{	
    uint32_t stdId;//电机控制报文ID

#ifdef __STM32F4xx_HAL_H
    CAN_HandleTypeDef *hcan;//can口选择
#endif
	
#ifdef STM32H7xx_HAL_H
    FDCAN_HandleTypeDef *hcan;//can口选择
#endif
	  int8_t order_correction;//正方向规定

}Motor_HT_Born_Info_t;

/*接收电机报文信息结构体*/
typedef struct Motor_HT_Rx_Info_struct_t
{
	float encoder;//上电后的角度累加(单位rad),-95,5~95.5
	
	float encoder_last;
	
	float encoder_err;
	
	float speed;//电机速度(单位rad/s)
	
	float torque;//电机转矩(单位N.m)
	
	float torque_current;//电机电流(单位A)
	
	float motor_angle_sum;
	
	float motor_angle_sum_vi;
	
	float motor_angle_sum_filter;

  float motor_angle;//电机弧度制绝对角度，0~2PI
	
	float motor_angle_last;
	
	/*更新时间计算*/
	uint32_t time_now;
	
	uint32_t time_last;
	
	float time;
}Motor_HT_Rx_Info_t;

/*发送电机报文信息结构体*/
typedef struct Motor_HT_Tx_Info_struct_t
{
	float torque;//需要发送的转矩(单位N.m)
	
	float target_speed;//目标速度(单位rad/s)
	
	float target_angle;//目标角度(单位rad)
	
	float Kp;//位置增益
	
	float Kd;//速度增益
	
	uint8_t single_tx_buff[8];//使用电机发送时的个人数组
}Motor_HT_Tx_Info_t;
/* 驱动参考力矩 = (torque + Kp*err_angle + Kd*err_speed) */

/*电机状态结构体*/
typedef struct Motor_HT_State_struct_t
{
    uint32_t offline_cnt;

    uint32_t offline_cnt_max;

    dev_work_state_t status;

		Motor_HT_Work_mode_e mode;
}Motor_HT_State_t;

/*单电机总结构体*/
typedef struct Motor_HT_struct_t
{
	Motor_HT_Born_Info_t* born_info;
	
	Motor_HT_Rx_Info_t* rx_info;
	
	Motor_HT_Tx_Info_t* tx_info;
	
	Motor_HT_State_t* state;
	
	void (*single_init)(struct Motor_HT_struct_t *motor);
	
	void (*single_sleep)(struct Motor_HT_struct_t *motor);
	
	void (*zero_position)(struct Motor_HT_struct_t *motor);
	
	void (*single_set_torque)(struct Motor_HT_struct_t *motor);
	
	void (*single_set_speed)(struct Motor_HT_struct_t *motor);
	
	void (*single_set_angle)(struct Motor_HT_struct_t *motor);
	
	void (*rx)(struct Motor_HT_struct_t *motor, uint8_t *rxBuf);
	
	void (*single_heart_beat)(struct Motor_HT_struct_t *motor);
}Motor_HT_t;

/*多电机结构体，由于海泰只有单电机控制，该结构体只是把单电机的一些通用的功能整合起来，方便控制*/
typedef struct Motor_HT_Group_struct_t
{
		Motor_HT_t* motor[4];
	
	  void (*group_set_torque)(struct Motor_HT_Group_struct_t *group);
	
		void (*group_sleep)(struct Motor_HT_Group_struct_t *group);
	
	  void (*group_init)(struct Motor_HT_Group_struct_t *group);
	
	  void (*group_heartbeat)(struct Motor_HT_Group_struct_t *group);
	
}Motor_HT_Group_t;

void HT_Single_Motor_Init(Motor_HT_t *motor);
void HT_Group_Motor_Init(Motor_HT_Group_t *group);
extern uint8_t flag_rx;

#endif
