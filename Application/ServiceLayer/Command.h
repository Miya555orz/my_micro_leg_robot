#ifndef __COMMAND_H
#define __COMMAND_H


#include "stdbool.h"
#include "stdint.h"
#include "stm32h7xx_hal.h"
#define OUT_TIME_OFF  (0xFFFF)//关闭超时退出

/**
 * @brief 命令类型枚举
 */
typedef enum
{
  HIGH_TRIGER_C, //高电平触发
  LOW_TRIGER_C,  //低电平触发
  RISE_TRIGER_C, //上升沿触发
  FALL_TRIGER_C, //下降沿触发
  NO_CMD,		 //不触发
}Cmd_Type_e;

/**
 * @brief 用户命令状态枚举
 */
typedef enum 
{
  KEEP_U,         // 保持
  SWITCH_HIGHT_U, // 触发(上升沿)
  SWITCH_LOW_U,   // 松开(下降沿)
}User_Status_e;

/**
 * @brief 命令状态枚举
 */
typedef enum 
{
	FINISH_C, //执行完成
	RUNING_C, //执行中
}Cmd_Status_e;

/**
 * @brief 命令初始化状态枚举
 * 
 */
typedef enum
{
  DEINIT_C, //未初始化
  INIT_C,   //初始化
}Cmd_Init_e;
/**
 * @brief 命令时间锁
 */
typedef struct 
{
  uint8_t Trigger_lock_on;
  uint32_t lock_time;

}Trigger_lock_t;

/**
 * @brief 命令结构体
 */
typedef struct command_class_t
{
  bool cmd_value;//命令值（符合触发条件：True）
  Cmd_Status_e cmd_status;//命令状态

  Cmd_Type_e cmd_type;//命令类型
  
  bool user_value;//当前用户命令
  bool user_value_last;//上一次用户命令
  User_Status_e user_status;//用户命令状态

  uint16_t run_time;//命令执行时间
  uint16_t run_time_max;//命令执行时间上限

  Cmd_Init_e init_flag;//命令初始化状态
  Trigger_lock_t   Trigger_lock;
  void (*init)(struct command_class_t *command);//命令初始化
  void (*heartbeat)(struct command_class_t *command);//命令心跳
  void (*update)(struct command_class_t *command,bool condition);//获取用户命令
  void (*clean)(struct command_class_t *command);//命令清除
  void (*s_run)(struct command_class_t *command);//命令切换到正在执行
  void (*s_finish)(struct command_class_t *command);//命令切换到执行完成
}command_t;


void Cmd_Class_Init(command_t *commond);
#endif
