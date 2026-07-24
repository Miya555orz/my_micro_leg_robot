/**
 * @file command.c
 * @author your name (you@domain.com)
 * @brief 命令包
 * @version 0.1
 * @date 2023-11-25
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "command.h"

/* Private function prototypes -----------------------------------------------*/

void User_Status_Update(command_t *command);
void Cmd_Switch_Run(command_t *command);
void Cmd_Switch_Finish(command_t *command);
void Cmd_Value_Update(command_t *command);
void Cmd_HeartBeat(command_t *command);
void Cmd_Class_Update(command_t *command, bool condition);
void Cmd_Clean(command_t *command);

/* Function  body --------------------------------------------------------*/

/**
 * @brief 命令初始化
 *
 * @param command
 */
void Cmd_Class_Init(command_t *command)
{
  command->user_value = false;
  command->user_value_last = false;
  command->user_status = KEEP_U;

  command->cmd_value = false;
  command->cmd_status = FINISH_C;

  command->run_time = 0;

  command->update = Cmd_Class_Update;
  command->s_run = Cmd_Switch_Run;
  command->s_finish = Cmd_Switch_Finish;
  command->clean = Cmd_Clean;
  command->heartbeat = Cmd_HeartBeat;

  command->init_flag = INIT_C; // 命令初始化完成
}
/**
 * @brief 用户命令状态更新
 *
 * @param command
 */
void User_Status_Update(command_t *command)
{
  /* 用户命令状态更新 */
  static uint32_t last_lock_tick;

  if (command->user_value == true && command->user_value_last == false)
  {
    // 防止双发命令
    if (command->Trigger_lock.Trigger_lock_on == 1)
    {
      if (HAL_GetTick() - last_lock_tick > command->Trigger_lock.lock_time)
      {

        command->user_status = SWITCH_HIGHT_U;
        // last_lock_tick=HAL_GetTick();//连点初次触发
      }
    }

    else
    {
      command->user_status = SWITCH_HIGHT_U;
    }
    last_lock_tick = HAL_GetTick(); // 连点末尾触发
  }
  else if (command->user_value == false && command->user_value_last == true)
  {
    command->user_status = SWITCH_LOW_U;
  }
  else
  {
    command->user_status = KEEP_U;
  }
  /* 当前命令值为前一次命令值 */
  command->user_value_last = command->user_value;
}

/**
 * @brief 命令执行时间更新
 *
 * @param command
 */
void Cmd_HeartBeat(command_t *command)
{
  /* 命令执行时间更新 */
  if (command->cmd_status == RUNING_C) // 命令正在执行
  {
    if (command->run_time_max != OUT_TIME_OFF) // 没有关闭超时退出
    {
      command->run_time++;
    }
  }
  else // 命令不在执行
  {
    command->run_time = 0;
  }

  /*超时退出*/
  if (command->run_time > command->run_time_max)
  {
    Cmd_Clean(command);
  }
  /*命令值更新*/
   Cmd_Value_Update(command);
}

/**
 * @brief 命令值更新
 *
 * @param command
 */
void Cmd_Value_Update(command_t *command)
{
  /* 命令值更新 */
  switch (command->cmd_type)
  {
  case RISE_TRIGER_C:
    if (command->user_status == SWITCH_HIGHT_U)
    {
      command->cmd_value = true;
    }
    else
    {
      command->cmd_value = false;
    }
    break;

  case FALL_TRIGER_C:
    if (command->user_status == SWITCH_LOW_U)
    {
      command->cmd_value = true;
    }
    else
    {
      command->cmd_value = false;
    }
    break;

  case HIGH_TRIGER_C:
    if (command->user_value == true)
    {
      command->cmd_value = true;
    }
    else
    {
      command->cmd_value = false;
    }
    break;

  case LOW_TRIGER_C:
    if (command->user_value == false)
    {
      command->cmd_value = true;
    }
    else
    {
      command->cmd_value = false;
    }
    break;

  case NO_CMD:
    command->cmd_value = false;
    break;

  default:
    break;
  }
}

/**
 * @brief 命令类更新 命令有初始化才会更新
 *
 * @param command
 * @param condition 高电平条件
 */
void Cmd_Class_Update(command_t *command, bool condition)
{
  if (command->init_flag == INIT_C)
  {
    if (condition == true)
    {
      command->user_value = true;
    }
    else
    {
      command->user_value = false;
    }
  }

  /*用户命令状态更新*/
  User_Status_Update(command);
  /*命令值更新*/
 // Cmd_Value_Update(command);
}

/**
 * @brief 命令标志位清零
 *
 * @param command
 */
void Cmd_Clean(command_t *command)
{
  command->user_value = false;
  command->user_value_last = false;
  command->user_status = KEEP_U;

  command->cmd_value = false;
  command->cmd_status = FINISH_C;

  command->run_time = 0;
}

/**
 * @brief 命令切换为正在执行
 *
 * @param command
 */
void Cmd_Switch_Run(command_t *command)
{
  command->cmd_status = RUNING_C;
}

/**
 * @brief 命令切换为执行完成
 *
 * @param command
 */
void Cmd_Switch_Finish(command_t *command)
{
  command->cmd_status = FINISH_C;
}
