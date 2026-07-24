#ifndef __COMMAND_Instance_H
#define __COMMAND_Instance_H

#include "command.h"
#include "Balance.h"

enum {
    JUMP,
    KNEE_STRIKE,
    JUMP_THEN_KNEE_STRIKE,
    SW_MID_LENGTH,      // 切换中腿长
    JUMP_AND_MID,       // 跳+中腿长
    RTS,                 // 收腿下二级台阶
    Energy_Engine_Mode, // 只自瞄能量机关模式
    Outpost_Mode,       // 只自瞄前哨模式
    Reset_to_Normal,    // 底盘归正、取消特殊模式
    PRE_CHARGE_MODE,    // 预充电模式
    GIMBAL_180,         // 云台180度转向命令
    COMMAND_LIST,
};

void Cmd_Init(void);
void Cmd_Heartbeat(void);
void Command_Update(void);
extern command_t command[COMMAND_LIST];
#endif
