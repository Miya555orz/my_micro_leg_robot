#include "control_task.h"
#include "RP_Log_task.h"
#include "cap.h"
#include "chassis_motor.h"
#include "ui.h"
#include "ui_priority.h"
static void Motor_Unload_Force(void);
static void All_CAN_Send_Here(void);

void StartCtrlTask(void const *argument)
{
    for (;;)
    {

#ifndef TEST_MY_LEG
        Chassis.status_react(&Chassis);
#else
        Chassis.mode = C_Test;
#endif
        My_Judge_Realtime_Task(&My_Judge);

        Chassis.ctrl(&Chassis);
        gimbal.work(&gimbal);
        shoot.work(&shoot);
        if (RC_ONLINE)
        {
            // Motor_Unload_Force();
            All_CAN_Send_Here();
        }
        else
        {
            Motor_Unload_Force();
        }

        Ui_Info_Update();
        Ui_Send();
        /*---------------- 日志 begin-------------------- */
        if (HAL_GetTick() % 20 == 0)
        {
        }
        /*---------------- 日志 end-------------------- */
        osDelay(1);
    }
}

static void All_CAN_Send_Here(void)
{
#if !defined(TEST_MY_LEG) && !defined(TEST_RESCUE)
    Yaw_Motor.single_set_torque(&Yaw_Motor);
    if (Balance.Flag->Chassis_Online_Flag == false)
    {
        Chassis.Sd->group_sleep(Chassis.Sd);
        Chassis.Wheel->group_sleep(Chassis.Wheel);
    }
    Sd_Group.group_set_torque(&Sd_Group);
    Chassis.Wheel->motor[R_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[R_WHEEL_M]);
    Chassis.Wheel->motor[L_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[L_WHEEL_M]);
    shoot.dail_info.dail_motor->W_iqControl(shoot.dail_info.dail_motor, shoot.dail_info.dail_output);
    shoot.dail_info.dail_motor->tx_W_cmd(shoot.dail_info.dail_motor, TORQUE_CLOSE_LOOP_ID);

    Send_To_Up_Board();
    cap.setdata(&cap);
    cap.txdata(&cap);
#else
    Sd_Group.group_set_torque(&Sd_Group);
#endif
}

static void Motor_Unload_Force(void)
{
    // if (RC_OFFLINE && Chassis.damping_delay_cnt >= DAMPING_DELAY_MAX_CNT)
    // {
    //     Sd_Group.group_sleep(&Sd_Group);
    // }
#ifdef TEST_MY_LEG
    Sd_Group.group_sleep(&Sd_Group);
#endif
    Chassis.Sd->group_sleep(Chassis.Sd);
    Chassis.Wheel->group_sleep(Chassis.Wheel);
    Chassis.Sd->group_set_torque(Chassis.Sd);
#ifndef TEST_RESCUE
    Yaw_Motor.tx_info->torque = 0;
    Yaw_Motor.single_set_torque(&Yaw_Motor);
    Chassis.Wheel->motor[R_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[R_WHEEL_M]);
    Chassis.Wheel->motor[L_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[L_WHEEL_M]);
    shoot.dail_info.dail_motor->W_iqControl(shoot.dail_info.dail_motor, 0);
    shoot.dail_info.dail_motor->tx_W_cmd(shoot.dail_info.dail_motor, TORQUE_CLOSE_LOOP_ID);
    Board_Tx_Info.pitch_output = 0;
    Board_Tx_Info.flag.bit.is_rc_online = 0;

    Send_To_Up_Board();
    cap.setdata(&cap);
    cap.txdata(&cap);
#endif
}