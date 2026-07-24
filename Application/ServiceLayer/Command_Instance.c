

#include "Command_Instance.h"

command_t command[COMMAND_LIST] =

    {

        [JUMP] = {

            .cmd_type = RISE_TRIGER_C,

            .run_time_max = OUT_TIME_OFF,

            .init = Cmd_Class_Init,

        },

        [KNEE_STRIKE] = {

            .cmd_type = RISE_TRIGER_C,

            .run_time_max = OUT_TIME_OFF,

            .init = Cmd_Class_Init,

        },

        [JUMP_THEN_KNEE_STRIKE] = {

            .cmd_type = RISE_TRIGER_C,

            .run_time_max = OUT_TIME_OFF,

            .init = Cmd_Class_Init,

        },

        [SW_MID_LENGTH] = {

            .cmd_type = RISE_TRIGER_C,

            .run_time_max = OUT_TIME_OFF,

            .init = Cmd_Class_Init,

        },

        [JUMP_AND_MID] = {

            .cmd_type = RISE_TRIGER_C,

            .run_time_max = OUT_TIME_OFF,

            .init = Cmd_Class_Init,

        },

        [RTS] = {
            .cmd_type = RISE_TRIGER_C,
            .run_time_max = OUT_TIME_OFF,
            .init = Cmd_Class_Init,
        },

        [Energy_Engine_Mode] = {

            .cmd_type = HIGH_TRIGER_C,

            .run_time_max = OUT_TIME_OFF,

            .init = Cmd_Class_Init,

        },
        [Outpost_Mode] = {

            .cmd_type = HIGH_TRIGER_C,

            .run_time_max = OUT_TIME_OFF,

            .init = Cmd_Class_Init,

        },

        [Reset_to_Normal] = {

            .cmd_type = HIGH_TRIGER_C,

            .run_time_max = OUT_TIME_OFF,

            .init = Cmd_Class_Init,

        },
        [PRE_CHARGE_MODE] = {

            .cmd_type = RISE_TRIGER_C,

            .run_time_max = OUT_TIME_OFF,

            .init = Cmd_Class_Init,

        },
        [GIMBAL_180] = {

            .cmd_type = RISE_TRIGER_C,

            .run_time_max = OUT_TIME_OFF,

            .init = Cmd_Class_Init,

        },

};

/**

 * @brief 命令初始化，调用一次

 */

void Cmd_Init(void)

{

    for (uint8_t i = 0; i < COMMAND_LIST; i++)

    {

        command[i].init(&command[i]);
    }
}

/**

 * @brief 命令心跳 循环调用

 */

void Cmd_Heartbeat(void)

{

    for (uint8_t i = 0; i < COMMAND_LIST; i++)

    {

        command[i].heartbeat(&command[i]);
    }
}

/**

 * @brief 命令更新 循环调用

 */

void Command_Update(void)

{

    static uint32_t RC_ONLINE_TICK;

    rc_sensor_info_t *rc_info = Balance.rc->sensor->info;

    static uint8_t wheel_up, last_wheel_up, wheel_dn, last_wheel_dn = 0;

    last_wheel_up = wheel_up;

    wheel_up = rc_info->thumbwheel.step[RC_TB_UP];

    last_wheel_dn = wheel_dn;

    wheel_dn = rc_info->thumbwheel.step[RC_TB_DN];

    if (Balance.rc->sensor->work_state == DEV_ONLINE)
    {
        RC_ONLINE_TICK++;
    }

    else
    {
        RC_ONLINE_TICK = 0;
    }

    static uint8_t last_rc_info_s1;

    if (RC_ONLINE_TICK >= 200) // 防止开控时触发命令

    {

        if (Balance.ctrl != KEY_CTRL)

        {
            command[Outpost_Mode].update(&command[Outpost_Mode], rc_info->s1 == RC_SW_MID &&
                                                                     rc_info->s2 == RC_SW_UP &&
                                                                     wheel_dn != last_wheel_dn);
            command[Energy_Engine_Mode].update(&command[Energy_Engine_Mode], rc_info->s1 == RC_SW_MID &&
                                                                                 rc_info->s2 == RC_SW_DOWN &&
                                                                                 wheel_dn != last_wheel_dn);
            // command[JUMP].update(&command[JUMP], rc_info->s1 == RC_SW_UP &&

            //                                          rc_info->s2 == RC_SW_UP &&

            //                                          wheel_up != last_wheel_up &&
            //                                          Balance.Flag->KNEE_STRIKE_Flag == false &&
            //                                          Balance.Flag->Middle_Flag == false);
            // command[RTS].update(&command[RTS], rc_info->s1 == RC_SW_UP &&
            //                                        rc_info->s2 == RC_SW_UP &&
            //                                        wheel_up != last_wheel_up &&
            //                                        Balance.Flag->KNEE_STRIKE_Flag == false &&
            //                                        Balance.Flag->Jumping_Flag == false &&
            //                                        Balance.Flag->Middle_Flag == false);

            command[KNEE_STRIKE].update(&command[KNEE_STRIKE], rc_info->s1 == RC_SW_UP &&

                                                                   rc_info->s2 == RC_SW_UP &&

                                                                   wheel_dn != last_wheel_dn &&
                                                                   Balance.Flag->Middle_Flag == false &&
                                                                   Balance.Flag->JUMP_AND_MID_Flag == false);

            command[SW_MID_LENGTH].update(&command[SW_MID_LENGTH], rc_info->s1 == RC_SW_UP &&

                                                                       rc_info->s2 == RC_SW_MID &&

                                                                       wheel_dn != last_wheel_dn &&
                                                                       Balance.Flag->KNEE_STRIKE_Flag == false &&
                                                                       Balance.Flag->JUMP_AND_MID_Flag == false);

            command[Reset_to_Normal].update(&command[Reset_to_Normal], rc_info->s1 == RC_SW_UP &&
                                                                           rc_info->s2 == RC_SW_DOWN &&
                                                                           wheel_up != last_wheel_up);

            // command[PRE_CHARGE_MODE].update(&command[PRE_CHARGE_MODE], rc_info->s1 == RC_SW_DOWN &&
            //                                                                rc_info->s2 == RC_SW_UP &&
            //                                                                wheel_up != last_wheel_up);
        }

        else

        {
            command[Outpost_Mode].update(&command[Outpost_Mode], rc_info->E.value == true);
            command[Energy_Engine_Mode].update(&command[Energy_Engine_Mode], rc_info->X.value == true);

            command[KNEE_STRIKE].update(&command[KNEE_STRIKE], rc_info->C.status == release_to_press &&
                                                                   Balance.Flag->Middle_Flag == false &&
                                                                   Balance.Flag->JUMP_AND_MID_Flag == false &&
                                                                   Balance.Flag->KNEE_STRIKE_Flag != true);

            command[Reset_to_Normal].update(&command[Reset_to_Normal], rc_info->Ctrl.status == release_to_press);

            command[SW_MID_LENGTH].update(&command[SW_MID_LENGTH], rc_info->G.status == release_to_press &&
                                                                       Balance.Flag->KNEE_STRIKE_Flag == false &&
                                                                       Balance.Flag->JUMP_AND_MID_Flag == false &&
                                                                       Balance.Flag->Middle_Flag != true);

            command[JUMP].update(&command[JUMP], rc_info->Q.status == release_to_press &&
                                                     Balance.Flag->KNEE_STRIKE_Flag == false &&
                                                     Balance.Flag->Middle_Flag == false);

            // command[RTS].update(&command[RTS],
            //                     rc_info->Q.status == release_to_press &&
            //                         Chassis.mode == C_Follow &&
            //                         Balance.Flag->Middle_Flag == false &&
            //                         Balance.Flag->JUMP_AND_MID_Flag == false &&
            //                         Balance.Flag->KNEE_STRIKE_Flag == false &&
            //                         Balance.Flag->RTS_Flag == false);
            // command[PRE_CHARGE_MODE].update(&command[PRE_CHARGE_MODE],);
            command[GIMBAL_180].update(&command[GIMBAL_180], rc_info->R.status == release_to_press &&
                                                                 Balance.Flag->GIMBAL_180_Flag != true);
        }
    }

    last_rc_info_s1 = rc_info->s1;
}
