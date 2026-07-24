#include "Balance.h"
#include "RP_Log.h"

void Balance_Init(Balance_t *balance);
static void Balance_Status_Update(Balance_t *balance);
static void Balance_ManualRescueKeyMonitor(Balance_t *balance);
static void RC_Move_Mode_Update(Balance_t *balance);
static void KEY_Move_Mode_Update(Balance_t *balance);
static void Rescue_Check(Balance_t *balance);
static void Balance_Special_Move_Command_Update(Balance_t *balance);
static void Balance_Clean_Process(Balance_t *balance);
static void Balance_Test_Rescue_Status_Update(Balance_t *balance);
static void Balance_SitDown_Mode_Status_Update(Balance_t *balance);
static void Check_Wheel_Down_5_Times(Balance_t *balance);
uint8_t last_step[4];

Balance_Remote_Ctrl Balance_Rc = {
    .sensor = &rc_sensor,
    .last_thumbwheel_step = last_step,
};
Balance_Flag_t Balance_Flag;
Balance_t Balance = {
    .ctrl = RC_CTRL,
    .mode = Sleep_Mode,
    .init = Balance_Init,
    .Flag = &Balance_Flag,
    .rc = &Balance_Rc,
};

void Balance_Init(Balance_t *balance)
{
#ifdef TEST_SITDOWN_MODE
    balance->update = Balance_SitDown_Mode_Status_Update;
#elif defined TEST_RESCUE
    balance->update = Balance_Test_Rescue_Status_Update;
#else
    balance->update = Balance_Status_Update;
#endif

    balance->command = command;
}

/**
 * @brief  Balance状态更新，包含开关控、模式切换、自救判断、打弹等
 * @note 状态机流程: Sleep_Mode -> Rescue_Mode -> Init_Mode -> Mec_Mode/Imu_Mode
 */
static void Balance_Status_Update(Balance_t *balance)
{
    /* 手动自救模式键盘检测 */
    Balance_ManualRescueKeyMonitor(balance);

    /* 保存上一次模式，用于检测自救完成下降沿 */
    Balance.last_mode = Balance.mode;

    /* ---------- RC在线状态计数 ---------- */
    if (RC_ONLINE)
    {
        balance->rc->rc_online_tick++;
    }
    else
    {
        balance->rc->rc_online_tick = 0;
    }
    /* ----------------------------------- Balance状态机更新相关 begin------------------------ */
    {
/* ---------- 自救检测 ---------- */
#ifndef TEST_NO_RESCUE
        Rescue_Check(balance);
#endif

        /* ---------- 波轮向下跳变5次检测 ---------- */
        Check_Wheel_Down_5_Times(balance);

        /* ---------- Wheel_Down_5_Flag触发SitDown模式 ---------- */
        if (balance->Flag->Wheel_Down_5_Flag == true)
        {
            balance->update = Balance_SitDown_Mode_Status_Update;
        }

        /* ---------- 根据chassis、gimbal状态来判断Balance_reset_NO ---------- */
        if (balance->Flag->Rescue_Flag == false &&
            gimbal.gimbal_reset_state == DEV_RESET_OK) // 不用自救并且云台复位完成
        {
            balance->reset_state = Balance_reset_OK;
        }
        else
        {
            balance->reset_state = Balance_reset_NO;
        }

        /* ---------- Balance状态机 ---------- */
        switch (balance->mode)
        {
        case Sleep_Mode:
            Balance_Clean_Process(balance);
#ifdef NO_REFEREE_SYSTEM
            if (rc_sensor.work_state == DEV_ONLINE)
#else
            // 开控并且机器人没阵亡
            if (rc_sensor.work_state == DEV_ONLINE &&
                (My_Judge.info->remain_HP != 0 && My_Judge.status->status == DEV_ONLINE))
#endif
            {
                if (balance->Flag->Rescue_Flag == true)
                {
                    balance->mode = Rescue_Mode;
                }
                else
                {
                    balance->mode = Init_Mode;
                }
            }
            break;

        case Init_Mode:
        case Rescue_Mode:
            if (balance->reset_state == Balance_reset_OK)
            {
                /* 自救完成下降沿: 启动2秒屏蔽计时，避免姿态未稳时再次触发自救 */
                if (Balance.last_mode == Rescue_Mode && balance->Flag->NoCheckRescueFlag == false)
                {
                    balance->Flag->NoCheckRescueFlag = true;
                    balance->Flag->NoCheckRescue_StartTick = HAL_GetTick();
                }
#ifdef TEST_MEC_MODE
                balance->mode = Mec_Mode;
#else
                balance->mode = Imu_Mode;
#endif
            }
            break;

        case Imu_Mode:
        case Mec_Mode:
        case Cycle_Mode:
            if (balance->Flag->Rescue_Flag == true)
            {
                balance->mode = Rescue_Mode;
                break;
            }
            /* 根据左右拨杆判断控制方式: s1=下 && s2=下 -> 键盘控制，否则 -> 遥控控制 */
            if (rc_sensor.info->s1 == RC_SW_DOWN && rc_sensor.info->s2 == RC_SW_DOWN)
            {
                balance->ctrl = KEY_CTRL;
            }
            else
            {
                balance->ctrl = RC_CTRL;
            }

            if (balance->ctrl == RC_CTRL)
            {
                RC_Move_Mode_Update(balance);
            }
            else
            {
                KEY_Move_Mode_Update(balance);
            }

            break;

        default:
            break;
        }
        /* ---------- 最高优先级：遥控掉线 -> Sleep_Mode ---------- */
#ifdef NO_REFEREE_SYSTEM
        if (rc_sensor.work_state == DEV_OFFLINE)
#else
        if (rc_sensor.work_state == DEV_OFFLINE ||
            (My_Judge.info->remain_HP == 0 && My_Judge.status->status == DEV_ONLINE))
#endif
        {
            balance->mode = Sleep_Mode;
        }
    }
    /* ----------------------------------- Balance状态机更新相关 end------------------------ */
}

/**
 * @brief  底盘自救判断
 * @note - 自救中Rescue_Check函数不生效，仅在Rescue_Flag = false负责首次进入状态机
 *       - 状态机处理函数Rescue_State_Process通过写Balance_reset_OK、Rescue_Flag=ture 标志位而Balance_Status_Update函数检测来退出自救
 */
static void Rescue_Check(Balance_t *balance)
{
    Chassis_Rescue_info_t *rescue_info = Chassis.rescue_info;
    Chassis_Posture_info_t *posture_info = Chassis.Posture->info;
    float pitch_degree = posture_info->pitch_degree;
    float roll_degree = posture_info->roll_degree;
    float theta_R_degree = Chassis.Leg_Unit[R_Leg]->Straight->info->thetal / Degree_to_rad;
    float theta_L_degree = Chassis.Leg_Unit[L_Leg]->Straight->info->thetal / Degree_to_rad;
    // 自救完成后屏蔽2秒，避免姿态未稳时再次触发自救
    // if (balance->Flag->NoCheckRescueFlag == true && my_abs(pitch_degree) < 40.f)
    // {
    //     uint32_t elapsed = HAL_GetTick() - balance->Flag->NoCheckRescue_StartTick;
    //     if (elapsed >= 1000)
    //     {
    //         balance->Flag->NoCheckRescueFlag = false;
    //     }
    //     else
    //     {
    //         return;
    //     }
    // }

    if (balance->Flag->Rescue_Flag == true)
    {
        return;
    }

    /*************************** 腿、机体姿态都正常 *******************************/
    if ((theta_R_degree < rescue_info->rescue_config.thetalBackwardThreshold &&
         theta_L_degree < rescue_info->rescue_config.thetalBackwardThreshold &&
         theta_R_degree > rescue_info->rescue_config.thetalForwardThreshold &&
         theta_L_degree > rescue_info->rescue_config.thetalForwardThreshold) &&
        (pitch_degree >= rescue_info->rescue_config.pitchForwardFlipThreshold &&
         pitch_degree <= rescue_info->rescue_config.pitchBackwardFlipThreshold) &&
        (roll_degree <= rescue_info->rescue_config.rollRightRolloverThreshold &&
         roll_degree >= rescue_info->rescue_config.rollLeftRolloverThreshold))
    {

        return;
    }
    /*************** 腿异常，Pitch、Roll正常，进入PRNormalBackwardLeg状态 *******************/
    else if ((theta_R_degree > rescue_info->rescue_config.thetalBackwardThreshold ||
              theta_L_degree > rescue_info->rescue_config.thetalBackwardThreshold ||
              theta_R_degree < rescue_info->rescue_config.thetalForwardThreshold ||
              theta_L_degree < rescue_info->rescue_config.thetalForwardThreshold) &&
             (pitch_degree >= rescue_info->rescue_config.pitchForwardFlipThreshold &&
              pitch_degree <= rescue_info->rescue_config.pitchBackwardFlipThreshold) &&
             (roll_degree <= rescue_info->rescue_config.rollRightRolloverThreshold &&
              roll_degree >= rescue_info->rescue_config.rollLeftRolloverThreshold))
    {
        balance->Flag->Rescue_Flag = true;
        rescue_info->stateMacineTimelineTick = HAL_GetTick();
        rescue_info->rescue_state_mac = PRNormalBackwardLeg;
        RP_LOG_INFO("Rescue_Check: enter PRNormalBackwardLeg, pitch=%.2f, roll=%.2f, thetaL=%.2f, thetaR=%.2f",
                    pitch_degree, roll_degree, theta_L_degree, theta_R_degree);
    }
    /************************** roll右翻 + ******************************/
    else if (roll_degree > rescue_info->rescue_config.rollRightRolloverThreshold)
    {
        if (pitch_degree < rescue_info->rescue_config.pitchForwardFlipThreshold)
        {
            balance->Flag->Rescue_Flag = true;
            rescue_info->rescue_state_mac = ForwardFlip_and_R_Rollover;
            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            RP_LOG_INFO("Rescue_Check: enter ForwardFlip_and_R_Rollover, pitch=%.2f, roll=%.2f",
                        pitch_degree, roll_degree);
        }
        else if (pitch_degree > rescue_info->rescue_config.pitchBackwardFlipThreshold)
        {
            balance->Flag->Rescue_Flag = true;
            rescue_info->rescue_state_mac = BackwardFlip_and_R_Rollover;
            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            RP_LOG_INFO("Rescue_Check: enter BackwardFlip_and_R_Rollover, pitch=%.2f, roll=%.2f",
                        pitch_degree, roll_degree);
        }
        else
        {
            balance->Flag->Rescue_Flag = true;
            rescue_info->rescue_state_mac = R_Rollover;
            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            RP_LOG_INFO("Rescue_Check: enter R_Rollover, pitch=%.2f, roll=%.2f",
                        pitch_degree, roll_degree);
        }
    }
    /************************ roll左翻 + ******************************/
    else if (roll_degree < rescue_info->rescue_config.rollLeftRolloverThreshold)
    {
        if (pitch_degree < rescue_info->rescue_config.pitchForwardFlipThreshold)
        {
            balance->Flag->Rescue_Flag = true;
            rescue_info->rescue_state_mac = ForwardFlip_and_L_Rollover;
            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            RP_LOG_INFO("Rescue_Check: enter ForwardFlip_and_L_Rollover, pitch=%.2f, roll=%.2f",
                        pitch_degree, roll_degree);
        }
        else if (pitch_degree > rescue_info->rescue_config.pitchBackwardFlipThreshold)
        {
            balance->Flag->Rescue_Flag = true;
            rescue_info->rescue_state_mac = BackwardFlip_and_L_Rollover;
            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            RP_LOG_INFO("Rescue_Check: enter BackwardFlip_and_L_Rollover, pitch=%.2f, roll=%.2f",
                        pitch_degree, roll_degree);
        }
        else
        {
            balance->Flag->Rescue_Flag = true;
            rescue_info->rescue_state_mac = L_Rollover;
            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            RP_LOG_INFO("Rescue_Check: enter L_Rollover, pitch=%.2f, roll=%.2f",
                        pitch_degree, roll_degree);
        }
    }
    /************************ Roll正常，判断前翻或后翻 ******************************/
    else if (pitch_degree < rescue_info->rescue_config.pitchForwardFlipThreshold)
    {
        balance->Flag->Rescue_Flag = true;
        rescue_info->rescue_state_mac = ForwardFlip;
        rescue_info->stateMacineTimelineTick = HAL_GetTick();
        RP_LOG_INFO("Rescue_Check: enter ForwardFlip, pitch=%.2f, roll=%.2f",
                    pitch_degree, roll_degree);
    }
    else if (pitch_degree > rescue_info->rescue_config.pitchBackwardFlipThreshold)
    {
        balance->Flag->Rescue_Flag = true;
        rescue_info->rescue_state_mac = BackwardFlip;
        rescue_info->stateMacineTimelineTick = HAL_GetTick();
        RP_LOG_INFO("Rescue_Check: enter BackwardFlip, pitch=%.2f, roll=%.2f",
                    pitch_degree, roll_degree);
    }
    else
    {
        rescue_info->unknown_posture_cnt++;
        rescue_info->rescue_state_mac = RetractLegs;
        RP_LOG_INFO("Rescue_Check: enter RetractLegs (unknown posture), pitch=%.2f, roll=%.2f, thetaL=%.2f, thetaR=%.2f",
                    pitch_degree, roll_degree, theta_L_degree, theta_R_degree);
    }
}

/**
 * @brief  RC/KEY共用的特殊移动动作命令识别
 * @param  Balance_t* balance
 * @retval None
 */
static void Balance_Special_Move_Command_Update(Balance_t *balance)
{
    // 跳跃命令标志位
    if (balance->command[JUMP].cmd_value == true && balance->Flag->Jumping_Flag != true)
    {
        balance->Flag->Jumping_Flag = true;
    }

    /*------------------------- 磕膝上台阶命令处理 begin----------------------------*/
    if (balance->command[KNEE_STRIKE].cmd_value == true)
    {
        if (balance->Flag->KNEE_STRIKE_Flag == false)
        {
            // 尚未进入磕膝流程，检查底盘朝向
            if (my_abs(gimbal.base_info.yaw_motor_angle) <= 0.3f &&
                my_abs(gimbal.base_info.yaw_motor_speed) <= 0.1f)
            {
                // 底盘已对齐：直接进入磕膝上台阶
                balance->Flag->KNEE_STRIKE_Flag = true;
            }
            else
            {
                // 底盘未对齐：等待对齐后再进入
                balance->Flag->Chassis_Alignment_Flag = true;
                balance->Flag->KNEE_STRIKE_after_Alignment = true;
            }
        }
        else
        {
            // 已经在磕膝流程，触发命令则退出
            balance->Flag->KNEE_STRIKE_Flag = false;
            balance->Flag->KNEE_STRIKE_after_Alignment = false;
        }
    }

    // 独立复位退出：Reset_to_Normal触发时，若已处于磕膝流程则直接退出
    if (balance->command[Reset_to_Normal].cmd_value == true &&
        balance->Flag->KNEE_STRIKE_Flag == true)
    {
        balance->Flag->KNEE_STRIKE_Flag = false;
        balance->Flag->KNEE_STRIKE_after_Alignment = false;
    }

    // 对齐检测：等待对齐过程中，一旦朝向对齐则进入磕膝
    if (balance->Flag->KNEE_STRIKE_after_Alignment == true &&
        my_abs(gimbal.base_info.yaw_motor_angle) <= 0.3f &&
        my_abs(gimbal.base_info.yaw_motor_speed) <= 0.1f)
    {
        balance->Flag->KNEE_STRIKE_after_Alignment = false;
        balance->Flag->KNEE_STRIKE_Flag = true;
    }
    /*------------------------- 磕膝上台阶命令处理 end----------------------------*/

    // 小跳后上台阶标志位
    if (balance->command[JUMP_THEN_KNEE_STRIKE].cmd_value == true)
    {
        balance->Flag->JUMP_THEN_KNEE_STRIKE_Flag = true;
    }

    // 复位到正常状态标志位（包括磕膝、底盘对齐模式等）
    if (balance->command[Reset_to_Normal].cmd_value == true)
    {
        balance->Flag->Chassis_Alignment_Flag = true;
    }

    // 小跳中腿长下台阶标志位
    if (balance->command[JUMP_AND_MID].cmd_value == true)
    {
        balance->Flag->JUMP_AND_MID_Flag = true;
    }

    /* 中腿长下台阶标志位 */
    if (balance->command[SW_MID_LENGTH].cmd_value == true)
    {
        balance->Flag->Middle_Flag = !balance->Flag->Middle_Flag;
    }

    /*------------------------- 收腿下二级台阶命令处理 begin-------------------------*/
    // 收到RTS命令时设置标志位为真
    if (balance->command[RTS].cmd_value == true)
    {
        balance->Flag->RTS_Flag = !balance->Flag->RTS_Flag;
    }
    /*------------------------- 收腿下二级台阶命令处理 end-------------------------*/
    // 云台180度转向标志位
    if (balance->command[GIMBAL_180].cmd_value == true)
    {
        balance->Flag->GIMBAL_180_Flag = true;
    }
    // 预充电模式标志位
    if (balance->command[PRE_CHARGE_MODE].cmd_value == true)
    {
        balance->Flag->Pre_Charge_Flag != balance->Flag->Pre_Charge_Flag;
    }
}

/**
 * @brief 遥控模式整车移动模式更新
 */
static void RC_Move_Mode_Update(Balance_t *balance)
{
    rc_sensor_info_t *rc_info = balance->rc->sensor->info;

    /*------------------------- 特殊移动动作命令识别 begin----------------------------*/
    if (balance->mode == Imu_Mode || balance->mode == Mec_Mode)
    {
        // RC/KEY共用的特殊移动动作命令识别
        Balance_Special_Move_Command_Update(balance);
    }
    else
    {
        // 其他模式不接受命令
        balance->command[JUMP].cmd_value = false;
        balance->command[KNEE_STRIKE].cmd_value = false;
        balance->command[JUMP_THEN_KNEE_STRIKE].cmd_value = false;
        balance->command[JUMP_AND_MID].cmd_value = false;
        balance->command[Reset_to_Normal].cmd_value = false;
    }
    /*------------------------- 特殊移动动作命令识别 end----------------------------*/
    /*------------------------- 模式切换、打弹命令识别 begin----------------------------*/
    static uint8_t wheel_up, last_wheel_up, wheel_dn, last_wheel_dn, last_rc_s1, rc_s1, last_rc_s2, rc_s2;
    last_wheel_up = wheel_up;
    wheel_up = rc_info->thumbwheel.step[RC_TB_UP];
    last_wheel_dn = wheel_dn;
    wheel_dn = rc_info->thumbwheel.step[RC_TB_DN];
    last_rc_s1 = rc_s1;
    rc_s1 = rc_info->s1;
    last_rc_s2 = rc_s2;
    rc_s2 = rc_info->s2;
    static uint32_t cnt;
    switch (balance->mode)
    {
    case Imu_Mode:
        if (rc_info->s1 == RC_SW_UP && rc_info->s2 == RC_SW_MID && last_wheel_up != wheel_up)
        {
            balance->mode = Cycle_Mode;
        }
        goto common_shoot_logic;
    return_from_imu_shoot_logic:
        break;

    case Mec_Mode:;

#ifdef TEST_MEC_MODE
        if (rc_info->s1 == RC_SW_UP && rc_info->s2 == RC_SW_MID && last_wheel_up != wheel_up)
#else
        if (rc_info->s1 == RC_SW_UP && rc_info->s2 == RC_SW_DOWN && last_wheel_up != wheel_up)
#endif

        {
#ifdef TEST_MEC_MODE

            balance->mode = Cycle_Mode;

#else
            balance->mode = Imu_Mode;
#endif
        }

        goto common_shoot_logic;
    return_from_mec_shoot_logic:
        break;

    case Cycle_Mode:
        if ((rc_info->s1 == RC_SW_UP && rc_info->s2 == RC_SW_MID && last_wheel_up != wheel_up) ||
            (balance->command[Reset_to_Normal].cmd_value == true))
        {
            // 让机械模式可以正常进退小陀螺测试
#ifdef TEST_MEC_MODE
            balance->mode = Mec_Mode;
#else
            balance->mode = Imu_Mode;
#endif
        }
        goto common_shoot_logic;
    return_from_cycle_shoot_logic:
        break;

    case SitDown_Mode:
        goto common_shoot_logic;
    return_from_sitdown_shoot_logic:
        break;

    default:
        break;
    }
    return;

    /****************************** 打弹、自瞄逻辑 ******************************/
common_shoot_logic:
    // 开关摩擦轮
    if (rc_info->s1 == RC_SW_MID && rc_info->s2 == RC_SW_MID && last_wheel_up != wheel_up)
    {
        balance->Shoot_Flag_struct.Enable_Shoot_Flag = !balance->Shoot_Flag_struct.Enable_Shoot_Flag;
    }
    // 切换自瞄标志位
    if (rc_info->s1 == RC_SW_MID && rc_info->s2 == RC_SW_MID && last_wheel_dn != wheel_dn)
    {
        balance->Vision.Auto_Catch_Flag = !balance->Vision.Auto_Catch_Flag;
    }
    // 切换自瞄前哨标志位
    if (balance->command[Outpost_Mode].cmd_value == true)
    {
        balance->Vision.Auto_Catch_Outpost_Flag = !balance->Vision.Auto_Catch_Outpost_Flag;
    }
    // 切换自瞄大符标志位
    if (balance->command[Energy_Engine_Mode].cmd_value == true)
    {
        balance->Vision.Auto_Catch_Engi_Flag = !balance->Vision.Auto_Catch_Engi_Flag;
    }

    // 开发射机构了，并且不在自瞄
    if (balance->Shoot_Flag_struct.Enable_Shoot_Flag &&
        rc_info->s1 == RC_SW_MID && balance->Vision.Auto_Catch_Flag == false)
    {
        // 左中 右跳上 单发
        if (rc_s2 == RC_SW_UP && last_rc_s2 == RC_SW_MID)
        {
            balance->Shoot_Flag_struct.Shoot_Ctrl_Flag = 1;
            balance->Shoot_Flag_struct.Shoot_Mode = 0;
        }
        // 左中 右下 持续连发
        else if (rc_s2 == RC_SW_DOWN)
        {
            balance->Shoot_Flag_struct.Shoot_Ctrl_Flag = 1;
            balance->Shoot_Flag_struct.Shoot_Mode = 1;
        }
        else
        {
            balance->Shoot_Flag_struct.Shoot_Ctrl_Flag = 0;
            balance->Shoot_Flag_struct.Shoot_Mode = 1;
        }
    }
    // 开发射机构并且在自瞄
    if (balance->Shoot_Flag_struct.Enable_Shoot_Flag == 1 &&
        balance->Vision.Auto_Catch_Flag == 1)
    {
        // 左中 右下 持续进自瞄火控
        if (rc_s1 == RC_SW_MID && rc_s2 == RC_SW_DOWN)
        {
            balance->Shoot_Flag_struct.Shoot_Ctrl_Flag = Board_Rx_Info.flag.hit_enable;
            balance->Shoot_Flag_struct.Shoot_Mode = Board_Rx_Info.flag.is_keep_shoot;
        }
        else
        {
            balance->Shoot_Flag_struct.Shoot_Ctrl_Flag = 0;
            balance->Shoot_Flag_struct.Shoot_Mode = 1;
        }

        // 左中 右跳上 单发，自瞄时也可以测试单发打弹功能
        if (rc_s2 == RC_SW_UP && last_rc_s2 == RC_SW_MID)
        {
            balance->Shoot_Flag_struct.Shoot_Ctrl_Flag = 1;
            balance->Shoot_Flag_struct.Shoot_Mode = 0;
        }
    }
    if (balance->mode == Imu_Mode)
    {
        goto return_from_imu_shoot_logic;
    }
    else if (balance->mode == Mec_Mode || balance->mode == LEG_TEST_Mode || balance->mode == SitDown_Mode)
    {
        goto return_from_mec_shoot_logic;
    }
    else if (balance->mode == Cycle_Mode)
    {
        goto return_from_cycle_shoot_logic;
    }
    else if (balance->mode == SitDown_Mode)
    {
        goto return_from_sitdown_shoot_logic;
    }

    /*------------------------- 模式切换、打弹命令识别 end----------------------------*/
}

/**
 * @brief 键鼠模式整车移动模式更新
 */
static void KEY_Move_Mode_Update(Balance_t *balance)
{
    rc_sensor_info_t *rc_info = balance->rc->sensor->info;
    static uint16_t cnt;
    /*------------------------- 特殊移动动作命令识别 begin----------------------------*/
    if (balance->mode == Imu_Mode || balance->mode == Mec_Mode)
    {
        // RC/KEY共用的特殊移动动作命令识别
        Balance_Special_Move_Command_Update(balance);
    }
    if (balance->command[Reset_to_Normal].cmd_value == true)
    {
        balance->Flag->RTS_Flag = false;
        balance->Flag->Chassis_Alignment_Flag = true;
        balance->Flag->Middle_Flag = false;
        balance->Flag->JUMP_AND_MID_Flag = false;
    }
    /*------------------------- 特殊移动动作命令识别 end----------------------------*/
    /*------------------------- 模式切换、打弹命令识别 begin----------------------------*/
    if (rc_info->Shift.status == release_to_press)
    {
        if (balance->mode != Cycle_Mode)
        {
            balance->mode = Cycle_Mode;
        }
        // else
        // {
        //     balance->mode = Imu_Mode;
        // }
    }
    if (balance->command[Reset_to_Normal].cmd_value == true)
    {
        balance->mode = Imu_Mode;
    }
    if ((balance->mode == Imu_Mode || balance->mode == Mec_Mode || balance->mode == Cycle_Mode) &&
        rc_info->mouse_btn_l.status == release_to_press)
    {
        balance->Shoot_Flag_struct.Enable_Shoot_Flag = true;
    }
    if ((balance->mode == Imu_Mode || balance->mode == Mec_Mode || balance->mode == Cycle_Mode) &&
        rc_info->B.status == release_to_press && balance->Shoot_Flag_struct.Enable_Shoot_Flag == true)
    {
        balance->Shoot_Flag_struct.Enable_Shoot_Flag = false;
    }
    /*------------------ 自瞄标志位相关 --------------------- */
    if (rc_info->mouse_btn_r.value == true)
    {
        balance->Vision.Auto_Catch_Flag = 1;
        // 只打能量机关标志位
        if (command[Energy_Engine_Mode].cmd_value == true)
        {
            balance->Vision.Auto_Catch_Engi_Flag = 1;
            balance->Vision.Auto_Catch_Outpost_Flag = 0;
        }
        else
        {
            balance->Vision.Auto_Catch_Engi_Flag = 0;
        }

        // 只打前哨标志位
        if (command[Outpost_Mode].cmd_value == true)
        {
            balance->Vision.Auto_Catch_Outpost_Flag = 1;
            balance->Vision.Auto_Catch_Engi_Flag = 0;
        }
        else
        {
            balance->Vision.Auto_Catch_Outpost_Flag = 0;
        }
    }
    else
    {
        balance->Vision.Auto_Catch_Flag = 0;
        balance->Vision.Auto_Catch_Outpost_Flag = 0;
        balance->Vision.Auto_Catch_Engi_Flag = 0;
    }

    /*------------------ 打弹标志位相关 --------------------- */
    if (balance->Shoot_Flag_struct.Enable_Shoot_Flag == 1)
    {
        if (balance->Vision.Auto_Catch_Flag == 0) // 不在自瞄
        {
            if (rc_info->mouse_btn_l.value == true)
            {
                balance->Shoot_Flag_struct.Shoot_Ctrl_Flag = 1; // 开火
                balance->Shoot_Flag_struct.Shoot_Mode = 1;      // 连发
            }
            else
            {
                balance->Shoot_Flag_struct.Shoot_Ctrl_Flag = 0;
            }
        }
        else
        {
            if (rc_info->mouse_btn_l.value == true)
            {
                balance->Shoot_Flag_struct.Shoot_Ctrl_Flag = Board_Rx_Info.flag.hit_enable;
                balance->Shoot_Flag_struct.Shoot_Mode = Board_Rx_Info.flag.is_keep_shoot;
            }
            else
            {
                balance->Shoot_Flag_struct.Shoot_Ctrl_Flag = 0;
            }
        }
    }

    /*------------------------- 模式切换、打弹命令识别 end----------------------------*/
}

static uint8_t key_count = 0;
static uint32_t last_z_press_time = 0;

/**
 * @brief 检测波轮向下跳变5次，中间每次间隔小于1s，则Wheel_Down_5_Flag置true
 */
static void Check_Wheel_Down_5_Times(Balance_t *balance)
{
    rc_sensor_info_t *rc_info = balance->rc->sensor->info;
    uint8_t wheel_dn = rc_info->thumbwheel.step[RC_TB_DN];
    static uint8_t last_wheel_dn = 0;
    static uint8_t jump_count = 0;
    static uint32_t last_jump_time = 0;

    // 检测到跳变（当前值与上一次不同）
    if (wheel_dn != last_wheel_dn)
    {
        uint32_t current_time = HAL_GetTick();

        // 第一次跳变或者间隔小于1s
        if (jump_count == 0 || (current_time - last_jump_time) < 1000)
        {
            jump_count++;
            last_jump_time = current_time;

            // 达到5次跳变
            if (jump_count >= 5)
            {
                balance->Flag->Wheel_Down_5_Flag = true;
                jump_count = 0;
            }
        }
        else
        {
            // 间隔超过1s，重置计数
            jump_count = 1;
            last_jump_time = current_time;
        }

        last_wheel_dn = wheel_dn;
    }
}

/**
 * @brief 检测Z键是否按下
 */
bool is_z_key_pressed(void)
{
    if (rc_sensor.info->Z.status == release_to_press)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Z键连续按5次检测逻辑（需周期性调用，约10ms/次）
 */
void check_z_key_5times(void)
{
    uint32_t current_time = HAL_GetTick();
    static bool last_z_state = false;
    bool current_z_state = is_z_key_pressed();
    if (current_z_state && !last_z_state)
    {
        if ((current_time - last_z_press_time) <= 1000 || key_count == 0)
        {
            key_count++;
            last_z_press_time = current_time;
            if (key_count >= 5)
            {
                key_count = 0;
                last_z_press_time = 0;
            }
        }
        else
        {
            key_count = 1;
            last_z_press_time = current_time;
        }
    }
    else if (!current_z_state && (current_time - last_z_press_time) > 1000)
    {
        key_count = 0;
        last_z_press_time = 0;
    }
    last_z_state = current_z_state;
}

static void Balance_Clean_Process(Balance_t *balance)
{
    balance->Flag->Middle_Flag = false;
    balance->Flag->NoCheckRescueFlag = false;
    balance->Flag->Jumping_Flag = false;
    balance->Flag->KNEE_STRIKE_Flag = false;
    balance->Flag->JUMP_THEN_KNEE_STRIKE_Flag = false;
    balance->Flag->Chassis_Alignment_Flag = false;
    balance->Flag->KNEE_STRIKE_after_Alignment = false;
    balance->Flag->JUMP_AND_MID_Flag = false;
    balance->Flag->RTS_Flag = false;
    balance->Shoot_Flag_struct.Enable_Shoot_Flag = false;
    balance->Flag->GIMBAL_180_Flag = false;
    balance->Vision.Auto_Catch_Outpost_Flag = false;
    balance->Vision.Auto_Catch_Engi_Flag = false;
}

/**
 * @brief 手动自救模式键盘检测
 * @param balance Balance对象指针
 * @note 任意模式下连续按Z键五次（每次间隔不超过1s）则进入Manual_Rescue_Mode
 *       在Manual_Rescue_Mode下检测Ctrl键按下，退出到Sleep模式
 */
static void Balance_ManualRescueKeyMonitor(Balance_t *balance)
{
    uint32_t current_time = HAL_GetTick();
    Manual_Rescue_Var_t *var = &Chassis.manual_rescue_info->var;

    if (balance->mode == Manual_Rescue_Mode)
    {
        // 在手动自救模式下检测Ctrl键按下跳变，退出到Sleep模式
        uint8_t ctrl_pressed = (rc_sensor.info->Ctrl.status == short_press) ||
                               (rc_sensor.info->Ctrl.status == long_press) ||
                               (rc_sensor.info->Ctrl.status == release_to_press);

        if (ctrl_pressed && !var->ctrl_pressed_last)
        {
            // Ctrl键按下跳变，退出到Sleep模式
            balance->mode = Sleep_Mode;
            // 清除相关标志位（如果需要）
        }
        var->ctrl_pressed_last = ctrl_pressed;
    }
    else
    {
        // 任意模式下检测Z键连续按下
        if (rc_sensor.info->Z.status == release_to_press)
        {
            if (current_time - var->z_key_last_time < 1000)
            {
                // 距离上次按下不到1秒，计数+1
                var->z_key_count++;
            }
            else
            {
                // 超过1秒，重置计数
                var->z_key_count = 1;
            }
            var->z_key_last_time = current_time;

            // 连续按5次进入Manual_Rescue_Mode
            if (var->z_key_count >= 5)
            {
                balance->mode = Manual_Rescue_Mode;
                var->z_key_count = 0;
                // 进入时清零目标腿部竖直力
                Chassis.manual_rescue_info->var.F_target_l = 0;
                Chassis.manual_rescue_info->var.F_target_r = 0;
            }
        }
    }
}

// 底盘卸力模式，确保开控情况下都是SitDown_Mode
static void Balance_SitDown_Mode_Status_Update(Balance_t *balance)
{
    Balance.last_mode = Balance.mode;
    if (RC_ONLINE)
    {
        balance->rc->rc_online_tick++;
    }
    else
    {
        balance->rc->rc_online_tick = 0;
    }
    if (rc_sensor.work_state == DEV_OFFLINE)
    {
        balance->mode = Sleep_Mode;
        balance->reset_state = Balance_reset_NO;
        Balance_Clean_Process(balance);
    }
    else if (balance->mode != SitDown_Mode)
    {
        balance->mode = SitDown_Mode;
    }
    else
    {
        if (rc_sensor.info->s1 == RC_SW_DOWN && rc_sensor.info->s2 == RC_SW_DOWN)
        {
            balance->ctrl = KEY_CTRL;
        }
        else
        {
            balance->ctrl = RC_CTRL;
        }
        if (balance->ctrl == RC_CTRL)
        {
            RC_Move_Mode_Update(balance);
        }
        else
        {
            KEY_Move_Mode_Update(balance);
        }
    }
}

/**
 * @brief 自救测试模式状态更新
 *
 * 状态机流程:
 * 1. OFF→ON边沿检测: 记录第一次OFF→ON，允许触发一次自救流程
 * 2. OFFLINE: 立即进入Sleep_Mode，重置所有状态
 * 3. Rescue_Check: 检测是否需要自救，设置Rescue_Flag
 * 4. Rescue_Mode: 自救中，累加reset_cnt
 * 5. Rescue完成: Rescue_Flag=false且reset_state=OK，或reset_cnt超时 → 进入Sleep_Mode
 * 6. 完成一次后: 保持Sleep_Mode，不再触发新的自救(rescue_test_done_once=1阻塞)
 *
 * @param balance Balance结构体指针
 */
static void Balance_Test_Rescue_Status_Update(Balance_t *balance)
{
    if (RC_ONLINE)
    {
        balance->rc->rc_online_tick++;
    }
    else
    {
        balance->rc->rc_online_tick = 0;
    }
    /*------------- 静态变量，MCU reset后会被重新初始化为0 -------------*/
    static uint8_t rescue_test_online_last = 0; // 上次RC在线状态
    static uint8_t rescue_test_allow_once = 0;  // OFF→ON边沿已检测标志
    static uint8_t rescue_test_done_once = 0;   // 自救已完成标志

    uint8_t online_now = (rc_sensor.work_state == DEV_ONLINE);

    /*------------- OFF→ON边沿检测: 仅在第一次OFF→ON时触发 -------------*/
    if (rescue_test_online_last == 0 && online_now == 1)
    {
        rescue_test_allow_once = 1; // 允许触发一次自救
        rescue_test_done_once = 0;  // 重置完成标志
        balance->reset_state = Balance_reset_NO;
        balance->Flag->Rescue_Flag = false;
    }

    /*------------- 保持ON状态，避免MCU reset后重新触发OFF→ON检测 -------------*/
    // rescue_test_allow_once==1表示已检测到OFF→ON，不会因MCU reset而重新触发
    // (static变量在reset时会重新初始化为0，但allow_once不会被重新初始化，因为它已经=1)
    if (rescue_test_allow_once == 1)
    {
        rescue_test_online_last = 1;
    }
    else
    {
        rescue_test_online_last = online_now;
    }

    Balance.last_mode = Balance.mode;

    /*------------- OFFLINE: 立即进入Sleep_Mode，重置所有状态 -------------*/
    if (rc_sensor.work_state == DEV_OFFLINE)
    {
        balance->mode = Sleep_Mode;

        balance->reset_state = Balance_reset_NO;
        Balance_Clean_Process(balance);
        rescue_test_allow_once = 0;
        rescue_test_done_once = 0;
        rescue_test_online_last = 0;
        return;
    }

    /*------------- ONLINE: 执行rescue检测/自救逻辑 -------------*/

    // Rescue_Flag检测，仅在允许且未完成时调用
    if (rescue_test_allow_once && !rescue_test_done_once)
    {
        Rescue_Check(balance);
    }

    // 进入Rescue_Mode: 允许触发 && 未完成 && 需要自救 && 姿态异常
    if (rescue_test_allow_once && !rescue_test_done_once &&
        balance->Flag->Rescue_Flag == true &&
        balance->reset_state == Balance_reset_NO)
    {
        balance->mode = Rescue_Mode;

        balance->reset_state = Balance_reset_NO;
    }
    // Rescue完成退出: ONLINE && 当前是Rescue_Mode && Rescue_Flag已清除 && reset_state=OK
    // 或者: reset_cnt超时
    else if ((online_now &&
              balance->mode == Rescue_Mode &&
              balance->Flag->Rescue_Flag == false &&
              balance->reset_state == Balance_reset_OK))
    {
        // 自救完成下降沿：启动2秒屏蔽计时
        if (Balance.last_mode == Rescue_Mode && balance->Flag->NoCheckRescueFlag == false)
        {
            balance->Flag->NoCheckRescueFlag = true;
            balance->Flag->NoCheckRescue_StartTick = HAL_GetTick();
        }

        balance->mode = Sleep_Mode;
        balance->reset_state = Balance_reset_OK;
        rescue_test_done_once = 1; // 标记已完成，阻塞后续Rescue_Check调用
        rescue_test_allow_once = 0;
    }
    // 已完成一次自救: 保持Sleep_Mode
    else if (rescue_test_done_once)
    {
        balance->mode = Sleep_Mode;
        balance->Flag->Rescue_Flag = false;
        Balance_Clean_Process(balance);
    }
    // 正常待机: Sleep_Mode
    else
    {
        balance->mode = Sleep_Mode;
    }
}
